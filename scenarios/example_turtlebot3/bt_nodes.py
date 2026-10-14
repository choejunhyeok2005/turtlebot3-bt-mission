"""One RViz goal starts one move -> photo -> return mission."""
import copy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from modules.base_bt_nodes import BTNodeList as BaseNodeList, Node, Sequence, Status
from modules.base_bt_nodes_ros import ActionWithROSAction, ActionWithROSService
from modules import utils

CUSTOM_ACTION_NODES = ['SaveStartPose', 'MoveToGoal', 'CaptureImage', 'ReturnToStart']
CUSTOM_CONDITION_NODES = ['WaitForGoal']


class BTNodeList(BaseNodeList):
    ACTION_NODES = BaseNodeList.ACTION_NODES + CUSTOM_ACTION_NODES
    CONDITION_NODES = BaseNodeList.CONDITION_NODES + CUSTOM_CONDITION_NODES
    DECORATOR_NODES = BaseNodeList.DECORATOR_NODES + ['PhotoMission']


def settings():
    return (utils.config or {}).get('turtlebot3', {})


class MissionContext:
    def __init__(self, agent):
        self.node = agent.ros_bridge.node
        self.cfg = settings()
        self.node.set_parameters([Parameter('use_sim_time', value=self.cfg.get('use_sim_time', True))])
        self.lock = threading.Lock()
        self.pending = None
        self.active = False
        self.buffer = Buffer(node=self.node)
        self.listener = TransformListener(self.buffer, self.node)
        self.subscription = self.node.create_subscription(
            PoseStamped, self.cfg.get('goal_topic', '/bt/goal_pose'), self.receive, 10)

    def receive(self, msg):
        frame = self.cfg.get('global_frame', 'map')
        values = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                  msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w]
        norm = sum(v * v for v in values[3:]) ** 0.5
        if msg.header.frame_id != frame or not all(math.isfinite(v) for v in values) or norm < 1e-6:
            self.node.get_logger().warning(f'Goal rejected: expected frame {frame} and a valid pose.')
            return
        goal = copy.deepcopy(msg)
        q = goal.pose.orientation
        q.x, q.y, q.z, q.w = [v / norm for v in values[3:]]
        with self.lock:
            if self.active:
                self.node.get_logger().warning('Mission busy; goal ignored. Send a new goal after completion.')
                return
            self.pending = goal


def context(agent):
    if not hasattr(agent, 'photo_mission'):
        agent.photo_mission = MissionContext(agent)
    return agent.photo_mission


class PhotoMission(Node):
    def __init__(self, name, child):
        super().__init__(name)
        self.children = [child]
        self.type = 'Decorator'

    async def run(self, agent, blackboard):
        self.status = await self.children[0].run(agent, blackboard)
        if self.status != Status.RUNNING:
            ctx = context(agent)
            ctx.node.get_logger().info(f'Mission finished: {self.status.name}')
            with ctx.lock:
                ctx.active = False
            blackboard.pop('goal_pose', None)
            blackboard.pop('start_pose', None)
        return self.status

    def halt(self):
        self.children[0].halt_children()
        self.children[0].halt()


class WaitForGoal(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.ctx = context(agent)
        self.type = 'Condition'

    async def run(self, agent, blackboard):
        with self.ctx.lock:
            if self.ctx.pending is None:
                self.status = Status.RUNNING
                return self.status
            blackboard['goal_pose'] = self.ctx.pending
            self.ctx.pending = None
            self.ctx.active = True
        blackboard.pop('photo_path', None)
        blackboard.pop('mission_error', None)
        self.ctx.node.get_logger().info('Goal received; saving start pose.')
        self.status = Status.SUCCESS
        return self.status


class SaveStartPose(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.ctx = context(agent)
        self.type = 'Action'
        self.started = None

    async def run(self, agent, blackboard):
        if self.started is None:
            self.started = time.monotonic()
        try:
            transform = self.ctx.buffer.lookup_transform(
                self.ctx.cfg.get('global_frame', 'map'),
                self.ctx.cfg.get('robot_frame', 'base_link'), Time())
        except TransformException:
            if time.monotonic() - self.started > self.ctx.cfg.get('setup_timeout', 15.0):
                blackboard['mission_error'] = 'Start pose TF unavailable; set the initial pose in RViz.'
                self.ctx.node.get_logger().error(blackboard['mission_error'])
                self.status = Status.FAILURE
            else:
                self.status = Status.RUNNING
            return self.status
        pose = PoseStamped()
        pose.header.frame_id = self.ctx.cfg.get('global_frame', 'map')
        pose.header.stamp = self.ctx.node.get_clock().now().to_msg()
        t = transform.transform.translation
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
        blackboard['start_pose'] = pose
        self.status = Status.SUCCESS
        return self.status

    def halt(self):
        self.started = None


class Navigation(ActionWithROSAction):
    pose_key = 'goal_pose'

    def __init__(self, name, agent):
        self.ctx = context(agent)
        super().__init__(name, agent, (NavigateToPose, self.ctx.cfg.get('navigation_action', '/navigate_to_pose')))
        self.sent = None
        self.started = None
        self.generation = 0

    def _build_goal(self, agent, blackboard):
        pose = blackboard.get(self.pose_key)
        if pose is None:
            return None
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(pose)
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        return goal

    def fail(self, blackboard, message):
        blackboard['mission_error'] = message
        self.ros.node.get_logger().error(message)
        self.halt()
        self.status = Status.FAILURE
        return self.status

    async def run(self, agent, blackboard):
        now = time.monotonic()
        if self.started is None:
            self.started = now
        timeout = self.ctx.cfg.get('navigation_timeout', 300.0)
        if now - self.started > timeout:
            return self.fail(blackboard, f'{self.name}: navigation timed out.')
        try:
            if self.sent is None:
                if not self.client.server_is_ready():
                    if now - self.started > self.ctx.cfg.get('setup_timeout', 15.0):
                        return self.fail(blackboard, f'{self.name}: Nav2 action server unavailable.')
                    self.status = Status.RUNNING
                    return self.status
                goal = self._build_goal(agent, blackboard)
                if goal is None:
                    return self.fail(blackboard, f'{self.name}: missing pose.')
                self.sent = self.client.send_goal_async(goal)
                generation = self.generation
                # A goal accepted after timeout/shutdown must also be canceled.
                def cancel_late(future):
                    if generation != self.generation:
                        try:
                            handle = future.result()
                            if handle.accepted:
                                handle.cancel_goal_async()
                        except Exception:
                            pass
                self.sent.add_done_callback(cancel_late)
                self.ros.node.get_logger().info(f'{self.name}: navigation requested.')
            if self._goal_handle is None and self.sent.done():
                self._goal_handle = self.sent.result()
                if not self._goal_handle.accepted:
                    return self.fail(blackboard, f'{self.name}: Nav2 rejected goal.')
                self._result_future = self._goal_handle.get_result_async()
            if self._result_future is not None and self._result_future.done():
                result = self._result_future.result()
                if result.status != GoalStatus.STATUS_SUCCEEDED or getattr(result.result, 'error_code', 0) != 0:
                    return self.fail(blackboard, f'{self.name}: navigation failed (status={result.status}).')
                self._goal_handle = None
                self._result_future = None
                self.sent = None
                self.started = None
                self.status = Status.SUCCESS
                return self.status
        except Exception as exc:
            return self.fail(blackboard, f'{self.name}: {exc}')
        self.status = Status.RUNNING
        return self.status

    def halt(self):
        self.generation += 1
        if self._goal_handle is not None and self._goal_handle.accepted:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None
        self.sent = None
        self.started = None


class MoveToGoal(Navigation):
    pass


class ReturnToStart(Navigation):
    pose_key = 'start_pose'


class CaptureImage(ActionWithROSService):
    def __init__(self, name, agent):
        self.ctx = context(agent)
        super().__init__(name, agent, (Trigger, self.ctx.cfg.get('capture_service', '/bt/capture_image')))
        self.started = None

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        if response.success:
            blackboard['photo_path'] = response.message
            self.ros.node.get_logger().info(f'Photo saved: {response.message}')
            return Status.SUCCESS
        blackboard['mission_error'] = response.message
        self.ros.node.get_logger().error(f'Capture failed: {response.message}')
        return Status.FAILURE

    async def run(self, agent, blackboard):
        if self.started is None:
            self.started = time.monotonic()
        if time.monotonic() - self.started > self.ctx.cfg.get('capture_timeout', 10.0):
            blackboard['mission_error'] = 'Capture service timed out.'
            self.ros.node.get_logger().error(blackboard['mission_error'])
            self.halt()
            self.status = Status.FAILURE
            return self.status
        try:
            return await super().run(agent, blackboard)
        except Exception as exc:
            blackboard['mission_error'] = str(exc)
            self.ros.node.get_logger().error(f'Capture failed: {exc}')
            self.halt()
            self.status = Status.FAILURE
            return self.status

    def halt(self):
        if self._future is not None and not self._future.done():
            self.client.remove_pending_request(self._future)
        self._future = None
        self._sent = False
        self.started = None
