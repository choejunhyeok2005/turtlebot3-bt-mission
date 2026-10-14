"""Offline tests: no simulator, DDS participant, or ROS servers required."""
import asyncio
from concurrent.futures import Future
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from modules import utils
from modules.base_bt_nodes import Status
from modules.bt_constructor import build_behavior_tree
from scenarios.example_turtlebot3.bt_nodes import MissionContext, MoveToGoal, ReturnToStart
from scenarios.example_turtlebot3.capture_image_server import CaptureImageServer


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        self.ctx = SimpleNamespace(node=self.node, cfg={}, lock=threading.Lock(),
                                   pending=None, active=False, buffer=Mock())
        self.agent = SimpleNamespace(ros_bridge=SimpleNamespace(node=self.node), photo_mission=self.ctx)
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.goal.pose.position.x = 3.0
        self.goal.pose.orientation.w = 1.0

    def navigation(self, cls=MoveToGoal):
        with patch('modules.base_bt_nodes_ros.ActionClient'):
            nav = cls(cls.__name__, self.agent)
        nav.client = Mock()
        nav.client.server_is_ready.return_value = True
        return nav

    def test_goal_is_frozen_and_busy_goal_ignored(self):
        MissionContext.receive(self.ctx, self.goal)
        self.goal.pose.position.x = 9.0
        self.assertEqual(self.ctx.pending.pose.position.x, 3.0)
        self.ctx.active = True
        MissionContext.receive(self.ctx, self.goal)
        self.assertEqual(self.ctx.pending.pose.position.x, 3.0)

    def test_invalid_frame_rejected(self):
        self.goal.header.frame_id = 'odom'
        MissionContext.receive(self.ctx, self.goal)
        self.assertIsNone(self.ctx.pending)

    def test_rejection_fails_without_resending(self):
        nav = self.navigation()
        nav.client.send_goal_async.return_value = completed(SimpleNamespace(accepted=False))
        self.assertEqual(asyncio.run(nav.run(self.agent, {'goal_pose': self.goal})), Status.FAILURE)
        nav.client.send_goal_async.assert_called_once()

    def test_return_uses_saved_start_pose(self):
        nav = self.navigation(ReturnToStart)
        start = PoseStamped()
        start.pose.position.x = -2.0
        start.pose.orientation.w = 1.0
        result = nav._build_goal(self.agent, {'goal_pose': self.goal, 'start_pose': start})
        self.assertEqual(result.pose.pose.position.x, -2.0)

    def test_late_acceptance_is_canceled(self):
        nav = self.navigation()
        future = Future()
        nav.client.send_goal_async.return_value = future
        self.assertEqual(asyncio.run(nav.run(self.agent, {'goal_pose': self.goal})), Status.RUNNING)
        nav.halt()
        handle = Mock(accepted=True)
        future.set_result(handle)
        handle.cancel_goal_async.assert_called_once()

    def test_timeout_cancels_active_goal(self):
        nav = self.navigation()
        nav.started = time.monotonic() - 301
        handle = Mock(accepted=True)
        nav._goal_handle = handle
        self.assertEqual(asyncio.run(nav.run(self.agent, {})), Status.FAILURE)
        handle.cancel_goal_async.assert_called_once()

    def test_full_tree_runs_once_and_waits_for_next_goal(self):
        xml = str(Path(__file__).with_name('default_bt.xml'))
        utils.config = {'turtlebot3': {}}
        transform = SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=-1., y=2., z=0.), rotation=self.goal.pose.orientation))
        self.ctx.buffer.lookup_transform.return_value = transform
        self.ctx.pending = self.goal
        with patch('modules.base_bt_nodes_ros.ActionClient') as client_class:
            client_class.return_value.server_is_ready.return_value = True
            handle = Mock(accepted=True)
            handle.get_result_async.return_value = completed(SimpleNamespace(status=4, result=SimpleNamespace(error_code=0)))
            client_class.return_value.send_goal_async.return_value = completed(handle)
            self.node.create_client.return_value.wait_for_service.return_value = True
            self.node.create_client.return_value.call_async.return_value = completed(Trigger.Response(success=True, message='/tmp/photo.png'))
            tree = build_behavior_tree(self.agent, xml, 'scenarios.example_turtlebot3')
            bb = {}
            self.assertEqual(asyncio.run(tree.run(self.agent, bb)), Status.RUNNING)
            self.assertEqual(asyncio.run(tree.run(self.agent, bb)), Status.SUCCESS)
            self.assertFalse(self.ctx.active)
            self.assertEqual(bb['photo_path'], '/tmp/photo.png')
            goals = client_class.return_value.send_goal_async.call_args_list
            self.assertEqual([call.args[0].pose.pose.position.x for call in goals], [3.0, -1.0])
            self.assertEqual(asyncio.run(tree.run(self.agent, bb)), Status.RUNNING)
            self.assertEqual(client_class.return_value.send_goal_async.call_count, 2)

    def test_fresh_camera_frame_saved_and_missing_frame_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            server = SimpleNamespace(condition=threading.Condition(), frame_number=0, frame=None,
                                     output_dir=Path(directory), bridge=Mock(), get_logger=Mock())
            server.get_parameter = lambda name: SimpleNamespace(value=0.2)
            server.bridge.imgmsg_to_cv2.return_value = np.zeros((4, 4, 3), dtype=np.uint8)
            def deliver():
                time.sleep(0.03)
                CaptureImageServer.receive(server, object())
            thread = threading.Thread(target=deliver)
            thread.start()
            response = CaptureImageServer.capture(server, Trigger.Request(), Trigger.Response())
            thread.join()
            self.assertTrue(response.success)
            self.assertTrue(Path(response.message).is_file())
            response = CaptureImageServer.capture(server, Trigger.Request(), Trigger.Response())
            self.assertFalse(response.success)


if __name__ == '__main__':
    unittest.main()
