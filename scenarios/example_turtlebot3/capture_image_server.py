#!/usr/bin/env python3
"""Save the first camera frame received after a Trigger request."""
from datetime import datetime
from pathlib import Path
import threading
import time
from uuid import uuid4

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class CaptureImageServer(Node):
    def __init__(self):
        super().__init__('capture_image_server')
        self.declare_parameter('image_topic', '/TurtleBot3Burger/front_camera/image_color')
        self.declare_parameter('output_dir', '~/robot_bt_ws/photos')
        self.declare_parameter('frame_timeout', 5.0)
        self.output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.frame = None
        self.frame_number = 0
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.receive,
            qos_profile_sensor_data, callback_group=self.image_group)
        self.service = self.create_service(
            Trigger, '/bt/capture_image', self.capture, callback_group=self.service_group)
        self.get_logger().info(f'Capture service ready; saving to {self.output_dir}')

    def receive(self, msg):
        with self.condition:
            self.frame = msg
            self.frame_number += 1
            self.condition.notify_all()

    def capture(self, request, response):
        deadline = time.monotonic() + float(self.get_parameter('frame_timeout').value)
        with self.condition:
            previous = self.frame_number
            while self.frame_number <= previous:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    response.success = False
                    response.message = 'No new camera frame received before timeout.'
                    return response
                self.condition.wait(timeout=remaining)
            frame = self.frame
        try:
            image = self.bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
            filename = f'{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.png'
            path = self.output_dir / filename
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f'Cannot write image: {path}')
            response.success = True
            response.message = str(path)
            self.get_logger().info(f'Saved {path}')
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        return response


def main():
    rclpy.init()
    node = CaptureImageServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
