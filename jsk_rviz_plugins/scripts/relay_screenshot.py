#!/usr/bin/env python3
"""
Call snapshot service of rviz (provided by ScreenshotListener tool)
when a topic is published.

This script is useful to automatically record result of ros processing.

NOTE:
  rviz should be in fron of other windows because
"""

from jsk_rviz_plugins_msgs.srv import Screenshot
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class RelayScreenshot(Node):
    def __init__(self):
        super(RelayScreenshot, self).__init__('relay_screenshot')
        self.counter = 0
        self.screenshot_srv = self.create_client(
            Screenshot, '/rviz/screenshot')
        self.file_format = self.declare_parameter(
            'file_format', 'rviz_screenshot_{0:0>5}.png').value
        self.sub = None
        # NOTE(ros2): rclpy has no AnyMsg, so poll the topic until we can
        # detect its type and then subscribe with a raw (serialized)
        # subscription.
        self.input_topic = self.resolve_topic_name('~/input')
        self.poll_timer = self.create_timer(1.0, self.poll_input_topic)

    def poll_input_topic(self):
        infos = self.get_publishers_info_by_topic(self.input_topic)
        if not infos:
            return
        msg_type = get_message(infos[0].topic_type)
        self.sub = self.create_subscription(
            msg_type, self.input_topic, self.callback, 1, raw=True)
        self.destroy_timer(self.poll_timer)

    def callback(self, msg):
        self.get_logger().info(
            'received a message, save a screenshot to {0}'.format(
                self.file_format.format(self.counter)))
        if not self.screenshot_srv.service_is_ready():
            self.get_logger().error(
                'Failed to call screenshot service call. Have you add ScreenshotListener to rviz and file_format is correct? file_format is "{0}"'.format(self.file_format))
            return
        req = Screenshot.Request()
        req.file_name = self.file_format.format(self.counter)
        self.screenshot_srv.call_async(req)
        self.counter = self.counter + 1


def main(args=None):
    rclpy.init(args=args)
    node = RelayScreenshot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
