# launch_testing port of the ROS 1 test_overlay_text_interface.test:
# run a node publishing through OverlayTextInterface and assert the
# OverlayText topic actually delivers a message.

import os
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy
from jsk_rviz_plugins_msgs.msg import OverlayText


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    publisher_node = launch_ros.actions.Node(
        package='jsk_rviz_plugins',
        executable='test_overlay_text_interface.py',
        name='publish_overlay_text_interface',
        output='screen')
    return launch.LaunchDescription([
        publisher_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'publisher_node': publisher_node}


class TestOverlayTextInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_overlay_text_interface')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_text_published(self):
        msgs = []
        sub = self.node.create_subscription(
            OverlayText, '/publish_overlay_text_interface/text',
            msgs.append, 1)
        try:
            end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=15)
            while self.node.get_clock().now() < end_time and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.5)
            self.assertGreater(len(msgs), 0,
                               'no OverlayText message received within 15s')
            self.assertEqual(msgs[0].text, 'test')
        finally:
            self.node.destroy_subscription(sub)
