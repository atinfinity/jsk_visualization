# Smoke test for the image_view2 node: publish a synthetic image and an
# ImageMarker2, and assert the node republishes the marked image.
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest

import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from image_view2.msg import ImageMarker2


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='image_view2',
        executable='image_view2',
        name='image_view2',
        parameters=[{'use_window': False}],
        remappings=[('image', 'test_image')],
        output='screen',
    )
    return (
        launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestImageView2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_image_view2_client')

    def tearDown(self):
        self.node.destroy_node()

    def _make_image(self):
        msg = Image()
        msg.header.frame_id = 'camera'
        msg.height = 64
        msg.width = 96
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = bytes([40] * (msg.step * msg.height))
        return msg

    def test_marked_image_published(self):
        img_pub = self.node.create_publisher(Image, 'test_image', 1)
        mk_pub = self.node.create_publisher(ImageMarker2, 'image_marker', 1)
        received = []
        self.node.create_subscription(
            Image, 'test_image/marked', lambda m: received.append(m), 1)

        # a CIRCLE marker to exercise the 2D drawing path
        marker = ImageMarker2()
        marker.ns = 'test'
        marker.id = 1
        marker.type = ImageMarker2.CIRCLE
        marker.position = Point(x=48.0, y=32.0)
        marker.scale = 10.0
        marker.outline_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        end = self.node.get_clock().now().nanoseconds + 15 * 1_000_000_000
        while not received and self.node.get_clock().now().nanoseconds < end:
            img_pub.publish(self._make_image())
            mk_pub.publish(marker)
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.assertTrue(received, 'no marked image was received')
        out = received[-1]
        self.assertEqual(out.width, 96)
        self.assertEqual(out.height, 64)
        self.assertEqual(out.encoding, 'bgr8')
