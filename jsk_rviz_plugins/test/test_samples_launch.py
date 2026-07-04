# launch_testing smoke test: start every installed rclpy sample publisher
# and assert that each one actually delivers a message on its topic. This
# exercises the message packages (imports, constants, field types) without
# a GUI.

import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy

from diagnostic_msgs.msg import DiagnosticArray
from jsk_footstep_msgs.msg import FootstepArray
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import SegmentArray
from jsk_recognition_msgs.msg import SimpleOccupancyGridArray
from jsk_recognition_msgs.msg import TorusArray
from jsk_rviz_plugins_msgs.msg import OverlayMenu
from jsk_rviz_plugins_msgs.msg import OverlayText
from jsk_rviz_plugins_msgs.msg import PictogramArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

# (executable, node_name, topic, type)
SAMPLES = [
    ('overlay_sample.py', 'overlay_sample', '/text_sample', OverlayText),
    ('overlay_menu_sample.py', 'overlay_menu_sample', '/test_menu',
     OverlayMenu),
    ('piechart_sample.py', 'piechart_sample', '/sample_piechart', Float32),
    ('pictogram_all.py', 'pictogram_all_sample', '/pictogram_array',
     PictogramArray),
    ('bounding_box_sample.py', 'bbox_sample', '/bbox', BoundingBoxArray),
    ('torus_array_sample.py', 'torus_sample', '/test_torus', TorusArray),
    ('polygon_array_sample.py', 'polygon_array_sample',
     '/polygon_array_sample/output', PolygonArray),
    ('segment_array_sample.py', 'segment_array_sample',
     '/segment_array_sample/output', SegmentArray),
    ('sparse_occupancy_grid_sample.py', 'occupancy_grid_sample',
     '/occupancy_grid', SimpleOccupancyGridArray),
    ('footstep_sample.py', 'footstep_sample', '/footsteps', FootstepArray),
    ('normal_sample.py', 'normal_sample', '/normal_sample/output',
     PointCloud2),
    ('camera_sample.py', 'camera_sample', '/camera_sample/image_raw', Image),
    ('diagnostics_sample.py', 'diagnostics_sample', '/diagnostics',
     DiagnosticArray),
]

TIMEOUT_SEC = 60.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    nodes = [
        launch_ros.actions.Node(
            package='jsk_rviz_plugins',
            executable=executable,
            name=node_name,
            output='screen')
        for executable, node_name, _, _ in SAMPLES
    ]
    return launch.LaunchDescription(
        nodes + [launch_testing.actions.ReadyToTest()]), {}


class TestSamplesPublish(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_samples_publish')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_all_samples_publish(self):
        received = {}
        subs = []
        for _, _, topic, msg_type in SAMPLES:
            def make_cb(t):
                def cb(msg):
                    received[t] = msg
                return cb
            subs.append(self.node.create_subscription(
                msg_type, topic, make_cb(topic), 1))
        try:
            end_time = (self.node.get_clock().now()
                        + rclpy.duration.Duration(seconds=TIMEOUT_SEC))
            while (self.node.get_clock().now() < end_time
                   and len(received) < len(SAMPLES)):
                rclpy.spin_once(self.node, timeout_sec=0.5)
            missing = sorted(
                topic for _, _, topic, _ in SAMPLES if topic not in received)
            self.assertEqual(
                [], missing,
                'no message within {}s on: {}'.format(TIMEOUT_SEC, missing))
        finally:
            for sub in subs:
                self.node.destroy_subscription(sub)
