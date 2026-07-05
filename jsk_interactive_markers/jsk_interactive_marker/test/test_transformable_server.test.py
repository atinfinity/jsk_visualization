# launch_testing functional test for transformable_server_sample:
# insert a box marker through /transformable_interactive_server/
# request_marker_operate, verify its existence, do a set_pose/get_pose
# round-trip and assert the interactive marker server publishes an
# InteractiveMarkerUpdate on /simple_marker/update.

import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

import rclpy

from geometry_msgs.msg import PoseStamped
from jsk_interactive_marker_msgs.srv import GetTransformableMarkerExistence
from jsk_interactive_marker_msgs.srv import GetTransformableMarkerPose
from jsk_interactive_marker_msgs.srv import SetTransformableMarkerPose
from jsk_rviz_plugins_msgs.msg import TransformableMarkerOperate
from jsk_rviz_plugins_msgs.srv import RequestMarkerOperate
from visualization_msgs.msg import InteractiveMarkerUpdate

SERVER = '/transformable_interactive_server'
SERVICE_TIMEOUT_SEC = 10.0
RESPONSE_TIMEOUT_SEC = 10.0
MARKER_NAME = 'test_box'
FRAME_ID = 'map'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    server_node = launch_ros.actions.Node(
        package='jsk_interactive_marker',
        executable='transformable_server_sample',
        name='transformable_interactive_server',
        output='screen')
    return launch.LaunchDescription([
        server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'server_node': server_node}


class TestTransformableServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_transformable_server')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        self.assertTrue(
            client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC),
            'service {} not available within {}s'.format(
                client.srv_name, SERVICE_TIMEOUT_SEC))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=RESPONSE_TIMEOUT_SEC)
        self.assertTrue(
            future.done(),
            'no response from {} within {}s'.format(
                client.srv_name, RESPONSE_TIMEOUT_SEC))
        return future.result()

    def test_marker_lifecycle(self):
        updates = []
        sub = self.node.create_subscription(
            InteractiveMarkerUpdate, '/simple_marker/update',
            updates.append, 10)
        req_operate = self.node.create_client(
            RequestMarkerOperate, SERVER + '/request_marker_operate')
        get_existence = self.node.create_client(
            GetTransformableMarkerExistence, SERVER + '/get_existence')
        set_pose = self.node.create_client(
            SetTransformableMarkerPose, SERVER + '/set_pose')
        get_pose = self.node.create_client(
            GetTransformableMarkerPose, SERVER + '/get_pose')
        try:
            # 1. insert a box marker
            insert_req = RequestMarkerOperate.Request()
            insert_req.operate = TransformableMarkerOperate(
                type=TransformableMarkerOperate.SHAPE_BOX,
                action=TransformableMarkerOperate.ACTION_INSERT,
                frame_id=FRAME_ID,
                name=MARKER_NAME,
                description='box inserted by launch test')
            self._call(req_operate, insert_req)

            # 2. the marker must now exist
            exist_req = GetTransformableMarkerExistence.Request()
            exist_req.target_name = MARKER_NAME
            exist_res = self._call(get_existence, exist_req)
            self.assertTrue(exist_res.existence,
                            'inserted marker not reported as existing')

            # 3. set_pose / get_pose round-trip
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = FRAME_ID
            pose_stamped.pose.position.x = 1.0
            pose_stamped.pose.position.y = 2.0
            pose_stamped.pose.position.z = 3.0
            pose_stamped.pose.orientation.w = 1.0
            set_req = SetTransformableMarkerPose.Request()
            set_req.target_name = MARKER_NAME
            set_req.pose_stamped = pose_stamped
            self._call(set_pose, set_req)

            get_req = GetTransformableMarkerPose.Request()
            get_req.target_name = MARKER_NAME
            get_res = self._call(get_pose, get_req)
            self.assertEqual(FRAME_ID, get_res.pose_stamped.header.frame_id)
            position = get_res.pose_stamped.pose.position
            self.assertAlmostEqual(1.0, position.x, places=5)
            self.assertAlmostEqual(2.0, position.y, places=5)
            self.assertAlmostEqual(3.0, position.z, places=5)

            # 4. the marker operations must have produced updates on
            # /simple_marker/update
            end_time = (self.node.get_clock().now()
                        + rclpy.duration.Duration(
                            seconds=RESPONSE_TIMEOUT_SEC))
            while self.node.get_clock().now() < end_time and not updates:
                rclpy.spin_once(self.node, timeout_sec=0.5)
            self.assertGreater(
                len(updates), 0,
                'no InteractiveMarkerUpdate received on /simple_marker/update'
                ' within {}s'.format(RESPONSE_TIMEOUT_SEC))
        finally:
            self.node.destroy_subscription(sub)
            for client in (req_operate, get_existence, set_pose, get_pose):
                self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestTransformableServerShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, server_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0], process=server_node)
