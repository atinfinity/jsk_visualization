# launch_testing smoke test: start every C++ node executable shipped by
# jsk_interactive_marker at once, let them run for a while and assert that
# none of them crashed, then assert they all shut down cleanly on SIGINT.

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

from ament_index_python.packages import get_package_share_directory
from launch.events.process import ProcessExited

RUN_TIME_SEC = 8.0

# executable -> extra node parameters
EXECUTABLES = {
    'marker_6dof': None,
    'bounding_box_marker': None,
    'transformable_server_sample': None,
    'point_cloud_config_marker': None,
    'triangle_foot': None,
    'door_foot': None,
    'polygon_marker': None,
    'interactive_point_cloud': None,
    'pointcloud_cropper': None,
    'urdf_model_marker': [{
        'models_config_file': os.path.join(
            get_package_share_directory('jsk_interactive_marker'),
            'launch', 'models', 'sample_models.yaml'),
    }],
    'urdf_control_marker': None,
    'interactive_marker_interface': None,
    'camera_info_publisher': None,
    'footstep_marker': None,
}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    nodes = {
        executable: launch_ros.actions.Node(
            package='jsk_interactive_marker',
            executable=executable,
            name='smoke_{}'.format(executable),
            output='screen',
            parameters=parameters)
        for executable, parameters in EXECUTABLES.items()
    }
    return launch.LaunchDescription(
        list(nodes.values()) + [launch_testing.actions.ReadyToTest()]
    ), {'nodes': nodes}


class TestExecutablesStayAlive(unittest.TestCase):

    def test_all_processes_stay_alive(self, proc_info, nodes):
        time.sleep(RUN_TIME_SEC)
        dead = []
        for executable, action in nodes.items():
            try:
                event = proc_info[action]
            except KeyError:
                dead.append('{} (never started)'.format(executable))
                continue
            if isinstance(event, ProcessExited):
                dead.append('{} (exited with code {})'.format(
                    executable, event.returncode))
        self.assertEqual(
            [], dead,
            'processes died within {}s: {}'.format(RUN_TIME_SEC, dead))


@launch_testing.post_shutdown_test()
class TestExecutablesShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info, nodes):
        for executable, action in nodes.items():
            with self.subTest(executable=executable):
                launch_testing.asserts.assertExitCodes(
                    proc_info, allowable_exit_codes=[0], process=action)
