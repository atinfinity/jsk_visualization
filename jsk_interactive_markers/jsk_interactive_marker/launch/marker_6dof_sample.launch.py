from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'config',
         'marker_6dof_sample.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='jsk_interactive_marker',
            executable='marker_6dof',
            name='marker_cube',
            parameters=[{
                'frame_id': 'map',
                'object_type': 'cube',
                'object_x': 0.2,
                'object_y': 0.2,
                'object_z': 0.3,
                'object_r': 1.0,
                'object_g': 0.2,
                'object_b': 0.2,
                'object_a': 1.0,
            }]),
        Node(
            package='jsk_interactive_marker',
            executable='marker_6dof',
            name='marker_sphere',
            parameters=[{
                'frame_id': 'map',
                'object_type': 'sphere',
                'object_x': 0.2,
                'object_y': 0.2,
                'object_z': 0.3,
                'object_r': 0.2,
                'object_g': 1.0,
                'object_b': 0.2,
                'object_a': 1.0,
            }]),
        Node(
            package='jsk_interactive_marker',
            executable='marker_6dof',
            name='marker_line',
            parameters=[{
                'frame_id': 'map',
                'object_type': 'line',
                'object_x': 0.1,
                'object_r': 0.2,
                'object_g': 0.2,
                'object_b': 1.0,
                'object_a': 1.0,
            }]),
        Node(
            package='jsk_interactive_marker',
            executable='marker_6dof',
            name='marker_mesh',
            parameters=[{
                'frame_id': 'map',
                'object_type': 'mesh',
                'object_x': 1.0,
                'object_y': 0.5,
                'object_z': 1.0,
                'object_r': 1.0,
                'object_g': 0.2,
                'object_b': 1.0,
                'object_a': 1.0,
                'mesh_file':
                    'package://jsk_interactive_marker/models/sample_mesh.dae',
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
