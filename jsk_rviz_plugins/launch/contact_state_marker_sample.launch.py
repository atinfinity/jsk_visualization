import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('jsk_rviz_plugins')
    with open(os.path.join(share, 'config', 'sample_robot.urdf')) as f:
        robot_description = f.read()
    rviz_config = os.path.join(share, 'config',
                               'contact_state_marker_sample.rviz')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'),
        Node(
            package='jsk_rviz_plugins',
            executable='contact_state_sample.py',
            name='contact_state_sample',
            parameters=[{'links': ['base_link', 'arm_link']}]),
        Node(
            package='jsk_rviz_plugins',
            executable='contact_state_marker.py',
            name='contact_state_marker',
            remappings=[('~/input', '/contact_state_sample/output')]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]),
    ])
