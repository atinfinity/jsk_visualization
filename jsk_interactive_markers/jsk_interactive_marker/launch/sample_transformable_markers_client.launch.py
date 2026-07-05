from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'config',
         'sample_transformable_markers_client.rviz'])
    client_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'config',
         'sample_transformable_markers_client.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='jsk_interactive_marker',
            executable='transformable_server_sample',
            name='transformable_interactive_server',
            parameters=[{
                'display_interactive_manipulator': True,
                'display_interactive_manipulator_only_selected': True,
            }]),
        Node(
            package='jsk_interactive_marker',
            executable='transformable_markers_client.py',
            name='transformable_markers_client',
            output='screen',
            parameters=[{
                # ROS 1 remapped '~server'; in ROS 2 the server node name
                # is passed as a parameter instead
                'server': 'transformable_interactive_server',
                'config_file': client_config,
                # Use True to save config updated on rviz
                'config_auto_save': False,
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
