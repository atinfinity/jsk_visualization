from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_info_yaml = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'config',
         'sample_camera_info.yaml'])
    return LaunchDescription([
        Node(
            package='jsk_interactive_marker',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen'),
        Node(
            package='jsk_interactive_marker',
            executable='camera_info_publisher',
            name='camera_info_publisher_with_yaml',
            output='screen',
            parameters=[{
                'yaml_filename': camera_info_yaml,
            }]),
    ])
