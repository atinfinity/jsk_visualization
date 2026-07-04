from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_rviz_plugins'), 'config', 'service_call_panel.rviz'])
    params_file = PathJoinSubstitution(
        [FindPackageShare('jsk_rviz_plugins'), 'config', 'command_samples.yaml'])
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[params_file]),
    ])
