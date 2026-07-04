from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_rviz_plugins'), 'config', 'overlay_sample.rviz'])
    return LaunchDescription([
        Node(
            package='jsk_rviz_plugins',
            executable='overlay_sample.py',
            name='overlay_sample',
            respawn=True),
        Node(
            package='jsk_rviz_plugins',
            executable='overlay_menu_sample.py',
            name='overlay_menu_sample',
            respawn=True),
        Node(
            package='jsk_rviz_plugins',
            executable='rosconsole_overlay_text.py',
            name='rosconsole_overlay_text',
            respawn=True,
            parameters=[{'reverse_lines': False}]),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '1', '/sample_string',
                 'std_msgs/msg/String',
                 "{data: 'This is a sample message of type std_msgs/String.'}"],
            name='sample_string_publisher'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])
