from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_rviz_plugins'), 'config', 'overlay_sample.rviz'])
    # ROS 1 used $(find rviz)/images/splash.png; rviz_common ships the same
    # image on ROS 2. The OverlayImage display subscribes to
    # /image_publisher/image_raw, so remap image_publisher's relative topics
    # into that namespace to match overlay_sample.rviz.
    splash_image = PathJoinSubstitution(
        [FindPackageShare('rviz_common'), 'images', 'splash.png'])
    return LaunchDescription([
        Node(
            package='jsk_rviz_plugins',
            executable='overlay_sample.py',
            name='overlay_sample',
            respawn=True),
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher',
            parameters=[{'filename': splash_image}],
            remappings=[('image_raw', '/image_publisher/image_raw'),
                        ('camera_info', '/image_publisher/camera_info')]),
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
