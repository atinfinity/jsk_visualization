from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # The 'image' argument is the input image topic that the
        # image_view2 node subscribes to and republishes as <image>/marked.
        DeclareLaunchArgument(
            'image', default_value='image',
            description='Input image topic for image_view2'),
        # image_view2 node (Phase 1 port), headless; rqt_image_view2 is the
        # GUI that displays <image>/marked and publishes <image>/event.
        Node(
            package='image_view2',
            executable='image_view2',
            name='image_view2',
            output='screen',
            parameters=[{'use_window': False}],
            remappings=[
                ('image', LaunchConfiguration('image')),
            ]),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_image_view2',
            name='rqt_image_view2',
            output='screen',
            remappings=[
                ('event', 'image_view2/event'),
                ('image_marked', 'image_marked'),
            ]),
    ])
