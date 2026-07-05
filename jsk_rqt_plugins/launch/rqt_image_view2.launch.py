from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # In ROS 1 the 'image' argument was remapped to the image_view2
        # C++ node input. image_view2 is not ported to ROS 2 yet (only its
        # messages exist as a stopgap), so the node is omitted here and
        # the argument is kept only for compatibility; the marked-image
        # round trip (image -> image_view2 -> rqt_image_view2) is
        # unavailable until image_view2 is ported.
        DeclareLaunchArgument(
            'image', default_value='image',
            description='Input image topic (unused until image_view2 is '
                        'ported to ROS 2)'),
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
