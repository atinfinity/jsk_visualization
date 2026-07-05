from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sample_image = PathJoinSubstitution(
        [FindPackageShare('jsk_rqt_plugins'), 'sample',
         'kiva_pod_image_color.jpg'])
    return LaunchDescription([
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='pub_sample_image',
            arguments=[sample_image],
            remappings=[
                ('image_raw', 'pub_sample_image/image_raw'),
                ('camera_info', 'pub_sample_image/camera_info'),
            ]),
        # NOTE: the ROS 1 version of this sample also launched the
        # image_view2 C++ node (use_window: false), which draws markers on
        # the incoming image and republishes it as <image>/marked while
        # listening to the <image>/event topic published by this rqt
        # plugin. image_view2 is not ported to ROS 2 yet (only its
        # messages exist as a stopgap), so the marked-image round trip
        # (image -> image_view2 -> rqt_image_view2) is unavailable until
        # image_view2 is ported.
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_image_view2',
            name='rqt_image_view2',
            remappings=[
                ('event', 'image_view2/event'),
                ('image_marked', 'image_marked'),
            ]),
    ])
