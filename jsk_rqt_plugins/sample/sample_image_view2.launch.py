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
        # image_view2 node (Phase 1 port): draws markers on the incoming
        # image and republishes it as <image>/marked, which rqt_image_view2
        # displays. Runs headless (use_window:=false); the rqt plugin is the
        # GUI. The <image>/event topic carries mouse events back from rqt.
        Node(
            package='image_view2',
            executable='image_view2',
            name='image_view2',
            parameters=[{'use_window': False}],
            remappings=[
                ('image', 'pub_sample_image/image_raw'),
            ]),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_image_view2',
            name='rqt_image_view2',
            remappings=[
                ('event', 'image_view2/event'),
                ('image_marked', 'image_marked'),
            ]),
    ])
