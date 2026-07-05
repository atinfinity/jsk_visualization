from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The ROS 1 sample ran the people-stack face_detector node on a live
    # RGBD camera. face_detector has no ROS 2 Jazzy release, so this launch
    # ports the visualization only and uses a synthetic publisher
    # (face_detector_sample.py) as a stand-in. Replace it with a real ROS 2
    # face detector publishing people_msgs/PositionMeasurementArray on
    # /face_detector/people_tracker_measurements_array for live results.
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_rviz_plugins'), 'config',
         'face_detector_sample.rviz'])
    return LaunchDescription([
        Node(
            package='jsk_rviz_plugins',
            executable='face_detector_sample.py',
            name='face_detector_sample'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]),
    ])
