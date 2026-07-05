from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SAMPLE_BOXES = """
header: {frame_id: map}
boxes:
- header: {frame_id: map}
  pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}
  dimensions: {x: 0.5, y: 0.4, z: 1.0}
  label: 0
- header: {frame_id: map}
  pose: {position: {x: 0.0, y: 1.0, z: 0.25}, orientation: {w: 1.0}}
  dimensions: {x: 0.3, y: 0.3, z: 0.5}
  label: 1
- header: {frame_id: map}
  pose: {position: {x: -1.0, y: 0.0, z: 0.15}, orientation: {w: 1.0}}
  dimensions: {x: 0.6, y: 0.2, z: 0.3}
  label: 2
"""


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'config',
         'bounding_box_marker_sample.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='jsk_interactive_marker',
            executable='bounding_box_marker',
            name='bounding_box_interactive_marker',
            output='screen'),
        # tiny sample publisher of the boxes the marker subscribes to
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '1',
                 '/bounding_box_interactive_marker/bounding_box_array',
                 'jsk_recognition_msgs/msg/BoundingBoxArray',
                 SAMPLE_BOXES],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
