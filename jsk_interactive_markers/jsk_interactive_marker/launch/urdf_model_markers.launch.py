from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'config',
         'urdf_model_markers.rviz'])
    models_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'models',
         'sample_models.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('models', default_value=models_config),
        Node(
            package='jsk_interactive_marker',
            executable='urdf_model_marker',
            name='jsk_model_marker_interface',
            output='screen',
            parameters=[{
                # ROS 1 loaded the models config into the structured
                # rosparam "model_config"; the ROS 2 port reads the YAML
                # file given by "models_config_file" instead
                # (see src/urdf_model_marker_main.cpp for the format).
                'models_config_file': LaunchConfiguration('models'),
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
