from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'launch', 'config',
         'transformable_server_sample.rviz'])
    menu_yaml = PathJoinSubstitution(
        [FindPackageShare('jsk_interactive_marker'), 'config',
         'default_menu.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='jsk_interactive_marker',
            executable='transformable_server_sample',
            name='server_sample',
            output='screen',
            parameters=[{
                'torus_udiv': 100,
                'torus_vdiv': 100,
                'display_interactive_manipulator': False,
                'yaml_filename': menu_yaml,
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
