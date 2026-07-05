from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # The button layout is given by the layout_yaml_file parameter,
        # which is read by ServiceButtonGeneralWidget
        # (jsk_rqt_plugins/button_general.py) and resolved with
        # resource_retriever.
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_service_buttons',
            name='sample_buttons',
            output='screen',
            parameters=[{
                'layout_yaml_file':
                'package://jsk_rqt_plugins/resource/'
                'service_button_layout.yaml',
            }]),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_service_buttons.py',
            name='sample_service_buttons',
            output='screen'),
    ])
