import os
import tempfile

from launch import LaunchDescription
from launch_ros.actions import Node

# In ROS 1 the tab layout was given inline as the nested ~tabbed_layout
# parameter. ROS 2 parameters cannot hold nested dicts, so the ported
# ServiceTabbedButtons plugin reads the same structure from a yaml file
# given by the tabbed_layout_yaml_file parameter. The layout is written
# to a temporary file when this launch file is evaluated.
TABBED_LAYOUT = """\
tab_list: ['push', 'radio']
push:
  name: 'push button'
  namespace: push
  type: push
  yaml_file: 'package://jsk_rqt_plugins/resource/service_button_layout.yaml'
radio:
  name: 'radio button'
  namespace: radio
  type: radio
  yaml_file: 'package://jsk_rqt_plugins/resource/service_radio_button_layout.yaml'
"""


def generate_launch_description():
    layout_yaml_file = os.path.join(
        tempfile.gettempdir(), 'sample_tabbed_buttons_layout.yaml')
    with open(layout_yaml_file, 'w') as f:
        f.write(TABBED_LAYOUT)

    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_tabbed_buttons',
            name='sample_buttons',
            output='screen',
            parameters=[{'tabbed_layout_yaml_file': layout_yaml_file}]),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_service_buttons.py',
            name='push_sample_service_buttons',
            namespace='push',
            output='screen'),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_service_radio_buttons.py',
            name='sample_service_buttons',
            namespace='radio',
            output='screen'),
    ])
