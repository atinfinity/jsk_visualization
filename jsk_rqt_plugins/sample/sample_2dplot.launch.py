from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_2d_plot',
            name='scatter_plot',
            arguments=['/sample_data/output']),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_2d_plot',
            name='line_plot',
            arguments=['/sample_data/output', '--line']),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_2d_plot',
            name='scatter_plot_fit',
            arguments=['/sample_data/output', '--fit-line']),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_2d_plot',
            name='line_plot_fit',
            arguments=['/sample_data/output', '--line', '--fit-line']),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_2d_plot',
            name='line_plot_fit_ransac',
            arguments=['/sample_data/output', '--line', '--fit-line-ransac']),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_2d_plot.py',
            name='sample_data'),
    ])
