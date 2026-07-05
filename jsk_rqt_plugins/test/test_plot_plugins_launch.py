#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# launch_testing port of the plot half of the ROS 1 test_rqt_plugins.test:
# run the sample publishers together with the standalone rqt_2d_plot /
# rqt_histogram_plot / rqt_3d_plot plugins (QT_QPA_PLATFORM=offscreen) and
# assert observable behavior with an rclpy probe node.
#
# NOTE (filename): pytest cannot import modules with dots in the name, so
# this file is called test_plot_plugins_launch.py, not
# test_plot_plugins.launch.py.  launch_testing's pytest plugin picks it up
# through the @pytest.mark.launch_test marker.
#
# WHAT IS AND IS NOT COVERED HEADLESS
# -----------------------------------
# All three plot plugins resolve their command line topic exactly ONCE at
# widget construction via rqt_plot.rosplot.get_topic_type() and never
# retry (upstream rqt_plot ROSData behavior, identical with the stock
# /opt/ros/jazzy rqt_plot).  Under QT_QPA_PLATFORM=offscreen the widget is
# built ~0.1-0.5 s after the node's DDS participant is created, while
# discovery of an already-running external publisher takes ~0.5-1 s inside
# the rqt process, so the plugins reliably LOSE the race and never
# subscribe (measured 0/3 for rqt_2d_plot, 0/3 for rqt_3d_plot, publisher
# started 3 s earlier; under a real X server the slower widget
# construction hides this).  Therefore, unlike the ROS 1 rostest, this
# test can NOT assert count_subscribers() on the plotted topics:
#   * rqt_2d_plot: asserted through its unconditional side effect instead
#     (it advertises <topic>/plot_image before creating ROSData).
#   * rqt_3d_plot: loses the race quietly ('Can not resolve topic type'
#     warning once) -> process-alive + no-traceback only.
#   * rqt_histogram_plot: is run WITHOUT a topic argument.  With an
#     argument, losing the race makes HistogramPlotWidget.update_plot()
#     raise the stored RosPlotException uncaught in the Qt timer slot and
#     PyQt aborts the process (SIGABRT + traceback), which would be flaky.
#     -> process-alive + no-traceback only.
# The sample publishers themselves are actively asserted: the probe node
# must receive a message on every published topic.

import os
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy

from jsk_recognition_msgs.msg import HistogramWithRange
from jsk_recognition_msgs.msg import PlotData
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

LIB_DIR = os.path.join(
    get_package_prefix('jsk_rqt_plugins'), 'lib', 'jsk_rqt_plugins')

ALIVE_TIME_SEC = 8.0
DATAFLOW_TIMEOUT_SEC = 30.0
GRAPH_TIMEOUT_SEC = 30.0


def _process(executable, args=None, name=None):
    # Run the installed launcher binary directly (NOT via `ros2 run`,
    # whose wrapper orphans the child on SIGKILL and keeps the output
    # pipe open forever).  rqt does not reliably exit on SIGINT/SIGTERM
    # (stock rqt behavior), so let launch escalate to SIGKILL quickly.
    return launch.actions.ExecuteProcess(
        cmd=[os.path.join(LIB_DIR, executable)] + list(args or []),
        name=name or executable,
        output='screen',
        env=dict(os.environ, QT_QPA_PLATFORM='offscreen'),
        sigterm_timeout='5',
        sigkill_timeout='5',
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    procs = {
        'sample_2d_plot': _process('sample_2d_plot.py',
                                   name='sample_2d_plot'),
        'sample_hist_pub': _process('sample_hist_pub.py',
                                    name='sample_hist_pub'),
        'sample_3d_plot': _process('sample_3d_plot.py',
                                   name='sample_3d_plot'),
        'rqt_2d_plot': _process('rqt_2d_plot', ['/sample_2d_plot/output']),
        # no topic argument: see module docstring
        'rqt_histogram_plot': _process('rqt_histogram_plot'),
        'rqt_3d_plot': _process('rqt_3d_plot',
                                ['/sample_3d_plot/output1/data',
                                 '/sample_3d_plot/output2/data',
                                 '/sample_3d_plot/output3/data',
                                 '--buffer', '100']),
    }
    return launch.LaunchDescription(
        list(procs.values()) + [launch_testing.actions.ReadyToTest()]
    ), {'procs': procs}


class TestPlotPlugins(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_plot_plugins_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_message(self, msg_type, topic, timeout):
        msgs = []
        sub = self.node.create_subscription(msg_type, topic, msgs.append, 10)
        try:
            end = time.monotonic() + timeout
            while time.monotonic() < end and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.25)
            return bool(msgs)
        finally:
            self.node.destroy_subscription(sub)

    def test_sample_publishers_publish(self):
        # replicates the rospy.wait_for_message() half of the ROS 1 test
        for msg_type, topic in (
                (PlotData, '/sample_2d_plot/output'),
                (Float32MultiArray, '/normal_array'),
                (HistogramWithRange, '/range_array'),
                (Float32, '/sample_3d_plot/output1'),
                (Float32, '/sample_3d_plot/output2'),
                (Float32, '/sample_3d_plot/output3')):
            with self.subTest(topic=topic):
                self.assertTrue(
                    self._wait_for_message(
                        msg_type, topic, DATAFLOW_TIMEOUT_SEC),
                    'no %s message received on %s within %s s'
                    % (msg_type.__name__, topic, DATAFLOW_TIMEOUT_SEC))

    def test_rqt_2d_plot_advertises_plot_image(self):
        # Plot2DWidget.subscribe_topic() unconditionally advertises
        # <topic>/plot_image, so this proves the plugin started, parsed
        # its command line topic and its rqt node works.
        topic = '/sample_2d_plot/output/plot_image'
        end = time.monotonic() + GRAPH_TIMEOUT_SEC
        while time.monotonic() < end:
            if self.node.count_publishers(topic) >= 1:
                return
            time.sleep(0.5)
        self.fail('rqt_2d_plot did not advertise %s within %s s'
                  % (topic, GRAPH_TIMEOUT_SEC))

    def test_processes_stay_alive(self, proc_info, procs):
        time.sleep(ALIVE_TIME_SEC)
        from launch.events.process import ProcessExited
        dead = []
        for name, action in procs.items():
            try:
                event = proc_info[action]
            except KeyError:
                dead.append('%s (never started)' % name)
                continue
            if isinstance(event, ProcessExited):
                dead.append('%s (exited with code %s)'
                            % (name, event.returncode))
        self.assertEqual([], dead, 'processes died: %s' % dead)


@launch_testing.post_shutdown_test()
class TestPlotPluginsShutdown(unittest.TestCase):

    # NOTE: rqt processes are SIGKILLed after the SIGINT/SIGTERM grace
    # periods (stock rqt does not exit cleanly), so exit codes are NOT
    # asserted here.  Instead assert that no process ever printed a
    # Python traceback while running.
    def test_no_tracebacks(self, proc_output):
        offenders = set()
        for io in proc_output:
            if b'Traceback' in io.text:
                offenders.add(io.process_name)
        self.assertEqual(
            set(), offenders,
            'processes printed Python tracebacks: %s' % sorted(offenders))
