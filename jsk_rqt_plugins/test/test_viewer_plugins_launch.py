#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# launch_testing port of the viewer half of the ROS 1 test_rqt_plugins.test:
#   * rqt_drc_mini_maxwell (DRCEnvironmentViewer) hard-subscribes the
#     /drc_2015_environment/* topics in its widget constructor, so - like
#     the ROS 1 rostest did against the master - a probe node asserts
#     count_subscribers() >= 1 on all three topics,
#   * sample_drc_mini_maxwell.py must actually publish (probe receives a
#     message),
#   * rqt_status_light and rqt_string_label only create their topic
#     subscription AFTER a GUI interaction (the user picks a topic in the
#     ComboBoxDialog opened from the plugin's configuration button;
#     src/jsk_rqt_plugins/status_light.py setupSubscriber() /
#     label.py setupSubscriber() are only reached from trigger_configuration
#     or from restored instance settings).  That cannot happen headless, so
#     for these two plugins this test only asserts process-alive +
#     no-traceback.
#
# NOTE (filename): pytest cannot import modules with dots in the name, so
# this file is called test_viewer_plugins_launch.py, not
# test_viewer_plugins.launch.py.

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

from std_msgs.msg import Bool

LIB_DIR = os.path.join(
    get_package_prefix('jsk_rqt_plugins'), 'lib', 'jsk_rqt_plugins')

ALIVE_TIME_SEC = 8.0
DATAFLOW_TIMEOUT_SEC = 30.0
GRAPH_TIMEOUT_SEC = 30.0

DRC_TOPICS = (
    '/drc_2015_environment/is_disabled',
    '/drc_2015_environment/is_blackout',
    '/drc_2015_environment/next_whiteout_time',
)


def _process(executable, args=None, name=None):
    # installed launcher binary, run directly (not via `ros2 run`); rqt
    # does not reliably exit on SIGINT/SIGTERM -> quick SIGKILL escalation
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
        'sample_drc_mini_maxwell': _process('sample_drc_mini_maxwell.py',
                                            name='sample_drc_mini_maxwell'),
        'rqt_drc_mini_maxwell': _process('rqt_drc_mini_maxwell'),
        # process-alive only, see module docstring
        'rqt_status_light': _process('rqt_status_light'),
        'rqt_string_label': _process('rqt_string_label'),
    }
    return launch.LaunchDescription(
        list(procs.values()) + [launch_testing.actions.ReadyToTest()]
    ), {'procs': procs}


class TestViewerPlugins(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_viewer_plugins_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_drc_viewer_subscribes(self):
        # replicates the ROS 1 master-based subscriber checks; the DRC
        # viewer creates these subscriptions unconditionally in its
        # constructor, the probe just has to wait for graph discovery
        end = time.monotonic() + GRAPH_TIMEOUT_SEC
        pending = list(DRC_TOPICS)
        while time.monotonic() < end and pending:
            pending = [t for t in pending
                       if self.node.count_subscribers(t) < 1]
            if pending:
                time.sleep(0.5)
        self.assertEqual(
            [], pending,
            'rqt_drc_mini_maxwell did not subscribe %s within %s s'
            % (pending, GRAPH_TIMEOUT_SEC))

    def test_sample_drc_publishes(self):
        # is_blackout is published every 4 s cycle (steps 0 and 2)
        msgs = []
        sub = self.node.create_subscription(
            Bool, '/drc_2015_environment/is_blackout', msgs.append, 10)
        try:
            end = time.monotonic() + DATAFLOW_TIMEOUT_SEC
            while time.monotonic() < end and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.25)
            self.assertTrue(
                msgs,
                'no Bool message on /drc_2015_environment/is_blackout '
                'within %s s' % DATAFLOW_TIMEOUT_SEC)
        finally:
            self.node.destroy_subscription(sub)

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
class TestViewerPluginsShutdown(unittest.TestCase):

    # rqt processes get SIGKILLed after the SIGINT/SIGTERM grace periods
    # (stock rqt shutdown hang), so exit codes are NOT asserted.
    def test_no_tracebacks(self, proc_output):
        offenders = set()
        for io in proc_output:
            if b'Traceback' in io.text:
                offenders.add(io.process_name)
        self.assertEqual(
            set(), offenders,
            'processes printed Python tracebacks: %s' % sorted(offenders))
