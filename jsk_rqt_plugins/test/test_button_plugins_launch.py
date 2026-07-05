#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# launch_testing test for the button plugins:
#   * sample_service_buttons.py must advertise the six /dummy/button*
#     services with the expected types,
#   * rqt_service_buttons (standalone, offscreen, layout_yaml_file
#     parameter like sample/sample_service_buttons.launch.py) must come up
#     and stay alive (it would exit on an unreadable/invalid layout yaml),
#   * rqt_yn_btn must advertise the /rqt_yn_btn service
#     (jsk_gui_msgs/srv/YesNo).
#
# /rqt_yn_btn is deliberately NOT called: the service handler blocks until
# a human clicks the Yes/No button (60 s poll loop), which cannot happen
# headless.  Button *clicks* of rqt_service_buttons are GUI interactions
# and are not simulated either; service availability is what the ROS 1
# test asserted, too.
#
# NOTE (filename): pytest cannot import modules with dots in the name, so
# this file is called test_button_plugins_launch.py, not
# test_button_plugins.launch.py.

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

LIB_DIR = os.path.join(
    get_package_prefix('jsk_rqt_plugins'), 'lib', 'jsk_rqt_plugins')

ALIVE_TIME_SEC = 8.0
GRAPH_TIMEOUT_SEC = 30.0

EXPECTED_DUMMY_SERVICES = {
    '/dummy/buttonA': 'std_srvs/srv/SetBool',
    '/dummy/buttonB': 'std_srvs/srv/SetBool',
    '/dummy/buttonC': 'std_srvs/srv/SetBool',
    '/dummy/buttonD': 'std_srvs/srv/Trigger',
    '/dummy/buttonE': 'std_srvs/srv/Empty',
    '/dummy/buttonF': 'std_srvs/srv/Empty',
}


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
        'sample_service_buttons': _process('sample_service_buttons.py',
                                           name='sample_service_buttons'),
        'rqt_service_buttons': _process(
            'rqt_service_buttons',
            ['--ros-args', '-p',
             'layout_yaml_file:=package://jsk_rqt_plugins/resource/'
             'service_button_layout.yaml']),
        'rqt_yn_btn': _process('rqt_yn_btn'),
    }
    return launch.LaunchDescription(
        list(procs.values()) + [launch_testing.actions.ReadyToTest()]
    ), {'procs': procs}


class TestButtonPlugins(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_button_plugins_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_services(self, expected, timeout):
        """Wait until all expected {name: type} services are advertised."""
        end = time.monotonic() + timeout
        services = {}
        while time.monotonic() < end:
            services = {
                name: types
                for name, types in self.node.get_service_names_and_types()}
            if all(name in services and expected_type in services[name]
                   for name, expected_type in expected.items()):
                return services
            time.sleep(0.5)
        return services

    def test_dummy_button_services_available(self):
        services = self._wait_for_services(
            EXPECTED_DUMMY_SERVICES, GRAPH_TIMEOUT_SEC)
        for name, expected_type in EXPECTED_DUMMY_SERVICES.items():
            with self.subTest(service=name):
                self.assertIn(
                    name, services,
                    '%s not advertised within %s s (found: %s)'
                    % (name, GRAPH_TIMEOUT_SEC, sorted(services)))
                self.assertIn(
                    expected_type, services[name],
                    '%s has type %s, expected %s'
                    % (name, services[name], expected_type))

    def test_yn_btn_service_advertised(self):
        # do NOT call the service: it blocks waiting for a button click
        expected = {'/rqt_yn_btn': 'jsk_gui_msgs/srv/YesNo'}
        services = self._wait_for_services(expected, GRAPH_TIMEOUT_SEC)
        self.assertIn(
            '/rqt_yn_btn', services,
            '/rqt_yn_btn not advertised within %s s (found: %s)'
            % (GRAPH_TIMEOUT_SEC, sorted(services)))
        self.assertIn('jsk_gui_msgs/srv/YesNo', services['/rqt_yn_btn'])

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
class TestButtonPluginsShutdown(unittest.TestCase):

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
