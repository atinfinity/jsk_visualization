#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Plain pytest port of the ROS 1 nose test for
# jsk_rqt_plugins.util.get_slot_type_field_names.
# No ROS graph is needed: message classes are resolved through
# rosidl_runtime_py and the util maps rosidl (IDL) field type names back
# to ROS 1 style names ('string', 'float64[]', 'std_msgs/ColorRGBA', ...),
# so the expected values below are identical to the ROS 1 test.

from rosidl_runtime_py.utilities import get_message

from jsk_rqt_plugins.util import get_slot_type_field_names


def test_get_slot_type_field_names_plain_type():
    # test for type as slot_type
    msg = get_message('jsk_rviz_plugins_msgs/msg/OverlayText')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert field_names == ['/font', '/text']


def test_get_slot_type_field_names_msg_type():
    # test for msg as slot_type (ROS 1 style type name)
    msg = get_message('jsk_rviz_plugins_msgs/msg/OverlayText')
    field_names = get_slot_type_field_names(msg,
                                            slot_type='std_msgs/ColorRGBA')
    assert field_names == ['/bg_color', '/fg_color']


def test_get_slot_type_field_names_type_array():
    # test for type array
    msg = get_message('jsk_recognition_msgs/msg/Histogram')
    field_names = get_slot_type_field_names(msg, slot_type='float64[]')
    assert field_names == ['/histogram']


def test_get_slot_type_field_names_msg_array():
    # test for msg array (fields of messages inside arrays are reported
    # with a '[]' suffix on the array field)
    msg = get_message('diagnostic_msgs/msg/DiagnosticArray')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert field_names == ['/header/frame_id', '/status[]/name',
                           '/status[]/message', '/status[]/hardware_id',
                           '/status[]/values[]/key',
                           '/status[]/values[]/value']
