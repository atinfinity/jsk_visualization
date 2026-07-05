#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re

from rosidl_runtime_py.utilities import get_message

# mapping from rosidl (IDL) type names to ROS 1 style type names,
# so that callers can keep using slot_type like 'string' or 'float64[]'
_IDL_TO_ROS1_TYPE = {
    'boolean': 'bool',
    'octet': 'byte',
    'float': 'float32',
    'double': 'float64',
}

_SEQUENCE_RE = re.compile(r'^sequence<(.+?)(?:,\s*\d+)?>$')
_ARRAY_RE = re.compile(r'^(.+)\[\d*\]$')
_BOUNDED_STRING_RE = re.compile(r'^(w?string)<.+>$')


def _normalize_field_type(field_type):
    """Convert a rosidl field type string to a ROS 1 style one.

    e.g. 'sequence<double>' -> 'float64[]', 'double[9]' -> 'float64[]',
         'boolean' -> 'bool', 'std_msgs/ColorRGBA' -> 'std_msgs/ColorRGBA'
    """
    is_array = False
    m = _SEQUENCE_RE.match(field_type)
    if m:
        is_array = True
        field_type = m.group(1)
    else:
        m = _ARRAY_RE.match(field_type)
        if m:
            is_array = True
            field_type = m.group(1)
    m = _BOUNDED_STRING_RE.match(field_type)
    if m:
        field_type = m.group(1)
    field_type = _IDL_TO_ROS1_TYPE.get(field_type, field_type)
    # drop 'msg' namespace for ROS 1 compatible notation
    field_type = re.sub(r'^([a-zA-Z0-9_]+)/msg/', r'\1/', field_type)
    if is_array:
        field_type += '[]'
    return field_type


def get_slot_type_field_names(msg, slot_type, field_name=None, found=None):
    if field_name is None:
        field_name = ''
    if found is None:
        found = []
    if msg is None:
        return []

    for slot, slot_t in msg.get_fields_and_field_types().items():
        slot_t = _normalize_field_type(slot_t)
        deeper_field_name = field_name + '/' + slot
        if slot_t == slot_type:
            found.append(deeper_field_name)
        elif slot_t == slot_type + '[]':
            # supports array of type field like string[]
            deeper_field_name += '[]'
            found.append(deeper_field_name)
        try:
            if slot_t.endswith('[]'):
                # supports array of ros message like std_msgs/Header[]
                deeper_field_name += '[]'
                slot_t = slot_t.rstrip('[]')
            msg_impl = get_message(slot_t)
        except (AttributeError, ModuleNotFoundError, ValueError, LookupError):
            continue
        found = get_slot_type_field_names(msg_impl, slot_type,
                                          deeper_field_name, found)
    return found
