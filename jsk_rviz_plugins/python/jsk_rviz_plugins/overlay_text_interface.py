#!/usr/bin/env python3

from jsk_rviz_plugins_msgs.msg import OverlayText
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult


class OverlayTextInterfaceConfig(object):
    """Simple namespace object which caches the current parameter values.

    It mimics the config object of dynamic_reconfigure so that
    `config.width` style access keeps working.
    """
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class OverlayTextInterface(object):
    # (name, default, min, max) taken from cfg/OverlayTextInterface.cfg
    _INT_PARAMETERS = [
        ('width', 1200, -1, 2024),
        ('height', 800, -1, 2024),
        ('top', 10, 0, 2024),
        ('left', 10, 0, 2024),
    ]
    _DOUBLE_PARAMETERS = [
        ('text_size', 12.0, 1.0, 32.0),
        ('bg_red', 0.0, 0.0, 1.0),
        ('bg_blue', 0.0, 0.0, 1.0),
        ('bg_green', 0.0, 0.0, 1.0),
        ('bg_alpha', 0.0, 0.0, 1.0),
        ('fg_red', 25.0 / 255, 0.0, 1.0),
        ('fg_blue', 1.0, 0.0, 1.0),
        ('fg_green', 240 / 255.5, 0.0, 1.0),
        ('fg_alpha', 1.0, 0.0, 1.0),
    ]

    def __init__(self, node, topic):
        self.node = node
        values = {}
        for name, default, min_value, max_value in self._INT_PARAMETERS:
            descriptor = ParameterDescriptor(
                integer_range=[IntegerRange(
                    from_value=min_value, to_value=max_value, step=1)])
            values[name] = node.declare_parameter(
                name, default, descriptor).value
        for name, default, min_value, max_value in self._DOUBLE_PARAMETERS:
            descriptor = ParameterDescriptor(
                floating_point_range=[FloatingPointRange(
                    from_value=min_value, to_value=max_value, step=0.0)])
            values[name] = node.declare_parameter(
                name, default, descriptor).value
        self.config = OverlayTextInterfaceConfig(**values)
        node.add_on_set_parameters_callback(self.callback)
        self.pub = node.create_publisher(OverlayText, topic, 1)

    def callback(self, params):
        for param in params:
            if hasattr(self.config, param.name):
                setattr(self.config, param.name, param.value)
        return SetParametersResult(successful=True)

    def publish(self, text):
        msg = OverlayText()
        msg.text = text
        msg.width = self.config.width
        msg.height = self.config.height
        msg.top = self.config.top
        msg.left = self.config.left
        msg.fg_color.a = self.config.fg_alpha
        msg.fg_color.r = self.config.fg_red
        msg.fg_color.g = self.config.fg_green
        msg.fg_color.b = self.config.fg_blue
        msg.bg_color.a = self.config.bg_alpha
        msg.bg_color.r = self.config.bg_red
        msg.bg_color.g = self.config.bg_green
        msg.bg_color.b = self.config.bg_blue
        msg.text_size = self.config.text_size
        self.pub.publish(msg)
