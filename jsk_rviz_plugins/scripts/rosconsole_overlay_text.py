#!/usr/bin/env python3

import re

from jsk_rviz_plugins_msgs.msg import OverlayText
from rcl_interfaces.msg import Log
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node


class RosconsoleOverlayText(Node):
    def __init__(self):
        super(RosconsoleOverlayText, self).__init__('rosconsole_overlay_text')
        dynamic = ParameterDescriptor(dynamic_typing=True)
        self.nodes = self.declare_parameter(
            'nodes', [], descriptor=dynamic).value or []
        self.nodes_regexp = self.declare_parameter('nodes_regexp', '').value
        if self.nodes_regexp:
            self.nodes_regexp_compiled = re.compile(self.nodes_regexp)
        self.ignore_nodes = self.declare_parameter(
            'ignore_nodes', [], descriptor=dynamic).value or []
        self.exclude_regexes = self.declare_parameter(
            'exclude_regexes', [], descriptor=dynamic).value or []
        self.line_buffer_length = self.declare_parameter(
            'line_buffer_length', 100).value
        self.reverse_lines = self.declare_parameter(
            'reverse_lines', True).value
        self.lines = []
        self.sub = self.create_subscription(Log, '/rosout', self.callback, 10)
        self.pub = self.create_publisher(OverlayText, '~/output', 1)

    def colored_message(self, msg):
        cmsg = msg.msg
        cmsg = re.sub(r'\x1b\[31m', '<span style="color: red">', cmsg)
        cmsg = re.sub(r'\x1b\[32m', '<span style="color: green">', cmsg)
        cmsg = re.sub(r'\x1b\[33m', '<span style="color: yellow">', cmsg)
        cmsg = re.sub(r'\x1b\[34m', '<span style="color: blue">', cmsg)
        cmsg = re.sub(r'\x1b\[35m', '<span style="color: purple">', cmsg)
        cmsg = re.sub(r'\x1b\[36m', '<span style="color: cyan">', cmsg)
        cmsg = re.sub(r'\x1b\[0m', '</span>', cmsg)
        # NOTE: in ROS 2 the log levels are uint8 (DEBUG=10, INFO=20,
        # WARN=30, ERROR=40, FATAL=50)
        if msg.level == Log.DEBUG:
            return '<span style="color: rgb(120,120,120);">%s</span>' % cmsg
        elif msg.level == Log.INFO:
            return '<span style="color: white;">%s</span>' % cmsg
        elif msg.level == Log.WARN:
            return '<span style="color: yellow;">%s</span>' % cmsg
        elif msg.level == Log.ERROR:
            return '<span style="color: red;">%s</span>' % cmsg
        elif msg.level == Log.FATAL:
            return '<span style="color: red;">%s</span>' % cmsg

    def callback(self, msg):
        for exclude_regex in self.exclude_regexes:
            if re.match(exclude_regex, msg.msg):
                return

        if msg.name not in self.ignore_nodes:
            if msg.name in self.nodes or len(self.nodes) == 0:
                if (len(self.nodes_regexp) == 0
                        or self.nodes_regexp_compiled.match(msg.name)):
                    if self.reverse_lines:
                        self.lines = [self.colored_message(msg)] + self.lines
                        if len(self.lines) > self.line_buffer_length:
                            self.lines = self.lines[0:self.line_buffer_length]
                    else:
                        self.lines = self.lines + [self.colored_message(msg)]
                        if len(self.lines) > self.line_buffer_length:
                            self.lines = self.lines[-self.line_buffer_length:]
                    text = OverlayText()
                    text.left = 20
                    text.top = 20
                    text.width = 1200
                    text.height = 1200
                    text.fg_color.a = 1.0
                    text.fg_color.r = 0.3
                    text.text_size = 12.0
                    text.text = "\n".join(self.lines)
                    self.pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = RosconsoleOverlayText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
