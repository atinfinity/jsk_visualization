#!/usr/bin/env python3

# it depends on jsk_rviz_plugins

from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
import rclpy
from rclpy.node import Node


class StaticOverlayText(Node):
    def __init__(self):
        super(StaticOverlayText, self).__init__('static_overlay_text')
        self.text = self.declare_parameter('text', '').value
        self.text_interface = OverlayTextInterface(self, '~/output')
        self.timer = self.create_timer(0.1, self.publish_text)

    def publish_text(self):
        self.text_interface.publish(str(self.text))


def main(args=None):
    rclpy.init(args=args)
    node = StaticOverlayText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
