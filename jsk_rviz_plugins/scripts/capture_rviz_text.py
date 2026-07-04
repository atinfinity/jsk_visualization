#!/usr/bin/env python3

from jsk_rviz_plugins_msgs.msg import OverlayText
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CaptureRvizText(Node):
    def __init__(self):
        super(CaptureRvizText, self).__init__('capture_rviz_text')
        self.pub = self.create_publisher(OverlayText, 'capture_text', 1)
        self.sub = self.create_subscription(
            Int32, 'capture_count', self.callback, 1)

    def callback(self, msg):
        count = msg.data
        text = OverlayText()
        color = (52, 152, 219)
        text.fg_color.r = color[0] / 255.0
        text.fg_color.g = color[1] / 255.0
        text.fg_color.b = color[2] / 255.0
        text.fg_color.a = 1.0
        text.bg_color.a = 0.0
        text.text = "Samples: %d" % (count)
        text.width = 500
        text.height = 100
        text.left = 10
        text.top = 10
        text.text_size = 30.0
        self.pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureRvizText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
