#!/usr/bin/env python3

import sys

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class TwistStampedAddHeader(Node):
    def __init__(self, frame_id, topic):
        super(TwistStampedAddHeader, self).__init__(
            'twist_stamped_add_header')
        self.frame_id = frame_id
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 1)
        self.sub = self.create_subscription(Twist, topic, self.callback, 1)

    def callback(self, msg):
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.frame_id
        output.twist = msg
        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv)
    if len(argv) != 3:
        print("Usage: twist_stamped_add_header frame_id topic")
    node = TwistStampedAddHeader(argv[1], argv[2])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
