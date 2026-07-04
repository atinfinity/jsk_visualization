#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PieChartSample(Node):

    def __init__(self):
        super(PieChartSample, self).__init__('piechart_sample')
        self.pub = self.create_publisher(
            Float32, '/sample_piechart', 1)
        self.timer = self.create_timer(0.1, self._timer_cb)
        self.count = 0

    def _timer_cb(self):
        msg = Float32()
        msg.data = abs(math.sin(3.14 * self.count / 100.0))
        self.count = self.count + 1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    app = PieChartSample()
    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
