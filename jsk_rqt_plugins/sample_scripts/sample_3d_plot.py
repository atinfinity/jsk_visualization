#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class Sample3DPlot(Node):
    def __init__(self):
        super(Sample3DPlot, self).__init__('sample_3d_plot')
        self.pub1 = self.create_publisher(Float32, '~/output1', 1)
        self.pub2 = self.create_publisher(Float32, '~/output2', 1)
        self.pub3 = self.create_publisher(Float32, '~/output3', 1)
        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        now = time.time()
        self.pub1.publish(Float32(data=float(now % 1)))
        self.pub2.publish(Float32(data=float(np.cos(now))))
        self.pub3.publish(Float32(data=float(np.sin(now))))


def main(args=None):
    rclpy.init(args=args)
    node = Sample3DPlot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
