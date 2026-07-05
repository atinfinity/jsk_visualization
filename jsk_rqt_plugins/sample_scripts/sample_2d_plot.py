#!/usr/bin/env python3

import math
from random import random

from jsk_recognition_msgs.msg import PlotData
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Sample2DPlot(Node):
    def __init__(self):
        super(Sample2DPlot, self).__init__('sample_2d_plot')
        self.pub = self.create_publisher(PlotData, '~/output', 1)
        self.offset = 0.0
        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        msg = PlotData()
        msg.xs = [float(x) for x in np.arange(0, 5, 0.1)]
        msg.ys = [math.sin(x + self.offset) for x in msg.xs]
        msg.label = 'sample data'
        if random() < 0.5:
            msg.type = PlotData.LINE
        else:
            msg.type = PlotData.SCATTER
        msg.fit_line = random() < 0.5
        msg.fit_line_ransac = random() < 0.5
        self.pub.publish(msg)
        self.offset = self.offset + 0.1


def main(args=None):
    rclpy.init(args=args)
    node = Sample2DPlot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
