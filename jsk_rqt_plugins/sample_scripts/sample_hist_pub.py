#!/usr/bin/env python3

from jsk_recognition_msgs.msg import HistogramWithRange
from jsk_recognition_msgs.msg import HistogramWithRangeBin
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SampleHistPub(Node):
    def __init__(self):
        super(SampleHistPub, self).__init__('sample_hist_pub')
        self.pub = self.create_publisher(
            Float32MultiArray, 'normal_array', 1)
        self.pub_range = self.create_publisher(
            HistogramWithRange, 'range_array', 1)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        data = np.random.normal(size=1000)
        range_msg = HistogramWithRange()
        hist, bins = np.histogram(data, bins=50)
        for v, min_value, max_value in zip(hist, bins[:-1], bins[1:]):
            msg_bin = HistogramWithRangeBin()
            msg_bin.max_value = float(max_value)
            msg_bin.min_value = float(min_value)
            msg_bin.count = int(v)
            range_msg.bins.append(msg_bin)
        msg = Float32MultiArray()
        msg.data = [float(v) for v in hist]
        self.pub.publish(msg)
        self.pub_range.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SampleHistPub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
