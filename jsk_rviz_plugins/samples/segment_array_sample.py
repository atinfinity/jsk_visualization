#!/usr/bin/env python3

import time

import numpy as np
from scipy.spatial import ConvexHull

import rclpy
from jsk_recognition_msgs.msg import Segment
from jsk_recognition_msgs.msg import SegmentArray
from geometry_msgs.msg import Point


def main():
    rclpy.init()
    node = rclpy.create_node('segment_array_sample')

    pub = node.create_publisher(SegmentArray, '~/output', 1)
    rate = node.declare_parameter('rate', 1.0).value

    segment_array = SegmentArray()
    segment_array.header.frame_id = node.declare_parameter(
        'frame_id', 'map').value

    N = 30
    while rclpy.ok():
        points = np.random.rand(N, 3)
        hull = ConvexHull(points)
        segment_array.header.stamp = node.get_clock().now().to_msg()
        segment_array.segments = []
        for i in hull.simplices:
            a, b, c = points[i]
            segment_array.segments.extend(
                [Segment(start_point=Point(x=a[0], y=a[1], z=a[2]),
                         end_point=Point(x=b[0], y=b[1], z=b[2])),
                 Segment(start_point=Point(x=b[0], y=b[1], z=b[2]),
                         end_point=Point(x=c[0], y=c[1], z=c[2])),
                 Segment(start_point=Point(x=a[0], y=a[1], z=a[2]),
                         end_point=Point(x=c[0], y=c[1], z=c[2])),
                ])
        pub.publish(segment_array)
        time.sleep(1.0 / rate)


if __name__ == '__main__':
    main()
