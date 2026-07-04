#!/usr/bin/env python3
# Publish a synthetic sphere point cloud with normal_{x,y,z} fields to
# exercise the jsk_rviz_plugin/NormalDisplay plugin.

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class NormalSample(Node):
    def __init__(self):
        super(NormalSample, self).__init__('normal_sample')
        self.pub = self.create_publisher(PointCloud2, '~/output', 1)
        self.timer = self.create_timer(1.0, self.publish_cloud)

    def publish_cloud(self):
        points = []
        for theta in np.linspace(0, math.pi, 30):
            for phi in np.linspace(0, 2 * math.pi, 60):
                nx = math.sin(theta) * math.cos(phi)
                ny = math.sin(theta) * math.sin(phi)
                nz = math.cos(theta)
                points.append((nx, ny, nz, nx, ny, nz, 0.0))
        data = np.array(points, dtype=np.float32)

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name=n, offset=4 * i,
                       datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(
                ['x', 'y', 'z',
                 'normal_x', 'normal_y', 'normal_z', 'curvature'])
        ]
        msg.is_bigendian = False
        msg.point_step = 28
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = data.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NormalSample()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
