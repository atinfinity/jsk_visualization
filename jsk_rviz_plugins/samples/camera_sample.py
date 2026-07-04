#!/usr/bin/env python3
# Publish a synthetic camera image + CameraInfo to exercise the
# jsk_rviz_plugin/OverlayCamera and CameraInfo displays.

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraSample(Node):
    def __init__(self):
        super(CameraSample, self).__init__('camera_sample')
        self.image_pub = self.create_publisher(Image, '~/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '~/camera_info', 1)
        self.count = 0
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        now = self.get_clock().now().to_msg()
        width, height = 320, 240

        img = Image()
        img.header.stamp = now
        img.header.frame_id = 'camera'
        img.height = height
        img.width = width
        img.encoding = 'rgb8'
        img.step = width * 3
        arr = np.zeros((height, width, 3), dtype=np.uint8)
        # scrolling grid pattern
        offset = self.count % 20
        arr[offset::20, :, 0] = 255
        arr[:, offset::20, 1] = 255
        img.data = arr.tobytes()

        info = CameraInfo()
        info.header = img.header
        info.height = height
        info.width = width
        info.k = [300.0, 0.0, 160.0,
                  0.0, 300.0, 120.0,
                  0.0, 0.0, 1.0]
        info.p = [300.0, 0.0, 160.0, 0.0,
                  0.0, 300.0, 120.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]

        self.image_pub.publish(img)
        self.info_pub.publish(info)
        self.count += 1


def main():
    rclpy.init()
    node = CameraSample()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
