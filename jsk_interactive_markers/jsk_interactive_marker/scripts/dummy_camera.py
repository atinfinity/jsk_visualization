#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class DummyCamera(Node):

    def __init__(self):
        super().__init__("dummy_camera")
        self.pub = self.create_publisher(Image, "~/image", 1)
        self.pub_info = self.create_publisher(CameraInfo, "~/camera_info", 1)
        self.frame_id = self.declare_parameter(
            'frame_id', 'dummy_camera').value
        self.width = self.declare_parameter('width', 640).value
        self.height = self.declare_parameter('height', 480).value
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        if not rclpy.ok():
            return
        width = self.width
        height = self.height
        now = self.get_clock().now().to_msg()
        dummy_camera_info = CameraInfo()
        dummy_camera_info.width = width
        dummy_camera_info.height = height
        dummy_camera_info.header.frame_id = self.frame_id
        dummy_camera_info.header.stamp = now
        dummy_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        dummy_camera_info.k = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        dummy_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        dummy_camera_info.p = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        dummy_camera_info.distortion_model = "plumb_bob"

        dummy_img = Image()
        dummy_img.width = width
        dummy_img.height = height
        dummy_img.header.frame_id = self.frame_id
        dummy_img.header.stamp = now
        dummy_img.step = width
        dummy_img.encoding = "mono8"
        dummy_img.data = [0] * width * height
        try:
            self.pub_info.publish(dummy_camera_info)
            self.pub.publish(dummy_img)
        except Exception:
            # ignore publish errors while the context is shutting down
            if rclpy.ok():
                raise


if __name__ == "__main__":
    rclpy.init()
    node = DummyCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
