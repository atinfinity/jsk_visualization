#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class RelayCameraInfo(Node):
    def __init__(self):
        super(RelayCameraInfo, self).__init__('relay_camera_info')
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.pub = self.create_publisher(CameraInfo, 'output', 1)
        self.sub = self.create_subscription(
            CameraInfo, 'input', self.callback, 1)

    def callback(self, info):
        info.header.frame_id = self.frame_id
        self.pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = RelayCameraInfo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
