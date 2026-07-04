#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros

try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    # tf_transformations may not be installed, fall back to scipy
    from scipy.spatial.transform import Rotation

    def euler_from_quaternion(quaternion):
        return Rotation.from_quat(quaternion).as_euler('xyz')

from jsk_rviz_plugins_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA


class TfDiffText(Node):
    def __init__(self):
        super(TfDiffText, self).__init__('tf_diff')
        self.src1 = self.declare_parameter('src1', '').value
        self.src2 = self.declare_parameter('src2', '').value
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(OverlayText, 'diff_text', 1)
        self.timer = self.create_timer(1.0, self.publish_diff)

    def publish_diff(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.src1, self.src2, rclpy.time.Time())
            trans = transform.transform.translation
            quat = transform.transform.rotation
            pos = [trans.x, trans.y, trans.z]
            rot = [quat.x, quat.y, quat.z, quat.w]
            pos_diff = np.linalg.norm(pos)
            # quaternion to rpy
            rpy = euler_from_quaternion(rot)
            rot_diff = np.linalg.norm(rpy)
            print((pos_diff, rot_diff))
            msg = OverlayText()
            msg.width = 1000
            msg.height = 200
            msg.left = 10
            msg.top = 10
            msg.text_size = 20.0
            msg.line_width = 2
            msg.font = "DejaVu Sans Mono"
            msg.text = """%s <-> %s
pos: %f
rot: %f
        """ % (self.src1, self.src2, pos_diff, rot_diff)
            msg.fg_color = ColorRGBA(
                r=25 / 255.0, g=1.0, b=240.0 / 255.0, a=1.0)
            msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
            self.pub.publish(msg)
        except Exception:
            self.get_logger().error("ignore error")


def main(args=None):
    rclpy.init(args=args)
    node = TfDiffText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
