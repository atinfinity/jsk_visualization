#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from jsk_rviz_plugins_msgs.msg import OverlayText


def quaternion_from_euler_rxyz(rx, ry, rz):
    # equivalent to tf.transformations.quaternion_from_euler(rx, ry, rz,
    # 'rxyz'): intrinsic rotations about x, then y, then z
    # returns (x, y, z, w)
    qx = (math.sin(rx / 2.0), 0.0, 0.0, math.cos(rx / 2.0))
    qy = (0.0, math.sin(ry / 2.0), 0.0, math.cos(ry / 2.0))
    qz = (0.0, 0.0, math.sin(rz / 2.0), math.cos(rz / 2.0))

    def qmult(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)

    return qmult(qmult(qx, qy), qz)


class TransformableSpacenavConfigure(Node):

    def __init__(self):
        super().__init__("transformable_spacenav_configure")
        ns = self.declare_parameter(
            'transformable_interactive_server_nodename', '').value

        self.x_max = self.declare_parameter("x_max", 0.1).value
        self.y_max = self.declare_parameter("y_max", 0.1).value
        self.z_max = self.declare_parameter("z_max", 0.1).value
        self.rx_max = self.declare_parameter("rx_max", 0.1).value
        self.ry_max = self.declare_parameter("ry_max", 0.1).value
        self.rz_max = self.declare_parameter("rz_max", 0.1).value

        self.separate_mode = self.declare_parameter("separate_mode", True).value
        self.display_separate_mode = self.declare_parameter(
            "display_separate_mode", True).value

        self.prev_b = 0
        self.translate_mode = True

        self.set_pose_pub = self.create_publisher(Pose, ns + "/add_pose", 1)
        self.send_text_pub = self.create_publisher(
            OverlayText, "separate_mode_text", 1)

        self.sub = self.create_subscription(Joy, "input_joy", self.callback, 1)

    def callback(self, msg):
        a = msg.axes
        b = msg.buttons

        if self.prev_b == 0 and (b[0] == 1 or b[1] == 1):
            self.translate_mode = not self.translate_mode

        if b[0] == 1 or b[1] == 1:
            self.prev_b = 1
        else:
            self.prev_b = 0

        x_v1, y_v1, z_v1, rx_v1, ry_v1, rz_v1 = a

        target_pose = Pose()
        if not self.separate_mode or (self.separate_mode and
                                      self.translate_mode):
            target_pose.position.x = x_v1 * self.x_max
            target_pose.position.y = y_v1 * self.y_max
            target_pose.position.z = z_v1 * self.z_max

        target_pose.orientation.w = 1.0
        if not self.separate_mode or (self.separate_mode and
                                      not self.translate_mode):
            q = quaternion_from_euler_rxyz(
                rx_v1 * self.rx_max, ry_v1 * self.ry_max, rz_v1 * self.rz_max)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

        self.set_pose_pub.publish(target_pose)
        if self.translate_mode and self.separate_mode:
            self.publish_mode_text("TranslateMode")
        elif self.separate_mode:
            self.publish_mode_text("RotateMode")

    def publish_mode_text(self, message):
        send_text = OverlayText()
        send_text.text = message
        send_text.top = 100
        send_text.left = 0
        send_text.width = 300
        send_text.height = 50

        send_text.bg_color.r = 0.9
        send_text.bg_color.b = 0.9
        send_text.bg_color.g = 0.9
        send_text.bg_color.a = 0.1
        send_text.fg_color.r = 0.3
        send_text.fg_color.g = 0.8
        send_text.fg_color.b = 0.3
        send_text.fg_color.a = 1.0
        send_text.line_width = 1
        send_text.text_size = 30.0
        self.send_text_pub.publish(send_text)


if __name__ == "__main__":
    rclpy.init()
    node = TransformableSpacenavConfigure()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
