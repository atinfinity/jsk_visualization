#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

MIN_VALUE = 0.001


class TransformableJoyConfigure(Node):

    def __init__(self):
        super().__init__("transformable_joy_configure")
        self.set_x_pub = self.create_publisher(Float32, "set_x", 1)
        self.set_y_pub = self.create_publisher(Float32, "set_y", 1)
        self.set_z_pub = self.create_publisher(Float32, "set_z", 1)
        self.set_r_pub = self.create_publisher(Float32, "set_radius", 1)
        self.set_sr_pub = self.create_publisher(Float32, "set_small_radius", 1)

        self.x_max = self.declare_parameter("x_max", 10.0).value
        self.y_max = self.declare_parameter("y_max", 10.0).value
        self.z_max = self.declare_parameter("z_max", 10.0).value
        self.r_max = self.declare_parameter("r_max", 10.0).value
        self.sr_max = self.declare_parameter("sr_max", 10.0).value

        self.x_small_max = self.declare_parameter("x_small_max", 1.0).value
        self.y_small_max = self.declare_parameter("y_small_max", 1.0).value
        self.z_small_max = self.declare_parameter("z_small_max", 1.0).value
        self.r_small_max = self.declare_parameter("r_small_max", 1.0).value
        self.sr_small_max = self.declare_parameter("sr_small_max", 1.0).value

        self.sub = self.create_subscription(
            Joy, "input_joy", self.callback, 1)

    def callback(self, msg):
        a = msg.axes

        x_v1, y_v1, z_v1, r_v1, sr_v1 = a[13:18]
        x_v2, y_v2, z_v2, r_v2, sr_v2 = a[0:5]

        x = Float32()
        y = Float32()
        z = Float32()
        r = Float32()
        sr = Float32()

        x.data = self.x_max * (x_v1 + 1) / 2 + self.x_small_max * (x_v2 + 1) / 2
        y.data = self.y_max * (y_v1 + 1) / 2 + self.y_small_max * (y_v2 + 1) / 2
        z.data = self.z_max * (z_v1 + 1) / 2 + self.z_small_max * (z_v2 + 1) / 2
        r.data = self.r_max * (r_v1 + 1) / 2 + self.r_small_max * (r_v2 + 1) / 2
        sr.data = self.sr_max * (sr_v1 + 1) / 2 + self.sr_small_max * (sr_v2 + 1) / 2

        x.data = max(MIN_VALUE, x.data)
        y.data = max(MIN_VALUE, y.data)
        z.data = max(MIN_VALUE, z.data)
        r.data = max(MIN_VALUE, r.data)
        sr.data = max(MIN_VALUE, sr.data)

        self.set_x_pub.publish(x)
        self.set_y_pub.publish(y)
        self.set_z_pub.publish(z)
        self.set_r_pub.publish(r)
        self.set_sr_pub.publish(sr)


if __name__ == "__main__":
    rclpy.init()
    node = TransformableJoyConfigure()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
