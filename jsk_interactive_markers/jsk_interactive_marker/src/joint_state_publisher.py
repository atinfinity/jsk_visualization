#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatesLatcher(Node):

    def __init__(self):
        super().__init__('joint_states_latcher')
        self.joint_states = None
        self.sub = self.create_subscription(
            JointState, "joint_states_sub", self.callback, 1)
        self.pub = self.create_publisher(JointState, 'joint_states_pub', 1)
        self.timer = self.create_timer(0.1, self.publish)

    def callback(self, msg):
        print(msg)
        self.joint_states = msg

    def publish(self):
        if not rclpy.ok():
            return
        if self.joint_states:
            print(self.joint_states)
            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.joint_states)


if __name__ == "__main__":
    rclpy.init()
    node = JointStatesLatcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
