#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
# TODO(ros2): pr2_msgs is not released for ROS 2 Jazzy.
# This script only works if pr2_msgs is built from source in the workspace.
# (the ROS 1 field 'averageCharge' is assumed to be renamed to
# 'average_charge' following the ROS 2 field naming rules)
from pr2_msgs.msg import BatteryServer


class PR2Visualization(Node):
    def __init__(self):
        super(PR2Visualization, self).__init__('pr2_rviz_visualization')
        self.battery_status = {}
        battery_pub0 = self.create_publisher(
            Float32, '/visualization/battery/value0', 1)
        battery_pub1 = self.create_publisher(
            Float32, '/visualization/battery/value1', 1)
        battery_pub2 = self.create_publisher(
            Float32, '/visualization/battery/value2', 1)
        battery_pub3 = self.create_publisher(
            Float32, '/visualization/battery/value3', 1)
        self.battery_pubs = [battery_pub0,
                             battery_pub1,
                             battery_pub2,
                             battery_pub3]
        self.sub = self.create_subscription(
            BatteryServer, '/battery/server', self.batteryCB, 1)

    def batteryCB(self, msg):
        self.battery_pubs[msg.id].publish(
            Float32(data=float(msg.average_charge)))


def main(args=None):
    rclpy.init(args=args)
    node = PR2Visualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
