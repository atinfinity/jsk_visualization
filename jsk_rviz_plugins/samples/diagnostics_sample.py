#!/usr/bin/env python3

import diagnostic_msgs
import diagnostic_updater
import rclpy
from rclpy.node import Node


def dummy_diagnostic(stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                 'This is a dummy diagnostics.')
    return stat


def main(args=None):
    rclpy.init(args=args)
    node = Node('diagnostics_sample')
    updater = diagnostic_updater.Updater(node)
    updater.setHardwareID('none')
    updater.add('sample task', dummy_diagnostic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
