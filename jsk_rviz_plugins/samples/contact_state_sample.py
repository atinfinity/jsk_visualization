#!/usr/bin/env python3
# ROS 2 port. Publishes a random ContactStatesStamped for the configured
# links so the contact_state_marker sample has data to visualise.

import random

import rclpy
from rclpy.node import Node

from hrpsys_ros_bridge.msg import ContactState
from hrpsys_ros_bridge.msg import ContactStateStamped
from hrpsys_ros_bridge.msg import ContactStatesStamped


class ContactStateSample(Node):

    def __init__(self):
        super().__init__('contact_state_sample')
        self.pub = self.create_publisher(ContactStatesStamped, '~/output', 1)
        self.link_names = self.declare_parameter(
            'links', ['r_shoulder_pan_link']).value
        self.timer = self.create_timer(1.0, self._timer_cb)

    def _timer_cb(self):
        states = ContactStatesStamped()
        now = self.get_clock().now().to_msg()
        states.header.stamp = now
        for link_name in self.link_names:
            state = ContactStateStamped()
            state.header.frame_id = link_name
            state.header.stamp = now
            if random.random() < 0.5:
                state.state.state = ContactState.ON
            else:
                state.state.state = ContactState.OFF
            states.states.append(state)
        self.pub.publish(states)


def main(args=None):
    rclpy.init(args=args)
    node = ContactStateSample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
