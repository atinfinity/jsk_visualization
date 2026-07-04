#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <me@furushchev.ru>


from jsk_rviz_plugins_msgs.msg import OverlayMenu
import rclpy
from rclpy.node import Node

OVERLAY_MENU_TYPE = 'jsk_rviz_plugins_msgs/msg/OverlayMenu'


class OverlayMenuBridge(Node):
    def __init__(self):
        super(OverlayMenuBridge, self).__init__('overlay_menu_bridge')

        # NOTE(ros2): md5sum based check of the old OverlayMenu definition
        # does not exist in ROS 2.

        self.queue_size = self.declare_parameter('queue_size', 10).value
        self.remap_suffix = self.declare_parameter(
            'remap_suffix', 'kinetic').value
        self.publishers_ = {}
        self.subscribers_ = {}

        poll_rate = self.declare_parameter('poll_rate', 1.0).value
        self.poll_timer = self.create_timer(
            1.0 / poll_rate, self.timerCallback)

    def remap(self, topic):
        return topic + '/' + self.remap_suffix

    def messageCallback(self, msg, topic):
        try:
            self.publishers_[self.remap(topic)].publish(msg)
        except Exception as exc:
            self.get_logger().error(
                'Error on publishing to {}: {}'.format(topic, exc))

    def timerCallback(self):
        topics = [name for name, types in self.get_topic_names_and_types()
                  if OVERLAY_MENU_TYPE in types]
        subscribed_topics = list(self.subscribers_.keys())
        managed_topics = subscribed_topics + list(self.publishers_.keys())
        for topic in topics:
            if topic not in managed_topics:
                self.publishers_[self.remap(topic)] = self.create_publisher(
                    OverlayMenu, self.remap(topic), self.queue_size)
                self.subscribers_[topic] = self.create_subscription(
                    OverlayMenu, topic,
                    lambda msg, topic=topic: self.messageCallback(msg, topic),
                    self.queue_size)

                self.get_logger().info(
                    'Remapped {} -> {}'.format(topic, self.remap(topic)))

        for topic in subscribed_topics:
            if topic not in topics:
                sub = self.subscribers_.pop(topic)
                self.destroy_subscription(sub)
                pub = self.publishers_.pop(self.remap(topic))
                self.destroy_publisher(pub)

                self.get_logger().info(
                    'Stopped Remap {} -> {}'.format(
                        topic, self.remap(topic)))


def main(args=None):
    rclpy.init(args=args)
    b = OverlayMenuBridge()
    rclpy.spin(b)
    b.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
