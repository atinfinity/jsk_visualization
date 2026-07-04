#!/usr/bin/env python3

from threading import Lock

from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Float32ToOverlayText(Node):
    def __init__(self):
        super(Float32ToOverlayText, self).__init__('float32_to_overlay_text')
        self.lock = Lock()
        self.msg = None
        self.multi_topic_msgs = dict()
        self.text_interface = OverlayTextInterface(self, '~/text')
        multi_topics = self.declare_parameter(
            'multi_topics', [],
            descriptor=ParameterDescriptor(dynamic_typing=True)).value or []
        self.format = self.declare_parameter('format', 'value: {0}').value
        if multi_topics:
            self.subs = []
            for topic in multi_topics:
                callback = MultiTopicCallback(self, topic)
                self.subs.append(self.create_subscription(
                    Float32, topic, callback.callback, 1))
            self.timer = self.create_timer(0.1, self.publish_text_multi)
        else:
            self.sub = self.create_subscription(
                Float32, '~/input', self.callback, 1)
            self.timer = self.create_timer(0.1, self.publish_text)

    def callback(self, msg):
        with self.lock:
            self.msg = msg

    def publish_text(self):
        with self.lock:
            if not self.msg:
                return
            self.text_interface.publish(self.format.format(self.msg.data))

    def publish_text_multi(self):
        with self.lock:
            if all([msg for topic, msg in self.multi_topic_msgs.items()]):
                self.text_interface.publish(self.format.format(
                    sum([msg.data
                         for topic, msg in self.multi_topic_msgs.items()])))


class MultiTopicCallback():
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic
        self.node.multi_topic_msgs[self.topic] = None

    def callback(self, msg):
        with self.node.lock:
            self.node.multi_topic_msgs[self.topic] = msg


def main(args=None):
    rclpy.init(args=args)
    node = Float32ToOverlayText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
