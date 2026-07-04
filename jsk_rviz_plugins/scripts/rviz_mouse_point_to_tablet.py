#!/usr/bin/env python3

from threading import Lock

from geometry_msgs.msg import PointStamped
from jsk_gui_msgs.msg import Tablet
import rclpy
from rclpy.node import Node


class RvizMousePointToTablet(Node):
    def __init__(self):
        super(RvizMousePointToTablet, self).__init__(
            'rviz_mouse_point_to_tablet')
        self.lock = Lock()
        self.latest_mouse_point = None
        self.pub = self.create_publisher(Tablet, '/Tablet/Command', 1)
        self.sub = self.create_subscription(
            PointStamped, '/rviz/current_mouse_point',
            self.point_callback, 1)
        self.timer = self.create_timer(0.3, self.timer_callback)

    def point_callback(self, msg):
        with self.lock:
            self.latest_mouse_point = msg

    def timer_callback(self):
        # print (latest_mouse_point and latest_mouse_point.point,
        #        prev_mouse_point and prev_mouse_point.point)
        with self.lock:
            if not self.latest_mouse_point:
                return
            next_x = self.latest_mouse_point.point.x * 640
            next_y = self.latest_mouse_point.point.y * 480
            msg = Tablet()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.action.task_name = "MoveCameraCenter"
            # chop
            msg.action.touch_x = next_x
            msg.action.touch_y = next_y
            self.get_logger().info(
                "neck actino to (%f, %f)" % (msg.action.touch_x,
                                             msg.action.touch_y))
            self.pub.publish(msg)
            self.latest_mouse_point = None


def main(args=None):
    rclpy.init(args=args)
    node = RvizMousePointToTablet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
