#!/usr/bin/env python3

from jsk_rviz_plugins_msgs.msg import OverlayMenu
import rclpy
from rclpy.node import Node


class OverlayMenuSample(Node):
  def __init__(self):
    super(OverlayMenuSample, self).__init__('test_menu')
    self.pub = self.create_publisher(OverlayMenu, 'test_menu', 1)
    self.counter = 0
    self.timer = self.create_timer(1.0 / 5, self.publish)

  def publish(self):
    menu = OverlayMenu()
    menu.title = "The Beatles"
    menu.menus = ["John Lennon", "Paul McCartney", "George Harrison",
                  "Ringo Starr"]
    menu.current_index = self.counter % len(menu.menus)
    if self.counter % 100 == 0:
      menu.action = OverlayMenu.ACTION_CLOSE
    menu.fg_color.r = 1.0
    menu.fg_color.g = 1.0
    menu.fg_color.b = 1.0
    menu.fg_color.a = 1.0
    menu.bg_color.r = 0.0
    menu.bg_color.g = 0.0
    menu.bg_color.b = 0.0
    menu.bg_color.a = 1.0
    self.pub.publish(menu)
    self.counter = self.counter + 1


def main(args=None):
  rclpy.init(args=args)
  node = OverlayMenuSample()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
