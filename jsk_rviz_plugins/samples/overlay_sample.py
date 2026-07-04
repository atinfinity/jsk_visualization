#!/usr/bin/env python3
import math

from jsk_rviz_plugins_msgs.msg import OverlayText
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32


class OverlaySample(Node):
  def __init__(self):
    super(OverlaySample, self).__init__('overlay_sample')
    self.text_pub = self.create_publisher(OverlayText, 'text_sample', 1)
    self.value_pub = self.create_publisher(Float32, 'value_sample', 1)
    self.counter = 0
    self.rate = 100
    self.timer = self.create_timer(1.0 / self.rate, self.publish)

  def publish(self):
    self.counter = self.counter + 1
    counter = self.counter
    text = OverlayText()
    theta = counter % 255 / 255.0
    text.width = 400
    text.height = 600
    #text.height = 600
    text.left = 10
    text.top = 10
    text.text_size = 12.0
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    text.text = """This is OverlayText plugin.
The update rate is %d Hz.
You can write several text to show to the operators.
New line is supported and automatical wrapping text is also supported.
And you can choose font, this text is now rendered by '%s'

You can specify background color and foreground color separatelly.

Of course, the text is not needed to be fixed, see the counter: %d.

You can change text color like <span style="color: red;">this</span>
by using <span style="font-style: italic;">css</style>.
  """ % (self.rate, text.font, counter)
    text.fg_color = ColorRGBA(r=25 / 255.0, g=1.0, b=240.0 / 255.0, a=1.0)
    text.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.2)
    self.text_pub.publish(text)
    self.value_pub.publish(
        Float32(data=math.sin(counter * math.pi * 2 / 100)))
    if int(counter % 500) == 0:
      self.get_logger().debug('This is ROS_DEBUG.')
    elif int(counter % 500) == 100:
      self.get_logger().info('This is ROS_INFO.')
    elif int(counter % 500) == 200:
      self.get_logger().warning('This is ROS_WARN.')
    elif int(counter % 500) == 300:
      self.get_logger().error('This is ROS_ERROR.')
    elif int(counter % 500) == 400:
      self.get_logger().fatal('This is ROS_FATAL.')


def main(args=None):
  rclpy.init(args=args)
  node = OverlaySample()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
