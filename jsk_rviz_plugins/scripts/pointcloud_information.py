#!/usr/bin/env python3
from jsk_rviz_plugins_msgs.msg import OverlayText
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA


class PointCloudInformation(Node):
  def __init__(self):
    super(PointCloudInformation, self).__init__(
        'pointcloud_information_text')
    self.text_pub = self.create_publisher(OverlayText, 'output_text', 1)
    self.sub = self.create_subscription(
        PointCloud2, 'input', self.cloud_cb, 1)

  def cloud_cb(self, cloud):
    point_num = cloud.width * cloud.height

    point_type = ""
    for field in cloud.fields:
      point_type += field.name
      frame_id = cloud.header.frame_id

    text = OverlayText()
    text.width = 500
    text.height = 80
    text.left = 10
    text.top = 10
    text.text_size = 12.0
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    text.text = """Point Cloud Num : %d
PointType       : %s
PointFrame      : %s
""" % (point_num, point_type, frame_id)
    text.fg_color = ColorRGBA(r=25 / 255.0, g=1.0, b=240.0 / 255.0, a=1.0)
    text.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.2)
    self.text_pub.publish(text)


def main(args=None):
  rclpy.init(args=args)
  node = PointCloudInformation()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
