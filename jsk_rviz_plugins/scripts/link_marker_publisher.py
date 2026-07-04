#!/usr/bin/env python3

"""
Publish a visualization_marker for specified link
"""

from xml.dom.minidom import parseString

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class LinkMarkerPublisher(Node):
    def __init__(self):
        super(LinkMarkerPublisher, self).__init__('link_marker_publisher')
        self.link_name = self.declare_parameter('link', '').value
        self.rgb = self.declare_parameter(
            'rgb', [1, 0, 0],
            descriptor=ParameterDescriptor(dynamic_typing=True)).value
        self.alpha = self.declare_parameter('alpha', 1.0).value
        self.scale = self.declare_parameter('scale', 1.02).value
        self.mesh_file = None
        robot_description = self.declare_parameter(
            'robot_description', '').value
        if robot_description:
            self.parse_robot_description(robot_description)
        else:
            # NOTE(ros2): fall back to the /robot_description topic
            # (published by robot_state_publisher with transient_local QoS)
            self.description_sub = self.create_subscription(
                String, '/robot_description', self.description_callback,
                QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Marker, '~/marker', 1)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def description_callback(self, msg):
        self.parse_robot_description(msg.data)

    def parse_robot_description(self, robot_description):
        # Parse robot_description using minidom directly
        # because urdf_parser_py cannot read PR2 urdf
        doc = parseString(robot_description)
        links = doc.getElementsByTagName('link')
        mesh_file = None
        for link in links:
            if self.link_name == link.getAttribute('name'):
                visual_mesh = link.getElementsByTagName('visual').item(0).getElementsByTagName('mesh').item(0)
                mesh_file = visual_mesh.getAttribute('filename')
                break
        if not mesh_file:
            raise Exception("Cannot find link: {0}".format(self.link_name))
        self.mesh_file = mesh_file

    def publish_marker(self):
        if not self.mesh_file:
            return
        marker = Marker()
        marker.header.frame_id = self.link_name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.MESH_RESOURCE
        marker.color.a = float(self.alpha)
        marker.color.r = float(self.rgb[0])
        marker.color.g = float(self.rgb[1])
        marker.color.b = float(self.rgb[2])
        marker.scale.x = float(self.scale)
        marker.scale.y = float(self.scale)
        marker.scale.z = float(self.scale)
        marker.mesh_resource = self.mesh_file
        marker.frame_locked = True
        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LinkMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
