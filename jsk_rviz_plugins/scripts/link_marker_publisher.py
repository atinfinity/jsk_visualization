#!/usr/bin/env python3

"""
Publish a visualization_marker for specified link
"""

import math
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
        # marker built from the link's <visual> geometry; either a
        # MESH_RESOURCE (mesh geometry) or a primitive (box/cylinder/
        # sphere). None until the robot description is parsed.
        self.marker_geometry = None
        self.marker_origin = ([0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 1.0))
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
        visual = None
        for link in doc.getElementsByTagName('link'):
            if self.link_name == link.getAttribute('name'):
                visual = link.getElementsByTagName('visual').item(0)
                break
        if visual is None:
            raise Exception(
                "Cannot find link with a visual: {0}".format(self.link_name))
        geometry = visual.getElementsByTagName('geometry').item(0)
        self.marker_geometry = self.geometry_to_marker(geometry)
        # keep the marker aligned with the link's visual origin so the
        # highlight overlaps the actual geometry (meshes usually bake the
        # origin in, but primitives are placed via <origin>)
        self.marker_origin = self.parse_origin(
            visual.getElementsByTagName('origin').item(0))

    @staticmethod
    def parse_origin(origin):
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.getAttribute('xyz'):
                xyz = [float(v) for v in origin.getAttribute('xyz').split()]
            if origin.getAttribute('rpy'):
                rpy = [float(v) for v in origin.getAttribute('rpy').split()]
        cr, cp, cy = (math.cos(a * 0.5) for a in rpy)
        sr, sp, sy = (math.sin(a * 0.5) for a in rpy)
        quat = (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)
        return xyz, quat

    def geometry_to_marker(self, geometry):
        # Build the (type, mesh_resource, scale) tuple describing the
        # marker from a URDF <geometry>. Supports mesh geometry (as in
        # ROS 1) plus box/cylinder/sphere primitives, so the marker can
        # highlight links that do not use a mesh.
        s = float(self.scale)
        mesh = geometry.getElementsByTagName('mesh').item(0)
        if mesh is not None:
            return (Marker.MESH_RESOURCE, mesh.getAttribute('filename'),
                    (s, s, s))
        box = geometry.getElementsByTagName('box').item(0)
        if box is not None:
            x, y, z = (float(v) for v in box.getAttribute('size').split())
            return (Marker.CUBE, '', (x * s, y * s, z * s))
        cylinder = geometry.getElementsByTagName('cylinder').item(0)
        if cylinder is not None:
            r = float(cylinder.getAttribute('radius'))
            length = float(cylinder.getAttribute('length'))
            return (Marker.CYLINDER, '', (2 * r * s, 2 * r * s, length * s))
        sphere = geometry.getElementsByTagName('sphere').item(0)
        if sphere is not None:
            d = 2 * float(sphere.getAttribute('radius')) * s
            return (Marker.SPHERE, '', (d, d, d))
        raise Exception(
            "Unsupported visual geometry for link: {0}".format(self.link_name))

    def publish_marker(self):
        if self.marker_geometry is None:
            return
        marker_type, mesh_resource, scale = self.marker_geometry
        marker = Marker()
        marker.header.frame_id = self.link_name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker_type
        marker.color.a = float(self.alpha)
        marker.color.r = float(self.rgb[0])
        marker.color.g = float(self.rgb[1])
        marker.color.b = float(self.rgb[2])
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        (marker.pose.position.x, marker.pose.position.y,
         marker.pose.position.z) = self.marker_origin[0]
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z,
         marker.pose.orientation.w) = self.marker_origin[1]
        marker.mesh_resource = mesh_resource
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
