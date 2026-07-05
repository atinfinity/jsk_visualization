#!/usr/bin/env python3

"""
Colorize links according to hrpsys_ros_bridge/ContactState using
visualization_msgs/MarkerArray (ROS 2 port).

The former dynamic_reconfigure ContactStateMarker parameters are plain
node parameters here, and the robot description is taken from the
/robot_description topic (published by robot_state_publisher) when it is
not given directly. Both mesh and primitive (box/cylinder/sphere) link
geometries are supported so the sample works with a mesh-less URDF.
"""

import math
from xml.dom.minidom import parseString

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from hrpsys_ros_bridge.msg import ContactState, ContactStatesStamped


class ContactStateMarker(Node):

    def __init__(self):
        super().__init__('contact_state_marker')
        # parameters (formerly the ContactStateMarker dynamic_reconfigure cfg)
        self.use_parent_link = self.declare_parameter('use_parent_link', False).value
        self.marker_scale = self.declare_parameter('marker_scale', 1.02).value
        self.on_color = (
            self.declare_parameter('on_red', 1.0).value,
            self.declare_parameter('on_green', 0.0).value,
            self.declare_parameter('on_blue', 0.0).value,
            self.declare_parameter('on_alpha', 0.8).value)
        self.visualize_off = self.declare_parameter('visualize_off', False).value
        self.off_color = (
            self.declare_parameter('off_red', 0.0).value,
            self.declare_parameter('off_green', 0.0).value,
            self.declare_parameter('off_blue', 1.0).value,
            self.declare_parameter('off_alpha', 0.8).value)

        # link_name -> (marker_type, mesh_resource, scale, origin)
        self.links = {}
        self.robot_xml = None

        self.pub = self.create_publisher(MarkerArray, '~/marker', 1)
        robot_description = self.declare_parameter('robot_description', '').value
        if robot_description:
            self.parse_robot_description(robot_description)
        else:
            # fall back to the /robot_description topic (transient_local)
            self.description_sub = self.create_subscription(
                String, '/robot_description', self.description_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sub = self.create_subscription(
            ContactStatesStamped, '~/input', self.callback, 1)

    def description_callback(self, msg):
        self.parse_robot_description(msg.data)

    def parse_robot_description(self, robot_description):
        self.robot_xml = robot_description
        doc = parseString(robot_description)
        links = {}
        for link in doc.getElementsByTagName('link'):
            visual = link.getElementsByTagName('visual').item(0)
            if visual is None:
                continue
            geometry = visual.getElementsByTagName('geometry').item(0)
            if geometry is None:
                continue
            try:
                geom = self.geometry_to_marker(geometry)
            except Exception as e:  # unsupported geometry, skip the link
                self.get_logger().warn(str(e))
                continue
            origin = self.parse_origin(
                visual.getElementsByTagName('origin').item(0))
            links[link.getAttribute('name')] = geom + (origin,)
        self.links = links

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
        s = float(self.marker_scale)
        mesh = geometry.getElementsByTagName('mesh').item(0)
        if mesh is not None:
            return (Marker.MESH_RESOURCE, mesh.getAttribute('filename'), (s, s, s))
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
        raise Exception('Unsupported visual geometry')

    def parent_link(self, link_name):
        # Resolve the parent link via urdf_parser_py (only used when
        # use_parent_link is set).
        try:
            from urdf_parser_py.urdf import URDF
            robot = URDF.from_xml_string(self.robot_xml)
            chain = robot.get_chain(robot.get_root(), link_name)
            return chain[-3]
        except Exception as e:
            self.get_logger().warn(
                'use_parent_link failed for {0}: {1}'.format(link_name, e))
            return link_name

    def callback(self, msgs):
        if not self.links:
            return
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, msg in enumerate(msgs.states):
            link_name = msg.header.frame_id
            if self.use_parent_link:
                link_name = self.parent_link(link_name)
            if link_name not in self.links:
                continue
            marker_type, mesh_resource, scale, origin = self.links[link_name]
            marker = Marker()
            marker.header.frame_id = link_name
            marker.header.stamp = now
            marker.type = marker_type
            marker.id = i
            marker.mesh_resource = mesh_resource
            marker.frame_locked = True
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            (marker.pose.position.x, marker.pose.position.y,
             marker.pose.position.z) = origin[0]
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = origin[1]
            if msg.state.state == ContactState.ON:
                color = self.on_color
            else:
                color = self.off_color
                if not self.visualize_off:
                    marker.action = Marker.DELETE
            (marker.color.r, marker.color.g,
             marker.color.b, marker.color.a) = (float(c) for c in color)
            marker_array.markers.append(marker)
        self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ContactStateMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
