#!/usr/bin/env python3


import copy
import math

import numpy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers import InteractiveMarkerServer


def quaternion_from_euler(ai, aj, ak):
    # tf.transformations.quaternion_from_euler with default 'sxyz' axes,
    # returns (x, y, z, w)
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci, si = math.cos(ai), math.sin(ai)
    cj, sj = math.cos(aj), math.sin(aj)
    ck, sk = math.cos(ak), math.sin(ak)
    return (si * cj * ck - ci * sj * sk,
            ci * sj * ck + si * cj * sk,
            ci * cj * sk - si * sj * ck,
            ci * cj * ck + si * sj * sk)


def quaternion_matrix(q):
    # tf.transformations.quaternion_matrix, q = (x, y, z, w)
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < numpy.finfo(float).eps:
        return numpy.identity(4)
    s = 2.0 / n
    return numpy.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w), 0.0],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w), 0.0],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0]])


class InteractiveMarkerTest(Node):
    poses = []
    colors = []
    status = []
    size = 2
    start_time = False
    done_time = False

    def processFeedback(self, feedback):
        if not self.start_time:
            self.start_time = self.get_clock().now()
        self.feedback = feedback
        size = self.size
        i = min(range(size*size*size),key=lambda i:numpy.linalg.norm(numpy.array([self.poses[i].position.x,self.poses[i].position.y,self.poses[i].position.z])-
                                                                     numpy.array([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z])))
        p = self.poses[i]
        pos = numpy.linalg.norm(numpy.array([p.position.x,p.position.y,p.position.z])-
                                numpy.array([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z]))
        rot = math.acos(numpy.dot(quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])[0:3,2],
                                  quaternion_matrix([feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w])[0:3,2]))
        self.get_logger().info('error %6.3f / %6.3f'%(pos, numpy.rad2deg(rot)))
        if pos < self.position_threshold and rot < numpy.deg2rad(self.rotation_threshold):
            self.colors[i] = ColorRGBA(r=1.0,g=0.2,b=0.2,a=0.8)
            self.status[i] = self.get_clock().now() - self.start_time

    def poseCB(self, msg):
        self.server.setPose('control', msg.pose, msg.header)
        self.server.applyChanges()
        self.processFeedback(msg)

    def __init__(self):
        super().__init__('publish_interactive_goal_marker')
        self.pub_goal = self.create_publisher(
            MarkerArray, 'interactive_goal_marker', 1)
        self.ui_type = self.declare_parameter('ui_type', 'mouse').value
        # setup
        size = self.size
        space = 0.5
        share_dir = get_package_share_directory('jsk_interactive_test')
        # meshes are installed to share/jsk_interactive_test/scripts/
        hand_mesh = 'file://' + share_dir + '/scripts/RobotHand.dae'
        hand_mesh_scale = Vector3(x=0.2, y=0.2, z=0.2)
        goal_mesh = 'file://' + share_dir + '/scripts/Bottle.dae'
        goal_mesh_scale = Vector3(x=1.0, y=1.0, z=1.0)

        self.position_threshold = self.declare_parameter(
            'position_threshold', 0.01).value  # 1 [cm]
        self.rotation_threshold = self.declare_parameter(
            'rotation_threshold', 2.0).value   # 2 [deg]

        self.sub_pose = self.create_subscription(
            PoseStamped, '/goal_marker/move_marker', self.poseCB, 1)
        #
        for i in range(size*size*size):
            x = space * ((i%(size) - size/2) + numpy.random.normal(0,0.1))
            y = space * (((i/(size))%size - size/2) + numpy.random.normal(0,0.1))
            z = space * ((i/(size*size) - size/2) + numpy.random.normal(0,0.1))
            q = quaternion_from_euler(*numpy.random.normal(0,0.7,3))
            self.poses.append(Pose(position=Point(x=x,y=y,z=z),
                                   orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])))
            self.colors.append(ColorRGBA(r=0.8,g=0.8,b=0.8,a=0.8))
            self.status.append(False)

        # interactive marker
        server = InteractiveMarkerServer(self, "goal_marker")
        self.server = server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "control"
        int_marker.scale = 0.3

        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale = hand_mesh_scale
        mesh_marker.color = ColorRGBA(r=0.2,g=0.2,b=1.0,a=0.8)
        mesh_marker.mesh_resource = hand_mesh

        target_marker = Marker()
        target_marker.type = Marker.CYLINDER
        target_marker.scale = Vector3(x=0.07,y=0.07,z=0.1)
        target_marker.color = ColorRGBA(r=0.4,g=0.4,b=1.0,a=0.6)

        mesh_control = InteractiveMarkerControl()
        mesh_control.always_visible = True
        mesh_control.markers.append(mesh_marker)
        mesh_control.markers.append(target_marker)

        int_marker.controls.append(mesh_control)

        if self.declare_parameter('make_interactive_marker_arrow', False).value:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(copy.deepcopy(control))
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(copy.deepcopy(control))
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(copy.deepcopy(control))
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(copy.deepcopy(control))
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(copy.deepcopy(control))
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(copy.deepcopy(control))

        server.insert(int_marker, feedback_callback=self.processFeedback)
        server.applyChanges()

        self.goal_mesh = goal_mesh
        self.goal_mesh_scale = goal_mesh_scale
        self.timer = self.create_timer(1.0 / 5.0, self.publish_markers)

    def publish_markers(self):
        if not rclpy.ok():
            return
        size = self.size
        array = MarkerArray()
        for i in range(size*size*size):
            msg = Marker()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.mesh_use_embedded_materials = False
            msg.mesh_resource = self.goal_mesh
            msg.type = Marker.MESH_RESOURCE
            msg.scale = self.goal_mesh_scale
            msg.color = self.colors[i]
            msg.pose = self.poses[i]
            msg.lifetime = Duration(seconds=10).to_msg()
            msg.id = i
            array.markers.append(msg)
        if self.done_time:
            msg = Marker()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.type = Marker.TEXT_VIEW_FACING
            msg.scale.z = 0.14
            msg.color.r = 1.0
            msg.color.g = 1.0
            msg.color.b = 1.0
            msg.color.a = 1.0
            msg.text = 'Done, time = %5.2f'%(self.done_time)
            msg.lifetime = Duration(seconds=10).to_msg()
            msg.id = size*size*size
            array.markers.append(msg)
        self.pub_goal.publish(array)
        if self.start_time:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(
                "%2d/%d %5.2f"%(len([x for x in self.status if x is not False]),
                                len(self.status), elapsed))
        if all(self.status):
            if self.done_time:
                self.get_logger().info("Done.. %5.2f"%self.done_time)
            else:
                self.done_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            for s in self.status:
                self.get_logger().info("%5.2f"%(s.nanoseconds / 1e9))


if __name__ == '__main__':
    rclpy.init()
    node = InteractiveMarkerTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
