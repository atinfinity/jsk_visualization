#!/usr/bin/env python3

import math
import numpy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

INTERACTIVE_MARKER_TOPIC = '/goal_marker'


def euler_from_quaternion(q):
    # q = (x, y, z, w) -> (roll, pitch, yaw), same as tf.transformations
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def quaternion_from_euler(ai, aj, ak):
    # (roll, pitch, yaw) -> (x, y, z, w), same as tf.transformations 'sxyz'
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


# The following status classes are inlined from ROS 1 jsk_joy
# (not available in ROS 2): trackpoint_status.py, nanokontrol_status.py
# and nanopad_status.py. Only the attributes used below are provided.
class TrackpointStatus(object):
    def __init__(self, msg):
        buttons = list(msg.buttons) + [0] * 3
        self.left = buttons[0] == 1
        self.middle = buttons[1] == 1
        self.right = buttons[2] == 1
        axes = list(msg.axes) + [0.0] * 2
        self.x = axes[0]
        self.y = axes[1]


class NanoKONTROL2Status(object):
    def __init__(self, msg):
        self.msg = msg


class NanoPAD2Status(object):
    # buttons[0:8] are the upper pads U1-U8,
    # buttons[8:16] are the lower pads L1-L8
    def __init__(self, msg):
        buttons = list(msg.buttons) + [0] * 16
        for i in range(8):
            setattr(self, 'buttonU%d' % (i + 1), buttons[i] == 1)
            setattr(self, 'buttonL%d' % (i + 1), buttons[8 + i] == 1)


class TrackpointController(Node):

    def __init__(self):
        super().__init__('trackpoint_controller')
        self.pre_pose = None
        self.nanokontrol_st = None
        self.nanopad_st = None
        self.camera_pub = self.create_publisher(
            CameraPlacement, '/rviz/camera_placement', 1)
        self.interactive_pub = self.create_publisher(
            PoseStamped, INTERACTIVE_MARKER_TOPIC + '/move_marker', 1)
        self.trackpoint_sub = self.create_subscription(
            Joy, '/trackpoint/joy', self.trackpoint_joyCB, 1)
        self.nanokontrol_sub = self.create_subscription(
            Joy, '/nanokontrol/joy', self.nanokontrol_joyCB, 1)
        self.nanopad_sub = self.create_subscription(
            Joy, '/nanopad/joy', self.nanopad_joyCB, 1)

    def nanokontrol_joyCB(self, msg):
        self.nanokontrol_st = NanoKONTROL2Status(msg)

    def nanopad_joyCB(self, msg):
        self.nanopad_st = NanoPAD2Status(msg)

    def trackpoint_joyCB(self, msg):
        if not self.pre_pose:
            self.pre_pose = PoseStamped()
        status = TrackpointStatus(msg)
        nanopad_st = self.nanopad_st

        # move interactive marker
        new_pose = PoseStamped()
        new_pose.header.frame_id = 'map'
        # move in local
        local_xy_move = numpy.array((0.0,
                                     0.0,
                                     0.0,
                                     1.0))
        z_move = 0.0
        yaw_move = 0.0
        if nanopad_st and nanopad_st.buttonL1:
            scale_xy = 1500.0
            scale_z = 1500.0
            scale_yaw = 1000.0
        else:
            scale_xy = 500.0
            scale_z = 500.0
            scale_yaw = 300.0

        if not (status.left or status.right or status.middle):
            if nanopad_st and nanopad_st.buttonU1:
                z_move = status.y / scale_z
            elif nanopad_st and nanopad_st.buttonU2:
                yaw_move = status.y / scale_yaw
            else:
                local_xy_move = numpy.array((- status.x / scale_xy,
                                             - status.y / scale_xy,
                                             0.0,
                                             1.0))
        q = numpy.array((self.pre_pose.pose.orientation.x,
                         self.pre_pose.pose.orientation.y,
                         self.pre_pose.pose.orientation.z,
                         self.pre_pose.pose.orientation.w))
        xy_move = local_xy_move

        new_pose.pose.position.x = self.pre_pose.pose.position.x + xy_move[0]
        new_pose.pose.position.y = self.pre_pose.pose.position.y + xy_move[1]
        new_pose.pose.position.z = self.pre_pose.pose.position.z + z_move
        (roll, pitch, yaw) = euler_from_quaternion(q)
        yaw = yaw + yaw_move

        new_q = quaternion_from_euler(roll, pitch, yaw)
        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]
        self.interactive_pub.publish(new_pose)
        self.pre_pose = new_pose


def main():
    rclpy.init()
    node = TrackpointController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
