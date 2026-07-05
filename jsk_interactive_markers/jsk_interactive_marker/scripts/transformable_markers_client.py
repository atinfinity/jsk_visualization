#!/usr/bin/env python3

import os.path as osp
import sys

import yaml

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from jsk_rviz_plugins_msgs.srv import RequestMarkerOperate
from jsk_rviz_plugins_msgs.msg import TransformableMarkerOperate
from jsk_interactive_marker_msgs.msg import MarkerDimensions
from jsk_interactive_marker_msgs.msg import PoseStampedWithName
from jsk_interactive_marker_msgs.srv import GetTransformableMarkerFocus
from jsk_interactive_marker_msgs.srv import SetTransformableMarkerPose
from jsk_interactive_marker_msgs.srv import SetTransformableMarkerColor
from jsk_interactive_marker_msgs.srv import SetMarkerDimensions
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def labelcolormap(N=256):
    # copied from jsk_recognition_utils.color.labelcolormap
    # (jsk_recognition_utils is not released for ROS 2)
    def bitget(byteval, idx):
        return (byteval & (1 << idx)) != 0

    cmap = []
    for i in range(N):
        id = i
        r, g, b = 0, 0, 0
        for j in range(8):
            r |= bitget(id, 0) << (7 - j)
            g |= bitget(id, 1) << (7 - j)
            b |= bitget(id, 2) << (7 - j)
            id >>= 3
        cmap.append((r / 255.0, g / 255.0, b / 255.0))
    return cmap


class TransformableMarkersClient(Node):

    def __init__(self):
        super().__init__('transformable_markers_client')
        # ROS 2 does not support remapping of unresolved private names,
        # so the server name is passed as the 'server' parameter instead
        # of remapping '~server'.
        self.server = self.declare_parameter(
            'server', 'transformable_interactive_server').value

        self.config_file = self.declare_parameter('config_file', '').value
        if not osp.exists(self.config_file):
            self.get_logger().fatal("config_file '{}' does not exist"
                                    .format(self.config_file))
            sys.exit(1)
        self.config = yaml.safe_load(open(self.config_file))

        self.req_marker = self.create_client(
            RequestMarkerOperate,
            osp.join(self.server, 'request_marker_operate'))
        self.req_color = self.create_client(
            SetTransformableMarkerColor, osp.join(self.server, 'set_color'))
        self.req_pose = self.create_client(
            SetTransformableMarkerPose, osp.join(self.server, 'set_pose'))
        self.req_dim = self.create_client(
            SetMarkerDimensions, osp.join(self.server, 'set_dimensions'))
        self.req_focus = self.create_client(
            GetTransformableMarkerFocus, osp.join(self.server, 'get_focus'))
        for client in [self.req_marker, self.req_color, self.req_pose,
                       self.req_dim, self.req_focus]:
            while not client.wait_for_service(timeout_sec=5.0):
                if not rclpy.ok():
                    sys.exit(1)
                self.get_logger().warn(
                    "waiting for service '{}'".format(client.srv_name))

        self.object_poses = {}
        self.object_dimensions = {}

        # TODO: support other than box: ex. torus, cylinder, mesh

        # insert box
        boxes = self.config['boxes']
        n_boxes = len(boxes)
        cmap = labelcolormap(n_boxes)
        for i in range(n_boxes):
            box = boxes[i]
            name = box['name']
            frame_id = box.get('frame_id', 'map')
            self.insert_marker(name, frame_id,
                               type=TransformableMarkerOperate.SHAPE_BOX)
            self.set_color(name, (cmap[i][0], cmap[i][1], cmap[i][2], 0.5))
            dim = box.get('dimensions', [1, 1, 1])
            self.set_dimensions(name, dim)
            pos = box.get('position', [0, 0, 0])
            ori = box.get('orientation', [0, 0, 0, 1])
            self.set_pose(box['name'], box['frame_id'], pos, ori)
            # rclpy requires Point here (ROS 1 code used Vector3)
            self.object_poses[box['name']] = Pose(
                position=Point(x=float(pos[0]), y=float(pos[1]),
                               z=float(pos[2])),
                orientation=Quaternion(x=float(ori[0]), y=float(ori[1]),
                                       z=float(ori[2]), w=float(ori[3])),
            )
            self.object_dimensions[box['name']] = Vector3(
                x=float(dim[0]), y=float(dim[1]), z=float(dim[2]))
            self.get_logger().info(
                "Inserted transformable box '{}'.".format(name))

        self.sub_dimensions = self.create_subscription(
            MarkerDimensions,
            osp.join(self.server, 'marker_dimensions'),
            self._dimensions_change_callback, 1)
        self.sub_pose = self.create_subscription(
            PoseStampedWithName,
            osp.join(self.server, 'pose_with_name'),
            self._pose_change_callback, 1)

        do_auto_save = self.declare_parameter('config_auto_save', True).value
        if do_auto_save:
            self.timer_save = self.create_timer(1.0, self._save_callback)

        self.pub_bboxes = self.create_publisher(
            BoundingBoxArray, '~/output/boxes', 1)
        self.timer_pub_bboxes = self.create_timer(
            0.1, self._pub_bboxes_callback)

    def _call(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            future.cancel()
            self.get_logger().warn(
                "service call to '{}' timed out".format(client.srv_name))
            return None
        return future.result()

    def insert_marker(self, name, frame_id, type):
        req = RequestMarkerOperate.Request()
        req.operate = TransformableMarkerOperate(
            name=name,
            type=type,
            action=TransformableMarkerOperate.ACTION_INSERT,
            frame_id=frame_id,
            description=name,
        )
        self._call(self.req_marker, req)

    def set_color(self, name, rgba):
        req = SetTransformableMarkerColor.Request()
        req.target_name = name
        req.color.r = float(rgba[0])
        req.color.g = float(rgba[1])
        req.color.b = float(rgba[2])
        req.color.a = float(rgba[3])
        self._call(self.req_color, req)

    def set_dimensions(self, name, dimensions):
        req = SetMarkerDimensions.Request()
        req.target_name = name
        req.dimensions.x = float(dimensions[0])
        req.dimensions.y = float(dimensions[1])
        req.dimensions.z = float(dimensions[2])
        self._call(self.req_dim, req)

    def set_pose(self, name, frame_id, position, orientation):
        req = SetTransformableMarkerPose.Request()
        req.target_name = name
        req.pose_stamped.header.frame_id = frame_id
        req.pose_stamped.pose.position.x = float(position[0])
        req.pose_stamped.pose.position.y = float(position[1])
        req.pose_stamped.pose.position.z = float(position[2])
        req.pose_stamped.pose.orientation.x = float(orientation[0])
        req.pose_stamped.pose.orientation.y = float(orientation[1])
        req.pose_stamped.pose.orientation.z = float(orientation[2])
        req.pose_stamped.pose.orientation.w = float(orientation[3])
        self._call(self.req_pose, req)

    def _dimensions_change_callback(self, msg):
        # request the focused marker asynchronously; a synchronous call
        # would deadlock inside the subscription callback
        req = GetTransformableMarkerFocus.Request()
        future = self.req_focus.call_async(req)

        def done_callback(future):
            res = future.result()
            if res is not None:
                self.object_dimensions[res.target_name] = msg

        future.add_done_callback(done_callback)

    def _pose_change_callback(self, msg):
        self.object_poses[msg.name] = msg.pose.pose

    def _pub_bboxes_callback(self):
        if not rclpy.ok():
            return
        bbox_array_msg = BoundingBoxArray()
        bbox_array_msg.header.frame_id = self.config['boxes'][0]['frame_id']
        bbox_array_msg.header.stamp = self.get_clock().now().to_msg()
        for box in self.config['boxes']:
            bbox_msg = BoundingBox()
            pose = self.object_poses[box['name']]
            bbox_msg.header.frame_id = bbox_array_msg.header.frame_id
            bbox_msg.header.stamp = bbox_array_msg.header.stamp
            bbox_msg.pose = pose
            dimensions = self.object_dimensions[box['name']]
            bbox_msg.dimensions.x = dimensions.x
            bbox_msg.dimensions.y = dimensions.y
            bbox_msg.dimensions.z = dimensions.z
            bbox_array_msg.boxes.append(bbox_msg)
        self.pub_bboxes.publish(bbox_array_msg)

    def _save_callback(self):
        for i, box in enumerate(self.config['boxes']):
            box_cfg = self.config['boxes'][i]
            box_cfg['name'] = box['name']
            pose = self.object_poses[box['name']]
            dimensions = self.object_dimensions[box['name']]
            # TODO: support transformed bbox dimensions like pose
            box_cfg.update({
                'frame_id': box['frame_id'],
                'dimensions': [
                    dimensions.x,
                    dimensions.y,
                    dimensions.z,
                ],
                'position': [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                'orientation': [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            })
        yaml.dump(self.config, open(self.config_file, 'w'))


if __name__ == '__main__':
    rclpy.init()
    node = TransformableMarkersClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
