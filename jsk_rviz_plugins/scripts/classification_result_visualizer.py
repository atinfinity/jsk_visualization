#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import message_filters as MF
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import PeoplePoseArray
from posedetection_msgs.msg import ObjectDetection
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def is_valid_pose(pose):
    return pose.position.x == 0.0 and\
           pose.position.y == 0.0 and\
           pose.position.z == 0.0


class ClassificationResultVisualizerConfig(object):
    """Simple namespace object which caches the current parameter values.

    It mimics the config object of dynamic_reconfigure so that
    `config.text_size` style access keeps working.
    """
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class ClassificationResultVisualizer(Node):
    # (name, default, min, max) taken from cfg/ClassificationResultVisualizer.cfg
    _DOUBLE_PARAMETERS = [
        ('text_color_red', 0.0, 0.0, 1.0),
        ('text_color_green', 0.0, 0.0, 1.0),
        ('text_color_blue', 1.0, 0.0, 1.0),
        ('text_color_alpha', 1.0, 0.0, 1.0),
        ('text_offset_x', 0.0, -0.5, 0.5),
        ('text_offset_y', 0.0, -0.5, 0.5),
        ('text_offset_z', 0.07, -0.5, 0.5),
        ('text_size', 0.05, 0.001, 0.5),
        ('marker_lifetime', 5.0, 0.1, 300.0),
    ]

    def __init__(self):
        super(ClassificationResultVisualizer, self).__init__(
            'classification_result_visualizer')
        values = {}
        for name, default, min_value, max_value in self._DOUBLE_PARAMETERS:
            descriptor = ParameterDescriptor(
                floating_point_range=[FloatingPointRange(
                    from_value=min_value, to_value=max_value, step=0.0)])
            values[name] = self.declare_parameter(
                name, default, descriptor).value
        values['show_proba'] = self.declare_parameter('show_proba', True).value
        self.config = ClassificationResultVisualizerConfig(**values)
        self.config_callback()
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.pub_marker = self.create_publisher(MarkerArray, '~/output', 10)
        self.subscribe()

    def subscribe(self):
        approximate_sync = self.declare_parameter(
            'approximate_sync', False).value
        queue_size = self.declare_parameter('queue_size', 100).value
        slop = self.declare_parameter('slop', 0.1).value

        sub_cls = MF.Subscriber(
            self, ClassificationResult, '~/input/classes', qos_profile=1)
        sub_box = MF.Subscriber(
            self, BoundingBoxArray, '~/input/boxes', qos_profile=1)
        sub_pose = MF.Subscriber(
            self, PoseArray, '~/input/poses', qos_profile=1)
        sub_people = MF.Subscriber(
            self, PeoplePoseArray, '~/input/people', qos_profile=1)
        sub_od = MF.Subscriber(
            self, ObjectDetection, '~/input/ObjectDetection', qos_profile=1)

        if approximate_sync:
            sync_box = MF.ApproximateTimeSynchronizer(
                [sub_box, sub_cls], queue_size=queue_size, slop=slop)
            sync_pose = MF.ApproximateTimeSynchronizer(
                [sub_pose, sub_cls], queue_size=queue_size, slop=slop)
            sync_people = MF.ApproximateTimeSynchronizer(
                [sub_people, sub_cls], queue_size=queue_size, slop=slop)
            sync_od = MF.ApproximateTimeSynchronizer(
                [sub_od, sub_cls], queue_size=queue_size, slop=slop)
        else:
            sync_box = MF.TimeSynchronizer(
                [sub_box, sub_cls], queue_size=queue_size)
            sync_pose = MF.TimeSynchronizer(
                [sub_pose, sub_cls], queue_size=queue_size)
            sync_people = MF.TimeSynchronizer(
                [sub_people, sub_cls], queue_size=queue_size)
            sync_od = MF.TimeSynchronizer(
                [sub_od, sub_cls], queue_size=queue_size)

        sync_box.registerCallback(self.box_msg_callback)
        sync_pose.registerCallback(self.pose_msg_callback)
        sync_people.registerCallback(self.people_msg_callback)
        sync_od.registerCallback(self.od_msg_callback)

        self.subscribers = [sub_cls, sub_box, sub_pose, sub_people, sub_od]

    def config_callback(self):
        config = self.config
        self.text_color = {'r': config.text_color_red,
                           'g': config.text_color_green,
                           'b': config.text_color_blue,
                           'a': config.text_color_alpha}
        self.text_offset = [config.text_offset_x,
                            config.text_offset_y,
                            config.text_offset_z]
        self.text_size = config.text_size
        self.marker_lifetime = config.marker_lifetime
        self.show_proba = config.show_proba

    def parameter_callback(self, params):
        for param in params:
            if hasattr(self.config, param.name):
                setattr(self.config, param.name, param.value)
        self.config_callback()
        return SetParametersResult(successful=True)

    def pose_msg_callback(self, pose, classes):
        bboxes = BoundingBoxArray(header=pose.header)
        for p in pose.poses:
            b = BoundingBox(header=pose.header)
            b.pose = p
            bboxes.boxes.append(b)
        self.box_msg_callback(bboxes, classes)

    def people_msg_callback(self, people, classes):
        bboxes = BoundingBoxArray(header=people.header)
        for p in people.poses:
            b = BoundingBox()
            for i, n in enumerate(p.limb_names):
                if n in ["Neck", "Nose", "REye", "LEye", "REar", "LEar"]:
                    b.header = people.header
                    b.pose = p.poses[i]
                    break
            if not b.header.frame_id:
                b.header = people.header
                b.pose = b.poses[0]
        self.box_msg_callback(bboxes, classes)

    def od_msg_callback(self, od, classes):
        bboxes = BoundingBoxArray(header=od.header)
        for obj in od.objects:
            b = BoundingBox()
            b.pose = obj.pose
        self.box_msg_callback(bboxes, classes)

    def box_msg_callback(self, bboxes, classes):
        msg = MarkerArray()
        show_proba = self.show_proba and len(classes.label_names) == len(classes.label_proba)
        if show_proba:
            cls_iter = list(zip(classes.label_names, classes.label_proba))
        else:
            cls_iter = classes.label_names
        for i, data in enumerate(zip(bboxes.boxes, cls_iter)):
            bbox, cls = data
            if show_proba:
                text = "%s (%.3f)" % cls
            else:
                text = cls

            if is_valid_pose(bbox.pose):
                continue

            m = Marker(type=Marker.TEXT_VIEW_FACING,
                       action=Marker.MODIFY,
                       header=bbox.header,
                       id=i,
                       pose=bbox.pose,
                       color=ColorRGBA(**self.text_color),
                       text=text,
                       ns=classes.classifier,
                       lifetime=Duration(
                           seconds=self.marker_lifetime).to_msg())
            m.scale.z = self.text_size
            m.pose.position.x += self.text_offset[0]
            m.pose.position.y += self.text_offset[1]
            m.pose.position.z += self.text_offset[2]
            msg.markers.append(m)

        self.pub_marker.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    viz = ClassificationResultVisualizer()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
