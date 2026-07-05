#!/usr/bin/env python3

"""
Demo stand-in for the (unported) face_detector node.

The ROS 1 face_detector_sample launched the people-stack `face_detector`
node against a live RGBD camera. That node has no ROS 2 Jazzy release, so
this script publishes a synthetic people_msgs/PositionMeasurementArray on
the same topic the jsk_rviz_plugin/PeoplePositionMeasurementArray display
subscribes to, letting the visualization be demonstrated without a camera
or the detector. Replace it with a real ROS 2 face detector publishing
people_msgs/PositionMeasurementArray to get live results.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from tf2_ros import StaticTransformBroadcaster

FRAME_ID = 'camera_depth_frame'


class FaceDetectorSample(Node):

    def __init__(self):
        super().__init__('face_detector_sample')
        self.pub = self.create_publisher(
            PositionMeasurementArray,
            '/face_detector/people_tracker_measurements_array', 1)
        # Publish a static transform so the fixed frame exists in TF.
        self._static_tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = FRAME_ID
        tf.transform.rotation.w = 1.0
        self._static_tf.sendTransform(tf)

        self.count = 0
        self.timer = self.create_timer(0.2, self._timer_cb)

    def _make_person(self, name, x, y, z):
        p = PositionMeasurement()
        p.header.frame_id = FRAME_ID
        p.header.stamp = self.get_clock().now().to_msg()
        p.name = name
        p.object_id = name
        p.pos.x = float(x)
        p.pos.y = float(y)
        p.pos.z = float(z)
        p.reliability = 0.9
        p.covariance = [0.0] * 9
        # 'initialization' is a byte field -> a length-1 bytes value in rclpy
        p.initialization = bytes([0])
        return p

    def _timer_cb(self):
        t = self.count * 0.2
        msg = PositionMeasurementArray()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = self.get_clock().now().to_msg()
        # two people slowly moving so the visualization is clearly live
        msg.people.append(
            self._make_person('frontalface', 0.4 * math.cos(t), 0.3, 2.0))
        msg.people.append(
            self._make_person('frontalface', -0.4 * math.cos(t), -0.2, 2.5))
        self.pub.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorSample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
