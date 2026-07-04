#!/usr/bin/env python3


import math
import time

import rclpy

from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

rclpy.init()
node = rclpy.create_node("bbox_sample")
pub = node.create_publisher(BoundingBoxArray, "bbox", 1)
counter = 0
while rclpy.ok():
  box_a = BoundingBox()
  box_b = BoundingBox()
  box_c = BoundingBox()
  box_a.label = 2
  box_b.label = 5
  box_b.label = 10
  box_arr = BoundingBoxArray()
  now = node.get_clock().now().to_msg()
  box_a.header.stamp = now
  box_b.header.stamp = now
  box_c.header.stamp = now
  box_arr.header.stamp = now
  box_a.header.frame_id = "map"
  box_b.header.frame_id = "map"
  box_c.header.frame_id = "map"
  box_arr.header.frame_id = "map"
  # quaternion about z axis
  theta = (counter % 100) * math.pi * 2 / 100.0
  box_a.pose.orientation.x = 0.0
  box_a.pose.orientation.y = 0.0
  box_a.pose.orientation.z = math.sin(theta / 2.0)
  box_a.pose.orientation.w = math.cos(theta / 2.0)
  box_b.pose.orientation.w = 1.0
  box_b.pose.position.y = 2.0
  box_c.pose.position.y = 1.0
  box_c.pose.position.z = -0.52
  box_b.dimensions.x = (counter % 10 + 1) * 0.1
  box_b.dimensions.y = ((counter + 1) % 10 + 1) * 0.1
  box_b.dimensions.z = ((counter + 2) % 10 + 1) * 0.1
  box_a.dimensions.x = 1.0
  box_a.dimensions.y = 1.0
  box_a.dimensions.z = 1.0
  box_c.dimensions.x = 3.0
  box_c.dimensions.y = 3.0
  box_c.dimensions.z = 0.02
  box_a.value = (counter % 100) / 100.0
  box_b.value = 1 - (counter % 100) / 100.0
  box_c.value = (counter % 300) / 100.0
  box_arr.boxes.append(box_a)
  box_arr.boxes.append(box_b)
  box_arr.boxes.append(box_c)
  pub.publish(box_arr)
  time.sleep(1.0 / 24)
  counter = counter + 1
