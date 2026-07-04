#!/usr/bin/env python3

import math
import time

import rclpy
from jsk_recognition_msgs.msg import TorusArray, Torus
from geometry_msgs.msg import Pose

rclpy.init()
node = rclpy.create_node("test_torus")
p = node.create_publisher(TorusArray, "test_torus", 1)
counter = 0
while rclpy.ok():
  torus_array = TorusArray()
  torus1 = Torus()
  torus1.header.frame_id = "base_link"
  torus1.large_radius = 4.0
  torus1.small_radius = 1.0
  p1 = Pose()
  p1.position.x = 2.0
  p1.position.z = 4.0
  p1.orientation.x = math.sqrt(0.5)
  p1.orientation.y = math.sqrt(0.5)
  torus1.pose = p1

  torus2 = Torus()
  torus2.header.frame_id = "base_link"
  torus2.large_radius = 5.0
  torus2.small_radius = 1.0
  p2 = Pose()
  p2.position.x = 3.0
  p2.position.y = -4.0
  p2.orientation.z = math.sqrt(0.5)
  p2.orientation.y = math.sqrt(0.5)
  torus2.pose = p2

  torus_array.header.frame_id = "base_link"

  torus_array.toruses.append(torus1)
  torus_array.toruses.append(torus2)

  p.publish(torus_array)
  time.sleep(1.0 / 5)
