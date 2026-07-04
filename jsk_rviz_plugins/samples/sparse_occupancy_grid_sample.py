#!/usr/bin/env python3

import time

import rclpy
from jsk_recognition_msgs.msg import SimpleOccupancyGrid, SimpleOccupancyGridArray
from geometry_msgs.msg import Point

rclpy.init()
node = rclpy.create_node("test_occupancy_grid")

p = node.create_publisher(SimpleOccupancyGridArray, "/occupancy_grid", 1)


def cells(x_offset):
    ret = []
    for i in range(0, 20):
        for j in range(0, 20):
            ret.append(Point(x = 0.05 * i + x_offset, y = 0.05 * j, z = 0.0))
    return ret


while rclpy.ok():
    now = node.get_clock().now().to_msg()
    occupancy_grid_array = SimpleOccupancyGridArray()
    for i in range(10):
        occupancy_grid = SimpleOccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.header.stamp = now
        occupancy_grid.coefficients = [0.0, 0.0, 1.0, i * 0.2]
        occupancy_grid.resolution = 0.05      #5cm resolution
        occupancy_grid.cells = cells(i / 2.0)
        occupancy_grid_array.grids.append(occupancy_grid)
    occupancy_grid_array.header.stamp = now
    occupancy_grid_array.header.frame_id = "map"
    p.publish(occupancy_grid_array)
    time.sleep(1.0)
