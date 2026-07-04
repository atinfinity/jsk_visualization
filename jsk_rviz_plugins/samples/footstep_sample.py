#!/usr/bin/env python3

FRAME_ID = "map"

import time

from jsk_footstep_msgs.msg import Footstep, FootstepArray

import rclpy

def main(node):
    pub = node.create_publisher(FootstepArray, "/footsteps", 1)
    ysize = 0
    zpos = 0.0
    while rclpy.ok():
        msg = FootstepArray()
        now = node.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = now
        xpos = 0.0

        for i in range(20):
            footstep = Footstep()
            if i % 2 == 0:
                footstep.leg = Footstep.LEFT
                footstep.pose.position.y = 0.21
            else:
                footstep.leg = Footstep.RIGHT
                footstep.pose.position.y = -0.21
            footstep.pose.orientation.w = 1.0
            footstep.pose.position.x = xpos
            footstep.pose.position.z = zpos
            footstep.dimensions.x = 0.25
            footstep.dimensions.y = 0.15
            footstep.dimensions.z = 0.01
            footstep.footstep_group = i // 5
            msg.footsteps.append(footstep)
            xpos = xpos + 0.25
            zpos = zpos + 0.1
            ysize = ysize + 0.01
            if ysize > 0.15:
                ysize = 0.0
            if zpos > 0.5:
                zpos = 0.0
        pub.publish(msg)
        time.sleep(1.0 / 3)

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("footstep_sample")
    main(node)
