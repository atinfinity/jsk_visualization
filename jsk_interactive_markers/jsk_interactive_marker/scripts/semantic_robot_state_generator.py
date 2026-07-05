#!/usr/bin/env python3

# simple script to generate srdf parameter on the fly
# with only world virtual joint
#
# ROS 2 semantic change: ROS 1 set the global parameter
# /robot_description_semantic; ROS 2 has no global parameter server,
# so following the robot_state_publisher convention the SRDF is instead
# published as std_msgs/String on the "robot_description_semantic" topic
# with transient_local (latched) QoS, and the URDF is read from the
# "robot_description" topic with the same QoS.
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from xml.dom.minidom import parseString


class SemanticRobotStateGenerator(Node):

    def __init__(self):
        super().__init__("semantic_robot_state_generator")
        self.root_link = self.declare_parameter("root_link", "BODY").value
        self.global_frame = self.declare_parameter("global_frame", "odom").value
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            String, "robot_description_semantic", latched_qos)
        self.sub = self.create_subscription(
            String, "robot_description", self.robot_description_callback,
            latched_qos)

    def robot_description_callback(self, msg):
        robot_description_doc = parseString(msg.data)
        robot_name = robot_description_doc.getElementsByTagName(
            "robot")[0].getAttribute("name")

        srdf_str = """<?xml version="1.0" ?>
        <robot name="%s">
        <virtual_joint name="world_joint" type="floating" parent_frame="%s" child_link="%s" />
        <passive_joint name="world_joint" />
        </robot>
        """ % (robot_name, self.global_frame, self.root_link)
        self.pub.publish(String(data=srdf_str))


if __name__ == "__main__":
    rclpy.init()
    node = SemanticRobotStateGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
