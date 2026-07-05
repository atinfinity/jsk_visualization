#!/usr/bin/env python3

# std_msgs/Time does not exist in ROS 2; builtin_interfaces/Time is used
# instead (the DRCEnvironmentViewer plugin expects it, too).
from builtin_interfaces.msg import Time
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class SampleDRCMiniMaxwell(Node):
    def __init__(self):
        super(SampleDRCMiniMaxwell, self).__init__('sample_drc_mini_maxwell')
        self.pub_is_disabled = self.create_publisher(
            Bool, '/drc_2015_environment/is_disabled', 1)
        self.pub_is_blackout = self.create_publisher(
            Bool, '/drc_2015_environment/is_blackout', 1)
        self.pub_next_whiteout_time = self.create_publisher(
            Time, '/drc_2015_environment/next_whiteout_time', 1)
        self.pub_is_disabled.publish(Bool(data=False))
        self.pub_is_blackout.publish(Bool(data=False))

        # The ROS 1 sample ran a 4 sec cycle:
        #   t=0: is_blackout=False
        #   t=1: is_disabled=True
        #   t=2: is_disabled=False, is_blackout=True,
        #        next_whiteout_time=now+2sec
        # This is reproduced with a 1 sec timer and a step counter.
        self.step = 0
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        if self.step == 0:
            self.pub_is_blackout.publish(Bool(data=False))
        elif self.step == 1:
            self.pub_is_disabled.publish(Bool(data=True))
        elif self.step == 2:
            self.pub_is_disabled.publish(Bool(data=False))
            self.pub_is_blackout.publish(Bool(data=True))
            next_whiteout_time = (
                self.get_clock().now() + Duration(seconds=2.0))
            self.pub_next_whiteout_time.publish(next_whiteout_time.to_msg())
        self.step = (self.step + 1) % 4


def main(args=None):
    rclpy.init(args=args)
    node = SampleDRCMiniMaxwell()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
