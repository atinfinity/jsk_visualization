#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty


class SampleServiceRadioButtons(Node):
    def __init__(self):
        super(SampleServiceRadioButtons, self).__init__(
            'sample_service_radio_buttons')
        self._services = [
            self.create_service(Empty, 'dummy/buttonA', self._empty_cb),
            self.create_service(Empty, 'dummy/buttonB', self._empty_cb),
            self.create_service(Empty, 'dummy/buttonC', self._empty_cb),
            self.create_service(Empty, 'dummy/buttonD', self._empty_cb),
            self.create_service(Empty, 'dummy/buttonE', self._empty_cb),
            self.create_service(Empty, 'dummy/buttonF', self._empty_cb),
        ]
        self._name = self.get_name()

    def _empty_cb(self, req, res):
        self.get_logger().info(
            '{} | Empty service called'.format(self._name))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SampleServiceRadioButtons()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
