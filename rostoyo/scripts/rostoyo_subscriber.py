# SPDX-FileCopyrightText: 2024 Oliver Mümken <o.muemken@gmail.com>
# SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
#
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node

from dial_gauge_msgs.msg import DialGauge


class RostoyoSubscriber(Node):

    def __init__(self):
        super().__init__('rostoyo_subscriber')
        self.subscription = self.create_subscription(
            DialGauge,
            'gauge',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Dial gauge value is: "%f"' % msg.value)


def main(args=None):
    print('Starting publisher')
    rclpy.init(args=args)

    rostoyo_subscriber = RostoyoSubscriber()

    rclpy.spin(rostoyo_subscriber)

    rostoyo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
