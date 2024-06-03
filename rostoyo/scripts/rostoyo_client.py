# SPDX-FileCopyrightText: 2024 Oliver Mümken <o.muemken@gmail.com>
# SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
#
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node
from dial_gauge_msgs.srv import DialGaugeRequest


class GaugeClientAsync(Node):

    def __init__(self):
        super().__init__('gauge_client')
        self.cli = self.create_client(DialGaugeRequest, 'read_gauge')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = DialGaugeRequest.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    gauge_client = GaugeClientAsync()
    response = gauge_client.send_request()
    print('Dial gauge value is: %f' % (response.value))

    gauge_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
