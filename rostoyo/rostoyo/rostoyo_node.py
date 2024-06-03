# SPDX-FileCopyrightText: 2024 Oliver Mümken <o.muemken@gmail.com>
# SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
#
# SPDX-License-Identifier: MIT

import time
import usb as _usb

from dial_gauge_msgs.msg import DialGauge
from dial_gauge_msgs.srv import DialGaugeRequest
from rostoyo.pytuyo import Pytuyo

import rclpy
from rclpy.node import Node


class RostoyoNode(Node):

    def __init__(self):
        super().__init__('rostoyo')
        while True:
            self.d = _usb.core.find(idVendor=0x0fe7, idProduct=0x4001)
            if self.d is not None:
                break
            print("Could not find USB-ITN (idVendor=0x0fe7, idProduct=0x4001)")
            time.sleep(1)

        self.p = Pytuyo(self.d)

        self.p.device_info_cb = lambda v: print("Device Info: {}".format(v))
        self.p.status_cb = lambda v: print("Device Status: {}".format(v))

        self.srv = self.create_service(
            DialGaugeRequest, 'read_gauge', self.read_callback)
        self.publisher_ = self.create_publisher(
            DialGauge, 'gauge', 10)
        # add parameter for publishing frequency
        self.declare_parameter('frequency', 5)
        self.frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)

    def read_callback(self, request, response):
        self.p.request_read()
        while not self.p.check_resp():
            pass

        response.header.stamp = self.get_clock().now().to_msg()
        response.header.frame_id = "gauge"
        response.value = self.p._last_data
        self.get_logger().debug('The current gauge value is: %f' % (response.value))

        return response

    def timer_callback(self):
        # only publish if there are subscribers
        if self.publisher_.get_subscription_count() > 0:
            self.p.request_read()
            while not self.p.check_resp():
                pass

            msg = DialGauge()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gauge"
            msg.value = self.p._last_data
            self.publisher_.publish(msg)
            self.get_logger().debug('Publishing: "%f"' % msg.value)


def main(args=None):
    rclpy.init(args=args)
    node = RostoyoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
