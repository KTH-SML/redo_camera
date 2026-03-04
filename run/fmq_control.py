#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import pi
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from svea_core.interfaces import ActuationInterface


class FmqControlReceiver(Node):
    def __init__(self):
        super().__init__("fmq_control_receiver")

        self.declare_parameter("fmq_topic", "/fmq/remote_control")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("timeout_s", 0.5)

        self.fmq_topic = self.get_parameter("fmq_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        self.steering = 0.0
        self.velocity = 0.0
        self.last_rx_t = time.monotonic()

        self.actuation = ActuationInterface()

        self.sub = self.create_subscription(
            Twist,
            self.fmq_topic,
            self.fmq_cb,
            10,
        )

        period = 1.0 / max(self.rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(
            f"Started. topic='{self.fmq_topic}', rate={self.rate_hz:.1f}Hz, timeout={self.timeout_s:.2f}s"
        )

    def fmq_cb(self, msg: Twist):
        self.steering = (pi / 2.0) * float(msg.angular.z)
        self.velocity = 1.0 * float(msg.linear.x)
        self.last_rx_t = time.monotonic()

    def loop(self):
        steering = self.steering
        velocity = self.velocity
        self.actuation.send_control(steering, velocity)


def main():
    rclpy.init()
    node = FmqControlReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
