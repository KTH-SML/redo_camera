#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from typing import Optional

import fleetmqsdk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


TOPIC_FLEETMQ = "control"
TOPIC_CMDVEL = "/cmd_vel"

# ---- Mapping parameters (tune these) ----
MAX_LINEAR_X = 1.0     # m/s at full throttle=1.0
MAX_ANGULAR_Z = 1.0    # rad/s at full steering=±1.0
BRAKE_THRESHOLD = 0.90 # brake>threshold => linear.x = 0
CMD_TIMEOUT_SEC = 0.30 # if no RX for this long => publish stop
DIRECTION_REV_THRESHOLD = 0.95 # if throttle>rev_threshold and steering near zero => consider it a reverse command (for direction-aware control)


def _to_float(x) -> Optional[float]:
    if x is None:
        return None
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class FleetMQCmdVelBridge(Node):
    def __init__(self):
        super().__init__("fleetmq_cmdvel_bridge")

        # QoS depth 10: control inputなので最新があればよい
        self.pub_cmd = self.create_publisher(Twist, TOPIC_CMDVEL, 10)

        # FleetMQ init
        self._fm = fleetmqsdk.FleetMQ()
        self._cfg, self._addr = self._fm.getConfig(True)
        self.get_logger().info("Got FleetMQ config. Starting receive loop...")

        # State
        self._last_rx_wall = 0.0
        self._last_cmd = Twist()
        self._rx_ok = 0
        self._rx_bad = 0

        # 100 Hz tick (receive + timeout handling)
        self._timer = self.create_timer(0.01, self._tick)

    def _publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)

    def _tick(self):
        now = time.time()

        # ---- RX (non-blocking) ----
        data = self._fm.receiveBytes(TOPIC_FLEETMQ)
        if data:
            try:
                msg = json.loads(data.decode("utf-8", errors="strict"))
            except Exception as e:
                self._rx_bad += 1
                self.get_logger().warn(f"RX parse failed ({self._rx_bad}): {e}; head={data[:80]!r}")
                return

            steering = _to_float(msg.get("steering"))   # -1..1
            throttle = _to_float(msg.get("throttle"))   # 0..1
            brake = _to_float(msg.get("brake"))         # 0..1
            direction = _to_float(msg.get("direction")) # -1..1, for direction-aware control

            # sanitize
            if steering is not None:
                steering = _clamp(steering, -1.0, 1.0) * -1.0  # invert: G29 steering +1 is left, but ROS cmd_vel angular.z + is right
            if throttle is not None:
                throttle = _clamp(throttle, 0.0, 1.0)
            if brake is not None:
                brake = _clamp(brake, 0.0, 1.0)
            if direction is not None:
                direction = _clamp(direction, 0.0, 1.0)
            

            # brake override (simple policy)
            if brake is not None and brake < BRAKE_THRESHOLD:
                throttle = 0.0

            # Build Twist (publish even if one field missing: keep last for missing)
            cmd = Twist()
            cmd.linear.x = self._last_cmd.linear.x
            cmd.angular.z = self._last_cmd.angular.z

            if throttle is not None:
                cmd.linear.x = throttle * MAX_LINEAR_X
            if steering is not None:
                cmd.angular.z = steering * MAX_ANGULAR_Z
            
            reverse = (direction is not None and direction < DIRECTION_REV_THRESHOLD)
            if reverse:
                cmd.linear.x = -abs(cmd.linear.x)

            self.pub_cmd.publish(cmd)
            self._last_cmd = cmd
            self._last_rx_wall = now
            self._rx_ok += 1

            if self._rx_ok % 100 == 0:
                self.get_logger().info(
                    f"rx_ok={self._rx_ok} cmd_vel: vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
                )

            return

        if self._last_rx_wall > 0.0 and (now - self._last_rx_wall) > CMD_TIMEOUT_SEC:
            self._publish_stop()
            self._last_rx_wall = now


def main():
    rclpy.init()
    node = FleetMQCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
