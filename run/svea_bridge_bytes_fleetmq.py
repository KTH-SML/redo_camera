#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import struct
import threading
from typing import Optional
from collections import deque

import fleetmqsdk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

TOPIC_FLEETMQ = "control"
TOPIC_CMDVEL = "/cmd_vel"

# ---- RCVE control packet format ----
EXPECTED_LEN = 24          # <Ifffii
STRUCT_FMT = "<Ifffii"     # uint32, float, float, float, int32, int32

# ---- Mapping parameters (tune these) ----
MAX_LINEAR_X = 0.5         # m/s at throttle=1.0
MAX_ANGULAR_Z = 1.0        # rad/s at full steering
BRAKE_THRESHOLD = 0.10     # brake>threshold => linear.x = 0
CMD_TIMEOUT_SEC = 1.0     # if no RX for this long => publish stop
DIRECTION_REV_THRESHOLD = 0.05 # if throttle>rev_threshold and steering near zero => consider it a reverse command (for direction-aware control)

STEER_RATE = 0.5             # max change in steering per second (for smoothing)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class FleetMQCmdVelBridge(Node):
    def __init__(self):
        super().__init__("fleetmq_cmdvel_bridge")

        self.pub_cmd = self.create_publisher(Twist, TOPIC_CMDVEL, 10)

        # FleetMQ init
        self._fm = fleetmqsdk.FleetMQ()
        self._cfg, self._addr = self._fm.getConfig(True)
        self.get_logger().info("Got FleetMQ config. Starting receive loop...")

        # State
        self._lock = threading.Lock()
        self._latest_cmd = Twist()
        self._last_rx_wall = 0.0

        # Keep only latest packet (avoids backlog burst)
        self._pktq = deque(maxlen=1)

        self._rx_ok = 0
        self._rx_bad = 0
        self._last_pkg: Optional[int] = None

        self._stop_evt = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Publish timer (fixed-rate output)
        self._timer = self.create_timer(0.02, self._publish_tick)  # 50 Hz

    def destroy_node(self):
        self._stop_evt.set()
        try:
            self._rx_thread.join(timeout=0.5)
        except Exception:
            pass
        super().destroy_node()

    def _publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)

    def _rx_loop(self):
        while not self._stop_evt.is_set():
            data = self._fm.receiveBytes(TOPIC_FLEETMQ)

            if not data:
                continue

            if len(data) != EXPECTED_LEN:
                self._rx_bad += 1
                if self._rx_bad % 50 == 1:
                    self.get_logger().warn(
                        f"RX bad len={len(data)} (expected {EXPECTED_LEN}); head={data[:32]!r}"
                    )
                continue

            # latest-only: キューに「最後の1個」だけ残す
            self._pktq.append((time.time(), data))

    def _decode_to_twist(self, data: bytes) -> Optional[Twist]:
        try:
            pkgNr, refStr, refThr, refBrk, refDrc, _model1_i = struct.unpack(STRUCT_FMT, data)
        except struct.error as e:
            self._rx_bad += 1
            self.get_logger().warn(f"RX unpack failed ({self._rx_bad}): {e}")
            return None

        # pkg continuity (optional)
        if self._last_pkg is not None:
            delta = (pkgNr - self._last_pkg) & 0xFFFFFFFF
            if delta != 1 and self._rx_ok % 50 == 0:
                self.get_logger().warn(f"pkg jump: prev={self._last_pkg} now={pkgNr}")
        self._last_pkg = pkgNr

        # sanitize/mapping
        refStr = _clamp(refStr * STEER_RATE, -1.0, 1.0) * -1.0
        throttle = 1.0 - _clamp(refThr, 0.0, 1.0)
        brake = 1.0 - _clamp(refBrk, 0.0, 1.0)
        direction = _clamp(refDrc, 0.0, 1.0)

        if brake > BRAKE_THRESHOLD:
            throttle = 0.0

        cmd = Twist()
        cmd.linear.x = throttle * MAX_LINEAR_X
        cmd.angular.z = refStr * MAX_ANGULAR_Z

        reverse = direction < DIRECTION_REV_THRESHOLD
        if reverse:
            cmd.linear.x = -abs(cmd.linear.x)

        return cmd

    def _publish_tick(self):
        # キューに溜まった古いのは捨てて「最後の1個」だけ反映
        pkt = None
        while True:
            try:
                pkt = self._pktq.pop()
                # popできたら、さらに古いのは不要なのでループで潰す…と思うかもだが
                # deque(maxlen=1)なので実際は常に最後の1個しかない
                break
            except IndexError:
                break

        now = time.time()

        if pkt is not None:
            rx_time, data = pkt
            cmd = self._decode_to_twist(data)
            if cmd is not None:
                with self._lock:
                    self._latest_cmd = cmd
                    self._last_rx_wall = rx_time
                self.pub_cmd.publish(cmd)

                self._rx_ok += 1
                if self._rx_ok % 10 == 0:
                    self.get_logger().info(
                        f"rx_ok={self._rx_ok} cmd_vel: vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
                    )
                return

        # no new packet -> timeout safety
        with self._lock:
            last_rx = self._last_rx_wall

        if last_rx > 0.0 and (now - last_rx) > CMD_TIMEOUT_SEC:
            self._publish_stop()
            # 連打防止: last_rx を更新して「次の tick まで待つ」
            with self._lock:
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
