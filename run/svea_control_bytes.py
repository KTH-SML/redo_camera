# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import time
# import struct
# import math
# from typing import Optional

# import fleetmqsdk

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# from svea_core.interfaces import ActuationInterface

# TOPIC_FLEETMQ = "control"
# TOPIC_CMDVEL = "/cmd_vel"

# # ---- RCVE control packet format ----
# EXPECTED_LEN = 24          # <Ifffii
# STRUCT_FMT = "<Ifffii"     # uint32, float, float, float, int32, int32

# # ---- Mapping parameters (tune these) ----
# MAX_LINEAR_X = 0.4         # m/s at throttle=1.0
# MAX_ANGULAR_Z = 1.0        # rad/s at full steering
# BRAKE_THRESHOLD = 0.10     # brake>threshold => linear.x = 0
# CMD_TIMEOUT_SEC = 0.30     # if no RX for this long => publish stop
# DIRECTION_REV_THRESHOLD = 0.95 # if throttle>rev_threshold and steering near zero => consider it a reverse command (for direction-aware control)

# STEER_RATE = 0.5             # max change in steering per second (for smoothing)


# def _clamp(x: float, lo: float, hi: float) -> float:
#     return lo if x < lo else hi if x > hi else x


# class FleetMQSVEAControl(Node):
#     def __init__(self):
#         super().__init__("fleetmq_svea_control")

#         self._fm = fleetmqsdk.FleetMQ()
#         self._cfg, self._addr = self._fm.getConfig(True)
#         self.declare_parameter("rate_control_hz", 20.0)
#         self.declare_parameter("rate_recv_hz", 20.0)
#         self.rate_control_hz = float(self.get_parameter("rate_control_hz").value)
#         self.rate_recv_hz = float(self.get_parameter("rate_recv_hz").value)
#         self.get_logger().info("Got FleetMQ config. Starting receive loop...")

#         self._last_rx_wall = 0.0
#         self._last_str = 0.0
#         self._last_vel = 0.0
#         self._rx_ok = 0
#         self._rx_bad = 0
#         self._last_pkg: Optional[int] = None

#         self.actuation = ActuationInterface()

#         period_recv = 1.0 / max(self.rate_recv_hz, 1e-6)
#         self._sub_timer = self.create_timer(period_recv, self._tick)

#     def _publish_stop(self):
#         pass

#     def _tick(self):
#         now = time.time()

#         data = self._fm.receiveBytes(TOPIC_FLEETMQ)
#         if data:
#             # length guard (helps detect JSON/mismatched topic)
#             if len(data) != EXPECTED_LEN:
#                 self._rx_bad += 1
#                 if self._rx_bad % 50 == 1:
#                     self.get_logger().warn(
#                         f"RX bad len={len(data)} (expected {EXPECTED_LEN}); head={data[:32]!r}"
#                     )
#                 return

#             try:
#                 pkgNr, refStr, refThr, refBrk, refDrc, model1_i = struct.unpack(STRUCT_FMT, data)
#             except struct.error as e:
#                 self._rx_bad += 1
#                 self.get_logger().warn(f"RX unpack failed ({self._rx_bad}): {e}; raw={data!r}")
#                 return

#             # pkg continuity (optional)
#             if self._last_pkg is not None:
#                 delta = (pkgNr - self._last_pkg) & 0xFFFFFFFF
#                 if delta != 1 and self._rx_ok % 50 == 0:
#                     self.get_logger().warn(f"pkg jump: prev={self._last_pkg} now={pkgNr}")
#             self._last_pkg = pkgNr

#             # sanitize
#             if refStr is not None:
#                 refStr = refStr * STEER_RATE
#                 steering = _clamp(refStr, -1.0, 1.0)  * -1.0  # invert: G29 steering +1 is left, but ROS cmd_vel angular.z + is right
#             if refThr is not None:
#                 throttle = 1.0 - _clamp(refThr, 0.0, 1.0) # invert: throttle=1.0 means no throttle, throttle=0.0 means full throttle
#             if refBrk is not None:
#                 brake = 1.0 - _clamp(refBrk, 0.0, 1.0) # invert: brake=1.0 means no brake, brake=0.0 means full brake
#             if refDrc is not None:
#                 direction = _clamp(refDrc, 0.0, 1.0) 
#             if brake is not None and brake > BRAKE_THRESHOLD:
#                 throttle = 0.0
#             # print(f"refStr={refStr:.3f}, refThr={refThr:.3f}, refBrk={refBrk:.3f}, refDrc={refDrc:.3f}")
            

#             # cmd = Twist()
#             # cmd.linear.x = self._last_cmd.linear.x
#             # cmd.angular.z = self._last_cmd.angular.z

#             self.velocity = self._last_vel
#             self.steering = self._last_str

#             if throttle is not None:
#                 # cmd.linear.x = throttle * MAX_LINEAR_X
#                 self.velocity = 1.0 * float(throttle * MAX_LINEAR_X)
#             if steering is not None:
#                 # cmd.angular.z = steering * MAX_ANGULAR_Z
#                 self.steering = (math.pi/2.0) * float(steering * MAX_ANGULAR_Z)
            
#             reverse = (direction is not None and direction < DIRECTION_REV_THRESHOLD)
#             if reverse:
#                 # cmd.linear.x = -abs(cmd.linear.x)
#                 self.velocity = self.velocity * -1.0

#             # self._last_cmd = cmd
#             self._last_rx_wall = now
#             self._rx_ok += 1

#             if self._rx_ok % 10 == 0: #tmp
#                 self.get_logger().info(
#                     f"rx_ok={self._rx_ok} refStr={refStr:.3f}, refThr={refThr:.3f}, refBrk={refBrk:.3f}, refDrc={refDrc:.3f}"
#                 )
#                 self.get_logger().info(f"control_loop: steering={self.steering:.3f}, velocity={self.velocity:.3f}")

            
#             self.actuation.send_control(self.steering, self.velocity)
#             self._last_str = self.steering
#             self._last_vel = self.velocity
#             return

#         # timeout -> stop
#         if self._last_rx_wall > 0.0 and (now - self._last_rx_wall) > CMD_TIMEOUT_SEC:
#             self._publish_stop()
#             self._last_rx_wall = now
    



# def main():
#     rclpy.init()
#     node = FleetMQSVEAControl()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
