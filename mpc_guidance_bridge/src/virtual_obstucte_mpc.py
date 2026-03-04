#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import time
import json
import socket
import csv
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node

from send_udp_function import MPCUdpSender

# =============================================================================
# UDP Sender (self-contained)
# =============================================================================

# class MPCUdpSender:
#     """
#     Minimal UDP sender.

#     Payload:
#       JSON string (utf-8) with keys:
#         - t: float (sec since node start)
#         - ok: bool
#         - steer_rate: float
#         - accel: float
#         - points: list of [x, y] length N+1
#     """
#     def __init__(self, host: str = "127.0.0.1", port: int = 50052, max_bytes: int = 65000):
#         self.addr = (host, port)
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.max_bytes = max_bytes

#     def send_points(self, pts_xy: np.ndarray, ok: bool, steer_rate: float, accel: float, t: float):
#         payload = {
#             "t": float(t),
#             "ok": bool(ok),
#             "steer_rate": float(steer_rate),
#             "accel": float(accel),
#             "points": pts_xy.tolist(),   # [[x,y], ...]
#         }
#         data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
#         # UDP size guard (MTU issues). For longer horizons you may need binary/fragmentation.
#         if len(data) > self.max_bytes:
#             # truncate points to fit (simple fallback)
#             # keep header info and first K points
#             # (You can replace this with a binary packing if needed.)
#             pts = payload["points"]
#             while pts and len(json.dumps({**payload, "points": pts}, separators=(",", ":")).encode("utf-8")) > self.max_bytes:
#                 pts = pts[:-1]
#             payload["points"] = pts
#             data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
#         self.sock.sendto(data, self.addr)


# =============================================================================
# MPC Config
# =============================================================================

@dataclass
class MPCConfig:
    # Horizon
    N: int = 20
    dt: float = 0.1

    # Delay compensation (optional)
    compute_time_sec: float = 0.0
    turnaround_delay_sec: float = 0.0

    # Kinematic bicycle
    wheelbase: float = 0.32

    # Constraints
    steering_min: float = np.radians(-40.0)
    steering_max: float = np.radians(40.0)
    steering_rate_min: float = np.radians(-20.0)
    steering_rate_max: float = np.radians(20.0)
    velocity_min: float = -0.4
    velocity_max: float = 1.5
    acceleration_min: float = -0.2
    acceleration_max: float = 0.4

    # Weights
    Q1: np.ndarray = field(default_factory=lambda: np.diag([100.0, 100.0, 10.0, 10.0]))
    Q2: np.ndarray = field(default_factory=lambda: np.diag([1e-3, 1e-2]))
    Q3: np.ndarray = field(default_factory=lambda: np.diag([10.0, 10.0]))
    Qf: np.ndarray = field(default_factory=lambda: np.diag([100.0, 100.0, 100.0, 70.0]))
    Qv_weight: float = 0.0

    # Obstacles
    # max_num_obstacles int = 8
    max_num_obstacles: int = 3

    # Virtual obstacles (test mode)
    use_virtual_obstacles: bool = True
    virtual_num_obstacles: int = 1
    virtual_radius: float = 0.25
    virtual_motion: str = "circle"   # "circle" or "line"


# =============================================================================
# Path generator (simple demo reference)
# =============================================================================

class PathGenerator:
    # @staticmethod
    # def generate_line_path(radius: float, center: Tuple[float, float], target_speed: float, dt: float) -> np.ndarray:
    #     x_start = center[0] + radius
    #     x_end = center[0] - radius
    #     y_const = center[1]

    #     path_length = 2 * radius
    #     ds_des = max(1e-3, target_speed * dt)
    #     N_points = int((path_length / ds_des) * 1.15)

    #     x_values = np.linspace(x_start, x_end, N_points)
    #     y_values = np.full_like(x_values, y_const)
    #     # yaw_values = np.full_like(x_values, math.pi)
    #     yaw_values = np.full_like(x_values, 0.0)

    #     return np.vstack((x_values, y_values, yaw_values))
    @staticmethod
    def generate_line_path(
        start: Tuple[float, float],
        end: Tuple[float, float],
        target_speed: float,
        dt: float
    ) -> np.ndarray:
        x0, y0 = start
        x1, y1 = end
        dx = x1 - x0
        dy = y1 - y0
        path_length = math.hypot(dx, dy)

        ds_des = max(1e-3, target_speed * dt)
        N_points = max(2, int((path_length / ds_des) * 1.15))

        x_values = np.linspace(x0, x1, N_points)
        y_values = np.linspace(y0, y1, N_points)

        yaw = math.atan2(dy, dx)
        yaw_values = np.full(N_points, yaw)

        return np.vstack((x_values, y_values, yaw_values))


# =============================================================================
# Virtual obstacles generator
# =============================================================================

class VirtualObstacleField:
    """
    Virtual obstacles generator for load-light testing.
    Returns obstacles as List[[cx, cy, r], ...]
    """
    def __init__(self, num: int, radius: float):
        self.num = max(0, int(num))
        self.r = float(radius)

        # Spread base centers
        self.centers = [(1.5 * i+5, 0.8 * (-1)**i - 1.8) for i in range(self.num)]
        # Frequencies and amplitudes
        self.omegas = [0.45 + 0.12 * i for i in range(self.num)]
        self.amps = [0.2 + 0.25 * i for i in range(self.num)]

    def circle_xyvv(self, t: float):
        out = []
        for i in range(self.num):
            cx0, cy0 = self.centers[i]
            w = self.omegas[i]
            a = self.amps[i]

            # pos
            x = cx0 + a * math.cos(w * t)
            y = cy0 + a * math.sin(w * t)

            # vel = d/dt
            vx = -a * w * math.sin(w * t)
            vy =  a * w * math.cos(w * t)

            out.append((x, y, vx, vy))
        return out

    def line_xyvv(self, t: float):
        out = []
        for i in range(self.num):
            cx0, cy0 = self.centers[i]
            v = 0.25 + 0.05 * i

            x = cx0 + v * t
            y = cy0
            vx = v
            vy = 0.0

            out.append((x, y, vx, vy))
        return out

    def circle_cxcy_r(self, t: float):
        obs = []
        for i in range(self.num):
            cx0, cy0 = self.centers[i]
            w = self.omegas[i]
            a = self.amps[i]
            cx = cx0 + a * math.cos(w * t)
            cy = cy0 + a * math.sin(w * t)
            obs.append([cx, cy, self.r])
        return obs

    def line_cxcy_r(self, t: float):
        obs = []
        for i in range(self.num):
            cx0, cy0 = self.centers[i]
            v = 0.25 + 0.05 * i
            cx = cx0 + v * t
            cy = cy0
            obs.append([cx, cy, self.r])
        return obs



# =============================================================================
# MPC core
# =============================================================================

class MPC:
    def __init__(self, cfg: MPCConfig) -> None:
        self.cfg = cfg
        self.total_delay_sec = self.cfg.compute_time_sec + self.cfg.turnaround_delay_sec
        self.delay_steps = int(max(0, round(self.total_delay_sec / self.cfg.dt)))

        self.opti = ca.Opti()
        self._define_variables()
        self._set_objective()
        self._set_constraints()
        self._set_solver_options()
        self.sol = None
        self._x_init_guess = None
        self._u_init_guess = None

    def _define_variables(self):
        # state: [x, y, theta, v, steering]
        self.x = self.opti.variable(5, self.cfg.N + 1)
        # input: [accel, steering_rate]
        self.u = self.opti.variable(2, self.cfg.N)

        self.x_init = self.opti.parameter(5)
        self.x_ref = self.opti.parameter(5, self.cfg.N + 1)

        # obstacle parameters (cx, cy, r)
        self.obj_cx = self.opti.parameter(self.cfg.max_num_obstacles)
        self.obj_cy = self.opti.parameter(self.cfg.max_num_obstacles)
        self.obj_r  = self.opti.parameter(self.cfg.max_num_obstacles)

    def _state_error(self, x_curr, x_ref):
        # return error for [x,y,theta,v] (4-d)
        diff = x_curr[0:4] - x_ref[0:4]
        yaw_diff = x_curr[2] - x_ref[2]
        diff[2] = ca.atan2(ca.sin(yaw_diff), ca.cos(yaw_diff))
        return diff

    def _set_objective(self):
        Q1 = ca.DM(self.cfg.Q1)
        Q2 = ca.DM(self.cfg.Q2)
        Q3 = ca.DM(self.cfg.Q3)
        Qv = ca.DM(self.cfg.Qv_weight)

        obj = 0
        for k in range(self.cfg.N):
            err = self._state_error(self.x[:, k], self.x_ref[:, k])
            obj += ca.mtimes([err.T, Q1, err])

            # delta u
            if k < self.cfg.N - 1:
                du = self.u[:, k + 1] - self.u[:, k]
            else:
                du = ca.DM.zeros(2, 1)
            obj += ca.mtimes([du.T, Q2, du])

            # input magnitude
            obj += ca.mtimes([self.u[:, k].T, Q3, self.u[:, k]])

            # optional negative velocity penalty
            v_pen = ca.fmax(0, -self.x[3, k])
            obj += ca.mtimes([v_pen.T, Qv, v_pen])

        Qf = ca.DM(self.cfg.Qf)
        terminal_err = self._state_error(self.x[:, self.cfg.N], self.x_ref[:, self.cfg.N])
        obj += ca.mtimes([terminal_err.T, Qf, terminal_err])

        self.opti.minimize(obj)

    def _set_constraints(self):
        self.opti.subject_to(self.x[:, 0] == self.x_init)

        dt = self.cfg.dt
        L = self.cfg.wheelbase

        for k in range(self.cfg.N):
            x_next  = self.x[0, k] + dt * self.x[3, k] * ca.cos(self.x[2, k])
            y_next  = self.x[1, k] + dt * self.x[3, k] * ca.sin(self.x[2, k])
            th_next = self.x[2, k] + dt * (self.x[3, k] / L) * (self.x[4, k])
            v_next  = self.x[3, k] + dt * self.u[0, k]
            st_next = self.x[4, k] + dt * self.u[1, k]

            self.opti.subject_to(self.x[0, k + 1] == x_next)
            self.opti.subject_to(self.x[1, k + 1] == y_next)
            self.opti.subject_to(self.x[2, k + 1] == th_next)
            self.opti.subject_to(self.x[3, k + 1] == v_next)
            self.opti.subject_to(self.x[4, k + 1] == st_next)

            # bounds
            self.opti.subject_to(self.opti.bounded(self.cfg.velocity_min, self.x[3, k], self.cfg.velocity_max))
            self.opti.subject_to(self.opti.bounded(self.cfg.steering_min, self.x[4, k], self.cfg.steering_max))
            self.opti.subject_to(self.opti.bounded(self.cfg.acceleration_min, self.u[0, k], self.cfg.acceleration_max))
            self.opti.subject_to(self.opti.bounded(self.cfg.steering_rate_min, self.u[1, k], self.cfg.steering_rate_max))

            # obstacle hard constraints
            for i in range(self.cfg.max_num_obstacles):
                # if (k % 2) == 0:
                #     continue
                dist_sq = (self.x[0, k] - self.obj_cx[i])**2 + (self.x[1, k] - self.obj_cy[i])**2
                min_sq  = (self.obj_r[i])**2
                self.opti.subject_to(dist_sq >= min_sq)
    
    def _shift_guess(self, X: np.ndarray, U: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Receding horizon warm start:
          X[:,k] <- X[:,k+1], last column repeated
          U[:,k] <- U[:,k+1], last column repeated
        """
        Xs = np.empty_like(X)
        Us = np.empty_like(U)

        Xs[:, :-1] = X[:, 1:]
        Xs[:, -1]  = X[:, -1]

        Us[:, :-1] = U[:, 1:]
        Us[:, -1]  = U[:, -1]
        return Xs, Us

    def _set_solver_options(self):
        # IPOPT: general-purpose nonlinear solver, very common in MPC prototyping with CasADi.
        opts = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.max_iter": 60,
            "ipopt.tol": 1e-2,
            "ipopt.acceptable_tol": 1e-2,
            "ipopt.acceptable_iter": 5,
            # "ipopt.hessian_approximation": "limited-memory",

        }
        self.opti.solver("ipopt", opts)

    def _bound_initial_state(self, s: np.ndarray) -> np.ndarray:
        s = s.astype(float).copy()
        s[3] = max(self.cfg.velocity_min, min(s[3], self.cfg.velocity_max))
        s[4] = max(self.cfg.steering_min, min(s[4], self.cfg.steering_max))
        return s

    def _rollout_delay(self, s: np.ndarray) -> np.ndarray:
        x = s.astype(float).copy()
        dt = self.cfg.dt
        L = self.cfg.wheelbase
        for _ in range(self.delay_steps):
            v = x[3]
            delta = x[4]
            x[0] += dt * v * math.cos(x[2])
            x[1] += dt * v * math.sin(x[2])
            # x[2] += dt * (v / L) * math.tan(delta)
            x[2] += dt * (v / L) * (delta)
        return x

    def solve(self, state_5: List[float], ref_5x: np.ndarray, obstacles: List[List[float]]) -> Tuple[float, float, bool]:
        # delay compensation
        s = np.array(state_5, dtype=float)
        if self.delay_steps > 0:
            s = self._rollout_delay(s)
        s = self._bound_initial_state(s)

        # ensure ref shape (5, N+1)
        if ref_5x.shape != (5, self.cfg.N + 1):
            raise ValueError(f"ref must be shape (5, N+1) = (5,{self.cfg.N+1}), got {ref_5x.shape}")

        # obstacle params
        cx = [0.0] * self.cfg.max_num_obstacles
        cy = [0.0] * self.cfg.max_num_obstacles
        rr = [0.0] * self.cfg.max_num_obstacles

        for i, obs in enumerate(obstacles[: self.cfg.max_num_obstacles]):
            cx[i], cy[i], rr[i] = float(obs[0]), float(obs[1]), float(obs[2])

        self.opti.set_value(self.x_init, s)
        self.opti.set_value(self.x_ref, ref_5x)
        self.opti.set_value(self.obj_cx, cx)
        self.opti.set_value(self.obj_cy, cy)
        self.opti.set_value(self.obj_r,  rr)

        # warm start
        if (self._x_init_guess is not None) and (self._u_init_guess is not None):
            # use shifted previous solution as initial guess
            X0, U0 = self._shift_guess(self._x_init_guess, self._u_init_guess)

            # enforce x0 = current state guess (helps feasibility)
            X0[:, 0] = s

            self.opti.set_initial(self.x, X0)
            self.opti.set_initial(self.u, U0)
        else:
            # first iteration fallback
            self.opti.set_initial(self.x, np.tile(s.reshape(-1, 1), (1, self.cfg.N + 1)))
            self.opti.set_initial(self.u, np.zeros((2, self.cfg.N)))


        ok = True
        try:
            self.sol = self.opti.solve()
        except RuntimeError:
            self.sol = self.opti.debug
            ok = False

        if ok:
            self._x_init_guess = np.array(self.sol.value(self.x))
            self._u_init_guess = np.array(self.sol.value(self.u))
            accel = float(self.sol.value(self.u[0, 0]))
            steer_rate = float(self.sol.value(self.u[1, 0]))
        else:
            accel, steer_rate = 0.0, 0.0
        return steer_rate, accel, ok

    def get_predicted_xy_points(self) -> Optional[np.ndarray]:
        """Return point set: array of shape (N+1, 2) = [(x0,y0),...]."""
        if self.sol is None:
            return None
        X = np.array(self.sol.value(self.x))  # shape (5, N+1)
        return X[0:2, :].T


# =============================================================================
# ROS2 Node: virtual obstacles -> run MPC -> send UDP point set
# =============================================================================

class MPCVirtualObstaclesNode(Node):
    def __init__(self):
        super().__init__("mpc_virtual_obstacles")

        self.cfg = MPCConfig()
        self.mpc = MPC(self.cfg)

        # reference path (demo). Replace with real planner output if needed.
        self.target_speed = 1.0
        # self.static_path = PathGenerator.generate_line_path(
        #     radius=5.0, center=(5.0, 0.0), target_speed=self.target_speed, dt=self.cfg.dt
        # )
        self.static_path = PathGenerator.generate_line_path(
            start=(0.0, 0.0),
            end=(10.0, 0.0),
            target_speed=self.target_speed,
            dt=self.cfg.dt
        )

        # ego initial state: [x, y, theta, v, steering]
        self.state = [0.0, 0.0, math.pi*0, self.target_speed, 0.0]

        # virtual obstacles
        self.virtual = VirtualObstacleField(
            num=min(self.cfg.virtual_num_obstacles, self.cfg.max_num_obstacles),
            radius=self.cfg.virtual_radius,
        )
        self.t0 = time.time()

        # UDP sender
        self.sender = MPCUdpSender(host="127.0.0.1", port_points=50052, port_obstacles=50051)

        # run MPC at fixed rate
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f"Started MPCVirtualObstaclesNode: dt={self.cfg.dt}s, N={self.cfg.N}, "
            f"virtual_obs={self.cfg.virtual_num_obstacles} ({self.cfg.virtual_motion}), udp=127.0.0.1:50052"
        )

    def _get_reference_trajectory(self) -> np.ndarray:
        # closest point on static path to current ego x,y
        xy = np.array(self.state[:2])
        dists = np.linalg.norm(self.static_path[:2, :].T - xy.reshape(1, 2), axis=1)
        idx = int(np.argmin(dists))
 
        start = idx + 1
        end = start + (self.cfg.N + 1)
        path_len = self.static_path.shape[1]

        if end <= path_len:
            ref3 = self.static_path[:, start:end]
        else:
            ref3 = self.static_path[:, start:]
            pad = (self.cfg.N + 1) - ref3.shape[1]
            last = self.static_path[:, -1].reshape(3, 1)
            ref3 = np.hstack([ref3, np.tile(last, (1, pad))])

        v_ref = np.full((1, self.cfg.N + 1), self.target_speed)
        steer_ref = np.zeros((1, self.cfg.N + 1))
        ref5 = np.vstack([ref3, v_ref, steer_ref])  # (5, N+1)
        return ref5

    def _virtual_obstacles_for_mpc(self, t: float) -> List[List[float]]:
        if self.cfg.virtual_motion == "line":
            return self.virtual.line_cxcy_r(t)
        return self.virtual.circle_cxcy_r(t)

    def _virtual_obstacles_for_udp(self, t: float):
        if self.cfg.virtual_motion == "line":
            return self.virtual.line_xyvv(t)
        return self.virtual.circle_xyvv(t)

    def _log_obstacles(self, t: float, obstacles_xyvv):
        if not obstacles_xyvv:
            # self.get_logger().info(f"[obs] t={t:.2f} none")
            return

        parts = []
        for i, (x, y, vx, vy) in enumerate(obstacles_xyvv):
            parts.append(f"#{i} p=({x:+.2f},{y:+.2f}) v=({vx:+.2f},{vy:+.2f})")
        # self.get_logger().info(f"[obs] t={t:.2f} " + " | ".join(parts))

    def on_timer(self):
        t = time.time() - self.t0
        t0 = time.perf_counter()

        ref = self._get_reference_trajectory()
        obstacles = self._virtual_obstacles_for_mpc(t)

        steer_rate, accel, ok = self.mpc.solve(self.state, ref, obstacles)
        t1 = time.perf_counter()
        t_diff = t1 - t0
        pts = self.mpc.get_predicted_xy_points()

        # self._save_points_csv(pts, obstacles, ok)

        if pts is None:
            self.get_logger().warn("No MPC solution points.")
            return

        # Send over UDP
        self.sender.send_points(pts, ok, steer_rate=steer_rate, accel=accel)

        obstacles_xyvv = self._virtual_obstacles_for_udp(t)
        self._log_obstacles(t, obstacles_xyvv)
        if not self.sender.send_obstacles_as_xyvv(obstacles_xyvv):
            self.get_logger().warn("Failed to send obstacle UDP packet(s).")

        self.get_logger().info(
            f"t_diff={t_diff:6.4f} ok={ok} obs={len(obstacles)} "
            # f"u=(sr={steer_rate:+.3f}, a={accel:+.3f}) "
            # f"p0=({pts[0,0]:+.2f},{pts[0,1]:+.2f}) pN=({pts[-1,0]:+.2f},{pts[-1,1]:+.2f})"
        )
    
    def _save_points_csv(self, pts_xy: np.ndarray, obstacles: List[List[float]], ok: bool):
        # pts_xy: (N+1,2)
        fname = "mpc_predicted_xy.csv"
        with open(fname, "w", newline="") as f:
            f.write("predicted_xy\n")
            f.write(f"ok: {ok}\n")
            f.write(f"objects: {obstacles}\n")
            w = csv.writer(f)
            w.writerow(["k", "x", "y"])
            for k, (x, y) in enumerate(pts_xy):
                w.writerow([k, float(x), float(y)])


def main():
    rclpy.init()
    node = MPCVirtualObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
