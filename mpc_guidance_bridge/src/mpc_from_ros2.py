#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import csv
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy,QoSReliabilityPolicy
from svea_vision_msgs.msg import PersonStateArray

from send_udp_function import MPCUdpSender


# =============================================================================
# MPC Config
# =============================================================================

@dataclass
class MPCConfig:
    # Horizon
    N: int = 20
    dt: float = 0.1

    # Delay compensation (optional)
    compute_time_sec: float = 0.5
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
    max_num_obstacles: int = 8


# =============================================================================
# Path generator (simple demo reference)
# =============================================================================

class PathGenerator:
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

    def _define_variables(self):
        # state: [x, y, theta, v, steering]
        self.x = self.opti.variable(5, self.cfg.N + 1)
        # input: [accel, steering_rate]
        self.u = self.opti.variable(2, self.cfg.N)

        self.x_init = self.opti.parameter(5)
        self.x_ref = self.opti.parameter(5, self.cfg.N + 1)

        self.obj_cx = self.opti.parameter(self.cfg.max_num_obstacles)
        self.obj_cy = self.opti.parameter(self.cfg.max_num_obstacles)
        self.obj_r = self.opti.parameter(self.cfg.max_num_obstacles)

    def _set_objective(self):
        Q1 = ca.DM(self.cfg.Q1)
        Q2 = ca.DM(self.cfg.Q2)
        Q3 = ca.DM(self.cfg.Q3)
        Qv = ca.DM(self.cfg.Qv_weight)

        obj = 0
        for k in range(self.cfg.N):
            # state error (wrap yaw)
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

            # negative velocity penalty (optional)
            v_pen = ca.fmax(0, -self.x[3, k])
            obj += ca.mtimes([v_pen.T, Qv, v_pen])

        # terminal cost (optional: using Qf on [x,y,theta,v])
        Qf = ca.DM(self.cfg.Qf)
        terminal_err = self._state_error(self.x[:, self.cfg.N], self.x_ref[:, self.cfg.N])
        # terminal_err is 4-d for [x,y,theta,v], so embed properly
        # here _state_error returns 4-d vector
        obj += ca.mtimes([terminal_err.T, Qf, terminal_err])

        self.opti.minimize(obj)

    def _set_constraints(self):
        self.opti.subject_to(self.x[:, 0] == self.x_init)

        dt = self.cfg.dt
        L = self.cfg.wheelbase

        for k in range(self.cfg.N):
            x_next = self.x[0, k] + dt * self.x[3, k] * ca.cos(self.x[2, k])
            y_next = self.x[1, k] + dt * self.x[3, k] * ca.sin(self.x[2, k])
            # th_next = self.x[2, k] + dt * (self.x[3, k] / L) * ca.tan(self.x[4, k])
            th_next = self.x[2, k] + dt * (self.x[3, k] / L) * (self.x[4, k])
            v_next = self.x[3, k] + dt * self.u[0, k]
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
                dist_sq = (self.x[0, k] - self.obj_cx[i])**2 + (self.x[1, k] - self.obj_cy[i])**2
                min_sq = (self.obj_r[i])**2
                self.opti.subject_to(dist_sq >= min_sq)

    def _set_solver_options(self):
        opts = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.warm_start_init_point": "yes",
        }
        self.opti.solver("ipopt", opts)

    def _state_error(self, x_curr, x_ref):
        # return error for [x,y,theta,v] (4-d)
        diff = x_curr[0:4] - x_ref[0:4]
        yaw_diff = x_curr[2] - x_ref[2]
        diff[2] = ca.atan2(ca.sin(yaw_diff), ca.cos(yaw_diff))
        return diff

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
            x[2] += dt * (v / L) * math.tan(delta)
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
            cx[i], cy[i], rr[i] = obs

        self.opti.set_value(self.x_init, s)
        self.opti.set_value(self.x_ref, ref_5x)
        self.opti.set_value(self.obj_cx, cx)
        self.opti.set_value(self.obj_cy, cy)
        self.opti.set_value(self.obj_r, rr)

        # warm start
        self.opti.set_initial(self.x, np.tile(s.reshape(-1, 1), (1, self.cfg.N + 1)))
        self.opti.set_initial(self.u, 0)

        # t0 = time.time()
        t0 = time.perf_counter()
        ok = True
        try:
            self.sol = self.opti.solve()
        except RuntimeError:
            self.sol = self.opti.debug
            ok = False
        solve_time = time.perf_counter() -t0
        # _ = time.time() - t0

        if ok:
            accel = float(self.sol.value(self.u[0, 0]))
            steer_rate = float(self.sol.value(self.u[1, 0]))
        else:
            accel, steer_rate = 0.0, 0.0
        return steer_rate, accel, ok, solve_time

    def get_predicted_xy_points(self) -> Optional[np.ndarray]:
        """Return point set: array of shape (N+1, 2) = [(x0,y0),...]."""
        if self.sol is None:
            return None
        X = np.array(self.sol.value(self.x))  # shape (5, N+1)
        return X[0:2, :].T


# =============================================================================
# ROS2 Node: subscribe pedestrians -> run MPC -> output point set
# =============================================================================

class MPCFromROS2(Node):
    def __init__(self):
        super().__init__("mpc_from_ros2")

        self.cfg = MPCConfig()
        self.mpc = MPC(self.cfg)

        # reference path (demo). You can replace with your real planner output.
        self.target_speed = 1.0
        self.static_path = PathGenerator.generate_line_path(
            start=(0.0, 0.0),
            end=(10.0, 0.0),
            target_speed=self.target_speed,
            dt=self.cfg.dt
        )

        # ego initial state: [x, y, theta, v, steering]
        self.state = [0.0, 0.0, math.pi*0, self.target_speed, 0.0]

        self.qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        # latest pedestrians cache
        self.latest_people: Optional[PersonStateArray] = None
        self.sub = self.create_subscription(
            PersonStateArray,
            "/person_state_estimation/person_states_kf",
            self.on_people,
            self.qos,
        )

        # run MPC at fixed rate (decouple from message frequency)
        self.timer = self.create_timer(self.cfg.dt, self.on_timer)

        self.get_logger().info("MPCFromROS2 started (subscribing PersonStateArray, running MPC on timer).")

        self.sender = MPCUdpSender(host="127.0.0.1", port_points=50052, port_obstacles=50051)

    def on_people(self, msg: PersonStateArray):
        self.latest_people = msg

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

    def _people_to_obstacles(self, msg: PersonStateArray) -> List[List[float]]:
        # simple: each pedestrian -> (cx, cy, r)
        # r is fixed here; you can inflate using speed if needed.
        obstacles: List[List[float]] = []
        r = 0.35  # [m] tune
        for p in msg.personstate[: self.cfg.max_num_obstacles]:
            x = float(p.pose.position.x)
            y = float(p.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                obstacles.append([x, y, r])
        return obstacles

    def on_timer(self):
        if self.latest_people is None:
            # No pedestrians yet; still run MPC with no obstacles if you want
            self.get_logger().debug("No PersonStateArray received yet.")
            return

        ref = self._get_reference_trajectory()
        obstacles = self._people_to_obstacles(self.latest_people)

        steer_rate, accel, ok, solve_time = self.mpc.solve(self.state, ref, obstacles)
        pts = self.mpc.get_predicted_xy_points()

        if pts is None:
            self.get_logger().warn("No MPC solution points.")
            return

        # Output: point set (x,y). Here: save a rolling CSV + print head.
        self._save_points_csv(pts, obstacles, ok)
        # self.get_logger().info(
        #     f"MPC ok={ok} obstacles={len(obstacles)} u=(steer_rate={steer_rate:.3f}, accel={accel:.3f}) "
        #     f"pred_xy[0]={pts[0,0]:.2f},{pts[0,1]:.2f} pred_xy[-1]={pts[-1,0]:.2f},{pts[-1,1]:.2f}"
        # )        
        self.get_logger().info(
            # f"MPC ok={ok} obstacles={len(obstacles)} u=(steer_rate={steer_rate:.3f}, accel={accel:.3f}) "
            # f"pred_xy[0]={pts[0,0]:.2f},{pts[0,1]:.2f} pred_xy[-1]={pts[-1,0]:.2f},{pts[-1,1]:.2f}"
            f"MPC ok={ok}, obstacles={len(obstacles)}, solve_time={solve_time*1000:.1f} ms"
        )

        
        self.sender.send_points(pts, ok, steer_rate=steer_rate, accel=accel)


        # visualization (optional)

        # try:
        #     from visualize_trajectory import plot_mpc_control, plot_state_profiles
        #     from visualize_inputs import plot_u
        #     plot_mpc_control(input_filename='./mpc_predicted_xy.csv', output_filename='mpc_trajectory_plot.png')
        #     # plot_state_profiles(input_filename='./mpc_predicted_xy.csv', output_filename='mpc_state_profiles.png')
        #     # plot_u('./1shot_mpc_u.csv', '1shot_mpc_u_plot.png')
        #     # print("Plots generated.")
        # except ImportError:
        #     print("Visualization modules not found, skipping plots.")

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
    node = MPCFromROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
