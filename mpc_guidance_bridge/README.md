# mpc_guidance_bridge

ROS 2 person-state and SVEA odometry bridge for MPC visual guidance.

This directory is the focus of the `feat/vision-guidance` branch. The bridge
consumes detected people, transforms them into the MPC frame, solves a SVEA
kinematic bicycle MPC trajectory, and sends both the predicted trajectory and
the exact obstacle set used by the solver over UDP for the video/HMI path.

## Data Flow

```text
PersonStateArray + TF + SVEA odometry
              |
              v
      mpc_guidance_bridge
              |
      +-------+--------+
      |                |
      v                v
UDP trajectory   UDP selected obstacles
127.0.0.1:50052 127.0.0.1:50051
      |
      v
video/HMI overlay
```

The main inputs are:

- `svea_vision_msgs/PersonStateArray` from
  `/person_state_estimation/person_states_kf`
- `nav_msgs/Odometry` from `svea2/odometry`
- TF for `zed_camera_center`, `self/base_link`, and the incoming person-state
  frame

The main outputs are:

- Predicted trajectory points by UDP to `127.0.0.1:50052`
- MPC-selected obstacles by UDP to `127.0.0.1:50051`
- `mpc/debug/predicted_path_map`
- `mpc/debug/reference_path_map`

## Recommended Node

For the current visual-guidance work, use:

```bash
cd redo_camera/mpc_guidance_bridge
python3 mpc_from_ros2_sync_bridge_acados_svea_moving_obstacles.py
```

This node uses SVEA odometry to anchor the local MPC state and handles detected
people as global constant-velocity moving obstacles.

## Script Guide

- `mpc_from_ros2_sync_bridge_acados_svea_moving_obstacles.py`: recommended
  moving-obstacle SVEA/acados bridge.
- `mpc_from_ros2_sync_bridge_acados_svea.py`: SVEA/acados bridge with static
  obstacle handling.
- `mpc_from_ros2_sync_bridge_acados.py`: acados sync bridge without SVEA odom
  anchoring.
- `mpc_from_ros2_sync_bridge.py`: sync bridge variant based on the base MPC
  implementation.
- `mpc_from_ros2_acados.py`: common acados/IPOPT MPC implementation and ROS 2
  node base.
- `mpc_from_ros2.py`: older/base MPC implementation.
- `person_state_udp_bridge.py`: lightweight person-state to UDP obstacle bridge
  without running MPC.
- `send_udp_function.py`: UDP packet helper shared by the bridge scripts.
- `zonotope_mpc_from_ros2_sync_bridge_svea.py`: zonotope-oriented SVEA bridge.
- `*_test_ideal_path.py` and `*_case_moving.py`: experiment/test variants.

## Runtime Settings

Common environment variables:

```bash
export MPC_SVEA_ODOM_TOPIC=svea2/odometry
export MPC_FIXED_TARGET_DISTANCE_M=3.0
export MPC_FIXED_TARGET_REACHED_TOL_M=0.35
export MPC_REAR_AXLE_FRAME=/self/base_link
export MPC_TARGET_FRAME_TO_REAR_AXLE_X_M=0.4
export MPC_TARGET_FRAME_TO_REAR_AXLE_Y_M=0.0
```

Solver/debug settings:

```bash
export MPC_SOLVER_BACKEND=acados_sqp_rti
export MPC_ACADOS_NLP_MAX_ITER=5
export MPC_ACADOS_QP_MAX_ITER=100
export MPC_ACADOS_FORCE_REBUILD=0
export MPC_COMPUTE_TIME_SEC=0.0
export MPC_TURNAROUND_DELAY_SEC=0.0
export MPC_DEBUG_PLOT_ENABLED=0
export MPC_DEBUG_PLOT_PATH=/tmp/mpc_debug_plot.png
```

`acados` generated solver files are cached under `/tmp/acados_*`. Set
`MPC_ACADOS_FORCE_REBUILD=1` when the solver shape or constraints change.

## Person-State UDP Bridge Only

Use this when another process runs guidance/control and only needs the detected
people as UDP obstacles:

```bash
cd redo_camera/mpc_guidance_bridge
python3 person_state_udp_bridge.py --ros-args \
  -p topic:=/person_state_estimation/person_states_kf \
  -p target_frame:=zed_camera_center \
  -p udp_host:=127.0.0.1 \
  -p udp_port_obstacles:=50051
```

Parameters:

- `topic`, default `/person_state_estimation/person_states_kf`
- `target_frame`, default `zed_camera_center`
- `udp_host`, default `127.0.0.1`
- `udp_port_obstacles`, default `50051`
- `tf_timeout_sec`, default `0.1`

## UDP Formats

Obstacle UDP payload is little-endian:

- `uint32 count`
- repeated `float32 x, float32 y, float32 vx, float32 vy`

Trajectory UDP payload:

- magic `MPCP`
- version `1`
- flags, where bit `0` means the MPC result is valid
- sequence number
- timestamp in nanoseconds
- point count
- repeated `float32 x, y` points

## Debugging

Enable a static debug plot:

```bash
export MPC_DEBUG_PLOT_ENABLED=1
export MPC_DEBUG_PLOT_PATH=/tmp/mpc_debug_plot.png
```

Inspect ROS debug paths:

```bash
ros2 topic echo /mpc/debug/predicted_path_map
ros2 topic echo /mpc/debug/reference_path_map
```

The moving-obstacle bridge logs selected obstacles and solver status, including
solve time and acados failure status when a solve fails.
