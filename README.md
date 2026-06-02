# redo_camera

Realtime camera streaming and visual guidance stack for SVEA remote driving.

This branch, `feat/vision-guidance`, focuses on connecting perception and
MPC-generated guidance to the video/HMI path. The main new area is
`mpc_guidance_bridge/`: Python ROS 2 nodes that consume detected person states
and SVEA odometry, solve an MPC trajectory, and publish the trajectory and the
obstacles used by MPC over UDP for visualization/control-side consumers.

## What This Branch Adds

- MPC guidance bridges for ROS 2 person tracking and SVEA odometry.
- acados SQP-RTI MPC backend, with IPOPT fallback in the base implementation.
- Moving-obstacle support using constant-velocity person prediction.
- UDP packets for predicted trajectory points on port `50052`.
- UDP packets for the exact obstacle set used by MPC on port `50051`.
- Debug ROS `nav_msgs/Path` topics for predicted and reference paths.
- SVEA HMI UDP bridge for odometry and steering values used by the stream.
- Additional camera entry points for USB, ZED, and Spinnaker-based streams.

## Repository Layout

- `main.cpp`, `main_usb.cpp`, `main_spinnaker.cpp`: stream entry points.
- `zed_stream/`, `usb_cam_stream/`, `spinnaker_stream/`: camera backends.
- `camera/`: calibration, fisheye/homography, overlays, and visual components.
- `cuda/`: GPU image conversion and formatting.
- `sensor/`: UDP sensor bridge and logging.
- `motion/`: CYRA and bicycle-model prediction helpers.
- `mpc_guidance_bridge/`: ROS 2 to MPC to UDP guidance bridge scripts.
- `svea_hmi_bridge/`: ROS 2 package that sends odometry/steering to UDP.
- `run/`: build, v4l2, streaming, FleetMQ, and remote-driving helper scripts.

## MPC Guidance Bridge

The MPC visual-guidance work lives in `mpc_guidance_bridge/`. It consumes
person-state detections, SVEA odometry, and TF; then sends MPC trajectory points
and selected obstacles over UDP for the video/HMI path.

Recommended script for this branch:

```bash
cd redo_camera/mpc_guidance_bridge
python3 mpc_from_ros2_sync_bridge_acados_svea_moving_obstacles.py
```

See `mpc_guidance_bridge/README.md` for script variants, runtime environment
variables, UDP packet formats, and debugging notes.

## SVEA HMI UDP Bridge

The ROS 2 package in `svea_hmi_bridge/` sends steering and velocity values as
UDP for the video overlay/HMI path.

```bash
cd redo_camera/svea_hmi_bridge
colcon build --symlink-install
source install/setup.bash
ros2 launch svea_hmi_bridge udp_bridge.launch.py
```

Important parameters:

- `udp_host`, default `0.0.0.0`
- `udp_port`, default `10086`
- `odom_topic`, default `svea2/odometry`
- `steering_source`, `manual_control` or `rc_out`
- `manual_control_topic`, default `/mavros/manual_control/send`
- `rc_out_topic`, default `/mavros/rc/out`
- `steering_max_rad`, default `pi/4`

The bridge sends `<float32 steering_angle_rad, float32 velocity_x_mps>` every
`0.1` seconds.

## Build

```bash
cd redo_camera
run/build
```

or build the Docker image:

```bash
cd redo_camera
run/docker-build
```

## Initialize v4l2loopback

```bash
cd redo_camera
run/init_v4l2
```

The processed stream is normally written to `/dev/video16`.

## Run Camera Stream

FleetMQ/remote-driving streaming helper:

```bash
cd redo_camera
run/streamming start [-delay <time_ms>] [-hmi] [-p_hmi] [-trajectory_bars]
```

Check available options:

```bash
run/streamming -h
```

Restart or stop:

```bash
run/streamming restart [-delay <time_ms>] [-hmi] [-p_hmi] [-trajectory_bars]
run/streamming stop
```

View the local v4l2 output:

```bash
ffplay /dev/video16
```

Logs are written under `run/logs/`, and output data is saved under
`run/output/`.

## Remote-Driving Helper

`run/remote_driving_minimal` starts the Docker-based remote-driving stack,
teleop, perception, mocap/odometry, FleetMQ, and the camera stream.

```bash
cd redo_camera
run/remote_driving_minimal start
run/remote_driving_minimal status
run/remote_driving_minimal logs
run/remote_driving_minimal stop
```

Run the MPC guidance bridge in the ROS container/session after the stack is up
when visual guidance is needed:

```bash
cd /app/redo_camera/mpc_guidance_bridge
python3 mpc_from_ros2_sync_bridge_acados_svea_moving_obstacles.py
```

## Stream Parameters

The native streaming binary accepts:

- `-h`: show help.
- `-d <device>`: video device, default `/dev/video16`.
- `-fps <fps>`: frames per second, default `60`.
- `-scale <scale>`: frame scale, default `1`.
- `-delay <time_ms>`: video delay in milliseconds, default `0`.
- `-s`: enable sensor-data overlay.
- `-hmi`: enable HMI overlay.
- `-p_hmi`: enable prediction HMI overlay.
- `-ip <ip>`: sensor UDP bind address, default `0.0.0.0`.
- `-p <port>`: base sensor UDP port, default `10086`.
- `-log <logger>`: logger output file.
- `-fc`: calibrate fisheye camera.
- `-fu <image>`: undistort an image.
- `-hc`: calibrate homography.

When `-s` is enabled, the stream listens on:

- `<port>` for lower-system data.
- `<port> + 1` for control-tower data.
- `<port> + 2` for FleetMQ latency.

## Dependencies

- ROS 2 Jazzy for bridge nodes.
- `svea_vision_msgs` for `PersonStateArray`.
- `nav_msgs`, `geometry_msgs`, `tf2_ros`, and `rclpy`.
- CasADi.
- acados/acados_template for the preferred MPC backend.
- OpenCV.
- CUDA.
- v4l2loopback.
- ZED SDK for ZED streaming.
- Spinnaker SDK for Spinnaker streaming.

## Calibration

### Fisheye Calibration

1. Save chessboard images to `run/data/*.jpg`.
   The chessboard should have `10x7` vertices with `2.5 cm` squares.
2. Run the stream binary from `run/` with `-fc`.
3. The output is written to `fisheye_calibration.yaml`.

### Fisheye Undistortion

```bash
cd redo_camera
run/svea_stream -fu <image>
```

The undistorted image is written next to the input image.

### Homography Calibration

Create `homography_points.yaml`:

```yaml
%YAML:1.0
---
# Format: [left front wheel x/y, right front wheel x/y,
#          left front 50m x/y, right front 50m x/y]
points: [479., 1079., 1440., 1079., 639., 720., 1280., 720.]
```

Then run:

```bash
cd redo_camera/run
./svea_stream -hc
```

The output is written to `homography_calibration.yaml`.

## Benchmark Note

Older production testing on NVIDIA Jetson AGX Xavier measured roughly:

| Processing Type | Time (ms) |
| --- | ---: |
| Bayer to RGB only | 8.7 |
| Sequential | 47.6 |
| Parallel | 16.0 |
| CUDA, pure stream | 8.0 |
| CUDA, with components | 10.0 |
