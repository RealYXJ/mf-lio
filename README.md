# MF-LIO

ROS2 package and build setup for MF-LIO, the multi-frequency LiDAR-inertial
odometry estimator described in the associated IEEE Transactions on
Robotics submission.

## Contents

- `ros2_ws/src/mf_lio_v51/` — the ROS2 package source (C++17, `ament_cmake`).
  Extracted from a 2026-09-26 development snapshot; this is a work-in-progress
  mirror of an actively iterated research codebase, not yet a trimmed
  camera-ready release. In particular `config/` currently contains many
  historical experiment variants alongside the configs actually used for the
  paper's results — picking out the canonical per-platform config is a
  pending cleanup task before this repo is considered release-ready.
- `docker/mf_lio_v51/Dockerfile` — builds a `ros:jazzy-ros-base` image with
  `Livox-SDK2` + `livox_ros_driver2` (the Livox Mid-360 CustomMsg dependency)
  and compiles `mf_lio_v51` against it. Used to get a working build
  environment independent of the host OS's ROS2/Ubuntu version.

## Build

```bash
docker build -f docker/mf_lio_v51/Dockerfile -t mf_lio_v51:jazzy .
```

## Status

Work in progress — pushed here to have a single canonical place for
implementation work going forward, rather than scattered across local
research-machine backups.
