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

## Status: recovered history + scoped reconstruction (not a full replay)

The source tree that produced the paper's final published numbers (the
13-sequence main table, the RDEF/CI diagnostic, the three-way low-tier
ablation, the compute table) was lost: the evaluation machine's `ros2_ws/`
and `results/` directories were wiped shortly after the final evaluation
campaign completed, and no git history survived anywhere in this project's
prior lineage. **This repo's git history is split into two honestly
distinguishable halves — read `git log` to see exactly where the line is:**

1. **Recovered** (commits `81aba61`, `1adcc86`, `2d9e75d`) — two real
   historical source snapshots that survived on a separate backup: a
   2026-09-26 full tarball and a 2026-09-29 partial diff. Both are genuine,
   byte-for-byte recovered code.
2. **Reconstructed** (commits `6c4b4c9`, `881b2de`, `5a18777`, `1a307bc`) —
   everything the Sep-29 snapshot doesn't cover, rebuilt from this
   project's own detailed task-history log (a persistent record of the
   real engineering work, including exact flag names, config values, and
   formulas) and the paper's own precise mathematical specification
   (`03_ProposedMethod.tex`), **not** recovered from any surviving binary
   or source. Each reconstruction commit message cites the specific paper
   section/equation and task number it implements, so it can be audited
   directly against the published spec.

**Scope**: reconstruction was deliberately limited to what the *current*
paper draft needs (two-tier default architecture, the RDEF/CI diagnostic,
the adaptive low-tier ablation, the three-way comparison configs) — not a
full replay of the much larger original engineering process, which also
included several abandoned/reverted mechanisms (a fast-tier range-adaptive
voxel that regressed ATE and was reverted, an extensive low-tier "own-map"
experimentation series, a loop-closure module) that never made it into the
paper and were intentionally not reconstructed here.

**Verification**: the reconstructed two-tier default was run against the
real UAV-1 sequence (212s) and scored with `evo` against the paper's
published numbers — see `VERIFICATION_UAV1.md` for the full methodology.
Result: ATE within 1.2% of the paper's 0.0390 m, RPE within the right
order of magnitude of the paper's 0.2813 (m/m). Notably, the two-tier
default config was present unchanged since the Sep-26 import — the
headline result was never actually lost, only the secondary disclosed-
negative mechanisms needed reconstruction.

Still work in progress: `config/` contains many historical experiment
variants alongside the canonical per-platform configs — collapsing that
down is a pending cleanup, not yet done here.
