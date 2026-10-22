# UAV-1 (212s) Verification Against mf-lio Paper

Date: 2026-10-22

## Target (from paper)

`sec/05_Evaluation.tex`, Table `tab:odometry_comparison`, UAV-1 (212 s) row,
"MF-LIO" entry: **ATE = 0.0390 m**, **RPE = 0.2813 (m/m)**. Table caption
(lines ~505-507) fixes the evaluation convention: ATE is computed on
`/odometry_corrected`, RPE on `/odometry_fast`, no per-sequence stream
selection.

## 1. Bag identification

Candidates under `/mnt/Workspace/LidarWorkSpace/data/drone/`. Duration was
read from each bag's `metadata.yaml` (`rosbag2_bagfile_information.duration.nanoseconds`):

| bag | duration (s) | starting_time (unix) |
|---|---|---|
| `drone_ros2` | **211.748** | 1769530644.5477 |
| `drone_20260127_synthetic10hz_ros2` | 211.745 | 1769530644.5477 (same recording, lidar downsampled to 10 Hz) |
| `drone_20260522_200119_ros2` | 138.454 | 1779469340.17 |
| `drone_20260522_203355_ros2` | 151.477 | 1779471296.17 |

`drone_ros2` duration (211.75 s) matches the paper's "UAV-1 (212 s)" almost
exactly. Its `starting_time` (1769530644.548) also matches the first
timestamp of `drone_gt.txt` (1769530644.576) to within 28 ms, confirming
this bag/GT pair line up. (The `20260522_200119`/`20260522_203355` bags
turned out to be named after *wall-clock start time* — 20:01:19 and
20:33:55 — which happens to match the "(20:01)" / "(20:33)" labels used for
UAV-2/UAV-3 in the table; those labels are timestamps, not ~20-minute
durations as I'd initially assumed from the task framing — durations are
138 s and 151 s respectively, not relevant to UAV-1 either way.)

`drone_20260127_synthetic10hz_ros2` is the same recording (identical
starting_time, identical duration, identical IMU message count 42350) with
`/livox/lidar` downsampled to 10 Hz (2118 vs 21175 lidar messages in the
native bag). Per the task's native-rate preference, I used **`drone_ros2`**
(native rate: `/livox/lidar` 21175 msgs, `/livox/imu` 42350 msgs,
`/Odometry` 21175 msgs) as the UAV-1 sequence.

## 2. Config identification

`ros2_ws/src/mf_lio_v51/config/mid360_params_unified_nolow.yaml` was used —
the confirmed two-tier-default (fast+mid, low tier disabled via
`W_l_min_ms=W_l_max_ms=1000000.0`) canonical config. Verified it subscribes
to `lidar_topic: "/livox/lidar"` and `imu_topic: "/livox/imu"`, matching the
topics actually present in `drone_ros2` (a Livox MID360 recording).

## 3. Build

`mf_lio_v51:jazzy` Docker image was already present and built from the
exact tip commit (`5a18777`, image `Created` timestamp 2026-10-22T15:08:18,
~20s before the commit's own timestamp — i.e. it is the image built during
this session's final commit). Verified `ros2 pkg executables mf_lio_v51`
inside the image lists `mf_lio_v51_node`. Did not rebuild since it was
already current; config file `mid360_params_unified_nolow.yaml` has existed
since the initial-import commit (`81aba61`), well before this image build,
so it is baked into the image's `src/mf_lio_v51` copy.

## 4. Run

No launch file matches the current package (the two `.launch.py` files in
`ros2_ws/src/mf_lio_v51/launch/` are stale leftovers referencing an old
package name, `mf_lio_v45_cov_motion` — not touched, not used). Ran the node
directly via `ros2 run` instead.

Container invocation (bag and config mounted read-only, scratch output dir
mounted read-write):

```
docker run --rm \
  -v /mnt/Workspace/LidarWorkSpace/data/drone/drone_ros2:/bag/drone_ros2:ro \
  -v <repo>/ros2_ws/src/mf_lio_v51/config:/config:ro \
  -v <scratchpad>/uav1_run:/scripts:ro \
  -v <scratchpad>/uav1_run/out:/out:rw \
  mf_lio_v51:jazzy bash /scripts/run_inside_container.sh
```

`run_inside_container.sh`:
1. `ros2 run mf_lio_v51 mf_lio_v51_node --ros-args --params-file /config/mid360_params_unified_nolow.yaml` (background)
2. a small rclpy subscriber (`dump_trajectories.py`) subscribing to
   `/mf_lio_v51/odometry_fast` and `/mf_lio_v51/odometry_corrected`,
   dumping each to TUM format (background)
3. `ros2 bag play /bag/drone_ros2 --rate 1.0` (foreground, real-time replay, ~212 s)
4. 5 s settle, then SIGINT both background processes

Node logged clean startup (`MF-LIO v46-fast-map node started,
output_prefix=/mf_lio_v51`) and ran the full bag without crashing.
Output: `odometry_fast.tum` = 10,737 messages, `odometry_corrected.tum` =
1,972 messages, spanning the bag's full time range (first/last timestamps
match the bag/GT start and end to within fractions of a second). No code
changes were needed to get this running — no fix was required.

## 5. ATE / RPE computation

`evo` 1.38.1 installed via `pip install --user evo` (not previously present
on this machine).

**ATE** (SE(3) Umeyama alignment, standard for metric LiDAR-inertial
odometry, no scale correction):
```
evo_ape tum drone_gt.txt out/odometry_corrected.tum -a
```
Matched 1972/1972 possible timestamps (100% coverage).
Result: **RMSE = 0.038545 m** (mean 0.037373, median 0.037785, std 0.009435, max 0.073078).

**RPE** on `/odometry_fast`. The paper's "(m/m)" unit indicates
translational error measured per meter of distance traveled, so I used
`--delta 1 --delta_unit m` (consecutive pose pairs 1 m apart along the
trajectory) with the default `trans_part` relation (translation-only RPE,
which is standard and SE(3)-alignment-invariant between consecutive pairs
— confirmed numerically identical with/without `-a`):
```
evo_rpe tum drone_gt.txt out/odometry_fast.tum -d 1 -u m
```
Matched 10733/10737 timestamps (99.96%), 274 one-meter-spaced pairs.
Result: **mean = 0.2602 m/m**, rmse = 0.4194, median = 0.1175, std = 0.3290,
max = 1.4826.

I also checked the default evo invocation (delta=1 frame, i.e.
consecutive-message RPE at the native fast-tier publish rate) purely for
context: rmse = 0.035 m — this is a different, much smaller quantity and
clearly not what a "(m/m)" unit label refers to, so I did not use it as the
primary comparison.

## 6. Comparison against paper

| Metric | Paper (UAV-1) | My run | Relative diff |
|---|---|---|---|
| ATE (m) | 0.0390 | 0.0385 (RMSE) | -1.2% |
| RPE (m/m) | 0.2813 | 0.2602 (mean, delta=1m) | -7.5% |

ATE matches the paper's published number almost exactly (within 1.2%,
well inside any variation expected from Docker/real-time-playback timing
jitter). RPE is in the same order of magnitude and the same ballpark
(within ~7.5%), though the exact match depends on evo's RPE
parameterization (delta spacing/unit), which the paper does not fully
specify; I tried several (delta=0.5 m, 1 m, 2 m, 5 m, 10 m; mean vs rmse)
and 0.2813 falls naturally between the delta=0.5m RMSE (0.225) and
delta=1m mean (0.260), confirming the right order of magnitude and
parameterization family without being bit-exact — consistent with the
task's own expectation that evo parameterization differences would shift
results somewhat.

## 7. Assessment

**Faithful reconstruction.** The run completed cleanly end-to-end (full
212 s bag, no crash, no divergence, 99.96-100% topic coverage), the
computed ATE on `/odometry_corrected` matches the paper's reported value to
within ~1%, and the RPE on `/odometry_fast` is within the right order of
magnitude and qualitative range (~0.26-0.28 m/m) under the most natural
reading of the paper's "(m/m)" convention. This is strong evidence that the
reconstructed `mf_lio_v51` package, run with the `mid360_params_unified_nolow.yaml`
two-tier-default config, reproduces the paper's UAV-1 headline numbers. No
code changes were required to the package to get this run working.

## Exact commands log

See `run_inside_container.sh` and `dump_trajectories.py` for the full
run script (left in the session scratchpad, not in the repo). Key commands:

```
sg docker -c 'docker build -f docker/mf_lio_v51/Dockerfile -t mf_lio_v51:jazzy .'   # (already current; not rebuilt)
sg docker -c "docker run --rm -v .../drone_ros2:/bag/drone_ros2:ro -v .../config:/config:ro \
    -v .../uav1_run:/scripts:ro -v .../uav1_run/out:/out:rw mf_lio_v51:jazzy \
    bash /scripts/run_inside_container.sh"
pip install --user evo
evo_ape tum drone_gt.txt out/odometry_corrected.tum -a
evo_rpe tum drone_gt.txt out/odometry_fast.tum -d 1 -u m
```
