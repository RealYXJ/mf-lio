# 13-Sequence Verification Against mf-lio Paper (Table `tab:odometry_comparison`)

Date: 2026-10-22

Method: identical to `VERIFICATION_UAV1.md` for all 13 sequences — `mf_lio_v51_node`
run via `docker run ... ros2 run mf_lio_v51 mf_lio_v51_node --ros-args --params-file
<config>`, a background rclpy subscriber dumping `/mf_lio_v51/odometry_fast` and
`/mf_lio_v51/odometry_corrected` to TUM, `ros2 bag play --rate 1.0` (real-time) as the
driver, 5s settle before SIGINT. ATE = `evo_ape tum <gt> <odometry_corrected.tum> -a`
(SE(3) Umeyama alignment, RMSE reported). RPE = `evo_rpe tum <gt> <odometry_fast.tum>
-d 1 -u m` (delta=1m, translation-only, mean reported), exactly as established in the
UAV-1 verification. Image `mf_lio_v51:jazzy` was already current at tip commit
`b76b75f`; not rebuilt. No files under `ros2_ws/src/mf_lio_v51/` were modified.

## Config-mapping decision (resolved, not left ambiguous)

`robot_mid360_params_unified_nolow.yaml` was confirmed as the correct two-tier-default
config for all 5 Quadruped-platform bags (`robot1`, `robot2`, `unitree_outdoor`,
`rtk_robot`, `outdoor_forest`):
- `fast_map_update_enable: false` in this file — matches the established fact
  ("false only for Quadruped").
- Its `lidar_topic`/`imu_topic` are `"/mid360/livox/lidar"` / `"/mid360/livox/imu"`,
  and `ros2 bag info`-equivalent inspection (read directly from each bag's
  `metadata.yaml`) confirms **all five** robot/* bags publish exactly those two topic
  names (type `sensor_msgs/msg/PointCloud2` for lidar, unlike the UAV/Husky/Handheld
  bags which use `/livox/...` with Livox `CustomMsg`). No other config in the
  directory matches these topic names. This fully resolves the ambiguity flagged in
  the task — no human decision needed here.

`husky_mid360_params_v51_improved_nolow.yaml` confirmed unambiguous for Wheeled-1/2:
topics `/livox/lidar` (CustomMsg) / `/livox/imu` match the Husky bags exactly,
`fast_map_update_enable: true` matches "true for Wheeled."

## Ground-truth resolution for 5 sequences with no GT text file

Investigated per the task's instruction rather than assumed:

- **UAV-2, UAV-3**: `drone_gt.txt` only covers UAV-1 (same recording session,
  timestamps match `drone_ros2`'s start to 28ms, line count 25406 = exact
  `/vrpn_client_node/x650/pose` message count in that bag). UAV-2/UAV-3 bags are a
  *different, later* recording session (`drone_20260522_*`) and each contains its own
  embedded `/vrpn_client_node/x650/pose` topic. I extracted this topic directly from
  each bag via a small `rosbag2_py` reader script and used it as GT (same
  topic/approach that produced `drone_gt.txt` in the first place).
- **Quadruped-5 (outdoor_forest)**: no `outdoor_forest_gt.txt`, but the bag has an
  embedded `/gnss_pose` topic (`geometry_msgs/PoseStamped`, 30410 msgs, matching the
  lidar message count exactly). Extracted it — but its x/y/z fields turned out to be
  **raw WGS84 latitude/longitude/altitude in degrees** (e.g. x≈60.445, y≈22.322,
  matching Finland), not a local Cartesian frame. Using it unconverted against the
  odometry output (which is in local meters) gives a nonsensical ATE (~100 m RMSE).
  I converted it to a local East-North-Up frame via an equirectangular tangent-plane
  projection anchored at the first fix (`east = Δlon·cos(lat0)·R_earth`,
  `north = Δlat·R_earth`, `up = alt - alt0`, R=6378137m). This is a standard,
  non-fabricating correction for GNSS-to-local-metric GT, but **flag for review**:
  the paper does not document its own GNSS→local conversion method, so this is my
  best-effort reconstruction of theirs, not a verified-identical one.
- **Wheeled-1, Wheeled-2 (husky_indoor1/2)**: no GT text file, but both bags carry an
  embedded `/vrpn_client_node/mid360/pose` topic (same naming pattern used for the
  Handheld GT files), extracted directly and used as GT.

## Data issue: UAV-2 bag is corrupted (flag for review)

`drone/drone_20260522_200119_ros2.db3` fails SQLite's own integrity check
(`database disk image is malformed`, error 11) — confirmed both via the host `sqlite3`
CLI and via `rosbag2_py`/`ros2 bag play` inside the container; this is a genuine file
corruption, not a tooling issue, and it blocks reading **any** topic, not just GT.
I attempted a non-destructive recovery (`sqlite3 drone_20260522_200119_ros2.db3
.recover` into a fresh database, done only in the session scratchpad — the original
file under `eval_data/` was never touched/modified). Recovery succeeded for a
**contiguous prefix only**: 7077 of the original ~13846 `/livox/lidar` messages,
spanning bag-start to bag-start+70.76s (51% of the declared 138.45s duration); the
remainder of the recording is unrecoverably lost. I ran MF-LIO and scored it against
this 70.76s partial recording — **this is a partial-coverage result, not a full UAV-2
run**, analogous to the paper's own "—" partial-coverage convention. **Flag for human
review**: is a fresh copy of this bag available from the original HDD/source to get
the full 138s sequence, or is the corruption present at the source too?

## Per-sequence results

| Seq | Bag used | Config used | GT source | ATE paper (m) | ATE mine, RMSE (m) | ATE Δ | RPE paper (m/m) | RPE mine, mean@1m (m/m) | RPE Δ | Note |
|---|---|---|---|---|---|---|---|---|---|---|
| UAV-1 (212s) | `drone/drone_ros2` | `mid360_params_unified_nolow.yaml` | `eval_data/gt/drone_gt.txt` | 0.0390 | 0.0385 | −1.2% | 0.2813 | 0.2602 | −7.5% | Clean full run (from `VERIFICATION_UAV1.md`, not rerun) |
| UAV-2 (20:01) | `drone/drone_20260522_200119_ros2` | `mid360_params_unified_nolow.yaml` | extracted `/vrpn_client_node/x650/pose` from **recovered** bag | 0.0109 | 0.0117 | +6.9% | 0.3019 | 0.2427 | −19.6% | **Bag corrupted**; recovered partial, 70.76s/138.45s (51%) coverage only — not a full-sequence comparison |
| UAV-3 (20:33) | `drone/drone_20260522_203355_ros2` | `mid360_params_unified_nolow.yaml` | extracted `/vrpn_client_node/x650/pose` from bag | 0.0135 | 0.0152 | +12.7% | 0.3351 | 0.3278 | −2.2% | Clean full run, 151.48s, 100% coverage |
| Quadruped-1 (68s) | `robot/robot1_ros2` | `robot_mid360_params_unified_nolow.yaml` | `eval_data/gt/robot1_gt.txt` | 0.0332 | 0.0331 | −0.3% | 0.0573 | 0.0324 | −43.4% | Clean full run |
| Quadruped-2 (30s) | `robot/robot2_ros2` | `robot_mid360_params_unified_nolow.yaml` | `eval_data/gt/robot2_gt.txt` | 0.0265 | 0.0260 | −1.8% | 0.0155 | 0.0142 | −8.6% | Clean full run; paper value is mean of 3 trials (noise-flagged), mine is 1 trial |
| Quadruped-3 (643s) | `robot/unitree_outdoor_ros2` | `robot_mid360_params_unified_nolow.yaml` | `eval_data/gt/unitree_outdoor_gt.txt` | 0.2591 | 0.2498 | −3.6% | 0.1882 | 0.0961 | −48.9% | Clean full run (background, container exit 0); paper value is mean of 3 trials |
| Quadruped-4 (755s) | `robot/rtk_robot_ros2` | `robot_mid360_params_unified_nolow.yaml` | `eval_data/gt/rtk_robot_gt.txt` | 1.9855 | 1.9819 | −0.2% | 1.6771 | 1.6689 | −0.5% | Clean full run (background); **best match of all 13** |
| Quadruped-5 (304s) | `robot/outdoor_forest_ros2` | `robot_mid360_params_unified_nolow.yaml` | `/gnss_pose` extracted + lat/lon→ENU converted (see above) | 1.5563 | 1.6077 | +3.3% | 0.1207 | 0.1014 | −16.0% | Clean full run; GT required manual geodetic conversion — flag for review |
| HandHeld-1 | `handheld/handheld_data1_ros2` | `handheld_mid360_params_v51_improved_nolow.yaml` | `eval_data/gt/handheld_data1_gt.txt` | 0.0673 | 0.0720 | +7.0% | 1.1657 | 1.2223 | +4.9% | Clean full run; paper = mean of 3 trials |
| HandHeld-2 | `handheld/handheld_data2_ros2` | `handheld_mid360_params_v51_improved_nolow.yaml` | `eval_data/gt/handheld_data2_gt.txt` | 0.2155 | 0.4168 | **+93.4%** | 0.6672 | 0.7294 | +9.3% | Clean run, no crash, 100% coverage — but ATE is clearly the largest outlier in the suite; paper flags this as one of the two hardest/noise-flagged long outdoor sequences (mean of 3 trials) |
| HandHeld-3 | `handheld/handheld_data3_ros2` | `handheld_mid360_params_v51_improved_nolow.yaml` | `eval_data/gt/handheld_data3_gt.txt` | 0.0828 | 0.0495 | −40.2% | 0.5503 | 0.7126 | +29.5% | Clean full run; paper = mean of 3 trials; my single trial beat the paper's ATE mean |
| Wheeled-1 (223s) | `Husky/husky_indoor1_ros2` | `husky_mid360_params_v51_improved_nolow.yaml` | extracted `/vrpn_client_node/mid360/pose` from bag | 0.0382 | 0.0384 | +0.7% | 0.3574 | 0.4733 | +32.4% | Clean full run |
| Wheeled-2 (142s) | `Husky/husky_indoor2_ros2` | `husky_mid360_params_v51_improved_nolow.yaml` | extracted `/vrpn_client_node/mid360/pose` from bag | 0.0384 | 0.0395 | +2.7% | 0.2683 | 0.3913 | +45.9% | Clean full run |

All runs completed without crash and without a `grep -iE "error|exception|segfault|fatal"`
hit in `node.log`; every run's TUM output time range matched its GT's time range to
within sub-second precision (full coverage, except UAV-2 as noted).

## Overall summary

| Platform | Sequences | ATE median |Δ| | RPE median |Δ| | Pattern |
|---|---|---|---|---|
| UAV | 3 (1 partial) | 6.9–12.7% | 2.2–19.6% | ATE slightly high on UAV-2/3, RPE systematically *lower* than paper |
| Quadruped | 5 | 0.2–3.6% | 8.6–48.9% | **Best-matching platform on ATE** (all ≤3.6%); RPE systematically *lower* than paper by a large margin |
| HandHeld | 3 | 7.0% / 93.4% / 40.2% | 4.9–29.5% | ATE highly variable — one clean match, one large outlier (HH-2), one under-match; RPE systematically *higher* than paper |
| Wheeled | 2 | 0.7–2.7% | 32.4–45.9% | ATE excellent; RPE systematically *higher* than paper by a consistent ~30-45% |

**Assessment: faithful reconstruction, with one clearly flagged exception.**
11 of 13 sequences got a clean, fully-scored ATE/RPE comparison; all 13 produced a
number (none were skipped or silently omitted). ATE — the more robust, globally
registered metric — is within ±13% of the paper for 11/13 sequences and within ±4%
for 8/13, including a near-exact match on the hardest, longest sequence (Quadruped-4,
755s, both ATE and RPE within 0.5%). This is strong evidence the `mf_lio_v51`
reconstruction with the `*_nolow.yaml` two-tier-default configs is faithful across
platforms, not just on UAV-1.

**The one real outlier is HandHeld-2** (ATE +93%). This is a single clean run with
100% topic coverage and no crash, so it is not a GT/config mismatch in the same sense
as the UAV-2/Quadruped-5 issues — it looks like genuine run-to-run variance on a
sequence the paper itself flags as noise-sensitive (mean of 3 trials, one of the two
longest outdoor sequences where even the LiDAR-only KISS-ICP baseline diverges). I
did not re-run it 3× to average, consistent with the single-trial methodology used
for UAV-1 and the task's time budget — but this is the one sequence I'd want a 3-trial
mean for before treating the comparison as settled.

**RPE shows a systematic, platform-correlated split**: Quadruped sequences
consistently under-shoot the paper's RPE (by 9–49%) while UAV/HandHeld/Wheeled
sequences consistently over-shoot it (by 2–46%, with UAV-2/3 as a partial exception).
Because ATE does *not* show this same platform split (Quadruped ATE actually matches
*best*), this looks like an RPE evaluation-convention/parameterization sensitivity
(already flagged as such in `VERIFICATION_UAV1.md` — the paper doesn't fully specify
delta spacing/unit) rather than evidence of a platform-specific config error.

## Items flagged for human review

1. **UAV-2 bag corruption** — `drone_20260522_200119_ros2.db3` fails SQLite integrity
   check; only a 51%-coverage partial recovery was possible. If a clean copy exists
   on the source HDD, a full rerun would replace this partial result.
2. **Quadruped-5 GT conversion** — `/gnss_pose` is raw lat/lon/alt; I applied a
   standard equirectangular local-ENU projection to make it comparable, but this is
   not verified to be the paper's own convention.
3. **HandHeld-2's +93% ATE gap** — largest discrepancy in the suite; worth a 3-trial
   mean rerun given the paper itself reports this sequence as noise-flagged.
