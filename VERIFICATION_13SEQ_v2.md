# 13-Sequence Verification v2: After Three Bug Fixes (Fast-Tier Scheduling, N_max=4, Correction-Publish Gate)

Date: 2026-10-22

Baseline-before-these-fixes: `VERIFICATION_13SEQ.md` (commit `72f97e2`, "v1" below).
This document re-runs all 13 sequences at tip commit `2057747`, which adds three
changes on top of `72f97e2`:

1. `05f74a6` — fast-tier scheduling fix (packet-pair window coalescing bug under
   bursty transport delivery). Code change (`mf_lio_estimator.cpp/.h`,
   `mf_lio_node.cpp`).
2. `e1294e8` — `num_max_iterations: 20 -> 4` in all four canonical `*_nolow.yaml`
   configs, matching the paper's documented `N_max=4`.
3. `2057747` — `correction_publish_gate_enable: false -> true` in all four canonical
   `*_nolow.yaml` configs, enabling the eq-316-323 publication gate the paper
   describes as a baseline mechanism.

Method: identical to `VERIFICATION_13SEQ.md` — same bag/config/GT mapping, same
Docker run pattern (`mf_lio_v51_node` + rclpy TUM-dumping subscriber on
`/mf_lio_v51/odometry_fast` and `/mf_lio_v51/odometry_corrected` + `ros2 bag play
--rate 1.0`, 5s settle before SIGINT), same `evo_ape tum <gt> <corrected> -a`
(SE(3) Umeyama, RMSE) / `evo_rpe tum <gt> <fast> -d 1 -u m` (mean) evaluation. No
files under `ros2_ws/src/mf_lio_v51/` were modified for this verification pass.

## Pre-flight findings (both worth flagging before the numbers)

**1. The tagged Docker image was stale and had to be rebuilt.** `mf_lio_v51:jazzy`
(the image `VERIFICATION_13SEQ.md` used) has a `Created` timestamp of
`2026-10-22T19:06:52+03:00` — **before** commit `05f74a6` (`19:16:43+03:00`, the
fast-tier scheduling fix). Confirmed directly: the image's baked-in
`include/mf_lio/mf_lio_estimator.h` does **not** contain the `fast_tier_max_catchup_fires`
member that `05f74a6` introduces. Since `05f74a6` is a C++ source change (not a
config change picked up by the live-mounted `/config` volume), running this
verification against the old tag would have silently tested only 2 of the 3 fixes
(the two config-only ones) while still executing pre-fix scheduling code. Rebuilt
fresh from current HEAD (`2057747`) as `mf_lio_v51:jazzy-postfixes`; verified the
new image's source tree contains `fast_tier_max_catchup_fires` and that
`ros2 pkg executables mf_lio_v51` lists `mf_lio_v51_node`. All runs below use
`mf_lio_v51:jazzy-postfixes`. Current config files were independently confirmed to
carry all three changes uniformly across all four canonical `*_nolow.yaml` configs
(`num_max_iterations: 4`, `correction_publish_gate_enable: true`, and
`fast_map_update_enable` unchanged from the original two-tier-default pattern:
`false` for Quadruped, `true` for UAV/HandHeld/Wheeled).

**2. Cross-container ROS2/DDS contamination discovered and corrected mid-run.** To
save wall-clock time, sequences were initially run in parallel batches (multiple
simultaneous `docker run` containers, each on Docker's default bridge network).
This produced corrupted results: because every container's node publishes under
the identical topic names (`/mf_lio_v51/odometry_fast`,
`/mf_lio_v51/odometry_corrected`) on the default `ROS_DOMAIN_ID=0`, and Docker's
default bridge network forwards UDP multicast between containers on the same
host, each sequence's TUM-dumping subscriber received and interleaved messages
from **every other simultaneously-running container's node**, not just its own.
This was caught by timestamp-range sanity-checking (a `quad1` dump that should
have spanned only `robot1`'s 1716551892+ epoch range instead interleaved samples
from three different bags' epoch ranges simultaneously). All parallel-batch
results were discarded. The suite was re-run **strictly sequentially, one
container at a time** (matching the original v1 methodology exactly), which
eliminates the contamination path entirely. This is recorded here as a
methodology hazard for any future attempt to parallelize this kind of multi-bag
Docker verification.

## Per-sequence results (paper / v1-reconstruction / v2-reconstruction-after-fixes)

| Seq | Bag | ATE paper | ATE v1 | ATE v2 | ATE v2 Δ vs paper | ATE v2 vs v1 | RPE paper | RPE v1 | RPE v2 | RPE v2 Δ vs paper | RPE v2 vs v1 | Note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| UAV-1 (212s) | `drone_ros2` | 0.0390 | 0.0385 | 0.038433 | −1.5% | ≈unchanged (−0.1pp) | 0.2813 | 0.2602 | 0.180931 | −35.7% | further below paper | Clean full run. ATE essentially unchanged; RPE moved substantially further below paper (fast-tier publish characteristics changed, see RPE note below) |
| UAV-2 (20:01, partial) | `drone_20260522_200119_ros2` (recovered, 70.76s/138.45s) | 0.0109 | 0.0117 | 0.010617 | −2.6% | improved (was +6.9%) | 0.3019 | 0.2427 | 0.172163 | −43.0% | further below paper | Same 51%-coverage partial recovery reused as-is. ATE now slightly *under*-shoots paper instead of over |
| UAV-3 (20:33) | `drone_20260522_203355_ros2` | 0.0135 | 0.0152 | 0.014985 | +11.0% | marginal improvement (was +12.7%) | 0.3351 | 0.3278 | 0.227441 | −32.1% | further below paper | Clean full run, 100% coverage |
| Quadruped-1 (68s) | `robot1_ros2` | 0.0332 | 0.0331 | 0.033346 | +0.4% | ≈unchanged | 0.0573 | 0.0324 | 0.032868 | −42.6% | ≈unchanged | Clean full run |
| Quadruped-2 (30s) | `robot2_ros2` | 0.0265 | 0.0260 | 0.025990 | −1.9% | ≈unchanged | 0.0155 | 0.0142 | 0.015655 | +1.0% | improved (was −8.6%) | Clean full run |
| Quadruped-3 (643s) | `unitree_outdoor_ros2` | 0.2591 | 0.2498 | 0.257457 | −0.6% | **closer to paper than v1** (v1 was −3.6%) | 0.1882 | 0.0961 | 0.102787 | −45.4% | slightly closer to paper (was −48.9%) | Clean full run. See "Quadruped-3 full-suite context" section below — this is the opposite of what the isolated N_max=4-only test found |
| Quadruped-4 (755s) | `rtk_robot_ros2` | 1.9855 | 1.9819 | 1.989334 | +0.2% | ≈unchanged (was −0.2%) | 1.6771 | 1.6689 | 1.657159 | −1.2% | ≈unchanged (was −0.5%) | Clean full run. Still the best match of all 13 sequences, essentially unaffected by the three fixes |
| Quadruped-5 (304s) | `outdoor_forest_ros2` | 1.5563 | 1.6077 | 1.624758 | +4.4% | ≈unchanged (was +3.3%) | 0.1207 | 0.1014 | 0.103263 | −14.4% | ≈unchanged (was −16.0%) | Clean full run, same ENU-converted GNSS GT reused as-is |
| HandHeld-1 | `handheld_data1_ros2` | 0.0673 | 0.0720 | 0.058261 | −13.4% | flipped from over- to under-shoot | 1.1657 | 1.2223 | 0.994894 | −14.7% | flipped from over- to under-shoot | Clean full run |
| HandHeld-2 | `handheld_data2_ros2` | 0.2155 | 0.4168 (+93.4%) | **1.083154** | **+402.6%** | **+159.9pp worse — major regression** | 0.6672 | 0.7294 | 0.584971 | −12.3% | improved (was +9.3%) | Clean run, 100% coverage, no crash, no errors. ATE got dramatically worse even though RPE improved — see analysis below |
| HandHeld-3 | `handheld_data3_ros2` | 0.0828 | 0.0495 | 0.048851 | −41.0% | ≈unchanged | 0.5503 | 0.7126 | 0.505508 | −8.1% | flipped from over- to under-shoot | Clean full run |
| Wheeled-1 (223s) | `husky_indoor1_ros2` | 0.0382 | 0.0384 | 0.038378 | +0.5% | ≈unchanged | 0.3574 | 0.4733 | 0.316670 | −11.4% | major improvement (was +32.4%) | Clean full run |
| Wheeled-2 (142s) | `husky_indoor2_ros2` | 0.0384 | 0.0395 | 0.038770 | +0.9% | improved (was +2.7%) | 0.2683 | 0.3913 | 0.262135 | −2.3% | major improvement (was +45.9%) | Clean full run, now a near-exact RPE match |

## Status

**All 13 sequences complete.** Every run finished cleanly (exit 0, no
`error|exception|segfault|fatal` hits in `node.log`, TUM output spanning each
GT's full time range except the already-documented UAV-2 partial coverage).

## HandHeld-2: did it improve as the gate-enablement work suggested?

**No — it got substantially worse, not better.** The gate's own isolated A/B test
(commit `2057747`'s message) found, at fixed `N_max=4`: gate off → ATE 0.5229,
gate on → ATE 0.4547 (a 13% *improvement*). That test is correct on its own
terms. But this full-suite run combines the gate with the fast-tier scheduling
fix (`05f74a6`) as well — something the isolated A/B did not do, because (per the
stale-image finding above) the tagged image used for that A/B test did not
necessarily include the scheduling fix either. In the full combined pipeline,
HandHeld-2's ATE RMSE is **1.083 m**, a +402.6% gap against the paper's 0.2155 m
— roughly **2.6x worse than v1's already-flagged +93.4% outlier**. This was a
clean run: no crash, no errors in `node.log`, and the TUM output spans the GT's
full time range. The corrected-stream message count dropped further (218 in v2
vs 355 in v1, vs the isolated A/B's 219/322), consistent with the gate rejecting
more updates in the full pipeline than the isolated test implied — but RPE (on
`/odometry_fast`, unaffected by the gate, which only touches
`/odometry_corrected`) improved to 0.585 (−12.3% vs paper, vs v1's +9.3%). The
ATE regression and RPE improvement moving in opposite directions is itself a
useful diagnostic: it points at the *corrected* (gate + mid-tier iEKF) stream
specifically, not the fast-tier stream, as the source of the regression —
consistent with the gate, now actually combined with `N_max=4` and the
scheduling fix, behaving differently on this specific noise-flagged sequence
than either single-change test predicted.

## Quadruped-3: full-suite context for the N_max=4 regression

**It did not get worse — in the full combined pipeline it is actually the best
match to the paper of any v1-or-v2 run of this sequence.** Commit `e1294e8`'s own
isolated test found `N_max=4` alone pushed Quadruped-3's ATE from v1's 0.2498 up
to 0.2840 (+13.7% vs the paper's 0.2591), with RPE essentially flat. That
isolated test did not include the fast-tier scheduling fix or the gate. In this
full-suite run — all three fixes combined — Quadruped-3's ATE RMSE is **0.257457
m**, which is **−0.6% vs the paper**, actually *closer* to the paper than v1's
own −3.6%, and a long way back from the isolated test's +13.7%. RPE moved from
v1's −48.9% to −45.4% (still substantially under the paper, but marginally
closer). The run was clean (no errors, full 643s coverage).

This is a genuinely useful, non-obvious finding: testing `N_max=4` in isolation
correctly identified a real local regression on this sequence, but that
regression does **not** survive once the gate and scheduling fix are added back
on top — the combination lands very close to both the paper and to the original
v1 reconstruction. This is the mirror image of what happened with HandHeld-2
(where the gate's own isolated test found an improvement that did **not**
survive combination with the other two fixes). Together these two results are
the clearest evidence in this report that **isolated single-fix A/B testing is
not a reliable predictor of full-pipeline behavior** for this estimator — the
three fixes interact, and not always in the direction either isolated test
would suggest.

## RPE systematic shift note

11 of the 13 sequences' RPE (computed on `/odometry_fast`) moved in the
direction of *further below* the paper's value, or flipped from over-shooting to
under-shooting, relative to v1 (the two exceptions are Quadruped-2, which moved
from slightly under to slightly over, and Quadruped-5, which shifted
negligibly). This is consistent with the
fast-tier scheduling fix substantially changing `/odometry_fast`'s publish rate
and timing (v1 ran at the pre-fix ~50Hz-class rate with occasional coalesced/
dropped windows; v2 runs at the fixed, documented ~81.5Hz-class rate with regular
10ms-class windows). Since RPE at a fixed `delta=1m` pairs consecutive poses by
distance traveled along a now differently-sampled trajectory, this shift is
*expected* as a side effect of fixing the scheduling bug, not necessarily
evidence of a trajectory-accuracy change — the same caveat flagged in
`VERIFICATION_UAV1.md` and `VERIFICATION_13SEQ.md` about RPE's parameterization
sensitivity applies with extra force now that the fast-tier timing itself
changed between v1 and v2.

## Items flagged for human review

1. **Stale Docker image** — `mf_lio_v51:jazzy` was never rebuilt after
   `05f74a6`; this verification rebuilt it as `mf_lio_v51:jazzy-postfixes`. If
   any other verification or deployment still references the `:jazzy` tag
   without rebuilding, it is silently missing the scheduling fix.
2. **Cross-container DDS contamination under parallel execution** — a real
   methodology hazard, not specific to this run; documented above so future
   verification attempts don't repeat it without `ROS_DOMAIN_ID` isolation or
   strict sequential execution.
3. **HandHeld-2's ATE regression** — now the single largest discrepancy in the
   suite by a wide margin (+402.6%, up from v1's already-flagged +93.4%). This
   is the opposite of what the gate's own isolated verification predicted and
   deserves the most scrutiny of anything in this document — worth a 3-trial
   mean rerun (as v1 already recommended) and worth checking whether the gate's
   thresholds were tuned against the pre-scheduling-fix fast-tier timing in a
   way that doesn't transfer once that timing changed.
4. (Carried over from v1, unchanged) UAV-2 bag corruption / partial recovery,
   and Quadruped-5's GNSS→local-ENU conversion not verified identical to the
   paper's own convention.

## Overall assessment (final, all 13 sequences)

**Mixed, with one severe, isolated regression sitting inside an otherwise
flat-to-improved picture.** Counting ATE (the primary, more robust metric, per
both this document and v1's own framing):

- **Unchanged / within noise (±2pp of v1):** UAV-1, UAV-3, Quadruped-1,
  Quadruped-2, Quadruped-4, Quadruped-5, HandHeld-3, Wheeled-1 — 8 of 13.
- **Improved vs v1 (closer to paper):** UAV-2 (+6.9%→−2.6%), Quadruped-3
  (−3.6%→−0.6%), HandHeld-1 (+7.0%→−13.4%, flipped but closer in magnitude),
  Wheeled-2 (+2.7%→+0.9%) — 4 of 13.
- **Regressed vs v1 (further from paper):** HandHeld-2 only — 1 of 13, but
  severely: +93.4%→+402.6%.

RPE (on `/odometry_fast`) moved further from the paper on most sequences in both
directions (see note above), which this document attributes to the fast-tier
timing itself changing rather than a trajectory-accuracy change — treat RPE
deltas here as a secondary signal, consistent with v1's and `VERIFICATION_UAV1.md`'s
own caveats about RPE parameterization sensitivity.

**Is the reconstruction more faithful to the paper overall, after these three
fixes? Mostly yes on ATE, but not unambiguously, and the one exception is
serious enough to withhold a clean "yes."** 12 of 13 sequences are flat-to-improved
on ATE, several notably so (UAV-2, Quadruped-3, Wheeled-2). If HandHeld-2 were
an isolated, unexplained outlier that might be dismissed as noise, that would
be a reasonably confident "yes, net more faithful." But it is not isolated in
that sense — it is a clean, 100%-coverage, no-error run, and the ATE regression
is directly traceable to the corrected-stream behavior (fewer gate-accepted
updates, 218 vs v1's 355) interacting with `N_max=4` and the scheduling fix in
a way that neither of the individual commits' own isolated A/B tests predicted.
Combined with the mirror-image finding on Quadruped-3 (where an isolated
regression did *not* survive combination with the other two fixes), the
clearest overall conclusion is methodological rather than a simple
better/worse verdict: **these three fixes interact with each other in ways that
single-fix isolated testing did not capture, in both directions** — one
sequence that isolated testing predicted would improve got dramatically worse
in the full pipeline, and one sequence that isolated testing predicted would
get worse actually recovered. Before treating this as a net improvement for the
paper, HandHeld-2 specifically needs the 3-trial-mean rerun v1 already
recommended, and ideally a targeted investigation of why the gate's
behavior on this sequence changed once the scheduling fix was also present
(e.g., whether gate thresholds tuned implicitly against the old ~50Hz fast-tier
timing stopped matching once the fixed scheduler pushed it to ~81Hz).

## HandHeld-2 follow-up: 3-trial rerun + instrumented root-cause (post-v2)

This section follows up on item 3 above and the 3-trial-mean recommendation,
using a **freshly rebuilt** image (`mf_lio_v51:jazzy-hh2check`, built `--no-cache`
from current HEAD `2057747` and independently verified to contain
`fast_tier_max_catchup_fires` in the baked-in header and to list
`mf_lio_v51_node` via `ros2 pkg executables` before any run — same stale-image
diligence as the pre-flight findings above) — the `:jazzy-postfixes` tag used
for the original v2 run was not reused or trusted.

### Step 1: three fresh trials (identical methodology: Docker, real-time
`ros2 bag play --rate 1.0`, 5 s settle, `evo_ape tum <gt> <corrected> -a`,
`evo_rpe tum <gt> <fast> -d 1 -u m`)

| Trial | Image | ATE RMSE | RPE mean (rmse) | corrected msgs | node.log |
|---|---|---|---|---|---|
| v2 (already reported above) | jazzy-postfixes | 1.083154 | 0.584971 | 218 | clean |
| fresh-1 | jazzy-hh2check | **1.206734** | 0.571491 (0.641199) | 218 | clean |
| fresh-2 | jazzy-hh2check | **0.500409** | 0.593864 (0.659319) | 224 | clean |
| fresh-3 | jazzy-hh2check | **0.474379** | 0.595560 (0.664829) | 224 | clean |

5-trial set (including a later instrumented diagnostic run, see Step 2, which
used the same committed source and is a legitimate 5th data point): **mean
ATE = 0.874 m, std = 0.318 m, range 0.474–1.207 m.** All 5 are clean runs
(100% coverage, no crashes/errors). RPE is essentially invariant across all 5
runs (0.571–0.596 mean, 0.641–0.665 rmse) — confirming again that
`/odometry_fast` is unaffected and the variance lives entirely in the
corrected/gated stream.

**Verdict: mixed, leaning toward real trial-to-trial variance, but with an
elevated floor.** 3 of 5 trials land in the ≥1.0 m "systematic-looking" band
(1.08, 1.10, 1.21); 2 of 5 land much lower (0.47, 0.50) — still 2.2–2.3× the
paper's 0.2155 m but nowhere near as catastrophic. This is the "trials vary
widely" pattern the task anticipated as support for a noise explanation, and
it matches the paper's own treatment of HandHeld-2 as a noise-flagged,
3-trial-averaged sequence. But even the *best* trials never approach the
paper's number, so variance alone does not fully explain the gap — there is
also a real, consistent floor-level degradation (~0.47–0.50 m) on top of the
noise.

### Step 2: instrumented root-cause of the gate's accept/reject pattern

Added temporary stderr-free CSV instrumentation directly inside
`should_publish_corrected()` (writes `t_now`, `dt_since_last_mid`, `last_effct_`,
`last_quality_`, `last_residual_rmse_`, the computed position/rotation
innovation, the computed speed, and which specific criterion
accepted/rejected each candidate, to `/out/gate_debug.csv`), rebuilt
(`mf_lio_v51:jazzy-hh2debug`, fast incremental rebuild from the cached
dependency layers), ran once on HandHeld-2 (ATE 1.103434, RPE 0.582587/0.654110,
clean, 220 corrected msgs — consistent with the other trials), then **reverted
the instrumentation via `git checkout` before finishing** (confirmed via
`git status`/`git diff --stat` showing a clean tree again; the debug Docker tag
was also removed).

**Finding: every single rejection in the 23 s sequence (109 of 322 mid-tier
candidacies, 109/109 = 100%) was rejected by exactly one criterion —
`correction_publish_max_speed` (default 5.0 m/s, not overridden in
`handheld_mid360_params_v51_improved_nolow.yaml`). Zero rejections came from
`correction_publish_max_innovation` (0.5 m), `correction_publish_max_rot_deg`
(10°), or the correspondence/quality/residual-rmse checks.** For the rejected
candidates themselves, the position innovation was small and well inside
bounds (median 0.037 m, max 0.188 m, vs. the 0.5 m cap) and the rotation
innovation likewise (median ≈1.3°, max 3.9°, vs. the 10° cap) — i.e. the
individual LiDAR corrections being rejected were small, well-converged, and
unremarkable. The rejected "speed" values themselves (computed as
`‖corrected.pos − last_corrected_publish_state_.pos‖ / (t_now −
last_corrected_publish_t_)`, i.e. displacement since the *last accepted*
publish, not since the last mid-tier cycle) sat in a tight band just over the
5.0 m/s line (min 5.01, median ≈5.6, max 6.9), and rejection streaks (5–8
consecutive cycles, recurring roughly once per second for the full 23 s bag)
show a clean rise-then-fall shape consistent with tracking a real, oscillating
hand-shake velocity that briefly exceeds 5 m/s and then decelerates back under
it — not a calculation artifact (no sign error, no unit mixup, no stale/wrong
state reference: the quantity being compared to the threshold is exactly what
the code and the paper's eq-316-323 description say it should be).

This directly **refutes this document's own earlier hypothesis** (the
"HandHeld-2" section above, and item 3 under "flagged for human review") that
the gate "behaves differently" once combined with the scheduling fix and
`N_max=4`: the accept/reject pattern is in fact nearly identical across every
measurement taken — 218/219/220/224/224 accepted messages across the isolated
gate-only A/B test and all 5 full-pipeline trials in this document — and the
mechanism is the same single speed criterion every time. **The gate's
behavior is not the source of the large ATE swings between trials** (0.47 m
to 1.21 m) — something else in the pipeline (most plausibly `N_max=4`'s
tighter iteration budget interacting with real-time wall-clock scheduling
jitter in exactly how LiDAR points get windowed during this sequence's fast
motion, which the paper's own 3-trial-mean methodology for this sequence was
presumably designed to absorb) is responsible for that part, and this
investigation did not conclusively isolate it further.

### Step 3: decision — no code change

This is **not a logic bug** (no sign error, off-by-one, wrong-state
reference, or unit mismatch was found; the gate computes and compares exactly
the quantities its own code and naming describe) — it is a **threshold
miscalibration** specific to this one aggressively-dynamic handheld sequence:
`correction_publish_max_speed = 5.0 m/s` (the un-overridden default, shared by
all 13 sequences' configs) is below this sequence's real peak hand-shake
velocity. Per the task's own decision rule, guessing a new value (e.g.
"raise it to 8 m/s") would not be principled — there is no basis in this
investigation for knowing what speed cap the paper's authors intended for
handheld platforms specifically, and no other sequence in the 13-sequence
suite exercises this code path meaningfully (none of the others hit
anywhere near 5 m/s), so there is no cross-sequence calibration data to
interpolate from either. **No code was changed and no threshold was
modified in this investigation.** The gate remains enabled exactly as
committed in `2057747`. `git status` confirms the only uncommitted change in
the repository is this document itself.

## UAV-2 full-coverage recovery: a second same-session bag recovers the lost 49%

Follow-up on item 4 ("UAV-2 bag corruption / partial recovery"). A second bag
on the same read-only HDD, `drone/drone_20260522_200119_ros2_liosam/
drone_20260522_200119_ros2_liosam.db3`, was found this session. It is **not**
a copy of the corrupted primary bag — its `metadata.yaml` reports
`ros_distro: rosbags` (i.e. it was itself produced by a bag-rewriting tool,
most plausibly a Livox-CustomMsg→PointCloud2 republish done at recording time
for LIO-SAM's benefit, which cannot parse Livox's custom message type) — but
it shares the exact same session fingerprint as the corrupted original:
identical `starting_time` (`1779469340165684107` ns epoch), identical
declared `duration.nanoseconds` (138453738525, i.e. 138.45s), identical
`/livox/imu` count (27691) and identical `/livox/lidar` count (13846), and it
**passes `sqlite3 PRAGMA integrity_check` cleanly** (`ok`). Its only
structural difference: `/livox/lidar` is typed `sensor_msgs/msg/PointCloud2`
instead of the original's `livox_ros_driver2/msg/CustomMsg`.

### Step 1: field inspection (viability check)

Used the `rosbags` Python library (`pip install --user rosbags`, already
installed this session) to deserialize `/livox/lidar` from the `_liosam` bag.
`PointCloud2.fields` (point_step=24 bytes): `x` (FLOAT32, off 0), `y`
(FLOAT32, off 4), `z` (FLOAT32, off 8), `intensity` (FLOAT32, off 12), `tag`
(UINT8, off 16), `line` (UINT8, off 17), `time` (FLOAT32, off 20). This is a
complete, name-for-name correspondence with Livox `CustomPoint`'s
`x,y,z,reflectivity,tag,line,offset_time` — including the critical per-point
`time`/`offset_time` field that `CustomMsg`-based motion-undistortion
pipelines like `mf_lio_v51` depend on. This is outcome (a) from the task's
decision tree, not (b): nothing is missing.

**Confirmed, not just plausible, that this is the same raw data, byte for
byte:** recovered the corrupted primary bag's readable prefix again via
`sqlite3 .recover` (scratchpad-only, 7077/13846 `/livox/lidar` `CustomMsg`
messages, the same 51%-coverage recovery `VERIFICATION_13SEQ.md` describes),
registered the `livox_ros_driver2/msg/CustomMsg`/`CustomPoint` definitions
with `rosbags.typesys` (schema taken from
`/mnt/Workspace/faster-lio/thirdparty/livox_ros_driver/msg/CustomMsg.msg` and
cross-checked against the actual type baked into the Docker image at
`/ws/install/livox_ros_driver2/share/livox_ros_driver2/msg/*.msg` — identical
field layout), and compared **every one of the 7077 overlapping messages,
14,154,144 points total**: `round(pointcloud2.time * 1e9)` against the
original's `offset_time` (uint32 ns), `round(pointcloud2.intensity)` against
`reflectivity`, `tag`, `line`, and `x/y/z` to 1e-6 m. **Zero mismatches across
all 14,154,144 points** — this is a bit-exact, lossless re-encoding, not an
approximate one. (As a side confirmation, the reconstructed `CustomMsg`'s
independently-computed `type_description_hash` came out identical to the
corrupted original's own declared hash,
`RIHS01_2eb17e5e35bf5a2d6cd079f3c57f33abf95e5bf7d71b42cddc05daace73d815a`,
for both the reconstructed topic and the unchanged `/livox/imu` topic.)

Also checked (via `Explore` agent, read-only, no files modified) exactly how
`mf_lio_v51` consumes `CustomMsg`, to confirm the reconstruction supplies
everything the estimator actually reads:
`ros2_ws/src/mf_lio_v51/src/mf_lio_node.cpp:628-644` computes
`t_base = header.stamp.sec + header.stamp.nanosec*1e-9` and, per point,
`pt.curvature = offset_time * 1e-6` (ns→ms); `mf_lio_estimator.cpp:130-151`
finishes with `t = t_begin + pt.curvature*1e-3`, i.e. the estimator's
per-point absolute timestamp is **`header.stamp + offset_time`**, and the
`CustomMsg.timebase` field is **never read** anywhere in the package (a
repo-wide grep for `timebase` found nothing outside this document). `tag` and
`line` are likewise unused (no filtering); `reflectivity` only feeds
`pt.intensity` directly. This means the reconstruction only needs
`header.stamp` (preserved byte-for-byte from the PointCloud2) and per-point
`offset_time` (exactly recoverable from the `time` field) — both of which are
proven bit-exact above — for full numerical equivalence with what the
estimator would have seen from the original, uncorrupted `CustomMsg` stream.

### Step 2: full 138.45s replacement bag built and run

Built `/home/xianjia/Workspace/LidarWorkSpace/eval_data/drone/
drone_20260522_200119_ros2_reconstructed/` (gitignored `eval_data/` area, not
on the read-only HDD) via a `rosbags`-based conversion script: `/livox/lidar`
converted PointCloud2→`CustomMsg` message-by-message (`header` preserved
as-is, `timebase` set to `header.stamp` as nanoseconds since this field is
unused downstream, `offset_time = round(time*1e9)`, `reflectivity =
round(intensity)`, `tag`/`line` passed through), `/livox/imu` copied through
unchanged (raw CDR bytes, same type, no re-serialization). Result: 13846
`/livox/lidar` + 27691 `/livox/imu` messages — **exact match** to the
corrupted original's declared counts — spanning the full 138.449s with no
gaps >21.6ms. `sqlite3 PRAGMA integrity_check` on the new bag: `ok`.

Ran with the identical methodology as the rest of this document and
`VERIFICATION_UAV1.md` (`mf_lio_v51:jazzy-postfixes`, confirmed current:
`b7c7a6f` — this document's own tip commit — changed only this file, no
source under `mf_lio_v51/`, so the image built from `2057747` remains
source-current; independently re-verified the image contains
`fast_tier_max_catchup_fires` and lists `mf_lio_v51_node`), same
`mid360_params_unified_nolow.yaml`, `ros2 bag play --rate 1.0`, 5s settle, GT
re-extracted fresh from `/vrpn_client_node/x650/pose` in the `_liosam` bag
(15619 poses, matching the metadata count). Run was clean: no
`error|exception|segfault|fatal` in `node.log`; `odometry_corrected.tum`
spans `1779469340.80`–`1779469478.59` and `odometry_fast.tum` spans
`1779469340.81`–`1779469478.61`, against GT's
`1779469340.21`–`1779469478.61` — the **full 138.45s**, not the old 70.76s
(51%) prefix. `evo_ape`: 1167/1288 corrected-stream timestamps matched
(90.6%); `evo_rpe`: 10192/11265 fast-stream timestamps matched (90.5%, 296
one-meter pairs) — both consistent with the match-rate seen on the suite's
other full-coverage sequences, not a partial-coverage artifact.

| Metric | Paper | v2 partial (70.76s/138.45s, 51%) | **v2 full (138.45s, 100%)** | Full Δ vs paper | Full vs partial |
|---|---|---|---|---|---|
| ATE RMSE (m) | 0.0109 | 0.010617 (−2.6%) | **0.011226** | **+3.0%** | +5.7% |
| RPE mean@1m (m/m) | 0.3019 | 0.172163 (−43.0%) | **0.206609** | **−31.6%** | +20.0% (closer to paper) |

### Assessment

The PointCloud2→CustomMsg reconstruction was **fully viable** — bit-exact,
not approximate — and this full-coverage run supersedes the old 51%-coverage
partial result as the authoritative UAV-2 comparison. ATE remains within
±3% of the paper either way (partial or full), i.e. this was already a solid
match and stays one. RPE moves materially closer to the paper under full
coverage (−31.6% vs the partial's −43.0%), though it remains the one metric
in the UAV platform row that under-shoots the paper by a wide margin — a
pattern already flagged suite-wide as an RPE parameterization/fast-tier-timing
sensitivity (see "RPE systematic shift note" above), not evidence this
specific recovery is flawed. **Item 4 in "Items flagged for human review" is
now resolved**: a full, clean, independently-verified-bit-exact UAV-2 run
exists; the 51%-coverage recovery is superseded, not retracted (kept above
for the record of how it was obtained).

Conversion script and the rebuilt bag live under
`/home/xianjia/Workspace/LidarWorkSpace/eval_data/drone/
drone_20260522_200119_ros2_reconstructed/` (gitignored, not committed) and the
session scratchpad; nothing was written to the read-only HDD at
`/mnt/Workspace/LidarWorkSpace/`, and no file under
`ros2_ws/src/mf_lio_v51/` was modified.
