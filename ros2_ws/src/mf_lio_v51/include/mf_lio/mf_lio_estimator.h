#pragma once

#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <deque>
#include <vector>
#include <fstream>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/imu.hpp>

#include "mf_lio/raw_point_buffer.h"
#include "mf_lio/feature_extractor.h"
#include "mf_lio/adaptive_policy.h"
#include "mf_lio/imu_processor.h"
#include "mf_lio/use-ikfom.hpp"
#include "mf_lio/common_lib.h"
#include "mf_lio/ikd-Tree/ikd_Tree.h"
#include "mf_lio/scan_context.h"
#include "mf_lio/sliding_window.h"
#include "mf_lio/voxel_plane_hash.h"

struct MfLioParams {
    // Sensor
    Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();
    Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();
    bool extrinsic_est_en = false;
    bool state_extrinsic_undistort = false;
    bool legacy_mixed_extrinsic_undistort = false;
    double extrinsic_rot_cov = 1e-8;
    double extrinsic_trans_cov = 1e-8;
    bool   extrinsic_update_gate_enable = false;
    double extrinsic_update_min_angular_velocity = 0.0;
    double extrinsic_update_min_quality = 0.0;
    double extrinsic_update_max_residual_rmse = 1e9;
    bool   extrinsic_update_adaptive_enable = false;
    double extrinsic_update_adaptive_high_angular_velocity = 7.0;
    int    extrinsic_update_adaptive_min_high_count = 3;
    double extrinsic_update_adaptive_min_quality = 0.02;
    double extrinsic_update_adaptive_max_residual_rmse = 0.50;
    // IMU noise
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    // Initial velocity covariance. Large (1.0) allows fast velocity tracking;
    // small (0.01) reduces K_vel_z amplification of weak z-constraints (high-altitude UAV).
    double init_vel_cov = 1.0;
    // Multi-frequency (3-tier)
    double W_f_ms = 10.0;   // fast_freq: fixed 100 Hz
    double W_m_ms = 100.0;  // mid_freq:  fixed 10 Hz (FAST-LIO2 equivalent, map write)
    double W_l_base_ms = 200.0, W_l_min_ms = 200.0, W_l_max_ms = 500.0;  // low_freq: adaptive 5 Hz
    double alpha = 0.5, beta = 0.001;
    // Feature
    float edge_thresh = 0.65f, plane_thresh = 0.75f;
    // Map
    double filter_size_map  = 0.2;
    double filter_size_scan = 0.0;  // 0 → use filter_size_map; >0 → separate scan density
    int    point_filter_num = 1;    // stride-N pre-filter before voxel (1=off, 3=FL2-equivalent)
    double cube_side_length = 1000.0;
    // iEKF
    int num_max_iterations = 4;

    // RIEKF ablation: when true, mid/low_freq_update's correction and
    // imu_processor's propagation both run mf_lio::riekf::InEKFFilter instead
    // of kf_'s own standard IKFoM math -- same ikdtree_/coarse_ikdtree_ maps,
    // same NN search/plane-fit, same multi-tier scheduling; only the filter's
    // error convention/propagate+update math differs. Result is bridged back
    // into kf_ via change_x/change_P after every step, so every other
    // consumer of kf_.get_x()/get_P() is unaffected by which filter actually
    // produced the state. Default false: provably a no-op (single top-level
    // gate at every new call site).
    bool riekf_enable = false;
    double esti_plane_threshold = 0.1;
    // v28/v29 accuracy improvements
    double angular_gate_rad_s    = 1000.0; // disabled (v29: replaced by CQG)
    double map_write_dist_thresh = 1000.0; // disabled (Change B off)
    double temporal_decay_alpha       = 1.0;   // TCW: point weight = exp(-alpha * t_rel/W_l); 0=off
    double min_correspondence_quality = 0.25;  // CQG: revert iEKF if effct/total < this; 0=off
    // v26-plus guard: reject only clearly bad low-frequency dense refinements.
    int    low_freq_min_correspondences = 20;
    double low_freq_min_quality         = 0.02;
    double low_freq_max_residual_rmse   = 0.30;
    double low_freq_max_pos_delta       = 1.50;
    double low_freq_max_rot_delta_deg   = 20.0;
    double publish_start_delay_ms       = 2500.0;
    bool   publish_requires_stable_update = true;
    int    publish_stable_min_correspondences = 20;
    double publish_stable_min_quality = 0.02;
    double publish_stable_max_residual_rmse = 0.50;
    double publish_stable_min_state_norm = 0.0;
    bool   publish_fast_odometry_on_main = true;
    bool   publish_corrected_odometry_on_main = true;
    bool   correction_publish_gate_enable = false;
    int    correction_publish_min_correspondences = 20;
    double correction_publish_min_quality = 0.02;
    double correction_publish_max_residual_rmse = 0.50;
    double correction_publish_max_innovation = 0.50;
    double correction_publish_max_rot_deg = 10.0;
    double correction_publish_max_speed = 5.0;
    // Conservative post-update guard. This is especially useful when online
    // extrinsic estimation is enabled: a bad scan association can otherwise
    // move pose and extrinsics together and poison the map.
    bool   update_motion_guard_enable = false;
    double update_motion_guard_max_pos_delta = 3.0;
    double update_motion_guard_max_rot_deg = 45.0;
    double update_motion_guard_max_speed = 30.0;
    // Reliability-aware low-frequency fusion. The dense low-tier update can
    // improve accuracy, but only when its residuals and innovation are sane.
    bool   low_freq_trust_enable        = true;
    double low_freq_trust_reject        = 0.20;
    double low_freq_trust_full          = 0.80;
    double low_freq_quality_good        = 0.08;
    double low_freq_residual_good       = 0.05;
    double low_freq_residual_bad        = 0.30;
    double low_freq_pos_delta_good      = 0.03;
    double low_freq_pos_delta_bad       = 1.50;
    double low_freq_rot_delta_good_deg  = 1.0;
    double low_freq_rot_delta_bad_deg   = 20.0;
    double low_freq_degen_good          = 1e-4;
    double low_freq_degen_bad           = 1e-7;
    double low_freq_motion_good         = 0.5;
    double low_freq_motion_bad          = 6.0;
    // Option C: covariance-gated low-freq influence.
    // When mid-freq posterior covariance is low (confident update), reduce low-freq weight.
    // When mid-freq was uncertain (high covariance), give low-freq full influence.
    bool   cov_gate_enable   = true;
    double cov_gate_pos_low  = 1e-4;  // below → mid confident → attenuate low-freq
    double cov_gate_pos_high = 1e-2;  // above → mid uncertain  → full low-freq
    double cov_gate_base     = 0.4;   // min multiplier when mid is fully confident
    // Option F: motion-triggered mid-freq scheduling.
    // Fires mid-freq early when accumulated rotation or translation exceeds thresholds.
    bool   mid_freq_motion_trigger   = true;
    double mid_freq_trigger_rot_deg  = 5.0;   // early trigger if Δrot > 5°
    double mid_freq_trigger_trans_m  = 0.10;  // early trigger if Δpos > 0.10 m
    double mid_freq_min_interval_ms  = 50.0;  // never faster than 20 Hz
    // Fast-tier read-only map correction: 1 iEKF iteration at 100 Hz, no map writes.
    // Corrects IMU drift between mid-freq updates without modifying the filter state.
    bool   fast_map_update_enable    = false;
    // Ablation-only: fully disables the fast tier's cadence loop (no propagation,
    // no GN correction, no /odometry_fast updates beyond whatever was last published).
    // Default true preserves all existing behavior.
    bool   fast_tier_enable          = true;
    bool   disjoint_scheduling_enable = true;
    int    fast_map_min_points       = 20;    // skip if scan too sparse
    double fast_map_min_quality      = 0.03;  // skip if correspondence quality too low
    double fast_map_voxel_size       = 0.10;  // downsampling for the 10ms scan
    // Fast-tier write-back to kf_: apply damped GN delta to the persistent filter state
    // only when the 6x6 normal-equation matrix is well-conditioned (all 6 DOF observable).
    // Guards against rank-deficient corrections in featureless/outdoor environments.
    bool   fast_writeback_enable      = false;
    double fast_writeback_lambda_thresh = 50.0;  // min eigenvalue of HtH required
    double fast_writeback_kappa_thresh  = 500.0; // max condition number (max/min eig)
    double fast_writeback_damp          = 0.3;   // fraction of delta applied per call
    // §5 Full 6-DoF fast correction (§6 normalization): use the full 6×6 Hessian instead
    // of position-only 3×3. fast_norm_rot_scale (s_R) normalizes rotation units relative
    // to position: H̄ = S·H·S where S=diag(1,1,1,s_R,s_R,s_R). s_R=1 = no rescaling.
    bool   fast_6dof_enable      = false;
    double fast_norm_rot_scale   = 1.0;   // s_R: rotation-unit scale for Hessian normalization
    // Degeneracy-aware mid-tier guard (v48):
    // After iEKF converges, eigendecompose the 3x3 position Hessian (H_pos^T * H_pos).
    // DOFs with eigenvalue below mid_degen_eigen_thresh are treated as unobservable
    // and their position correction is zeroed out (projection onto constrained subspace).
    // The map write is also gated: if any DOF is degenerate, the scan is not added to
    // the ikd-Tree map, preventing map contamination from ill-conditioned updates.
    bool   mid_degen_project_enable  = false; // project out degenerate position DOFs
    double mid_degen_eigen_thresh    = 1e-3;  // eigenvalue below this → degenerate DOF
    bool   mid_degen_map_gate_enable = false; // skip map write when any DOF is degenerate
    // Projection is disabled after this many seconds from the first scan (0 = unlimited).
    // Keeps it active only for sparse-map initialization; natural Kalman gain handles later.
    double mid_degen_project_max_sec = 0.0;

    // NDT-eigenplane correspondences for low-tier (v51 experiment):
    // When true, replaces esti_plane (QR Ax=-1) with eigendecomposition of the
    // 5-NN covariance matrix in low_freq_update. Normal = smallest eigenvector;
    // planarity gate = lambda_min > esti_plane_threshold. Mid-tier unchanged.
    bool low_freq_ndt_enable = false;

    // Loop closure via Scan Context + SVD ICP (low-tier keyframes)
    bool   loop_closure_enable             = false;
    float  loop_closure_score_thresh       = 0.35f;
    double loop_closure_min_age_s          = 20.0;
    double loop_closure_min_path_dist_m    = 40.0;
    float  loop_closure_icp_max_dist       = 3.0f;
    float  loop_closure_icp_fitness_thresh = 2.5f;
    float  loop_closure_max_trans_m        = 20.0f;
    float  loop_closure_max_rot_deg        = 180.0f;
    float  loop_closure_keyframe_dist_m    = 5.0f;

    // UQ: mid-freq soft correspondence weighting and P improvement gate
    float  mid_freq_quality_min_weight   = 0.9f;  // hard-reject below this (default=0.9, original behavior)
    float  mid_freq_quality_weight_power = 0.0f;  // s_w^power applied to H/h; 0.0=no weighting
    double mid_freq_min_p_improvement    = 0.0;   // rollback if pos-trace P reduced < this fraction; 0=disabled
    // UQ: low-freq soft correspondence weighting (composes with temporal decay)
    float  low_freq_quality_min_weight   = 0.9f;  // default=0.9 (original hard-reject threshold)
    float  low_freq_quality_weight_power = 0.0f;  // default=0.0 (no weighting, s_w^0=1.0)

    // Fast-tier undistortion correction: apply interpolated δpos to undistorted points
    bool  fast_undistort_enable  = false;
    float fast_undistort_min_eig = 5.0f;  // min HtH eigenvalue to trust the δpos correction

    // v18: quality-gated tentative iEKF at fast-tier (replaces decoupled GN).
    // Runs a full iEKF update at 100Hz; commits to kf_ only when quality gate passes.
    // Falls back to v3 behavior (pure IMU) when gate fails → 0 divergences preserved.
    bool   fast_eskf_enable                = false;
    int    fast_eskf_max_iter              = 2;     // max iEKF iterations (keep fast)
    int    fast_eskf_min_correspondences   = 50;    // gate: minimum valid plane correspondences
    double fast_eskf_max_residual          = 0.05;  // gate: max residual RMSE (metres)
    double fast_eskf_min_speed_m_s         = 0.0;   // speed gate: skip fast-tier if vel < threshold (m/s), 0=off
    double fast_eskf_min_correction_m      = 0.0;   // correction gate: revert if |Δpos| < threshold (m), 0=off

    // v19: asynchronous 100Hz dual-resolution fusion.
    // fast_eskf_enable branch is replaced by async_100hz_enable.
    // Coarse map (fast_coarse_voxel_size) is used at 100Hz; low-tier writes both maps.
    bool   async_100hz_enable          = false;
    double fast_coarse_voxel_size      = 0.30;   // voxel size for coarse map (m)
    bool   low_freq_map_write_enable   = false;  // low-tier writes fine + coarse maps
    double fast_max_correction_m       = 0.10;   // v19v2: reject coarse commit if |Δpos| > this (m)
    // Motion + quality dual gate for async coarse iEKF.
    // fast_gate_omega_thresh: minimum omega_ema_ to activate scan-to-map at 100Hz.
    // 0 = always activate (backward-compatible). Husky indoor ≈ 0.1 rad/s, so 0.5 never fires there.
    double fast_gate_omega_thresh        = 0.0;   // rad/s; 0 = always activate
    double fast_gate_min_quality         = 0.0;   // min last_quality_ to accept commit
    double fast_gate_max_rmse            = 1e9;   // max last_residual_rmse_ to accept commit (m)
    int    fast_gate_min_correspondences = 0;     // min last_effct_ to accept commit
    // Joint multi-resolution fusion: mid/low-tier fuses both coarse and fine map correspondences.
    // Coarse rows are scaled by 1/joint_coarse_noise_scale (default 3 = 0.3m/0.1m voxel ratio).
    bool   joint_coarse_enable         = true;
    double joint_coarse_noise_scale    = 3.0;
    // Adaptive scheduling: trigger mid/low tier on motion thresholds instead of fixed time.
    // Slow motion → longer accumulation windows → better plane normals → lower ATE.
    bool   mid_adaptive_enable           = false;
    double mid_adaptive_trigger_pos_m    = 0.05;   // trigger mid when pos delta >= 5 cm
    double mid_adaptive_trigger_rot_deg  = 3.0;    // trigger mid when rot delta >= 3°
    double mid_adaptive_max_interval_ms  = 300.0;  // hard cap: always trigger by 300 ms
    bool   low_adaptive_enable           = false;
    double low_adaptive_trigger_pos_m    = 0.10;   // trigger low when pos delta >= 10 cm
    double low_adaptive_trigger_rot_deg  = 5.0;
    double low_adaptive_max_interval_ms  = 800.0;  // hard cap: always trigger by 800 ms
    double low_adaptive_min_interval_ms  = 100.0;  // never trigger low faster than this

    // Sliding-window factor graph: retain K keyframes and jointly optimize
    // their 6-DOF poses via Gauss-Newton.  Contributions:
    //   - scan-to-map residuals captured at iEKF convergence (each mid/low trigger)
    //   - IMU relative-motion constraints (from predict_state) between KFs
    // The refined pose of the latest KF overwrites kf_ after each mid/low update.
    bool   graph_enable          = false;
    int    graph_window_size     = 4;    // K keyframes in window
    int    graph_max_iter        = 5;    // GN iterations per trigger
    double graph_imu_sigma_pos   = 0.05; // preintegration position noise (m)
    double graph_imu_sigma_rot   = 0.10; // preintegration rotation noise (rad)
    // Maximum pose change the graph is allowed to apply to kf_ after optimization.
    // Protects against degenerate window histories that would push state far from iEKF.
    double graph_max_correction_m     = 0.10; // max |Δpos| accepted from graph
    double graph_max_correction_deg   = 5.0;  // max |Δrot| accepted from graph

    // Fast-tier plane-cache GN: re-use mid-tier correspondences at 100 Hz.
    // Skips ikd-tree search; evaluates cached plane residuals with current IMU pose.
    bool  fast_plane_cache_enable  = false;
    float fast_plane_cache_min_eig = 5.0f;    // min HtH eigenvalue to accept correction
    int    fast_plane_cache_min_points   = 10;    // hard min: skip if fewer planes pass freshness gate
    double fast_plane_cache_rot_decay    = 2.0;   // λ_θ: trust = exp(-λ_θ * Δθ) — decay per radian
    double fast_plane_cache_trans_decay  = 0.5;   // λ_v: trust = exp(-λ_v * Δv) — decay per m/s
    int    fast_plane_cache_count_full   = 50;    // N_full: plane count for trust_count = 1
    double fast_plane_cache_min_trust    = 0.05;  // skip correction if combined trust < this
    // When false: compute correction on a copy and save delta to fast_delta_cache_ for
    // mid-tier undistortion improvement — published fast-tier pose stays pure IMU (ATE preserved).
    // When true (default): apply correction to published 100Hz pose (v6/v8 behavior, better RPE).
    bool  fast_plane_cache_publish = true;
    // v11: motion-coupled adaptive eigenvalue weighting.
    // eig_power is the MAX power applied at high motion (trust→0); collapses to v8 at low motion.
    double fast_plane_cache_eig_power = 2.0;
    // hard-zero DOFs with λ_i < threshold × λ_max (structural degeneracy gate)
    double fast_plane_cache_eig_threshold = 0.15;
    // v10: LiDAR-based early mid-tier trigger — fire mid-tier when planes become stale.
    double fast_plane_cache_stale_trigger          = 0.5;   // stale fraction threshold
    double fast_plane_cache_stale_min_interval_ms  = 25.0;  // minimum gap between LiDAR triggers

    // Voxel-hash fast-tier correction: replaces the ikd-tree decoupled-3x3-GN search
    // (drone/handheld) and gives fast tier real correction where fast_map_update_enable
    // is false (robot/husky/rtk/forest/unitree). Default false = today's behavior for
    // both families, unchanged. When true, this path runs INSTEAD of the
    // fast_map_update_enable branch in fast_freq_update, regardless of that flag's value.
    bool   voxel_hash_enable    = false;
    double voxel_hash_cell_size = 0.2;   // world-frame voxel edge length, meters

    // Adaptive Multi-Resolution (AMR) per-tier frequency ranges.
    // Defaults collapse to fixed windows — no adaptation without explicit config range.
    double W_f_min_ms = 10.0;           // fast-tier lower bound (ms); shorter → fires faster
    double W_f_max_ms = 10.0;           // fast-tier upper bound
    double W_m_min_ms = 100.0;          // mid-tier lower bound
    double W_m_max_ms = 100.0;          // mid-tier upper bound
    double fast_quality_bad  = 0.0;     // coarse quality at/below → fire at W_f_min
    double fast_quality_good = 1.0;     // coarse quality at/above → fire at W_f_max
    // When false (default): mid/low-tier init ignores accumulated coarse corrections.
    bool   fast_state_seed_mid_enable = false;
    // Fast-tier degeneracy → early mid-tier pre-emption (only meaningful with async_100hz_enable).
    bool   fast_degen_early_mid_enable = false;
    double fast_degen_early_mid_thresh = 5.0;  // min H^T*H eigenvalue below which mid pre-empts
    // Dual angular-velocity policy parameters.
    double omega_ema_alpha = 0.9;   // EMA decay at 100Hz (~100ms smoothing window)
    double W_m_omega_lo = 0.3;      // rad/s: below this omega has no effect on W_m
    double W_m_omega_hi = 2.5;      // rad/s: above this → W_m_min (fire faster)
    double W_m_alpha    = 1.0;      // blend weight for RMSE vs omega in W_m (1.0=pure RMSE)
    double W_l_omega_lo = 0.5;      // rad/s: below this omega has no effect on W_l
    double W_l_omega_hi = 2.5;      // rad/s: above this → W_l_max (protect from fast motion)
    double W_l_cov_growth_bad = 3.0; // cov growth factor at which W_l → W_l_min (e.g. 3.0 = 3x growth)
    double W_l_smooth_alpha   = 0.0; // temporal smoothing on W_l (0=instant, 0.5=half-life ≈ 1 update)
    // Cross-tier backward propagation + IMU preintegration consistency trigger.
    double W_l_imu_consistency_thresh = 0.0; // m; early W_l trigger on IMU-vs-scan drift (0=disabled)
    bool   backward_correction_enable = false; // retroactively correct mid-tier history after each low-tier
};

// Callback types for publishing from the ROS2 node
using OdomCallback = std::function<void(const state_ikfom&, double /*timestamp*/)>;
using FreqCallback = std::function<void(double W_h_ms, double W_l_ms)>;

class MfLioEstimator {
public:
    explicit MfLioEstimator(const MfLioParams& params);
    ~MfLioEstimator();

    void push_imu(const sensor_msgs::msg::Imu& msg);
    void push_livox_points(double t_begin,
                           const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud);

    void set_odom_callback(OdomCallback cb)           { odom_cb_ = cb; }
    void set_odom_fast_callback(OdomCallback cb)      { odom_fast_cb_ = cb; }
    void set_odom_corrected_callback(OdomCallback cb) { odom_corrected_cb_ = cb; }
    void set_odom_smooth_callback(OdomCallback cb)    { odom_smooth_cb_ = cb; }
    void set_freq_callback(FreqCallback cb)           { freq_cb_ = cb; }

    void start();
    void stop();

    const AdaptivePolicy& policy() const { return policy_; }

private:
    void estimation_loop();
    void fast_freq_update(double t_now);
    void mid_freq_update(double t_now);
    void low_freq_update(double t_now);
    bool can_publish(double t_now) const;
    bool is_stable_publish_update() const;
    void reset_update_diagnostics();
    bool should_publish_corrected(const state_ikfom& pre_update,
                                  const state_ikfom& corrected,
                                  double t_now);
    bool should_accept_update_motion(const state_ikfom& pre_update,
                                     const state_ikfom& corrected,
                                     double dt) const;
    void publish_corrected_if_safe(const state_ikfom& pre_update,
                                   const state_ikfom& corrected,
                                   double t_now);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr
        collect_and_undistort(double t_start, double t_end);

    // iEKF measurement model registered at init time via lambda capture
    void h_share_model(state_ikfom& s,
                       esekfom::dyn_share_datastruct<double>& ekfom_data);

    // RIEKF ablation's own measurement model + correction driver -- mirrors
    // h_share_model's per-point NN-search/plane-fit exactly, INCLUDING the
    // two-tree joint-coarse-map stacking (ikdtree_ then coarse_ikdtree_ when
    // joint_coarse_enable), quality-weight scaling, and temporal decay --
    // but builds H in the InEKF's 24-col body-frame layout via
    // riekf_convert_h_row instead of IKFoM's 12-col world-frame h_x, and
    // drives InEKFFilter::update_state_only/update_covariance directly
    // rather than going through kf_'s own IKFoM driver.
    bool h_share_model_riekf(bool find_neighbors, Eigen::MatrixXd& H, Eigen::VectorXd& z);
    // Iterate-until-converged loop (mirrors h_share_model's own iteration
    // count via num_max_iterations) + final covariance update + write-back
    // into kf_ via change_x/change_P. Called from mid_freq_update/
    // low_freq_update in place of kf_.update_iterated_dyn_share_modified()
    // when params_.riekf_enable.
    void run_riekf_correction();
    // Re-seeds riekf_filter_ (imu_proc_'s InEKFFilter) from state_point_/
    // kf_.get_P() -- called after any existing code mutates kf_'s state
    // directly (v48 degeneracy projection, motion-acceptance rollback,
    // mid_freq_min_p_improvement rollback, IGR blend) so riekf_filter_ never
    // goes stale relative to kf_'s authoritative state.
    void sync_riekf_from_kf();

    // Sliding-window graph: add KF, run GN, write refined pose back.
    bool run_graph_update(double t_now);

    MfLioParams params_;

    RawPointBuffer   point_buf_;
    FeatureExtractor extractor_;
    AdaptivePolicy   policy_;
    ImuProcessor     imu_proc_;

    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    KD_TREE<PointType> ikdtree_;          // fine map (0.1m), written by low-tier
    KD_TREE<PointType> coarse_ikdtree_;   // coarse map (0.3m), written by 100Hz + low-tier

    double last_fast_freq_t_    = 0.0;
    double last_mid_freq_t_     = 0.0;
    double last_low_freq_t_     = 0.0;
    double last_fast_map_gn_t_  = 0.0;  // rate-limits fast-map GN to avoid thread overload
    double last_fast_commit_t_  = 0.0;  // v18: last time fast-tier iEKF committed (pub_anchor)
    // v19v4 dual-state: fast tracking state, separate from kf_ (fine localization state)
    state_ikfom fast_state_;           // coarse-resolution tracking state (100Hz)
    state_ikfom prev_kf_imu_;          // kf_.get_x() before last fast-tier iEKF (delta reference)
    bool        fast_state_valid_ = false;
    // §4 anchor-relative fast correction: accumulated GN position delta since last mid-tier.
    // Reset to zero at each mid_freq_update; never written to kf_.
    Eigen::Vector3d fast_delta_xi_pos_ = Eigen::Vector3d::Zero();
    // §5 anchor-relative rotation correction (only active when fast_6dof_enable=true).
    // Accumulated body-frame rotation delta (Lie algebra) since last mid-tier.
    Eigen::Vector3d fast_delta_xi_rot_ = Eigen::Vector3d::Zero();
    double init_time_        = 0.0;
    bool   initialized_      = false;
    bool   first_update_done_ = false;  // true after first iEKF correction — gates publication
    bool   stable_update_done_ = false; // true after first stable LiDAR correction
    bool   last_corrected_publish_valid_ = false;
    state_ikfom last_corrected_publish_state_;
    double last_corrected_publish_t_ = 0.0;

    // Workspace set before each kf_.update_iterated_dyn_share_modified() call
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr meas_cloud_;
    std::vector<PointVector> nearest_points_;
    std::vector<PointVector> coarse_nearest_points_;  // v19: NN buffer for coarse iEKF
    int meas_size_{0};
    bool in_low_freq_   = false;  // TCW flag: active during low-tier kf_.update call
    bool in_async_freq_ = false;  // v19: active during 100Hz async iEKF on coarse map
    int  last_effct_  = 0;      // valid correspondence count from last h_share_model call
    double last_residual_rmse_ = 0.0;
    double last_quality_ = 0.0;
    double current_W_f_ms_ = 10.0;   // adaptive fast-tier firing interval (ms)
    double current_W_m_ms_ = 100.0;  // adaptive mid-tier firing interval (ms)
    double current_W_l_ms_ = 200.0;  // adaptive low-tier firing interval (ms)
    double last_scalar_alpha_l_ = 0.5;  // IGR from last low-tier update (0.5 = neutral)
    double omega_ema_ = 0.0;  // 100Hz EMA of IMU angular velocity magnitude (rad/s)
    double last_low_pos_cov_trace_ = 1e-2;  // position cov trace at last low-tier update
    int  extrinsic_high_excitation_count_ = 0;
    bool extrinsic_permissive_latched_ = false;

    // Option C: mid-freq posterior covariance for gating low-freq influence
    double last_mid_pos_cov_trace_ = 1e-2;  // initialized to "uncertain"

    // v48 degeneracy guard: accumulated position Hessian from the final iEKF iteration
    Eigen::Matrix3d last_pos_HtH_       = Eigen::Matrix3d::Zero();
    bool            last_pos_hth_valid_ = false;
    int             h_share_call_count_ = 0;

    // IGR: sum of last_pos_HtH_ across all mid-tier updates since last low-tier update.
    // Reset at the start of each low_freq_update(); used to compute alpha_l.
    Eigen::Matrix3d H_m_accumulated_ = Eigen::Matrix3d::Zero();

    // Fast-tier δpos cache: position corrections from fast-tier GN vs pure IMU integration.
    // Keyed by timestamp; used to post-hoc correct undistorted points before mid-tier iEKF.
    struct FastDeltaEntry { double t; Eigen::Vector3d delta_pos; };
    std::deque<FastDeltaEntry> fast_delta_cache_;
    std::mutex                 fast_delta_mutex_;

    // Cross-tier backward propagation: ring buffer of mid-tier published poses.
    // After each low-tier update, its SE3 delta is retroactively applied to all entries
    // in the current W_l window and republished on odom_smooth_cb_.
    struct PoseEntry { double t; state_ikfom state; };
    std::deque<PoseEntry> pose_history_;

    // Fast-tier plane cache: mid-tier correspondences re-used at 100 Hz without ikd-tree search.
    // Populated at mid-tier iEKF convergence; each entry holds the body-frame point,
    // world-frame plane normal, and plane offset d (n·pw + d ≈ 0 at convergence time).
    struct CachedPlane {
        Eigen::Vector3f p_body;  // body-frame point from mid-tier scan
        Eigen::Vector3d n;       // unit plane normal (world frame at mid convergence)
        double          d;       // plane offset: n·pw_converged + d ≈ 0
    };
    std::vector<CachedPlane> plane_cache_;
    std::mutex               plane_cache_mutex_;
    bool                     plane_cache_valid_ = false;

    // Voxel-hash plane map: accumulates world-frame planes across many mid-tier
    // cycles (unlike plane_cache_'s single-cycle snapshot). Queried by the fast
    // tier every ~10ms against FRESH points, giving it a genuine, cheap,
    // continuously-updated correspondence source instead of ikd-tree search.
    VoxelPlaneHash voxel_hash_;
    std::mutex     voxel_hash_mutex_;

    // State snapshots for motion-based adaptive scheduling
    state_ikfom last_mid_freq_state_;
    bool        last_mid_freq_state_valid_ = false;
    state_ikfom last_low_freq_state_;
    bool        last_low_freq_state_valid_ = false;

    // Sliding-window factor graph
    SlidingWindowOptimizer graph_opt_;
    bool        graph_initialized_    = false;
    state_ikfom last_graph_kf_x_;   // state at last keyframe insertion
    double      last_graph_kf_t_    = 0.0;

    // Capture mode: when true h_share_model saves H/r at convergence without
    // disrupting the iEKF update.
    bool            capture_h_r_on_converge_ = false;
    Eigen::MatrixXd captured_H_;   // ekfom_data.h_x at convergence (M×12)
    Eigen::VectorXd captured_r_;   // ekfom_data.h  at convergence (M)

    // v10: fast-tier LiDAR scheduling signal
    double last_fast_stale_fraction_ = 0.0;  // set by apply_plane_cache_gn each fast-tier tick
    bool   lidar_early_mid_request_  = false; // set when stale fraction exceeds threshold

    std::atomic<bool>  running_{false};
    std::thread        est_thread_;

    // Loop closure
    ScanContextManager      sc_manager_;
    std::mutex              lc_mutex_;
    std::condition_variable lc_cv_;
    std::thread             lc_thread_;
    double                  lc_path_dist_    = 0.0;
    V3D                     lc_last_kf_pos_  = V3D::Zero();
    bool                    lc_kf_pos_valid_ = false;
    bool                    lc_has_correction_ = false;
    Eigen::Matrix4f         lc_correction_;

    // Smooth-correction log for post-hoc TUM file generation.
    // kf_originals_: iEKF keyframe pose at capture time, never modified.
    // lc_corr_log_: each LC event records (cand_idx, query_idx, T_corr).
    // At shutdown lc_loop() applies a linear ramp of each T_corr from
    // cand_idx to query_idx and writes the smooth trajectory to /tmp/v51_lc_kf.txt.
    struct OrigKF {
        double          timestamp;
        Eigen::Vector3d pos;
        Eigen::Quaterniond rot;
    };
    struct LcCorrEvent {
        int             cand_idx;
        int             query_idx;
        Eigen::Matrix4f T_corr;
    };
    std::vector<OrigKF>        kf_originals_;   // guarded by lc_mutex_
    std::vector<LcCorrEvent>   lc_corr_log_;    // guarded by lc_mutex_

    void lc_loop();
    void apply_loop_correction(const Eigen::Matrix4f& T_corr_f);
    void apply_fast_undistort(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double t_start);
    void apply_plane_cache_gn(state_ikfom& predicted, double dtheta, double dv);
    void apply_voxel_hash_gn(state_ikfom& predicted, double t_now, double dtheta, double dv);
    PointCloudXYZI::Ptr downsample_for_lc(const PointCloudXYZI::Ptr& in);

    OdomCallback odom_cb_;
    OdomCallback odom_fast_cb_;
    OdomCallback odom_corrected_cb_;
    OdomCallback odom_smooth_cb_;
    FreqCallback freq_cb_;

    pcl::VoxelGrid<pcl::PointXYZINormal> vox_filter_;
    pcl::VoxelGrid<pcl::PointXYZINormal> vox_fast_filter_;

    // Diagnostic-only: opt-in per-cycle jitter log, gated by MFLIO_JITTER_LOG env var.
    // No effect on estimation when unset (jitter_log_enabled_ stays false).
    bool jitter_log_enabled_ = false;
    std::ofstream jitter_log_;
    long jitter_mid_call_idx_ = 0;
    long jitter_low_call_idx_ = 0;
    long jitter_h_call_idx_ = 0;
    void jitter_log_cycle(const char* tier, long call_idx, double t_now, double t_start,
                          int raw_scan_size, int ds_scan_size);
    void jitter_log_h_share(const char* tier, long call_idx, int n, int point_valid_count,
                            double mean_quality_weight, bool converge);

    static constexpr double LASER_POINT_COV = 0.001;
};
