#include "mf_lio/mf_lio_estimator.h"
#include "mf_lio/riekf/h_convert.hpp"
#include "mf_lio/rdef_ci_diagnostic.h"
#include "mf_lio/adaptive_window_policy.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <thread>
#include <omp.h>


MfLioEstimator::MfLioEstimator(const MfLioParams& p) : params_(p) {
    extractor_.edge_thresh  = p.edge_thresh;
    extractor_.plane_thresh = p.plane_thresh;

    // W_l is adaptive; W_f and W_m are fixed — policy only adapts W_l
    // W_m adapts with motion (alpha); W_l adapts with degeneracy (beta)
    policy_.W_h_base = p.W_m_ms;      // base = nominal mid-tier cadence
    policy_.W_h_min  = 20.0;           // floor: 50Hz (as fast as the fast tier)
    policy_.W_h_max  = p.W_m_ms;      // ceiling: never slower than the nominal cadence
    policy_.W_l_base = p.W_l_base_ms; policy_.W_l_min = p.W_l_min_ms;
    policy_.W_l_max  = p.W_l_max_ms;
    policy_.alpha    = p.alpha;        // motion-adaptive mid-tier window
    policy_.beta     = p.beta;
    policy_.reset_to_base();
    current_W_f_ms_ = p.W_f_min_ms;  // start at fast end; adapts after first quality signal
    current_W_m_ms_ = p.W_m_ms;
    current_W_l_ms_ = p.W_l_base_ms;

    vox_filter_.setLeafSize(
        static_cast<float>(p.filter_size_map),
        static_cast<float>(p.filter_size_map),
        static_cast<float>(p.filter_size_map));
    // vox_fast_filter_ is repurposed as the scan downsampler (separate from map resolution).
    // filter_size_scan=0 falls back to filter_size_map for backward compatibility.
    const float scan_leaf = static_cast<float>(
        p.filter_size_scan > 0.0 ? p.filter_size_scan : p.filter_size_map);
    vox_fast_filter_.setLeafSize(scan_leaf, scan_leaf, scan_leaf);

    imu_proc_.set_extrinsic(
        p.extrinsic_T, p.extrinsic_R,
        p.extrinsic_est_en, p.state_extrinsic_undistort,
        p.legacy_mixed_extrinsic_undistort,
        p.extrinsic_rot_cov, p.extrinsic_trans_cov);
    imu_proc_.set_cov(p.gyr_cov, p.acc_cov, p.b_gyr_cov, p.b_acc_cov);
    imu_proc_.set_init_vel_cov(p.init_vel_cov);
    imu_proc_.set_riekf_enable(p.riekf_enable);
    voxel_hash_.set_cell_size(p.voxel_hash_cell_size);

    // Sliding-window graph: reinitialise with configured window size.
    graph_opt_ = SlidingWindowOptimizer(p.graph_window_size);

    double epsi[23];
    std::fill(epsi, epsi + 23, 1e-3);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
            h_share_model(s, ekfom_data);
        },
        p.num_max_iterations, epsi);

    // Diagnostic-only jitter log: fully inert unless MFLIO_JITTER_LOG is set.
    if (const char* jitter_path = std::getenv("MFLIO_JITTER_LOG")) {
        jitter_log_.open(jitter_path, std::ios::out | std::ios::trunc);
        if (jitter_log_.is_open()) {
            jitter_log_enabled_ = true;
            jitter_log_ << "kind,tier,call_idx,t_now,t_start,window_ms,raw_scan_size,ds_scan_size,"
                           "n,point_valid_count,mean_quality_weight,converge\n";
        }
    }

    // Diagnostic-only low-tier accept/reject log: fully inert unless MFLIO_LOWTIER_LOG is set.
    if (const char* lowtier_path = std::getenv("MFLIO_LOWTIER_LOG")) {
        lowtier_log_.open(lowtier_path, std::ios::out | std::ios::trunc);
        if (lowtier_log_.is_open()) {
            lowtier_log_enabled_ = true;
            lowtier_log_ << "t_now,accepted,reject_reason,scalar_alpha_l,effct,quality,"
                            "residual_rmse,pos_delta,rot_delta_deg,eig1,eig2,eig3,"
                            "alpha1,alpha2,alpha3,corr_raw_norm,corr_applied_norm,"
                            "corr_applied_wx,corr_applied_wy,corr_applied_wz,"
                            "corr_applied_bx,corr_applied_by,corr_applied_bz,"
                            "hm_eig1,hm_eig2,hm_eig3\n";
        }
    }
}

MfLioEstimator::~MfLioEstimator() { stop(); }

void MfLioEstimator::jitter_log_cycle(const char* tier, long call_idx, double t_now,
                                      double t_start, int raw_scan_size, int ds_scan_size) {
    if (!jitter_log_enabled_) return;
    jitter_log_ << "cycle," << tier << ',' << call_idx << ','
                << std::setprecision(17) << t_now << ',' << t_start << ','
                << (t_now - t_start) * 1e3 << ',' << raw_scan_size << ',' << ds_scan_size
                << ",,,,\n";
}

void MfLioEstimator::jitter_log_h_share(const char* tier, long call_idx, int n,
                                        int point_valid_count, double mean_quality_weight,
                                        bool converge) {
    if (!jitter_log_enabled_) return;
    jitter_log_ << "h_share," << tier << ',' << call_idx << ",,,,,,"
                << n << ',' << point_valid_count << ','
                << std::setprecision(9) << mean_quality_weight << ','
                << (converge ? 1 : 0) << '\n';
}

// ---------------------------------------------------------------------------
void MfLioEstimator::start() {
    running_    = true;
    est_thread_ = std::thread(&MfLioEstimator::estimation_loop, this);
    if (params_.loop_closure_enable)
        lc_thread_ = std::thread(&MfLioEstimator::lc_loop, this);
}

void MfLioEstimator::stop() {
    running_ = false;
    lc_cv_.notify_all();
    if (lc_thread_.joinable()) lc_thread_.join();
    if (est_thread_.joinable())  est_thread_.join();
}

void MfLioEstimator::push_imu(const sensor_msgs::msg::Imu& msg) {
    imu_proc_.push_imu(msg);
}

void MfLioEstimator::push_livox_points(
    double t_begin,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud)
{
    if (!cloud) return;
    // Build the whole scan locally, then insert it into point_buf_ in one
    // locked batch (push_batch). The old per-point push() let a concurrent
    // reader on the estimation-loop thread observe point_buf_ mid-scan --
    // latest_time()/get_window() could return a state where only some of a
    // scan's points had been pushed yet, and exactly how many depended on
    // real-time scheduling between this callback thread and the polling
    // estimation thread. That race, not anything about which clock a
    // timestamp is drawn from (t_begin/pt.curvature are both bag-deterministic
    // already), was the actual source of the run-to-run point-set divergence.
    std::vector<StampedPoint> batch;
    batch.reserve(cloud->points.size());
    for (const auto& pt : cloud->points) {
        double t = t_begin + pt.curvature * 1e-3;
        batch.push_back({t, pt});
    }
    point_buf_.push_batch(batch);
}

// ---------------------------------------------------------------------------
void MfLioEstimator::estimation_loop() {
    while (running_) {
        if (point_buf_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        double t_now = point_buf_.latest_time();

        if (!initialized_) {
            // Fire mid_freq immediately on the next iteration (t_now - W_m triggers it).
            // Without this, the first iEKF is delayed by one extra 100ms window, giving
            // 1+ second of unconstrained IMU propagation. That grows P[vel,pos] by 10×,
            // causing the first scan's Kalman gain to apply a large wrong velocity correction.
            last_mid_freq_t_  = t_now - params_.W_m_ms * 1e-3;
            last_fast_freq_t_ = last_mid_freq_t_;
            last_low_freq_t_  = t_now;
            shared_origin_t_  = t_now;
            init_time_        = t_now;
            initialized_      = true;
            continue;
        }

        // Only one tier per loop iteration to prevent overlapping iEKF updates.
        // After mid/low fires, reset fast_freq timer to prevent same-scan double correction.

        // Low-tier trigger: adaptive (motion-based) or fixed time.
        bool low_trigger;
        if (params_.low_adaptive_enable && last_low_freq_state_valid_) {
            const double elapsed = t_now - last_low_freq_t_;
            if (elapsed < params_.low_adaptive_min_interval_ms * 1e-3) {
                low_trigger = false;
            } else if (elapsed >= params_.low_adaptive_max_interval_ms * 1e-3) {
                low_trigger = true;  // hard cap
            } else {
                const double dp =
                    (state_point_.pos - last_low_freq_state_.pos).norm();
                const double dr =
                    Eigen::AngleAxisd(last_low_freq_state_.rot.conjugate() *
                                      state_point_.rot).angle() * 180.0 / M_PI;
                low_trigger = (dp >= params_.low_adaptive_trigger_pos_m ||
                               dr >= params_.low_adaptive_trigger_rot_deg);
            }
        } else {
            low_trigger = (t_now - last_low_freq_t_ >= current_W_l_ms_ * 1e-3);
        }

        // IMU-preintegration consistency check: fire low-tier early if IMU-propagated
        // position has diverged from scan-to-map state since the last low-tier update.
        // Detects IMU drift accumulation that RMSE and omega signals miss.
        if (!low_trigger && last_low_freq_state_valid_
                && params_.W_l_imu_consistency_thresh > 0.0) {
            const state_ikfom imu_pred =
                imu_proc_.predict_state(last_low_freq_t_, t_now, last_low_freq_state_);
            if ((imu_pred.pos - state_point_.pos).norm()
                    > params_.W_l_imu_consistency_thresh)
                low_trigger = true;
        }

        // Degeneracy-gated firing: defer this cycle (no attempt at all) if mid-tier's
        // own accumulated info already covers every direction -- H_m_accumulated_ is
        // left untouched so it keeps building toward the next check. Falls through to
        // the mid-tier trigger below exactly as if low_trigger had been false.
        if (params_.low_freq_degen_gate_enable && low_trigger &&
            (t_now - last_low_freq_t_) <
                params_.low_freq_degen_gate_max_interval_ms * 1e-3) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_gate(H_m_accumulated_);
            if (eig_gate.eigenvalues().minCoeff() >= params_.low_freq_degen_gate_min_eig) {
                low_trigger = false;
            }
        }

        if (low_trigger) {
            low_freq_update(t_now);
            last_low_freq_t_    = t_now;
            if (params_.overlap_bound_steps > 0) {
                // Bounded overlap: allow mid-tier to reuse at most N window-depths
                // of data just consumed by the low tier, instead of a full reset.
                last_mid_freq_t_  = t_now - params_.overlap_bound_steps * current_W_m_ms_ * 1e-3;
                last_fast_freq_t_ = t_now;
            } else if (params_.disjoint_scheduling_enable) {
                if (params_.low_reset_tm_enable) {
                    last_mid_freq_t_    = t_now;  // prevent immediate mid_freq follow-up
                }
                last_fast_freq_t_   = t_now;
            }
            if (params_.shared_origin_scheduling_enable) {
                // Persistent shared origin resets only when low tier fires; mid-tier
                // windows grow from this origin until the next low-tier reset.
                shared_origin_t_ = t_now;
            }
            last_fast_commit_t_ = 0.0;  // reset: low/mid-tier is now the anchor
            continue;
        }

        // Option F: motion-triggered mid-freq — fire early if significant rotation
        // or translation has accumulated since the last mid-freq update.
        bool motion_trigger = false;
        if (params_.mid_freq_motion_trigger && last_mid_freq_state_valid_) {
            const double elapsed = t_now - last_mid_freq_t_;
            if (elapsed >= params_.mid_freq_min_interval_ms * 1e-3) {
                const double acc_rot_deg =
                    Eigen::AngleAxisd(last_mid_freq_state_.rot.conjugate() *
                                      state_point_.rot).angle() * 180.0 / M_PI;
                const double acc_trans =
                    (state_point_.pos - last_mid_freq_state_.pos).norm();
                motion_trigger = (acc_rot_deg >= params_.mid_freq_trigger_rot_deg ||
                                  acc_trans   >= params_.mid_freq_trigger_trans_m);
            }
        }

        // Mid-tier trigger: adaptive (motion + hard cap) or fixed time.
        bool mid_time_trigger;
        if (params_.mid_adaptive_enable && last_mid_freq_state_valid_) {
            const double elapsed = t_now - last_mid_freq_t_;
            if (elapsed < params_.mid_freq_min_interval_ms * 1e-3) {
                mid_time_trigger = false;
            } else if (elapsed >= params_.mid_adaptive_max_interval_ms * 1e-3) {
                mid_time_trigger = true;  // hard cap
            } else {
                const double dp =
                    (state_point_.pos - last_mid_freq_state_.pos).norm();
                const double dr =
                    Eigen::AngleAxisd(last_mid_freq_state_.rot.conjugate() *
                                      state_point_.rot).angle() * 180.0 / M_PI;
                mid_time_trigger = (dp >= params_.mid_adaptive_trigger_pos_m ||
                                    dr >= params_.mid_adaptive_trigger_rot_deg);
            }
        } else {
            mid_time_trigger = (t_now - last_mid_freq_t_ >= current_W_m_ms_ * 1e-3);
        }

        // v10: LiDAR-based early mid-tier trigger (set by fast_freq_update via stale fraction)
        bool lidar_trigger = lidar_early_mid_request_;
        lidar_early_mid_request_ = false;
        if (mid_time_trigger || motion_trigger || lidar_trigger) {
            mid_freq_update(t_now);
            last_mid_freq_t_    = t_now;
            if (params_.disjoint_scheduling_enable) {
                last_fast_freq_t_   = t_now;
            }
            last_fast_commit_t_ = 0.0;  // reset: mid-tier is now the anchor
            continue;
        }

        if (params_.fast_tier_enable &&
            t_now - last_fast_freq_t_ >= current_W_f_ms_ * 1e-3) {
            // Catch up one window at a time instead of jumping straight to
            // t_now (= point_buf_.latest_time()). Under bursty transport
            // delivery, two or more packet-interval's worth of points can
            // land in point_buf_ before this poll iteration runs, so
            // t_now - last_fast_freq_t_ can exceed current_W_f_ms_ by more
            // than one window. Firing a single fast_freq_update(t_now) in
            // that case silently coalesces the whole backlog into one
            // oversized window and advances last_fast_freq_t_ straight past
            // the skipped window boundary -- consuming the "missing" fire
            // and roughly halving the achieved fast-tier rate under bursty
            // delivery (measured ~51 Hz instead of the ~100 Hz design
            // target). Advancing by exactly one current_W_f_ms_ step per
            // fire, bounded, restores one publish per window.
            int catchup_fires = 0;
            while (t_now - last_fast_freq_t_ >= current_W_f_ms_ * 1e-3 &&
                   catchup_fires < params_.fast_tier_max_catchup_fires) {
                const double t_step = last_fast_freq_t_ + current_W_f_ms_ * 1e-3;
                fast_freq_update(t_step);
                last_fast_freq_t_ = t_step;
                ++catchup_fires;
            }
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// ---------------------------------------------------------------------------
// iEKF measurement model — called by the filter for each iteration.
// Populates h_x (effct×12): pos(0-2), rot(3-5), offset_R_L_I(6-8), offset_T_L_I(9-11).
// Extrinsic Jacobians enable online LiDAR-IMU extrinsic estimation.
// ---------------------------------------------------------------------------
void MfLioEstimator::h_share_model(
    state_ikfom& s,
    esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    // v19: select active map and NN buffer based on current tier
    KD_TREE<PointType>& active_tree = in_async_freq_ ? coarse_ikdtree_ : ikdtree_;
    std::vector<PointVector>& active_nearest = in_async_freq_ ? coarse_nearest_points_ : nearest_points_;

    if (!meas_cloud_ || meas_cloud_->empty() || active_tree.Root_Node == nullptr) {
        ekfom_data.valid = false;
        return;
    }

    const int n = static_cast<int>(meas_cloud_->size());
    if (static_cast<int>(active_nearest.size()) != n)
        active_nearest.resize(n);

    std::vector<bool>   point_valid(n, false);
    std::vector<V3D>    norm_vecs(n);
    std::vector<double> residuals_raw(n, 0.0);
    std::vector<float>  quality_weights(n, 1.0f);

    // Parallel NN search: each iteration is independent (different nearest_points_[i])
    // IKD-tree Nearest_Search is thread-safe for concurrent reads via search_mutex_counter.
    #pragma omp parallel for num_threads(8) schedule(guided)
    for (int i = 0; i < n; ++i) {
        const auto& p_body = meas_cloud_->points[i];

        V3D pb(p_body.x, p_body.y, p_body.z);
        V3D pw = s.rot * (s.offset_R_L_I * pb + s.offset_T_L_I) + s.pos;

        PointType pw_pcl;
        pw_pcl.x = static_cast<float>(pw(0));
        pw_pcl.y = static_cast<float>(pw(1));
        pw_pcl.z = static_cast<float>(pw(2));
        pw_pcl.intensity = p_body.intensity;

        if (ekfom_data.converge) {
            std::vector<float> sq_dist(NUM_MATCH_POINTS);
            active_tree.Nearest_Search(pw_pcl, NUM_MATCH_POINTS, active_nearest[i], sq_dist);
            if (static_cast<int>(active_nearest[i].size()) < NUM_MATCH_POINTS
                || sq_dist.back() > 5.0f)
                continue;
        }
        if (active_nearest[i].size() < NUM_MATCH_POINTS) continue;

        if (in_low_freq_ && params_.low_freq_ndt_enable) {
            // NDT-eigenplane: compute surface normal from covariance of 5 NN.
            // More statistically principled than QR plane fit (esti_plane).
            Eigen::Vector3f mu = Eigen::Vector3f::Zero();
            for (const auto& np : active_nearest[i]) mu += np.getVector3fMap();
            mu /= float(NUM_MATCH_POINTS);

            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto& np : active_nearest[i]) {
                Eigen::Vector3f d = np.getVector3fMap() - mu;
                cov += d * d.transpose();
            }
            cov /= float(NUM_MATCH_POINTS - 1);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
            if (es.info() != Eigen::Success) continue;
            float lambda_min = es.eigenvalues()(0);
            if (lambda_min > float(params_.esti_plane_threshold)) continue;
            Eigen::Vector3f n = es.eigenvectors().col(0);

            float pd = n(0)*(pw_pcl.x - mu(0)) + n(1)*(pw_pcl.y - mu(1)) + n(2)*(pw_pcl.z - mu(2));

            point_valid[i]   = true;
            norm_vecs[i]     = V3D(n(0), n(1), n(2));
            residuals_raw[i] = -static_cast<double>(pd);
        } else {
            Eigen::Matrix<float,4,1> pabcd;
            if (!esti_plane(pabcd, active_nearest[i], static_cast<float>(params_.esti_plane_threshold)))
                continue;

            float pd = pabcd(0)*pw_pcl.x + pabcd(1)*pw_pcl.y
                     + pabcd(2)*pw_pcl.z + pabcd(3);

            float s_w = 1.0f - 0.9f * std::abs(pd) / std::sqrt(static_cast<float>(pb.norm()));
            if (in_low_freq_) {
                if (s_w < params_.low_freq_quality_min_weight) continue;
                quality_weights[i] = std::pow(s_w, params_.low_freq_quality_weight_power);
            } else {
                if (s_w < params_.mid_freq_quality_min_weight) continue;
                quality_weights[i] = std::pow(s_w, params_.mid_freq_quality_weight_power);
            }

            point_valid[i]   = true;
            norm_vecs[i]     = V3D(pabcd(0), pabcd(1), pabcd(2));
            residuals_raw[i] = -static_cast<double>(pd);
        }
    }

    // Joint coarse-map fusion for mid/low tier: append correspondences from coarse
    // map with noise scaled by joint_coarse_noise_scale (1/scale applied to h rows).
    const bool use_coarse_joint = !in_async_freq_
        && params_.joint_coarse_enable
        && coarse_ikdtree_.Root_Node != nullptr;

    std::vector<bool>   coarse_valid(n, false);
    std::vector<V3D>    coarse_norm(n);
    std::vector<double> coarse_residual(n, 0.0);

    if (use_coarse_joint) {
        if (static_cast<int>(coarse_nearest_points_.size()) != n)
            coarse_nearest_points_.resize(n);

        const float coarse_plane_thresh =
            static_cast<float>(params_.esti_plane_threshold * params_.joint_coarse_noise_scale);
        const float q_min = in_low_freq_
            ? static_cast<float>(params_.low_freq_quality_min_weight)
            : static_cast<float>(params_.mid_freq_quality_min_weight);

        #pragma omp parallel for num_threads(8) schedule(guided)
        for (int i = 0; i < n; ++i) {
            const auto& p_body = meas_cloud_->points[i];
            V3D pb(p_body.x, p_body.y, p_body.z);
            V3D pw = s.rot * (s.offset_R_L_I * pb + s.offset_T_L_I) + s.pos;
            PointType pw_pcl;
            pw_pcl.x = static_cast<float>(pw(0));
            pw_pcl.y = static_cast<float>(pw(1));
            pw_pcl.z = static_cast<float>(pw(2));
            pw_pcl.intensity = p_body.intensity;

            if (ekfom_data.converge) {
                std::vector<float> sq_dist_c(NUM_MATCH_POINTS);
                coarse_ikdtree_.Nearest_Search(
                    pw_pcl, NUM_MATCH_POINTS, coarse_nearest_points_[i], sq_dist_c);
                if (static_cast<int>(coarse_nearest_points_[i].size()) < NUM_MATCH_POINTS
                    || sq_dist_c.back() > 5.0f)
                    continue;
            }
            if (static_cast<int>(coarse_nearest_points_[i].size()) < NUM_MATCH_POINTS) continue;

            Eigen::Matrix<float,4,1> pabcd_c;
            if (!esti_plane(pabcd_c, coarse_nearest_points_[i], coarse_plane_thresh))
                continue;

            float pd_c = pabcd_c(0)*pw_pcl.x + pabcd_c(1)*pw_pcl.y
                       + pabcd_c(2)*pw_pcl.z + pabcd_c(3);
            float s_w  = 1.0f - 0.9f * std::abs(pd_c) / std::sqrt(static_cast<float>(pb.norm()));
            if (s_w < q_min) continue;

            coarse_valid[i]    = true;
            coarse_norm[i]     = V3D(pabcd_c(0), pabcd_c(1), pabcd_c(2));
            coarse_residual[i] = -static_cast<double>(pd_c);
        }
    }

    // Prototype low-tier-only map join: same pattern as use_coarse_joint above, but
    // sourced from low_ikdtree_ (independent of mid-tier's writes) and gated to the
    // low-tier call only.
    const bool use_low_own_map = in_low_freq_
        && params_.low_tier_own_map_enable
        && low_ikdtree_.Root_Node != nullptr;

    std::vector<bool>   low_valid(n, false);
    std::vector<V3D>    low_norm(n);
    std::vector<double> low_residual(n, 0.0);

    if (use_low_own_map) {
        if (static_cast<int>(low_nearest_points_.size()) != n)
            low_nearest_points_.resize(n);

        const float low_plane_thresh =
            static_cast<float>(params_.esti_plane_threshold * params_.low_own_map_noise_scale);
        const float q_min = static_cast<float>(params_.low_freq_quality_min_weight);

        #pragma omp parallel for num_threads(8) schedule(guided)
        for (int i = 0; i < n; ++i) {
            const auto& p_body = meas_cloud_->points[i];
            V3D pb(p_body.x, p_body.y, p_body.z);
            V3D pw = s.rot * (s.offset_R_L_I * pb + s.offset_T_L_I) + s.pos;
            PointType pw_pcl;
            pw_pcl.x = static_cast<float>(pw(0));
            pw_pcl.y = static_cast<float>(pw(1));
            pw_pcl.z = static_cast<float>(pw(2));
            pw_pcl.intensity = p_body.intensity;

            if (ekfom_data.converge) {
                std::vector<float> sq_dist_l(NUM_MATCH_POINTS);
                low_ikdtree_.Nearest_Search(
                    pw_pcl, NUM_MATCH_POINTS, low_nearest_points_[i], sq_dist_l);
                if (static_cast<int>(low_nearest_points_[i].size()) < NUM_MATCH_POINTS
                    || sq_dist_l.back() > 5.0f)
                    continue;
            }
            if (static_cast<int>(low_nearest_points_[i].size()) < NUM_MATCH_POINTS) continue;

            Eigen::Matrix<float,4,1> pabcd_l;
            if (!esti_plane(pabcd_l, low_nearest_points_[i], low_plane_thresh))
                continue;

            float pd_l = pabcd_l(0)*pw_pcl.x + pabcd_l(1)*pw_pcl.y
                       + pabcd_l(2)*pw_pcl.z + pabcd_l(3);
            float s_w  = 1.0f - 0.9f * std::abs(pd_l) / std::sqrt(static_cast<float>(pb.norm()));
            if (s_w < q_min) continue;

            low_valid[i]    = true;
            low_norm[i]     = V3D(pabcd_l(0), pabcd_l(1), pabcd_l(2));
            low_residual[i] = -static_cast<double>(pd_l);
        }
    }

    int effct = 0;
    for (int i = 0; i < n; ++i) if (point_valid[i]) ++effct;
    int effct_c = 0;
    if (use_coarse_joint)
        for (int i = 0; i < n; ++i) if (coarse_valid[i]) ++effct_c;
    int effct_low = 0;
    if (use_low_own_map)
        for (int i = 0; i < n; ++i) if (low_valid[i]) ++effct_low;
    if (effct < 1 && effct_c < 1 && effct_low < 1) { ekfom_data.valid = false; return; }

    // Anisotropic own-map trust: compare aggregate information from the own-map's
    // rows against the main map's rows, per eigen-direction, once per call.
    Eigen::Matrix3d low_aniso_V = Eigen::Matrix3d::Identity();
    Eigen::Vector3d low_aniso_beta = Eigen::Vector3d::Zero();
    double low_kappa = 0.0;
    if (use_low_own_map && params_.low_own_map_anisotropic_enable) {
        Eigen::Matrix3d H_own_agg = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d H_main_agg = Eigen::Matrix3d::Zero();
        for (int i = 0; i < n; ++i) {
            if (low_valid[i])   H_own_agg  += low_norm[i]   * low_norm[i].transpose();
            if (point_valid[i]) H_main_agg += norm_vecs[i] * norm_vecs[i].transpose();
        }
        // Attempt #6: commutator diagnostic kappa -- 3x3 position-only stand-in
        // for the idea card's full 6x6 SE(3) information-matrix commutator norm.
        {
            const Eigen::Matrix3d comm = H_own_agg * H_main_agg - H_main_agg * H_own_agg;
            const double denom = H_own_agg.norm() * H_main_agg.norm() + 1e-6;
            low_kappa = comm.norm() / denom;
        }
        if (!params_.low_own_map_kappa_log_path.empty()) {
            std::ofstream kappa_ofs(params_.low_own_map_kappa_log_path, std::ios::app);
            if (kappa_ofs.is_open()) {
                const double t_now_wall = std::chrono::duration<double>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                kappa_ofs << std::setprecision(17) << t_now_wall << ','
                          << std::setprecision(9) << low_kappa << ',' << effct_low << "\n";
            }
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_aniso(H_own_agg + H_main_agg);
        low_aniso_V = eig_aniso.eigenvectors();
        for (int k = 0; k < 3; ++k) {
            const double lam_own  = low_aniso_V.col(k).dot(H_own_agg  * low_aniso_V.col(k));
            const double lam_main = low_aniso_V.col(k).dot(H_main_agg * low_aniso_V.col(k));
            const double d = lam_own + lam_main;
            low_aniso_beta(k) = (d > 1e-12) ? std::clamp(lam_own / d, 0.0, 1.0) : 0.0;
        }
    }

    last_effct_ = effct;
    last_quality_ = (n > 0) ? static_cast<double>(effct) / n : 0.0;
    {
        double residual_sse = 0.0;
        for (int i = 0; i < n; ++i) {
            if (point_valid[i]) residual_sse += residuals_raw[i] * residuals_raw[i];
        }
        last_residual_rmse_ = std::sqrt(residual_sse / std::max(effct, 1));
    }

    const int effct_total = effct + effct_c + effct_low;
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_total, 12);
    ekfom_data.h.resize(effct_total);

    const double angular_velocity = imu_proc_.last_angular_velocity().norm();
    if (params_.extrinsic_update_adaptive_enable &&
        !extrinsic_permissive_latched_) {
        const bool high_observability =
            angular_velocity >=
                params_.extrinsic_update_adaptive_high_angular_velocity &&
            last_quality_ >= params_.extrinsic_update_adaptive_min_quality &&
            last_residual_rmse_ <=
                params_.extrinsic_update_adaptive_max_residual_rmse;
        if (high_observability) {
            ++extrinsic_high_excitation_count_;
            if (extrinsic_high_excitation_count_ >=
                params_.extrinsic_update_adaptive_min_high_count) {
                extrinsic_permissive_latched_ = true;
            }
        }
    }

    const bool allow_extrinsic_update =
        params_.extrinsic_est_en &&
        (extrinsic_permissive_latched_ ||
         !params_.extrinsic_update_gate_enable ||
         (angular_velocity >=
              params_.extrinsic_update_min_angular_velocity &&
          last_quality_ >= params_.extrinsic_update_min_quality &&
          last_residual_rmse_ <= params_.extrinsic_update_max_residual_rmse));

    int row = 0;
    for (int i = 0; i < n; ++i) {
        if (!point_valid[i]) continue;

        const auto& p_body = meas_cloud_->points[i];
        V3D pb(p_body.x, p_body.y, p_body.z);
        V3D p_imu = s.offset_R_L_I * pb + s.offset_T_L_I;

        M3D p_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_imu);
        V3D C = s.rot.conjugate() * norm_vecs[i];  // R^T * n
        V3D A = p_crossmat * C;                     // [p_imu]× * R^T * n

        ekfom_data.h_x.block<1,3>(row, 0) = norm_vecs[i].transpose();  // ∂/∂pos
        ekfom_data.h_x.block<1,3>(row, 3) = A.transpose();              // ∂/∂rot
        if (allow_extrinsic_update) {
            M3D pb_crossmat;
            pb_crossmat << SKEW_SYM_MATRX(pb);
            V3D B = pb_crossmat * s.offset_R_L_I.conjugate() * C;  // ∂/∂offset_R_L_I
            ekfom_data.h_x.block<1,3>(row, 6) = B.transpose();
            ekfom_data.h_x.block<1,3>(row, 9) = C.transpose();      // ∂/∂offset_T_L_I
        }
        ekfom_data.h(row)                  = residuals_raw[i];

        ekfom_data.h(row) *= quality_weights[i];
        ekfom_data.h_x.block<1,12>(row, 0) *= quality_weights[i];
        if (in_low_freq_ && params_.temporal_decay_alpha > 0.0) {
            // Down-weight recent points: they are at the leading edge of robot
            // motion and likely in areas not yet fully mapped (map lags at 10 Hz).
            // t_rel=0 (oldest) gets full weight; t_rel=W_l (newest) gets
            // exp(-alpha/2). This remains tunable through temporal_decay_alpha.
            const double age_frac =
                static_cast<double>(p_body.curvature) / policy_.W_l_ms();
            const double sqrt_w = std::exp(
                -0.5 * params_.temporal_decay_alpha * age_frac);
            ekfom_data.h(row) *= sqrt_w;
            ekfom_data.h_x.block<1,12>(row, 0) *= sqrt_w;
        }

        ++row;
    }

    // Coarse map rows: same Jacobian structure but scaled by 1/noise_scale.
    if (use_coarse_joint) {
        const double coarse_scale = 1.0 / params_.joint_coarse_noise_scale;
        for (int i = 0; i < n; ++i) {
            if (!coarse_valid[i]) continue;
            const auto& p_body = meas_cloud_->points[i];
            V3D pb(p_body.x, p_body.y, p_body.z);
            V3D p_imu = s.offset_R_L_I * pb + s.offset_T_L_I;
            M3D p_crossmat;
            p_crossmat << SKEW_SYM_MATRX(p_imu);
            V3D C = s.rot.conjugate() * coarse_norm[i];
            V3D A = p_crossmat * C;
            ekfom_data.h_x.block<1,3>(row, 0) = coarse_norm[i].transpose();
            ekfom_data.h_x.block<1,3>(row, 3) = A.transpose();
            ekfom_data.h(row) = coarse_residual[i] * coarse_scale;
            ekfom_data.h_x.block<1,12>(row, 0) *= coarse_scale;
            ++row;
        }
    }

    // Prototype low-tier-only map rows: scaled either by a flat 1/noise_scale
    // (isotropic, default) or by a per-row direction-dependent weight (anisotropic).
    if (use_low_own_map) {
        const double low_scale = 1.0 / params_.low_own_map_noise_scale;
        const Eigen::Matrix3d beta_diag = low_aniso_beta.asDiagonal();
        Eigen::Vector3d low_per_axis_gamma = Eigen::Vector3d::Constant(low_scale * low_scale);
        if (params_.low_own_map_anisotropic_enable
                && params_.low_own_map_per_axis_hybrid_enable) {
            int k_star = 0;
            low_aniso_beta.maxCoeff(&k_star);
            low_per_axis_gamma(k_star) = low_aniso_beta(k_star);
        }
        for (int i = 0; i < n; ++i) {
            if (!low_valid[i]) continue;
            const auto& p_body = meas_cloud_->points[i];
            V3D pb(p_body.x, p_body.y, p_body.z);
            V3D p_imu = s.offset_R_L_I * pb + s.offset_T_L_I;
            M3D p_crossmat;
            p_crossmat << SKEW_SYM_MATRX(p_imu);
            V3D C = s.rot.conjugate() * low_norm[i];
            V3D A = p_crossmat * C;
            ekfom_data.h_x.block<1,3>(row, 0) = low_norm[i].transpose();
            ekfom_data.h_x.block<1,3>(row, 3) = A.transpose();
            ekfom_data.h(row) = low_residual[i];
            double w;
            if (params_.low_own_map_anisotropic_enable
                    && params_.low_own_map_per_axis_hybrid_enable) {
                const double proj = low_norm[i].transpose() * low_aniso_V
                                    * low_per_axis_gamma.asDiagonal()
                                    * low_aniso_V.transpose() * low_norm[i];
                w = std::sqrt(std::clamp(proj, 1e-6, 1.0));
            } else if (params_.low_own_map_anisotropic_enable) {
                const double proj = low_norm[i].transpose() * low_aniso_V * beta_diag
                                    * low_aniso_V.transpose() * low_norm[i];
                const double w_aniso = std::sqrt(std::clamp(proj, 1e-6, 1.0));
                double mix;
                if (params_.low_own_map_kappa_gate_enable) {
                    mix = 1.0 / (1.0 + low_kappa / std::max(params_.low_own_map_kappa0, 1e-9));
                } else {
                    mix = std::clamp(params_.low_own_map_aniso_mix, 0.0, 1.0);
                }
                w = (1.0 - mix) * low_scale + mix * w_aniso;
            } else {
                w = low_scale;
            }
            ekfom_data.h(row) *= w;
            ekfom_data.h_x.block<1,12>(row, 0) *= w;
            ++row;
        }
    }

    // v48: accumulate position Hessian on every iEKF iteration; after the iEKF
    // finishes, last_pos_HtH_ reflects the final iteration's constraint matrix.
    // Columns 0-2 of h_x are n^T (∂res/∂pos = plane_normal), so H_pos^T*H_pos
    // captures how well each position DOF is constrained by the current scan.
    // Always update (not just on the Nth call) so the gate fires even when the
    // iEKF converges early (< num_max_iterations iterations).
    ++h_share_call_count_;
    last_pos_HtH_.setZero();
    for (int i = 0; i < effct_total; i++) {
        Eigen::Vector3d n = ekfom_data.h_x.block<1,3>(i, 0).transpose();
        last_pos_HtH_ += n * n.transpose();
    }
    last_pos_hth_valid_ = true;

    if (jitter_log_enabled_) {
        int valid_count = 0;
        double weight_sum = 0.0;
        for (int i = 0; i < n; ++i) {
            if (!point_valid[i]) continue;
            ++valid_count;
            weight_sum += quality_weights[i];
        }
        const char* h_tier = in_async_freq_ ? "async" : (in_low_freq_ ? "low" : "mid");
        long h_cycle_idx = in_async_freq_ ? 0 : (in_low_freq_ ? jitter_low_call_idx_ : jitter_mid_call_idx_);
        jitter_log_h_share(h_tier, h_cycle_idx, n, valid_count,
                           valid_count > 0 ? weight_sum / valid_count : 0.0,
                           ekfom_data.converge);
    }

    // Graph capture: at iEKF convergence, save H/r for the sliding-window optimizer.
    // Uses the final-iteration correspondences (best linearisation point).
    // Does NOT abort the iEKF update — the filter still applies its correction.
    if (capture_h_r_on_converge_ && ekfom_data.converge) {
        captured_H_ = ekfom_data.h_x;  // M×12
        captured_r_ = ekfom_data.h;    // M
    }

    // Plane cache for fast-tier GN: capture high-quality mid-tier correspondences
    // at convergence so the 100 Hz fast tier can skip ikd-tree search entirely.
    // Only populated on mid-tier calls (not low-tier, which uses different windows).
    if (params_.fast_plane_cache_enable && ekfom_data.converge && !in_low_freq_) {
        const M3D R  = s.rot.toRotationMatrix();
        const M3D RL = s.offset_R_L_I.toRotationMatrix();
        const V3D TL = s.offset_T_L_I;
        std::vector<CachedPlane> tmp;
        tmp.reserve(static_cast<size_t>(effct));
        for (int i = 0; i < n; ++i) {
            if (!point_valid[i]) continue;
            const auto& pt = meas_cloud_->points[i];
            Eigen::Vector3d pb(pt.x, pt.y, pt.z);
            Eigen::Vector3d pw = R * (RL * pb + TL) + s.pos;
            tmp.push_back({pb.cast<float>(), norm_vecs[i], -norm_vecs[i].dot(pw)});
        }
        std::lock_guard<std::mutex> lk(plane_cache_mutex_);
        plane_cache_      = std::move(tmp);
        plane_cache_valid_ = true;
    }

    // Voxel-hash population: accumulate mid-tier's converged, quality-gated
    // correspondences into the world-frame voxel map, independent of
    // fast_plane_cache_enable/fast_map_update_enable. Unlike plane_cache_ this
    // is never wholesale-replaced — it accumulates across many mid-tier cycles,
    // so the fast tier can query fresh points against a real spatial map.
    if (params_.voxel_hash_enable && ekfom_data.converge && !in_low_freq_) {
        const M3D R  = s.rot.toRotationMatrix();
        const M3D RL = s.offset_R_L_I.toRotationMatrix();
        const V3D TL = s.offset_T_L_I;
        std::lock_guard<std::mutex> lk(voxel_hash_mutex_);
        for (int i = 0; i < n; ++i) {
            if (!point_valid[i]) continue;
            const auto& pt = meas_cloud_->points[i];
            Eigen::Vector3d pb(pt.x, pt.y, pt.z);
            Eigen::Vector3d pw = R * (RL * pb + TL) + s.pos;
            const double offset = -norm_vecs[i].dot(pw);
            voxel_hash_.insert_or_update(pw, norm_vecs[i], offset);
        }
    }
}

// ---------------------------------------------------------------------------
// RIEKF ablation: h_share_model's equivalent for the InEKF filter path.
// Mirrors h_share_model's NN-search/esti_plane AND its two-tree joint-coarse
// stacking exactly (same ikdtree_/coarse_ikdtree_, same esti_plane threshold,
// same quality-weight/temporal-decay/coarse-noise-scale scaling) -- the only
// real difference is building rows in the InEKF's 24-col body-frame layout
// via riekf_convert_h_row instead of IKFoM's 12-col world-frame h_x.
// Deliberately WITHOUT the NDT-eigenplane branch (low_freq_ndt_enable is off
// by default; this ablation targets the canonical baseline path). Returns
// false exactly when h_share_model would have set ekfom_data.valid = false.
bool MfLioEstimator::h_share_model_riekf(bool find_neighbors, Eigen::MatrixXd& H, Eigen::VectorXd& z)
{
    KD_TREE<PointType>& active_tree = in_async_freq_ ? coarse_ikdtree_ : ikdtree_;
    std::vector<PointVector>& active_nearest = in_async_freq_ ? coarse_nearest_points_ : nearest_points_;

    if (!meas_cloud_ || meas_cloud_->empty() || active_tree.Root_Node == nullptr) return false;

    const mf_lio::riekf::InEKFState& s = imu_proc_.riekf_filter().x;
    const M3D R    = s.rot;
    const V3D pos  = s.pos;
    const M3D R_L  = s.offset_R_L_I;
    const V3D T_L  = s.offset_T_L_I;

    const int n = static_cast<int>(meas_cloud_->size());
    if (static_cast<int>(active_nearest.size()) != n) active_nearest.resize(n);

    std::vector<bool>   point_valid(n, false);
    std::vector<V3D>    norm_vecs(n);
    std::vector<double> residuals_raw(n, 0.0);
    std::vector<float>  quality_weights(n, 1.0f);

    #pragma omp parallel for num_threads(8) schedule(guided)
    for (int i = 0; i < n; ++i) {
        const auto& p_body = meas_cloud_->points[i];
        V3D pb(p_body.x, p_body.y, p_body.z);
        V3D pw = R * (R_L * pb + T_L) + pos;

        PointType pw_pcl;
        pw_pcl.x = static_cast<float>(pw(0));
        pw_pcl.y = static_cast<float>(pw(1));
        pw_pcl.z = static_cast<float>(pw(2));
        pw_pcl.intensity = p_body.intensity;

        if (find_neighbors) {
            std::vector<float> sq_dist(NUM_MATCH_POINTS);
            active_tree.Nearest_Search(pw_pcl, NUM_MATCH_POINTS, active_nearest[i], sq_dist);
            if (static_cast<int>(active_nearest[i].size()) < NUM_MATCH_POINTS
                || sq_dist.back() > 5.0f)
                continue;
        }
        if (active_nearest[i].size() < NUM_MATCH_POINTS) continue;

        Eigen::Matrix<float,4,1> pabcd;
        if (!esti_plane(pabcd, active_nearest[i], static_cast<float>(params_.esti_plane_threshold)))
            continue;

        float pd = pabcd(0)*pw_pcl.x + pabcd(1)*pw_pcl.y + pabcd(2)*pw_pcl.z + pabcd(3);
        float s_w = 1.0f - 0.9f * std::abs(pd) / std::sqrt(static_cast<float>(pb.norm()));
        const float q_min = in_low_freq_
            ? static_cast<float>(params_.low_freq_quality_min_weight)
            : static_cast<float>(params_.mid_freq_quality_min_weight);
        if (s_w < q_min) continue;
        const float q_power = in_low_freq_
            ? static_cast<float>(params_.low_freq_quality_weight_power)
            : static_cast<float>(params_.mid_freq_quality_weight_power);
        quality_weights[i] = std::pow(s_w, q_power);

        point_valid[i]   = true;
        norm_vecs[i]     = V3D(pabcd(0), pabcd(1), pabcd(2));
        residuals_raw[i] = -static_cast<double>(pd);
    }

    // Joint coarse-map fusion (same condition as h_share_model): identical
    // esti_plane/Jacobian math against coarse_ikdtree_, rows scaled by
    // 1/joint_coarse_noise_scale instead of quality_weights.
    const bool use_coarse_joint = !in_async_freq_
        && params_.joint_coarse_enable
        && coarse_ikdtree_.Root_Node != nullptr;

    std::vector<bool>   coarse_valid(n, false);
    std::vector<V3D>    coarse_norm(n);
    std::vector<double> coarse_residual(n, 0.0);

    if (use_coarse_joint) {
        if (static_cast<int>(coarse_nearest_points_.size()) != n)
            coarse_nearest_points_.resize(n);
        const float coarse_plane_thresh =
            static_cast<float>(params_.esti_plane_threshold * params_.joint_coarse_noise_scale);
        const float q_min = in_low_freq_
            ? static_cast<float>(params_.low_freq_quality_min_weight)
            : static_cast<float>(params_.mid_freq_quality_min_weight);

        #pragma omp parallel for num_threads(8) schedule(guided)
        for (int i = 0; i < n; ++i) {
            const auto& p_body = meas_cloud_->points[i];
            V3D pb(p_body.x, p_body.y, p_body.z);
            V3D pw = R * (R_L * pb + T_L) + pos;
            PointType pw_pcl;
            pw_pcl.x = static_cast<float>(pw(0));
            pw_pcl.y = static_cast<float>(pw(1));
            pw_pcl.z = static_cast<float>(pw(2));
            pw_pcl.intensity = p_body.intensity;

            if (find_neighbors) {
                std::vector<float> sq_dist_c(NUM_MATCH_POINTS);
                coarse_ikdtree_.Nearest_Search(
                    pw_pcl, NUM_MATCH_POINTS, coarse_nearest_points_[i], sq_dist_c);
                if (static_cast<int>(coarse_nearest_points_[i].size()) < NUM_MATCH_POINTS
                    || sq_dist_c.back() > 5.0f)
                    continue;
            }
            if (static_cast<int>(coarse_nearest_points_[i].size()) < NUM_MATCH_POINTS) continue;

            Eigen::Matrix<float,4,1> pabcd_c;
            if (!esti_plane(pabcd_c, coarse_nearest_points_[i], coarse_plane_thresh))
                continue;

            float pd_c = pabcd_c(0)*pw_pcl.x + pabcd_c(1)*pw_pcl.y
                       + pabcd_c(2)*pw_pcl.z + pabcd_c(3);
            float s_w  = 1.0f - 0.9f * std::abs(pd_c) / std::sqrt(static_cast<float>(pb.norm()));
            if (s_w < q_min) continue;

            coarse_valid[i]    = true;
            coarse_norm[i]     = V3D(pabcd_c(0), pabcd_c(1), pabcd_c(2));
            coarse_residual[i] = -static_cast<double>(pd_c);
        }
    }

    int effct = 0;
    for (int i = 0; i < n; ++i) if (point_valid[i]) ++effct;
    int effct_c = 0;
    if (use_coarse_joint)
        for (int i = 0; i < n; ++i) if (coarse_valid[i]) ++effct_c;
    if (effct < 1 && effct_c < 1) return false;

    last_effct_ = effct;
    last_quality_ = (n > 0) ? static_cast<double>(effct) / n : 0.0;
    {
        double rss = 0.0;
        for (int i = 0; i < n; ++i) if (point_valid[i]) rss += residuals_raw[i] * residuals_raw[i];
        last_residual_rmse_ = std::sqrt(rss / std::max(effct, 1));
    }

    const double angular_velocity = imu_proc_.last_angular_velocity().norm();
    const bool allow_extrinsic_update =
        params_.extrinsic_est_en &&
        (extrinsic_permissive_latched_ ||
         !params_.extrinsic_update_gate_enable ||
         (angular_velocity >= params_.extrinsic_update_min_angular_velocity &&
          last_quality_ >= params_.extrinsic_update_min_quality &&
          last_residual_rmse_ <= params_.extrinsic_update_max_residual_rmse));

    const int effct_total = effct + effct_c;
    H = Eigen::MatrixXd::Zero(effct_total, mf_lio::riekf::INEKF_DOF);
    z.resize(effct_total);

    // last_pos_HtH_ MUST stay in the SAME world-frame convention h_share_model
    // uses (accumulated from norm_vecs/coarse_norm directly, not from H's
    // body-frame dp_b block) -- mid/low_freq_update's v48 degeneracy guard
    // eigendecomposes it and projects a WORLD-frame position correction onto
    // its eigenvectors; feeding it a body-frame Jacobian would silently mix
    // coordinate frames.
    last_pos_HtH_.setZero();

    int row = 0;
    for (int i = 0; i < n; ++i) {
        if (!point_valid[i]) continue;

        const auto& p_body = meas_cloud_->points[i];
        V3D pb(p_body.x, p_body.y, p_body.z);
        V3D p_imu = R_L * pb + T_L;

        M3D p_crossmat; p_crossmat << SKEW_SYM_MATRX(p_imu);
        V3D C = R.transpose() * norm_vecs[i];
        V3D A = p_crossmat * C;

        Eigen::Matrix<double, 1, mf_lio::riekf::INEKF_DOF> h_row;
        if (allow_extrinsic_update) {
            M3D pb_crossmat; pb_crossmat << SKEW_SYM_MATRX(pb);
            V3D B = pb_crossmat * R_L.transpose() * C;
            mf_lio::riekf::riekf_convert_h_row(A, C, h_row, /*include_extrinsics=*/true, &B);
        } else {
            mf_lio::riekf::riekf_convert_h_row(A, C, h_row, /*include_extrinsics=*/false);
        }
        double z_val = residuals_raw[i];

        double w = quality_weights[i];
        if (in_low_freq_ && params_.temporal_decay_alpha > 0.0) {
            const double age_frac = static_cast<double>(p_body.curvature) / policy_.W_l_ms();
            w *= std::exp(-0.5 * params_.temporal_decay_alpha * age_frac);
        }
        h_row *= w;
        z_val *= w;

        H.row(row) = h_row;
        z(row)     = z_val;
        last_pos_HtH_ += norm_vecs[i] * norm_vecs[i].transpose();
        ++row;
    }

    if (use_coarse_joint) {
        const double coarse_scale = 1.0 / params_.joint_coarse_noise_scale;
        for (int i = 0; i < n; ++i) {
            if (!coarse_valid[i]) continue;
            const auto& p_body = meas_cloud_->points[i];
            V3D pb(p_body.x, p_body.y, p_body.z);
            V3D p_imu = R_L * pb + T_L;
            M3D p_crossmat; p_crossmat << SKEW_SYM_MATRX(p_imu);
            V3D C = R.transpose() * coarse_norm[i];
            V3D A = p_crossmat * C;

            Eigen::Matrix<double, 1, mf_lio::riekf::INEKF_DOF> h_row;
            mf_lio::riekf::riekf_convert_h_row(A, C, h_row, /*include_extrinsics=*/false);
            h_row *= coarse_scale;
            double z_val = coarse_residual[i] * coarse_scale;

            H.row(row) = h_row;
            z(row)     = z_val;
            last_pos_HtH_ += coarse_norm[i] * coarse_norm[i].transpose();
            ++row;
        }
    }
    last_pos_hth_valid_ = true;

    return true;
}

// ---------------------------------------------------------------------------
// RIEKF ablation: iterate-until-converged correction loop, same pattern as
// mf_lio_inekf's own driving loop. find_neighbors=true only on the first
// iteration, converge when the state correction norm drops below 1e-6 or
// num_max_iterations is reached, then one final covariance update using the
// last iteration's H/z. Writes the result back into kf_ via change_x/
// change_P so every other consumer of kf_.get_x()/get_P() is unaffected by
// which filter produced it. Called from mid_freq_update/low_freq_update in
// place of kf_.update_iterated_dyn_share_modified() when params_.riekf_enable.
void MfLioEstimator::run_riekf_correction()
{
    auto& filt = imu_proc_.riekf_filter();
    const Eigen::Matrix<double, mf_lio::riekf::INEKF_DOF, mf_lio::riekf::INEKF_DOF> P_pred = filt.P;

    Eigen::MatrixXd H, H_final;
    Eigen::VectorXd z, z_final;
    bool has_meas = false;

    for (int iter = 0; iter < params_.num_max_iterations; ++iter) {
        if (!h_share_model_riekf(iter == 0, H, z)) break;
        has_meas = true;
        H_final = H; z_final = z;
        const double dx_norm = filt.update_state_only(H, z, P_pred, LASER_POINT_COV);
        if (dx_norm < 1e-6) break;
    }
    if (has_meas) filt.update_covariance(H_final, z_final, LASER_POINT_COV);

    state_ikfom s_new = mf_lio::riekf::riekf_to_state_ikfom(filt.x);
    Eigen::Matrix<double, 23, 23> P_new = mf_lio::riekf::riekf_P_to_ikfom(filt.P, s_new);
    kf_.change_x(s_new);
    kf_.change_P(P_new);
}

// ---------------------------------------------------------------------------
// RIEKF ablation: re-seeds riekf_filter_ from kf_'s current (authoritative)
// state/covariance. No-op unless params_.riekf_enable. Called before every
// early return and once at the end of each tier's correction+guard-rail
// block, so it uniformly picks up whatever the v48 guard / motion-acceptance
// rollback / mid_freq_min_p_improvement rollback / IGR blend code settled
// on, regardless of how many intermediate mutations happened this cycle.
void MfLioEstimator::sync_riekf_from_kf()
{
    if (!params_.riekf_enable) return;
    const state_ikfom x = kf_.get_x();
    imu_proc_.riekf_filter().x = mf_lio::riekf::state_ikfom_to_riekf(x);
    imu_proc_.riekf_filter().P = mf_lio::riekf::ikfom_P_to_riekf(kf_.get_P(), x);
}

// ---------------------------------------------------------------------------
bool MfLioEstimator::can_publish(double t_now) const {
    return initialized_ &&
           (!params_.publish_requires_stable_update || stable_update_done_) &&
           (t_now - init_time_) >= params_.publish_start_delay_ms * 1e-3;
}

bool MfLioEstimator::is_stable_publish_update() const {
    return last_effct_ >= params_.publish_stable_min_correspondences &&
           last_quality_ >= params_.publish_stable_min_quality &&
           last_residual_rmse_ <= params_.publish_stable_max_residual_rmse &&
           state_point_.pos.norm() >= params_.publish_stable_min_state_norm;
}

void MfLioEstimator::reset_update_diagnostics() {
    last_effct_ = 0;
    last_quality_ = 0.0;
    last_residual_rmse_ = std::numeric_limits<double>::infinity();
}

// ---------------------------------------------------------------------------
// Fast-frequency update: ~100 Hz, IMU-only propagation, NO iEKF, NO map access.
// Predicts state forward from last mid_freq update to t_now using IMU only.
// Does not modify kf_ — mid_freq always starts from a clean IMU-propagated prior.
// ---------------------------------------------------------------------------
void MfLioEstimator::fast_freq_update(double t_now) {
    if (!imu_proc_.initialized()) return;
    // v19 async: allow fast_freq to run even before first_update_done_ (coarse map init).
    // Non-async modes keep the original gate.
    if (!params_.async_100hz_enable) {
        if (!first_update_done_) return;
        if (!can_publish(t_now)) return;
    }
    state_ikfom kf_state = kf_.get_x();
    state_ikfom predicted = imu_proc_.predict_state(last_mid_freq_t_, t_now, kf_state);

    // Fast-tier: update 100Hz omega EMA (smooth motion signal for W_m and W_l policies).
    omega_ema_ = params_.omega_ema_alpha * omega_ema_
               + (1.0 - params_.omega_ema_alpha) * imu_proc_.last_angular_velocity().norm();
    // Motion-triggered mid pre-empt: fire mid-tier early when rotation exceeds threshold.
    if (omega_ema_ > params_.W_m_omega_hi &&
        (t_now - last_mid_freq_t_) >= params_.W_m_min_ms * 1e-3)
        lidar_early_mid_request_ = true;

    // IMU motion since last mid-tier — shared by plane-cache and fast-map trust models
    Eigen::Quaterniond dq = predicted.rot * kf_state.rot.inverse();
    double dtheta = 2.0 * std::acos(std::min(1.0, std::abs(dq.w())));
    double dv     = (predicted.vel - kf_state.vel).norm();

    if (params_.fast_plane_cache_enable) {
        if (params_.fast_plane_cache_publish) {
            // Ground mode: apply correction to published 100Hz pose
            apply_plane_cache_gn(predicted, dtheta, dv);
        } else {
            // Aerial mode: compute correction on a copy, save delta for mid-tier undistortion
            state_ikfom corrected = predicted;
            apply_plane_cache_gn(corrected, dtheta, dv);
            V3D delta_p = corrected.pos - predicted.pos;
            if (delta_p.norm() > 1e-6 && params_.fast_undistort_enable) {
                std::lock_guard<std::mutex> lk(fast_delta_mutex_);
                fast_delta_cache_.push_back({t_now, delta_p});
                while (fast_delta_cache_.size() > 1 &&
                       t_now - fast_delta_cache_.front().t > 0.2)
                    fast_delta_cache_.pop_front();
            }
            // predicted unchanged → published pose is pure IMU → ATE preserved
        }

        // v10: LiDAR-based mid-tier scheduling — trigger early when cached planes are stale
        if (last_fast_stale_fraction_ > params_.fast_plane_cache_stale_trigger &&
            (t_now - last_mid_freq_t_) >= params_.fast_plane_cache_stale_min_interval_ms * 1e-3)
            lidar_early_mid_request_ = true;
    }

    bool fast_quality_ok = false;  // set when async coarse iEKF passes quality gate
    if (params_.voxel_hash_enable) {
        // Voxel-hash correction runs INSTEAD of the fast_map_update_enable branch
        // below, regardless of that flag's value — this is what lets it activate
        // on the robot/husky/rtk/forest/unitree family too, where
        // fast_map_update_enable is false and fast-tier otherwise does nothing.
        apply_voxel_hash_gn(predicted, t_now, dtheta, dv);
    } else if (params_.fast_map_update_enable &&
        (ikdtree_.Root_Node != nullptr || coarse_ikdtree_.Root_Node != nullptr ||
         params_.async_100hz_enable)) {
        auto raw_pts = point_buf_.get_window(last_fast_freq_t_, t_now);
        if (!raw_pts.empty()) {
            // Anchor time for undistortion: advances when fast-tier commits (v18)
            const double anchor_t  = std::max(last_mid_freq_t_, last_fast_commit_t_);
            const double t_win_start = last_fast_freq_t_;
            const double t_win_len   = std::max(t_now - last_fast_freq_t_, 1e-9);
            const int n_raw = static_cast<int>(raw_pts.size());
            const int stride = std::max(1, n_raw / 300);

            // Predict IMU states at window start and end (read-only, no side effects)
            const state_ikfom x_ws = imu_proc_.predict_state(anchor_t, last_fast_freq_t_, kf_state);
            const state_ikfom x_we = imu_proc_.predict_state(anchor_t, t_now, kf_state);

            if (params_.async_100hz_enable) {
                // v19v2: asynchronous 100Hz iEKF on coarse map — save/revert with correction-norm gate.
                // Accepts commit only if |Δpos| < fast_max_correction_m; coarse map is read-only here.
                const M3D R_we   = x_we.rot.toRotationMatrix();
                const M3D R_L    = x_we.offset_R_L_I.toRotationMatrix();
                const V3D T_L    = x_we.offset_T_L_I;
                const V3D pos_we = x_we.pos;

                auto fast_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
                fast_scan->reserve((n_raw + stride - 1) / stride);
                for (int ki = 0; ki < n_raw; ki += stride) {
                    const auto& sp  = raw_pts[ki];
                    const double alpha = std::max(0.0, std::min(1.0,
                        (sp.timestamp - t_win_start) / t_win_len));
                    const V3D pos_p = (1.0 - alpha) * x_ws.pos + alpha * pos_we;
                    const Eigen::Quaterniond rot_p = x_ws.rot.slerp(alpha, x_we.rot);
                    const V3D pb(sp.point.x, sp.point.y, sp.point.z);
                    const V3D pw_u = rot_p.toRotationMatrix() * (R_L * pb + T_L) + pos_p;
                    const V3D p_lidar = R_L.transpose() * (R_we.transpose() * (pw_u - pos_we) - T_L);
                    pcl::PointXYZINormal pt;
                    pt.x = static_cast<float>(p_lidar.x());
                    pt.y = static_cast<float>(p_lidar.y());
                    pt.z = static_cast<float>(p_lidar.z());
                    pt.intensity = sp.point.intensity;
                    fast_scan->push_back(pt);
                }

                if ((int)fast_scan->size() >= params_.fast_map_min_points) {
                    const auto prev_meas_cloud = meas_cloud_;
                    const int  prev_meas_size  = meas_size_;

                    meas_cloud_ = fast_scan;
                    meas_size_  = static_cast<int>(fast_scan->size());
                    coarse_nearest_points_.resize(meas_size_);
                    // Save fine diagnostics so can_publish stays governed by fine-map quality.
                    const double fast_saved_quality = last_quality_;
                    const double fast_saved_rmse    = last_residual_rmse_;
                    const int    fast_saved_effct   = last_effct_;
                    reset_update_diagnostics();
                    h_share_call_count_ = 0;
                    last_pos_hth_valid_ = false;

                    // Skip coarse iEKF entirely when motion gate would never fire —
                    // avoids timing perturbation on slow platforms (Husky indoor etc.).
                    const bool motion_gate_could_fire =
                        (params_.fast_gate_omega_thresh <= 0.0 ||
                         omega_ema_ >= params_.fast_gate_omega_thresh * 0.5);

                    if (coarse_ikdtree_.Root_Node == nullptr) {
                        // First call: build coarse map from this sub-scan, skip iEKF
                        PointVector init_pts;
                        init_pts.reserve(fast_scan->size());
                        for (const auto& p : fast_scan->points) {
                            const V3D pbpt(p.x, p.y, p.z);
                            const V3D pw = kf_state.rot * (kf_state.offset_R_L_I * pbpt
                                           + kf_state.offset_T_L_I) + kf_state.pos;
                            PointType wp; wp.x=static_cast<float>(pw.x());
                            wp.y=static_cast<float>(pw.y()); wp.z=static_cast<float>(pw.z());
                            init_pts.push_back(wp);
                        }
                        coarse_ikdtree_.set_downsample_param(
                            static_cast<float>(params_.fast_coarse_voxel_size));
                        coarse_ikdtree_.Build(init_pts);
                        state_point_ = kf_state;
                        first_update_done_ = true;
                    } else if (motion_gate_could_fire) {
                        // v19v4 dual-state: fast_state_ tracks position via coarse iEKF;
                        // kf_ is never committed by the fast-tier — P only updated by mid/low-tier.
                        auto x_kf_now = kf_.get_x();
                        auto P_kf_now = kf_.get_P();

                        // Apply accumulated coarse correction delta to get better iEKF init point.
                        // delta = fast_state_ - prev_kf_imu_ = sum of coarse corrections since last mid-tier.
                        V3D fast_init_pos = x_kf_now.pos;
                        if (fast_state_valid_) {
                            state_ikfom x_init = x_kf_now;
                            x_init.pos = x_kf_now.pos + (fast_state_.pos - prev_kf_imu_.pos);
                            x_init.rot = (fast_state_.rot * prev_kf_imu_.rot.inverse()) * x_kf_now.rot;
                            x_init.vel = x_kf_now.vel + (fast_state_.vel - prev_kf_imu_.vel);
                            kf_.change_x(x_init);
                            fast_init_pos = x_init.pos;
                        }

                        const int saved_max_iter = kf_.get_maximum_iter();
                        kf_.set_maximum_iter(params_.fast_eskf_max_iter);
                        double solve_t = 0.0;
                        in_async_freq_ = true;
                        kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_t);
                        in_async_freq_ = false;
                        kf_.set_maximum_iter(saved_max_iter);

                        // Quality gate: accept coarse correction if physically plausible.
                        // prev_kf_imu_ is updated atomically with fast_state_ so the delta
                        // (fast_state_ - prev_kf_imu_) always refers to the same time point.
                        // If quality gate fails, both stay at the previous successful update,
                        // and delta remains a valid (stale but non-diverging) correction.
                        const double corr_norm = (kf_.get_x().pos - fast_init_pos).norm();
                        const bool motion_gate  = (omega_ema_ >= params_.fast_gate_omega_thresh
                                                   || params_.fast_gate_omega_thresh <= 0.0);
                        const bool quality_gate = (last_quality_      >= params_.fast_gate_min_quality)
                                               && (last_residual_rmse_ <= params_.fast_gate_max_rmse)
                                               && (last_effct_         >= params_.fast_gate_min_correspondences);
                        if (corr_norm < params_.fast_max_correction_m && motion_gate && quality_gate) {
                            fast_state_         = kf_.get_x();
                            prev_kf_imu_        = x_kf_now;  // atomic with fast_state_
                            fast_state_valid_   = true;
                            last_fast_commit_t_ = t_now;
                            fast_quality_ok     = true;
                        }
                        // Restore kf_ and P unless write-back is enabled AND the correction
                        // was accepted by the quality gate — in that case, let the fast-tier
                        // correction persist in the shared filter for mid/low tiers to inherit.
                        if (!(params_.fast_tier_writeback_enable && fast_quality_ok)) {
                            kf_.change_x(x_kf_now);
                            kf_.change_P(P_kf_now);
                        }
                        // AMR: degenerate coarse map → pre-empt mid-tier immediately.
                        if (params_.fast_degen_early_mid_enable && last_pos_hth_valid_) {
                            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_f(last_pos_HtH_);
                            if (eig_f.eigenvalues()(0) < params_.fast_degen_early_mid_thresh)
                                lidar_early_mid_request_ = true;
                        }
                        // AMR: adapt fast-tier window from coarse iEKF quality.
                        // good quality → slow down (W_f_max); poor quality → fire fast (W_f_min).
                        {
                            const double denom = params_.fast_quality_good - params_.fast_quality_bad;
                            const double q = denom > 1e-9
                                ? std::clamp((last_quality_ - params_.fast_quality_bad) / denom,
                                             0.0, 1.0)
                                : 1.0;
                            current_W_f_ms_ = params_.W_f_min_ms
                                            + (params_.W_f_max_ms - params_.W_f_min_ms) * q;
                        }
                    }

                    // Restore fine-map diagnostics: W_f adaptation above uses coarse quality
                    // (intentional), but can_publish and mid-tier W_m must use fine quality.
                    last_quality_       = fast_saved_quality;
                    last_residual_rmse_ = fast_saved_rmse;
                    last_effct_         = fast_saved_effct;
                    meas_cloud_ = prev_meas_cloud;
                    meas_size_  = prev_meas_size;
                }
            } else if (params_.fast_eskf_enable) {
                // v18: quality-gated tentative iEKF (kept for backward compatibility)
                const double speed = kf_state.vel.norm();
                if (params_.fast_eskf_min_speed_m_s > 0.0 &&
                    speed < params_.fast_eskf_min_speed_m_s) {
                    // skip: platform too slow
                } else {
                const M3D R_we   = x_we.rot.toRotationMatrix();
                const M3D R_L    = x_we.offset_R_L_I.toRotationMatrix();
                const V3D T_L    = x_we.offset_T_L_I;
                const V3D pos_we = x_we.pos;

                auto fast_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
                fast_scan->reserve((n_raw + stride - 1) / stride);
                for (int ki = 0; ki < n_raw; ki += stride) {
                    const auto& sp  = raw_pts[ki];
                    const double alpha = std::max(0.0, std::min(1.0,
                        (sp.timestamp - t_win_start) / t_win_len));
                    const V3D pos_p = (1.0 - alpha) * x_ws.pos + alpha * pos_we;
                    const Eigen::Quaterniond rot_p = x_ws.rot.slerp(alpha, x_we.rot);
                    const V3D pb(sp.point.x, sp.point.y, sp.point.z);
                    const V3D pw_u = rot_p.toRotationMatrix() * (R_L * pb + T_L) + pos_p;
                    const V3D p_lidar = R_L.transpose() * (R_we.transpose() * (pw_u - pos_we) - T_L);
                    pcl::PointXYZINormal pt;
                    pt.x = static_cast<float>(p_lidar.x());
                    pt.y = static_cast<float>(p_lidar.y());
                    pt.z = static_cast<float>(p_lidar.z());
                    pt.intensity = sp.point.intensity;
                    fast_scan->push_back(pt);
                }

                if ((int)fast_scan->size() >= params_.fast_map_min_points) {
                    auto x_saved = kf_.get_x();
                    auto P_saved = kf_.get_P();
                    const auto prev_meas_cloud = meas_cloud_;
                    const int  prev_meas_size  = meas_size_;

                    meas_cloud_ = fast_scan;
                    meas_size_  = static_cast<int>(fast_scan->size());
                    nearest_points_.resize(meas_size_);
                    reset_update_diagnostics();
                    h_share_call_count_ = 0;
                    last_pos_hth_valid_ = false;

                    const int saved_max_iter = kf_.get_maximum_iter();
                    kf_.set_maximum_iter(params_.fast_eskf_max_iter);
                    double solve_t = 0.0;
                    kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_t);
                    kf_.set_maximum_iter(saved_max_iter);

                    const double pos_corr_norm = (kf_.get_x().pos - x_saved.pos).norm();
                    const bool quality_ok =
                        last_effct_ >= params_.fast_eskf_min_correspondences &&
                        last_residual_rmse_ <= params_.fast_eskf_max_residual &&
                        (params_.fast_eskf_min_correction_m <= 0.0 ||
                         pos_corr_norm >= params_.fast_eskf_min_correction_m);

                    if (quality_ok) {
                        state_point_ = kf_.get_x();
                        PointVector fast_world_pts;
                        fast_world_pts.reserve(fast_scan->size());
                        const state_ikfom& st = state_point_;
                        for (const auto& p : fast_scan->points) {
                            const V3D pbpt(p.x, p.y, p.z);
                            const V3D pw = st.rot * (st.offset_R_L_I * pbpt + st.offset_T_L_I) + st.pos;
                            PointType pw_pcl;
                            pw_pcl.x = static_cast<float>(pw.x());
                            pw_pcl.y = static_cast<float>(pw.y());
                            pw_pcl.z = static_cast<float>(pw.z());
                            fast_world_pts.push_back(pw_pcl);
                        }
                        ikdtree_.Add_Points(fast_world_pts, true);
                        last_fast_commit_t_ = t_now;
                    } else {
                        kf_.change_x(x_saved);
                        kf_.change_P(P_saved);
                    }

                    meas_cloud_ = prev_meas_cloud;
                    meas_size_  = prev_meas_size;
                }
                } // else speed gate passes
            } else {
                // v14/v15/v17: decoupled 3×3 GN (kept for backward compatibility)
                const int npts = (n_raw + stride - 1) / stride;
                if (npts >= params_.fast_map_min_points) {
                    const M3D R_end  = predicted.rot.toRotationMatrix();
                    const M3D R_L    = predicted.offset_R_L_I.toRotationMatrix();
                    const V3D T_L    = predicted.offset_T_L_I;
                    const V3D pos_end = predicted.pos;

                    Eigen::Matrix<double,6,6> HtH = Eigen::Matrix<double,6,6>::Zero();
                    Eigen::Matrix<double,6,1> Htr = Eigen::Matrix<double,6,1>::Zero();
                    int effct = 0;

                    std::vector<PointVector> nn_fast(static_cast<size_t>(npts));
                    for (int ki = 0; ki < npts; ++ki) {
                        const auto& sp = raw_pts[ki * stride];
                        double alpha = std::max(0.0, std::min(1.0,
                            (sp.timestamp - t_win_start) / t_win_len));
                        Eigen::Vector3d pos_p = (1.0 - alpha) * x_ws.pos + alpha * pos_end;
                        Eigen::Quaterniond rot_p = x_ws.rot.slerp(alpha, predicted.rot);
                        const M3D R_p = rot_p.toRotationMatrix();
                        V3D pb(sp.point.x, sp.point.y, sp.point.z);
                        V3D p_imu_raw = R_L * pb + T_L;
                        V3D pw_u = R_p * p_imu_raw + pos_p;
                        V3D p_imu_canon = R_end.transpose() * (pw_u - pos_end);
                        PointType pw_pcl;
                        pw_pcl.x = static_cast<float>(pw_u(0));
                        pw_pcl.y = static_cast<float>(pw_u(1));
                        pw_pcl.z = static_cast<float>(pw_u(2));
                        std::vector<float> sq_dist;
                        ikdtree_.Nearest_Search(pw_pcl, NUM_MATCH_POINTS, nn_fast[ki], sq_dist);
                        if ((int)nn_fast[ki].size() < NUM_MATCH_POINTS || sq_dist.empty() ||
                            sq_dist.back() > 5.0f) continue;
                        Eigen::Matrix<float,4,1> pabcd;
                        if (!esti_plane(pabcd, nn_fast[ki],
                                        static_cast<float>(params_.esti_plane_threshold))) continue;
                        float pd = pabcd(0)*pw_pcl.x + pabcd(1)*pw_pcl.y + pabcd(2)*pw_pcl.z + pabcd(3);
                        float s_w = 1.0f - 0.9f * std::abs(pd)
                                    / std::sqrt(static_cast<float>(p_imu_canon.norm() + 1e-6));
                        if (s_w < 0.9f) continue;
                        V3D n_vec(pabcd(0), pabcd(1), pabcd(2));
                        M3D p_imu_cross; p_imu_cross << SKEW_SYM_MATRX(p_imu_canon);
                        V3D A = p_imu_cross * (R_end.transpose() * n_vec);
                        Eigen::Matrix<double,1,6> J;
                        J.block<1,3>(0,0) = n_vec.transpose();
                        J.block<1,3>(0,3) = A.transpose();
                        double r = -static_cast<double>(pd);
                        HtH += J.transpose() * J;
                        Htr -= J.transpose() * r;
                        ++effct;
                    }
                    double trust_rot_v = std::exp(-params_.fast_plane_cache_rot_decay * dtheta);
                    double trust_tra_v = std::exp(-params_.fast_plane_cache_trans_decay * dv);
                    double trust_cnt_v = std::min(1.0, static_cast<double>(effct)
                                                        / params_.fast_plane_cache_count_full);
                    double trust_v = trust_rot_v * trust_tra_v * trust_cnt_v;
                    if (effct >= params_.fast_map_min_points &&
                        trust_v >= params_.fast_plane_cache_min_trust) {
                        Eigen::Vector3d delta_pos;
                        Eigen::Vector3d delta_rot = Eigen::Vector3d::Zero();
                        if (params_.fast_6dof_enable) {
                            // §5+§6: Full 6-DoF GN with rotation-unit normalization.
                            // S = diag(1,1,1, s_R, s_R, s_R); H̄ = S·HtH·S; ḡ = S·Htr
                            const double s_R = params_.fast_norm_rot_scale;
                            Eigen::Matrix<double,6,1> S_vec;
                            S_vec << 1.0, 1.0, 1.0, s_R, s_R, s_R;
                            Eigen::Matrix<double,6,6> Hn = S_vec.asDiagonal() * HtH * S_vec.asDiagonal();
                            Eigen::Matrix<double,6,1> gn = S_vec.asDiagonal() * Htr;
                            Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd6(Hn,
                                Eigen::ComputeFullU | Eigen::ComputeFullV);
                            Eigen::Matrix<double,6,1> sv6 = svd6.singularValues();
                            for (int i = 0; i < 6; ++i)
                                sv6(i) = (sv6(i) > params_.fast_plane_cache_min_eig)
                                         ? 1.0 / sv6(i) : 0.0;
                            Eigen::Matrix<double,6,1> xi_n =
                                svd6.matrixV() * sv6.asDiagonal() * svd6.matrixU().transpose() * gn;
                            Eigen::Matrix<double,6,1> xi = S_vec.asDiagonal() * xi_n;
                            delta_pos = xi.head<3>() * trust_v;
                            delta_rot = xi.tail<3>() * trust_v;
                        } else {
                            // §4 position-only (default): 3×3 truncated pseudo-inverse
                            auto pinv3 = [&](const Eigen::Matrix3d& A3) -> Eigen::Matrix3d {
                                Eigen::JacobiSVD<Eigen::Matrix3d> svd(A3,
                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
                                Eigen::Vector3d sv = svd.singularValues();
                                for (int i = 0; i < 3; ++i)
                                    sv(i) = (sv(i) > params_.fast_plane_cache_min_eig)
                                            ? 1.0 / sv(i) : 0.0;
                                return svd.matrixV() * sv.asDiagonal() * svd.matrixU().transpose();
                            };
                            delta_pos = pinv3(HtH.block<3,3>(0,0)) * Htr.head<3>() * trust_v;
                        }
                        if (params_.fast_undistort_enable && delta_pos.norm() > 1e-6) {
                            Eigen::JacobiSVD<Eigen::Matrix3d> svd_p(HtH.block<3,3>(0,0),
                                Eigen::ComputeFullU | Eigen::ComputeFullV);
                            if (svd_p.singularValues().minCoeff() >= params_.fast_undistort_min_eig) {
                                std::lock_guard<std::mutex> lk(fast_delta_mutex_);
                                fast_delta_cache_.push_back({t_now, delta_pos});
                                while (fast_delta_cache_.size() > 1 &&
                                       t_now - fast_delta_cache_.front().t > 0.2)
                                    fast_delta_cache_.pop_front();
                            }
                        }
                        // §4 anchor-relative: accumulate position correction; never touch kf_
                        fast_delta_xi_pos_ += delta_pos;
                        const double afc_max = 0.5;
                        if (fast_delta_xi_pos_.norm() > afc_max)
                            fast_delta_xi_pos_ = fast_delta_xi_pos_.normalized() * afc_max;
                        // RDEF/CI diagnostic: accumulate this fast-tier correction's own
                        // position information (same convention as last_pos_HtH_: raw
                        // sum of normal outer products, no extra R^-1 scaling). Gated so
                        // it costs nothing when the diagnostic is off.
                        if (params_.rdef_ci_diagnostic_enable)
                            Lambda_F_pos_ += HtH.block<3,3>(0,0);
                        // §5 rotation correction: accumulate, bounded at 0.1 rad (~5.7°)
                        if (params_.fast_6dof_enable) {
                            fast_delta_xi_rot_ += delta_rot;
                            const double rot_max = 0.1;
                            if (fast_delta_xi_rot_.norm() > rot_max)
                                fast_delta_xi_rot_ = fast_delta_xi_rot_.normalized() * rot_max;
                        }

                        if (params_.fast_writeback_enable) {
                            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> es(HtH);
                            const double min_eig = es.eigenvalues().minCoeff();
                            const double max_eig = es.eigenvalues().maxCoeff();
                            const double kappa   = (min_eig > 1e-12) ? (max_eig / min_eig) : 1e15;
                            if (min_eig > params_.fast_writeback_lambda_thresh &&
                                kappa    < params_.fast_writeback_kappa_thresh) {
                                auto x = kf_.get_x();
                                x.pos += params_.fast_writeback_damp * delta_pos;
                                V3D wb_drot = params_.fast_writeback_damp * delta_rot;
                                double wb_angle = wb_drot.norm();
                                if (wb_angle > 1e-10) {
                                    Eigen::AngleAxisd aa(wb_angle, wb_drot / wb_angle);
                                    x.rot = x.rot * Eigen::Quaterniond(aa);
                                    x.rot.normalize();
                                }
                                kf_.change_x(x);
                            }
                        }
                    }
                }
            }
        }
    }

    // Publish fast-tier pose.
    // AMR: fast odometry publishes scan-corrected fast_state_ when quality gate passes.
    // Main odometry always gets IMU propagation to preserve ATE from fine iEKF.
    const double pub_anchor = std::max(last_mid_freq_t_, last_fast_commit_t_);
    kf_state = kf_.get_x();
    const state_ikfom imu_published = imu_proc_.predict_state(pub_anchor, t_now, kf_state);
    // §4/§5 anchor-relative: add accumulated GN delta to IMU-propagated pose for fast output
    state_ikfom fast_corrected = imu_published;
    fast_corrected.pos += fast_delta_xi_pos_;
    if (params_.fast_6dof_enable && fast_delta_xi_rot_.norm() > 1e-9) {
        const double half = fast_delta_xi_rot_.norm() * 0.5;
        const Eigen::Vector3d ax = fast_delta_xi_rot_.normalized();
        Eigen::Quaterniond dq(std::cos(half),
                              ax.x() * std::sin(half),
                              ax.y() * std::sin(half),
                              ax.z() * std::sin(half));
        fast_corrected.rot = fast_corrected.rot * dq;
    }
    const state_ikfom& fast_published =
        (params_.async_100hz_enable && fast_quality_ok) ? fast_state_ : fast_corrected;

    // In async mode: gate publish on can_publish (was implicit via early return in non-async)
    if (!params_.async_100hz_enable || can_publish(t_now)) {
        if (odom_fast_cb_) odom_fast_cb_(fast_published, t_now);
        if (params_.publish_fast_odometry_on_main && odom_cb_) {
            odom_cb_(imu_published, t_now);  // main always uses fine-estimator IMU propagation
        }
        if (freq_cb_) freq_cb_(current_W_f_ms_, policy_.W_l_ms());
    }
}

// ---------------------------------------------------------------------------
// Apply cached fast-tier δpos corrections to undistorted scan points.
// Each point's curvature field holds its ms offset from t_start; we look up the
// interpolated δpos from fast_delta_cache_ and rotate it into body frame.
// ---------------------------------------------------------------------------
void MfLioEstimator::apply_fast_undistort(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double t_start)
{
    std::lock_guard<std::mutex> lk(fast_delta_mutex_);
    if (fast_delta_cache_.empty()) return;
    const Eigen::Matrix3d R_end_inv = state_point_.rot.toRotationMatrix().transpose();
    for (auto& pt : cloud->points) {
        double t_p = t_start + pt.curvature * 1e-3;
        auto it = std::lower_bound(fast_delta_cache_.begin(), fast_delta_cache_.end(), t_p,
                                   [](const FastDeltaEntry& e, double t) { return e.t < t; });
        if (it == fast_delta_cache_.end()) continue;
        Eigen::Vector3d dp_world;
        if (it == fast_delta_cache_.begin()) {
            dp_world = it->delta_pos;
        } else {
            auto prev = std::prev(it);
            double alpha = (t_p - prev->t) / (it->t - prev->t);
            dp_world = (1.0 - alpha) * prev->delta_pos + alpha * it->delta_pos;
        }
        Eigen::Vector3d dp_body = R_end_inv * dp_world;
        pt.x += static_cast<float>(dp_body.x());
        pt.y += static_cast<float>(dp_body.y());
        pt.z += static_cast<float>(dp_body.z());
    }
}

// ---------------------------------------------------------------------------
// Fast-tier plane-cache GN: apply mid-tier correspondences to the IMU-propagated
// predicted state at 100 Hz — no ikd-tree search, no esti_plane.
// Uses same sign convention as fast_freq_update's standalone GN block.
// ---------------------------------------------------------------------------
void MfLioEstimator::apply_plane_cache_gn(state_ikfom& predicted, double dtheta, double dv) {
    std::lock_guard<std::mutex> lk(plane_cache_mutex_);
    if (!plane_cache_valid_ || plane_cache_.empty()) {
        last_fast_stale_fraction_ = 1.0;
        return;
    }

    const M3D R  = predicted.rot.toRotationMatrix();
    const M3D RL = predicted.offset_R_L_I.toRotationMatrix();
    const V3D TL = predicted.offset_T_L_I;

    Eigen::Matrix<double,6,6> HtH = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,1> Htr = Eigen::Matrix<double,6,1>::Zero();
    int effct_accepted = 0;

    for (const auto& cp : plane_cache_) {
        Eigen::Vector3d pb(cp.p_body.x(), cp.p_body.y(), cp.p_body.z());
        Eigen::Vector3d p_imu = RL * pb + TL;
        Eigen::Vector3d pw    = R * p_imu + predicted.pos;
        double pd = cp.n.dot(pw) + cp.d;

        if (std::abs(pd) > 0.3) continue;  // plane is stale or sensor moved too far
        ++effct_accepted;

        M3D cross;
        cross << SKEW_SYM_MATRX(p_imu);
        V3D A = cross * (R.transpose() * cp.n);

        Eigen::Matrix<double,1,6> J;
        J.block<1,3>(0,0) = cp.n.transpose();
        J.block<1,3>(0,3) = A.transpose();

        double r = -pd;            // same sign convention as fast_freq_update GN
        HtH += J.transpose() * J;
        Htr -= J.transpose() * r; // Htr += J^T * pd
    }

    if (effct_accepted < params_.fast_plane_cache_min_points) {
        last_fast_stale_fraction_ = 1.0;
        return;
    }

    // IMU-derived UQ trust: exponential decay with rotation and velocity change since last mid-tier
    double trust_rot   = std::exp(-params_.fast_plane_cache_rot_decay   * dtheta);
    double trust_trans = std::exp(-params_.fast_plane_cache_trans_decay  * dv);
    double trust_count = std::min(1.0, static_cast<double>(effct_accepted)
                                       / static_cast<double>(params_.fast_plane_cache_count_full));
    double trust = trust_rot * trust_trans * trust_count;
    if (trust < params_.fast_plane_cache_min_trust) {
        last_fast_stale_fraction_ = 1.0;
        return;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> es(HtH);
    if (es.eigenvalues().minCoeff() < static_cast<double>(params_.fast_plane_cache_min_eig)) {
        last_fast_stale_fraction_ = 1.0;
        return;
    }

    // v13: geometry-coupled adaptive subspace GN
    // Power scales from 0 (well-conditioned → v8 behavior) to max (degenerate → strong suppression).
    // Trust scalar (delta *= trust) already handles motion-based damping separately.
    Eigen::Matrix<double,6,1> delta_raw = HtH.ldlt().solve(Htr);
    const double lambda_max = es.eigenvalues().maxCoeff();
    const double cond_ratio = es.eigenvalues().minCoeff() / (lambda_max + 1e-9);
    const double eig_power_dyn = params_.fast_plane_cache_eig_power *
                                 std::max(0.0, 1.0 - cond_ratio / params_.fast_plane_cache_eig_threshold);
    Eigen::Matrix<double,6,1> w_diag;
    for (int i = 0; i < 6; ++i)
        w_diag(i) = std::pow(es.eigenvalues()(i) / (lambda_max + 1e-9), eig_power_dyn);
    Eigen::Matrix<double,6,1> delta = es.eigenvectors() *
                                      w_diag.asDiagonal() *
                                      es.eigenvectors().transpose() * delta_raw;
    delta *= trust;

    // Store stale fraction for LiDAR-based mid-tier scheduling
    last_fast_stale_fraction_ = (effct_accepted < static_cast<int>(plane_cache_.size()))
        ? (1.0 - static_cast<double>(effct_accepted) / plane_cache_.size()) : 0.0;

    predicted.pos += delta.head<3>();
    V3D dr = delta.tail<3>();
    double angle = dr.norm();
    if (angle > 1e-10) {
        Eigen::AngleAxisd aa(angle, dr / angle);
        predicted.rot = predicted.rot * Eigen::Quaterniond(aa);
        predicted.rot.normalize();
    }
}

// ---------------------------------------------------------------------------
// Voxel-hash fast-tier GN: query the incrementally-built world-frame voxel
// map against FRESH points from the current fast-tier window (point_buf_
// .get_window(last_fast_freq_t_, t_now)) — unlike apply_plane_cache_gn (frozen
// body-frame points from the last mid-tier cycle) and unlike the ikd-tree
// decoupled-3x3-GN path (real search, but O(log n) per point every ~10ms),
// this is O(1) amortized per point and incorporates genuinely new LiDAR
// returns every fast-tier cycle. Mirrors the decoupled-3x3-GN path's solve
// (single-step GN, same clamping/eigenvalue-conditioning), only the
// correspondence source differs, so the ablation isolates that one variable.
// ---------------------------------------------------------------------------
void MfLioEstimator::apply_voxel_hash_gn(state_ikfom& predicted, double t_now,
                                          double dtheta, double dv) {
    auto raw_pts = point_buf_.get_window(last_fast_freq_t_, t_now);
    if (jitter_log_enabled_) {
        jitter_log_ << "vh_entry," << t_now << ',' << raw_pts.size() << ','
                    << (jitter_log_enabled_ ? 1 : 0) << "\n";
    }
    if (raw_pts.empty()) return;

    const double anchor_t    = std::max(last_mid_freq_t_, last_fast_commit_t_);
    const double t_win_start = last_fast_freq_t_;
    const double t_win_len   = std::max(t_now - last_fast_freq_t_, 1e-9);
    const int    n_raw       = static_cast<int>(raw_pts.size());
    const int    stride      = std::max(1, n_raw / 300);
    const int    npts        = (n_raw + stride - 1) / stride;
    if (jitter_log_enabled_) jitter_log_ << "vh_npts," << npts << ',' << params_.fast_map_min_points << "\n";
    if (npts < params_.fast_map_min_points) return;

    state_ikfom kf_state = kf_.get_x();
    const state_ikfom x_ws = imu_proc_.predict_state(anchor_t, last_fast_freq_t_, kf_state);
    const state_ikfom x_we = imu_proc_.predict_state(anchor_t, t_now, kf_state);

    const M3D R_end   = predicted.rot.toRotationMatrix();
    const M3D R_L     = predicted.offset_R_L_I.toRotationMatrix();
    const V3D T_L     = predicted.offset_T_L_I;
    const V3D pos_end = predicted.pos;

    Eigen::Matrix<double,6,6> HtH = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,1> Htr = Eigen::Matrix<double,6,1>::Zero();
    int effct = 0;
    int voxel_hits = 0, voxel_miss = 0, voxel_rejected = 0;
    double max_accepted_pd = 0.0, max_accepted_range = 0.0;
    double sum_accepted_range = 0.0;

    {
        std::lock_guard<std::mutex> lk(voxel_hash_mutex_);
        for (int ki = 0; ki < npts; ++ki) {
            const auto& sp = raw_pts[ki * stride];
            const double alpha = std::max(0.0, std::min(1.0,
                (sp.timestamp - t_win_start) / t_win_len));
            const V3D pos_p = (1.0 - alpha) * x_ws.pos + alpha * pos_end;
            const Eigen::Quaterniond rot_p = x_ws.rot.slerp(alpha, predicted.rot);
            const M3D R_p = rot_p.toRotationMatrix();
            const V3D pb(sp.point.x, sp.point.y, sp.point.z);
            const V3D p_imu_raw = R_L * pb + T_L;
            const V3D pw_u = R_p * p_imu_raw + pos_p;
            const V3D p_imu_canon = R_end.transpose() * (pw_u - pos_end);

            const VoxelPlaneEntry* pe = voxel_hash_.query(pw_u);
            if (!pe) { ++voxel_miss; continue; }
            ++voxel_hits;
            const double pd = pe->normal.dot(pw_u) + pe->offset;
            const float  s_w = 1.0f - 0.9f * static_cast<float>(std::abs(pd))
                               / std::sqrt(static_cast<float>(p_imu_canon.norm() + 1e-6));
            if (s_w < 0.9f) { ++voxel_rejected; continue; }

            M3D p_imu_cross; p_imu_cross << SKEW_SYM_MATRX(p_imu_canon);
            const V3D A = p_imu_cross * (R_end.transpose() * pe->normal);
            Eigen::Matrix<double,1,6> J;
            J.block<1,3>(0,0) = pe->normal.transpose();
            J.block<1,3>(0,3) = A.transpose();
            const double r = -pd;
            HtH += J.transpose() * J;
            Htr -= J.transpose() * r;
            ++effct;
            const double rng = p_imu_canon.norm();
            if (std::abs(pd) > max_accepted_pd) max_accepted_pd = std::abs(pd);
            if (rng > max_accepted_range) max_accepted_range = rng;
            sum_accepted_range += rng;
        }
    }
    if (jitter_log_enabled_) {
        size_t vh_cellcount = 0;
        { std::lock_guard<std::mutex> lk2(voxel_hash_mutex_); vh_cellcount = voxel_hash_.size(); }
        jitter_log_ << "vh_cycle," << t_now << ',' << npts << ',' << voxel_hits << ','
                    << voxel_miss << ',' << voxel_rejected << ',' << effct << ','
                    << max_accepted_pd << ',' << max_accepted_range << ','
                    << (effct > 0 ? sum_accepted_range / effct : 0.0) << ','
                    << vh_cellcount << "\n";
    }

    const double trust_rot_v = std::exp(-params_.fast_plane_cache_rot_decay   * dtheta);
    const double trust_tra_v = std::exp(-params_.fast_plane_cache_trans_decay * dv);
    const double trust_cnt_v = std::min(1.0, static_cast<double>(effct)
                                              / params_.fast_plane_cache_count_full);
    const double trust_v = trust_rot_v * trust_tra_v * trust_cnt_v;
    if (jitter_log_enabled_) {
        jitter_log_ << "vh_trust," << t_now << ',' << effct << ',' << trust_rot_v << ','
                    << trust_tra_v << ',' << trust_cnt_v << ',' << trust_v << ','
                    << (effct < params_.fast_map_min_points || trust_v < params_.fast_plane_cache_min_trust ? 1 : 0)
                    << "\n";
    }
    if (effct < params_.fast_map_min_points || trust_v < params_.fast_plane_cache_min_trust)
        return;

    Eigen::Vector3d delta_pos;
    Eigen::Vector3d delta_rot = Eigen::Vector3d::Zero();
    if (params_.fast_6dof_enable) {
        const double s_R = params_.fast_norm_rot_scale;
        Eigen::Matrix<double,6,1> S_vec;
        S_vec << 1.0, 1.0, 1.0, s_R, s_R, s_R;
        Eigen::Matrix<double,6,6> Hn = S_vec.asDiagonal() * HtH * S_vec.asDiagonal();
        Eigen::Matrix<double,6,1> gn = S_vec.asDiagonal() * Htr;
        Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd6(Hn,
            Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<double,6,1> sv6 = svd6.singularValues();
        for (int i = 0; i < 6; ++i)
            sv6(i) = (sv6(i) > params_.fast_plane_cache_min_eig) ? 1.0 / sv6(i) : 0.0;
        Eigen::Matrix<double,6,1> xi_n =
            svd6.matrixV() * sv6.asDiagonal() * svd6.matrixU().transpose() * gn;
        Eigen::Matrix<double,6,1> xi = S_vec.asDiagonal() * xi_n;
        delta_pos = xi.head<3>() * trust_v;
        delta_rot = xi.tail<3>() * trust_v;
    } else {
        auto pinv3 = [&](const Eigen::Matrix3d& A3) -> Eigen::Matrix3d {
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(A3,
                Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d sv = svd.singularValues();
            for (int i = 0; i < 3; ++i)
                sv(i) = (sv(i) > params_.fast_plane_cache_min_eig) ? 1.0 / sv(i) : 0.0;
            return svd.matrixV() * sv.asDiagonal() * svd.matrixU().transpose();
        };
        delta_pos = pinv3(HtH.block<3,3>(0,0)) * Htr.head<3>() * trust_v;
    }

    fast_delta_xi_pos_ += delta_pos;
    const double afc_max = 0.5;
    if (fast_delta_xi_pos_.norm() > afc_max)
        fast_delta_xi_pos_ = fast_delta_xi_pos_.normalized() * afc_max;
    // RDEF/CI diagnostic: same bookkeeping as the decoupled-3x3-GN path above.
    if (params_.rdef_ci_diagnostic_enable)
        Lambda_F_pos_ += HtH.block<3,3>(0,0);
    if (params_.fast_6dof_enable) {
        fast_delta_xi_rot_ += delta_rot;
        const double rot_max = 0.1;
        if (fast_delta_xi_rot_.norm() > rot_max)
            fast_delta_xi_rot_ = fast_delta_xi_rot_.normalized() * rot_max;
    }
    if (jitter_log_enabled_) {
        jitter_log_ << "vh_applied," << t_now << ',' << trust_v << ',' << effct << ','
                    << delta_pos.x() << ',' << delta_pos.y() << ',' << delta_pos.z() << ','
                    << delta_pos.norm() << ',' << fast_delta_xi_pos_.norm() << ','
                    << delta_rot.norm() << ',' << fast_delta_xi_rot_.norm() << "\n";
    }
}

// ---------------------------------------------------------------------------
// Graph update: add a new keyframe to the sliding window and run GN.
// Called at the end of each mid/low iEKF.  Returns true if the graph
// produced an accepted refinement (state_point_ / kf_ are updated).
// ---------------------------------------------------------------------------
bool MfLioEstimator::run_graph_update(double t_now) {
    if (!params_.graph_enable || captured_H_.rows() == 0) return false;

    // Build IMU constraint from last keyframe to now.
    ImuConstraint imu;
    imu.sigma_pos = params_.graph_imu_sigma_pos;
    imu.sigma_rot = params_.graph_imu_sigma_rot;

    if (graph_initialized_) {
        imu.dt = t_now - last_graph_kf_t_;
        state_ikfom x_pred = imu_proc_.predict_state(
            last_graph_kf_t_, t_now, last_graph_kf_x_);
        imu.delta_pos = last_graph_kf_x_.rot.conjugate()
                        * (x_pred.pos - last_graph_kf_x_.pos);
        imu.delta_rot = last_graph_kf_x_.rot.inverse() * x_pred.rot;
    }

    // Build graph keyframe from current iEKF result.
    GraphKeyframe gkf;
    gkf.timestamp = t_now;
    gkf.x         = state_point_;
    gkf.H_6       = captured_H_.leftCols(6);  // M×6 (pos, rot columns)
    gkf.r         = captured_r_;
    gkf.sigma2    = LASER_POINT_COV;
    gkf.has_measurement = (gkf.H_6.rows() > 0);

    graph_opt_.add_keyframe(gkf, imu);
    graph_initialized_ = true;
    last_graph_kf_x_   = state_point_;
    last_graph_kf_t_   = t_now;

    if (graph_opt_.window_size() < 2) return false;

    graph_opt_.optimize(params_.graph_max_iter);

    const state_ikfom& refined = graph_opt_.get_latest();

    // Safety gate: reject large corrections (linearisation error).
    const double dp = (refined.pos - state_point_.pos).norm();
    const double dr = Eigen::AngleAxisd(
        state_point_.rot.conjugate() * refined.rot).angle() * 180.0 / M_PI;

    if (dp > params_.graph_max_correction_m ||
        dr > params_.graph_max_correction_deg) {
        return false;  // iEKF result stands
    }

    // Accept: keep vel/bg/ba/grav from iEKF, update pos/rot from graph.
    state_ikfom refined_full = state_point_;
    refined_full.pos = refined.pos;
    refined_full.rot = refined.rot;
    kf_.change_x(refined_full);
    state_point_     = refined_full;
    last_graph_kf_x_ = state_point_;

    return true;
}

// ---------------------------------------------------------------------------
// Mid-frequency update: adaptive 10–20 Hz, window = current_W_m_ms_, MAP WRITE.
// FAST-LIO2-equivalent tier — keeps the map current at the adaptive mid rate.
// t_start tracks current_W_m_ms_ so consecutive scans are non-overlapping;
// this prevents double-counting Fisher info in H_m_accumulated_ for the IGR.
// ---------------------------------------------------------------------------
void MfLioEstimator::mid_freq_update(double t_now) {
    double t_start = params_.shared_origin_scheduling_enable
        ? shared_origin_t_
        : (t_now - current_W_m_ms_ * 1e-3);
    auto scan = collect_and_undistort(t_start, t_now);
    if (!scan || scan->empty()) return;

    // Apply fast-tier δpos corrections to undistorted points before iEKF.
    {
        std::lock_guard<std::mutex> lk(fast_delta_mutex_);
        while (!fast_delta_cache_.empty() &&
               fast_delta_cache_.front().t < t_start - 0.01)
            fast_delta_cache_.pop_front();
    }
    if (params_.fast_undistort_enable)
        apply_fast_undistort(scan, t_start);

    // Stride pre-filter (matches FL2's point_filter_num=3 applied before voxel filter).
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_strided = scan;
    if (params_.point_filter_num > 1) {
        scan_strided = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
        scan_strided->reserve(scan->size() / params_.point_filter_num + 1);
        for (std::size_t i = 0; i < scan->size(); ++i) {
            if (i % static_cast<std::size_t>(params_.point_filter_num) == 0)
                scan_strided->push_back((*scan)[i]);
        }
    }

    auto scan_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    scan_ds->points.reserve(scan_strided->size() + 128);
    vox_fast_filter_.setInputCloud(scan_strided);   // scan density: filter_size_scan (or filter_size_map)
    vox_fast_filter_.filter(*scan_ds);
    if (jitter_log_enabled_) {
        jitter_log_cycle("mid", ++jitter_mid_call_idx_, t_now, t_start,
                         static_cast<int>(scan->size()), static_cast<int>(scan_ds->size()));
    }
    if (scan_ds->empty()) return;

    if (ikdtree_.Root_Node == nullptr) {
        // First frame: seed map in world frame
        ikdtree_.set_downsample_param(params_.filter_size_map);
        state_point_ = kf_.get_x();
        PointVector pts;
        pts.reserve(scan_ds->size());
        for (const auto& p : scan_ds->points) {
            V3D pb(p.x, p.y, p.z);
            V3D pw = state_point_.rot
                     * (state_point_.offset_R_L_I * pb + state_point_.offset_T_L_I)
                     + state_point_.pos;
            PointType pw_pt;
            pw_pt.x = static_cast<float>(pw(0));
            pw_pt.y = static_cast<float>(pw(1));
            pw_pt.z = static_cast<float>(pw(2));
            pw_pt.intensity = p.intensity;
            pts.push_back(pw_pt);
        }
        ikdtree_.Build(pts);
    } else {
        // v19v4 dual-state: initialize fine iEKF from fast_state_-corrected pose.
        // fast_state_ carries accumulated coarse corrections since last mid-tier.
        if (params_.fast_state_seed_mid_enable && fast_state_valid_) {
            state_ikfom x_kf_now = kf_.get_x();
            state_ikfom x_init   = x_kf_now;
            x_init.pos = x_kf_now.pos + (fast_state_.pos - prev_kf_imu_.pos);
            x_init.rot = (fast_state_.rot * prev_kf_imu_.rot.inverse()) * x_kf_now.rot;
            x_init.vel = x_kf_now.vel + (fast_state_.vel - prev_kf_imu_.vel);
            kf_.change_x(x_init);
        }
        state_ikfom pre_upd = kf_.get_x();
        auto pre_cov = kf_.get_P();
        meas_cloud_  = scan_ds;
        meas_size_   = static_cast<int>(scan_ds->size());
        nearest_points_.resize(meas_size_);
        reset_update_diagnostics();
        h_share_call_count_ = 0;
        last_pos_hth_valid_ = false;
        double solve_t = 0.0;
        captured_H_.resize(0, 0);
        captured_r_.resize(0);
        capture_h_r_on_converge_ = params_.graph_enable;
        if (params_.riekf_enable) run_riekf_correction();
        else kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_t);
        capture_h_r_on_converge_ = false;
        state_point_ = kf_.get_x();

        // AMR: adapt mid-tier window — RMSE (reactive) blended with omega EMA (predictive).
        // high omega → fire faster (W_m_min); low omega + good RMSE → slower (W_m_max).
        {
            const double rmse_good = params_.low_freq_residual_good;
            const double rmse_bad  = params_.low_freq_residual_bad;
            const double denom = rmse_bad - rmse_good;
            const double q_rmse = denom > 1e-9
                ? std::clamp((rmse_bad - last_residual_rmse_) / denom, 0.0, 1.0)
                : 1.0;
            const double omega_denom = params_.W_m_omega_hi - params_.W_m_omega_lo;
            const double q_omega = omega_denom > 1e-9
                ? std::clamp(1.0 - (omega_ema_ - params_.W_m_omega_lo) / omega_denom, 0.0, 1.0)
                : 1.0;
            const double q = params_.W_m_alpha * q_rmse + (1.0 - params_.W_m_alpha) * q_omega;
            current_W_m_ms_ = params_.W_m_min_ms
                             + (params_.W_m_max_ms - params_.W_m_min_ms) * q;
        }

        if (params_.mid_freq_min_p_improvement > 0.0) {
            const double pre_pos_trace  = pre_cov.block<3,3>(0,0).trace();
            const double post_pos_trace = kf_.get_P().block<3,3>(0,0).trace();
            const double improvement    = (pre_pos_trace - post_pos_trace) / pre_pos_trace;
            if (improvement < params_.mid_freq_min_p_improvement) {
                kf_.change_x(pre_upd);
                kf_.change_P(pre_cov);
                state_point_ = pre_upd;
                sync_riekf_from_kf();
                return;
            }
        }

        // v48: project out unobservable position DOFs before accepting the update.
        // Eigendecompose H_pos^T*H_pos; zero position correction along eigenvectors
        // with eigenvalue below mid_degen_eigen_thresh (e.g. x/y over flat ground).
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_mid;
        bool mid_is_degenerate = false;
        const bool proj_window_active =
            params_.mid_degen_project_max_sec <= 0.0 ||
            (t_now - init_time_) < params_.mid_degen_project_max_sec;
        if (params_.mid_degen_project_enable && last_pos_hth_valid_ && proj_window_active) {
            eig_mid.compute(last_pos_HtH_);
            mid_is_degenerate = (eig_mid.eigenvalues()(0) < params_.mid_degen_eigen_thresh);
            if (mid_is_degenerate) {
                Eigen::Vector3d raw_correction = state_point_.pos - pre_upd.pos;
                Eigen::Vector3d projected = Eigen::Vector3d::Zero();
                for (int k = 0; k < 3; k++) {
                    if (eig_mid.eigenvalues()(k) >= params_.mid_degen_eigen_thresh) {
                        Eigen::Vector3d ev = eig_mid.eigenvectors().col(k);
                        projected += ev * ev.dot(raw_correction);
                    }
                }
                state_point_.pos = pre_upd.pos + projected;
                kf_.change_x(state_point_);
            }
        }

        const double dt = std::max(t_now - last_mid_freq_t_, params_.W_m_ms * 1e-3);
        const bool accept_mid_motion =
            should_accept_update_motion(pre_upd, state_point_, dt);

        if (!accept_mid_motion) {
            kf_.change_x(pre_upd);
            kf_.change_P(pre_cov);
            state_point_ = pre_upd;
            sync_riekf_from_kf();
            return;
        }

        // Graph refinement: add keyframe and jointly optimise the window.
        // state_point_ / kf_ are updated in-place if graph accepts correction.
        run_graph_update(t_now);

        // MAP WRITE: add points to ikd-Tree in world frame.
        // v48: skip map write when position update is geometrically degenerate —
        // prevents ill-conditioned states from corrupting the shared map.
        const bool skip_map_write =
            params_.mid_degen_map_gate_enable && last_pos_hth_valid_ &&
            [&]() -> bool {
                if (params_.mid_degen_project_enable) {
                    return mid_is_degenerate;
                }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_gate(last_pos_HtH_);
                return eig_gate.eigenvalues()(0) < params_.mid_degen_eigen_thresh;
            }();

        PointVector new_pts;
        new_pts.reserve(scan_ds->size());
        for (const auto& p : scan_ds->points) {
            V3D pb(p.x, p.y, p.z);
            V3D pw = state_point_.rot
                     * (state_point_.offset_R_L_I * pb + state_point_.offset_T_L_I)
                     + state_point_.pos;
            PointType pw_pt;
            pw_pt.x = static_cast<float>(pw(0));
            pw_pt.y = static_cast<float>(pw(1));
            pw_pt.z = static_cast<float>(pw(2));
            pw_pt.intensity = p.intensity;
            new_pts.push_back(pw_pt);
        }
        if (!skip_map_write) {
            ikdtree_.Add_Points(new_pts, true);
            // v19v3: refresh coarse map at mid-tier rate (10Hz) for fresher fast-tier correspondences
            if (params_.async_100hz_enable && coarse_ikdtree_.Root_Node != nullptr)
                coarse_ikdtree_.Add_Points(new_pts, true);
        }

        // Option C: capture posterior covariance for low-freq gating.
        // Option F: snapshot state for motion accumulation tracking.
        // P rows/cols: 0-2=pos, 3-5=rot, 6-8=vel (IKFoM error-state ordering).
        // v19v4: sync fast tracking state from authoritative fine estimate.
        // Resets delta reference so next fast-tier starts with delta=0.
        fast_state_       = kf_.get_x();
        prev_kf_imu_      = kf_.get_x();
        fast_state_valid_ = true;

        // RDEF/CI fusion diagnostic (disclosed negative, see paper Sec.
        // "Fast/Mid Fusion: RDEF and Covariance Intersection"). MUST run
        // before fast_delta_xi_pos_/Lambda_F_pos_ are reset below, so that
        // Lambda_F_pos_ still reflects everything accumulated since the
        // shared prior pre_cov (captured at the top of this function, before
        // this mid-tier update ran). Strictly side-channel: reads pre_cov/
        // kf_.get_P()/Lambda_F_pos_ but never writes back into kf_,
        // state_point_, or anything else that feeds the published output.
        if (params_.rdef_ci_diagnostic_enable) {
            const Eigen::Matrix3d P0_pos       = pre_cov.block<3,3>(0,0);
            const Eigen::Matrix3d P_corr_M_pos = kf_.get_P().block<3,3>(0,0);
            const auto rdef = mf_lio::rdef_ci::compute_rdef_fusion(
                P0_pos, Lambda_F_pos_, P_corr_M_pos);
            const auto ci = mf_lio::rdef_ci::compute_ci_fusion(
                rdef.P_corr_F, P_corr_M_pos, 101);
            if (!params_.rdef_ci_diagnostic_log_path.empty()) {
                std::ofstream rdef_ci_ofs(params_.rdef_ci_diagnostic_log_path, std::ios::app);
                if (rdef_ci_ofs.is_open()) {
                    rdef_ci_ofs << std::setprecision(17) << t_now << ','
                                << std::setprecision(9)
                                << rdef.P_fused_rdef.trace() << ','
                                << ci.P_fused_ci.trace() << ','
                                << P_corr_M_pos.trace() << ','
                                << ci.omega << "\n";
                }
            }
        }

        fast_delta_xi_pos_ = V3D::Zero();  // §4: reset anchor-relative delta at each anchor
        fast_delta_xi_rot_ = V3D::Zero();  // §5: reset anchor-relative rotation at each anchor
        Lambda_F_pos_.setZero();  // RDEF/CI diagnostic: reset accumulated Λ_F at each anchor

        last_mid_pos_cov_trace_    = kf_.get_P().block<3,3>(0,0).trace();
        last_mid_freq_state_       = state_point_;
        last_mid_freq_state_valid_ = true;

        // IGR: accumulate position Fisher information from this mid-tier update.
        if (last_pos_hth_valid_) {
            const double w_scale = current_W_m_ms_ / params_.W_m_ms;
            H_m_accumulated_ += last_pos_HtH_ * w_scale;
        }

        {
            bool stable = is_stable_publish_update();
            if (stable) {
                first_update_done_ = true;
                stable_update_done_ = true;
            }
        }
        if (can_publish(t_now)) {
            publish_corrected_if_safe(pre_upd, state_point_, t_now);
            // Backward propagation: record published mid-tier pose in history ring buffer.
            if (params_.backward_correction_enable) {
                pose_history_.push_back({t_now, state_point_});
                while (!pose_history_.empty()
                       && t_now - pose_history_.front().t > params_.W_l_max_ms * 1e-3 * 1.5)
                    pose_history_.pop_front();
            }
        }

        // RIEKF ablation: re-seed riekf_filter_ from kf_'s FINAL state for
        // this cycle -- whatever the v48 guard / mid_freq_min_p_improvement /
        // motion-acceptance / graph refinement settled on. One call here
        // (not scattered after each individual change_x above) covers every
        // mutation path uniformly.
        sync_riekf_from_kf();
    }

    // Trim raw point buffer. When low-tier is disabled (W_l >> sequence length),
    // the low_freq_update clear_before() never runs and the buffer grows without bound,
    // causing get_window() to scan O(total_runtime × rate) points each call.
    // Cap retention at 10× mid-tier window to keep buffer size bounded.
    {
        const double retention_s = std::min(policy_.W_l_ms(),
                                             std::max(params_.W_m_ms, params_.W_f_ms) * 10.0)
                                     * 1e-3 * 1.1;
        point_buf_.clear_before(t_now - retention_s);
    }
}

// ---------------------------------------------------------------------------
// Low-frequency update: 5 Hz, 200 ms window, all downsampled points, read-only.
// Dense scan refines state; mid_freq owns the map.
// Also updates adaptive policy from D_degenerate.
// ---------------------------------------------------------------------------
void MfLioEstimator::low_freq_update(double t_now) {
    // If the map hasn't been seeded yet, delegate to mid_freq_update first.
    // This prevents the low-tier timer from blocking map initialization when IMU init
    // delays the first two mid-tier calls (low_trigger fires at W_l before mid can seed map).
    if (!params_.async_100hz_enable && ikdtree_.Root_Node == nullptr) {
        mid_freq_update(t_now);
        return;
    }

    // Apply any pending loop closure correction before the iEKF iteration.
    // Applying here (before iEKF) means this scan sees the corrected state AND corrected map.
    if (params_.loop_closure_enable) {
        Eigen::Matrix4f T_corr;
        bool has_corr = false;
        {
            std::lock_guard<std::mutex> lk(lc_mutex_);
            if (lc_has_correction_) {
                T_corr = lc_correction_;
                lc_has_correction_ = false;
                has_corr = true;
            }
        }
        if (has_corr) apply_loop_correction(T_corr);
    }

    double t_start = t_now - current_W_l_ms_ * 1e-3;
    auto scan = collect_and_undistort(t_start, t_now);
    if (!scan || scan->empty()) return;

    auto scan_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    scan_ds->points.reserve(scan->size() + 128);
    vox_fast_filter_.setInputCloud(scan);   // scan density: filter_size_scan (or filter_size_map)
    vox_fast_filter_.filter(*scan_ds);
    if (jitter_log_enabled_) {
        jitter_log_cycle("low", ++jitter_low_call_idx_, t_now, t_start,
                         static_cast<int>(scan->size()), static_cast<int>(scan_ds->size()));
    }
    if (scan_ds->empty()) return;

    auto features_l = extractor_.extract(scan_ds);

    // v19 async: low-tier owns fine map initialization (coarse map was built by fast-tier)
    if (params_.async_100hz_enable && params_.low_freq_map_write_enable &&
        ikdtree_.Root_Node == nullptr && coarse_ikdtree_.Root_Node != nullptr) {
        ikdtree_.set_downsample_param(static_cast<float>(params_.filter_size_map));
        PointVector init_pts;
        init_pts.reserve(scan_ds->size());
        for (const auto& p : scan_ds->points) {
            const V3D pb(p.x, p.y, p.z);
            const V3D pw = state_point_.rot * (state_point_.offset_R_L_I * pb
                           + state_point_.offset_T_L_I) + state_point_.pos;
            PointType wp; wp.x=static_cast<float>(pw.x());
            wp.y=static_cast<float>(pw.y()); wp.z=static_cast<float>(pw.z());
            init_pts.push_back(wp);
        }
        ikdtree_.Build(init_pts);
        sync_riekf_from_kf();
        return;  // skip iEKF on init call (no existing map to match against)
    }

    state_ikfom correction_publish_ref = state_point_;
    if (ikdtree_.Root_Node != nullptr) {
        // v19v4 dual-state: initialize fine iEKF from fast_state_-corrected pose (same as mid-tier).
        if (params_.fast_state_seed_mid_enable && fast_state_valid_) {
            state_ikfom x_kf_now = kf_.get_x();
            state_ikfom x_init   = x_kf_now;
            x_init.pos = x_kf_now.pos + (fast_state_.pos - prev_kf_imu_.pos);
            x_init.rot = (fast_state_.rot * prev_kf_imu_.rot.inverse()) * x_kf_now.rot;
            x_init.vel = x_kf_now.vel + (fast_state_.vel - prev_kf_imu_.vel);
            kf_.change_x(x_init);
        }
        state_ikfom pre_upd = kf_.get_x();
        correction_publish_ref = pre_upd;
        auto pre_cov = kf_.get_P();

        // Capture and reset mid-tier accumulated Fisher info for IGR computation below.
        Eigen::Matrix3d H_m_captured = H_m_accumulated_;
        H_m_accumulated_.setZero();

        in_low_freq_ = true;
        reset_update_diagnostics();
        h_share_call_count_ = 0;
        last_pos_hth_valid_ = false;
        meas_cloud_  = scan_ds;
        meas_size_   = static_cast<int>(scan_ds->size());
        nearest_points_.resize(meas_size_);
        double solve_t = 0.0;
        captured_H_.resize(0, 0);
        captured_r_.resize(0);
        capture_h_r_on_converge_ = params_.graph_enable;
        if (params_.riekf_enable) run_riekf_correction();
        else kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_t);
        capture_h_r_on_converge_ = false;
        in_low_freq_ = false;
        state_point_ = kf_.get_x();

        // v48: project out degenerate position DOFs in low-freq update too,
        // mirroring the mid-freq projection so flat-ground can't corrupt state
        // via this path either (same params and threshold reused).
        const bool low_proj_window_active =
            params_.mid_degen_project_max_sec <= 0.0 ||
            (t_now - init_time_) < params_.mid_degen_project_max_sec;
        if (params_.mid_degen_project_enable && last_pos_hth_valid_ && low_proj_window_active) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_low;
            eig_low.compute(last_pos_HtH_);
            if (eig_low.eigenvalues()(0) < params_.mid_degen_eigen_thresh) {
                Eigen::Vector3d raw_corr = state_point_.pos - pre_upd.pos;
                Eigen::Vector3d proj = Eigen::Vector3d::Zero();
                for (int k = 0; k < 3; k++) {
                    if (eig_low.eigenvalues()(k) >= params_.mid_degen_eigen_thresh) {
                        Eigen::Vector3d ev = eig_low.eigenvectors().col(k);
                        proj += ev * ev.dot(raw_corr);
                    }
                }
                state_point_.pos = pre_upd.pos + proj;
                kf_.change_x(state_point_);
            }
        }

        auto post_cov = kf_.get_P();
        const double update_dt = std::max(current_W_l_ms_ * 1e-3, 1e-3);

        // Low-frequency fusion guard. Hard limits catch totally degenerate scans;
        // per-axis information-filter fusion handles principled blending.
        double quality = last_quality_;
        const double pos_delta = (state_point_.pos - pre_upd.pos).norm();
        Eigen::AngleAxisd rot_delta(pre_upd.rot.conjugate() * state_point_.rot);
        const double rot_delta_deg = std::abs(rot_delta.angle()) * 180.0 / M_PI;

        // Scalar IGR safety gate: tr(H_l) / (tr(H_l) + tr(H_m)).
        // If total low-tier information is negligible, reject outright without
        // computing the per-axis decomposition.
        double scalar_alpha_l = 0.5;
        if (last_pos_hth_valid_) {
            const double tr_hl = last_pos_HtH_.trace();
            const double tr_hm = H_m_captured.trace();
            const double denom  = tr_hl + tr_hm;
            if (denom > 1e-12) scalar_alpha_l = tr_hl / denom;
        }

        const bool motion_ok = should_accept_update_motion(pre_upd, state_point_, update_dt);
        const bool reject_low_update =
            last_effct_ < params_.low_freq_min_correspondences ||
            quality < params_.low_freq_min_quality ||
            (params_.min_correspondence_quality > 0.0 &&
             quality < params_.min_correspondence_quality) ||
            last_residual_rmse_ > params_.low_freq_max_residual_rmse ||
            pos_delta > params_.low_freq_max_pos_delta ||
            rot_delta_deg > params_.low_freq_max_rot_delta_deg ||
            !motion_ok;
        // Diagnostic-only accumulators for MFLIO_LOWTIER_LOG (see below); harmless
        // when logging is disabled, never read by the actual estimator logic.
        Eigen::Vector3d log_lam_l = Eigen::Vector3d::Zero();
        Eigen::Vector3d log_alpha_k = Eigen::Vector3d::Zero();
        double log_corr_raw_norm = 0.0;
        double log_corr_applied_norm = 0.0;
        Eigen::Vector3d log_corr_applied_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d log_corr_applied_body  = Eigen::Vector3d::Zero();
        bool log_trust_reject = false;
        bool log_low_trust_floor_reject = false;
        // Diagnostic-only: eigenvalues of the mid-tier's own accumulated Fisher info
        // (H_m_captured, captured above before this low-tier attempt began). Used to
        // characterize whether mid-tier was already well-conditioned in all directions
        // at firing time -- the basis for a future degeneracy-gated firing policy.
        Eigen::Vector3d log_hm_eig = Eigen::Vector3d::Zero();
        if (lowtier_log_enabled_) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_hm(H_m_captured);
            log_hm_eig = eig_hm.eigenvalues();
        }

        bool accepted_low_update = true;
        if (!params_.low_freq_trust_enable) {
            if (reject_low_update) {
                kf_.change_x(pre_upd);
                kf_.change_P(pre_cov);
                state_point_ = pre_upd;
                accepted_low_update = false;
            }
        } else if (reject_low_update || scalar_alpha_l <= params_.low_freq_trust_reject) {
            log_trust_reject = !reject_low_update && scalar_alpha_l <= params_.low_freq_trust_reject;
            kf_.change_x(pre_upd);
            kf_.change_P(pre_cov);
            state_point_ = pre_upd;
            accepted_low_update = false;
        } else if (last_pos_hth_valid_) {
            // Per-axis information-filter fusion.
            //
            // H_l = sum_i(w_i * n_i * n_i^T): position Fisher info from the 200ms low-tier
            // window, already temporally-weighted by h_share_model (sqrt_w scales h_x rows).
            // Eigendecompose H_l to identify which position directions are well-constrained.
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(last_pos_HtH_);
            const Eigen::Matrix3d& V     = eig.eigenvectors();
            const Eigen::Vector3d& lam_l = eig.eigenvalues();  // ascending

            // Per-direction IGR: project mid-tier accumulated info onto each eigenvector of H_l.
            // alpha_k = lambda_l_k / (lambda_l_k + v_k^T * H_m * v_k)
            // alpha_k ≈ 1: low-tier dominates this direction → apply full correction.
            // alpha_k ≈ 0: mid-tier already constrained this direction → suppress correction.
            Eigen::Vector3d alpha_k;
            for (int k = 0; k < 3; k++) {
                const double lm = V.col(k).dot(H_m_captured * V.col(k));
                const double ll = lam_l(k);
                const double d  = ll + lm;
                alpha_k(k) = (d > 1e-12) ? std::clamp(ll / d, 0.0, 1.0) : 0.0;
            }

            // Ablation-only: replace the continuous per-axis blend with a hard
            // per-axis accept/reject. An axis clearing the threshold gets the
            // low tier's full raw correction instead of a damped fraction of
            // it; an axis below threshold gets none at all instead of a small
            // biased nudge. Default off (threshold check skipped) preserves
            // the original continuous alpha_k values.
            if (params_.low_freq_axis_gate_enable) {
                for (int k = 0; k < 3; k++) {
                    alpha_k(k) = (alpha_k(k) >= params_.low_freq_axis_gate_thresh) ? 1.0 : 0.0;
                }
            }

            // Ablation-only: if even the best-trusted direction is below the floor,
            // skip the correction entirely (full rollback) instead of applying a
            // heavily-damped update. Default floor 0.0 never triggers (preserves
            // existing always-damp-and-apply behavior).
            if (params_.low_freq_min_trust_alpha > 0.0 &&
                alpha_k.maxCoeff() < params_.low_freq_min_trust_alpha) {
                kf_.change_x(pre_upd);
                kf_.change_P(pre_cov);
                state_point_ = pre_upd;
                accepted_low_update = false;
                log_low_trust_floor_reject = true;
            } else {
            // 3×3 position blend matrix: A_pos = V * diag(alpha_k) * V^T.
            // Applies direction-specific trust to the iEKF position correction.
            const Eigen::Matrix3d A_pos = V * alpha_k.asDiagonal() * V.transpose();

            const Eigen::Vector3d dp = state_point_.pos - pre_upd.pos;
            state_point_.pos = pre_upd.pos + A_pos * dp;
            kf_.change_x(state_point_);

            // Diagnostic-only capture for MFLIO_LOWTIER_LOG.
            log_lam_l = lam_l;
            log_alpha_k = alpha_k;
            log_corr_raw_norm = dp.norm();
            const Eigen::Vector3d applied_vec = A_pos * dp;
            log_corr_applied_norm = applied_vec.norm();
            log_corr_applied_world = applied_vec;
            log_corr_applied_body  = state_point_.rot.conjugate() * applied_vec;

            // Covariance-consistent update via information filter for position block.
            // Ω_fused = Σ_pre_pos^{-1} + H_l_eff  →  Σ_pos_fused = Ω_fused^{-1}
            // where H_l_eff = V * diag(alpha_k * lambda_l_k) * V^T reflects only the
            // information we actually incorporated (zero for degenerate directions).
            // This is the exact Bayesian posterior; the old linear cov-blend was not.
            const Eigen::Matrix3d H_l_eff =
                V * (alpha_k.cwiseProduct(lam_l)).asDiagonal() * V.transpose();
            const Eigen::Matrix3d Sigma_pre_pos = pre_cov.block<3,3>(3,3);
            static constexpr double COV_EPS = 1e-12;
            const Eigen::Matrix3d Sigma_pos_fused =
                (Sigma_pre_pos.inverse() + H_l_eff +
                 COV_EPS * Eigen::Matrix3d::Identity()).inverse();

            // Use iEKF posterior for all DOFs (rotation, velocity, bias handled by iEKF gain);
            // replace only the position block with the information-filter posterior.
            auto fused_cov = post_cov;
            fused_cov.block<3,3>(3,3) = Sigma_pos_fused;
            kf_.change_P(fused_cov);
            }
        }

        // Diagnostic-only low-tier accept/reject log (MFLIO_LOWTIER_LOG). Computes
        // eigenvalues of last_pos_HtH_ independently of the fusion branch above so a
        // rejected attempt still gets diagnosed. No effect on estimation when disabled.
        if (lowtier_log_enabled_) {
            std::string reject_reason;
            if (!accepted_low_update) {
                if (last_effct_ < params_.low_freq_min_correspondences) reject_reason += "low_effct;";
                if (quality < params_.low_freq_min_quality) reject_reason += "low_quality;";
                if (params_.min_correspondence_quality > 0.0 &&
                    quality < params_.min_correspondence_quality) reject_reason += "low_quality_global;";
                if (last_residual_rmse_ > params_.low_freq_max_residual_rmse) reject_reason += "high_residual;";
                if (pos_delta > params_.low_freq_max_pos_delta) reject_reason += "big_pos_delta;";
                if (rot_delta_deg > params_.low_freq_max_rot_delta_deg) reject_reason += "big_rot_delta;";
                if (!motion_ok) reject_reason += "motion_reject;";
                if (log_trust_reject) reject_reason += "trust_reject;";
                if (log_low_trust_floor_reject) reject_reason += "low_trust_floor;";
                if (reject_reason.empty()) reject_reason = "unknown;";
            }
            Eigen::Vector3d eig_for_log = log_lam_l;
            if (eig_for_log.isZero(0) && last_pos_hth_valid_) {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_diag(last_pos_HtH_);
                eig_for_log = eig_diag.eigenvalues();
            }
            lowtier_log_ << std::setprecision(17) << t_now << ',' << (accepted_low_update ? 1 : 0)
                         << ',' << reject_reason << ',' << std::setprecision(9) << scalar_alpha_l
                         << ',' << last_effct_ << ',' << quality << ',' << last_residual_rmse_
                         << ',' << pos_delta << ',' << rot_delta_deg
                         << ',' << eig_for_log(0) << ',' << eig_for_log(1) << ',' << eig_for_log(2)
                         << ',' << log_alpha_k(0) << ',' << log_alpha_k(1) << ',' << log_alpha_k(2)
                         << ',' << log_corr_raw_norm << ',' << log_corr_applied_norm
                         << ',' << log_corr_applied_world(0) << ',' << log_corr_applied_world(1)
                         << ',' << log_corr_applied_world(2)
                         << ',' << log_corr_applied_body(0) << ',' << log_corr_applied_body(1)
                         << ',' << log_corr_applied_body(2)
                         << ',' << log_hm_eig(0) << ',' << log_hm_eig(1) << ',' << log_hm_eig(2) << '\n';
        }

        // Graph refinement after accepted low-tier update.
        if (accepted_low_update) {
            run_graph_update(t_now);
        }

        if (accepted_low_update && is_stable_publish_update()) {
            first_update_done_ = true;
            stable_update_done_ = true;
        }

        // v19 async: write the accumulated low-tier scan to both fine and coarse maps.
        if (params_.low_freq_map_write_enable && accepted_low_update) {
            PointVector low_world_pts;
            low_world_pts.reserve(scan_ds->size());
            for (const auto& p : scan_ds->points) {
                const V3D pb(p.x, p.y, p.z);
                const V3D pw = state_point_.rot * (state_point_.offset_R_L_I * pb
                               + state_point_.offset_T_L_I) + state_point_.pos;
                PointType wp; wp.x=static_cast<float>(pw.x());
                wp.y=static_cast<float>(pw.y()); wp.z=static_cast<float>(pw.z());
                low_world_pts.push_back(wp);
            }
            ikdtree_.Add_Points(low_world_pts, true);
            // Match mid-tier's own coarse-map write gate exactly (mid_freq_update's
            // analogous block): only touch coarse_ikdtree_ when async_100hz_enable is
            // on. Without this, low-tier would write into the coarse map even when
            // mid-tier never does under canonical (async-disabled) settings -- an
            // asymmetry with no relation to the fine-map-enrichment hypothesis being
            // tested, and an uncontrolled confound for fast-tier's coarse-map queries.
            if (params_.async_100hz_enable && coarse_ikdtree_.Root_Node != nullptr)
                coarse_ikdtree_.Add_Points(low_world_pts, true);
        }

        // Prototype (attempt #5): populate the independent low-tier-only map.
        // Uses the same accepted-update points as above but a SEPARATE tree that
        // mid-tier never writes to, so it reflects only the low tier's own
        // observation history rather than one continuously overwritten by
        // mid-tier's more frequent, drift-following corrections.
        if (params_.low_tier_own_map_enable && accepted_low_update) {
            PointVector low_own_pts;
            low_own_pts.reserve(scan_ds->size());
            for (const auto& p : scan_ds->points) {
                const V3D pb(p.x, p.y, p.z);
                const V3D pw = state_point_.rot * (state_point_.offset_R_L_I * pb
                               + state_point_.offset_T_L_I) + state_point_.pos;
                PointType wp; wp.x=static_cast<float>(pw.x());
                wp.y=static_cast<float>(pw.y()); wp.z=static_cast<float>(pw.z());
                low_own_pts.push_back(wp);
            }
            if (low_ikdtree_.Root_Node == nullptr) {
                low_ikdtree_.set_downsample_param(static_cast<float>(params_.filter_size_map));
                low_ikdtree_.Build(low_own_pts);
            } else {
                low_ikdtree_.Add_Points(low_own_pts, true);
            }
        }

        // v19v4: sync fast tracking state from accepted low-tier fine estimate.
        if (accepted_low_update) {
            fast_state_       = kf_.get_x();
            prev_kf_imu_      = kf_.get_x();
            fast_state_valid_ = true;
        }

        // Save IGR for W_l adaptation (used outside this block).
        last_scalar_alpha_l_ = scalar_alpha_l;

        // Cross-tier backward propagation: retroactively apply this low-tier SE3
        // correction to all mid-tier poses buffered since the last low-tier update.
        // Uses rigid-body approximation (valid for small Δx, which is always the case
        // at convergence). Only applies when the update was accepted.
        if (params_.backward_correction_enable && accepted_low_update
                && !pose_history_.empty()) {
            const auto delta_rot = pre_upd.rot.inverse() * state_point_.rot;
            const M3D  R_delta   = delta_rot.toRotationMatrix();
            for (auto& entry : pose_history_) {
                if (entry.t < last_low_freq_t_) continue;
                entry.state.pos = R_delta * (entry.state.pos - pre_upd.pos)
                                  + state_point_.pos;
                entry.state.rot = delta_rot * entry.state.rot;
            }
            if (odom_smooth_cb_) {
                for (const auto& entry : pose_history_)
                    odom_smooth_cb_(entry.state, entry.t);
            }
        }

        // RIEKF ablation: re-seed riekf_filter_ from kf_'s FINAL state for
        // this cycle. Unlike mid_freq_update, none of the three accept/
        // reject/fuse branches above return early -- they all fall through
        // to here -- so this single call uniformly covers all three.
        sync_riekf_from_kf();
    }

    // Save state for low-tier adaptive scheduling.
    last_low_freq_state_       = state_point_;
    last_low_freq_state_valid_ = true;

    // Adaptive policy from dense scan's D_degenerate
    {
        PolicyInput pin_l;
        pin_l.v_linear     = state_point_.vel.norm();
        pin_l.v_angular    = imu_proc_.last_angular_velocity().norm();
        pin_l.D_degenerate = features_l.D_degenerate;
        policy_.update(pin_l);
    }

    // Adaptive window policy (paper Sec. "Adaptive Window Policy", eq. policy):
    // q = max(q_rmse, q_omega, q_geo); W_l <- W_l_min + (W_l_max-W_l_min)*q.
    // q=1 -> W_l_max (accumulate evidence, don't disturb a working estimate);
    // q=0 -> W_l_min (favor reactivity).
    //   q_rmse: fit-quality term (UNCONDITIONAL) from the low tier's own just-
    //           completed iEKF point-to-plane residual RMSE (last_residual_rmse_).
    //   q_omega: rotation-rate gate (UNCONDITIONAL) from the EMA-smoothed gyro
    //            magnitude (omega_ema_).
    //   q_geo:  geometric-conditioning term (this work, FLAG-GATED by
    //           low_window_geo_conditioning_enable; off by default => 0,
    //           recovering exactly the two-term policy). Reuses the position
    //           information block J_p (last_pos_HtH_) already accumulated for
    //           the (disabled) mid-tier degeneracy guard -- its eigendecomposition
    //           is computed here unconditionally on the already-accumulated
    //           matrix when the flag is set, independent of whether
    //           mid_degen_project_enable's own gated projection step ran.
    {
        using namespace mf_lio::adaptive_window;

        const double q_rmse = q_rmse_term(last_residual_rmse_,
                                           params_.low_freq_residual_good,
                                           params_.low_freq_residual_bad);
        const double q_omega = q_omega_term(omega_ema_,
                                             params_.W_l_omega_lo,
                                             params_.W_l_omega_hi);

        double q_geo = 0.0;
        if (params_.low_window_geo_conditioning_enable && last_pos_hth_valid_) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_geo(last_pos_HtH_);
            const double lambda_min_Jp = eig_geo.eigenvalues()(0);  // ascending order
            q_geo = q_geo_term(true, lambda_min_Jp,
                                params_.low_window_lambda_good,
                                params_.low_window_lambda_bad);
        }

        current_W_l_ms_ = combine_window_ms(q_rmse, q_omega, q_geo,
                                             params_.W_l_min_ms, params_.W_l_max_ms);
    }
    last_low_pos_cov_trace_ = kf_.get_P().block<3,3>(0,0).trace();

    // Trim raw point buffer — keep enough for the longest possible low-tier window.
    const double keep_window_s = std::max(
        current_W_l_ms_ * 1e-3,
        params_.low_adaptive_enable ? params_.low_adaptive_max_interval_ms * 1e-3 : 0.0);
    point_buf_.clear_before(t_now - keep_window_s * 1.1);

    if (can_publish(t_now)) {
        publish_corrected_if_safe(correction_publish_ref, state_point_, t_now);
    }

    // Loop closure keyframe: capture if moved far enough from the last keyframe.
    if (params_.loop_closure_enable && initialized_) {
        const double moved = lc_kf_pos_valid_
            ? (state_point_.pos - lc_last_kf_pos_).norm() : 9999.0;
        if (moved >= params_.loop_closure_keyframe_dist_m) {
            auto kf_scan = downsample_for_lc(scan_ds);
            KeyFrame kf;
            kf.timestamp  = t_now;
            kf.path_dist  = lc_path_dist_ + moved;
            kf.pose       = state_point_;
            kf.scan_local = kf_scan;
            kf.sc         = ScanContextManager::compute_sc(kf_scan);
            kf.ring_key   = ScanContextManager::compute_ring_key(kf.sc);
            {
                std::lock_guard<std::mutex> lk(lc_mutex_);
                sc_manager_.keyframes_.push_back(std::move(kf));
                OrigKF orig;
                orig.timestamp = t_now;
                orig.pos       = state_point_.pos;
                orig.rot       = Eigen::Quaterniond(state_point_.rot.toRotationMatrix());
                kf_originals_.push_back(orig);
            }
            lc_cv_.notify_one();
            lc_path_dist_   += moved;
            lc_last_kf_pos_  = state_point_.pos;
            lc_kf_pos_valid_ = true;
        }
    }
}

// ---------------------------------------------------------------------------
bool MfLioEstimator::should_publish_corrected(const state_ikfom& pre_update,
                                              const state_ikfom& corrected,
                                              double t_now) {
    if (!params_.correction_publish_gate_enable) return true;
    if (last_effct_ < params_.correction_publish_min_correspondences) return false;
    if (last_quality_ < params_.correction_publish_min_quality) return false;
    if (last_residual_rmse_ > params_.correction_publish_max_residual_rmse) return false;

    const double innovation = (corrected.pos - pre_update.pos).norm();
    Eigen::AngleAxisd innovation_rot(pre_update.rot.conjugate() * corrected.rot);
    const double innovation_rot_deg = std::abs(innovation_rot.angle()) * 180.0 / M_PI;
    if (innovation > params_.correction_publish_max_innovation) return false;
    if (innovation_rot_deg > params_.correction_publish_max_rot_deg) return false;

    if (last_corrected_publish_valid_) {
        const double dt = t_now - last_corrected_publish_t_;
        if (dt > 1e-3) {
            const double speed =
                (corrected.pos - last_corrected_publish_state_.pos).norm() / dt;
            if (speed > params_.correction_publish_max_speed) return false;
        }
    }

    return true;
}

bool MfLioEstimator::should_accept_update_motion(const state_ikfom& pre_update,
                                                 const state_ikfom& corrected,
                                                 double dt) const {
    if (!params_.update_motion_guard_enable) return true;

    const double safe_dt = std::max(dt, 1e-3);
    const double pos_delta = (corrected.pos - pre_update.pos).norm();
    Eigen::AngleAxisd rot_delta(pre_update.rot.conjugate() * corrected.rot);
    const double rot_delta_deg = std::abs(rot_delta.angle()) * 180.0 / M_PI;
    const double speed = pos_delta / safe_dt;

    return pos_delta <= params_.update_motion_guard_max_pos_delta &&
           rot_delta_deg <= params_.update_motion_guard_max_rot_deg &&
           speed <= params_.update_motion_guard_max_speed;
}

void MfLioEstimator::publish_corrected_if_safe(const state_ikfom& pre_update,
                                               const state_ikfom& corrected,
                                               double t_now) {
    if (!should_publish_corrected(pre_update, corrected, t_now)) return;

    last_corrected_publish_state_ = corrected;
    last_corrected_publish_t_ = t_now;
    last_corrected_publish_valid_ = true;

    if (params_.publish_corrected_odometry_on_main && odom_cb_) {
        odom_cb_(corrected, t_now);
    }
    if (odom_corrected_cb_) {
        odom_corrected_cb_(corrected, t_now);
    }
}

// ---------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
MfLioEstimator::collect_and_undistort(double t_start, double t_end)
{
    auto pts = point_buf_.get_window(t_start, t_end);
    if (pts.empty()) return nullptr;

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    cloud_in->reserve(pts.size());
    for (const auto& sp : pts) {
        auto pt = sp.point;
        pt.curvature = static_cast<float>((sp.timestamp - t_start) * 1e3);
        cloud_in->push_back(pt);
    }

    auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    bool ok = imu_proc_.process(t_start, t_end, *cloud_in, *cloud_out, kf_);
    if (!ok) return nullptr;
    return cloud_out;
}

// ---------------------------------------------------------------------------
// Loop Closure methods
// ---------------------------------------------------------------------------

PointCloudXYZI::Ptr MfLioEstimator::downsample_for_lc(const PointCloudXYZI::Ptr& in)
{
    constexpr size_t kTarget = 2000;
    auto out = std::make_shared<PointCloudXYZI>();
    if (!in || in->empty()) return out;
    const size_t step = in->size() > kTarget ? in->size() / kTarget : 1;
    out->reserve(std::min(kTarget, in->size()));
    for (size_t i = 0; i < in->size(); i += step)
        out->push_back(in->points[i]);
    return out;
}

void MfLioEstimator::apply_loop_correction(const Eigen::Matrix4f& T_corr_f)
{
    const Eigen::Matrix3d R = T_corr_f.block<3,3>(0,0).cast<double>();
    const Eigen::Vector3d t = T_corr_f.block<3,1>(0,3).cast<double>();

    // Transform state
    state_point_.pos = R * state_point_.pos + t;
    state_point_.rot = SO3(Eigen::Quaterniond(R * state_point_.rot.toRotationMatrix()).normalized());
    kf_.change_x(state_point_);
    sync_riekf_from_kf();  // RIEKF ablation: loop-closure correction also mutates kf_ directly

    // Update all stored keyframe poses to the new world frame so subsequent LC
    // corrections use a consistent world-frame reference for T_cand.
    {
        std::lock_guard<std::mutex> lk(lc_mutex_);
        for (auto& kf : sc_manager_.keyframes_) {
            kf.pose.pos = R * kf.pose.pos + t;
            kf.pose.rot = SO3(Eigen::Quaterniond(R * kf.pose.rot.toRotationMatrix()).normalized());
        }
    }
    // lc_last_kf_pos_ is only touched by the main estimation thread — no lock needed.
    lc_last_kf_pos_ = R * lc_last_kf_pos_ + t;

    // Transform all ikd-tree map points so the map stays consistent with the corrected state.
    // Without this, exact kNN residuals would immediately pull the state back to the old position.
    if (ikdtree_.Root_Node != nullptr) {
        PointVector all_pts;
        ikdtree_.flatten(ikdtree_.Root_Node, all_pts, NOT_RECORD);
        const Eigen::Matrix3f R_f = T_corr_f.block<3,3>(0,0);
        const Eigen::Vector3f t_f = T_corr_f.block<3,1>(0,3);
        for (auto& p : all_pts) {
            const Eigen::Vector3f pw(p.x, p.y, p.z);
            const Eigen::Vector3f pw_new = R_f * pw + t_f;
            p.x = pw_new.x(); p.y = pw_new.y(); p.z = pw_new.z();
        }
        ikdtree_.set_downsample_param(static_cast<float>(params_.filter_size_map));
        ikdtree_.Build(all_pts);
    }

    std::printf("[LC] Correction applied: (%.3f, %.3f, %.3f) m, map rebuilt with %zu pts\n",
                t.x(), t.y(), t.z(),
                ikdtree_.Root_Node ? static_cast<size_t>(ikdtree_.size()) : 0UL);
    std::fflush(stdout);
}

void MfLioEstimator::lc_loop()
{
    // Apply all recorded LC corrections as linear ramps to kf_originals_ and write TUM.
    // Must be called with lc_mutex_ already held.
    auto write_smooth_tum = [this]() {
        const int n_kf = static_cast<int>(kf_originals_.size());
        if (n_kf == 0 || lc_corr_log_.empty()) return;

        std::vector<Eigen::Vector3d>    pos(n_kf);
        std::vector<Eigen::Quaterniond> rot(n_kf);
        for (int i = 0; i < n_kf; ++i) {
            pos[i] = kf_originals_[i].pos;
            rot[i] = kf_originals_[i].rot;
        }

        for (const auto& ev : lc_corr_log_) {
            const Eigen::Matrix3d    R_full = ev.T_corr.block<3,3>(0,0).cast<double>();
            const Eigen::Vector3d    t_full = ev.T_corr.block<3,1>(0,3).cast<double>();
            const Eigen::Quaterniond q_full(R_full);
            const int c    = ev.cand_idx;
            const int q    = ev.query_idx;
            const int span = (q > c) ? (q - c) : 1;
            for (int i = 0; i < n_kf; ++i) {
                double alpha = 0.0;
                if (i <= c)      alpha = 0.0;
                else if (i >= q) alpha = 1.0;
                else             alpha = static_cast<double>(i - c) / static_cast<double>(span);
                const Eigen::Quaterniond q_i = Eigen::Quaterniond::Identity().slerp(alpha, q_full);
                const Eigen::Matrix3d    R_i = q_i.toRotationMatrix();
                pos[i] = R_i * pos[i] + alpha * t_full;
                rot[i] = Eigen::Quaterniond(R_i * rot[i].toRotationMatrix()).normalized();
            }
        }

        std::ofstream tum("/tmp/v51_lc_kf.txt");
        tum << std::fixed << std::setprecision(9);
        for (int i = 0; i < n_kf; ++i) {
            const Eigen::Quaterniond& q = rot[i];
            tum << kf_originals_[i].timestamp << " "
                << pos[i].x() << " " << pos[i].y() << " " << pos[i].z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        std::printf("[LC] Wrote %d smoothly-corrected keyframes to /tmp/v51_lc_kf.txt"
                    " (%zu LC events)\n", n_kf, lc_corr_log_.size());
        std::fflush(stdout);
    };

    size_t last_kf_count = 0;
    while (running_.load()) {
        // Wait until a new keyframe arrives or shutdown.
        {
            std::unique_lock<std::mutex> lk(lc_mutex_);
            lc_cv_.wait_for(lk, std::chrono::milliseconds(200),
                [this, &last_kf_count]{
                    return !running_.load() || sc_manager_.keyframes_.size() > last_kf_count;
                });
            if (!running_.load()) break;
            if (sc_manager_.keyframes_.size() <= last_kf_count) continue;
            last_kf_count = sc_manager_.keyframes_.size();
        }

        // SC matching against all older keyframes.
        KeyFrame query;
        std::vector<ScanContextManager::CandMeta> meta;
        {
            std::lock_guard<std::mutex> lk(lc_mutex_);
            query = sc_manager_.keyframes_.back();
            meta.reserve(sc_manager_.keyframes_.size() - 1);
            for (size_t i = 0; i + 1 < sc_manager_.keyframes_.size(); ++i) {
                const auto& kf = sc_manager_.keyframes_[i];
                meta.push_back({kf.timestamp, kf.path_dist, kf.sc, kf.ring_key});
            }
        }

        auto [best_idx, best_dist, best_shift, best_any] = ScanContextManager::search_meta(
            query, meta,
            params_.loop_closure_min_age_s,
            params_.loop_closure_min_path_dist_m,
            params_.loop_closure_score_thresh);

        std::printf("[LC] query t=%.1fs path=%.0fm | best_any=%.3f thresh=%.2f",
                    query.timestamp, query.path_dist,
                    best_any >= 0.f ? best_any : -1.f,
                    params_.loop_closure_score_thresh);
        if (best_idx >= 0)
            std::printf(" | CANDIDATE idx=%d SC=%.3f\n", best_idx, best_dist);
        else
            std::printf(" | no match\n");
        std::fflush(stdout);

        if (best_idx < 0) continue;

        KeyFrame candidate;
        {
            std::lock_guard<std::mutex> lk(lc_mutex_);
            if (best_idx >= static_cast<int>(sc_manager_.keyframes_.size()))
                continue;
            candidate = sc_manager_.keyframes_[best_idx];
        }

        // SC rotation shift → initial yaw guess for SVD.
        const float angle_deg = best_shift * (360.f / SC_SECTORS);
        const float angle_rad = angle_deg * (static_cast<float>(M_PI) / 180.f);
        Eigen::Matrix4f T_guess = Eigen::Matrix4f::Identity();
        T_guess(0,0) =  std::cos(angle_rad); T_guess(0,1) = -std::sin(angle_rad);
        T_guess(1,0) =  std::sin(angle_rad); T_guess(1,1) =  std::cos(angle_rad);

        Eigen::Matrix4f T_local;
        bool ok = ScanContextManager::verify_icp(
            query.scan_local, candidate.scan_local, T_guess,
            params_.loop_closure_icp_max_dist,
            params_.loop_closure_icp_fitness_thresh,
            params_.loop_closure_max_trans_m,
            params_.loop_closure_max_rot_deg,
            &T_local);

        if (!ok) {
            std::printf("[LC] ICP failed\n");
            std::fflush(stdout);
            continue;
        }

        // World-frame correction: T_cand * T_local * T_query^{-1}.
        // Zero z-component: ground-robot iEKF height differences must not shift the map.
        Eigen::Matrix4f T_cand  = Eigen::Matrix4f::Identity();
        T_cand.block<3,3>(0,0)  = candidate.pose.rot.toRotationMatrix().cast<float>();
        T_cand.block<3,1>(0,3)  = candidate.pose.pos.cast<float>();
        Eigen::Matrix4f T_query = Eigen::Matrix4f::Identity();
        T_query.block<3,3>(0,0) = query.pose.rot.toRotationMatrix().cast<float>();
        T_query.block<3,1>(0,3) = query.pose.pos.cast<float>();

        Eigen::Matrix4f T_corr = T_cand * T_local * T_query.inverse();
        T_corr(2, 3) = 0.f;

        const Eigen::Vector3f t_corr = T_corr.block<3,1>(0,3);

        if (t_corr.norm() < 0.05f) {
            std::printf("[LC] correction trivial (%.3f m) — skip\n", t_corr.norm());
            std::fflush(stdout);
            continue;
        }
        if (t_corr.norm() > params_.loop_closure_max_trans_m) {
            std::printf("[LC] correction too large (%.3f m > %.1f m) — skip\n",
                        t_corr.norm(), params_.loop_closure_max_trans_m);
            std::fflush(stdout);
            continue;
        }

        std::printf("[LC] Queueing correction (%.3f, %.3f, %.3f) m, SC=%.3f, cand=%d query=%d\n",
                    t_corr.x(), t_corr.y(), t_corr.z(), best_dist,
                    best_idx, static_cast<int>(last_kf_count) - 1);
        std::fflush(stdout);

        {
            std::lock_guard<std::mutex> lk(lc_mutex_);
            LcCorrEvent ev;
            ev.cand_idx  = best_idx;
            ev.query_idx = static_cast<int>(last_kf_count) - 1;
            ev.T_corr    = T_corr;
            lc_corr_log_.push_back(ev);
            lc_correction_     = T_corr;
            lc_has_correction_ = true;
            // Write TUM immediately — do not rely on the shutdown path (node may be
            // killed mid-cooldown before the shutdown TUM write block is reached).
            write_smooth_tum();
        }

        // Cooldown: 300s — prevents re-triggering while map/state settle.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
        while (running_.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    // At shutdown: write the final TUM file including any keyframes added after the
    // last LC event (they get the full last correction in the ramp's alpha=1 region).
    {
        std::lock_guard<std::mutex> lk(lc_mutex_);
        write_smooth_tum();
    }
}
