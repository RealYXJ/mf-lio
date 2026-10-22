#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <limits>
#include <csignal>
#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>

static void sigsegv_handler(int sig) {
    void* frames[60];
    int n = backtrace(frames, 60);
    int fd = open("/tmp/mf_lio_crash.txt",
                  O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (fd >= 0) {
        dprintf(fd, "\n=== CRASH signal %d ===\n", sig);
        backtrace_symbols_fd(frames, n, fd);
        dprintf(fd, "=== end ===\n");
        close(fd);
    }
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    _exit(1);
}

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "mf_lio/mf_lio_estimator.h"

class MfLioNode : public rclcpp::Node {
public:
    MfLioNode() : Node("mf_lio_node") {
        load_params();

        estimator_ = std::make_unique<MfLioEstimator>(params_);

        estimator_->set_odom_callback([this](const state_ikfom& s, double t) {
            publish_odometry(s, t);
        });
        estimator_->set_odom_fast_callback([this](const state_ikfom& s, double t) {
            publish_odometry_fast(s, t);
        });
        estimator_->set_odom_corrected_callback([this](const state_ikfom& s, double t) {
            publish_odometry_corrected(s, t);
        });
        estimator_->set_odom_smooth_callback([this](const state_ikfom& s, double t) {
            publish_odometry_smooth(s, t);
        });
        estimator_->set_freq_callback([this](double Wh, double Wl) {
            publish_adaptive_windows(Wh, Wl);
        });

        // Subscribe to LiDAR — CustomMsg (lidar_type=0) or PointCloud2 (lidar_type=1)
        lidar_type_ = declare_parameter<int>("lidar_type", 0);
        if (lidar_type_ == 0) {
            sub_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lidar_topic_, 1000,
                [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                    livox_callback(msg);
                });
        } else {
            sub_pc2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_topic_, 1000,
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    pc2_callback(msg);
                });
        }

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 200,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                estimator_->push_imu(*msg);
            });

        output_prefix_ = declare_parameter<std::string>("output_prefix", "/mf_lio_v51");
        pub_odom_       = create_publisher<nav_msgs::msg::Odometry>(output_prefix_ + "/odometry", 10);
        pub_odom_fast_  = create_publisher<nav_msgs::msg::Odometry>(output_prefix_ + "/odometry_fast", 10);
        pub_odom_corr_  = create_publisher<nav_msgs::msg::Odometry>(output_prefix_ + "/odometry_corrected", 10);
        pub_odom_smooth_= create_publisher<nav_msgs::msg::Odometry>(output_prefix_ + "/odometry_smooth", 10);
        pub_path_      = create_publisher<nav_msgs::msg::Path>(output_prefix_ + "/path", 10);
        pub_freq_      = create_publisher<std_msgs::msg::Float64MultiArray>(
                             output_prefix_ + "/adaptive_windows", 10);
        pub_latency_   = create_publisher<std_msgs::msg::Float64MultiArray>(
                             output_prefix_ + "/latency_diagnostics", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        estimator_->start();
        RCLCPP_INFO(get_logger(), "MF-LIO v46-fast-map node started, output_prefix=%s",
                    output_prefix_.c_str());
    }

    ~MfLioNode() {
        if (estimator_) estimator_->stop();
    }

private:
    void load_params() {
        lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        imu_topic_   = declare_parameter<std::string>("imu_topic", "/imu/data");
        odometry_time_offset_ms_ =
            declare_parameter<double>("odometry_time_offset_ms", 0.0);
        latency_estimate_ms_ = odometry_time_offset_ms_;
        online_latency_estimation_enable_ =
            declare_parameter<bool>("online_latency_estimation_enable", false);
        latency_min_ms_ =
            declare_parameter<double>("latency_min_ms", -30.0);
        latency_max_ms_ =
            declare_parameter<double>("latency_max_ms", 30.0);
        latency_search_radius_ms_ =
            declare_parameter<double>("latency_search_radius_ms", 8.0);
        latency_search_step_ms_ =
            declare_parameter<double>("latency_search_step_ms", 1.0);
        latency_smoothing_alpha_ =
            declare_parameter<double>("latency_smoothing_alpha", 0.08);
        latency_transport_alpha_ =
            declare_parameter<double>("latency_transport_alpha", 0.05);
        latency_transport_weight_ =
            declare_parameter<double>("latency_transport_weight", 0.05);
        latency_update_requires_transport_agreement_ =
            declare_parameter<bool>("latency_update_requires_transport_agreement", true);
        latency_transport_agreement_ms_ =
            declare_parameter<double>("latency_transport_agreement_ms", 5.0);
        latency_pos_sigma_m_ =
            declare_parameter<double>("latency_pos_sigma_m", 0.05);
        latency_rot_sigma_deg_ =
            declare_parameter<double>("latency_rot_sigma_deg", 2.0);
        latency_min_speed_mps_ =
            declare_parameter<double>("latency_min_speed_mps", 0.10);
        latency_buffer_sec_ =
            declare_parameter<double>("latency_buffer_sec", 1.0);
        if (latency_min_ms_ > latency_max_ms_) {
            std::swap(latency_min_ms_, latency_max_ms_);
        }
        latency_estimate_ms_ =
            std::clamp(latency_estimate_ms_, latency_min_ms_, latency_max_ms_);
        latency_search_radius_ms_ = std::max(latency_search_radius_ms_, 0.0);
        latency_search_step_ms_ = std::max(latency_search_step_ms_, 0.1);
        latency_smoothing_alpha_ =
            std::clamp(latency_smoothing_alpha_, 0.0, 1.0);
        latency_transport_alpha_ =
            std::clamp(latency_transport_alpha_, 0.0, 1.0);
        latency_transport_weight_ = std::max(latency_transport_weight_, 0.0);
        latency_transport_agreement_ms_ =
            std::max(latency_transport_agreement_ms_, 0.0);
        latency_pos_sigma_m_ = std::max(latency_pos_sigma_m_, 1e-4);
        latency_rot_sigma_deg_ = std::max(latency_rot_sigma_deg_, 1e-3);
        latency_min_speed_mps_ = std::max(latency_min_speed_mps_, 0.0);
        latency_buffer_sec_ = std::max(latency_buffer_sec_, 0.1);

        auto T = declare_parameter<std::vector<double>>("extrinsic_T", {0,0,0});
        auto R = declare_parameter<std::vector<double>>(
            "extrinsic_R", {1,0,0, 0,1,0, 0,0,1});

        params_.extrinsic_T = Eigen::Vector3d(T[0], T[1], T[2]);
        params_.extrinsic_R << R[0],R[1],R[2], R[3],R[4],R[5], R[6],R[7],R[8];
        params_.extrinsic_est_en =
            declare_parameter<bool>("extrinsic_est_en", false);
        params_.state_extrinsic_undistort =
            declare_parameter<bool>("state_extrinsic_undistort", false);
        params_.legacy_mixed_extrinsic_undistort =
            declare_parameter<bool>("legacy_mixed_extrinsic_undistort", false);
        params_.extrinsic_rot_cov =
            declare_parameter<double>("extrinsic_rot_cov", 1e-8);
        params_.extrinsic_trans_cov =
            declare_parameter<double>("extrinsic_trans_cov", 1e-8);
        params_.extrinsic_update_gate_enable =
            declare_parameter<bool>("extrinsic_update_gate_enable", false);
        params_.extrinsic_update_min_angular_velocity =
            declare_parameter<double>("extrinsic_update_min_angular_velocity", 0.0);
        params_.extrinsic_update_min_quality =
            declare_parameter<double>("extrinsic_update_min_quality", 0.0);
        params_.extrinsic_update_max_residual_rmse =
            declare_parameter<double>("extrinsic_update_max_residual_rmse", 1e9);
        params_.extrinsic_update_adaptive_enable =
            declare_parameter<bool>("extrinsic_update_adaptive_enable", false);
        params_.extrinsic_update_adaptive_high_angular_velocity =
            declare_parameter<double>("extrinsic_update_adaptive_high_angular_velocity", 7.0);
        params_.extrinsic_update_adaptive_min_high_count =
            declare_parameter<int>("extrinsic_update_adaptive_min_high_count", 3);
        params_.extrinsic_update_adaptive_min_quality =
            declare_parameter<double>("extrinsic_update_adaptive_min_quality", 0.02);
        params_.extrinsic_update_adaptive_max_residual_rmse =
            declare_parameter<double>("extrinsic_update_adaptive_max_residual_rmse", 0.50);

        params_.gyr_cov   = declare_parameter<double>("gyr_cov",   0.1);
        params_.acc_cov   = declare_parameter<double>("acc_cov",   0.1);
        params_.b_gyr_cov = declare_parameter<double>("b_gyr_cov", 0.0001);
        params_.b_acc_cov = declare_parameter<double>("b_acc_cov", 0.0001);
        params_.init_vel_cov = declare_parameter<double>("init_vel_cov", 1.0);

        params_.W_f_ms      = declare_parameter<double>("W_f_ms",      10.0);
        params_.W_m_ms      = declare_parameter<double>("W_m_ms",      100.0);
        params_.W_l_base_ms = declare_parameter<double>("W_l_base_ms", 200.0);
        params_.W_l_min_ms  = declare_parameter<double>("W_l_min_ms",  200.0);
        params_.W_l_max_ms  = declare_parameter<double>("W_l_max_ms",  500.0);
        params_.alpha = declare_parameter<double>("alpha", 0.5);
        params_.beta  = declare_parameter<double>("beta",  0.001);

        params_.edge_thresh  = static_cast<float>(
            declare_parameter<double>("edge_thresh",  0.65));
        params_.plane_thresh = static_cast<float>(
            declare_parameter<double>("plane_thresh", 0.75));

        params_.filter_size_map       = declare_parameter<double>("filter_size_map", 0.2);
        params_.filter_size_scan      = declare_parameter<double>("filter_size_scan", 0.0);
        params_.point_filter_num      = declare_parameter<int>("point_filter_num", 1);
        params_.cube_side_length      = declare_parameter<double>("cube_side_length", 1000.0);
        params_.num_max_iterations    = declare_parameter<int>("num_max_iterations", 4);
        params_.esti_plane_threshold  = declare_parameter<double>("esti_plane_threshold", 0.1);
        params_.angular_gate_rad_s         = declare_parameter<double>("angular_gate_rad_s", 1000.0);
        params_.map_write_dist_thresh      = declare_parameter<double>("map_write_dist_thresh", 1000.0);
        params_.temporal_decay_alpha       = declare_parameter<double>("temporal_decay_alpha", 0.0);
        params_.min_correspondence_quality = declare_parameter<double>("min_correspondence_quality", 0.0);
        params_.low_freq_min_correspondences =
            declare_parameter<int>("low_freq_min_correspondences", 20);
        params_.low_freq_min_quality =
            declare_parameter<double>("low_freq_min_quality", 0.02);
        params_.low_freq_max_residual_rmse =
            declare_parameter<double>("low_freq_max_residual_rmse", 0.30);
        params_.low_freq_max_pos_delta =
            declare_parameter<double>("low_freq_max_pos_delta", 1.50);
        params_.low_freq_max_rot_delta_deg =
            declare_parameter<double>("low_freq_max_rot_delta_deg", 20.0);
        params_.publish_start_delay_ms =
            declare_parameter<double>("publish_start_delay_ms", 2500.0);
        params_.publish_requires_stable_update =
            declare_parameter<bool>("publish_requires_stable_update", true);
        params_.publish_stable_min_correspondences =
            declare_parameter<int>("publish_stable_min_correspondences", 20);
        params_.publish_stable_min_quality =
            declare_parameter<double>("publish_stable_min_quality", 0.02);
        params_.publish_stable_max_residual_rmse =
            declare_parameter<double>("publish_stable_max_residual_rmse", 0.50);
        params_.publish_stable_min_state_norm =
            declare_parameter<double>("publish_stable_min_state_norm", 0.0);
        params_.publish_fast_odometry_on_main =
            declare_parameter<bool>("publish_fast_odometry_on_main", true);
        params_.publish_corrected_odometry_on_main =
            declare_parameter<bool>("publish_corrected_odometry_on_main", true);
        params_.correction_publish_gate_enable =
            declare_parameter<bool>("correction_publish_gate_enable", false);
        params_.correction_publish_min_correspondences =
            declare_parameter<int>("correction_publish_min_correspondences", 20);
        params_.correction_publish_min_quality =
            declare_parameter<double>("correction_publish_min_quality", 0.02);
        params_.correction_publish_max_residual_rmse =
            declare_parameter<double>("correction_publish_max_residual_rmse", 0.50);
        params_.correction_publish_max_innovation =
            declare_parameter<double>("correction_publish_max_innovation", 0.50);
        params_.correction_publish_max_rot_deg =
            declare_parameter<double>("correction_publish_max_rot_deg", 10.0);
        params_.correction_publish_max_speed =
            declare_parameter<double>("correction_publish_max_speed", 5.0);
        params_.update_motion_guard_enable =
            declare_parameter<bool>("update_motion_guard_enable", false);
        params_.update_motion_guard_max_pos_delta =
            declare_parameter<double>("update_motion_guard_max_pos_delta", 3.0);
        params_.update_motion_guard_max_rot_deg =
            declare_parameter<double>("update_motion_guard_max_rot_deg", 45.0);
        params_.update_motion_guard_max_speed =
            declare_parameter<double>("update_motion_guard_max_speed", 30.0);
        params_.low_freq_trust_enable =
            declare_parameter<bool>("low_freq_trust_enable", true);
        params_.low_freq_trust_reject =
            declare_parameter<double>("low_freq_trust_reject", 0.20);
        params_.low_freq_trust_full =
            declare_parameter<double>("low_freq_trust_full", 0.80);
        params_.low_freq_quality_good =
            declare_parameter<double>("low_freq_quality_good", 0.08);
        params_.low_freq_residual_good =
            declare_parameter<double>("low_freq_residual_good", 0.045);
        params_.low_freq_residual_bad =
            declare_parameter<double>("low_freq_residual_bad", 0.13);
        params_.low_freq_pos_delta_good =
            declare_parameter<double>("low_freq_pos_delta_good", 0.03);
        params_.low_freq_pos_delta_bad =
            declare_parameter<double>("low_freq_pos_delta_bad", 1.50);
        params_.low_freq_rot_delta_good_deg =
            declare_parameter<double>("low_freq_rot_delta_good_deg", 1.0);
        params_.low_freq_rot_delta_bad_deg =
            declare_parameter<double>("low_freq_rot_delta_bad_deg", 20.0);
        params_.low_freq_degen_good =
            declare_parameter<double>("low_freq_degen_good", 1e-4);
        params_.low_freq_degen_bad =
            declare_parameter<double>("low_freq_degen_bad", 1e-7);
        params_.low_freq_motion_good =
            declare_parameter<double>("low_freq_motion_good", 0.5);
        params_.low_freq_motion_bad =
            declare_parameter<double>("low_freq_motion_bad", 6.0);

        // Option C: covariance-gated low-freq influence
        params_.cov_gate_enable =
            declare_parameter<bool>("cov_gate_enable", true);
        params_.cov_gate_pos_low =
            declare_parameter<double>("cov_gate_pos_low", 1e-4);
        params_.cov_gate_pos_high =
            declare_parameter<double>("cov_gate_pos_high", 1e-2);
        params_.cov_gate_base =
            declare_parameter<double>("cov_gate_base", 0.4);
        // Option F: motion-triggered mid-freq scheduling
        params_.mid_freq_motion_trigger =
            declare_parameter<bool>("mid_freq_motion_trigger", true);
        params_.mid_freq_trigger_rot_deg =
            declare_parameter<double>("mid_freq_trigger_rot_deg", 5.0);
        params_.mid_freq_trigger_trans_m =
            declare_parameter<double>("mid_freq_trigger_trans_m", 0.10);
        params_.mid_freq_min_interval_ms =
            declare_parameter<double>("mid_freq_min_interval_ms", 50.0);

        // Fast-tier read-only map correction
        params_.fast_map_update_enable =
            declare_parameter<bool>("fast_map_update_enable", false);
        params_.fast_tier_writeback_enable =
            declare_parameter<bool>("fast_tier_writeback_enable", false);
        params_.fast_tier_enable =
            declare_parameter<bool>("fast_tier_enable", true);
        params_.disjoint_scheduling_enable =
            declare_parameter<bool>("disjoint_scheduling_enable", true);
        params_.overlap_bound_steps =
            declare_parameter<int>("overlap_bound_steps", 0);
        params_.low_reset_tm_enable =
            declare_parameter<bool>("low_reset_tm_enable", true);
        params_.low_freq_min_trust_alpha =
            declare_parameter<double>("low_freq_min_trust_alpha", 0.0);
        params_.low_freq_degen_gate_enable =
            declare_parameter<bool>("low_freq_degen_gate_enable", false);
        params_.low_freq_degen_gate_min_eig =
            declare_parameter<double>("low_freq_degen_gate_min_eig", 0.0);
        params_.low_freq_degen_gate_max_interval_ms =
            declare_parameter<double>("low_freq_degen_gate_max_interval_ms", 2000.0);
        params_.low_freq_axis_gate_enable =
            declare_parameter<bool>("low_freq_axis_gate_enable", false);
        params_.low_freq_axis_gate_thresh =
            declare_parameter<double>("low_freq_axis_gate_thresh", 0.35);
        params_.low_tier_own_map_enable =
            declare_parameter<bool>("low_tier_own_map_enable", false);
        params_.low_own_map_noise_scale =
            declare_parameter<double>("low_own_map_noise_scale", 3.0);
        params_.low_own_map_anisotropic_enable =
            declare_parameter<bool>("low_own_map_anisotropic_enable", false);
        params_.low_own_map_aniso_mix =
            declare_parameter<double>("low_own_map_aniso_mix", 1.0);
        params_.low_own_map_per_axis_hybrid_enable =
            declare_parameter<bool>("low_own_map_per_axis_hybrid_enable", false);
        params_.low_own_map_kappa_gate_enable =
            declare_parameter<bool>("low_own_map_kappa_gate_enable", false);
        params_.low_own_map_kappa0 =
            declare_parameter<double>("low_own_map_kappa0", 0.05);
        params_.low_own_map_kappa_log_path =
            declare_parameter<std::string>("low_own_map_kappa_log_path", "");
        // RDEF/CI fusion diagnostic (disclosed negative): default off is a
        // true no-op -- see MfLioParams::rdef_ci_diagnostic_enable.
        params_.rdef_ci_diagnostic_enable =
            declare_parameter<bool>("rdef_ci_diagnostic_enable", false);
        params_.rdef_ci_diagnostic_log_path =
            declare_parameter<std::string>("rdef_ci_diagnostic_log_path", "");
        params_.shared_origin_scheduling_enable =
            declare_parameter<bool>("shared_origin_scheduling_enable", false);
        params_.fast_map_min_points =
            declare_parameter<int>("fast_map_min_points", 20);
        params_.fast_map_min_quality =
            declare_parameter<double>("fast_map_min_quality", 0.03);
        params_.fast_map_voxel_size =
            declare_parameter<double>("fast_map_voxel_size", 0.10);
        params_.fast_writeback_enable =
            declare_parameter<bool>("fast_writeback_enable", false);
        params_.fast_writeback_lambda_thresh =
            declare_parameter<double>("fast_writeback_lambda_thresh", 50.0);
        params_.fast_writeback_kappa_thresh =
            declare_parameter<double>("fast_writeback_kappa_thresh", 500.0);
        params_.fast_writeback_damp =
            declare_parameter<double>("fast_writeback_damp", 0.3);
        params_.fast_6dof_enable =
            declare_parameter<bool>("fast_6dof_enable", false);
        params_.fast_norm_rot_scale =
            declare_parameter<double>("fast_norm_rot_scale", 1.0);
        params_.fast_undistort_enable =
            declare_parameter<bool>("fast_undistort_enable", false);
        params_.fast_undistort_min_eig =
            static_cast<float>(declare_parameter<double>("fast_undistort_min_eig", 5.0));
        params_.fast_eskf_enable =
            declare_parameter<bool>("fast_eskf_enable", false);
        params_.fast_eskf_max_iter =
            declare_parameter<int>("fast_eskf_max_iter", 2);
        params_.fast_eskf_min_correspondences =
            declare_parameter<int>("fast_eskf_min_correspondences", 50);
        params_.fast_eskf_max_residual =
            declare_parameter<double>("fast_eskf_max_residual", 0.05);
        params_.fast_eskf_min_speed_m_s =
            declare_parameter<double>("fast_eskf_min_speed_m_s", 0.0);
        params_.fast_eskf_min_correction_m =
            declare_parameter<double>("fast_eskf_min_correction_m", 0.0);
        params_.async_100hz_enable =
            declare_parameter<bool>("async_100hz_enable", false);
        params_.fast_coarse_voxel_size =
            declare_parameter<double>("fast_coarse_voxel_size", 0.30);
        params_.low_freq_map_write_enable =
            declare_parameter<bool>("low_freq_map_write_enable", false);
        params_.fast_max_correction_m =
            declare_parameter<double>("fast_max_correction_m", 0.10);
        params_.fast_gate_omega_thresh =
            declare_parameter<double>("fast_gate_omega_thresh", 0.0);
        params_.fast_gate_min_quality =
            declare_parameter<double>("fast_gate_min_quality", 0.0);
        params_.fast_gate_max_rmse =
            declare_parameter<double>("fast_gate_max_rmse", 1e9);
        params_.fast_gate_min_correspondences =
            declare_parameter<int>("fast_gate_min_correspondences", 0);
        params_.joint_coarse_enable =
            declare_parameter<bool>("joint_coarse_enable", true);
        params_.joint_coarse_noise_scale =
            declare_parameter<double>("joint_coarse_noise_scale", 3.0);
        params_.mid_adaptive_enable =
            declare_parameter<bool>("mid_adaptive_enable", false);
        params_.mid_adaptive_trigger_pos_m =
            declare_parameter<double>("mid_adaptive_trigger_pos_m", 0.05);
        params_.mid_adaptive_trigger_rot_deg =
            declare_parameter<double>("mid_adaptive_trigger_rot_deg", 3.0);
        params_.mid_adaptive_max_interval_ms =
            declare_parameter<double>("mid_adaptive_max_interval_ms", 300.0);
        params_.low_adaptive_enable =
            declare_parameter<bool>("low_adaptive_enable", false);
        params_.low_adaptive_trigger_pos_m =
            declare_parameter<double>("low_adaptive_trigger_pos_m", 0.10);
        params_.low_adaptive_trigger_rot_deg =
            declare_parameter<double>("low_adaptive_trigger_rot_deg", 5.0);
        params_.low_adaptive_max_interval_ms =
            declare_parameter<double>("low_adaptive_max_interval_ms", 800.0);
        params_.low_adaptive_min_interval_ms =
            declare_parameter<double>("low_adaptive_min_interval_ms", 100.0);
        params_.fast_plane_cache_enable =
            declare_parameter<bool>("fast_plane_cache_enable", false);
        params_.voxel_hash_enable =
            declare_parameter<bool>("voxel_hash_enable", false);
        params_.voxel_hash_cell_size =
            declare_parameter<double>("voxel_hash_cell_size", 0.2);
        params_.fast_plane_cache_min_eig =
            static_cast<float>(declare_parameter<double>("fast_plane_cache_min_eig", 5.0));
        params_.fast_plane_cache_min_points =
            declare_parameter<int>("fast_plane_cache_min_points", 10);
        params_.fast_plane_cache_rot_decay =
            declare_parameter<double>("fast_plane_cache_rot_decay", 2.0);
        params_.fast_plane_cache_trans_decay =
            declare_parameter<double>("fast_plane_cache_trans_decay", 0.5);
        params_.fast_plane_cache_count_full =
            declare_parameter<int>("fast_plane_cache_count_full", 50);
        params_.fast_plane_cache_min_trust =
            declare_parameter<double>("fast_plane_cache_min_trust", 0.05);
        params_.fast_plane_cache_publish =
            declare_parameter<bool>("fast_plane_cache_publish", true);
        // v11: motion-coupled adaptive eigenvalue weighting + hard DOF gate
        params_.fast_plane_cache_eig_power =
            declare_parameter<double>("fast_plane_cache_eig_power", 2.0);
        params_.fast_plane_cache_eig_threshold =
            declare_parameter<double>("fast_plane_cache_eig_threshold", 0.15);
        params_.fast_plane_cache_stale_trigger =
            declare_parameter<double>("fast_plane_cache_stale_trigger", 0.5);
        params_.fast_plane_cache_stale_min_interval_ms =
            declare_parameter<double>("fast_plane_cache_stale_min_interval_ms", 25.0);
        // v48: degeneracy-aware mid-tier guard
        params_.mid_degen_project_enable =
            declare_parameter<bool>("mid_degen_project_enable", false);
        params_.mid_degen_eigen_thresh =
            declare_parameter<double>("mid_degen_eigen_thresh", 1e-3);
        params_.mid_degen_map_gate_enable =
            declare_parameter<bool>("mid_degen_map_gate_enable", false);
        params_.mid_degen_project_max_sec =
            declare_parameter<double>("mid_degen_project_max_sec", 0.0);
        params_.low_freq_ndt_enable =
            declare_parameter<bool>("low_freq_ndt_enable", false);

        // Sliding-window factor graph
        params_.graph_enable =
            declare_parameter<bool>("graph_enable", false);
        params_.graph_window_size =
            declare_parameter<int>("graph_window_size", 4);
        params_.graph_max_iter =
            declare_parameter<int>("graph_max_iter", 5);
        params_.graph_imu_sigma_pos =
            declare_parameter<double>("graph_imu_sigma_pos", 0.05);
        params_.graph_imu_sigma_rot =
            declare_parameter<double>("graph_imu_sigma_rot", 0.10);
        params_.graph_max_correction_m =
            declare_parameter<double>("graph_max_correction_m", 0.10);
        params_.graph_max_correction_deg =
            declare_parameter<double>("graph_max_correction_deg", 5.0);

        params_.loop_closure_enable =
            declare_parameter<bool>("loop_closure_enable", false);
        params_.loop_closure_score_thresh =
            static_cast<float>(declare_parameter<double>("loop_closure_score_thresh", 0.35));
        params_.loop_closure_min_age_s =
            declare_parameter<double>("loop_closure_min_age_s", 20.0);
        params_.loop_closure_min_path_dist_m =
            declare_parameter<double>("loop_closure_min_path_dist_m", 40.0);
        params_.loop_closure_icp_max_dist =
            static_cast<float>(declare_parameter<double>("loop_closure_icp_max_dist", 3.0));
        params_.loop_closure_icp_fitness_thresh =
            static_cast<float>(declare_parameter<double>("loop_closure_icp_fitness_thresh", 2.5));
        params_.loop_closure_max_trans_m =
            static_cast<float>(declare_parameter<double>("loop_closure_max_trans_m", 20.0));
        params_.loop_closure_max_rot_deg =
            static_cast<float>(declare_parameter<double>("loop_closure_max_rot_deg", 180.0));
        params_.loop_closure_keyframe_dist_m =
            static_cast<float>(declare_parameter<double>("loop_closure_keyframe_dist_m", 5.0));

        // UQ: mid-freq soft correspondence weighting and P improvement gate
        params_.mid_freq_quality_min_weight =
            static_cast<float>(declare_parameter<double>("mid_freq_quality_min_weight", 0.9));
        params_.mid_freq_quality_weight_power =
            static_cast<float>(declare_parameter<double>("mid_freq_quality_weight_power", 0.0));
        params_.mid_freq_min_p_improvement =
            declare_parameter<double>("mid_freq_min_p_improvement", 0.0);
        // UQ: low-freq soft correspondence weighting
        params_.low_freq_quality_min_weight =
            static_cast<float>(declare_parameter<double>("low_freq_quality_min_weight", 0.9));
        params_.low_freq_quality_weight_power =
            static_cast<float>(declare_parameter<double>("low_freq_quality_weight_power", 0.0));

        // AMR: adaptive per-tier frequency ranges
        params_.W_f_min_ms = declare_parameter<double>("W_f_min_ms", 10.0);
        params_.W_f_max_ms = declare_parameter<double>("W_f_max_ms", 10.0);
        params_.W_m_min_ms = declare_parameter<double>("W_m_min_ms", 100.0);
        params_.W_m_max_ms = declare_parameter<double>("W_m_max_ms", 100.0);
        params_.fast_quality_bad  = declare_parameter<double>("fast_quality_bad",  0.0);
        params_.fast_quality_good = declare_parameter<double>("fast_quality_good", 1.0);
        params_.fast_state_seed_mid_enable =
            declare_parameter<bool>("fast_state_seed_mid_enable", false);
        params_.fast_degen_early_mid_enable =
            declare_parameter<bool>("fast_degen_early_mid_enable", false);
        params_.fast_degen_early_mid_thresh =
            declare_parameter<double>("fast_degen_early_mid_thresh", 5.0);
        params_.omega_ema_alpha = declare_parameter<double>("omega_ema_alpha", 0.9);
        params_.W_m_omega_lo    = declare_parameter<double>("W_m_omega_lo", 0.3);
        params_.W_m_omega_hi    = declare_parameter<double>("W_m_omega_hi", 2.5);
        params_.W_m_alpha       = declare_parameter<double>("W_m_alpha", 1.0);
        params_.W_l_omega_lo       = declare_parameter<double>("W_l_omega_lo", 0.5);
        params_.W_l_omega_hi       = declare_parameter<double>("W_l_omega_hi", 2.5);
        params_.low_window_geo_conditioning_enable =
            declare_parameter<bool>("low_window_geo_conditioning_enable", false);
        params_.low_window_lambda_good =
            declare_parameter<double>("low_window_lambda_good", 200.0);
        params_.low_window_lambda_bad =
            declare_parameter<double>("low_window_lambda_bad", 70.0);
        params_.W_l_cov_growth_bad = declare_parameter<double>("W_l_cov_growth_bad", 3.0);
        params_.W_l_smooth_alpha   = declare_parameter<double>("W_l_smooth_alpha", 0.0);
        params_.W_l_imu_consistency_thresh =
            declare_parameter<double>("W_l_imu_consistency_thresh", 0.0);
        params_.backward_correction_enable =
            declare_parameter<bool>("backward_correction_enable", false);

        RCLCPP_INFO(get_logger(),
            "v37-adaptive params: iter=%d extrinsic_est=%s state_undistort=%s legacy_mix=%s[rot_cov=%.1e trans_cov=%.1e] ext_gate=%s[gyro=%.2f quality=%.3f rms=%.3f] adaptive_ext=%s[high_gyro=%.2f count=%d quality=%.3f rms=%.3f] publish_delay=%.0fms odom_offset=%.1fms online_latency=%s[range=%.1f..%.1fms alpha=%.2f] stable_gate=%s[min_pts=%d quality=%.3f rms=%.3f norm=%.2f] main_fast=%s main_corrected=%s corr_gate=%s[innov=%.2f speed=%.2f] update_guard=%s[pos=%.2f rot=%.1f speed=%.1f] low_gate[min_pts=%d quality=%.3f rms=%.3f pos=%.2f rot=%.1fdeg] trust=%s[reject=%.2f full=%.2f]",
            params_.num_max_iterations,
            params_.extrinsic_est_en ? "on" : "off",
            params_.state_extrinsic_undistort ? "on" : "off",
            params_.legacy_mixed_extrinsic_undistort ? "on" : "off",
            params_.extrinsic_rot_cov,
            params_.extrinsic_trans_cov,
            params_.extrinsic_update_gate_enable ? "on" : "off",
            params_.extrinsic_update_min_angular_velocity,
            params_.extrinsic_update_min_quality,
            params_.extrinsic_update_max_residual_rmse,
            params_.extrinsic_update_adaptive_enable ? "on" : "off",
            params_.extrinsic_update_adaptive_high_angular_velocity,
            params_.extrinsic_update_adaptive_min_high_count,
            params_.extrinsic_update_adaptive_min_quality,
            params_.extrinsic_update_adaptive_max_residual_rmse,
            params_.publish_start_delay_ms,
            odometry_time_offset_ms_,
            online_latency_estimation_enable_ ? "on" : "off",
            latency_min_ms_,
            latency_max_ms_,
            latency_smoothing_alpha_,
            params_.publish_requires_stable_update ? "on" : "off",
            params_.publish_stable_min_correspondences,
            params_.publish_stable_min_quality,
            params_.publish_stable_max_residual_rmse,
            params_.publish_stable_min_state_norm,
            params_.publish_fast_odometry_on_main ? "on" : "off",
            params_.publish_corrected_odometry_on_main ? "on" : "off",
            params_.correction_publish_gate_enable ? "on" : "off",
            params_.correction_publish_max_innovation,
            params_.correction_publish_max_speed,
            params_.update_motion_guard_enable ? "on" : "off",
            params_.update_motion_guard_max_pos_delta,
            params_.update_motion_guard_max_rot_deg,
            params_.update_motion_guard_max_speed,
            params_.low_freq_min_correspondences,
            params_.low_freq_min_quality,
            params_.low_freq_max_residual_rmse,
            params_.low_freq_max_pos_delta,
            params_.low_freq_max_rot_delta_deg,
            params_.low_freq_trust_enable ? "on" : "off",
            params_.low_freq_trust_reject,
            params_.low_freq_trust_full);
    }

    rclcpp::Time odom_stamp(double t) const {
        const double t_offset = t + current_latency_ms() * 1e-3;
        return rclcpp::Time(static_cast<int64_t>(t_offset * 1e9));
    }

    double current_latency_ms() const {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        return latency_estimate_ms_;
    }

    void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();

        double t_base = msg->header.stamp.sec +
                        msg->header.stamp.nanosec * 1e-9;
        observe_sensor_arrival(t_base);

        for (const auto& p : msg->points) {
            pcl::PointXYZINormal pt;
            pt.x = p.x;  pt.y = p.y;  pt.z = p.z;
            pt.intensity = p.reflectivity;
            pt.curvature = static_cast<float>(p.offset_time) * 1e-6f;  // ns → ms
            cloud->push_back(pt);
        }

        estimator_->push_livox_points(t_base, cloud);
    }

    // PointCloud2 from Livox Mid-360 via livox_ros_driver2 in PointCloud2 mode.
    // Field layout (point_step=26): x(f32,0) y(f32,4) z(f32,8)
    //   intensity(f32,12) tag(u8,16) line(u8,17) timestamp(f64,18)
    void pc2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        const uint32_t ps = msg->point_step;
        const uint8_t* data = msg->data.data();
        const uint32_t n = msg->width * msg->height;

        // Locate timestamp field offset dynamically for robustness
        int ts_offset = -1;
        for (const auto& f : msg->fields) {
            if (f.name == "timestamp") { ts_offset = static_cast<int>(f.offset); break; }
        }

        double t_base = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        observe_sensor_arrival(t_base);
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
        cloud->reserve(n);

        for (uint32_t i = 0; i < n; i++) {
            const uint8_t* p = data + i * ps;
            float x, y, z, inten;
            std::memcpy(&x,     p + 0,  4);
            std::memcpy(&y,     p + 4,  4);
            std::memcpy(&z,     p + 8,  4);
            std::memcpy(&inten, p + 12, 4);
            if (x == 0.f && y == 0.f && z == 0.f) continue;

            pcl::PointXYZINormal pt;
            pt.x = x; pt.y = y; pt.z = z;
            pt.intensity = inten;

            if (ts_offset >= 0) {
                double ts_abs;
                std::memcpy(&ts_abs, p + ts_offset, 8);
                double ts_sec = ts_abs * 1e-9;  // Livox stores nanoseconds as float64
                pt.curvature = static_cast<float>((ts_sec - t_base) * 1e3);  // s → ms offset
            } else {
                pt.curvature = 0.f;
            }
            cloud->push_back(pt);
        }
        estimator_->push_livox_points(t_base, cloud);
    }

    void publish_odometry(const state_ikfom& s, double t) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = odom_stamp(t);
        odom.header.frame_id = "map";
        odom.child_frame_id  = "body";

        odom.pose.pose.position.x = s.pos(0);
        odom.pose.pose.position.y = s.pos(1);
        odom.pose.pose.position.z = s.pos(2);

        Eigen::Quaterniond q(s.rot.matrix());
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        odom.twist.twist.linear.x  = s.vel(0);
        odom.twist.twist.linear.y  = s.vel(1);
        odom.twist.twist.linear.z  = s.vel(2);

        pub_odom_->publish(odom);

        // Path
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose   = odom.pose.pose;
        path_.header = odom.header;
        path_.poses.push_back(ps);
        pub_path_->publish(path_);

        // TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header           = odom.header;
        tf.child_frame_id   = "body";
        tf.transform.translation.x = s.pos(0);
        tf.transform.translation.y = s.pos(1);
        tf.transform.translation.z = s.pos(2);
        tf.transform.rotation      = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    void publish_odometry_fast(const state_ikfom& s, double t) {
        record_fast_sample(s, t);
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = odom_stamp(t);
        odom.header.frame_id = "map";
        odom.child_frame_id  = "body";
        odom.pose.pose.position.x = s.pos(0);
        odom.pose.pose.position.y = s.pos(1);
        odom.pose.pose.position.z = s.pos(2);
        Eigen::Quaterniond q(s.rot.matrix());
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.twist.twist.linear.x  = s.vel(0);
        odom.twist.twist.linear.y  = s.vel(1);
        odom.twist.twist.linear.z  = s.vel(2);
        pub_odom_fast_->publish(odom);
    }

    void publish_odometry_corrected(const state_ikfom& s, double t) {
        update_latency_from_corrected(s, t);
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = odom_stamp(t);
        odom.header.frame_id = "map";
        odom.child_frame_id  = "body";
        odom.pose.pose.position.x = s.pos(0);
        odom.pose.pose.position.y = s.pos(1);
        odom.pose.pose.position.z = s.pos(2);
        Eigen::Quaterniond q(s.rot.matrix());
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        pub_odom_corr_->publish(odom);
    }

    void publish_odometry_smooth(const state_ikfom& s, double t) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = odom_stamp(t);
        odom.header.frame_id = "map";
        odom.child_frame_id  = "body";
        odom.pose.pose.position.x = s.pos(0);
        odom.pose.pose.position.y = s.pos(1);
        odom.pose.pose.position.z = s.pos(2);
        Eigen::Quaterniond q(s.rot.matrix());
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        pub_odom_smooth_->publish(odom);
    }

    void observe_sensor_arrival(double sensor_t) {
        if (!online_latency_estimation_enable_) return;
        const double now_s = this->now().seconds();
        const double raw_ms = (now_s - sensor_t) * 1e3;
        // If /use_sim_time is not enabled while replaying epoch-stamped bags,
        // raw delay is meaningless. Ignore those observations instead of
        // driving the estimator to a bound.
        if (!std::isfinite(raw_ms) || std::abs(raw_ms) > 1000.0) return;

        std::lock_guard<std::mutex> lock(latency_mutex_);
        const double clipped =
            std::clamp(raw_ms, latency_min_ms_, latency_max_ms_);
        if (!transport_delay_valid_) {
            transport_delay_ms_ = clipped;
            transport_delay_valid_ = true;
        } else {
            transport_delay_ms_ =
                (1.0 - latency_transport_alpha_) * transport_delay_ms_ +
                latency_transport_alpha_ * clipped;
        }
    }

    struct TimedPose {
        double t = 0.0;
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    };

    void record_fast_sample(const state_ikfom& s, double t) {
        if (!online_latency_estimation_enable_) return;
        std::lock_guard<std::mutex> lock(latency_mutex_);
        TimedPose sample;
        sample.t = t;
        sample.p = s.pos;
        sample.q = Eigen::Quaterniond(s.rot.matrix()).normalized();
        fast_pose_buffer_.push_back(sample);
        const double keep_after = t - std::max(latency_buffer_sec_, 0.1);
        while (!fast_pose_buffer_.empty() &&
               fast_pose_buffer_.front().t < keep_after) {
            fast_pose_buffer_.pop_front();
        }
    }

    bool interpolate_fast_pose_locked(double t_query,
                                      Eigen::Vector3d& p,
                                      Eigen::Quaterniond& q) const {
        if (fast_pose_buffer_.size() < 2) return false;
        if (t_query < fast_pose_buffer_.front().t ||
            t_query > fast_pose_buffer_.back().t) {
            return false;
        }

        for (size_t i = 1; i < fast_pose_buffer_.size(); ++i) {
            const auto& a = fast_pose_buffer_[i - 1];
            const auto& b = fast_pose_buffer_[i];
            if (t_query > b.t) continue;
            const double dt = b.t - a.t;
            const double u = dt > 1e-6 ? (t_query - a.t) / dt : 0.0;
            const double uc = std::clamp(u, 0.0, 1.0);
            p = (1.0 - uc) * a.p + uc * b.p;
            q = a.q.slerp(uc, b.q).normalized();
            return true;
        }
        return false;
    }

    void update_latency_from_corrected(const state_ikfom& s, double t) {
        if (!online_latency_estimation_enable_) return;
        if (s.vel.norm() < latency_min_speed_mps_) return;

        double new_estimate = 0.0;
        double best_score = std::numeric_limits<double>::infinity();
        double transport_ms = 0.0;
        bool transport_valid = false;
        bool update_gated = false;
        {
            std::lock_guard<std::mutex> lock(latency_mutex_);
            const double step_ms = std::max(latency_search_step_ms_, 0.1);
            const int steps =
                std::max(1, static_cast<int>(std::ceil(latency_search_radius_ms_ / step_ms)));
            transport_ms = transport_delay_ms_;
            transport_valid = transport_delay_valid_;
            if (latency_update_requires_transport_agreement_ &&
                (!transport_valid ||
                     std::abs(transport_ms - latency_estimate_ms_) >
                     latency_transport_agreement_ms_)) {
                new_estimate = latency_estimate_ms_;
                best_score = -1.0;
                update_gated = true;
            } else {

                for (int k = -steps; k <= steps; ++k) {
                    const double cand_ms = std::clamp(
                        latency_estimate_ms_ + static_cast<double>(k) * step_ms,
                        latency_min_ms_, latency_max_ms_);
                    Eigen::Vector3d pf;
                    Eigen::Quaterniond qf;
                    if (!interpolate_fast_pose_locked(t + cand_ms * 1e-3, pf, qf)) {
                        continue;
                    }

                    const Eigen::Quaterniond qc =
                        Eigen::Quaterniond(s.rot.matrix()).normalized();
                    const double pos_score =
                        (pf - s.pos).squaredNorm() /
                        std::max(latency_pos_sigma_m_ * latency_pos_sigma_m_, 1e-8);
                    const double rot_rad =
                        Eigen::AngleAxisd(qf.conjugate() * qc).angle();
                    const double rot_sigma_rad =
                        std::max(latency_rot_sigma_deg_ * M_PI / 180.0, 1e-6);
                    const double rot_score = (rot_rad * rot_rad) /
                                             (rot_sigma_rad * rot_sigma_rad);
                    const double transport_score =
                        transport_valid
                            ? latency_transport_weight_ *
                                  (cand_ms - transport_ms) * (cand_ms - transport_ms)
                            : 0.0;
                    const double score = pos_score + rot_score + transport_score;
                    if (score < best_score) {
                        best_score = score;
                        new_estimate = cand_ms;
                    }
                }
            }

            if (!update_gated && !std::isfinite(best_score) && transport_valid) {
                new_estimate =
                    std::clamp(transport_ms, latency_min_ms_, latency_max_ms_);
                best_score = 0.0;
            }
            if (!std::isfinite(best_score)) return;

            if (!update_gated) {
                latency_estimate_ms_ =
                    std::clamp((1.0 - latency_smoothing_alpha_) * latency_estimate_ms_ +
                                   latency_smoothing_alpha_ * new_estimate,
                               latency_min_ms_, latency_max_ms_);
            }
            new_estimate = latency_estimate_ms_;
        }

        publish_latency_diagnostics(new_estimate, transport_ms,
                                    transport_valid ? 1.0 : 0.0, best_score);
    }

    void publish_latency_diagnostics(double estimate_ms,
                                     double transport_ms,
                                     double transport_valid,
                                     double score) {
        if (!pub_latency_) return;
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {estimate_ms, transport_ms, transport_valid, score};
        pub_latency_->publish(msg);
    }

    void publish_adaptive_windows(double Wh, double Wl) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {Wh, Wl};
        pub_freq_->publish(msg);
    }

    std::string lidar_topic_, imu_topic_, output_prefix_;
    int lidar_type_ = 0;
    double odometry_time_offset_ms_ = 0.0;
    bool online_latency_estimation_enable_ = false;
    double latency_estimate_ms_ = 0.0;
    double latency_min_ms_ = -30.0;
    double latency_max_ms_ = 30.0;
    double latency_search_radius_ms_ = 8.0;
    double latency_search_step_ms_ = 1.0;
    double latency_smoothing_alpha_ = 0.08;
    double latency_transport_alpha_ = 0.05;
    double latency_transport_weight_ = 0.05;
    bool latency_update_requires_transport_agreement_ = true;
    double latency_transport_agreement_ms_ = 5.0;
    double latency_pos_sigma_m_ = 0.05;
    double latency_rot_sigma_deg_ = 2.0;
    double latency_min_speed_mps_ = 0.10;
    double latency_buffer_sec_ = 1.0;
    mutable std::mutex latency_mutex_;
    bool transport_delay_valid_ = false;
    double transport_delay_ms_ = 0.0;
    std::deque<TimedPose> fast_pose_buffer_;
    MfLioParams params_;

    std::unique_ptr<MfLioEstimator> estimator_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_fast_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_corr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_smooth_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_freq_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_latency_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path path_;
};

int main(int argc, char** argv) {
    signal(SIGSEGV, sigsegv_handler);
    signal(SIGABRT, sigsegv_handler);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MfLioNode>());
    rclcpp::shutdown();
    return 0;
}
