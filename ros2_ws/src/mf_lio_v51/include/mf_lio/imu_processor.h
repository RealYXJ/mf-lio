#pragma once

#include <deque>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>

#include "mf_lio/use-ikfom.hpp"
#include "mf_lio/common_lib.h"
#include "mf_lio/riekf/inekf_filter.hpp"
#include "mf_lio/riekf/state_convert.hpp"

#define MAX_INI_COUNT (10)

class ImuProcessor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcessor();

    void set_extrinsic(const V3D& T, const M3D& R,
                       bool estimate = false,
                       bool use_state_for_undistort = false,
                       bool legacy_mixed_undistort = false,
                       double rot_cov = 1e-8,
                       double trans_cov = 1e-8);
    void set_cov(double gyr_cov, double acc_cov, double b_gyr_cov, double b_acc_cov);
    void set_init_vel_cov(double v) { init_vel_cov_ = v; }

    // Thread-safe; call from ROS IMU callback
    void push_imu(const sensor_msgs::msg::Imu& msg);

    // Propagates kf forward through buffered IMU in [~last_end, t_end],
    // then undistorts cloud_in → cloud_out to the end-of-window frame.
    // Points must carry curvature = ms offset from t_start.
    // Returns false while still initialising.
    bool process(
        double t_start,
        double t_end,
        const pcl::PointCloud<pcl::PointXYZINormal>& cloud_in,
        pcl::PointCloud<pcl::PointXYZINormal>& cloud_out,
        esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);

    // Forward-propagates state from t_start to t_end using IMU buffer.
    // Pure function: no side effects on kf, traj_abs_, or last_lidar_end_time_.
    // Used by fast_freq to publish IMU-predicted poses at ~100Hz.
    state_ikfom predict_state(double t_start, double t_end,
                               const state_ikfom& x0) const;

    void reset();
    bool initialized() const { return !imu_need_init_; }
    V3D  last_angular_velocity() const { return angvel_last_; }

    // RIEKF ablation: when enabled, propagate() below runs the ported
    // invariant filter (mf_lio::riekf::InEKFFilter) instead of kf's own
    // standard math, then bridges its result back into kf via
    // change_x/change_P -- so every existing consumer of kf.get_x()/get_P()
    // stays unaware of which filter actually produced the state. See
    // mf_lio_estimator.cpp's h_share_model_riekf/run_riekf_correction for
    // the measurement-update half of this same bridge.
    void set_riekf_enable(bool b) { riekf_enable_ = b; }
    mf_lio::riekf::InEKFFilter& riekf_filter() { return riekf_filter_; }
    bool riekf_enabled() const { return riekf_enable_; }

    Eigen::Matrix<double, 12, 12> Q;

private:
    void imu_init(const std::vector<sensor_msgs::msg::Imu>& imus,
                  esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);

    // Seeds riekf_filter_ from (x_before, P_before) on first use, propagates
    // it by this step's dt/omega/acc, then overwrites kf's state/covariance
    // with the result -- called immediately after kf.predict() at both call
    // sites in process(), using the state as of BEFORE that predict() call
    // as the seed (so the seeding cycle doesn't double-propagate).
    void riekf_propagate_and_sync(
        double dt, const V3D& angvel_avr, const V3D& acc_avr,
        const state_ikfom& x_before, const Eigen::Matrix<double, 23, 23>& P_before,
        esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);

    void undistort(
        const pcl::PointCloud<pcl::PointXYZINormal>& cloud_in,
        pcl::PointCloud<pcl::PointXYZINormal>& cloud_out,
        const state_ikfom& end_state,
        double t_start);

    std::deque<sensor_msgs::msg::Imu> buf_;
    mutable std::mutex mtx_;

    sensor_msgs::msg::Imu last_imu_;

    V3D Lidar_T_wrt_IMU_{V3D::Zero()};
    M3D Lidar_R_wrt_IMU_{M3D::Identity()};
    bool estimate_extrinsic_{false};
    bool use_state_extrinsic_for_undistort_{false};
    bool legacy_mixed_extrinsic_undistort_{false};
    double extrinsic_rot_cov_{1e-8};
    double extrinsic_trans_cov_{1e-8};

    V3D cov_gyr_{0.1, 0.1, 0.1};
    V3D cov_acc_{0.1, 0.1, 0.1};
    V3D cov_bias_gyr_{0.0001, 0.0001, 0.0001};
    V3D cov_bias_acc_{0.0001, 0.0001, 0.0001};

    V3D mean_acc_{0, 0, -1};
    V3D mean_gyr_{0, 0, 0};
    V3D angvel_last_{V3D::Zero()};
    V3D acc_s_last_{V3D::Zero()};

    double last_lidar_end_time_{0.0};
    double first_lidar_time_{0.0};
    double init_vel_cov_{1.0};

    int  init_iter_num_{1};
    bool imu_need_init_{true};
    bool b_first_frame_{true};

    bool riekf_enable_{false};
    bool riekf_seeded_{false};
    mf_lio::riekf::InEKFFilter riekf_filter_;

    // Persistent absolute-time pose trajectory across all process() calls.
    // Pose6D::offset_time stores the ABSOLUTE timestamp (seconds) rather than
    // the window-relative offset used in the original single-window design.
    // Accumulated by each process() call; trimmed to the last 2 s.
    std::deque<Pose6D> traj_abs_;
};
