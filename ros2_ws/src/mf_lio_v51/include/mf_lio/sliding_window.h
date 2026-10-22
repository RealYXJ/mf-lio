#pragma once

#include <deque>
#include <Eigen/Dense>
#include "mf_lio/common_lib.h"
#include "mf_lio/use-ikfom.hpp"

// ---------------------------------------------------------------------------
// Sliding-window Gauss-Newton optimizer for multi-resolution LiDAR-inertial
// odometry.  Three tiers (fast/mid/low) contribute scan-to-map factors at
// motion-triggered intervals; IMU preintegration ties consecutive keyframes.
//
// State per keyframe: 6-DOF [pos(3), rot(3)] (right-perturbation).
// The remaining state (vel, bg, ba, grav) is updated by the iEKF; the graph
// corrects only the pose block and writes it back to kf_.
// ---------------------------------------------------------------------------

// IMU relative-motion constraint between two consecutive keyframes.
// delta_pos is in the body frame of the *from* keyframe.
struct ImuConstraint {
    double dt        = 0.0;
    V3D    delta_pos = V3D::Zero();   // R_from^T * (p_to - p_from) at linearisation
    SO3    delta_rot;                  // R_from^T * R_to  at linearisation
    double sigma_pos = 0.05;          // 1-sigma position noise (m)
    double sigma_rot = 0.10;          // 1-sigma rotation noise (rad)
};

// One keyframe in the sliding window.
struct GraphKeyframe {
    double          timestamp = 0.0;
    state_ikfom     x;                // current pose estimate (23-DOF)

    // Scan-to-map factor captured at iEKF convergence.
    // H_6: M×6 (columns 0-2 = pos Jac, 3-5 = rot Jac in right-perturbation).
    // r:   M×1 residuals (= ekfom_data.h at convergence, unit = m).
    Eigen::MatrixXd H_6;              // M×6
    Eigen::VectorXd r;                // M
    double          sigma2 = 0.001;   // LASER_POINT_COV
    bool            has_measurement = false;
};

// ---------------------------------------------------------------------------
class SlidingWindowOptimizer {
public:
    explicit SlidingWindowOptimizer(int window_size = 4);

    // Add a new keyframe; imu is the preintegrated constraint from the
    // previous keyframe to this one (ignored if the window is currently empty).
    // Returns true when a keyframe was dropped (window was full).
    bool add_keyframe(const GraphKeyframe& kf, const ImuConstraint& imu);

    // Set / update the scan-to-map measurement for the latest keyframe.
    void set_latest_measurement(const Eigen::MatrixXd& H_6,
                                const Eigen::VectorXd& r,
                                double sigma2);

    // Run Gauss-Newton over the full window.  Requires >= 2 keyframes.
    void optimize(int max_iter = 5);

    // Return the optimised state of the most-recent keyframe.
    const state_ikfom& get_latest() const { return window_.back().x; }

    int window_size() const { return static_cast<int>(window_.size()); }

    // Direct const access for inspection / logging.
    const GraphKeyframe& keyframe(int i) const { return window_[i]; }

private:
    // SO(3) utilities (small-angle safe)
    static V3D   so3_log(const SO3& R);
    static SO3   so3_exp(const V3D& w);
    static Eigen::Matrix3d skew(const V3D& v);

    // Drop the oldest keyframe (no full marginalization — simpler fixed-lag).
    void drop_oldest();

    int K_;
    std::deque<GraphKeyframe> window_;
    // imu_cons_[i] is the constraint from window_[i] to window_[i+1].
    std::deque<ImuConstraint> imu_cons_;
};
