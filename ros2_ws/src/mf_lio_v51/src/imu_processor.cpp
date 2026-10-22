#include "mf_lio/imu_processor.h"
#include <algorithm>
#include <cmath>

ImuProcessor::ImuProcessor()
    : b_first_frame_(true), imu_need_init_(true)
{
    Q = process_noise_cov();
    last_imu_ = sensor_msgs::msg::Imu();
}

void ImuProcessor::reset()
{
    mean_acc_           = V3D(0, 0, -1);
    mean_gyr_           = V3D(0, 0, 0);
    angvel_last_        = Zero3d;
    acc_s_last_         = Zero3d;
    imu_need_init_      = true;
    b_first_frame_      = true;
    init_iter_num_      = 1;
    last_lidar_end_time_ = 0.0;
    traj_abs_.clear();
    std::lock_guard<std::mutex> lk(mtx_);
    buf_.clear();
    last_imu_ = sensor_msgs::msg::Imu();
}

void ImuProcessor::set_extrinsic(const V3D& T, const M3D& R,
                                 bool estimate,
                                 bool use_state_for_undistort,
                                 bool legacy_mixed_undistort,
                                 double rot_cov,
                                 double trans_cov)
{
    Lidar_T_wrt_IMU_ = T;
    Lidar_R_wrt_IMU_ = R;
    estimate_extrinsic_ = estimate;
    use_state_extrinsic_for_undistort_ = use_state_for_undistort;
    legacy_mixed_extrinsic_undistort_ = legacy_mixed_undistort;
    extrinsic_rot_cov_ = rot_cov;
    extrinsic_trans_cov_ = trans_cov;
}

void ImuProcessor::set_cov(double gyr_cov, double acc_cov,
                            double b_gyr_cov, double b_acc_cov)
{
    cov_gyr_      = V3D::Constant(gyr_cov);
    cov_acc_      = V3D::Constant(acc_cov);
    cov_bias_gyr_ = V3D::Constant(b_gyr_cov);
    cov_bias_acc_ = V3D::Constant(b_acc_cov);
}

void ImuProcessor::push_imu(const sensor_msgs::msg::Imu& msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    buf_.push_back(msg);
    // Drop data older than 2 s to bound memory
    while (!buf_.empty()) {
        double oldest = rclcpp::Time(buf_.front().header.stamp).seconds();
        double newest = rclcpp::Time(buf_.back().header.stamp).seconds();
        if (newest - oldest > 2.0)
            buf_.pop_front();
        else
            break;
    }
}

// ---------------------------------------------------------------------------
// IMU gravity/bias initialisation
// ---------------------------------------------------------------------------
void ImuProcessor::imu_init(
    const std::vector<sensor_msgs::msg::Imu>& imus,
    esekfom::esekf<state_ikfom, 12, input_ikfom>& kf)
{
    if (b_first_frame_) {
        b_first_frame_ = false;
        init_iter_num_ = 1;
        const auto& a0 = imus.front().linear_acceleration;
        const auto& g0 = imus.front().angular_velocity;
        mean_acc_  << a0.x, a0.y, a0.z;
        mean_gyr_  << g0.x, g0.y, g0.z;
    }

    for (const auto& imu : imus) {
        V3D cur_acc(imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z);
        V3D cur_gyr(imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z);
        mean_acc_ += (cur_acc - mean_acc_) / init_iter_num_;
        mean_gyr_ += (cur_gyr - mean_gyr_) / init_iter_num_;
        init_iter_num_++;
    }

    if (init_iter_num_ <= MAX_INI_COUNT) return;

    // Gravity direction and gyro bias settled — write into kf state
    state_ikfom init_state = kf.get_x();
    init_state.grav         = S2(-mean_acc_ / mean_acc_.norm() * G_m_s2);
    init_state.bg           = mean_gyr_;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU_;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU_;
    kf.change_x(init_state);

    // Initial covariance — indices matched to state_ikfom in use-ikfom.hpp:
    //   0-2:  pos        3-5:  rot
    //   6-8:  offset_R_L_I    9-11: offset_T_L_I
    //  12-14: vel        15-17: bg     18-20: ba     21-22: grav
    esekfom::esekf<state_ikfom,12,input_ikfom>::cov P = kf.get_P();
    P.setIdentity();
    P(6,6) = P(7,7) = P(8,8) =
        estimate_extrinsic_ ? extrinsic_rot_cov_ : 1e-8;
    P(9,9) = P(10,10) = P(11,11) =
        estimate_extrinsic_ ? extrinsic_trans_cov_ : 1e-8;
    P(12,12) = P(13,13) = P(14,14) = init_vel_cov_;  // vel: 1.0 for ground, 0.05 for high-altitude UAV
    P(15,15) = P(16,16) = P(17,17) = 1e-4; // bg
    P(18,18) = P(19,19) = P(20,20) = 1e-3; // ba
    P(21,21) = P(22,22) = 1e-5;            // grav S2
    kf.change_P(P);

    imu_need_init_ = false;
}

// ---------------------------------------------------------------------------
// Backward undistortion: compensate each point to end-of-window frame.
// Uses traj_abs_ whose offset_time stores ABSOLUTE timestamps so that
// both high_freq (20 ms) and low_freq (200 ms) windows are correctly covered
// even when process() was called incrementally for each high_freq step.
// ---------------------------------------------------------------------------
void ImuProcessor::undistort(
    const pcl::PointCloud<pcl::PointXYZINormal>& cloud_in,
    pcl::PointCloud<pcl::PointXYZINormal>& cloud_out,
    const state_ikfom& end_state,
    double t_start)
{
    cloud_out = cloud_in;

    if (traj_abs_.size() < 2) return;

    M3D R_fwd = Lidar_R_wrt_IMU_;
    V3D T_fwd = Lidar_T_wrt_IMU_;
    M3D R_inv = Lidar_R_wrt_IMU_;
    V3D T_inv = Lidar_T_wrt_IMU_;
    if (legacy_mixed_extrinsic_undistort_) {
        R_inv = end_state.offset_R_L_I;
        T_inv = end_state.offset_T_L_I;
    } else if (estimate_extrinsic_ || use_state_extrinsic_for_undistort_) {
        R_fwd = end_state.offset_R_L_I;
        T_fwd = end_state.offset_T_L_I;
        R_inv = end_state.offset_R_L_I;
        T_inv = end_state.offset_T_L_I;
    }

    // Sort points by time (ascending curvature = ms offset from t_start)
    std::sort(cloud_out.points.begin(), cloud_out.points.end(),
              [](const pcl::PointXYZINormal& a, const pcl::PointXYZINormal& b){
                  return a.curvature < b.curvature;
              });

    auto it_pt = cloud_out.points.end() - 1;

    for (auto it_kp = traj_abs_.end() - 1; it_kp != traj_abs_.begin(); --it_kp) {
        const Pose6D& head = *(it_kp - 1);
        const Pose6D& tail = *it_kp;

        M3D R_head;
        R_head << MAT_FROM_ARRAY(head.rot);
        V3D vel_head(VEC_FROM_ARRAY(head.vel));
        V3D pos_head(VEC_FROM_ARRAY(head.pos));
        V3D acc_tail(VEC_FROM_ARRAY(tail.acc));
        V3D gyr_tail(VEC_FROM_ARRAY(tail.gyr));

        // head.offset_time is the ABSOLUTE time of this pose.
        // Point absolute time = t_start + curvature * 1e-3.
        for (; t_start + it_pt->curvature * 1e-3 > head.offset_time
               && it_pt != cloud_out.points.begin(); --it_pt)
        {
            double t_pt_abs = t_start + it_pt->curvature * 1e-3;
            double dt = t_pt_abs - head.offset_time;

            M3D R_pt = R_head * Exp(gyr_tail, dt);

            V3D p_lidar(it_pt->x, it_pt->y, it_pt->z);
            V3D p_imu_t = R_fwd * p_lidar + T_fwd;
            V3D T_delta = pos_head + vel_head * dt + 0.5 * acc_tail * dt * dt
                          - end_state.pos;

            V3D p_end_imu = end_state.rot.conjugate()
                            * (R_pt * p_imu_t + T_delta);
            V3D p_end_lidar = R_inv.conjugate() * (p_end_imu - T_inv);

            it_pt->x = static_cast<float>(p_end_lidar(0));
            it_pt->y = static_cast<float>(p_end_lidar(1));
            it_pt->z = static_cast<float>(p_end_lidar(2));

            if (it_pt == cloud_out.points.begin()) break;
        }
        if (it_pt == cloud_out.points.begin()) break;
    }
}

// ---------------------------------------------------------------------------
// Pure IMU forward propagation — no side effects on any member state.
// Used by fast_freq to publish predicted poses at ~100Hz without touching kf_.
// ---------------------------------------------------------------------------
state_ikfom ImuProcessor::predict_state(double t_start, double t_end,
                                         const state_ikfom& x0) const
{
    std::vector<sensor_msgs::msg::Imu> imus;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (const auto& m : buf_) {
            double t = rclcpp::Time(m.header.stamp).seconds();
            if (t >= t_start - 0.01 && t <= t_end + 0.01)
                imus.push_back(m);
        }
    }
    if (imus.empty()) return x0;

    state_ikfom s = x0;  // local copy — no side effects
    double acc_scale = (mean_acc_.norm() > 0.1) ? (G_m_s2 / mean_acc_.norm()) : 1.0;

    sensor_msgs::msg::Imu prev = imus.front();
    for (size_t i = 1; i < imus.size(); ++i) {
        double t_h = rclcpp::Time(prev.header.stamp).seconds();
        double t_t = rclcpp::Time(imus[i].header.stamp).seconds();
        if (t_t < t_start) { prev = imus[i]; continue; }

        double dt = (t_h < t_start) ? (t_t - t_start) : (t_t - t_h);
        if (dt <= 0.0) { prev = imus[i]; continue; }

        V3D w = 0.5 * (V3D(prev.angular_velocity.x, prev.angular_velocity.y, prev.angular_velocity.z)
                     + V3D(imus[i].angular_velocity.x, imus[i].angular_velocity.y, imus[i].angular_velocity.z))
                - s.bg;
        V3D a = 0.5 * (V3D(prev.linear_acceleration.x, prev.linear_acceleration.y, prev.linear_acceleration.z)
                     + V3D(imus[i].linear_acceleration.x, imus[i].linear_acceleration.y, imus[i].linear_acceleration.z))
                - s.ba;
        a *= acc_scale;
        V3D a_world = s.rot * a;
        for (int k = 0; k < 3; ++k) a_world[k] += s.grav[k];

        s.pos += s.vel * dt + 0.5 * a_world * dt * dt;
        s.vel += a_world * dt;
        s.rot  = s.rot * Exp(w, dt);
        prev   = imus[i];
    }

    // Extrapolate to t_end if last IMU sample is before t_end
    double t_last = rclcpp::Time(imus.back().header.stamp).seconds();
    if (t_last < t_end - 1e-6) {
        double dt = t_end - t_last;
        V3D w(imus.back().angular_velocity.x,
              imus.back().angular_velocity.y,
              imus.back().angular_velocity.z);
        w -= s.bg;
        V3D a(imus.back().linear_acceleration.x,
              imus.back().linear_acceleration.y,
              imus.back().linear_acceleration.z);
        a = (a - s.ba) * acc_scale;
        V3D a_world = s.rot * a;
        for (int k = 0; k < 3; ++k) a_world[k] += s.grav[k];
        s.pos += s.vel * dt + 0.5 * a_world * dt * dt;
        s.vel += a_world * dt;
        s.rot  = s.rot * Exp(w, dt);
    }
    return s;
}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------
void ImuProcessor::riekf_propagate_and_sync(
    double dt, const V3D& angvel_avr, const V3D& acc_avr,
    const state_ikfom& x_before, const Eigen::Matrix<double, 23, 23>& P_before,
    esekfom::esekf<state_ikfom, 12, input_ikfom>& kf)
{
    if (!riekf_enable_) return;
    if (!riekf_seeded_) {
        riekf_filter_.init(mf_lio::riekf::state_ikfom_to_riekf(x_before),
                            mf_lio::riekf::ikfom_P_to_riekf(P_before, x_before));
        riekf_seeded_ = true;
    }
    riekf_filter_.propagate(dt, angvel_avr, acc_avr, Q);
    state_ikfom x_new = mf_lio::riekf::riekf_to_state_ikfom(riekf_filter_.x);
    Eigen::Matrix<double, 23, 23> P_new = mf_lio::riekf::riekf_P_to_ikfom(riekf_filter_.P, x_new);
    kf.change_x(x_new);
    kf.change_P(P_new);
}

bool ImuProcessor::process(
    double t_start,
    double t_end,
    const pcl::PointCloud<pcl::PointXYZINormal>& cloud_in,
    pcl::PointCloud<pcl::PointXYZINormal>& cloud_out,
    esekfom::esekf<state_ikfom, 12, input_ikfom>& kf)
{
    // Snapshot IMU buffer under lock
    std::vector<sensor_msgs::msg::Imu> imus;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return false;
        for (const auto& m : buf_) {
            double t = rclcpp::Time(m.header.stamp).seconds();
            if (t >= last_lidar_end_time_ - 0.01 && t <= t_end + 0.01)
                imus.push_back(m);
        }
    }
    if (imus.empty()) return false;

    // --- Initialisation phase ---
    if (imu_need_init_) {
        if (first_lidar_time_ == 0.0) first_lidar_time_ = t_start;
        imu_init(imus, kf);
        if (imu_need_init_) return false;  // still collecting samples
        last_imu_            = imus.back();
        last_lidar_end_time_ = t_end;
        // Skip this scan: next call will return a properly undistorted cloud for map init.
        return false;
    }

    // --- Forward propagation + persistent trajectory accumulation ---
    // Prepend last_imu_ so propagation starts smoothly from the previous frame
    std::deque<sensor_msgs::msg::Imu> v_imu(imus.begin(), imus.end());
    v_imu.push_front(last_imu_);

    state_ikfom imu_state = kf.get_x();

    // Seed traj_abs_ with the current state at last_lidar_end_time_.
    // offset_time stores the ABSOLUTE timestamp so the undistorter can
    // cover any window length without requiring the window to start here.
    traj_abs_.push_back(
        set_pose6d(last_lidar_end_time_, acc_s_last_, angvel_last_,
                   imu_state.vel, imu_state.pos,
                   imu_state.rot.toRotationMatrix()));

    V3D angvel_avr, acc_avr;
    double dt = 0.0;
    input_ikfom in;

    for (auto it = v_imu.begin(); it + 1 != v_imu.end(); ++it) {
        const auto& head = *it;
        const auto& tail = *(it + 1);

        double t_head = rclcpp::Time(head.header.stamp).seconds();
        double t_tail = rclcpp::Time(tail.header.stamp).seconds();

        if (t_tail < last_lidar_end_time_) continue;

        angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
                      0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
                      0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
        acc_avr    << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
                      0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
                      0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

        acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();

        if (t_head < last_lidar_end_time_)
            dt = t_tail - last_lidar_end_time_;
        else
            dt = t_tail - t_head;

        in.acc  = acc_avr;
        in.gyro = angvel_avr;
        Q.block<3,3>(0,0).diagonal() = cov_gyr_;
        Q.block<3,3>(3,3).diagonal() = cov_acc_;
        Q.block<3,3>(6,6).diagonal() = cov_bias_gyr_;
        Q.block<3,3>(9,9).diagonal() = cov_bias_acc_;
        const state_ikfom x_before = kf.get_x();
        const auto P_before = kf.get_P();
        kf.predict(dt, Q, in);
        riekf_propagate_and_sync(dt, angvel_avr, acc_avr, x_before, P_before, kf);

        imu_state    = kf.get_x();
        angvel_last_ = angvel_avr - imu_state.bg;
        acc_s_last_  = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; ++i) acc_s_last_[i] += imu_state.grav[i];

        // Store with ABSOLUTE timestamp
        traj_abs_.push_back(
            set_pose6d(t_tail, acc_s_last_, angvel_last_,
                       imu_state.vel, imu_state.pos,
                       imu_state.rot.toRotationMatrix()));
    }

    // Propagate exactly to t_end if the last IMU sample is before t_end.
    // Do NOT back-propagate when t_imu_end > t_end (filter already past t_end).
    double t_imu_end = rclcpp::Time(v_imu.back().header.stamp).seconds();
    dt = t_end - t_imu_end;
    if (dt > 1e-6) {
        const state_ikfom x_before_end = kf.get_x();
        const auto P_before_end = kf.get_P();
        kf.predict(dt, Q, in);
        riekf_propagate_and_sync(dt, in.gyro, in.acc, x_before_end, P_before_end, kf);
        imu_state = kf.get_x();
        // Add final pose at t_end
        angvel_last_ = in.gyro - imu_state.bg;
        acc_s_last_  = imu_state.rot * (in.acc - imu_state.ba);
        for (int i = 0; i < 3; ++i) acc_s_last_[i] += imu_state.grav[i];
        traj_abs_.push_back(
            set_pose6d(t_end, acc_s_last_, angvel_last_,
                       imu_state.vel, imu_state.pos,
                       imu_state.rot.toRotationMatrix()));
    } else {
        imu_state = kf.get_x();
    }

    last_imu_            = v_imu.back();
    last_lidar_end_time_ = t_end;

    // Trim trajectory to keep only the last 2 s (max W_l_max is 0.5 s)
    while (traj_abs_.size() > 2 &&
           traj_abs_.front().offset_time < t_end - 2.0)
        traj_abs_.pop_front();

    // --- Undistort points to end-of-window frame ---
    undistort(cloud_in, cloud_out, imu_state, t_start);
    return true;
}
