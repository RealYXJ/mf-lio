#include "mf_lio/sliding_window.h"
#include <cmath>

// ---------------------------------------------------------------------------
SlidingWindowOptimizer::SlidingWindowOptimizer(int K) : K_(K) {}

// ---------------------------------------------------------------------------
// SO(3) utilities
// ---------------------------------------------------------------------------
V3D SlidingWindowOptimizer::so3_log(const SO3& R) {
    Eigen::AngleAxisd aa(R.toRotationMatrix());
    return aa.axis() * aa.angle();
}

SO3 SlidingWindowOptimizer::so3_exp(const V3D& w) {
    double angle = w.norm();
    if (angle < 1e-10) return SO3::Identity();
    Eigen::AngleAxisd aa(angle, w / angle);
    return SO3(Eigen::Quaterniond(aa));
}

Eigen::Matrix3d SlidingWindowOptimizer::skew(const V3D& v) {
    Eigen::Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

// ---------------------------------------------------------------------------
bool SlidingWindowOptimizer::add_keyframe(const GraphKeyframe& kf,
                                           const ImuConstraint& imu) {
    if (!window_.empty()) {
        imu_cons_.push_back(imu);
    }
    window_.push_back(kf);

    if (static_cast<int>(window_.size()) > K_) {
        drop_oldest();
        return true;
    }
    return false;
}

void SlidingWindowOptimizer::set_latest_measurement(
    const Eigen::MatrixXd& H_6,
    const Eigen::VectorXd& r,
    double sigma2)
{
    if (window_.empty()) return;
    auto& kf = window_.back();
    kf.H_6 = H_6;
    kf.r   = r;
    kf.sigma2 = sigma2;
    kf.has_measurement = true;
}

void SlidingWindowOptimizer::drop_oldest() {
    window_.pop_front();
    if (!imu_cons_.empty()) imu_cons_.pop_front();
}

// ---------------------------------------------------------------------------
// Gauss-Newton optimization over the window.
// State: δX ∈ R^{6K}, δX[6i:6i+6] = [δpos_i (3), δrot_i (3)] for keyframe i.
// Right-perturbation: pos += δpos, rot = rot * Exp(δrot).
// ---------------------------------------------------------------------------
void SlidingWindowOptimizer::optimize(int max_iter) {
    const int K = static_cast<int>(window_.size());
    if (K < 1) return;

    const int dim = 6 * K;

    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::MatrixXd HtH = Eigen::MatrixXd::Zero(dim, dim);
        Eigen::VectorXd Htb = Eigen::VectorXd::Zero(dim);

        // ── 1. Scan-to-map factors ────────────────────────────────────────
        for (int i = 0; i < K; ++i) {
            const auto& kf = window_[i];
            if (!kf.has_measurement || kf.H_6.rows() == 0) continue;

            const double inv_s2 = 1.0 / kf.sigma2;
            const int col = 6 * i;

            // HtH[6i:6i+6, 6i:6i+6] += H^T H / sigma2
            HtH.block<6,6>(col, col).noalias() +=
                inv_s2 * kf.H_6.transpose() * kf.H_6;

            // Htb[6i:6i+6] += H^T r / sigma2
            // (iEKF convention: r = -pd, want H δx = r → minimize)
            Htb.segment<6>(col).noalias() +=
                inv_s2 * kf.H_6.transpose() * kf.r;
        }

        // ── 2. IMU relative-motion factors ────────────────────────────────
        for (int i = 0; i < static_cast<int>(imu_cons_.size()); ++i) {
            if (i + 1 >= K) break;

            const ImuConstraint& imu = imu_cons_[i];
            const state_ikfom&   xi  = window_[i].x;
            const state_ikfom&   xi1 = window_[i + 1].x;

            // Residuals (want each → 0)
            // r_pos = R_i^T (p_{i+1} - p_i) - Δp_imu
            const V3D dp = xi1.pos - xi.pos;
            const V3D r_pos = xi.rot.conjugate() * dp - imu.delta_pos;

            // r_rot = log(ΔR_imu^T · R_i^T · R_{i+1})
            const SO3 dR_actual = xi.rot.inverse() * xi1.rot;
            const V3D r_rot = so3_log(imu.delta_rot.inverse() * dR_actual);

            // Jacobians (right-perturbation, see derivation in sliding_window.h)
            // w.r.t. x_i:  J_i (6×6)
            Eigen::Matrix<double,6,6> Ji;
            Ji.setZero();
            Ji.block<3,3>(0,0) = -xi.rot.toRotationMatrix().transpose();
            Ji.block<3,3>(0,3) =  skew(xi.rot.conjugate() * dp);
            // ∂r_rot/∂φ_i ≈ -I  (BCH first-order)
            Ji.block<3,3>(3,3) = -Eigen::Matrix3d::Identity();

            // w.r.t. x_{i+1}:  J_{i+1} (6×6)
            Eigen::Matrix<double,6,6> Ji1;
            Ji1.setZero();
            Ji1.block<3,3>(0,0) = xi.rot.toRotationMatrix().transpose();
            // ∂r_rot/∂φ_{i+1} ≈ +I
            Ji1.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

            // Information matrices (block-diagonal)
            const double inv_sp2 = 1.0 / (imu.sigma_pos * imu.sigma_pos);
            const double inv_sr2 = 1.0 / (imu.sigma_rot * imu.sigma_rot);
            Eigen::Matrix<double,6,6> Lam;
            Lam.setZero();
            Lam.block<3,3>(0,0) = inv_sp2 * Eigen::Matrix3d::Identity();
            Lam.block<3,3>(3,3) = inv_sr2 * Eigen::Matrix3d::Identity();

            // Residual vector
            Eigen::Matrix<double,6,1> res;
            res.head<3>() = r_pos;
            res.tail<3>() = r_rot;

            const int ci  = 6 * i;
            const int ci1 = 6 * (i + 1);

            HtH.block<6,6>(ci,  ci ).noalias() += Ji.transpose() * Lam * Ji;
            HtH.block<6,6>(ci1, ci1).noalias() += Ji1.transpose() * Lam * Ji1;
            HtH.block<6,6>(ci,  ci1).noalias() += Ji.transpose() * Lam * Ji1;
            HtH.block<6,6>(ci1, ci ).noalias() += Ji1.transpose() * Lam * Ji;

            // GN: H^T Λ J δX = -H^T Λ r  → Htb stores the RHS (with sign for solve)
            // We solve HtH δX = Htb where Htb = -J^T Λ r
            Htb.segment<6>(ci ).noalias() -= Ji.transpose() * Lam * res;
            Htb.segment<6>(ci1).noalias() -= Ji1.transpose() * Lam * res;
        }

        // ── Tikhonov regularisation ───────────────────────────────────────
        HtH.diagonal().array() += 1e-6;

        // ── Solve ─────────────────────────────────────────────────────────
        Eigen::VectorXd dX = HtH.ldlt().solve(Htb);

        // ── Apply update: x_i ⊕ dX[6i:6i+6] ─────────────────────────────
        for (int i = 0; i < K; ++i) {
            auto& xi = window_[i].x;
            xi.pos += dX.segment<3>(6 * i);
            const V3D drot = dX.segment<3>(6 * i + 3);
            xi.rot = xi.rot * so3_exp(drot);
            xi.rot.normalize();
        }

        // Convergence check
        if (dX.norm() < 1e-5) break;
    }
}
