#pragma once
// InEKF filter, ported from mf_lio_inekf (a separate, older-lineage package) into
// faster_lio_multiresolution's own namespace so it can run alongside v51's existing
// IKFoM-based kf_ as an alternate, flag-gated (riekf_enable) filter path -- see
// mf_lio_estimator.h's riekf_state_/riekf_filter_ members. Source math unchanged
// from the original; only the wrapping namespace and this header comment differ.
// (The original also #included "mf_lio/se2_3.hpp", but nothing in this file
// actually calls into that namespace -- propagate() below does its own direct
// Euler integration and analytic F/G, not a group-exponential composition -- so
// that dependency is dropped here rather than carried over unused.)
//
// State DOF layout (24): [φ(3)|δv_b(3)|δp_b(3)|δbg(3)|δba(3)|δRL(3)|δTL(3)|δgrav(3)]
// Left-invariant error convention: η = X̂⁻¹·X.
// Position/velocity errors are in BODY frame; rotation is right-perturbation (same as IKFoM).
// After measurement update: X_new = X_est · Exp(δξ_core), biases updated Euclidean.

#include <Eigen/Dense>
#include "mf_lio/so3_math.h"

namespace mf_lio {
namespace riekf {

static constexpr int INEKF_DOF = 24;

struct InEKFState {
    // Navigation core — same field names as state_ikfom for minimal node changes
    Eigen::Vector3d pos    = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot    = Eigen::Matrix3d::Identity();  // Eigen Matrix3d (not IKFoM SO3)
    Eigen::Vector3d vel    = Eigen::Vector3d::Zero();

    Eigen::Vector3d bg     = Eigen::Vector3d::Zero();   // gyro bias
    Eigen::Vector3d ba     = Eigen::Vector3d::Zero();   // accel bias
    Eigen::Matrix3d offset_R_L_I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d offset_T_L_I = Eigen::Vector3d::Zero();
    Eigen::Vector3d grav   = Eigen::Vector3d(0, 0, -9.81);
};

// Compute left-invariant box-minus: dx = state - ref, DOF=24
inline Eigen::Matrix<double,INEKF_DOF,1>
inekf_boxminus(const InEKFState& x, const InEKFState& ref) {
    Eigen::Matrix<double,INEKF_DOF,1> dx;
    dx.segment<3>(0)  = Log(Eigen::Matrix3d(ref.rot.transpose() * x.rot));
    dx.segment<3>(3)  = x.vel - ref.vel;
    dx.segment<3>(6)  = x.pos - ref.pos;
    dx.segment<3>(9)  = x.bg  - ref.bg;
    dx.segment<3>(12) = x.ba  - ref.ba;
    dx.segment<3>(15) = Log(Eigen::Matrix3d(ref.offset_R_L_I.transpose() * x.offset_R_L_I));
    dx.segment<3>(18) = x.offset_T_L_I - ref.offset_T_L_I;
    dx.segment<3>(21) = x.grav - ref.grav;
    return dx;
}

// Apply scaled box-plus: result = ref ⊞ (alpha * dx)
inline InEKFState
inekf_boxplus(const InEKFState& ref, const Eigen::Matrix<double,INEKF_DOF,1>& dx, double alpha=1.0) {
    InEKFState r = ref;
    r.rot          = ref.rot          * Exp<double>(alpha * dx.segment<3>(0));
    r.vel          = ref.vel          + alpha * dx.segment<3>(3);
    r.pos          = ref.pos          + alpha * dx.segment<3>(6);
    r.bg           = ref.bg           + alpha * dx.segment<3>(9);
    r.ba           = ref.ba           + alpha * dx.segment<3>(12);
    r.offset_R_L_I = ref.offset_R_L_I * Exp<double>(alpha * dx.segment<3>(15));
    r.offset_T_L_I = ref.offset_T_L_I + alpha * dx.segment<3>(18);
    r.grav         = ref.grav         + alpha * dx.segment<3>(21);
    return r;
}

class InEKFFilter {
public:
    InEKFState x;
    Eigen::Matrix<double,INEKF_DOF,INEKF_DOF> P =
        Eigen::Matrix<double,INEKF_DOF,INEKF_DOF>::Identity() * 0.01;

    void init(const InEKFState& x0,
              const Eigen::Matrix<double,INEKF_DOF,INEKF_DOF>& P0) {
        x = x0;
        P = P0;
    }

    // Left-invariant Euler propagation.
    // Q is 12×12 process noise: [n_gyro(3), n_acc(3), n_bg(3), n_ba(3)].
    // F_out/G_out (optional): if non-null, receive the exact analytic F/G used for
    // this step's P update -- lets a validation harness finite-difference-check the
    // real code path instead of a re-typed copy of the derivation.
    void propagate(double dt,
                   const Eigen::Vector3d& omega_meas,
                   const Eigen::Vector3d& acc_meas,
                   const Eigen::Matrix<double,12,12>& Q,
                   Eigen::Matrix<double,INEKF_DOF,INEKF_DOF>* F_out = nullptr,
                   Eigen::Matrix<double,INEKF_DOF,12>* G_out = nullptr)
    {
        if (dt <= 0.0 || dt > 0.5) return;

        const Eigen::Vector3d omega_c = omega_meas - x.bg;
        const Eigen::Vector3d acc_c   = acc_meas   - x.ba;

        // ---- Euler state integration ----
        const Eigen::Matrix3d R_old = x.rot;
        const Eigen::Vector3d v_old = x.vel;

        x.rot = R_old * Exp<double>(omega_c * dt);
        x.vel = v_old + (R_old * acc_c + x.grav) * dt;
        x.pos = x.pos + v_old * dt;

        // ---- Left-invariant Jacobian F (24×24) ----
        // DOF: [φ(0:3)|δv_b(3:6)|δp_b(6:9)|δbg(9:12)|δba(12:15)|δRL(15:18)|δTL(18:21)|δgrav(21:24)]
        // Derived from explicit body-frame error propagation analysis.
        using M24 = Eigen::Matrix<double,INEKF_DOF,INEKF_DOF>;
        M24 F = M24::Identity();

        const Eigen::Matrix3d Kw = skew_sym_mat(omega_c);  // [ω_c]×
        const Eigen::Matrix3d Ka = skew_sym_mat(acc_c);    // [a_c]×

        // φ row: φ_{k+1} = φ_k  −  δbg * dt
        F.block<3,3>(0,0)   -= Kw * dt;   // -[ω_c]× * dt (body-frame rotation self-decay)
        F.block<3,3>(0,9)    = -Eigen::Matrix3d::Identity() * dt;   // φ ← δbg

        // δv_b row: −[a_c]× φ dt  +  (I − [ω_c]× dt) δv_b  −  δba dt
        F.block<3,3>(3,0)    = -Ka * dt;           // δv_b ← φ (body-frame, traj-independent!)
        F.block<3,3>(3,3)   -= Kw * dt;            // δv_b self
        F.block<3,3>(3,12)   = -Eigen::Matrix3d::Identity() * dt;   // δv_b ← δba

        // δv_b ← δgrav: world-frame gravity variation → body-frame velocity error.
        // Use full R^T dt coupling; gravity covariance stays well-conditioned (no rank reduction).
        F.block<3,3>(3,21)   = x.rot.transpose() * dt;

        // δp_b row: I dt δv_b  +  (I − [ω_c]× dt) δp_b
        F.block<3,3>(6,3)    = Eigen::Matrix3d::Identity() * dt;    // δp_b ← δv_b
        F.block<3,3>(6,6)   -= Kw * dt;            // δp_b self

        // Extrinsic / gravity blocks: identity (no dynamics)
        // Already set by M24::Identity() initialization.

        // ---- Process noise input matrix G (24×12) ----
        // w = [n_gyro(3), n_acc(3), n_bg(3), n_ba(3)]
        Eigen::Matrix<double,INEKF_DOF,12> G = Eigen::Matrix<double,INEKF_DOF,12>::Zero();
        G.block<3,3>(0,0)   = -Eigen::Matrix3d::Identity();   // gyro noise → φ
        G.block<3,3>(3,3)   = -Eigen::Matrix3d::Identity();   // acc  noise → δv_b
        G.block<3,3>(9,6)   =  Eigen::Matrix3d::Identity();   // gyro bias noise
        G.block<3,3>(12,9)  =  Eigen::Matrix3d::Identity();   // acc  bias noise

        P = F * P * F.transpose() + G * Q * G.transpose();
        if (F_out) *F_out = F;
        if (G_out) *G_out = G;
    }

    // iEKF re-linearization step: update state only, do NOT update P.
    // Uses Woodbury identity (normal equations) for O(DOF³) cost vs O(effct³).
    // M = P^{-1} + H^T H / σ²  (24×24);  δξ = M^{-1} H^T z / σ²
    double update_state_only(const Eigen::MatrixXd& H,
                             const Eigen::VectorXd& z,
                             const Eigen::Matrix<double,INEKF_DOF,INEKF_DOF>& P_pred,
                             double meas_noise_var)
    {
        const int m = static_cast<int>(H.rows());
        if (m == 0) return 0.0;

        const double inv_s2 = 1.0 / meas_noise_var;
        using M24 = Eigen::Matrix<double, INEKF_DOF, INEKF_DOF>;
        // Normal equations: (P^{-1} + H^T H / σ²) δξ = H^T z / σ²
        M24 M = P_pred.inverse() + inv_s2 * H.transpose() * H;
        Eigen::Matrix<double,INEKF_DOF,1> dxi =
            M.ldlt().solve(inv_s2 * H.transpose() * z);

        const Eigen::Matrix3d R_est = x.rot;
        x.rot = R_est * Exp<double>(dxi.head<3>());
        x.vel = x.vel + R_est * dxi.segment<3>(3);
        x.pos = x.pos + R_est * dxi.segment<3>(6);
        x.bg           += dxi.segment<3>(9);
        x.ba           += dxi.segment<3>(12);
        x.offset_R_L_I  = x.offset_R_L_I * Exp<double>(dxi.segment<3>(15));
        x.offset_T_L_I += dxi.segment<3>(18);
        // Project gravity correction onto tangent space of S² (direction only, not magnitude).
        // IKFoM's MTK::S2 does the same: update is the exponential map on the sphere.
        {
            const double g_mag = x.grav.norm();
            const Eigen::Vector3d g_hat2 = x.grav / g_mag;
            const Eigen::Vector3d dg = dxi.segment<3>(21);
            // Remove radial component — only tangent perturbation changes gravity direction.
            const Eigen::Vector3d dg_tang = dg - g_hat2 * g_hat2.dot(dg);
            x.grav += dg_tang;
            if (g_mag > 1e-6) x.grav = x.grav.normalized() * g_mag;
        }

        return dxi.norm();
    }

    // Final covariance update after iEKF convergence (Joseph form, O(DOF³)).
    // K = M^{-1} H^T / σ²  where  M = P^{-1} + H^T H / σ²
    void update_covariance(const Eigen::MatrixXd& H,
                           const Eigen::VectorXd& z,
                           double meas_noise_var)
    {
        const int m = static_cast<int>(H.rows());
        if (m == 0) return;

        const double inv_s2 = 1.0 / meas_noise_var;
        using M24 = Eigen::Matrix<double, INEKF_DOF, INEKF_DOF>;
        M24 M = P.inverse() + inv_s2 * H.transpose() * H;
        M24 M_inv = M.ldlt().solve(M24::Identity());

        // K = M^{-1} H^T / σ²  (DOF × effct via 24×24 × 24×effct)
        Eigen::MatrixXd K = inv_s2 * M_inv * H.transpose();

        // Joseph form: P_new = (I-KH) P (I-KH)^T + σ² K K^T
        M24 IKH = M24::Identity() - K * H;
        P = IKH * P * IKH.transpose() + meas_noise_var * (K * K.transpose());
    }
};

}  // namespace riekf
}  // namespace mf_lio
