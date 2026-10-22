#pragma once

// RDEF / Covariance-Intersection fusion diagnostic (disclosed negative).
//
// Reconstructs the diagnostic described in the paper's "Fast/Mid Fusion:
// RDEF and Covariance Intersection (disclosed negative)" section. Between
// mid-tier corrections, the fast tier accumulates its own small pose
// correction relative to the SAME shared prior (P0) that the next mid-tier
// update also starts from. Treating that accumulated correction as the fast
// tier's own information update Lambda_F gives it a well-defined posterior
// covariance, which can then be fused with the mid tier's own (already
// computed) posterior in two different ways:
//
//   - RDEF (Recursion-Derived Exact Fusion): uses the analytically known
//     cross-covariance induced by the shared prior (Bar-Shalom/Campo
//     track-to-track fusion with correlated estimates).
//   - Covariance Intersection (CI): a conservative alternative that does not
//     assume the cross-covariance is known, found instead via a 1-D grid
//     search over the mixing weight omega that minimizes trace(P_fused).
//
// This header is pure math: plain Eigen matrices in, plain Eigen matrices
// out, no estimator-specific types, no I/O, no estimator state mutation.
// It is deliberately standalone so it can be unit-tested without pulling in
// the rest of the estimator.
//
// IMPORTANT: this is a diagnostic-only computation. Nothing in the
// estimator should ever feed P_fused back into the published state/odometry
// output -- see the call site in mf_lio_estimator.cpp's mid_freq_update for
// the enforced side-channel-only usage.

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mf_lio {
namespace rdef_ci {

struct RdefResult {
    Eigen::MatrixXd P_corr_F;     // fast tier's own posterior covariance: (P0^-1 + Lambda_F)^-1
    Eigen::MatrixXd P_fused_rdef; // RDEF-fused covariance
};

struct CiResult {
    double          omega = 0.0;  // grid-searched mixing weight in [0,1]
    Eigen::MatrixXd P_fused_ci;   // CI-fused covariance at the chosen omega
};

// RDEF (Recursion-Derived Exact Fusion).
//
//   P_corr_F  = (P0^-1 + Lambda_F)^-1
//   C_cross   = P_corr_F * P0^-1 * P_corr_M
//   P_fused   = P_corr_F - (P_corr_F - C_cross) * S^-1 * (P_corr_F - C_cross)^T
//   where S   = P_corr_F + P_corr_M - C_cross - C_cross^T
//
// P_corr_M is the mid tier's own IESKF posterior covariance -- it is used
// as-is (nothing new needs to be computed there; it is already the
// estimator's existing output).
//
// Note: this intentionally returns covariance-only results. The fused-state
// formula (x_fused = x_F + (P_F - C) S^-1 (x_M - x_F)) is the natural
// companion but is NOT computed here, since this diagnostic never produces
// a state that is allowed to affect the estimator's published output --
// only trace(P_fused) is of interest (see paper's reported finding).
inline RdefResult compute_rdef_fusion(const Eigen::MatrixXd& P0,
                                       const Eigen::MatrixXd& Lambda_F,
                                       const Eigen::MatrixXd& P_corr_M) {
    const int n = static_cast<int>(P0.rows());
    RdefResult out;

    const Eigen::MatrixXd P0_inv = P0.inverse();
    out.P_corr_F = (P0_inv + Lambda_F).inverse();

    const Eigen::MatrixXd C_cross = out.P_corr_F * P0_inv * P_corr_M;
    const Eigen::MatrixXd diff    = out.P_corr_F - C_cross;
    const Eigen::MatrixXd S       = out.P_corr_F + P_corr_M - C_cross - C_cross.transpose();

    // S can be ill-conditioned (e.g. Lambda_F -> 0 degenerates the fusion,
    // since the fast tier then carries no information beyond the prior).
    // Use a pseudo-inverse so the diagnostic stays well-defined everywhere
    // rather than throwing/NaN-ing out on edge cases.
    const Eigen::MatrixXd S_pinv = S.completeOrthogonalDecomposition().pseudoInverse();

    out.P_fused_rdef = out.P_corr_F - diff * S_pinv * diff.transpose();
    (void)n;
    return out;
}

// Covariance Intersection (CI).
//
//   P_fused^-1 = omega * P_corr_F^-1 + (1-omega) * P_corr_M^-1
//
// omega in [0,1] is chosen by a 1-D grid search (grid_steps points,
// inclusive of both endpoints) minimizing trace(P_fused).
inline CiResult compute_ci_fusion(const Eigen::MatrixXd& P_corr_F,
                                   const Eigen::MatrixXd& P_corr_M,
                                   int grid_steps = 101) {
    CiResult out;
    grid_steps = std::max(2, grid_steps);

    const Eigen::MatrixXd P_F_inv = P_corr_F.inverse();
    const Eigen::MatrixXd P_M_inv = P_corr_M.inverse();

    double best_trace = std::numeric_limits<double>::infinity();
    Eigen::MatrixXd best_P = P_corr_M;
    double best_omega = 0.0;

    for (int i = 0; i < grid_steps; ++i) {
        const double omega = static_cast<double>(i) / static_cast<double>(grid_steps - 1);
        const Eigen::MatrixXd P_inv = omega * P_F_inv + (1.0 - omega) * P_M_inv;
        const Eigen::MatrixXd P     = P_inv.inverse();
        const double tr = P.trace();
        if (tr < best_trace) {
            best_trace = tr;
            best_P     = P;
            best_omega = omega;
        }
    }

    out.omega      = best_omega;
    out.P_fused_ci = best_P;
    return out;
}

} // namespace rdef_ci
} // namespace mf_lio
