#pragma once

// Low-tier adaptive window policy (paper Sec. "Adaptive Window Policy",
// eq. policy). W_l is set once per low-tier cycle from a scalar quality
// index q = max(q_rmse, q_omega, q_geo):
//
//   W_l <- W_l_min + (W_l_max - W_l_min) * q
//
// Three clamped-linear terms feed q:
//   - q_rmse: fit-quality term, UNCONDITIONAL, driven by the low tier's own
//     just-completed iEKF point-to-plane residual RMSE. Low residual (good
//     fit) -> q_rmse near 1 (extend window); high residual -> q_rmse near 0
//     (shorten window, re-assess sooner with fresher data).
//   - q_omega: rotation-rate gate, UNCONDITIONAL, driven by an EMA-smoothed
//     gyroscope magnitude. Sustained fast rotation -> q_omega near 1, a
//     protective floor independent of fit quality.
//   - q_geo: geometric-conditioning term (this work), FLAG-GATED (default
//     off => q_geo == 0, recovering exactly the two-term policy). Driven by
//     the smallest eigenvalue of the position information block J_p
//     accumulated over the low tier's own correspondences -- the same 3x3
//     matrix already accumulated for the (disabled) mid-tier degeneracy
//     guard, reused here at negligible extra cost.
//
// This header is pure math: scalars in, scalars out, no estimator-specific
// types, no I/O, no estimator state mutation. Deliberately standalone so it
// can be unit-tested without pulling in the rest of the estimator (mirrors
// the house style of rdef_ci_diagnostic.h).

#include <algorithm>
#include <cmath>

namespace mf_lio {
namespace adaptive_window {

inline double clamp01(double x) {
    return std::clamp(x, 0.0, 1.0);
}

// Fit-quality term. r is the low tier's own just-completed iEKF point-to-
// plane residual RMSE. r <= r_good -> 1.0; r >= r_bad -> 0.0; linear between.
inline double q_rmse_term(double r, double r_good, double r_bad) {
    const double denom = r_bad - r_good;
    if (denom <= 1e-9) return 1.0;
    return clamp01((r_bad - r) / denom);
}

// Rotation-rate gate. omega_bar is the EMA-smoothed gyroscope magnitude
// (rad/s). omega_bar <= omega_lo -> 0.0; omega_bar >= omega_hi -> 1.0;
// linear between.
inline double q_omega_term(double omega_bar, double omega_lo, double omega_hi) {
    const double denom = omega_hi - omega_lo;
    if (denom <= 1e-9) return 0.0;
    return clamp01((omega_bar - omega_lo) / denom);
}

// Geometric-conditioning term. lambda_min_Jp is the smallest eigenvalue of
// the position information block J_p accumulated over the low tier's own
// correspondences. lambda_min_Jp >= lambda_good -> 0.0 (well-conditioned);
// lambda_min_Jp <= lambda_bad -> 1.0 (degenerate); linear between.
// Pass enabled=false to force q_geo == 0 (recovers the two-term policy),
// matching low_window_geo_conditioning_enable's default-off semantics.
inline double q_geo_term(bool enabled, double lambda_min_Jp,
                          double lambda_good, double lambda_bad) {
    if (!enabled) return 0.0;
    const double denom = lambda_good - lambda_bad;
    if (denom <= 1e-9) return 0.0;
    return clamp01((lambda_good - lambda_min_Jp) / denom);
}

// Combines the three terms via q = max(q_rmse, q_omega, q_geo) and maps it
// onto the window-size interval [W_l_min_ms, W_l_max_ms]. Optionally writes
// the composed q to *q_out for logging/testing.
inline double combine_window_ms(double q_rmse, double q_omega, double q_geo,
                                 double W_l_min_ms, double W_l_max_ms,
                                 double* q_out = nullptr) {
    const double q = std::max({q_rmse, q_omega, q_geo});
    if (q_out) *q_out = q;
    return W_l_min_ms + (W_l_max_ms - W_l_min_ms) * q;
}

} // namespace adaptive_window
} // namespace mf_lio
