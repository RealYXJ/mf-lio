#pragma once
#include <algorithm>

struct PolicyInput {
    double v_linear;     // m/s — from IMU velocity estimate
    double v_angular;    // rad/s — from IMU gyroscope
    double D_degenerate; // smallest eigenvalue of J^T*J (degeneracy score)
};

class AdaptivePolicy {
public:
    // Tunable parameters (set from yaml before first call)
    double W_h_base = 20.0,  W_h_min = 10.0,  W_h_max = 50.0;   // ms
    double W_l_base = 200.0, W_l_min = 100.0, W_l_max = 500.0;  // ms
    double alpha = 0.5;   // motion sensitivity: larger → W_h shrinks faster
    double beta  = 1.0;   // degeneracy sensitivity: larger → W_l grows faster

    AdaptivePolicy() : W_h_(20.0), W_l_(200.0) {}

    // Call after setting W_*_base/min/max to sync internal state
    void reset_to_base() {
        W_h_ = std::clamp(W_h_base, W_h_min, W_h_max);
        W_l_ = std::clamp(W_l_base, W_l_min, W_l_max);
    }

    void update(const PolicyInput& in) {
        // W_h: use translational velocity only — immune to gait-induced angular oscillations
        double motion_lin = in.v_linear;
        double D = std::max(in.D_degenerate, 1e-6);  // avoid divide-by-zero

        double new_Wh = W_h_base / (1.0 + alpha * motion_lin);
        double new_Wl = W_l_base * (1.0 + beta / D);

        W_h_ = std::clamp(new_Wh, W_h_min, W_h_max);
        W_l_ = std::clamp(new_Wl, W_l_min, W_l_max);
    }

    double W_h_ms() const { return W_h_; }
    double W_l_ms() const { return W_l_; }

private:
    double W_h_;
    double W_l_;
};
