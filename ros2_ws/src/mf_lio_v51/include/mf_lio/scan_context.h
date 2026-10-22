#pragma once

#include <cmath>
#include <deque>
#include <limits>
#include <tuple>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mf_lio/common_lib.h"
#include "mf_lio/use-ikfom.hpp"

static constexpr int   SC_RINGS   = 20;
static constexpr int   SC_SECTORS = 60;
static constexpr float SC_MAX_R   = 80.0f;   // max radial range for descriptor (m)

// Keyframe stored by the loop closure subsystem.
// scan_local is in sensor LOCAL frame (sensor at origin), downsampled to ~2000 pts.
struct KeyFrame {
    double              timestamp = 0.0;
    double              path_dist = 0.0;    // cumulative path length (m) at capture
    state_ikfom         pose;               // world-frame pose at capture
    PointCloudXYZI::Ptr scan_local;         // LOCAL-frame scan
    Eigen::MatrixXf     sc;                 // SC_RINGS × SC_SECTORS descriptor
    Eigen::VectorXf     ring_key;           // SC_RINGS-dim pre-filter key
};

class ScanContextManager {
public:
    std::deque<KeyFrame> keyframes_;   // protected by lc_mutex_ in the estimator

    // Build Scan Context descriptor from LOCAL-frame scan.
    // Each bin: max Z of points in that ring × sector cell.
    static Eigen::MatrixXf compute_sc(const PointCloudXYZI::Ptr& scan) {
        Eigen::MatrixXf sc = Eigen::MatrixXf::Constant(SC_RINGS, SC_SECTORS, -1e6f);
        for (const auto& pt : scan->points) {
            float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (r < 0.5f || r >= SC_MAX_R) continue;
            int ri = static_cast<int>(r / SC_MAX_R * SC_RINGS);
            if (ri >= SC_RINGS) ri = SC_RINGS - 1;
            float ang = std::atan2(pt.y, pt.x);   // [-pi, pi]
            int si = static_cast<int>((ang + static_cast<float>(M_PI))
                                      / (2.f * static_cast<float>(M_PI)) * SC_SECTORS);
            si = std::max(0, std::min(SC_SECTORS - 1, si));
            if (pt.z > sc(ri, si)) sc(ri, si) = pt.z;
        }
        // Replace "no-point" cells with 0
        for (int r = 0; r < SC_RINGS; ++r)
            for (int s = 0; s < SC_SECTORS; ++s)
                if (sc(r, s) < -1e5f) sc(r, s) = 0.f;
        return sc;
    }

    // Ring key: per-ring maximum across sectors (SC_RINGS-dim vector).
    static Eigen::VectorXf compute_ring_key(const Eigen::MatrixXf& sc) {
        return sc.rowwise().maxCoeff();
    }

    // Cosine distance between descriptor a and descriptor b shifted by `shift` columns.
    static float sc_dist(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b, int shift) {
        double dot = 0., na = 0., nb = 0.;
        for (int r = 0; r < SC_RINGS; ++r)
            for (int s = 0; s < SC_SECTORS; ++s) {
                float va = a(r, s);
                float vb = b(r, (s + shift) % SC_SECTORS);
                dot += va * vb; na += va * va; nb += vb * vb;
            }
        if (na < 1e-8 || nb < 1e-8) return 1.f;
        return static_cast<float>(1.0 - dot / std::sqrt(na * nb));
    }

    // Search for the best loop closure candidate in a metadata snapshot (lock-free).
    // meta: array of (timestamp, path_dist, sc, ring_key) for all candidates.
    // Returns (best_index_in_meta, best_sc_dist, best_rot_shift), or (-1, inf, 0).
    struct CandMeta {
        double         timestamp;
        double         path_dist;
        Eigen::MatrixXf sc;
        Eigen::VectorXf ring_key;
    };

    // Returns (best_idx, best_dist, best_shift, unconstrained_best_dist).
    // best_idx == -1 when no candidate passes all gates AND score < thresh.
    // unconstrained_best_dist is the best SC score over eligible candidates ignoring thresh.
    static std::tuple<int, float, int, float> search_meta(
        const KeyFrame& query,
        const std::vector<CandMeta>& meta,
        double min_age_s, double min_path_m, float thresh)
    {
        int   besti = -1;
        float bestd = thresh;
        int   bests = 0;
        float best_any = 1.f;  // unconstrained best for debug
        int   n_eligible = 0;
        for (int i = 0; i < static_cast<int>(meta.size()); ++i) {
            const auto& m = meta[i];
            if (query.timestamp - m.timestamp < min_age_s)  continue;
            if (query.path_dist  - m.path_dist  < min_path_m) continue;
            // Ring-key pre-filter disabled — outdoor forest is too homogeneous
            // (query.ring_key - m.ring_key).norm() check skipped intentionally
            ++n_eligible;
            for (int shift = 0; shift < SC_SECTORS; ++shift) {
                float d = sc_dist(query.sc, m.sc, shift);
                if (d < best_any) best_any = d;
                if (d < bestd) { bestd = d; besti = i; bests = shift; }
            }
        }
        if (n_eligible == 0) best_any = -1.f;  // signal: no eligible candidates
        return {besti, bestd, bests, best_any};
    }

    // Iterative 2D ICP (5 passes of brute-force NN → 2D Kabsch SVD).
    // Zero PCL allocation inside: avoids FLANN/ICP heap corruption in this PCL version.
    // T_out maps source LOCAL → target LOCAL.
    static bool verify_icp(
        const PointCloudXYZI::Ptr& src,
        const PointCloudXYZI::Ptr& tgt,
        const Eigen::Matrix4f& T_guess,
        float max_dist, float /*fitness_thresh*/,
        float max_trans_m, float max_rot_deg,
        Eigen::Matrix4f* T_out)
    {
        if (!src || !tgt || src->empty() || tgt->empty()) return false;

        // Extract to plain Eigen vectors — NO PCL allocation after this point
        constexpr size_t kCap = 300;
        auto extract = [kCap](const PointCloudXYZI::Ptr& cloud) {
            std::vector<Eigen::Vector3f> pts;
            const size_t step = cloud->size() > kCap ? cloud->size() / kCap : 1;
            pts.reserve(std::min(kCap, cloud->size()));
            for (size_t i = 0; i < cloud->size(); i += step) {
                const auto& p = cloud->points[i];
                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
                    pts.emplace_back(p.x, p.y, p.z);
            }
            return pts;
        };
        std::vector<Eigen::Vector3f> src_pts = extract(src);
        std::vector<Eigen::Vector3f> tgt_pts = extract(tgt);
        if (src_pts.empty() || tgt_pts.empty()) return false;

        const float max_dist_sq = max_dist * max_dist;
        Eigen::Matrix4f T_accum = T_guess;

        for (int iter = 0; iter < 5; ++iter) {
            // 2D brute-force NN: height ignored so z-differences don't block correspondences.
            std::vector<Eigen::Vector3f> sp, tp;
            sp.reserve(src_pts.size());
            tp.reserve(src_pts.size());

            for (const auto& p : src_pts) {
                Eigen::Vector4f q = T_accum * Eigen::Vector4f(p.x(), p.y(), p.z(), 1.f);
                float min_d = max_dist_sq;
                int best = -1;
                for (size_t j = 0; j < tgt_pts.size(); ++j) {
                    float dx = q[0] - tgt_pts[j].x();
                    float dy = q[1] - tgt_pts[j].y();
                    float d  = dx*dx + dy*dy;
                    if (d < min_d) { min_d = d; best = static_cast<int>(j); }
                }
                if (best >= 0) { sp.push_back(p); tp.push_back(tgt_pts[best]); }
            }

            const float overlap = static_cast<float>(sp.size()) / src_pts.size();
            if (iter == 0) {
                std::printf("[LC-svd] overlap=%.2f corr=%zu/%zu src=%zu tgt=%zu\n",
                            overlap, sp.size(), src_pts.size(), src_pts.size(), tgt_pts.size());
                std::fflush(stdout);
            }
            if (sp.size() < 8 || overlap < 0.25f) return false;

            // 2D Kabsch SVD: zero z → pure Rz + xy-translation.
            for (size_t i = 0; i < sp.size(); ++i) { sp[i].z() = 0.f; tp[i].z() = 0.f; }
            const float fn = static_cast<float>(sp.size());
            Eigen::Vector3f sc = Eigen::Vector3f::Zero(), tc = Eigen::Vector3f::Zero();
            for (size_t i = 0; i < sp.size(); ++i) { sc += sp[i]; tc += tp[i]; }
            sc /= fn; tc /= fn;
            Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
            for (size_t i = 0; i < sp.size(); ++i)
                H += (sp[i] - sc) * (tp[i] - tc).transpose();
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
            if (R.determinant() < 0.f) R.col(2) *= -1.f;
            Eigen::Vector3f t = tc - R * sc;
            t.z() = 0.f;

            Eigen::Matrix4f T_new = Eigen::Matrix4f::Identity();
            T_new.block<3,3>(0,0) = R;
            T_new.block<3,1>(0,3) = t;

            // Convergence check
            float delta_t = (T_new.block<3,1>(0,3) - T_accum.block<3,1>(0,3)).norm();
            T_accum = T_new;
            if (delta_t < 0.001f) break;
        }

        // Gate: total refinement from T_guess must be plausible
        Eigen::Matrix4f T_delta = T_accum * T_guess.inverse();
        float t_delta = T_delta.block<3,1>(0,3).norm();
        float cos_d = std::clamp((T_delta.block<3,3>(0,0).trace() - 1.f) * 0.5f, -1.f, 1.f);
        float r_delta = std::acos(cos_d) * (180.f / static_cast<float>(M_PI));
        std::printf("[LC-svd] icp_t=%.3fm icp_r=%.1fdeg\n", t_delta, r_delta);
        std::fflush(stdout);

        if (t_delta > max_trans_m || r_delta > max_rot_deg) return false;

        *T_out = T_accum;
        return true;
    }
};
