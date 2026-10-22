#pragma once
// Bridges v51's IKFoM state_ikfom (23-dim error state) with the ported InEKF's
// InEKFState (24-dim error state) -- both mean and covariance. This is the one
// genuinely new derivation in this port (everything else is either a verbatim
// port from mf_lio_inekf or a reindex of values v51 already computes).
//
// Column layouts (both confirmed against the real, production code):
//   IKFoM error-state (23): [pos(0-2)|rot(3-5)|offR(6-8)|offT(9-11)|vel(12-14)
//                            |bg(15-17)|ba(18-20)|grav(21-22, S2 TANGENT 2-dim)]
//     (confirmed from use-ikfom.hpp's df_dx block structure)
//   InEKF error-state (24): [phi(0-2)|dv_b(3-5)|dp_b(6-8)|dbg(9-11)|dba(12-14)
//                            |dRL(15-17)|dTL(18-20)|dgrav(21-23, ambient 3-dim)]
//
// Both conventions use body-frame/right-perturbation tangent for rot/offR
// (direct copy, no transform) and plain-additive for bg/ba/offT (direct copy).
// pos/vel differ: IKFoM's error is WORLD-frame additive, InEKF's is BODY-frame
// (dp_world = R*dp_b) -- so those two blocks need an R (or R^T) rotation.
// grav differs in representation entirely: IKFoM's S2 type tracks a proper
// 2-dim tangent-space coordinate (with its own basis matrix, S2_Mx, already
// used by v51's own df_dx); InEKF's ported code uses a naive 3-dim ambient
// tangent-projection. T_expand's grav block embeds IKFoM's 2-dim tangent into
// InEKF's 3-dim ambient space via S2_Mx -- reusing IKFoM's OWN implementation
// of that basis (not reimplementing MTK's S2 math by hand, which would be a
// second, needless source of transcription risk).

#include <Eigen/Dense>
#include "mf_lio/riekf/inekf_filter.hpp"
#include "mf_lio/use-ikfom.hpp"

namespace mf_lio {
namespace riekf {

inline InEKFState state_ikfom_to_riekf(const state_ikfom& s) {
    InEKFState r;
    r.pos           = s.pos;
    r.rot           = s.rot.toRotationMatrix();
    r.vel           = s.vel;
    r.bg            = s.bg;
    r.ba            = s.ba;
    r.offset_R_L_I  = s.offset_R_L_I.toRotationMatrix();
    r.offset_T_L_I  = s.offset_T_L_I;
    r.grav          = s.grav.get_vect();
    return r;
}

inline state_ikfom riekf_to_state_ikfom(const InEKFState& r) {
    state_ikfom s;
    s.pos           = r.pos;
    s.rot           = SO3(r.rot);
    s.vel           = r.vel;
    s.bg            = r.bg;
    s.ba            = r.ba;
    s.offset_R_L_I  = SO3(r.offset_R_L_I);
    s.offset_T_L_I  = r.offset_T_L_I;
    s.grav          = S2(r.grav.x(), r.grav.y(), r.grav.z());
    return s;
}

// dxi_inekf (24) ~= T_expand * dx_ikfom (23) for a small perturbation of s.
inline Eigen::Matrix<double, 24, 23> build_T_expand(state_ikfom s) {
    Eigen::Matrix<double, 24, 23> T = Eigen::Matrix<double, 24, 23>::Zero();
    const Eigen::Matrix3d R = s.rot.toRotationMatrix();

    T.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity();   // phi   <- rot
    T.block<3, 3>(3, 12)  = R.transpose();                  // dv_b  <- vel  (world -> body)
    T.block<3, 3>(6, 0)   = R.transpose();                  // dp_b  <- pos  (world -> body)
    T.block<3, 3>(9, 15)  = Eigen::Matrix3d::Identity();   // dbg   <- bg
    T.block<3, 3>(12, 18) = Eigen::Matrix3d::Identity();   // dba   <- ba
    T.block<3, 3>(15, 6)  = Eigen::Matrix3d::Identity();   // dRL   <- offR
    T.block<3, 3>(18, 9)  = Eigen::Matrix3d::Identity();   // dTL   <- offT

    Eigen::Matrix<double, 3, 2> Mx;
    Eigen::Matrix<double, 2, 1> zero2 = Eigen::Matrix<double, 2, 1>::Zero();
    s.S2_Mx(Mx, zero2, 21);
    T.block<3, 2>(21, 21) = Mx;                             // dgrav <- grav (2-dim tangent -> 3-dim ambient)

    return T;
}

inline Eigen::Matrix<double, 24, 24> ikfom_P_to_riekf(
    const Eigen::Matrix<double, 23, 23>& P23, const state_ikfom& s) {
    const auto T = build_T_expand(s);
    return T * P23 * T.transpose();
}

inline Eigen::Matrix<double, 23, 23> riekf_P_to_ikfom(
    const Eigen::Matrix<double, 24, 24>& P24, const state_ikfom& s) {
    const auto T = build_T_expand(s);
    const Eigen::Matrix<double, 23, 24> T_reduce =
        T.completeOrthogonalDecomposition().pseudoInverse();
    return T_reduce * P24 * T_reduce.transpose();
}

}  // namespace riekf
}  // namespace mf_lio
