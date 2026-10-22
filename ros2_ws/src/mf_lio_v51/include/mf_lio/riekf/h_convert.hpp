#pragma once
// Converts one point-to-plane measurement row from v51's existing per-point
// Jacobian building blocks into the InEKF's 24-col body-frame H layout.
//
// v51's h_share_model (mf_lio_estimator.cpp) already computes, per point:
//   C = R^T * n_vec            (used for v51's own offset_T_L_I column, and --
//                                since InEKF's position error is body-frame,
//                                pos_world = pos + R*delta_p_b -- this is ALSO
//                                exactly InEKF's body-frame position Jacobian:
//                                no new derivation needed, just reuse)
//   A = [p_imu]x * C            (rotation Jacobian -- IDENTICAL formula to the
//                                one already used in mf_lio_inekf's own
//                                h_share_model_inekf, since both conventions
//                                use right-perturbation for rotation)
//   B = [p_imu_L]x * offset_R_L_I^T * C   (offset_R_L_I Jacobian, unchanged)
//
// So this "conversion" is a reindex of values v51 already computes into the
// InEKF's 24-col DOF layout [phi(0:3)|dv_b(3:6)|dp_b(6:9)|dbg(9:12)|dba(12:15)
// |dRL(15:18)|dTL(18:21)|dgrav(21:24)] -- not a new Jacobian derivation.

#include <Eigen/Dense>
#include "mf_lio/riekf/inekf_filter.hpp"

namespace mf_lio {
namespace riekf {

// include_extrinsics must match v51's own allow_extrinsic_update gate exactly
// (mf_lio_estimator.cpp's h_share_model): offset_R_L_I/offset_T_L_I columns
// are populated only when extrinsic estimation is both enabled and currently
// permitted by the observability gate -- position (dp_b) is NOT extrinsic-
// related and stays populated unconditionally.
inline void riekf_convert_h_row(
    const Eigen::Vector3d& A,                    // v51's existing rotation block
    const Eigen::Vector3d& C,                     // v51's existing R^T*n (position/offT block)
    Eigen::Matrix<double, 1, INEKF_DOF>& h_row_out,
    bool include_extrinsics,
    const Eigen::Vector3d* B = nullptr)           // v51's existing offset_R_L_I block (required if include_extrinsics)
{
    h_row_out.setZero();
    h_row_out.segment<3>(0) = A.transpose();       // phi (rotation): unchanged
    h_row_out.segment<3>(6) = C.transpose();       // delta_p_b (body-frame position): always active
    if (include_extrinsics) {
        h_row_out.segment<3>(18) = C.transpose();  // delta T_L (offset_T_L_I): same C, v51-confirmed
        if (B) h_row_out.segment<3>(15) = B->transpose();  // delta R_L (offset_R_L_I): unchanged
    }
}

}  // namespace riekf
}  // namespace mf_lio
