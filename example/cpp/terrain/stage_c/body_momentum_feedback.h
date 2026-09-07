#pragma once
#include "body_acceleration.h"
namespace go2_terrain { namespace stage_c {
struct BodyMomentumCorrection {
    Eigen::Matrix<double,18,1> qacc_delta=Eigen::Matrix<double,18,1>::Zero();
    Eigen::Vector3d momentum_rate_delta_world=Eigen::Vector3d::Zero();
    JointPlannerFailure failure=JointPlannerFailure::kNumericalFailure;
    double residual_inf=std::numeric_limits<double>::infinity();
    bool valid=false;
};
// Instantaneous correction at the SAME actual q/dq: keep COM and all foot
// accelerations unchanged while requesting a body-frame angular-acceleration
// increment. Jdot*qvel and Adot*qvel cancel in the difference. This is not a
// future-state approximation, force/torque certificate or attitude planner.
inline BodyMomentumCorrection MapBodyAngularCorrection(
    const go2_control::RigidBodyPlanningKinematics &model,
    const Eigen::Vector3d &angular_acc_delta_body) {
    BodyMomentumCorrection out;
    if(!angular_acc_delta_body.allFinite()) {
        out.failure=JointPlannerFailure::kInvalidInput;return out;
    }
    if(!detail::ValidPlanningModel(model)) {
        out.failure=JointPlannerFailure::kObservationUnavailable;return out;
    }
    Eigen::Matrix<double,18,18> map=Eigen::Matrix<double,18,18>::Zero();
    map.topRows<3>()=model.com_jacobian_world;
    map.block<3,3>(3,3).setIdentity(); // Existing Go2 body angular DOF convention.
    for(int leg=0;leg<4;++leg)
        map.block<3,18>(6+3*leg,0)=model.dynamics.foot_jac_world[leg];
    Eigen::Matrix<double,18,1> target=Eigen::Matrix<double,18,1>::Zero();
    target.segment<3>(3)=angular_acc_delta_body;
    const Eigen::FullPivLU<Eigen::Matrix<double,18,18>> factor(map);
    if(factor.rank()!=18 || factor.rcond()<1e-10) return out;
    out.qacc_delta=factor.solve(target);
    out.momentum_rate_delta_world=model.angular_momentum_matrix_world*out.qacc_delta;
    out.residual_inf=(map*out.qacc_delta-target).lpNorm<Eigen::Infinity>();
    out.valid=out.qacc_delta.allFinite() && out.momentum_rate_delta_world.allFinite() &&
        std::isfinite(out.residual_inf) && out.residual_inf<=1e-8;
    if(out.valid) out.failure=JointPlannerFailure::kNone;
    return out;
}
}} // namespace
