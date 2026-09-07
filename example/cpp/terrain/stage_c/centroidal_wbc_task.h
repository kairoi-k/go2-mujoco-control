#pragma once
#include "go2_rigid_body.h"
#include "inverse_dynamics_wbc.h"
namespace go2_terrain { namespace stage_c {
// Evaluate the articulated centroidal task at one actual state. No inverse
// kinematic projection, foot velocity reset or cached-state substitution.
inline bool SetCentroidalWbcTask(go2_control::Go2RigidBody &robot,
    const go2_control::RigidBodyState &state,
    const Eigen::Matrix<double,6,1> &desired_derivative,
    const Eigen::Matrix<double,6,1> &weights,
    go2_control::IdWbcInput &input) {
    input.have_centroidal_motion_task=true;
    input.centroidal_motion_map.setConstant(std::numeric_limits<double>::quiet_NaN());
    input.centroidal_motion_bias.setConstant(std::numeric_limits<double>::quiet_NaN());
    if(!desired_derivative.allFinite() || !weights.allFinite() ||
       (weights.array()<0).any() || !(weights.maxCoeff()>0)) return false;
    go2_control::RigidBodyPlanningKinematics model,plus,minus;
    if(!robot.EvaluatePlanningKinematics(state,model)) return false;
    constexpr double epsilon=1e-6;
    go2_control::RigidBodyState sp,sm;
    if(!robot.IntegrateConfiguration(state,model.dynamics.qvel,epsilon,sp) ||
       !robot.IntegrateConfiguration(state,model.dynamics.qvel,-epsilon,sm) ||
       !robot.EvaluatePlanningKinematics(sp,plus) ||
       !robot.EvaluatePlanningKinematics(sm,minus)) return false;
    input.dynamics=model.dynamics;
    input.centroidal_motion_map.topRows<3>()=model.com_jacobian_world;
    input.centroidal_motion_map.bottomRows<3>()=model.angular_momentum_matrix_world;
    input.centroidal_motion_bias.head<3>()=
        (plus.com_jacobian_world-minus.com_jacobian_world)*model.dynamics.qvel/(2*epsilon);
    input.centroidal_motion_bias.tail<3>()=
        (plus.angular_momentum_matrix_world-minus.angular_momentum_matrix_world)*model.dynamics.qvel/(2*epsilon);
    input.desired_centroidal_derivative=desired_derivative;
    input.centroidal_motion_weights=weights;
    return input.centroidal_motion_map.allFinite() && input.centroidal_motion_bias.allFinite();
}
}} // namespace
