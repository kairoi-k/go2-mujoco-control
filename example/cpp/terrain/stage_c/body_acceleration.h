#pragma once
#include "body_reconstruction.h"
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
namespace go2_terrain
{
namespace stage_c
{
// This is an acceleration lift only. It maps kinematic/centroidal
// acceleration targets into generalized qdd; it is not a force, torque,
// contact, collision, or actuator certificate.
struct BodyAccelerationTarget
{
    bool com_acceleration_valid = false;
    Eigen::Vector3d com_acceleration_world = Eigen::Vector3d::Zero();
    bool angular_momentum_derivative_valid = false;
    Eigen::Vector3d angular_momentum_derivative_world = Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d, go2::kLegCount> foot_acceleration_world{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    // Every foot row is required by the square 18-row lift. There is no
    // implicit zero target for an invalid or omitted foot acceleration.
    std::array<bool, go2::kLegCount> foot_acceleration_valid{};
};
struct BodyAccelerationLiftOptions
{
    // This is a configuration integration interval used only for central
    // differentiation of the actual model Jacobians.
    double configuration_epsilon_s = 1.0e-6;
    double minimum_map_rcond = 1.0e-10;
    double residual_tolerance = 1.0e-8;
};
struct BodyAccelerationLift
{
    Eigen::Matrix<double, go2_control::kGo2Nv, 1> qacc =
        Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
    Eigen::Matrix<double, go2_control::kGo2Nv, go2_control::kGo2Nv> map =
        Eigen::Matrix<double, go2_control::kGo2Nv, go2_control::kGo2Nv>::Zero();
    Eigen::Matrix<double, go2_control::kGo2Nv, go2_control::kGo2Nv> map_dot =
        Eigen::Matrix<double, go2_control::kGo2Nv, go2_control::kGo2Nv>::Zero();
    Eigen::Matrix<double, go2_control::kGo2Nv, 1> rhs =
        Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
    JointPlannerFailure failure = JointPlannerFailure::kNumericalFailure;
    int map_rank = 0;
    double map_rcond = 0.0;
    double residual_inf = std::numeric_limits<double>::infinity();
    double configuration_epsilon_s = 0.0;
    bool valid = false;
};
inline bool FiniteBodyAccelerationTarget(const BodyAccelerationTarget &target)
{
    if (!target.com_acceleration_valid ||
        !target.angular_momentum_derivative_valid ||
        !target.com_acceleration_world.allFinite() ||
        !target.angular_momentum_derivative_world.allFinite())
        return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (!target.foot_acceleration_valid[leg] ||
            !target.foot_acceleration_world[leg].allFinite())
            return false;
    return true;
}
namespace detail
{
using Matrix18 = Eigen::Matrix<double, go2_control::kGo2Nv, go2_control::kGo2Nv>;
using Vector18 = Eigen::Matrix<double, go2_control::kGo2Nv, 1>;
inline bool ValidPlanningModel(
    const go2_control::RigidBodyPlanningKinematics &model)
{
    if (!model.valid || !model.dynamics.valid ||
        !model.com_jacobian_world.allFinite() ||
        !model.angular_momentum_matrix_world.allFinite() ||
        !model.dynamics.qvel.allFinite())
        return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (!model.dynamics.foot_geometry_valid[leg] ||
            !model.dynamics.foot_jac_world[leg].allFinite())
            return false;
    return true;
}
inline Matrix18 ConstraintMap(
    const go2_control::RigidBodyPlanningKinematics &model)
{
    Matrix18 map = Matrix18::Zero();
    map.topRows<3>() = model.com_jacobian_world;
    map.middleRows<3>(3) = model.angular_momentum_matrix_world;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        map.block<3, go2_control::kGo2Nv>(6 + 3 * leg, 0) =
            model.dynamics.foot_jac_world[leg];
    return map;
}
inline Vector18 ConstraintTarget(const BodyAccelerationTarget &target)
{
    Vector18 value = Vector18::Zero();
    value.head<3>() = target.com_acceleration_world;
    value.segment<3>(3) = target.angular_momentum_derivative_world;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        value.segment<3>(6 + 3 * leg) = target.foot_acceleration_world[leg];
    return value;
}
inline bool ValidOptions(const BodyAccelerationLiftOptions &options)
{
    return std::isfinite(options.configuration_epsilon_s) &&
        options.configuration_epsilon_s > 0.0 &&
        std::isfinite(options.minimum_map_rcond) &&
        options.minimum_map_rcond > 0.0 && options.minimum_map_rcond <= 1.0 &&
        std::isfinite(options.residual_tolerance) &&
        options.residual_tolerance > 0.0;
}
} // namespace detail
inline BodyAccelerationLift LiftBodyAcceleration(
    go2_control::Go2RigidBody &robot, const BodyReconstruction &reconstruction,
    const BodyAccelerationTarget &target,
    const BodyAccelerationLiftOptions &options = {})
{
    BodyAccelerationLift out;
    out.configuration_epsilon_s = options.configuration_epsilon_s;
    if (!detail::ValidOptions(options) || !FiniteBodyAccelerationTarget(target)) {
        out.failure = JointPlannerFailure::kInvalidInput;
        return out;
    }
    if (!reconstruction.kinematics_valid ||
        !reconstruction.state.position_world.allFinite() ||
        !reconstruction.state.quat_world_from_body.coeffs().allFinite() ||
        !reconstruction.state.linear_vel_world.allFinite() ||
        !reconstruction.state.angular_vel_body.allFinite() ||
        !reconstruction.state.q.allFinite() ||
        !reconstruction.state.dq.allFinite() ||
        !detail::ValidPlanningModel(reconstruction.model)) {
        out.failure = JointPlannerFailure::kObservationUnavailable;
        return out;
    }
    // Reevaluate the stated configuration; a caller-owned cached model must
    // not silently supply another tick's Jacobians or velocities.
    go2_control::RigidBodyPlanningKinematics model;
    if(!robot.EvaluatePlanningKinematics(reconstruction.state,model) ||
       !detail::ValidPlanningModel(model)) {
        out.failure=JointPlannerFailure::kObservationUnavailable;return out;
    }
    out.map = detail::ConstraintMap(model);
    out.rhs = detail::ConstraintTarget(target);
    if (!out.map.allFinite() || !out.rhs.allFinite()) {
        out.failure = JointPlannerFailure::kObservationUnavailable;
        return out;
    }
    // qdot is evaluated at the reconstructed state. Integrating this same
    // generalized velocity in both directions keeps map_dot a central
    // configuration derivative of the actual MuJoCo model.
    go2_control::RigidBodyState plus_state, minus_state;
    if (!robot.IntegrateConfiguration(
            reconstruction.state, model.dynamics.qvel,
            options.configuration_epsilon_s, plus_state) ||
        !robot.IntegrateConfiguration(
            reconstruction.state, model.dynamics.qvel,
            -options.configuration_epsilon_s, minus_state)) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    go2_control::RigidBodyPlanningKinematics plus_model, minus_model;
    if (!robot.EvaluatePlanningKinematics(plus_state, plus_model) ||
        !robot.EvaluatePlanningKinematics(minus_state, minus_model) ||
        !detail::ValidPlanningModel(plus_model) ||
        !detail::ValidPlanningModel(minus_model)) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    out.map_dot = (detail::ConstraintMap(plus_model) -
                   detail::ConstraintMap(minus_model)) /
        (2.0 * options.configuration_epsilon_s);
    if (!out.map_dot.allFinite()) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    out.rhs -= out.map_dot * model.dynamics.qvel;
    if (!out.rhs.allFinite()) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    const Eigen::FullPivLU<detail::Matrix18> factor(out.map);
    out.map_rank = factor.rank();
    out.map_rcond = factor.rcond();
    if (out.map_rank != go2_control::kGo2Nv ||
        !std::isfinite(out.map_rcond) ||
        out.map_rcond < options.minimum_map_rcond) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    out.qacc = factor.solve(out.rhs);
    if (!out.qacc.allFinite()) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    out.residual_inf = (out.map * out.qacc - out.rhs).lpNorm<Eigen::Infinity>();
    if (!std::isfinite(out.residual_inf) ||
        out.residual_inf > options.residual_tolerance) {
        out.failure = JointPlannerFailure::kNumericalFailure;
        return out;
    }
    out.failure = JointPlannerFailure::kNone;
    out.valid = true;
    return out;
}
} // namespace stage_c
} // namespace go2_terrain
