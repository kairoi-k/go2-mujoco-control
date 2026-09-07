#include "stage_c/articulated_certificate.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#ifndef GO2_MODEL_PATH
#define GO2_MODEL_PATH "unitree_robots/go2/go2.xml"
#endif
namespace
{
using go2_control::Go2RigidBody;
using go2_control::RigidBodyPlanningKinematics;
using go2_control::RigidBodyState;
using go2_terrain::stage_c::ContactForceInterval;
using go2_terrain::stage_c::ContactSurface;
using go2_terrain::stage_c::Frame;
using go2_terrain::stage_c::MapCoverageState;
using go2_terrain::stage_c::TimeNs;
void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
RigidBodyState SymmetricStandingState()
{
    RigidBodyState state;
    state.position_world = Eigen::Vector3d(0.0, 0.0, 0.42);
    state.quat_world_from_body = Eigen::Quaterniond::Identity();
    state.q << 0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763;
    state.dq.setZero();
    return state;
}
std::array<Eigen::Vector3d, go2::kLegCount> SitePoints(
    const RigidBodyPlanningKinematics &model)
{
    std::array<Eigen::Vector3d, go2::kLegCount> points{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d &p = model.dynamics.foot_site_world[leg];
        points[leg] = p;
    }
    return points;
}
// Independent static wrench oracle. Unknowns are four vertical loads. The
// rows impose total support weight and zero moment about the actual COM; no
// articulated certificate or generalized-force residual is used here.
Eigen::Vector4d SolveVerticalSupport(
    const RigidBodyPlanningKinematics &model,
    const std::array<Eigen::Vector3d, go2::kLegCount> &points)
{
    Eigen::Matrix<double, 6, 4> wrench =
        Eigen::Matrix<double, 6, 4>::Zero();
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        wrench.block<3, 1>(0, static_cast<int>(leg)) = up;
        wrench.block<3, 1>(3, static_cast<int>(leg)) =
            (points[leg] - model.dynamics.com_world).cross(up);
    }
    Eigen::Matrix<double, 6, 1> target =
        Eigen::Matrix<double, 6, 1>::Zero();
    target[2] = model.dynamics.mass_kg * 9.81;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(
        wrench, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector4d load = svd.solve(target);
    Check(load.allFinite(), "static support oracle finite");
    Check((wrench * load - target).norm() < 1.0e-8,
          "static support wrench equilibrium");
    for (int leg = 0; leg < 4; ++leg)
        Check(load[leg] > 1.0, "standing support load positive");
    return load;
}
std::array<ContactSurface, go2::kLegCount> KnownFlatSurfaces()
{
    std::array<ContactSurface, go2::kLegCount> surfaces{};
    for (auto &surface : surfaces)
    {
        surface.basis_world = Eigen::Matrix3d::Identity();
        surface.frame = Frame::kWorld;
        surface.coverage = MapCoverageState::kKnown;
        surface.map_epoch = 1;
        surface.valid_until = TimeNs::FromSeconds(1.0);
        surface.friction_mu = 0.8;
        surface.min_normal_n = 1.0;
        surface.max_normal_n = 180.0;
    }
    return surfaces;
}
ContactForceInterval StandingForce(
    const RigidBodyPlanningKinematics &model,
    const std::array<Eigen::Vector3d, go2::kLegCount> &points)
{
    ContactForceInterval force;
    force.start = TimeNs::FromSeconds(0.0);
    force.end = TimeNs::FromSeconds(0.02);
    force.contact.fill(true);
    force.force_world.fill(go2::Vec3{});
    const Eigen::Vector4d load = SolveVerticalSupport(model, points);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        force.force_world[leg] = go2::Vec3{0.0, 0.0, load[static_cast<int>(leg)]};
    return force;
}
Eigen::Matrix<double, go2_control::kGo2Nv, 1> ZeroQacc()
{
    return Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
}
Eigen::Matrix<double, go2_control::kGo2Nv, 1> FreefallQacc()
{
    auto qacc = ZeroQacc();
    qacc[2] = -9.81;
    return qacc;
}
bool HasFailure(
    const go2_control::IdWbcPhysicalCertificate &certificate,
    go2_control::IdWbcPhysicalCertificateFailure failure)
{
    return (certificate.failure_bitmask &
            go2_control::IdWbcPhysicalCertificateFailureBit(failure)) != 0u;
}
void RunFixtures()
{
    using Failure = go2_control::IdWbcPhysicalCertificateFailure;
    Go2RigidBody robot;
    Check(robot.Load(GO2_MODEL_PATH), "model load");
    const RigidBodyState standing_state = SymmetricStandingState();
    RigidBodyPlanningKinematics standing_model;
    Check(robot.EvaluatePlanningKinematics(standing_state, standing_model),
          "standing model evaluation");
    const auto standing_points = SitePoints(standing_model);
    const auto surfaces = KnownFlatSurfaces();
    const auto standing_force = StandingForce(standing_model, standing_points);
    const auto qacc = ZeroQacc();
    const auto standing = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, standing_force, standing_points, surfaces,
        35.0, 10.0);
    Check(standing.sample_feasible && standing.dynamics.checked &&
              standing.dynamics.input_valid && standing.dynamics.valid &&
              standing.dynamics.feasible,
          "actual model static standing certificate");
    Check(standing.max_surface_force_violation_n <= 1.0e-8,
          "standing per-patch force limits");
    Check(standing.dynamics.max_dynamics_force_residual_N < 1.0e-5 &&
              standing.dynamics.max_dynamics_moment_residual_Nm < 1.0e-5,
          "standing independent wrench residual");
    auto aerial_force = standing_force;
    aerial_force.contact.fill(false);
    aerial_force.force_world.fill(go2::Vec3{});
    const auto aerial = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, FreefallQacc(), aerial_force, standing_points,
        surfaces, 35.0, 10.0);
    Check(aerial.sample_feasible && aerial.dynamics.checked &&
              aerial.dynamics.feasible &&
              aerial.dynamics.max_swing_force_violation_N == 0.0,
          "actual model aerial freefall certificate");
    Check(!aerial.dynamics.normal_defaulted_by_leg[0] &&
              !aerial.dynamics.flat_normal_default_used,
          "aerial does not invent flat contact normals");
    auto insufficient_force = standing_force;
    insufficient_force.force_world.fill(go2::Vec3{});
    const auto insufficient = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, insufficient_force, standing_points,
        surfaces, 35.0, 10.0);
    Check(!insufficient.sample_feasible && insufficient.dynamics.checked &&
              insufficient.dynamics.valid && !insufficient.dynamics.feasible &&
              HasFailure(insufficient.dynamics, Failure::kDynamicsResidual) &&
              insufficient.dynamics.max_dynamics_force_residual_N > 100.0,
          "insufficient support is sample-infeasible");
    auto excessive_qacc = qacc;
    const int motor_dof = robot.MotorDof(0);
    Check(motor_dof >= 6 && motor_dof < go2_control::kGo2Nv,
          "motor dof for torque fixture");
    excessive_qacc[motor_dof] = 1.0e4;
    const auto excessive = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, excessive_qacc, standing_force, standing_points,
        surfaces, 35.0, 10.0);
    Check(!excessive.sample_feasible && excessive.dynamics.checked &&
              excessive.dynamics.valid &&
              HasFailure(excessive.dynamics, Failure::kTorque) &&
              excessive.dynamics.max_tau_violation_Nm > 1.0,
          "excessive joint acceleration torque rejection");
    auto wrong_center_points = standing_points;
    wrong_center_points[0].x() += 0.022;
    const auto wrong_center = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, standing_force, wrong_center_points,
        surfaces, 35.0, 10.0);
    Check(!wrong_center.sample_feasible && wrong_center.dynamics.checked &&
              wrong_center.dynamics.valid &&
              HasFailure(wrong_center.dynamics, Failure::kDynamicsResidual) &&
              wrong_center.dynamics.max_dynamics_moment_residual_Nm > 0.1,
          "tangent application-point shift exposes moment residual");
    auto unknown_surfaces = surfaces;
    unknown_surfaces[0].frame = Frame::kUnknown;
    const auto unknown = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, standing_force, standing_points,
        unknown_surfaces, 35.0, 10.0);
    Check(!unknown.sample_feasible && !unknown.dynamics.checked &&
              unknown.failure == go2_terrain::stage_c::JointPlannerFailure::kInvalidInput,
          "unknown surface rejected before sample certificate");

    auto unknown_epoch_surfaces = surfaces;
    unknown_epoch_surfaces[0].map_epoch = 0;
    const auto unknown_epoch = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, standing_force, standing_points,
        unknown_epoch_surfaces, 35.0, 10.0);
    Check(!unknown_epoch.sample_feasible && !unknown_epoch.dynamics.checked &&
              unknown_epoch.failure == go2_terrain::stage_c::JointPlannerFailure::kInvalidInput,
          "active surface with unknown map epoch rejected");

    auto negative_interval = standing_force;
    negative_interval.start.value = -1;
    const auto negative_time = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, negative_interval, standing_points,
        surfaces, 35.0, 10.0);
    Check(!negative_time.sample_feasible && !negative_time.dynamics.checked &&
              negative_time.failure == go2_terrain::stage_c::JointPlannerFailure::kInvalidInput,
          "negative force interval rejected");
    auto nonfinite_force = standing_force;
    nonfinite_force.force_world[0].z =
        std::numeric_limits<double>::quiet_NaN();
    const auto nonfinite = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, standing_state, qacc, nonfinite_force, standing_points, surfaces,
        35.0, 10.0);
    Check(!nonfinite.sample_feasible && !nonfinite.dynamics.checked &&
              nonfinite.failure == go2_terrain::stage_c::JointPlannerFailure::kInvalidInput,
          "nonfinite force rejected before sample certificate");
    // A joint-limit rejection is intentionally kept separate from the global
    // physical certificate: recompute its actual model and static support so
    // dynamics remains feasible while the sample bound fails.
    auto joint_limit_state = standing_state;
    joint_limit_state.q[0] = standing_model.joint_upper[0] + 0.01;
    RigidBodyPlanningKinematics joint_limit_model;
    Check(robot.EvaluatePlanningKinematics(joint_limit_state, joint_limit_model),
          "joint-limit model evaluation");
    const auto joint_limit_points = SitePoints(joint_limit_model);
    const auto joint_limit = go2_terrain::stage_c::VerifyArticulatedSample(
        robot, joint_limit_state, FreefallQacc(), aerial_force,
        joint_limit_points, surfaces, 100.0, 10.0);
    Check(!joint_limit.sample_feasible && joint_limit.dynamics.checked &&
              joint_limit.dynamics.valid && joint_limit.dynamics.feasible &&
              joint_limit.max_joint_position_violation_rad > 0.009,
          "joint-bound sample rejection stays distinct from dynamics feasibility");
    std::cout << "stage-c articulated certificate fixtures passed: mass_kg="
              << standing_model.dynamics.mass_kg
              << " static_force_N="
              << standing_force.force_world[0].z << "\n";
}
}  // namespace
int main()
{
    try
    {
        RunFixtures();
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
