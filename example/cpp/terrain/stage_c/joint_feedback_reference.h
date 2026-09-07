#pragma once
// Shared deterministic reference construction for replay and execution.
// No plant state reader, motor writer, scheduler, or command authority.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "centroidal_joint_proposal.h"
#include "centroidal_wbc_task.h"
#include "contact_state_filter.h"
#include "foot_trajectory.h"
#include "go2_rigid_body.h"
#include "inverse_dynamics_wbc.h"
namespace go2_terrain { namespace stage_c { namespace joint_feedback_reference {
inline Eigen::Vector3d Vec(const go2::Vec3 &v)
{
    return {v.x, v.y, v.z};
}
inline go2::Vec3 Vec(const Eigen::Vector3d &v)
{
    return {v.x(), v.y(), v.z()};
}
inline Eigen::Vector3d NanVec()
{
    return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
}
inline bool FiniteState(const go2_control::RigidBodyState &state)
{
    const double qnorm = state.quat_world_from_body.coeffs().norm();
    return state.position_world.allFinite() &&
        state.quat_world_from_body.coeffs().allFinite() &&
        std::isfinite(qnorm) && qnorm > 1.0e-12 &&
        state.linear_vel_world.allFinite() &&
        state.angular_vel_body.allFinite() && state.q.allFinite() &&
        state.dq.allFinite();
}
inline const FixedScheduleInterval *FindSchedule(
    const CentroidalProblem &problem, TimeNs time)
{
    for (const auto &interval : problem.schedule)
        if (interval.start <= time && time < interval.end)
            return &interval;
    return nullptr;
}
inline bool ValidReplaySurface(
    const ContactSurface &surface, std::uint64_t epoch, TimeNs end)
{
    return surface.frame == Frame::kWorld &&
        surface.coverage == MapCoverageState::kKnown &&
        surface.map_epoch == epoch && surface.valid_until >= end &&
        surface.basis_world.allFinite() &&
        (surface.basis_world.transpose() * surface.basis_world -
            Eigen::Matrix3d::Identity()).norm() <= 1.0e-8 &&
        std::abs(surface.basis_world.determinant() - 1.0) <= 1.0e-8 &&
        std::isfinite(surface.friction_mu) && surface.friction_mu >= 0.0 &&
        std::isfinite(surface.min_normal_n) && surface.min_normal_n >= 0.0 &&
        std::isfinite(surface.max_normal_n) &&
        surface.max_normal_n >= surface.min_normal_n;
}
inline bool ResolveScheduleSurface(
    const CentroidalProblem &problem, const FixedScheduleInterval &interval,
    std::size_t leg, TimeNs end, Eigen::Vector3d &point,
    ContactSurface &surface)
{
    if (!interval.contact[leg])
    {
        point = Eigen::Vector3d::Zero();
        surface = ContactSurface{};
        return true;
    }
    const int event = interval.event_index[leg];
    if (event < 0)
    {
        if (event != -1 || !problem.request.input.feet[leg].measured_support_anchor_valid)
            return false;
        point = Vec(problem.request.input.feet[leg].measured_support_anchor_world.value);
        surface = problem.initial_surfaces[leg];
    }
    else
    {
        const std::size_t e = static_cast<std::size_t>(event);
        if (e >= problem.combination.size() ||
            e >= problem.request.candidate_sets.size())
            return false;
        const std::size_t candidate_index = problem.combination[e];
        const auto &set = problem.request.candidate_sets[e];
        if (candidate_index >= set.candidates.size())
            return false;
        point = Vec(set.candidates[candidate_index].target_world.value);
        if (!problem.candidate_surfaces.empty())
        {
            if (e >= problem.candidate_surfaces.size() ||
                candidate_index >= problem.candidate_surfaces[e].size())
                return false;
            surface = problem.candidate_surfaces[e][candidate_index];
        }
        else
        {
            if (e >= problem.event_surfaces.size())
                return false;
            surface = problem.event_surfaces[e];
        }
    }
    return point.allFinite() && ValidReplaySurface(
        surface, problem.request.input.identity.map_epoch, end);
}
inline double WrapAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
inline Eigen::Vector3d Rpy(const Eigen::Quaterniond &q)
{
    const Eigen::Matrix3d r = q.toRotationMatrix();
    return Eigen::Vector3d(
        std::atan2(r(2, 1), r(2, 2)),
        std::asin(std::clamp(-r(2, 0), -1.0, 1.0)),
        std::atan2(r(1, 0), r(0, 0)));
}
struct ClosedLoopResearchConfig
{
    // These are research replay settings, recorded in the sidecar metadata by
    // the caller. They are not B1 thresholds and do not change the planner.
    bool primary_include_orientation = true;
    double swing_clearance_m = 0.03;
    double com_kp_xy = 18.0;
    double com_kp_z = 24.0;
    double com_kd_xy = 7.0;
    double com_kd_z = 8.0;
    double momentum_kp = 3.0;
    double foot_kp = 80.0;
    double foot_kd = 8.0;
    double foot_feedback_component_limit_mps2 = 25.0;
    double orientation_kp_roll = 20.0;
    double orientation_kp_pitch = 16.0;
    double orientation_kp_yaw = 8.0;
    double orientation_kd_roll = 3.0;
    double orientation_kd_pitch = 2.5;
    double orientation_kd_yaw = 1.5;
    double orientation_acc_limit_radps2 = 8.0;
    double torque_limit_nm = 35.0;
    double force_track_weight = 1.0e-4;
    double centroidal_weight = 1.0;
    double momentum_weight = 4.0;
};
inline bool ValidClosedLoopConfig(const ClosedLoopResearchConfig &c)
{
    const std::array<double, 19> values = {
        c.swing_clearance_m, c.com_kp_xy, c.com_kp_z, c.com_kd_xy,
        c.com_kd_z, c.momentum_kp, c.foot_kp, c.foot_kd,
        c.foot_feedback_component_limit_mps2, c.orientation_kp_roll,
        c.orientation_kp_pitch, c.orientation_kp_yaw, c.orientation_kd_roll,
        c.orientation_kd_pitch, c.orientation_kd_yaw,
        c.orientation_acc_limit_radps2, c.torque_limit_nm,
        c.force_track_weight,
        c.centroidal_weight};
    for (double value : values)
        if (!std::isfinite(value) || value < 0.0)
            return false;
    return std::isfinite(c.momentum_weight) && c.momentum_weight >= 0.0;
}
inline bool ValidateProposalForReplay(
    const CentroidalJointProposal &proposal,
    const go2_control::RigidBodyState &state, double duration_s,
    std::string &failure)
{
    if (!proposal.selected_valid || !proposal.search.feasible)
    {
        failure = "proposal_not_valid";
        return false;
    }
    if (!FiniteState(state))
    {
        failure = "source_state_nonfinite";
        return false;
    }
    if (!std::isfinite(duration_s) || duration_s < 0.15 || duration_s > 0.25)
    {
        failure = "duration_out_of_diagnostic_range";
        return false;
    }
    const auto &p = proposal.selected_problem;
    const auto &r = proposal.selected_result;
    const auto &input = p.request.input;
    if (!input.basic_valid() || !input.identity.valid() ||
        p.schedule.empty() || p.grid.size() < 2 ||
        r.states.size() != p.grid.size() ||
        r.forces.size() + 1 != p.grid.size() ||
        !r.certificate.feasible || !r.certificate.input_checked ||
        !r.certificate.original_dynamics_checked)
    {
        failure = "proposal_shape_or_certificate_invalid";
        return false;
    }
    if (p.grid.front() != input.identity.source_state_time ||
        p.required_start != p.grid.front() ||
        p.grid.back().value < p.grid.front().value +
            TimeNs::FromSeconds(duration_s).value)
    {
        failure = "proposal_grid_does_not_cover_replay";
        return false;
    }
    const auto horizon = TimeNs{p.grid.front().value +
        TimeNs::FromSeconds(duration_s).value};
    if (horizon <= p.grid.front() || horizon > p.grid.back())
    {
        failure = "replay_end_outside_source_coverage";
        return false;
    }
    for (std::size_t k = 1; k < p.grid.size(); ++k)
        if (p.grid[k] <= p.grid[k - 1])
        {
            failure = "proposal_grid_nonmonotonic";
            return false;
        }
    return true;
}
inline bool CopyReplaySchedule(
    const CentroidalProblem &source, TimeNs start, TimeNs end,
    CentroidalProblem &copy, std::string &failure)
{
    copy = source;
    copy.schedule.clear();
    TimeNs cursor = start;
    for (const auto &interval : source.schedule)
    {
        if (interval.end <= start)
            continue;
        if (interval.start > cursor || interval.start < start ||
            interval.end <= interval.start || interval.start != cursor)
        {
            failure = "source_schedule_gap_or_overlap";
            return false;
        }
        if (interval.start >= end)
            break;
        if (interval.end > end)
        {
            FixedScheduleInterval clipped = interval;
            clipped.end = end;
            copy.schedule.push_back(clipped);
            cursor = end;
            break;
        }
        copy.schedule.push_back(interval);
        cursor = interval.end;
        if (cursor == end)
            break;
    }
    if (cursor != end || copy.schedule.empty())
    {
        failure = "replay_schedule_not_covered";
        return false;
    }
    copy.required_end = end;
    // Only the foot-reference schedule is restricted. The solved dynamics
    // grid and states remain immutable; the sampler supports interior times.
    return true;
}
inline bool BuildFootReplayRequest(
    const CentroidalProblem &problem, go2_control::Go2RigidBody &robot,
    const go2_control::RigidBodyState &state, TimeNs end,
    const ClosedLoopResearchConfig &config, FootTrajectoryRequest &request,
    std::array<Eigen::Vector3d, go2::kLegCount> &actual_velocity,
    std::string &failure)
{
    go2_control::RigidBodyDynamics dyn;
    if (!robot.Evaluate(state, dyn) || !dyn.valid)
    {
        failure = "controller_dynamics_unavailable";
        return false;
    }
    request = FootTrajectoryRequest{};
    request.problem = &problem;
    request.start = problem.request.input.identity.source_state_time;
    request.end = end;
    request.swing_clearance_m = config.swing_clearance_m;
    request.allow_surface_contact_tail_beyond_horizon = true;
    request.add_clearance_to_inflight_continuation = false;
    const auto *source_interval = FindSchedule(problem, request.start);
    if (source_interval == nullptr)
    {
        failure = "source_schedule_interval_missing";
        return false;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!dyn.foot_geometry_valid[leg] ||
            !dyn.foot_pos_world[leg].allFinite())
        {
            failure = "controller_foot_geometry_invalid";
            return false;
        }
        const auto &point = problem.request.input.feet[leg].foot_collision_center_world;
        if (!TimedPointValidAt(point, PointRole::kFootCollisionCenter,
                               Frame::kWorld, request.start))
        {
            failure = "source_foot_observation_invalid";
            return false;
        }
        const Eigen::Vector3d source_center = Vec(point.value);
        if ((source_center - dyn.foot_pos_world[leg]).norm() > 1.0e-6)
        {
            failure = "source_foot_observation_model_mismatch";
            return false;
        }
        const double radius = dyn.foot_geometry[leg].collision_radius_m;
        request.collision_radius_m[leg] = radius;
        request.collision_radius_valid[leg] =
            std::isfinite(radius) && radius > 0.0;
        actual_velocity[leg] = dyn.foot_jac_world[leg] * dyn.qvel;
        if (!actual_velocity[leg].allFinite())
        {
            failure = "source_foot_velocity_nonfinite";
            return false;
        }
        request.initial_velocity_world[leg] = Vec(actual_velocity[leg]);
        request.initial_velocity_valid[leg] = true;
        bool in_flight = false;
        // Swing schedule intervals intentionally carry event_index=-1. Scan
        // the authoritative event table by leg to preserve a nonzero actual
        // initial velocity when the source timestamp lies inside a swing.
        for (const auto &event : problem.request.events.events)
            if (static_cast<std::size_t>(event.id.leg) == leg &&
                event.liftoff_valid && event.liftoff_time < request.start &&
                request.start < event.touchdown_time)
                in_flight = true;
        // A moving stance foot is measured plant state and stays untouched in
        // mjData. The zero below is only a nominal stationary stance reference;
        // nonzero measured velocity is retained in actual_velocity telemetry.
        if (!in_flight)
            request.initial_velocity_world[leg] = go2::Vec3{0.0, 0.0, 0.0};
    }
    return true;
}
inline bool BuildCentroidalReference(
    const CentroidalProblem &problem, const CentroidalResult &result,
    TimeNs time, const go2_control::RigidBodyPlanningKinematics &actual,
    const ClosedLoopResearchConfig &config, Eigen::Matrix<double, 6, 1> &desired,
    Eigen::Matrix<double, 6, 1> &weights, ContactForceInterval &planned_force,
    CentroidalState &planned_state, std::string &failure)
{
    const auto sample = SampleCentroidalTrajectory(problem, result, time);
    if (!sample.state_valid || !sample.force_valid ||
        !sample.state.allFinite() || !actual.valid || !actual.dynamics.valid)
    {
        failure = "centroidal_reference_unavailable";
        return false;
    }
    planned_state = sample.state;
    planned_force = sample.force;
    if (planned_force.start > time || planned_force.end <= time)
    {
        failure = "centroidal_force_interval_mismatch";
        return false;
    }
    const auto *interval = FindSchedule(problem, time);
    if (interval == nullptr)
    {
        failure = "centroidal_schedule_missing";
        return false;
    }
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_moment = Eigen::Vector3d::Zero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d force = Vec(planned_force.force_world[leg]);
        if (!force.allFinite() || planned_force.contact[leg] != interval->contact[leg] ||
            (!planned_force.contact[leg] && force.norm() > 1.0e-9))
        {
            failure = "centroidal_force_mask_mismatch";
            return false;
        }
        if (!planned_force.contact[leg])
            continue;
        Eigen::Vector3d point;
        ContactSurface surface;
        if (!ResolveScheduleSurface(problem, *interval, leg, planned_force.end,
                                    point, surface))
        {
            failure = "centroidal_surface_unavailable";
            return false;
        }
        total_force += force;
        total_moment += (point - planned_state.head<3>()).cross(force);
    }
    if (!std::isfinite(problem.model.mass_kg) || problem.model.mass_kg <= 0.0 ||
        !std::isfinite(problem.model.gravity_mps2) ||
        !actual.dynamics.com_world.allFinite() ||
        !actual.com_velocity_world.allFinite() ||
        !actual.angular_momentum_world.allFinite())
    {
        failure = "centroidal_physical_reference_nonfinite";
        return false;
    }
    const Eigen::Vector3d com_error = planned_state.head<3>() - actual.dynamics.com_world;
    const Eigen::Vector3d velocity_error =
        planned_state.segment<3>(3) - actual.com_velocity_world;
    const Eigen::Vector3d momentum_error =
        planned_state.tail<3>() - actual.angular_momentum_world;
    Eigen::Vector3d com_acc = Eigen::Vector3d(0.0, 0.0,
        -problem.model.gravity_mps2) + total_force / problem.model.mass_kg;
    com_acc.x() += config.com_kp_xy * com_error.x() + config.com_kd_xy * velocity_error.x();
    com_acc.y() += config.com_kp_xy * com_error.y() + config.com_kd_xy * velocity_error.y();
    com_acc.z() += config.com_kp_z * com_error.z() + config.com_kd_z * velocity_error.z();
    desired.head<3>() = com_acc;
    desired.tail<3>() = total_moment + config.momentum_kp * momentum_error;
    weights << config.centroidal_weight, config.centroidal_weight,
        config.centroidal_weight, config.momentum_weight,
        config.momentum_weight, config.momentum_weight;
    return desired.allFinite() && weights.allFinite();
}

inline Eigen::Vector3d ClampComponentwise(
    const Eigen::Vector3d &value, double limit, bool &clipped)
{
    clipped = false;
    if (!value.allFinite() || !std::isfinite(limit) || limit < 0.0)
        return NanVec();
    Eigen::Vector3d out = value;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double old = out[axis];
        out[axis] = std::clamp(old, -limit, limit);
        clipped = clipped || old != out[axis];
    }
    return out;
}
inline bool BuildWbcReplayInput(
    const CentroidalProblem &problem,
    go2_control::Go2RigidBody &robot,
    const go2_control::RigidBodyState &state,
    const go2_control::RigidBodyPlanningKinematics &actual,
    const FootTrajectorySample &feet,
    const ContactForceInterval &planned_force,
    const Eigen::Matrix<double, 6, 1> &desired,
    const Eigen::Matrix<double, 6, 1> &weights,
    const FixedScheduleInterval &nominal,
    const std::array<bool, go2::kLegCount> &measured_contact,
    double initial_yaw, const ClosedLoopResearchConfig &config,
    go2_control::IdWbcParams &params, go2_control::IdWbcInput &input,
    std::array<Eigen::Vector3d, go2::kLegCount> &application_points,
    std::size_t &feedback_clipped_count, Eigen::Vector3d &orientation_acc,
    bool &orientation_clipped, std::string &failure)
{
    if (!actual.valid || !actual.dynamics.valid || !feet.valid ||
        !desired.allFinite() || !weights.allFinite() ||
        planned_force.start > feet.time || feet.time >= planned_force.end)
    {
        failure = "actual_or_reference_sample_invalid";
        return false;
    }
    if (!SetCentroidalWbcTask(robot, state, desired, weights, input))
    {
        failure = "centroidal_task_setup_failed";
        return false;
    }
    const auto *interval = FindSchedule(problem, feet.time);
    if (interval == nullptr || interval->start != nominal.start ||
        interval->end != nominal.end || interval->contact != nominal.contact ||
        interval->event_index != nominal.event_index)
    {
        failure = "wbc_schedule_interval_mismatch";
        return false;
    }
    input.contact = go2_control::MergeRunningTrotContact(
        nominal.contact, measured_contact, 0);
    input.measured_contact = measured_contact;
    input.measured_contact_valid = true;
    input.fused_contact = input.contact;
    input.fused_contact_valid = true;
    input.planned_contact = nominal.contact;
    input.planned_contact_valid = true;
    input.have_stance_acc = true;
    input.have_force_ref = true;
    input.contact_normal.fill(Eigen::Vector3d::Zero());
    input.contact_normal_valid.fill(false);
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    input.force_ref.setZero();
    feedback_clipped_count = 0;
    params = go2_control::IdWbcParams{};
    params.tau_limit_nm = config.torque_limit_nm;
    params.use_primal_active_set = true;
    params.prioritize_body_and_stance = true;
    params.primary_include_orientation = config.primary_include_orientation;
    params.w_force_track = config.force_track_weight;
    params.min_normal_n = 0.0;
    params.max_normal_n = std::numeric_limits<double>::infinity();
    params.friction_mu = std::numeric_limits<double>::infinity();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d ref_position = Vec(feet.center_world[leg].value);
        const Eigen::Vector3d ref_velocity = Vec(feet.velocity_world[leg]);
        const Eigen::Vector3d actual_position = actual.dynamics.foot_pos_world[leg];
        const Eigen::Vector3d actual_velocity =
            actual.dynamics.foot_jac_world[leg] * actual.dynamics.qvel;
        if (!feet.leg_valid[leg] ||
            !TimedPointValidForRole(feet.center_world[leg],
                PointRole::kFootCollisionCenter, Frame::kWorld) ||
            !ref_position.allFinite() ||
            !ref_velocity.allFinite() || !Vec(feet.acceleration_world[leg]).allFinite() ||
            !actual_position.allFinite() || !actual_velocity.allFinite())
        {
            failure = "foot_reference_or_actual_invalid";
            return false;
        }
        bool clipped = false;
        const Eigen::Vector3d feedback = ClampComponentwise(
            config.foot_kp * (ref_position - actual_position) +
            config.foot_kd * (ref_velocity - actual_velocity),
            config.foot_feedback_component_limit_mps2, clipped);
        if (!feedback.allFinite())
        {
            failure = "foot_feedback_nonfinite";
            return false;
        }
        feedback_clipped_count += clipped ? 1u : 0u;
        input.swing_acc_world[leg] = Vec(feet.acceleration_world[leg]) + feedback;
        input.stance_acc_world[leg] = input.swing_acc_world[leg];
        const Eigen::Vector3d force = Vec(planned_force.force_world[leg]);
        if (!force.allFinite() || planned_force.contact[leg] != nominal.contact[leg] ||
            (!planned_force.contact[leg] && force.norm() > 1.0e-9))
        {
            failure = "force_reference_mask_invalid";
            return false;
        }
        input.force_ref.segment<3>(3 * static_cast<int>(leg)) = force;
        if (!input.contact[leg])
        {
            application_points[leg] = actual_position;
            continue;
        }
        Eigen::Vector3d point;
        ContactSurface surface;
        if (!ResolveScheduleSurface(problem, nominal, leg, planned_force.end,
                                    point, surface))
        {
            failure = "wbc_surface_unavailable";
            return false;
        }
        const Eigen::Vector3d normal = surface.basis_world.col(2);
        const double radius = actual.dynamics.foot_geometry[leg].collision_radius_m;
        if (!normal.allFinite() || std::abs(normal.norm() - 1.0) > 1.0e-8 ||
            !std::isfinite(radius) || radius <= 0.0)
        {
            failure = "actual_contact_geometry_invalid";
            return false;
        }
        input.contact_normal[leg] = normal;
        input.contact_normal_valid[leg] = true;
        params.min_normal_n_by_leg[leg] = surface.min_normal_n;
        params.friction_mu = std::min(params.friction_mu, surface.friction_mu);
        params.max_normal_n = std::min(params.max_normal_n, surface.max_normal_n);
        // The map point is a reference. WBC force application follows the
        // actual sphere center and current surface normal, preserving measured
        // plant geometry when the actual foot has drifted from that reference.
        application_points[leg] = actual_position - radius * normal;
    }
    if (params.max_normal_n == std::numeric_limits<double>::infinity())
    {
        params.max_normal_n = 0.0;
        params.friction_mu = 0.0;
    }
    if (!robot.EvaluateContactJacobians(
            state, application_points, input.force_application_jac_world))
    {
        failure = "force_application_jacobian_failed";
        return false;
    }
    input.have_force_application_jacobian = true;
    const Eigen::Vector3d rpy = Rpy(state.quat_world_from_body);
    orientation_acc = Eigen::Vector3d(
        -config.orientation_kp_roll * rpy.x() -
            config.orientation_kd_roll * state.angular_vel_body.x(),
        -config.orientation_kp_pitch * rpy.y() -
            config.orientation_kd_pitch * state.angular_vel_body.y(),
        -config.orientation_kp_yaw * WrapAngle(rpy.z() - initial_yaw) -
            config.orientation_kd_yaw * state.angular_vel_body.z());
    orientation_acc = ClampComponentwise(
        orientation_acc, config.orientation_acc_limit_radps2,
        orientation_clipped);
    if (!orientation_acc.allFinite())
    {
        failure = "orientation_reference_nonfinite";
        return false;
    }
    input.desired_angular_acc_body = orientation_acc;
    // This flag is consumed only by the proposed centroidal orientation seam;
    // without that seam the underlying WBC must reject or visibly ignore it.
    input.have_centroidal_orientation_task = true;
    return true;
}

}}} // namespaces
