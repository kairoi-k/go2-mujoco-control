#include "stage_c/model_observation.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#ifndef GO2_MODEL_PATH
#define GO2_MODEL_PATH "unitree_robots/go2/go2.xml"
#endif
using go2_control::Go2RigidBody;
using go2_control::RigidBodyPlanningKinematics;
using go2_control::RigidBodyState;
using go2_terrain::stage_c::CaptureModelPlanningObservation;
using go2_terrain::stage_c::ContactEvidence;
using go2_terrain::stage_c::ContactProvenance;
using go2_terrain::stage_c::Frame;
using go2_terrain::stage_c::MapObservation;
using go2_terrain::stage_c::Phase1CommandAuthority;
using go2_terrain::stage_c::PlanningBudget;
using go2_terrain::stage_c::PlanningIdentity;
using go2_terrain::stage_c::PointRole;
using go2_terrain::TerrainSource;
using go2_terrain::stage_c::TimedPoint;
using go2_terrain::stage_c::TimeNs;
using go2_terrain::stage_c::JointPlannerFailure;
namespace
{
void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
RigidBodyState TiltedMovingState()
{
    RigidBodyState state;
    state.position_world = Eigen::Vector3d(.4, -.1, .43);
    state.quat_world_from_body =
        Eigen::AngleAxisd(.30, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(.22, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-.16, Eigen::Vector3d::UnitX());
    state.q << .05, .70, -1.40, -.05, .65, -1.35,
        .03, .68, -1.37, -.04, .72, -1.42;
    state.dq.setZero();
    state.dq[1] = .65;
    state.dq[2] = -.25;
    state.linear_vel_world.setZero();
    state.angular_vel_body.setZero();
    return state;
}
PlanningIdentity Identity()
{
    return {19, TimeNs::FromSeconds(1.25), 7, 3, 0};
}
MapObservation Map()
{
    MapObservation map;
    map.metadata_valid = true;
    map.frame_id = "world";
    map.source = TerrainSource::kTestFixture;
    map.epoch = 7;
    map.acquisition_time = TimeNs::FromSeconds(1.24);
    map.resolution_m = .02;
    map.width = map.height = map.total_cells = map.known_cells = 1;
    return map;
}
Phase1CommandAuthority Command()
{
    Phase1CommandAuthority command;
    command.command_epoch = 5;
    command.shaped_vx_mps = 1.0;
    command.applied_vx_mps = 1.0;
    command.period_s = .24;
    command.duty_factor = .58;
    command.valid = true;
    return command;
}
PlanningBudget Budget()
{
    PlanningBudget budget;
    budget.max_candidate_combinations = 4;
    budget.max_solver_iterations = 8;
    budget.prediction_start = TimeNs::FromSeconds(1.25);
    budget.prediction_end = TimeNs::FromSeconds(1.50);
    budget.max_source_age_ns = 100000000;
    budget.deadline_us = 5000.0;
    return budget;
}
TimedPoint SurfacePoint(
    const Eigen::Vector3d &center, const TimeNs time)
{
    // Deliberately use a supplied terrain point. It is not copied from the
    // model site or collision center by CaptureModelPlanningObservation.
    return {{center.x() + .013, center.y() - .017, center.z() - .071},
            Frame::kWorld, time, true, PointRole::kSurfaceContactPoint};
}
ContactEvidence MeasuredContacts(const TimeNs time)
{
    ContactEvidence contact;
    contact.mask = {true, false, true, false};
    contact.provenance = ContactProvenance::kMeasured;
    contact.source_time = time;
    contact.valid = true;
    return contact;
}
std::array<TimedPoint, go2::kLegCount> SurfacePoints(
    const RigidBodyPlanningKinematics &model, const TimeNs time)
{
    std::array<TimedPoint, go2::kLegCount> points{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        points[leg] = SurfacePoint(model.dynamics.foot_pos_world[leg], time);
    return points;
}
double VecDistance(const go2::Vec3 &left, const go2::Vec3 &right)
{
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
} // namespace
int main()
{
    try
    {
        Go2RigidBody robot;
        Check(robot.Load(GO2_MODEL_PATH), "model load");
        const RigidBodyState state = TiltedMovingState();
        RigidBodyPlanningKinematics model;
        Check(robot.EvaluatePlanningKinematics(state, model),
              "planning kinematics");
        Check(model.com_velocity_world.allFinite(), "COM velocity finite");
        Check(model.dynamics.foot_geometry_valid[0], "foot metadata");
        Check(model.com_velocity_world.norm() > 1.0e-8,
              "moving leg must change COM velocity");
        const TimeNs sample_time = Identity().source_state_time;
        const auto measured = MeasuredContacts(sample_time);
        const auto surfaces = SurfacePoints(model, sample_time);
        const auto captured = CaptureModelPlanningObservation(
            robot, state, Identity(), measured, surfaces, Map(), Command(),
            Budget(), model);
        Check(captured.ok, "model observation capture");
        Check(captured.failure == JointPlannerFailure::kNone,
              "capture failure");
        // COM velocity comes from the articulated model; copying the base
        // velocity would incorrectly report zero for this moving-leg state.
        const auto &body = captured.input.body;
        Check(VecDistance(body.base_velocity_world, {0.0, 0.0, 0.0}) < 1.0e-12,
              "base velocity oracle");
        Check(VecDistance(body.com_velocity_world, body.base_velocity_world) > 1.0e-8,
              "COM velocity must differ from base velocity");
        Check(body.model_com_world.role == PointRole::kCenterOfMass,
              "COM role");
        Check(body.model_com_world.frame == Frame::kWorld,
              "COM frame");
        Check(body.model_com_world.source_time == sample_time,
              "COM source time");
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &foot = captured.input.feet[leg];
            Check(foot.foot_site_world.role == PointRole::kFootSite,
                  "foot site role");
            Check(foot.foot_collision_center_world.role ==
                      PointRole::kFootCollisionCenter,
                  "collision center role");
            Check(foot.foot_site_world.source_time == sample_time &&
                      foot.foot_collision_center_world.source_time == sample_time,
                  "foot model source times");
            Check(VecDistance(foot.foot_site_world.value,
                              {model.dynamics.foot_site_world[leg].x(),
                               model.dynamics.foot_site_world[leg].y(),
                               model.dynamics.foot_site_world[leg].z()}) < 1.0e-12,
                  "site observation");
            Check(VecDistance(foot.foot_collision_center_world.value,
                              {model.dynamics.foot_pos_world[leg].x(),
                               model.dynamics.foot_pos_world[leg].y(),
                               model.dynamics.foot_pos_world[leg].z()}) < 1.0e-12,
                  "collision center observation");
            Check(VecDistance(foot.foot_site_world.value,
                              foot.foot_collision_center_world.value) > 1.0e-6,
                  "site and collision center must remain distinct");
            if (measured.mask[leg])
            {
                Check(foot.measured_support_anchor_valid,
                      "measured surface anchor valid");
                Check(foot.measured_support_anchor_world.role ==
                          PointRole::kSurfaceContactPoint,
                      "surface point role");
                Check(foot.contact_patch_world.role ==
                          PointRole::kSurfaceContactPoint,
                      "contact patch role");
                Check(VecDistance(foot.contact_patch_world.value,
                                  surfaces[leg].value) < 1.0e-12,
                      "supplied surface point retained");
                Check(VecDistance(foot.contact_patch_world.value,
                                  foot.foot_collision_center_world.value) > 1.0e-4,
                      "surface point distinct from collision center");
            }
            else
            {
                // No measured contact means no invented terrain anchor.
                Check(!foot.measured_support_anchor_valid &&
                          !foot.contact_patch_world.valid,
                      "unmeasured anchor remains unknown");
            }
        }
        auto planned = measured;
        planned.provenance = ContactProvenance::kPlanned;
        const auto planned_result = CaptureModelPlanningObservation(
            robot, state, Identity(), planned, surfaces, Map(), Command(),
            Budget(), model);
        Check(!planned_result.ok &&
                  planned_result.failure == JointPlannerFailure::kObservationUnavailable,
              "planned provenance rejected as measured");
        auto applied = measured;
        applied.provenance = ContactProvenance::kApplied;
        const auto applied_result = CaptureModelPlanningObservation(
            robot, state, Identity(), applied, surfaces, Map(), Command(),
            Budget(), model);
        Check(!applied_result.ok &&
                  applied_result.failure == JointPlannerFailure::kObservationUnavailable,
              "applied provenance rejected as measured");
        auto stale_contact = measured;
        stale_contact.source_time = TimeNs::FromSeconds(1.24);
        const auto stale_contact_result = CaptureModelPlanningObservation(
            robot, state, Identity(), stale_contact, surfaces, Map(), Command(),
            Budget(), model);
        Check(!stale_contact_result.ok &&
                  stale_contact_result.failure == JointPlannerFailure::kObservationUnavailable,
              "stale contact timestamp rejected");
        auto stale_surface = surfaces;
        stale_surface[0].source_time = TimeNs::FromSeconds(1.24);
        const auto stale_surface_result = CaptureModelPlanningObservation(
            robot, state, Identity(), measured, stale_surface, Map(), Command(),
            Budget(), model);
        Check(!stale_surface_result.ok &&
                  stale_surface_result.missing_anchor_count == 1,
              "stale surface timestamp rejected");
        std::array<TimedPoint, go2::kLegCount> absent_surfaces{};
        const auto absent_anchor_result = CaptureModelPlanningObservation(
            robot, state, Identity(), measured, absent_surfaces, Map(), Command(),
            Budget(), model);
        Check(!absent_anchor_result.ok &&
                  absent_anchor_result.missing_anchor_count == 2,
              "missing measured anchors rejected without fallback");
        std::cout << "model observation passed: com_base_speed_delta="
                  << VecDistance(body.com_velocity_world, body.base_velocity_world)
                  << " missing_anchors=" << absent_anchor_result.missing_anchor_count
                  << "\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
