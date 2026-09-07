#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include "centroidal_subproblem.h"
#include "event_schedule.h"
#include "model_observation.h"
#include "terrain_candidates.h"
namespace go2_terrain
{
namespace stage_c
{
struct PrepareJointProblemResult
{
    CentroidalProblem problem{};
    JointPlannerFailure failure = JointPlannerFailure::kInvalidInput;
    std::string detail;
    bool ok = false;
};
namespace prepare_joint_problem_detail
{
inline Eigen::Vector3d Vec(const go2::Vec3 &value)
{
    return {value.x, value.y, value.z};
}
inline bool SameVec(const Eigen::Vector3d &left, const Eigen::Vector3d &right)
{
    return (left - right).norm() == 0.0;
}
inline bool SameMatrix(const Eigen::Matrix3d &left, const Eigen::Matrix3d &right)
{
    return (left - right).norm() == 0.0;
}
inline bool ValidSurface(
    const ContactSurface &surface, std::uint64_t map_epoch, TimeNs end)
{
    return surface.frame == Frame::kWorld &&
        surface.coverage == MapCoverageState::kKnown &&
        surface.map_epoch == map_epoch && surface.valid_until >= end &&
        surface.basis_world.allFinite() &&
        (surface.basis_world.transpose() * surface.basis_world -
             Eigen::Matrix3d::Identity())
                .norm() < 1.0e-10 &&
        std::abs(surface.basis_world.determinant() - 1.0) < 1.0e-10 &&
        std::isfinite(surface.friction_mu) && surface.friction_mu >= 0.0 &&
        std::isfinite(surface.min_normal_n) && surface.min_normal_n >= 0.0 &&
        std::isfinite(surface.max_normal_n) &&
        surface.max_normal_n >= surface.min_normal_n;
}
inline bool StrictGrid(const std::vector<TimeNs> &grid)
{
    if (grid.size() < 2 || grid.size() > 49 || grid.front().value < 0)
        return false;
    for (std::size_t i = 1; i < grid.size(); ++i)
        if (grid[i] <= grid[i - 1])
            return false;
    return true;
}
inline bool StrictSchedule(
    const FixedSchedulePreview &preview, const std::vector<TimeNs> &grid)
{
    if (!preview.complete || preview.intervals.size() != grid.size() - 1 ||
        preview.intervals.size() > 48)
        return false;
    for (std::size_t i = 0; i < preview.intervals.size(); ++i)
    {
        const auto &interval = preview.intervals[i];
        if (interval.start != grid[i] || interval.end != grid[i + 1] ||
            interval.start >= interval.end)
            return false;
        for (int event : interval.event_index)
            if (event < -1)
                return false;
    }
    return true;
}
inline bool ValidPreviewEvents(
    const FixedSchedulePreview &preview, const TerrainPlanningInput &input,
    const std::vector<TimeNs> &grid)
{
    if (preview.events.events.size() > kStageCMaxEvents)
        return false;
    if (preview.events.events.empty())
        return true;
    // The preview owns timing and IDs. Its future targets intentionally remain
    // unset until a candidate combination is bound; committed events must
    // already carry their old measured/planned target.
    if (!preview.events.valid(false))
        return false;
    for (std::size_t i = 0; i < preview.events.events.size(); ++i)
    {
        const auto &event = preview.events.events[i];
        if (event.id.schedule_epoch != input.identity.schedule_epoch ||
            event.touchdown_time < grid.front() ||
            event.touchdown_time >= grid.back() ||
            event.contact_interval_end <= event.touchdown_time ||
            !event.liftoff_valid || event.liftoff_time.value < 0 ||
            event.liftoff_time >= event.touchdown_time ||
            !std::binary_search(grid.begin(), grid.end(),
                                event.touchdown_time))
            return false;
        const TimeNs covered_end = std::min(event.contact_interval_end,
                                             grid.back());
        if (!std::binary_search(grid.begin(), grid.end(), covered_end))
            return false;
        if (i > 0 && preview.events.events[i - 1].touchdown_time >
                          event.touchdown_time)
            return false;
    }
    return true;
}
inline JointPlannerFailure ValidateScheduleSemantics(
    const FixedSchedulePreview &preview, const TerrainPlanningInput &input,
    const std::array<ContactSurface, go2::kLegCount> &initial_surfaces,
    std::string &detail)
{
    using F = JointPlannerFailure;
    const auto &grid = preview.grid;
    const auto &events = preview.events.events;
    std::array<bool, go2::kLegCount> anchor_ended{};
    std::array<bool, go2::kLegCount> previous_contact{};
    std::array<int, go2::kLegCount> previous_event{{-1, -1, -1, -1}};
    for (std::size_t k = 0; k < preview.intervals.size(); ++k)
    {
        const auto &interval = preview.intervals[k];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int event_index = interval.event_index[leg];
            if (!interval.contact[leg])
            {
                if (event_index != -1)
                {
                    detail = "swing_interval_has_event_reference";
                    return F::kInvalidInput;
                }
                anchor_ended[leg] = true;
                previous_contact[leg] = false;
                previous_event[leg] = -1;
                continue;
            }
            if (event_index < 0)
            {
                if (event_index != -1 || anchor_ended[leg] ||
                    !input.measured_contact.mask[leg] ||
                    !input.feet[leg].measured_support_anchor_valid ||
                    !TimedPointValidAt(
                        input.feet[leg].measured_support_anchor_world,
                        PointRole::kSurfaceContactPoint, Frame::kWorld,
                        input.identity.source_state_time))
                {
                    detail = "initial_contact_anchor_unavailable";
                    return F::kObservationUnavailable;
                }
                if (!ValidSurface(initial_surfaces[leg], input.identity.map_epoch,
                                  interval.end))
                {
                    detail = "initial_contact_surface_unknown_or_expired";
                    return F::kCoverageIncomplete;
                }
                previous_contact[leg] = true;
                previous_event[leg] = -1;
                continue;
            }
            if (previous_contact[leg] && previous_event[leg] != event_index)
            {
                detail = "contact_event_changed_without_swing";
                return F::kInvalidInput;
            }
            if (event_index >= static_cast<int>(events.size()))
            {
                detail = "schedule_event_index_out_of_range";
                return F::kInvalidInput;
            }
            const auto &event = events[static_cast<std::size_t>(event_index)];
            if (static_cast<std::size_t>(event.id.leg) != leg ||
                interval.start < event.touchdown_time ||
                interval.end > event.contact_interval_end)
            {
                detail = "schedule_event_time_or_leg_conflict";
                return F::kInvalidInput;
            }
            previous_contact[leg] = true;
            previous_event[leg] = event_index;
        }
    }
    for (std::size_t event_index = 0; event_index < events.size();
         ++event_index)
    {
        const auto &event = events[event_index];
        const TimeNs covered_end = std::min(event.contact_interval_end,
                                             grid.back());
        bool covered = false;
        for (const auto &interval : preview.intervals)
        {
            if (interval.start < covered_end && interval.end >
                                                    event.touchdown_time)
            {
                const std::size_t leg = static_cast<std::size_t>(event.id.leg);
                if (!interval.contact[leg] ||
                    interval.event_index[leg] != static_cast<int>(event_index))
                {
                    detail = "event_not_preserved_by_schedule";
                    return F::kInvalidInput;
                }
                covered = true;
            }
        }
        if (!covered)
        {
            detail = "event_has_no_scheduled_contact_interval";
            return F::kCoverageIncomplete;
        }
    }
    return F::kNone;
}
inline JointPlannerFailure ValidateCandidateMatches(
    const TerrainCandidateGenerationResult &generated,
    const FixedSchedulePreview &preview, const TerrainPlanningInput &input,
    const std::vector<TimeNs> &grid, std::vector<std::vector<ContactSurface>> &surfaces,
    std::string &detail)
{
    using F = JointPlannerFailure;
    const auto &events = preview.events.events;
    if (!generated.valid || generated.failure != F::kNone)
    {
        detail = "candidate_generation_not_valid";
        return generated.failure == F::kNone ? F::kObservationUnavailable
                                             : generated.failure;
    }
    if (generated.sets.size() != events.size())
    {
        detail = "candidate_event_count_mismatch";
        return F::kInvalidInput;
    }
    surfaces.clear();
    surfaces.resize(events.size());
    for (std::size_t event_index = 0; event_index < events.size();
         ++event_index)
    {
        const auto &generated_set = generated.sets[event_index];
        if (!generated_set.complete || !generated_set.event_set.complete)
        {
            detail = "candidate_generation_incomplete";
            return F::kSearchIncomplete;
        }
        if (!(generated_set.event_set.event_id == events[event_index].id))
        {
            detail = "candidate_event_identity_mismatch";
            return F::kInvalidInput;
        }
        if (generated_set.event_set.candidates.empty() ||
            generated_set.event_set.candidates.size() !=
                generated_set.matched_surfaces.size())
        {
            detail = "candidate_surface_shape_mismatch";
            return F::kInvalidInput;
        }
        surfaces[event_index].reserve(generated_set.matched_surfaces.size());
        for (std::size_t candidate_index = 0;
             candidate_index < generated_set.event_set.candidates.size();
             ++candidate_index)
        {
            const auto &candidate =
                generated_set.event_set.candidates[candidate_index];
            const auto &match = generated_set.matched_surfaces[candidate_index];
            if (!candidate.geometry_hard_feasible)
            {
                detail = "candidate_geometry_rejected";
                return F::kNoFeasibleCandidateInSet;
            }
            if (!candidate.valid() || candidate.candidate_id == 0)
            {
                detail = "candidate_malformed";
                return F::kInvalidInput;
            }
            if (candidate.coverage != MapCoverageState::kKnown)
            {
                detail = "candidate_coverage_unknown";
                return F::kCoverageIncomplete;
            }
            if (!TimedPointValidForRole(
                    candidate.target_world, PointRole::kSurfaceContactPoint,
                    Frame::kWorld) ||
                candidate.target_world.source_time > input.identity.source_state_time)
            {
                detail = "candidate_provenance_invalid";
                return F::kInvalidInput;
            }
            if (!match.surface_world.valid ||
                match.contact_surface.coverage != MapCoverageState::kKnown)
            {
                detail = "candidate_surface_coverage_unknown";
                return F::kCoverageIncomplete;
            }
            if (!match.valid())
            {
                detail = "candidate_surface_match_invalid";
                return F::kInvalidInput;
            }
            if (!SameTimedPoint(candidate.target_world, match.surface_world))
            {
                detail = "candidate_surface_target_mismatch";
                return F::kInvalidInput;
            }
            if (match.surface_world.source_time > input.identity.source_state_time ||
                !ValidSurface(match.contact_surface, input.identity.map_epoch,
                              grid.back()))
            {
                detail = "candidate_surface_epoch_or_horizon_conflict";
                return F::kCoverageIncomplete;
            }
            surfaces[event_index].push_back(match.contact_surface);
        }
    }
    return F::kNone;
}
inline JointPlannerFailure ValidateModel(
    const TerrainPlanningInput &input,
    const go2_control::RigidBodyPlanningKinematics &actual_model,
    const go2_control::SrbdMpcParams &model, std::string &detail)
{
    using F = JointPlannerFailure;
    if (!actual_model.valid || !actual_model.dynamics.valid ||
        !actual_model.angular_momentum_world.allFinite() ||
        !actual_model.dynamics.com_world.allFinite() ||
        !actual_model.com_velocity_world.allFinite() ||
        !actual_model.dynamics.inertia_com_world.allFinite() ||
        !std::isfinite(actual_model.dynamics.mass_kg) ||
        actual_model.dynamics.mass_kg <= 0.0)
    {
        detail = "actual_model_kinematics_unavailable";
        return F::kObservationUnavailable;
    }
    const Eigen::Vector3d input_com_world = Vec(input.body.model_com_world.value);
    const Eigen::Vector3d input_com_velocity = Vec(input.body.com_velocity_world);
    if (!input.body.model_com_valid ||
        !TimedPointValidAt(input.body.model_com_world,
                           PointRole::kCenterOfMass, Frame::kWorld,
                           input.identity.source_state_time) ||
        !input_com_world.allFinite() || !input_com_velocity.allFinite())
    {
        detail = "normalized_input_com_unavailable";
        return F::kObservationUnavailable;
    }
    if (!std::isfinite(input.body.mass_kg) ||
        input.body.mass_kg != actual_model.dynamics.mass_kg ||
        !std::isfinite(model.mass_kg) || model.mass_kg != input.body.mass_kg ||
        !model.inertia_com_world.allFinite() ||
        !SameMatrix(model.inertia_com_world,
                    actual_model.dynamics.inertia_com_world))
    {
        detail = "production_model_actual_model_conflict";
        return F::kInvalidInput;
    }
    constexpr double kCaptureConsistencyTolerance = 1.0e-12;
    if ((input_com_world - actual_model.dynamics.com_world).norm() >
            kCaptureConsistencyTolerance ||
        (input_com_velocity - actual_model.com_velocity_world).norm() >
            kCaptureConsistencyTolerance)
    {
        detail = "normalized_input_actual_model_com_conflict";
        return F::kInvalidInput;
    }
    if (!std::isfinite(model.gravity_mps2) || model.gravity_mps2 <= 0.0)
    {
        detail = "production_model_gravity_invalid";
        return F::kInvalidInput;
    }
    return F::kNone;
}
} // namespace prepare_joint_problem_detail
// Assemble the solver input seam without choosing a candidate. The event table
// may contain unbound future targets; each selected candidate remains paired
// with its own surface in candidate_surfaces for the later evaluator.
inline PrepareJointProblemResult PrepareJointProblem(
    const TerrainPlanningInput &normalized_input,
    const FixedSchedulePreview &preview,
    const TerrainCandidateGenerationResult &generated,
    const std::array<ContactSurface, go2::kLegCount> &initial_surfaces,
    const go2_control::RigidBodyPlanningKinematics &actual_model,
    const go2_control::SrbdMpcParams &model,
    const std::vector<StateBox> &bounds,
    const std::vector<CentroidalState> &references = {})
{
    using namespace prepare_joint_problem_detail;
    PrepareJointProblemResult result;
    auto fail = [&](JointPlannerFailure failure, const std::string &detail) {
        result.failure = failure;
        result.detail = detail;
        result.ok = false;
        return result;
    };
    if (!normalized_input.basic_valid())
        return fail(JointPlannerFailure::kObservationUnavailable,
                    "normalized_input_not_ready");
    if (!StrictGrid(preview.grid) ||
        preview.grid.front() != normalized_input.identity.source_state_time ||
        normalized_input.budget.prediction_start != preview.grid.front() ||
        normalized_input.budget.prediction_end != preview.grid.back())
        return fail(JointPlannerFailure::kCoverageIncomplete,
                    "absolute_grid_not_bound_to_observation");
    if (!StrictSchedule(preview, preview.grid) ||
        !ValidPreviewEvents(preview, normalized_input, preview.grid))
        return fail(JointPlannerFailure::kCoverageIncomplete,
                    "absolute_preview_invalid");
    std::string detail;
    const auto schedule_failure = ValidateScheduleSemantics(
        preview, normalized_input, initial_surfaces, detail);
    if (schedule_failure != JointPlannerFailure::kNone)
        return fail(schedule_failure, detail);
    std::vector<std::vector<ContactSurface>> candidate_surfaces;
    const auto candidate_failure = ValidateCandidateMatches(
        generated, preview, normalized_input, preview.grid, candidate_surfaces,
        detail);
    if (candidate_failure != JointPlannerFailure::kNone)
        return fail(candidate_failure, detail);
    if (bounds.size() != preview.grid.size())
        return fail(JointPlannerFailure::kInvalidInput, "bounds_grid_shape_mismatch");
    for (const auto &box : bounds)
        if (!box.lower.allFinite() || !box.upper.allFinite() ||
            (box.lower.array() > box.upper.array()).any())
            return fail(JointPlannerFailure::kInvalidInput, "invalid_state_bounds");
    if (!references.empty())
    {
        if (references.size() != preview.grid.size())
            return fail(JointPlannerFailure::kInvalidInput,
                        "reference_grid_shape_mismatch");
        for (const auto &reference : references)
            if (!reference.allFinite())
                return fail(JointPlannerFailure::kInvalidInput,
                            "nonfinite_state_reference");
    }
    const auto model_failure =
        ValidateModel(normalized_input, actual_model, model, detail);
    if (model_failure != JointPlannerFailure::kNone)
        return fail(model_failure, detail);
    CentroidalProblem problem;
    problem.request.input = normalized_input;
    problem.request.events = preview.events;
    problem.request.candidate_sets.reserve(generated.sets.size());
    for (const auto &generated_set : generated.sets)
        problem.request.candidate_sets.push_back(generated_set.event_set);
    problem.combination.clear();
    problem.schedule_epoch = normalized_input.identity.schedule_epoch;
    problem.schedule = preview.intervals;
    problem.initial_surfaces = initial_surfaces;
    problem.event_surfaces.clear();
    problem.candidate_surfaces = std::move(candidate_surfaces);
    problem.grid = preview.grid;
    problem.required_start = problem.grid.front();
    problem.required_end = problem.grid.back();
    problem.initial_momentum_world = actual_model.angular_momentum_world;
    problem.initial_momentum_valid = true;
    problem.model = model;
    problem.bounds = bounds;
    problem.references = references;
    result.problem = std::move(problem);
    result.failure = JointPlannerFailure::kNone;
    result.detail.clear();
    result.ok = true;
    return result;
}
} // namespace stage_c
} // namespace go2_terrain
