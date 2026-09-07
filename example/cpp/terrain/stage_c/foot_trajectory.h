#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include "centroidal_subproblem.h"
namespace go2_terrain
{
namespace stage_c
{
// This is a deterministic reference sampler only. Its output does not prove
// swept collision avoidance, IK reachability, torque limits, or execution.
struct FootSwingContinuation
{
    // Optional terminal swing target for one leg. It is a reference-only
    // continuation and never becomes a CentroidalProblem event or commitment.
    bool valid = false;
    TouchdownEvent event{};
    StageCCandidate candidate{};
    ContactSurface surface{};
};
// Optional command-only initial boundary for a trajectory handover. It is
// reference provenance, never measured plant state, contact evidence, or an
// identity replacement. Every leg must share the same source timestamp.
struct CommandedReferenceBoundary
{
    bool enabled = false;
    std::uint64_t command_epoch = 0;
    TimeNs source_time{};
    TimeNs valid_until{};
    std::array<TimedPoint, go2::kLegCount> center_world{};
    std::array<go2::Vec3, go2::kLegCount> velocity_world{};
    std::array<bool, go2::kLegCount> valid{};
    bool valid_for(TimeNs reference_start) const
    {
        // The timestamp is the exact handover boundary. Later samples are
        // generated from this seed; no stale p/v is silently reused as a new
        // boundary.
        if (!enabled || command_epoch == 0 || source_time.value < 0 ||
            valid_until < source_time || reference_start != source_time ||
            reference_start > valid_until)
            return false;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!valid[leg] ||
                !TimedPointValidAt(center_world[leg],
                                   PointRole::kFootCollisionCenter,
                                   Frame::kWorld, source_time) ||
                !FinitePointValue(velocity_world[leg]))
                return false;
        }
        return true;
    }
};
struct FootTrajectoryRequest
{
    // CentroidalProblem is the existing owner of input, events, candidate
    // choices, candidate surfaces, and accepted commitments. No parallel
    // event/surface schema is introduced here.
    const CentroidalProblem *problem = nullptr;
    TimeNs start{};
    TimeNs end{};
    std::array<double, go2::kLegCount> collision_radius_m{};
    std::array<bool, go2::kLegCount> collision_radius_valid{};
    std::array<go2::Vec3, go2::kLegCount> initial_velocity_world{};
    std::array<bool, go2::kLegCount> initial_velocity_valid{};
    std::array<FootSwingContinuation, go2::kLegCount> continuation{};
    double swing_clearance_m = 0.0;
    // A measured-state continuation may already be descending after apex.
    // Disable a new extra bump without altering its p/v or touchdown time.
    bool add_clearance_to_inflight_continuation = true;
    // Opt-in phase-preserving continuation for a measured in-flight swing.
    // The source p/v seed is kept and the residual quartic bump is scaled by
    // the fourth power of the remaining original swing phase. This option
    // takes precedence over add_clearance_to_inflight_continuation.
    bool preserve_inflight_clearance_phase = false;
    // A feedback prefix only needs stance surface coverage through its end.
    // Future swing targets still require coverage through touchdown. Default
    // preserves the historical full contact-lifetime contract.
    bool allow_surface_contact_tail_beyond_horizon = false;
    // Optional command-only handover seed. When enabled, Prepare reads only
    // this boundary for initial center/p/v and leaves measured input untouched.
    CommandedReferenceBoundary commanded_initial{};
    // Opt-in soft-reference settling for a commanded stance velocity. This
    // changes only the generated WBC reference prefix; it never changes
    // measured input, contact schedule, touchdown targets or plant state.
    bool enable_commanded_stance_settling = false;
    double commanded_stance_settling_duration_s = 0.0;
};
struct FootTrajectorySample
{
    TimeNs time{};
    std::array<TimedPoint, go2::kLegCount> center_world{};
    std::array<go2::Vec3, go2::kLegCount> velocity_world{};
    std::array<go2::Vec3, go2::kLegCount> acceleration_world{};
    std::array<bool, go2::kLegCount> leg_valid{};
    bool valid = false;
};
struct FootTrajectoryResult
{
    JointPlannerFailure failure = JointPlannerFailure::kNone;
    std::vector<FootTrajectorySample> samples;
    bool valid = false;
};
namespace foot_trajectory_detail
{
struct PreparedEvent
{
    std::size_t event_index = 0;
    std::size_t leg = 0;
    bool continuation = false;
    TimeNs liftoff{};
    TimeNs touchdown{};
    TimeNs contact_end{};
    TimeNs interpolation_start{};
    TimeNs target_source_time{};
    // Residual clearance coefficient for a phase-preserving initial
    // in-flight continuation.
    double inflight_clearance_m = 0.0;
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d surface_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    bool starts_in_flight = false;
};
struct PreparedTrajectory
{
    bool preserve_inflight_clearance_phase = false;
    TimeNs start{};
    TimeNs end{};
    // Source provenance for generated initial references. For the default path
    // this is the measured observation time; for a handover it is the command
    // boundary timestamp.
    TimeNs initial_reference_source_time{};
    double clearance_m = 0.0;
    bool add_clearance_to_inflight_continuation = true;
    std::array<Eigen::Vector3d, go2::kLegCount> initial_center;
    std::array<Eigen::Vector3d, go2::kLegCount> initial_velocity;
    std::array<bool, go2::kLegCount> commanded_stance_settling{};
    std::array<TimeNs, go2::kLegCount> commanded_stance_settling_end{};
    std::array<std::vector<std::size_t>, go2::kLegCount> events_by_leg{};
    std::vector<PreparedEvent> events;

    PreparedTrajectory()
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            initial_center[leg].setZero();
            initial_velocity[leg].setZero();
        }
    }
};
inline Eigen::Vector3d ToEigen(const go2::Vec3 &value)
{
    return Eigen::Vector3d(value.x, value.y, value.z);
}
inline go2::Vec3 ToVec3(const Eigen::Vector3d &value)
{
    return {value.x(), value.y(), value.z()};
}
inline bool Finite(const Eigen::Vector3d &value)
{
    return value.allFinite();
}
inline bool ValidObservedPoint(
    const TimedPoint &point, PointRole role, TimeNs observation_time)
{
    return TimedPointValidForRole(point, role, Frame::kWorld) &&
        point.source_time <= observation_time;
}
inline bool ValidMeasuredPointAt(
    const TimedPoint &point, PointRole role, TimeNs observation_time)
{
    return TimedPointValidAt(point, role, Frame::kWorld, observation_time);
}
inline bool ValidSurface(
    const ContactSurface &surface, std::uint64_t map_epoch, TimeNs end_time)
{
    if (surface.frame != Frame::kWorld ||
        surface.coverage != MapCoverageState::kKnown ||
        surface.map_epoch != map_epoch || surface.valid_until < end_time ||
        !surface.basis_world.allFinite() ||
        (surface.basis_world.transpose() * surface.basis_world -
         Eigen::Matrix3d::Identity()).norm() > 1.0e-10 ||
        std::abs(surface.basis_world.determinant() - 1.0) > 1.0e-10 ||
        !std::isfinite(surface.friction_mu) || surface.friction_mu < 0.0 ||
        !std::isfinite(surface.min_normal_n) || surface.min_normal_n < 0.0 ||
        !std::isfinite(surface.max_normal_n) ||
        surface.max_normal_n < surface.min_normal_n)
        return false;
    const Eigen::Vector3d normal = surface.basis_world.col(2);
    return normal.allFinite() && std::abs(normal.norm() - 1.0) <= 1.0e-10;
}
inline FootTrajectoryResult Failed(JointPlannerFailure failure)
{
    FootTrajectoryResult result;
    result.failure = failure;
    return result;
}
inline bool SamePointMetadata(
    const TimedPoint &a, const TimedPoint &b)
{
    return a.valid == b.valid && a.role == b.role && a.frame == b.frame &&
        a.source_time == b.source_time &&
        (!a.valid || (a.value.x == b.value.x && a.value.y == b.value.y &&
                      a.value.z == b.value.z));
}
inline JointPlannerFailure SelectSurface(
    const CentroidalProblem &problem, std::size_t event_index,
    std::size_t candidate_index, const ContactSurface *&surface)
{
    const auto &request = problem.request;
    if (!problem.candidate_surfaces.empty())
    {
        if (!request.candidate_sets.empty() &&
            event_index < problem.candidate_surfaces.size() &&
            candidate_index < problem.candidate_surfaces[event_index].size())
        {
            surface = &problem.candidate_surfaces[event_index][candidate_index];
            return JointPlannerFailure::kNone;
        }
        return JointPlannerFailure::kInvalidInput;
    }
    if (event_index >= problem.event_surfaces.size())
        return JointPlannerFailure::kInvalidInput;
    surface = &problem.event_surfaces[event_index];
    return JointPlannerFailure::kNone;
}
inline JointPlannerFailure ValidateSchedule(
    const CentroidalProblem &problem, const PreparedTrajectory &prepared,
    TimeNs start, TimeNs end)
{
    const auto &schedule = problem.schedule;
    const auto &input = problem.request.input;
    const auto &events = problem.request.events.events;
    if (schedule.empty())
        return JointPlannerFailure::kCoverageIncomplete;
    TimeNs cursor = start;
    bool covered_any = false;
    std::array<bool, go2::kLegCount> anchor_ended{};
    for (const auto &interval : schedule)
    {
        if (interval.end <= interval.start)
            return JointPlannerFailure::kCoverageIncomplete;
        if (interval.end <= start)
            continue;
        if (interval.start >= end)
            break;
        const TimeNs segment_start = interval.start < start ? start : interval.start;
        const TimeNs segment_end = interval.end < end ? interval.end : end;
        if (segment_end <= segment_start)
            continue;
        if (!covered_any)
        {
            if (segment_start != start)
                return JointPlannerFailure::kCoverageIncomplete;
            covered_any = true;
        }
        else if (segment_start != cursor)
            return JointPlannerFailure::kCoverageIncomplete;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int event_index = interval.event_index[leg];
            if (interval.contact[leg])
            {
                if (event_index < -1)
                    return JointPlannerFailure::kInvalidInput;
                if (event_index < 0)
                {
                    if (anchor_ended[leg] ||
                        !input.measured_contact.mask[leg] ||
                        !input.feet[leg].measured_support_anchor_valid ||
                        !TimedPointValidAt(
                            input.feet[leg].measured_support_anchor_world,
                            PointRole::kSurfaceContactPoint, Frame::kWorld,
                            input.identity.source_state_time))
                        return JointPlannerFailure::kObservationUnavailable;
                }
                else
                {
                    if (static_cast<std::size_t>(event_index) >= events.size())
                        return JointPlannerFailure::kInvalidInput;
                    const auto &event = events[static_cast<std::size_t>(event_index)];
                    if (static_cast<std::size_t>(event.id.leg) != leg ||
                        segment_start < event.touchdown_time)
                        return JointPlannerFailure::kInvalidInput;
                    if (segment_end > event.contact_interval_end)
                        return JointPlannerFailure::kCoverageIncomplete;
                    const bool prepared_core = std::any_of(
                        prepared.events.begin(), prepared.events.end(),
                        [&](const PreparedEvent &candidate) {
                            return !candidate.continuation &&
                                candidate.event_index ==
                                    static_cast<std::size_t>(event_index);
                        });
                    if (!prepared_core)
                        return JointPlannerFailure::kInvalidInput;
                    anchor_ended[leg] = true;
                }
            }
            else
            {
                if (event_index != -1)
                    return JointPlannerFailure::kInvalidInput;
                const bool covered = std::any_of(
                    prepared.events_by_leg[leg].begin(),
                    prepared.events_by_leg[leg].end(),
                    [&](std::size_t prepared_index) {
                        const auto &event = prepared.events[prepared_index];
                        return event.liftoff <= segment_start &&
                            segment_end <= event.touchdown;
                    });
                if (!covered)
                    return JointPlannerFailure::kCoverageIncomplete;
                anchor_ended[leg] = true;
            }
        }
        cursor = segment_end;
        if (cursor == end)
            break;
    }
    if (!covered_any || cursor != end)
        return JointPlannerFailure::kCoverageIncomplete;
    // Every core event that intersects the requested horizon must be visible
    // in the authoritative schedule. Continuations are intentionally absent
    // here and are represented only by contact=false swing intervals above.
    for (std::size_t event_index = 0; event_index < events.size();
         ++event_index)
    {
        const auto prepared_it = std::find_if(
            prepared.events.begin(), prepared.events.end(),
            [&](const PreparedEvent &candidate) {
                return !candidate.continuation &&
                    candidate.event_index == event_index;
            });
        if (prepared_it == prepared.events.end())
            return JointPlannerFailure::kInvalidInput;
        for (const auto &interval : schedule)
        {
            if (interval.end <= start || interval.start >= end)
                continue;
            const TimeNs segment_start = interval.start < start ? start : interval.start;
            const TimeNs segment_end = interval.end < end ? interval.end : end;
            if (segment_end <= segment_start)
                continue;
            if (static_cast<std::size_t>(prepared_it->leg) >=
                go2::kLegCount)
                return JointPlannerFailure::kInvalidInput;
            const std::size_t leg = prepared_it->leg;
            const bool swing_overlap =
                segment_start < prepared_it->touchdown &&
                segment_end > prepared_it->liftoff;
            if (swing_overlap &&
                (interval.contact[leg] || interval.event_index[leg] != -1))
                return JointPlannerFailure::kInvalidInput;
            const bool stance_overlap =
                segment_start < prepared_it->contact_end &&
                segment_end > prepared_it->touchdown;
            if (stance_overlap &&
                (!interval.contact[leg] ||
                 interval.event_index[leg] !=
                     static_cast<int>(event_index)))
                return JointPlannerFailure::kInvalidInput;
        }
    }
    return JointPlannerFailure::kNone;
}
inline JointPlannerFailure ValidateCommittedPrefix(
    const CentroidalProblem &problem,
    const std::vector<PreparedEvent> &prepared,
    TimeNs observation_time)
{
    const auto &accepted = problem.request.accepted_commitments;
    if (accepted.events.empty())
        return JointPlannerFailure::kNone;
    const auto &events = problem.request.events.events;
    for (const auto &old_event : accepted.events)
    {
        if (!old_event.committed || !old_event.liftoff_valid ||
            old_event.liftoff_time.value < 0 ||
            !ValidObservedPoint(old_event.target_world,
                                  PointRole::kSurfaceContactPoint,
                                  observation_time))
            return JointPlannerFailure::kCommitmentConflict;
        auto current = std::find_if(
            events.begin(), events.end(), [&](const TouchdownEvent &event) {
                return event.id == old_event.id;
            });
        if (current == events.end() || !current->committed ||
            current->touchdown_time != old_event.touchdown_time ||
            current->contact_interval_end != old_event.contact_interval_end ||
            current->liftoff_valid != old_event.liftoff_valid ||
            current->liftoff_time != old_event.liftoff_time)
            return JointPlannerFailure::kCommitmentConflict;
        auto selected = std::find_if(
            prepared.begin(), prepared.end(), [&](const PreparedEvent &event) {
                return event.event_index ==
                    static_cast<std::size_t>(&*current - events.data());
            });
        if (selected == prepared.end() ||
            (current->target_world.valid &&
             !SamePointMetadata(old_event.target_world,
                                current->target_world)) ||
            !SamePointMetadata(
                old_event.target_world,
                TimedPoint{ToVec3(selected->surface_point), Frame::kWorld,
                           selected->target_source_time, true,
                           PointRole::kSurfaceContactPoint}))
            return JointPlannerFailure::kCommitmentConflict;
    }
    return JointPlannerFailure::kNone;
}
inline JointPlannerFailure Prepare(
    const FootTrajectoryRequest &request, PreparedTrajectory &prepared)
{
    if (request.problem == nullptr)
        return JointPlannerFailure::kInvalidInput;
    const auto &problem = *request.problem;
    const auto &input = problem.request.input;
    const auto observation_time = input.identity.source_state_time;
    const bool has_commanded_boundary = request.commanded_initial.enabled;
    if (!input.basic_valid() || !input.identity.valid() ||
        request.end <= request.start ||
        (!has_commanded_boundary && request.start != observation_time) ||
        (has_commanded_boundary && request.start < observation_time) ||
        !std::isfinite(request.swing_clearance_m) ||
        request.swing_clearance_m < 0.0 ||
        problem.schedule_epoch != input.identity.schedule_epoch ||
        problem.schedule_epoch == 0)
        return JointPlannerFailure::kObservationUnavailable;
    if (request.enable_commanded_stance_settling &&
        (!has_commanded_boundary ||
         !std::isfinite(request.commanded_stance_settling_duration_s) ||
         request.commanded_stance_settling_duration_s <= 0.0 ||
         request.commanded_stance_settling_duration_s > 0.020))
        return JointPlannerFailure::kInitialConditionConflict;
    prepared.preserve_inflight_clearance_phase =
        request.preserve_inflight_clearance_phase;
    if (has_commanded_boundary &&
        !request.commanded_initial.valid_for(request.start))
        return JointPlannerFailure::kObservationUnavailable;
    prepared.start = request.start;
    prepared.end = request.end;
    prepared.initial_reference_source_time = has_commanded_boundary
        ? request.commanded_initial.source_time : observation_time;
    prepared.clearance_m = request.swing_clearance_m;
    prepared.add_clearance_to_inflight_continuation = request.add_clearance_to_inflight_continuation;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        prepared.initial_center[leg].setZero();
        prepared.initial_velocity[leg].setZero();
        if (!ValidMeasuredPointAt(
                input.feet[leg].foot_collision_center_world,
                PointRole::kFootCollisionCenter, observation_time) ||
            !request.collision_radius_valid[leg] ||
            !std::isfinite(request.collision_radius_m[leg]) ||
            request.collision_radius_m[leg] <= 0.0)
            return JointPlannerFailure::kObservationUnavailable;
        if (has_commanded_boundary)
        {
            // This branch is deliberately independent of measured feet and
            // never writes back into problem.request.input.
            prepared.initial_center[leg] =
                ToEigen(request.commanded_initial.center_world[leg].value);
            prepared.initial_velocity[leg] =
                ToEigen(request.commanded_initial.velocity_world[leg]);
        }
        else
        {
            if (!ValidMeasuredPointAt(
                    input.feet[leg].foot_collision_center_world,
                    PointRole::kFootCollisionCenter, observation_time) ||
                !request.initial_velocity_valid[leg] ||
                !Finite(ToEigen(request.initial_velocity_world[leg])))
                return JointPlannerFailure::kObservationUnavailable;
            prepared.initial_center[leg] =
                ToEigen(input.feet[leg].foot_collision_center_world.value);
            prepared.initial_velocity[leg] =
                ToEigen(request.initial_velocity_world[leg]);
        }
    }
    const auto &events = problem.request.events.events;
    const auto &sets = problem.request.candidate_sets;
    if (sets.size() != events.size() || problem.combination.size() != events.size() ||
        (!problem.candidate_surfaces.empty() &&
         (!problem.event_surfaces.empty() ||
          problem.candidate_surfaces.size() != events.size())) ||
        (problem.candidate_surfaces.empty() &&
         problem.event_surfaces.size() != events.size()))
        return JointPlannerFailure::kInvalidInput;
    prepared.events.reserve(events.size());
    for (std::size_t index = 0; index < events.size(); ++index)
    {
        const auto &event = events[index];
        const auto &set = sets[index];
        const std::size_t candidate_index = problem.combination[index];
        if (event.id.schedule_epoch != input.identity.schedule_epoch ||
            event.id.sequence == 0 ||
            static_cast<std::size_t>(event.id.leg) >= go2::kLegCount ||
            !event.liftoff_valid || event.liftoff_time.value < 0 ||
            event.liftoff_time >= event.touchdown_time ||
            event.contact_interval_end <= event.touchdown_time ||
            (index > 0 && events[index - 1].touchdown_time >
                event.touchdown_time) ||
            !(set.event_id == event.id) || !set.complete ||
            candidate_index >= set.candidates.size())
            return JointPlannerFailure::kInvalidInput;
        const auto &candidate = set.candidates[candidate_index];
        if (!candidate.geometry_hard_feasible || !candidate.valid())
            return candidate.geometry_hard_feasible
                ? JointPlannerFailure::kInvalidInput
                : JointPlannerFailure::kNoFeasibleCandidateInSet;
        if (candidate.coverage != MapCoverageState::kKnown ||
            candidate.target_world.source_time > observation_time)
            return candidate.coverage == MapCoverageState::kKnown
                ? JointPlannerFailure::kInvalidInput
                : JointPlannerFailure::kCoverageIncomplete;
        if (event.target_world.valid &&
            !ValidObservedPoint(event.target_world,
                                PointRole::kSurfaceContactPoint,
                                observation_time))
            return JointPlannerFailure::kInvalidInput;
        const ContactSurface *surface = nullptr;
        const auto surface_failure = SelectSurface(
            problem, index, candidate_index, surface);
        if (surface_failure != JointPlannerFailure::kNone)
            return surface_failure;
        const TimeNs surface_end = request.allow_surface_contact_tail_beyond_horizon
            ? std::min(event.contact_interval_end,
                       std::max(request.end,event.touchdown_time))
            : event.contact_interval_end;
        if (surface == nullptr ||
            !ValidSurface(*surface, input.identity.map_epoch, surface_end))
            return JointPlannerFailure::kCoverageIncomplete;
        const Eigen::Vector3d normal = surface->basis_world.col(2);
        const Eigen::Vector3d target = ToEigen(candidate.target_world.value) +
            request.collision_radius_m[static_cast<std::size_t>(event.id.leg)] *
                normal;
        if (!Finite(target))
            return JointPlannerFailure::kInvalidInput;
        PreparedEvent prepared_event;
        prepared_event.event_index = index;
        prepared_event.leg = static_cast<std::size_t>(event.id.leg);
        prepared_event.liftoff = event.liftoff_time;
        prepared_event.touchdown = event.touchdown_time;
        prepared_event.contact_end = event.contact_interval_end;
        prepared_event.surface_point =
            ToEigen(candidate.target_world.value);
        prepared_event.p1 = target;
        prepared_event.target_source_time = candidate.target_world.source_time;
        prepared_event.normal = normal;
        prepared.events.push_back(prepared_event);
        prepared.events_by_leg[prepared_event.leg].push_back(
            prepared.events.size() - 1);
    }
    // A terminal continuation is the only permitted source for a swing whose
    // touchdown lies at or beyond the dynamic horizon. It is validated from
    // the same event, candidate and surface roles, but stays outside the
    // problem event table and therefore cannot alter commitments or dynamics.
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &continuation = request.continuation[leg];
        if (!continuation.valid)
            continue;
        const auto &event = continuation.event;
        const auto &candidate = continuation.candidate;
        if (event.id.schedule_epoch != input.identity.schedule_epoch ||
            event.id.leg != static_cast<go2::Leg>(leg) ||
            event.id.sequence == 0 || !event.liftoff_valid ||
            event.liftoff_time.value < 0 ||
            event.liftoff_time >= request.end ||
            event.liftoff_time >= event.touchdown_time ||
            event.touchdown_time < request.end ||
            event.contact_interval_end <= event.touchdown_time ||
            !candidate.geometry_hard_feasible || !candidate.valid() ||
            candidate.coverage != MapCoverageState::kKnown ||
            candidate.target_world.source_time > observation_time ||
            (event.target_world.valid &&
             !ValidObservedPoint(event.target_world,
                                 PointRole::kSurfaceContactPoint,
                                 observation_time)) ||
            !ValidSurface(continuation.surface, input.identity.map_epoch,
                          event.contact_interval_end))
            return JointPlannerFailure::kCoverageIncomplete;
        for (const auto &existing : events)
            if (existing.id == event.id)
                return JointPlannerFailure::kInvalidInput;
        for (const auto prepared_index : prepared.events_by_leg[leg])
        {
            const auto &existing = prepared.events[prepared_index];
            if (existing.touchdown >= event.touchdown_time)
                return JointPlannerFailure::kInvalidInput;
        }
        const Eigen::Vector3d normal = continuation.surface.basis_world.col(2);
        const Eigen::Vector3d target =
            ToEigen(candidate.target_world.value) +
            request.collision_radius_m[leg] * normal;
        if (!Finite(target))
            return JointPlannerFailure::kInvalidInput;
        PreparedEvent prepared_event;
        prepared_event.event_index =
            std::numeric_limits<std::size_t>::max();
        prepared_event.continuation = true;
        prepared_event.leg = leg;
        prepared_event.liftoff = event.liftoff_time;
        prepared_event.touchdown = event.touchdown_time;
        prepared_event.contact_end = event.contact_interval_end;
        prepared_event.surface_point =
            ToEigen(candidate.target_world.value);
        prepared_event.p1 = target;
        prepared_event.target_source_time = candidate.target_world.source_time;
        prepared_event.normal = normal;
        prepared.events.push_back(prepared_event);
        prepared.events_by_leg[leg].push_back(
            prepared.events.size() - 1);
    }
    const auto schedule_failure =
        ValidateSchedule(problem, prepared, request.start, request.end);
    if (schedule_failure != JointPlannerFailure::kNone)
        return schedule_failure;
    TimeNs commanded_stance_settling_duration{};
    if (request.enable_commanded_stance_settling)
    {
        commanded_stance_settling_duration =
            TimeNs::FromSeconds(request.commanded_stance_settling_duration_s);
        if (commanded_stance_settling_duration.value <= 0)
            return JointPlannerFailure::kInitialConditionConflict;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        bool in_flight_at_start = false;
        bool swing_at_start = false;
        TimeNs next_stance_boundary = request.end;
        for (const auto prepared_index : prepared.events_by_leg[leg])
        {
            auto &event = prepared.events[prepared_index];
            if (event.liftoff < request.start &&
                request.start < event.touchdown)
                in_flight_at_start = true;
            if (event.liftoff <= request.start &&
                request.start < event.touchdown)
                swing_at_start = true;
            if (event.liftoff > request.start &&
                event.liftoff < next_stance_boundary)
                next_stance_boundary = event.liftoff;
            if (event.touchdown <= request.start &&
                request.start < event.contact_end &&
                event.contact_end < next_stance_boundary)
                next_stance_boundary = event.contact_end;
        }
        const bool initial_stance = !swing_at_start;
        bool settling = false;
        const bool needs_settling =
            prepared.initial_velocity[leg].norm() > 1.0e-12;
        if (request.enable_commanded_stance_settling && initial_stance &&
            needs_settling)
        {
            const TimeNs settling_limit =
                next_stance_boundary < request.end ? next_stance_boundary
                                                    : request.end;
            if (settling_limit <= request.start)
                return JointPlannerFailure::kInitialConditionConflict;
            const std::int64_t available_ns =
                settling_limit.value - request.start.value;
            const std::int64_t duration_ns = std::min(
                commanded_stance_settling_duration.value, available_ns);
            if (duration_ns <= 0 ||
                request.start.value >
                    std::numeric_limits<std::int64_t>::max() - duration_ns)
                return JointPlannerFailure::kInitialConditionConflict;
            const TimeNs settling_end{request.start.value + duration_ns};
            prepared.commanded_stance_settling[leg] = true;
            prepared.commanded_stance_settling_end[leg] = settling_end;
            settling = true;
        }
        if (prepared.initial_velocity[leg].norm() > 1.0e-12 &&
            !in_flight_at_start && !settling)
            return JointPlannerFailure::kInitialConditionConflict;
        const PreparedEvent *completed_at_start = nullptr;
        if (has_commanded_boundary && !in_flight_at_start)
        {
            for (const auto prepared_index : prepared.events_by_leg[leg])
            {
                const auto &event = prepared.events[prepared_index];
                if (event.touchdown <= request.start)
                    completed_at_start = &event;
            }
            // SamplePrepared selects the completed touchdown state at the
            // handover boundary. Reject a mismatched commanded p/v instead
            // of silently truncating or restarting a swing. Settling may only
            // bridge from a previously completed touchdown; it cannot hide a
            // position mismatch at an exact touchdown boundary.
            if (completed_at_start != nullptr &&
                ((prepared.initial_center[leg] - completed_at_start->p1).norm() >
                     1.0e-10 ||
                 (prepared.initial_velocity[leg].norm() > 1.0e-12 &&
                  !settling)))
                return JointPlannerFailure::kInitialConditionConflict;
        }
        const PreparedEvent *previous = nullptr;
        for (const auto prepared_index : prepared.events_by_leg[leg])
        {
            auto &event = prepared.events[prepared_index];
            if (previous != nullptr && previous->contact_end != event.liftoff &&
                previous->contact_end > request.start &&
                event.liftoff < request.end)
                return JointPlannerFailure::kInvalidInput;
            event.starts_in_flight = event.liftoff < request.start &&
                request.start < event.touchdown;
            if (event.starts_in_flight && prepared.preserve_inflight_clearance_phase) {
                const double fraction=static_cast<double>(event.touchdown.value-request.start.value)/
                    static_cast<double>(event.touchdown.value-event.liftoff.value);
                event.inflight_clearance_m=prepared.clearance_m*fraction*fraction*fraction*fraction;
            }
            if (event.touchdown <= request.start)
            {
                event.interpolation_start = request.start;
                event.p0 = event.p1;
                event.v0.setZero();
            }
            else if (event.starts_in_flight)
            {
                event.interpolation_start = request.start;
                event.p0 = prepared.initial_center[leg];
                event.v0 = prepared.initial_velocity[leg];
            }
            else
            {
                event.interpolation_start = event.liftoff;
                event.p0 = previous == nullptr ||
                        previous->contact_end <= request.start
                    ? prepared.initial_center[leg] : previous->p1;
                event.v0.setZero();
            }
            previous = &event;
        }
        if (!prepared.events_by_leg[leg].empty())
        {
            const auto &last = prepared.events[
                prepared.events_by_leg[leg].back()];
            // [start,end) is the ordinary sampler domain. A final contact
            // interval must cover its tail, or an explicit continuation must
            // cover the swing. Otherwise this API would invent a stance after
            // contact ended while the next target is unknown.
            if (last.contact_end > request.start &&
                last.contact_end < request.end)
                return JointPlannerFailure::kCoverageIncomplete;
        }
    }
    return ValidateCommittedPrefix(problem, prepared.events, observation_time);
}
inline void EvaluateSwing(
    const PreparedEvent &event, TimeNs time, double clearance_m,
    Eigen::Vector3d &position, Eigen::Vector3d &velocity,
    Eigen::Vector3d &acceleration)
{
    const double dt = static_cast<double>(
        event.touchdown.value - event.interpolation_start.value) * 1.0e-9;
    const double elapsed = static_cast<double>(
        time.value - event.interpolation_start.value) * 1.0e-9;
    const double u = std::clamp(elapsed / dt, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h00_d = 6.0 * u2 - 6.0 * u;
    const double h10_d = 3.0 * u2 - 4.0 * u + 1.0;
    const double h01_d = -6.0 * u2 + 6.0 * u;
    const double h00_dd = 12.0 * u - 6.0;
    const double h10_dd = 6.0 * u - 4.0;
    const double h01_dd = -12.0 * u + 6.0;
    position = h00 * event.p0 + h10 * dt * event.v0 + h01 * event.p1;
    velocity = (h00_d * event.p0 + h10_d * dt * event.v0 +
                h01_d * event.p1) / dt;
    acceleration = (h00_dd * event.p0 + h10_dd * dt * event.v0 +
                    h01_dd * event.p1) / (dt * dt);
    const double bump = 16.0 * u2 * (1.0 - u) * (1.0 - u);
    const double bump_d = 32.0 * u - 96.0 * u2 + 64.0 * u3;
    const double bump_dd = 32.0 - 192.0 * u + 192.0 * u2;
    position += clearance_m * bump * event.normal;
    velocity += clearance_m * bump_d * event.normal / dt;
    acceleration += clearance_m * bump_dd * event.normal / (dt * dt);
}
inline void EvaluateCommandedStanceSettling(
    const Eigen::Vector3d &position_start,
    const Eigen::Vector3d &velocity_start, TimeNs start, TimeNs end,
    TimeNs time, Eigen::Vector3d &position, Eigen::Vector3d &velocity,
    Eigen::Vector3d &acceleration)
{
    const double dt = static_cast<double>(end.value - start.value) * 1.0e-9;
    const double elapsed = static_cast<double>(time.value - start.value) * 1.0e-9;
    const double u = std::clamp(elapsed / dt, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h10_d = 3.0 * u2 - 4.0 * u + 1.0;
    const double h10_dd = 6.0 * u - 4.0;
    position = position_start + h10 * dt * velocity_start;
    velocity = h10_d * velocity_start;
    acceleration = (h10_dd / dt) * velocity_start;
}
inline FootTrajectorySample SamplePrepared(
    const PreparedTrajectory &prepared, TimeNs time)
{
    FootTrajectorySample sample;
    sample.time = time;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        Eigen::Vector3d position = prepared.initial_center[leg];
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        bool done = false;
        for (const auto prepared_index : prepared.events_by_leg[leg])
        {
            const auto &event = prepared.events[prepared_index];
            if (time < event.liftoff)
                break;
            if (time < event.touchdown)
            {
                double clearance_m = prepared.clearance_m;
                if (event.starts_in_flight)
                {
                    if (prepared.preserve_inflight_clearance_phase)
                        clearance_m = event.inflight_clearance_m;
                    else if (!prepared.add_clearance_to_inflight_continuation)
                        clearance_m = 0.0;
                }
                EvaluateSwing(event, time, clearance_m, position, velocity,
                              acceleration);
                done = true;
                break;
            }
            position = event.p1;
            velocity.setZero();
            acceleration.setZero();
            if (time < event.contact_end)
            {
                done = true;
                break;
            }
        }
        (void)done;
        if (prepared.commanded_stance_settling[leg] &&
            time >= prepared.start &&
            time <= prepared.commanded_stance_settling_end[leg])
            EvaluateCommandedStanceSettling(
                prepared.initial_center[leg], prepared.initial_velocity[leg],
                prepared.start, prepared.commanded_stance_settling_end[leg],
                time, position, velocity, acceleration);
        // time is the applicability stamp; source_time remains the
        // initial-reference provenance of this generated reference.
        sample.center_world[leg] = {
            ToVec3(position), Frame::kWorld,
            prepared.initial_reference_source_time, true,
            PointRole::kFootCollisionCenter};
        sample.velocity_world[leg] = ToVec3(velocity);
        sample.acceleration_world[leg] = ToVec3(acceleration);
        sample.leg_valid[leg] = true;
    }
    sample.valid = true;
    return sample;
}
} // namespace foot_trajectory_detail
inline FootTrajectoryResult SampleFootTrajectory(
    const FootTrajectoryRequest &request,
    const std::vector<TimeNs> &sample_times)
{
    if (sample_times.empty())
        return foot_trajectory_detail::Failed(JointPlannerFailure::kInvalidInput);
    foot_trajectory_detail::PreparedTrajectory prepared;
    const auto failure = foot_trajectory_detail::Prepare(request, prepared);
    if (failure != JointPlannerFailure::kNone)
        return foot_trajectory_detail::Failed(failure);
    for (std::size_t i = 0; i < sample_times.size(); ++i)
        if (sample_times[i] < prepared.start ||
            sample_times[i] >= prepared.end ||
            (i > 0 && sample_times[i] <= sample_times[i - 1]))
            return foot_trajectory_detail::Failed(JointPlannerFailure::kInvalidInput);
    FootTrajectoryResult result;
    result.failure = JointPlannerFailure::kNone;
    result.samples.reserve(sample_times.size());
    for (const auto time : sample_times)
        result.samples.push_back(
            foot_trajectory_detail::SamplePrepared(prepared, time));
    result.valid = true;
    return result;
}
inline FootTrajectoryResult SampleFootTrajectoryAt(
    const FootTrajectoryRequest &request, TimeNs time)
{
    return SampleFootTrajectory(request, std::vector<TimeNs>{time});
}

// The ordinary sampler is half-open: request.start <= t < request.end.
// This separate state query exposes the closed terminal state at exactly end
// after the same complete-coverage validation, without changing that domain.
inline FootTrajectoryResult SampleFootTrajectoryEndState(
    const FootTrajectoryRequest &request)
{
    foot_trajectory_detail::PreparedTrajectory prepared;
    const auto failure = foot_trajectory_detail::Prepare(request, prepared);
    if (failure != JointPlannerFailure::kNone)
        return foot_trajectory_detail::Failed(failure);
    FootTrajectoryResult result;
    result.failure = JointPlannerFailure::kNone;
    result.samples.push_back(
        foot_trajectory_detail::SamplePrepared(prepared, prepared.end));
    result.valid = true;
    return result;
}
} // namespace stage_c
} // namespace go2_terrain
