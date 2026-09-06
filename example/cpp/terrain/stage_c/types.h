#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "go2_forward_kinematics.h"
#include "terrain_model.h"

namespace go2_terrain
{
namespace stage_c
{

constexpr std::size_t kStageCMaxEvents = 32;

struct TimeNs
{
    std::int64_t value = 0;

    static TimeNs FromSeconds(double seconds)
    {
        const long double ns = static_cast<long double>(seconds) * 1.0e9L;
        if (!std::isfinite(seconds) ||
            ns >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
            ns <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()))
            return {std::numeric_limits<std::int64_t>::min()};
        return {static_cast<std::int64_t>(std::round(ns))};
    }

    double seconds() const
    {
        return static_cast<double>(value) * 1.0e-9;
    }
};

inline bool operator==(TimeNs a, TimeNs b) { return a.value == b.value; }
inline bool operator!=(TimeNs a, TimeNs b) { return !(a == b); }
inline bool operator<(TimeNs a, TimeNs b) { return a.value < b.value; }
inline bool operator<=(TimeNs a, TimeNs b) { return a.value <= b.value; }
inline bool operator>(TimeNs a, TimeNs b) { return b < a; }
inline bool operator>=(TimeNs a, TimeNs b) { return b <= a; }

enum class Frame : std::uint8_t
{
    kUnknown = 0,
    kWorld,
    kBase,
    kHeadingMap,
};

struct TimedPoint
{
    go2::Vec3 value{};
    Frame frame = Frame::kUnknown;
    TimeNs source_time{};
    bool valid = false;
};

enum class CaptureMode : std::uint8_t
{
    kShadow = 0,
    kActuation,
};

enum class ContactProvenance : std::uint8_t
{
    kUnknown = 0,
    kMeasured,
    kPlanned,
    kApplied,
};

struct ContactEvidence
{
    std::array<bool, go2::kLegCount> mask{};
    ContactProvenance provenance = ContactProvenance::kUnknown;
    TimeNs source_time{};
    bool valid = false;
};

enum class MapCoverageState : std::uint8_t
{
    kMetadataUnavailable = 0,
    kMetadataOnly,
    kOutsideGrid,
    kUnknownInside,
    kPartial,
    kKnown,
};

inline const char *MapCoverageStateName(MapCoverageState state)
{
    switch (state)
    {
    case MapCoverageState::kMetadataOnly: return "metadata_only";
    case MapCoverageState::kOutsideGrid: return "outside_grid";
    case MapCoverageState::kUnknownInside: return "unknown_inside";
    case MapCoverageState::kPartial: return "partial";
    case MapCoverageState::kKnown: return "known";
    default: return "metadata_unavailable";
    }
}

struct MapObservation
{
    bool metadata_valid = false;
    std::string frame_id;
    TerrainSource source = TerrainSource::kNone;
    std::uint64_t epoch = 0;
    TimeNs acquisition_time{};
    double resolution_m = 0.0;
    std::array<double, 2> origin_m{0.0, 0.0};
    std::size_t width = 0;
    std::size_t height = 0;
    std::size_t known_cells = 0;
    std::size_t total_cells = 0;
    std::size_t outside_cells = 0;
    MapCoverageState coverage = MapCoverageState::kMetadataUnavailable;

    std::size_t unknown_inside_cells() const
    {
        const std::size_t accounted = known_cells + outside_cells;
        return total_cells > accounted ? total_cells - accounted : 0;
    }
};

inline MapCoverageState ClassifyMapCoverage(const MapObservation &map)
{
    if (!map.metadata_valid || map.width == 0 || map.height == 0 ||
        map.total_cells == 0 || map.known_cells > map.total_cells ||
        map.outside_cells > map.total_cells ||
        map.known_cells > map.total_cells - map.outside_cells)
        return MapCoverageState::kMetadataUnavailable;
    if (map.known_cells == 0)
        return map.outside_cells == map.total_cells
            ? MapCoverageState::kOutsideGrid
            : MapCoverageState::kUnknownInside;
    if (map.unknown_inside_cells() > 0 || map.outside_cells > 0)
        return MapCoverageState::kPartial;
    return MapCoverageState::kKnown;
}

struct BodyObservation
{
    TimedPoint base_position_world{};
    TimedPoint model_com_world{};
    go2::Vec3 base_velocity_world{};
    go2::Vec3 com_velocity_world{};
    go2::Vec3 angular_velocity_body{};
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double mass_kg = 0.0;
    bool valid = false;
    bool model_com_valid = false;
};

struct FootObservation
{
    TimedPoint foot_site_world{};
    TimedPoint contact_patch_world{};
    TimedPoint contact_patch_base{};
    TimedPoint measured_support_anchor_world{};
    bool measured_support_anchor_valid = false;
};

struct PlanningIdentity
{
    std::uint64_t source_state_tick = 0;
    TimeNs source_state_time{};
    std::uint64_t map_epoch = 0;
    std::uint64_t schedule_epoch = 0;
    std::uint64_t source_plan_id = 0;

    bool valid() const
    {
        return source_state_tick != 0 && source_state_time.value >= 0 &&
            map_epoch != 0 && schedule_epoch != 0;
    }
};

struct Phase1CommandAuthority
{
    std::uint64_t command_epoch = 0;
    double shaped_vx_mps = 0.0;
    double shaped_vy_mps = 0.0;
    double applied_vx_mps = 0.0;
    double applied_vy_mps = 0.0;
    double period_s = 0.0;
    double duty_factor = 0.0;
    bool valid = false;
};

struct PlanningBudget
{
    std::size_t max_candidate_combinations = 0;
    std::size_t max_solver_iterations = 0;
    TimeNs prediction_start{};
    TimeNs prediction_end{};
    std::int64_t max_source_age_ns = 0;
    double deadline_us = 0.0;
};

struct TerrainPlanningInput
{
    PlanningIdentity identity{};
    BodyObservation body{};
    std::array<FootObservation, go2::kLegCount> feet{};
    ContactEvidence measured_contact{};
    ContactEvidence planned_contact{};
    ContactEvidence applied_contact{};
    MapObservation map{};
    Phase1CommandAuthority command{};
    PlanningBudget budget{};
    double initial_support_margin_m = std::numeric_limits<double>::quiet_NaN();
    bool initial_support_margin_valid = false;

    bool basic_valid() const
    {
        return identity.valid() && body.valid && body.base_position_world.valid &&
            body.base_position_world.frame == Frame::kWorld &&
            measured_contact.valid &&
            measured_contact.provenance == ContactProvenance::kMeasured &&
            map.metadata_valid;
    }
};

struct TouchdownEventId
{
    std::uint64_t schedule_epoch = 0;
    go2::Leg leg = go2::Leg::FR;
    std::uint32_t sequence = 0;

    bool operator==(const TouchdownEventId &other) const
    {
        return schedule_epoch == other.schedule_epoch && leg == other.leg &&
            sequence == other.sequence;
    }
};

inline bool operator<(const TouchdownEventId &a, const TouchdownEventId &b)
{
    if (a.schedule_epoch != b.schedule_epoch)
        return a.schedule_epoch < b.schedule_epoch;
    if (a.leg != b.leg)
        return static_cast<std::size_t>(a.leg) <
            static_cast<std::size_t>(b.leg);
    return a.sequence < b.sequence;
}

struct TouchdownEvent
{
    TouchdownEventId id{};
    TimeNs touchdown_time{};
    TimeNs contact_interval_end{};
    TimedPoint target_world{};
    std::uint64_t source_plan_id = 0;
    bool committed = false;
};

struct TouchdownEventTable
{
    std::vector<TouchdownEvent> events;

    bool valid() const
    {
        if (events.empty() || events.size() > kStageCMaxEvents)
            return false;
        for (std::size_t i = 0; i < events.size(); ++i)
        {
            const auto &event = events[i];
            if (event.id.schedule_epoch == 0 || event.id.sequence == 0 ||
                static_cast<std::size_t>(event.id.leg) >= go2::kLegCount ||
                !std::isfinite(event.target_world.value.x) ||
                !std::isfinite(event.target_world.value.y) ||
                !std::isfinite(event.target_world.value.z) ||
                event.touchdown_time.value < 0 ||
                event.contact_interval_end < event.touchdown_time ||
                !event.target_world.valid ||
                event.target_world.frame != Frame::kWorld)
                return false;
            if (i > 0 && events[i - 1].touchdown_time > event.touchdown_time)
                return false;
            for (std::size_t j = 0; j < i; ++j)
                if (events[j].id == event.id)
                    return false;
        }
        bool saw_uncommitted = false;
        for (const auto &event : events)
        {
            if (!event.committed)
                saw_uncommitted = true;
            else if (saw_uncommitted)
                return false;
        }
        return true;
    }

    bool committed_prefix_compatible(const TouchdownEventTable &proposal) const
    {
        if (!valid() || !proposal.valid())
            return false;
        for (const auto &old_event : events)
        {
            if (!old_event.committed)
                continue;
            auto it = std::find_if(
                proposal.events.begin(), proposal.events.end(),
                [&](const TouchdownEvent &candidate) {
                    return candidate.id == old_event.id;
                });
            if (it == proposal.events.end() || !it->committed ||
                it->touchdown_time != old_event.touchdown_time ||
                it->contact_interval_end != old_event.contact_interval_end ||
                it->target_world.value.x != old_event.target_world.value.x ||
                it->target_world.value.y != old_event.target_world.value.y ||
                it->target_world.value.z != old_event.target_world.value.z)
                return false;
        }
        return true;
    }
};

struct StageCCandidate
{
    std::uint32_t candidate_id = 0;
    TimedPoint target_world{};
    double foothold_cost = 0.0;
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    MapCoverageState coverage = MapCoverageState::kMetadataUnavailable;
    bool geometry_hard_feasible = false;
};

struct EventCandidateSet
{
    TouchdownEventId event_id{};
    std::vector<StageCCandidate> candidates;
    bool complete = true;

    bool has_hard_candidate() const
    {
        return std::any_of(
            candidates.begin(), candidates.end(),
            [](const StageCCandidate &candidate) {
                return candidate.geometry_hard_feasible;
            });
    }
};

// All planner methods in the future comparison harness consume this same
// post-adapter fixture. The method label is metadata, never a reason to alter
// state, candidates, timing, or contact provenance.
enum class PlannerEntryPoint : std::uint8_t
{
    kLegacyRaw = 0,
    kLegacyNormalized,
    kJointC0,
};

struct PlannerComparisonFixture
{
    TerrainPlanningInput input{};
    TouchdownEventTable events{};
    std::vector<EventCandidateSet> candidate_sets;
};

enum class JointPlannerFailure : std::uint8_t
{
    kNone = 0,
    kInvalidInput,
    kObservationUnavailable,
    kInitialConditionConflict,
    kNoFeasibleCandidateInSet,
    kSearchIncomplete,
    kBudgetExhausted,
    kNumericalFailure,
    kCoverageIncomplete,
    kCommitmentConflict,
    kDeadlineExceeded,
    kDynamicsInfeasible,
};

inline const char *JointPlannerFailureName(JointPlannerFailure failure)
{
    switch (failure)
    {
    case JointPlannerFailure::kDynamicsInfeasible:
        return "dynamics_infeasible";
    case JointPlannerFailure::kObservationUnavailable:
        return "observation_unavailable";
    case JointPlannerFailure::kInitialConditionConflict:
        return "initial_condition_conflict";
    case JointPlannerFailure::kNoFeasibleCandidateInSet:
        return "no_feasible_candidate_in_set";
    case JointPlannerFailure::kSearchIncomplete:
        return "search_incomplete";
    case JointPlannerFailure::kBudgetExhausted:
        return "budget_exhausted";
    case JointPlannerFailure::kNumericalFailure:
        return "numerical_failure";
    case JointPlannerFailure::kCoverageIncomplete:
        return "coverage_incomplete";
    case JointPlannerFailure::kCommitmentConflict:
        return "commitment_conflict";
    case JointPlannerFailure::kDeadlineExceeded:
        return "deadline_exceeded";
    case JointPlannerFailure::kInvalidInput:
        return "invalid_input";
    default:
        return "none";
    }
}

struct FailureWitness
{
    JointPlannerFailure failure = JointPlannerFailure::kNone;
    std::size_t event_index = 0;
    int knot = -1;
    std::string detail;
};

struct RolloutKnot
{
    TimeNs time{};
    go2::Vec3 com_world{};
    go2::Vec3 body_position_world{};
    go2::Vec3 com_velocity_world{};
    go2::Vec3 angular_momentum_world{};
    bool body_pose_valid = false;
    std::array<double, 3> orientation_rpy{};
    int contact_mask = 0;
};

struct ContactForceInterval
{
    TimeNs start{};
    TimeNs end{};
    std::array<go2::Vec3, go2::kLegCount> force_world{};
    std::array<bool, go2::kLegCount> contact{};
};

struct JointRollout
{
    std::vector<RolloutKnot> knots;
    std::vector<ContactForceInterval> force_intervals;
    bool complete = false;
};

struct FeasibilityCertificate
{
    bool input_checked = false;
    bool geometry_checked = false;
    bool original_model_checked = false;
    bool complete_coverage = false;
    bool commitments_preserved = false;
    bool continuous_solver_run = false;
};

struct PlanCandidate
{
    std::vector<std::size_t> candidate_indices;
    JointRollout rollout{};
    FeasibilityCertificate certificate{};
    double cost = std::numeric_limits<double>::infinity();
};

struct PendingExecutionBundle
{
    std::uint64_t proposal_id = 0;
    TouchdownEventTable events{};
    PlanCandidate plan{};
};

struct AcceptedExecutionBundle
{
    std::uint64_t execution_version = 0;
    std::uint64_t source_proposal_id = 0;
    TouchdownEventTable events{};
    PlanCandidate plan{};
    bool valid = false;
};

struct HorizontalReferenceTrace
{
    double phase1_command_vx_mps = 0.0;
    double supplied_anchor_x_first_m = 0.0;
    double supplied_anchor_x_last_m = 0.0;
    double solver_reference_x_first_m = 0.0;
    double solver_reference_x_last_m = 0.0;
    double predicted_state_x_first_m = 0.0;
    double predicted_state_x_last_m = 0.0;
};

inline double SolverHorizontalReferenceSpan(
    const HorizontalReferenceTrace &trace)
{
    return std::abs(trace.solver_reference_x_last_m -
                    trace.solver_reference_x_first_m);
}

} // namespace stage_c
} // namespace go2_terrain
