#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "terrain_motion_plan.h"

namespace go2_terrain
{

struct TerrainTouchdownEvent
{
    bool valid = false;
    std::size_t knot = 0;
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    std::size_t leg = 0;
    double event_time_s = 0.0;
    go2::Vec3 target_world{};
};

struct TerrainExecutionCommitment
{
    bool valid = false;
    bool in_flight = false;
    bool measured_touchdown = false;
    // Source identity is provenance only. A replan may inherit this
    // commitment when its touchdown event has the same leg, time and target.
    std::uint64_t source_plan_id = 0;
    std::uint64_t source_plan_epoch = 0;
    double target_time_s = 0.0;
    go2::Vec3 target_world{};
};

// One immutable, diagnostic-only consumer snapshot.  The fields deliberately
// keep measured, planned, and applied contact separate; no field is inferred
// from another source.  This path is shadow-only until an explicit caller opts
// in, so the accepted Phase 1/B0 route remains unchanged.
struct TerrainExecutionSnapshot
{
    bool valid = false;
    bool horizon_covered = false;
    double state_time_s = 0.0;
    double mpc_dt_s = 0.0;
    std::size_t mpc_horizon = 0;
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    go2::Vec3 model_com_world{};
    std::array<bool, go2::kLegCount> measured_contact{};
    std::array<bool, go2::kLegCount> applied_contact{};
    bool measured_contact_valid = false;
    bool applied_contact_valid = false;
    std::array<std::array<bool, go2::kLegCount>, kTerrainContactMaxKnots>
        planned_contact{};
    std::array<std::array<TerrainTouchdownEvent, go2::kLegCount>,
               kTerrainContactMaxKnots>
        touchdown_events{};
    std::array<double, kTerrainContactMaxKnots> sample_time_s{};
    std::array<TerrainExecutionCommitment, go2::kLegCount>
        committed_targets{};
    std::array<bool, go2::kLegCount> commitment_inherited{};
};

struct TerrainPlanHorizonCoverage
{
    bool valid = false;
    double first_sample_time_s = 0.0;
    double last_sample_time_s = 0.0;
    std::size_t first_knot = 0;
    std::size_t last_knot = 0;
};

inline bool FiniteTerrainVec3(const go2::Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
        std::isfinite(value.z);
}

constexpr double kTerrainExecutionTimeToleranceS = 1.0e-6;
constexpr double kTerrainExecutionTargetToleranceM = 1.0e-5;

// A shadow rejection must identify the first fail-closed contract that
// rejected the immutable consumer snapshot. These values are telemetry
// provenance, not acceptance-policy overrides.
enum class TerrainExecutionShadowFailureReason : std::uint8_t
{
    kNone = 0,
    kPlanInvalid,
    kPlanExpired,
    kHorizonCoverage,
    kCommitmentCoherence,
    kTargetEventMismatch,
    kModelComInvalid,
    kMeasuredContactInvalid,
    kAppliedContactInvalid,
    kOtherSnapshotPrecondition,
};

inline const char *TerrainExecutionShadowFailureReasonName(
    TerrainExecutionShadowFailureReason reason)
{
    switch (reason)
    {
    case TerrainExecutionShadowFailureReason::kPlanInvalid:
        return "plan_invalid";
    case TerrainExecutionShadowFailureReason::kPlanExpired:
        return "plan_expired";
    case TerrainExecutionShadowFailureReason::kHorizonCoverage:
        return "horizon_coverage";
    case TerrainExecutionShadowFailureReason::kCommitmentCoherence:
        return "commitment_coherence";
    case TerrainExecutionShadowFailureReason::kTargetEventMismatch:
        return "target_event_mismatch";
    case TerrainExecutionShadowFailureReason::kModelComInvalid:
        return "model_com_invalid";
    case TerrainExecutionShadowFailureReason::kMeasuredContactInvalid:
        return "measured_contact_invalid";
    case TerrainExecutionShadowFailureReason::kAppliedContactInvalid:
        return "applied_contact_invalid";
    case TerrainExecutionShadowFailureReason::kOtherSnapshotPrecondition:
        return "other_snapshot_precondition";
    default:
        return "none";
    }
}

inline bool TerrainTimeClose(double lhs, double rhs)
{
    return std::isfinite(lhs) && std::isfinite(rhs) &&
        std::abs(lhs - rhs) <= kTerrainExecutionTimeToleranceS;
}

inline bool TerrainTargetClose(
    const go2::Vec3 &lhs, const go2::Vec3 &rhs)
{
    return FiniteTerrainVec3(lhs) && FiniteTerrainVec3(rhs) &&
        std::hypot(std::hypot(lhs.x - rhs.x, lhs.y - rhs.y),
                   lhs.z - rhs.z) <= kTerrainExecutionTargetToleranceM;
}

inline bool TerrainPlanHasTouchdownEvent(
    const TerrainMotionPlan &plan, std::size_t leg, double event_time_s,
    const go2::Vec3 &target_world)
{
    if (leg >= go2::kLegCount || !std::isfinite(event_time_s))
        return false;
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
    {
        const auto &foot = plan.predicted_foothold[k][leg];
        if (!foot.valid || !foot.touchdown ||
            !TerrainTimeClose(foot.touchdown_time_s, event_time_s) ||
            !TerrainTargetClose(foot.position_world, target_world))
            continue;
        const auto lookup = TerrainPlanKnotAtTime(plan, event_time_s);
        if (lookup.valid && lookup.knot == k)
            return true;
    }
    return false;
}

inline bool TerrainPlanCoversMpcHorizon(
    const TerrainMotionPlan &plan, double state_time_s, double mpc_dt_s,
    std::size_t mpc_horizon, TerrainPlanHorizonCoverage *coverage = nullptr)
{
    TerrainPlanHorizonCoverage local;
    local.first_sample_time_s = state_time_s;
    local.last_sample_time_s = state_time_s +
        (mpc_horizon == 0 ? 0.0 : mpc_dt_s *
            static_cast<double>(mpc_horizon - 1));
    if (!plan.valid() || mpc_horizon == 0 ||
        mpc_horizon > kTerrainContactMaxKnots ||
        !std::isfinite(state_time_s) || !std::isfinite(mpc_dt_s) ||
        mpc_dt_s <= 0.0 || !plan.usable_at(state_time_s))
    {
        if (coverage != nullptr)
            *coverage = local;
        return false;
    }
    for (std::size_t k = 0; k < mpc_horizon; ++k)
    {
        const double sample_time_s = state_time_s +
            static_cast<double>(k) * mpc_dt_s;
        if (sample_time_s > plan.valid_until_s +
                kTerrainExecutionTimeToleranceS)
        {
            if (coverage != nullptr)
                *coverage = local;
            return false;
        }
        const auto lookup = TerrainPlanKnotAtTime(plan, sample_time_s);
        if (!lookup.valid)
        {
            if (coverage != nullptr)
                *coverage = local;
            return false;
        }
        if (k == 0)
            local.first_knot = lookup.knot;
        local.last_knot = lookup.knot;
    }
    local.valid = true;
    if (coverage != nullptr)
        *coverage = local;
    return true;
}

inline TerrainTouchdownEvent TerrainPlanNextTouchdown(
    const TerrainMotionPlan &plan, std::size_t leg, double minimum_time_s)
{
    TerrainTouchdownEvent event;
    if (leg >= go2::kLegCount || !std::isfinite(minimum_time_s))
        return event;
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
    {
        const auto &foot = plan.predicted_foothold[k][leg];
        if (!foot.valid || !foot.touchdown ||
            !std::isfinite(foot.touchdown_time_s) ||
            foot.touchdown_time_s <= minimum_time_s +
                kTerrainExecutionTimeToleranceS ||
            !FiniteTerrainVec3(foot.position_world))
            continue;
        const auto lookup = TerrainPlanKnotAtTime(
            plan, foot.touchdown_time_s);
        if (!lookup.valid || lookup.knot != k)
            continue;
        event.valid = true;
        event.knot = k;
        event.plan_id = plan.plan_id;
        event.plan_epoch = plan.plan_epoch;
        event.leg = leg;
        event.event_time_s = foot.touchdown_time_s;
        event.target_world = foot.position_world;
        return event;
    }
    return event;
}

inline bool TerrainExecutionCommitmentsCoherent(
    const TerrainMotionPlan &plan, double state_time_s,
    const std::array<TerrainExecutionCommitment, go2::kLegCount>
        &commitments,
    std::array<bool, go2::kLegCount> *inherited = nullptr,
    TerrainExecutionShadowFailureReason *failure = nullptr,
    std::uint32_t *failure_leg_mask = nullptr)
{
    if (inherited != nullptr)
        inherited->fill(false);
    if (failure != nullptr)
        *failure = TerrainExecutionShadowFailureReason::kNone;
    if (failure_leg_mask != nullptr)
        *failure_leg_mask = 0;
    const auto reject = [&](TerrainExecutionShadowFailureReason reason,
                            std::uint32_t leg_mask = 0u) {
        if (failure != nullptr)
            *failure = reason;
        if (failure_leg_mask != nullptr)
            *failure_leg_mask = leg_mask;
        return false;
    };
    if (!plan.valid())
        return reject(TerrainExecutionShadowFailureReason::kPlanInvalid);
    if (!std::isfinite(state_time_s))
        return reject(
            TerrainExecutionShadowFailureReason::kOtherSnapshotPrecondition);
    if (!plan.usable_at(state_time_s))
        return reject(state_time_s > plan.valid_until_s
                ? TerrainExecutionShadowFailureReason::kPlanExpired
                : TerrainExecutionShadowFailureReason::kPlanInvalid);
    const double last_covered_time_s = plan.state_stamp_s +
        plan.knot_dt_s * static_cast<double>(plan.horizon_knots - 1);
    if (!std::isfinite(last_covered_time_s))
        return reject(
            TerrainExecutionShadowFailureReason::kOtherSnapshotPrecondition);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &commitment = commitments[leg];
        if (!commitment.valid)
            continue;
        if (!std::isfinite(commitment.target_time_s) ||
            commitment.target_time_s > last_covered_time_s +
                kTerrainExecutionTimeToleranceS ||
            commitment.target_time_s > plan.valid_until_s +
                kTerrainExecutionTimeToleranceS ||
            !FiniteTerrainVec3(commitment.target_world))
            return reject(
                TerrainExecutionShadowFailureReason::kTargetEventMismatch,
                1u << leg);
        if (!commitment.in_flight && commitment.measured_touchdown)
        {
            if (commitment.target_time_s > state_time_s +
                    kTerrainExecutionTimeToleranceS)
                return reject(
                    TerrainExecutionShadowFailureReason::kTargetEventMismatch,
                    1u << leg);
            continue;

        }
        if (commitment.target_time_s < state_time_s -
                kTerrainExecutionTimeToleranceS)
            return reject(
                TerrainExecutionShadowFailureReason::kTargetEventMismatch,
                1u << leg);
        if (!TerrainPlanHasTouchdownEvent(
                plan, leg, commitment.target_time_s,
                commitment.target_world))
            return reject(
                TerrainExecutionShadowFailureReason::kTargetEventMismatch,
                1u << leg);
        if (inherited != nullptr &&
            (commitment.source_plan_id != plan.plan_id ||
             commitment.source_plan_epoch != plan.plan_epoch))
            (*inherited)[leg] = true;
    }
    return true;
}
struct TerrainExecutionCommitmentUpdate
{
    bool checked = false;
    bool valid = false;
    std::uint32_t inherited_mask = 0;
    std::uint32_t prepared_mask = 0;
    std::uint32_t rejected_mask = 0;
    TerrainExecutionShadowFailureReason rejection_reason =
        TerrainExecutionShadowFailureReason::kNone;
    std::uint32_t rejection_reason_leg_mask = 0;
    std::array<TerrainExecutionCommitment, go2::kLegCount>
        commitments{};
};

inline TerrainExecutionCommitmentUpdate AdvanceTerrainExecutionCommitments(
    const TerrainMotionPlan &plan, double state_time_s,
    const std::array<bool, go2::kLegCount> &measured_contact,
    bool measured_contact_valid,
    const std::array<TerrainExecutionCommitment, go2::kLegCount>
        &previous)
{
    TerrainExecutionCommitmentUpdate result;
    result.checked = true;
    result.commitments = previous;
    if (!measured_contact_valid)
    {
        result.rejection_reason =
            TerrainExecutionShadowFailureReason::kMeasuredContactInvalid;
        return result;
    }
    if (!plan.valid())
    {
        result.rejection_reason =
            TerrainExecutionShadowFailureReason::kPlanInvalid;
        return result;
    }
    if (!plan.usable_at(state_time_s))
    {
        result.rejection_reason = std::isfinite(state_time_s) &&
                state_time_s > plan.valid_until_s
            ? TerrainExecutionShadowFailureReason::kPlanExpired
            : TerrainExecutionShadowFailureReason::kPlanInvalid;
        return result;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &old = previous[leg];
        bool retained = false;
        if (old.valid && old.in_flight)
        {
            const bool reached = measured_contact[leg] &&
                old.target_time_s <= state_time_s +
                    kTerrainExecutionTimeToleranceS;
            if (!reached)
            {
                if (old.target_time_s < state_time_s -
                        kTerrainExecutionTimeToleranceS ||
                    !TerrainPlanHasTouchdownEvent(
                        plan, leg, old.target_time_s, old.target_world))
                {
                    result.rejected_mask |= 1u << leg;
                    result.rejection_reason =
                        TerrainExecutionShadowFailureReason::kTargetEventMismatch;
                    result.rejection_reason_leg_mask |= 1u << leg;
                    result.commitments[leg] = {};
                    continue;
                }
                retained = true;
                result.commitments[leg] = old;
                if (old.source_plan_id != plan.plan_id ||
                    old.source_plan_epoch != plan.plan_epoch)
                    result.inherited_mask |= 1u << leg;
            }
        }
        if (retained)
            continue;
        double minimum_time_s = state_time_s;
        if (old.valid && std::isfinite(old.target_time_s))
            minimum_time_s = std::max(
                minimum_time_s,
                old.target_time_s + kTerrainExecutionTimeToleranceS);
        const auto next = TerrainPlanNextTouchdown(
            plan, leg, minimum_time_s);
        result.commitments[leg] = {};
        if (!next.valid)
            continue;
        auto &current = result.commitments[leg];
        current.valid = true;
        current.in_flight = true;
        current.source_plan_id = next.plan_id;
        current.source_plan_epoch = next.plan_epoch;
        current.target_time_s = next.event_time_s;
        current.target_world = next.target_world;
        result.prepared_mask |= 1u << leg;
    }
    result.valid = result.rejected_mask == 0;
    return result;
}

inline bool BuildTerrainExecutionSnapshot(
    const TerrainMotionPlan &plan, double state_time_s, double mpc_dt_s,
    std::size_t mpc_horizon, const go2::Vec3 &model_com_world,
    const std::array<bool, go2::kLegCount> &measured_contact,
    bool measured_contact_valid,
    const std::array<bool, go2::kLegCount> &applied_contact,
    bool applied_contact_valid,
    const std::array<TerrainExecutionCommitment, go2::kLegCount>
        &commitments,
    TerrainExecutionSnapshot &snapshot,
    TerrainExecutionShadowFailureReason *failure = nullptr,
    std::uint32_t *failure_leg_mask = nullptr)
{
    if (failure != nullptr)
        *failure = TerrainExecutionShadowFailureReason::kNone;
    if (failure_leg_mask != nullptr)
        *failure_leg_mask = 0;
    snapshot = TerrainExecutionSnapshot{};
    snapshot.state_time_s = state_time_s;
    snapshot.mpc_dt_s = mpc_dt_s;
    snapshot.mpc_horizon = mpc_horizon;
    snapshot.plan_id = plan.plan_id;
    snapshot.plan_epoch = plan.plan_epoch;
    snapshot.model_com_world = model_com_world;
    snapshot.measured_contact = measured_contact;
    snapshot.applied_contact = applied_contact;
    snapshot.measured_contact_valid = measured_contact_valid;
    snapshot.applied_contact_valid = applied_contact_valid;
    snapshot.committed_targets = commitments;
    if (!plan.valid())
    {
        if (failure != nullptr)
            *failure = TerrainExecutionShadowFailureReason::kPlanInvalid;
        return false;
    }
    if (std::isfinite(state_time_s) && state_time_s > plan.valid_until_s)
    {
        if (failure != nullptr)
            *failure = TerrainExecutionShadowFailureReason::kPlanExpired;
        return false;
    }
    if (!FiniteTerrainVec3(model_com_world))
    {
        if (failure != nullptr)
            *failure = TerrainExecutionShadowFailureReason::kModelComInvalid;
        return false;
    }
    if (!measured_contact_valid)
    {
        if (failure != nullptr)
            *failure = TerrainExecutionShadowFailureReason::kMeasuredContactInvalid;
        return false;
    }
    if (!applied_contact_valid)
    {
        if (failure != nullptr)
            *failure = TerrainExecutionShadowFailureReason::kAppliedContactInvalid;
        return false;
    }
    TerrainExecutionShadowFailureReason commitment_failure =
        TerrainExecutionShadowFailureReason::kNone;
    std::uint32_t commitment_failure_leg_mask = 0;
    if (!TerrainExecutionCommitmentsCoherent(
            plan, state_time_s, commitments, &snapshot.commitment_inherited,
            &commitment_failure, &commitment_failure_leg_mask))
    {
        if (failure != nullptr)
            *failure = commitment_failure ==
                    TerrainExecutionShadowFailureReason::kTargetEventMismatch
                ? commitment_failure
                : TerrainExecutionShadowFailureReason::kCommitmentCoherence;
        if (failure_leg_mask != nullptr)
            *failure_leg_mask = commitment_failure_leg_mask;
        return false;
    }
    TerrainPlanHorizonCoverage coverage;
    if (!TerrainPlanCoversMpcHorizon(
            plan, state_time_s, mpc_dt_s, mpc_horizon, &coverage))
    {
        if (failure != nullptr)
            *failure = TerrainExecutionShadowFailureReason::kHorizonCoverage;
        return false;
    }
    for (std::size_t k = 0; k < mpc_horizon; ++k)
    {
        const double sample_time_s = state_time_s +
            static_cast<double>(k) * mpc_dt_s;
        const auto lookup = TerrainPlanKnotAtTime(plan, sample_time_s);
        if (!lookup.valid)
        {
            if (failure != nullptr)
                *failure = TerrainExecutionShadowFailureReason::kHorizonCoverage;
            return false;
        }
        snapshot.sample_time_s[k] = sample_time_s;
        snapshot.planned_contact[k] =
            plan.contact_schedule.planned_contact[lookup.knot];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &foot =
                plan.predicted_foothold[lookup.knot][leg];
            auto &event = snapshot.touchdown_events[k][leg];
            if (foot.valid && foot.touchdown &&
                std::isfinite(foot.touchdown_time_s) &&
                FiniteTerrainVec3(foot.position_world))
            {
                event.valid = true;
                event.knot = lookup.knot;
                event.plan_id = plan.plan_id;
                event.plan_epoch = plan.plan_epoch;
                event.leg = leg;
                event.event_time_s = foot.touchdown_time_s;
                event.target_world = foot.position_world;
            }
        }
    }
    snapshot.horizon_covered = coverage.valid;
    snapshot.valid = true;
    return true;
}

} // namespace go2_terrain
