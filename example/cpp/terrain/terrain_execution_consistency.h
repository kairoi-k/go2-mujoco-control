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
    double event_time_s = 0.0;
    go2::Vec3 target_world{};
};

struct TerrainExecutionCommitment
{
    bool valid = false;
    bool in_flight = false;
    bool measured_touchdown = false;
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
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
    std::array<std::array<bool, go2::kLegCount>, kTerrainContactMaxKnots>
        touchdown_events{};
    std::array<double, kTerrainContactMaxKnots> sample_time_s{};
    std::array<TerrainExecutionCommitment, go2::kLegCount>
        committed_targets{};
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

inline bool TerrainPlanCoversMpcHorizon(
    const TerrainMotionPlan &plan, double state_time_s, double mpc_dt_s,
    std::size_t mpc_horizon, TerrainPlanHorizonCoverage *coverage = nullptr)
{
    TerrainPlanHorizonCoverage local;
    local.first_sample_time_s = state_time_s;
    local.last_sample_time_s = state_time_s +
        (mpc_horizon == 0 ? 0.0 : mpc_dt_s *
            static_cast<double>(mpc_horizon - 1));
    if (mpc_horizon == 0 || mpc_horizon > kTerrainContactMaxKnots ||
        !std::isfinite(state_time_s) || !std::isfinite(mpc_dt_s) ||
        mpc_dt_s <= 0.0)
    {
        if (coverage != nullptr)
            *coverage = local;
        return false;
    }
    for (std::size_t k = 0; k < mpc_horizon; ++k)
    {
        const auto lookup = TerrainPlanKnotAtTime(
            plan, state_time_s + static_cast<double>(k) * mpc_dt_s);
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
            foot.touchdown_time_s + 1.0e-9 < minimum_time_s ||
            !FiniteTerrainVec3(foot.position_world))
            continue;
        const auto lookup = TerrainPlanKnotAtTime(
            plan, foot.touchdown_time_s);
        if (!lookup.valid || lookup.knot != k)
            continue;
        event.valid = true;
        event.knot = k;
        event.plan_id = plan.plan_id;
        event.event_time_s = foot.touchdown_time_s;
        event.target_world = foot.position_world;
        return event;
    }
    return event;
}

inline bool TerrainExecutionCommitmentsCoherent(
    const TerrainMotionPlan &plan,
    const std::array<TerrainExecutionCommitment, go2::kLegCount>
        &commitments)
{
    const double last_covered_time_s = plan.state_stamp_s +
        plan.knot_dt_s * static_cast<double>(plan.horizon_knots - 1);
    if (!std::isfinite(last_covered_time_s))
        return false;
    for (const auto &commitment : commitments)
    {
        if (!commitment.valid)
            continue;
        if (commitment.plan_id != plan.plan_id ||
            commitment.plan_epoch != plan.plan_epoch ||
            !std::isfinite(commitment.target_time_s) ||
            commitment.target_time_s < plan.state_stamp_s - 1.0e-9 ||
            commitment.target_time_s > last_covered_time_s + 1.0e-9 ||
            !FiniteTerrainVec3(commitment.target_world))
            return false;
    }
    return true;
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
    TerrainExecutionSnapshot &snapshot)
{
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
    if (!plan.valid() || !FiniteTerrainVec3(model_com_world) ||
        !measured_contact_valid || !applied_contact_valid ||
        !TerrainExecutionCommitmentsCoherent(plan, commitments))
        return false;
    TerrainPlanHorizonCoverage coverage;
    if (!TerrainPlanCoversMpcHorizon(
            plan, state_time_s, mpc_dt_s, mpc_horizon, &coverage))
        return false;
    for (std::size_t k = 0; k < mpc_horizon; ++k)
    {
        const double sample_time_s = state_time_s +
            static_cast<double>(k) * mpc_dt_s;
        const auto lookup = TerrainPlanKnotAtTime(plan, sample_time_s);
        if (!lookup.valid)
            return false;
        snapshot.sample_time_s[k] = sample_time_s;
        snapshot.planned_contact[k] =
            plan.contact_schedule.planned_contact[lookup.knot];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            snapshot.touchdown_events[k][leg] =
                plan.predicted_foothold[lookup.knot][leg].touchdown;
    }
    snapshot.horizon_covered = coverage.valid;
    snapshot.valid = true;
    return true;
}

} // namespace go2_terrain
