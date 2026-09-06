#pragma once
#include "cartesian_world_trot.h"
#include <cmath>

namespace go2_trot
{

struct TerrainCommitmentDecision
{
    // A completed commitment remains valid through stance and is cleared only
    // at the next liftoff before any replacement is prepared.
    bool clear_latched_target = false;
    bool prepare_allowed = false;
    bool apply_swing_target = false;
    bool hold_stance_target = false;
    bool completion = false;
};

// Decide the per-leg commitment transition for one running-trot tick. The
// current terrain plan gates preparation only; an already in-flight target is
// an execution commitment and survives plan expiry until stance completion.
inline bool HoldTerrainWorldHeight(
    const go2::Vec3& base, const std::array<double,4>& quaternion,
    const go2::Vec3& current_body_foot, double target_world_z,
    go2::Vec3& output_body_foot) noexcept
{
    double norm2 = 0;
    for (double v : quaternion) norm2 += v*v;
    if (!std::isfinite(norm2) || std::abs(norm2-1.0) > 0.01 ||
        !std::isfinite(target_world_z) || !std::isfinite(base.x) ||
        !std::isfinite(base.y) || !std::isfinite(base.z) ||
        !std::isfinite(current_body_foot.x) ||
        !std::isfinite(current_body_foot.y) ||
        !std::isfinite(current_body_foot.z)) return false;
    auto q = quaternion;
    for (double& v : q) v /= std::sqrt(norm2);
    auto world = go2_control::BodyToWorld(base, q, current_body_foot);
    // Retain the existing commanded world X/Y, not the target's old X/Y.
    world.z = target_world_z;
    output_body_foot = go2_control::WorldToBody(base, q, world);
    return true;
}

inline TerrainCommitmentDecision DecideTerrainCommitment(
    bool in_swing,
    bool current_plan_usable,
    bool measured_contact,
    bool target_valid,
    bool in_flight,
    bool completion_recorded) noexcept
{
    TerrainCommitmentDecision decision{};
    if (!in_swing)
    {
        if (target_valid)
        {
            decision.hold_stance_target = true;
            decision.completion = measured_contact && !completion_recorded;
        }
        return decision;
    }

    if (in_flight && target_valid)
    {
        decision.apply_swing_target = true;
        return decision;
    }

    // Liftoff starts a new transaction. Clearing happens before preparation,
    // so an absent/invalid plan cannot revive the completed target.
    decision.clear_latched_target = true;
    decision.prepare_allowed = current_plan_usable;
    return decision;
}

}  // namespace go2_trot
