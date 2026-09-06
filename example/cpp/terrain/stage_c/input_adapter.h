#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "types.h"

namespace go2_terrain
{
namespace stage_c
{

struct RawPlanningObservation
{
    CaptureMode capture_mode = CaptureMode::kShadow;
    PlanningIdentity identity{};
    BodyObservation body{};
    std::array<FootObservation, go2::kLegCount> feet{};
    ContactEvidence measured_contact{};
    MapObservation map{};
    Phase1CommandAuthority command{};
    PlanningBudget budget{};
    double initial_support_margin_m = std::numeric_limits<double>::quiet_NaN();
    bool initial_support_margin_valid = false;
};

struct InputAdapterResult
{
    TerrainPlanningInput input{};
    JointPlannerFailure failure = JointPlannerFailure::kNone;
    std::size_t missing_anchor_count = 0;
    bool ok = false;
};

// The capture mode is diagnostic provenance only. It is deliberately not an
// input to normalization: shadow and actuation must consume the same measured
// support evidence when the raw state/force/FK snapshot is the same.
inline InputAdapterResult NormalizePlanningInput(
    const RawPlanningObservation &raw)
{
    InputAdapterResult result;
    result.input.identity = raw.identity;
    result.input.body = raw.body;
    result.input.feet = raw.feet;
    result.input.measured_contact = raw.measured_contact;
    result.input.map = raw.map;
    result.input.map.coverage = ClassifyMapCoverage(raw.map);
    result.input.command = raw.command;
    result.input.budget = raw.budget;
    result.input.initial_support_margin_m = raw.initial_support_margin_m;
    result.input.initial_support_margin_valid = raw.initial_support_margin_valid;

    if (!result.input.identity.valid() || !result.input.body.valid ||
        !result.input.body.base_position_world.valid ||
        result.input.body.base_position_world.frame != Frame::kWorld ||
        !result.input.measured_contact.valid ||
        result.input.measured_contact.provenance !=
            ContactProvenance::kMeasured || !result.input.map.metadata_valid)
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }

    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        auto &foot = result.input.feet[leg];
        if (!result.input.measured_contact.mask[leg])
            continue;
        if (!foot.measured_support_anchor_valid ||
            !foot.measured_support_anchor_world.valid ||
            foot.measured_support_anchor_world.frame != Frame::kWorld)
        {
            ++result.missing_anchor_count;
            continue;
        }
        // Keep this as a measured field. It must never become a planned or
        // applied contact just because a future event is being evaluated.
        foot.contact_patch_world = foot.measured_support_anchor_world;
    }
    if (result.missing_anchor_count != 0)
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }
    result.ok = true;
    return result;
}

inline bool SameVec3(const go2::Vec3 &a, const go2::Vec3 &b,
                     double tolerance = 1.0e-12)
{
    return std::abs(a.x - b.x) <= tolerance &&
        std::abs(a.y - b.y) <= tolerance &&
        std::abs(a.z - b.z) <= tolerance;
}

// This compares planner-semantic fields and intentionally ignores capture
// mode. It is a small replay seam, not a claim that the legacy producer has
// already been wired through this adapter.
inline bool EquivalentPlannerInput(const TerrainPlanningInput &a,
                                   const TerrainPlanningInput &b)
{
    if (a.identity.source_state_tick != b.identity.source_state_tick ||
        a.identity.source_state_time != b.identity.source_state_time ||
        a.identity.map_epoch != b.identity.map_epoch ||
        a.identity.schedule_epoch != b.identity.schedule_epoch ||
        a.measured_contact.mask != b.measured_contact.mask ||
        a.measured_contact.valid != b.measured_contact.valid ||
        a.map.coverage != b.map.coverage ||
        a.map.known_cells != b.map.known_cells ||
        a.map.total_cells != b.map.total_cells ||
        a.map.outside_cells != b.map.outside_cells)
        return false;
    if (!SameVec3(a.body.base_position_world.value,
                  b.body.base_position_world.value) ||
        !SameVec3(a.body.model_com_world.value,
                  b.body.model_com_world.value))
        return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (a.feet[leg].measured_support_anchor_valid !=
                b.feet[leg].measured_support_anchor_valid ||
            !SameVec3(a.feet[leg].measured_support_anchor_world.value,
                      b.feet[leg].measured_support_anchor_world.value))
            return false;
    }
    return true;
}

inline go2::Vec3 RotateHeadingMapToWorld(
    const go2::Vec3 &base_world, double yaw_rad, const go2::Vec3 &map_point)
{
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    return {base_world.x + c * map_point.x - s * map_point.y,
            base_world.y + s * map_point.x + c * map_point.y,
            base_world.z + map_point.z};
}

// World FK uses full body orientation. The old terrain helper is yaw-only and
// remains a legacy path; this function is the explicit Stage C frame seam.
inline go2::Vec3 RotateBodyToWorld(
    const go2::Vec3 &base_world, double roll_rad, double pitch_rad,
    double yaw_rad, const go2::Vec3 &body_point)
{
    const double sr = std::sin(roll_rad);
    const double cr = std::cos(roll_rad);
    const double sp = std::sin(pitch_rad);
    const double cp = std::cos(pitch_rad);
    const double sy = std::sin(yaw_rad);
    const double cy = std::cos(yaw_rad);
    const double r00 = cy * cp;
    const double r01 = cy * sp * sr - sy * cr;
    const double r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp;
    const double r11 = sy * sp * sr + cy * cr;
    const double r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp;
    const double r21 = cp * sr;
    const double r22 = cp * cr;
    return {base_world.x + r00 * body_point.x + r01 * body_point.y +
                r02 * body_point.z,
            base_world.y + r10 * body_point.x + r11 * body_point.y +
                r12 * body_point.z,
            base_world.z + r20 * body_point.x + r21 * body_point.y +
                r22 * body_point.z};
}

} // namespace stage_c
} // namespace go2_terrain
