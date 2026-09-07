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
    const TimeNs state_time = result.input.identity.source_state_time;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (result.input.measured_contact.mask[leg] &&
            (!result.input.feet[leg].measured_support_anchor_valid ||
             !TimedPointValidAt(
                 result.input.feet[leg].measured_support_anchor_world,
                 PointRole::kSurfaceContactPoint, Frame::kWorld, state_time)))
            ++result.missing_anchor_count;
    if (result.missing_anchor_count != 0 || !result.input.basic_valid())
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }
    const auto optional_point_valid_at = [state_time](
        const TimedPoint &point, PointRole role, Frame frame) {
        return !point.valid || TimedPointValidAt(point, role, frame, state_time);
    };
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &foot = result.input.feet[leg];
        if (!optional_point_valid_at(foot.foot_site_world,
                                     PointRole::kFootSite, Frame::kWorld) ||
            !optional_point_valid_at(foot.foot_collision_center_world,
                                     PointRole::kFootCollisionCenter,
                                     Frame::kWorld) ||
            !optional_point_valid_at(foot.contact_patch_world,
                                     PointRole::kSurfaceContactPoint,
                                     Frame::kWorld) ||
            !optional_point_valid_at(foot.contact_patch_base,
                                     PointRole::kSurfaceContactPoint,
                                     Frame::kBase))
        {
            result.failure = JointPlannerFailure::kObservationUnavailable;
            return result;
        }
        if (!result.input.measured_contact.mask[leg])
            continue;
        // A measured support anchor is already a terrain point. A foot site
        // or collision center is a different point role and must be converted
        // by a geometry/terrain producer before it reaches this seam.
        // Keep this as a measured field. It must never become a planned or
        // applied contact just because a future event is being evaluated.
        result.input.feet[leg].contact_patch_world =
            foot.measured_support_anchor_world;
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
inline bool SameTimedPoint(const TimedPoint &a, const TimedPoint &b)
{
    return a.valid == b.valid && a.frame == b.frame && a.role == b.role &&
        a.source_time == b.source_time &&
        (!a.valid || SameVec3(a.value, b.value));
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
        a.measured_contact.provenance != b.measured_contact.provenance ||
        a.measured_contact.source_time != b.measured_contact.source_time ||
        a.measured_contact.valid != b.measured_contact.valid ||
        a.map.coverage != b.map.coverage ||
        a.map.known_cells != b.map.known_cells ||
        a.map.total_cells != b.map.total_cells ||
        a.map.outside_cells != b.map.outside_cells ||
        a.body.valid != b.body.valid ||
        a.body.model_com_valid != b.body.model_com_valid)
        return false;
    if (!SameTimedPoint(a.body.base_position_world,
                        b.body.base_position_world) ||
        !SameTimedPoint(a.body.model_com_world,
                        b.body.model_com_world))
        return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &left = a.feet[leg];
        const auto &right = b.feet[leg];
        if (!SameTimedPoint(left.foot_site_world, right.foot_site_world) ||
            !SameTimedPoint(left.foot_collision_center_world,
                            right.foot_collision_center_world) ||
            !SameTimedPoint(left.contact_patch_world,
                            right.contact_patch_world) ||
            !SameTimedPoint(left.contact_patch_base,
                            right.contact_patch_base) ||
            left.measured_support_anchor_valid !=
                right.measured_support_anchor_valid ||
            !SameTimedPoint(left.measured_support_anchor_world,
                            right.measured_support_anchor_world))
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
