#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "go2_forward_kinematics.h"

namespace go2_terrain
{

using TerrainSwingEaseFunction = double (*)(double);
using TerrainSwingProfileFunction = double (*)(double, double);

enum class TerrainSwingStartSource : std::uint8_t
{
    kMeasured,
    kCommanded,
};

struct TerrainSwingContract
{
    go2::Vec3 start_world{};
    go2::Vec3 target_world{};
    double start_time_s = std::numeric_limits<double>::quiet_NaN();
    double touchdown_time_s = std::numeric_limits<double>::quiet_NaN();
    double resolved_lift_m = std::numeric_limits<double>::quiet_NaN();
    double peak_phase = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t map_epoch = 0;
    std::uint64_t plan_epoch = 0;
    double model_state_stamp_s = std::numeric_limits<double>::quiet_NaN();
    TerrainSwingStartSource start_source =
        TerrainSwingStartSource::kCommanded;
};

inline bool TerrainSwingFiniteVec3(const go2::Vec3 &v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool TerrainSwingPeakValid(double peak_phase) noexcept
{
    return std::isfinite(peak_phase) && peak_phase >= 0.10 &&
        peak_phase <= 0.90;
}

inline bool TerrainSwingScheduleValid(
    const TerrainSwingContract &contract) noexcept
{
    if (!TerrainSwingFiniteVec3(contract.start_world) ||
        !TerrainSwingFiniteVec3(contract.target_world) ||
        !std::isfinite(contract.start_time_s) ||
        !std::isfinite(contract.touchdown_time_s) ||
        !(contract.touchdown_time_s > contract.start_time_s))
        return false;
    const double duration = contract.touchdown_time_s -
        contract.start_time_s;
    return std::isfinite(duration) && duration > 0.0;
}

inline bool TerrainSwingResolved(
    const TerrainSwingContract &contract) noexcept
{
    return TerrainSwingScheduleValid(contract) &&
        std::isfinite(contract.resolved_lift_m) &&
        contract.resolved_lift_m >= 0.0 &&
        TerrainSwingPeakValid(contract.peak_phase);
}

// The plan stores touchdown on a coarse knot grid.  Execution owns the
// current gait clock; accept the coarse event only when it is the nearest
// event for this leg, then resolve the endpoint from this tick's phase.
inline bool ResolveTerrainSwingTouchdownTime(
    double state_time_s, double leg_phase, double period_s,
    double coarse_touchdown_time_s, double &resolved_touchdown_time_s)
{
    resolved_touchdown_time_s =
        std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(state_time_s) || !std::isfinite(leg_phase) ||
        leg_phase < 0.0 || leg_phase >= 1.0 || !std::isfinite(period_s) ||
        period_s <= 0.0 || !std::isfinite(coarse_touchdown_time_s) ||
        coarse_touchdown_time_s <= state_time_s)
        return false;
    const double resolved = state_time_s + (1.0 - leg_phase) * period_s;
    const double tolerance = 0.5 * period_s;
    if (!std::isfinite(resolved) || resolved <= state_time_s ||
        !std::isfinite(tolerance) ||
        std::abs(coarse_touchdown_time_s - resolved) > tolerance)
        return false;
    resolved_touchdown_time_s = resolved;
    return true;
}

inline bool EvaluateTerrainSwingAtTime(
    const TerrainSwingContract &contract, double now_s, go2::Vec3 &out_world,
    TerrainSwingEaseFunction ease, TerrainSwingProfileFunction profile,
    double *normalized_phase = nullptr)
{
    if (!TerrainSwingResolved(contract) || ease == nullptr ||
        profile == nullptr || !std::isfinite(now_s) ||
        now_s < contract.start_time_s || now_s > contract.touchdown_time_s)
        return false;
    const double duration = contract.touchdown_time_s -
        contract.start_time_s;
    const double u = (now_s - contract.start_time_s) / duration;
    if (!std::isfinite(u) || u < 0.0 || u > 1.0)
        return false;
    const double progress = ease(u);
    const double lift = contract.resolved_lift_m *
        profile(u, contract.peak_phase);
    if (!std::isfinite(progress) || !std::isfinite(lift))
        return false;
    out_world = {
        contract.start_world.x + progress *
            (contract.target_world.x - contract.start_world.x),
        contract.start_world.y + progress *
            (contract.target_world.y - contract.start_world.y),
        contract.start_world.z + progress *
            (contract.target_world.z - contract.start_world.z) + lift};
    if (normalized_phase != nullptr)
        *normalized_phase = u;
    return TerrainSwingFiniteVec3(out_world);
}

inline bool RebaseTerrainSwing(
    const TerrainSwingContract &planned, double latch_time_s,
    const go2::Vec3 &actual_start_world, TerrainSwingStartSource source,
    TerrainSwingContract &execution)
{
    if (!TerrainSwingScheduleValid(planned) ||
        !TerrainSwingFiniteVec3(actual_start_world) ||
        !std::isfinite(latch_time_s) || latch_time_s < planned.start_time_s ||
        latch_time_s >= planned.touchdown_time_s)
        return false;
    execution = planned;
    execution.start_world = actual_start_world;
    execution.start_time_s = latch_time_s;
    execution.resolved_lift_m = std::numeric_limits<double>::quiet_NaN();
    execution.peak_phase = std::numeric_limits<double>::quiet_NaN();
    execution.start_source = source;
    return true;
}

// This copies clearance output only. It is deliberately not an IK or
// dynamics certificate.
inline bool ApplyTerrainSwingClearanceResult(
    TerrainSwingContract &contract, double lift_m, double peak_phase) noexcept
{
    if (!TerrainSwingScheduleValid(contract) || !std::isfinite(lift_m) ||
        lift_m < 0.0 || !TerrainSwingPeakValid(peak_phase))
        return false;
    contract.resolved_lift_m = lift_m;
    contract.peak_phase = peak_phase;
    return true;
}

struct TerrainSwingFrameAdapter
{
    go2::Vec3 base_world{};
    std::array<double, 4> q_world_from_body{};
    double yaw_rad = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t map_epoch = 0;
    double model_state_stamp_s = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;

    bool Bind(const go2::Vec3 &base, const std::array<double, 4> &quaternion,
              double yaw)
    {
        return Bind(base, quaternion, yaw, 0,
                    std::numeric_limits<double>::quiet_NaN());
    }

    bool Bind(const go2::Vec3 &base, const std::array<double, 4> &quaternion,
              double yaw, std::uint64_t epoch, double state_stamp_s)
    {
        valid = false;
        if (!TerrainSwingFiniteVec3(base) || !std::isfinite(yaw))
            return false;
        double norm_squared = 0.0;
        for (double value : quaternion)
        {
            if (!std::isfinite(value))
                return false;
            norm_squared += value * value;
        }
        if (!(norm_squared > 1.0e-12) || !std::isfinite(norm_squared))
            return false;
        base_world = base;
        q_world_from_body = quaternion;
        yaw_rad = yaw;
        map_epoch = epoch;
        model_state_stamp_s = state_stamp_s;
        valid = true;
        return true;
    }

    bool WorldToBody(const go2::Vec3 &world, go2::Vec3 &body) const
    {
        if (!valid || !TerrainSwingFiniteVec3(world))
            return false;
        const go2::Vec3 relative{world.x - base_world.x,
                                 world.y - base_world.y,
                                 world.z - base_world.z};
        return RotateQuaternion(relative, body, true);
    }

    bool BodyToWorld(const go2::Vec3 &body, go2::Vec3 &world) const
    {
        if (!valid || !TerrainSwingFiniteVec3(body))
            return false;
        go2::Vec3 rotated{};
        if (!RotateQuaternion(body, rotated, false))
            return false;
        world = {base_world.x + rotated.x, base_world.y + rotated.y,
                 base_world.z + rotated.z};
        return TerrainSwingFiniteVec3(world);
    }

    bool WorldToHeading(const go2::Vec3 &world, go2::Vec3 &heading) const
    {
        if (!valid || !TerrainSwingFiniteVec3(world))
            return false;
        const double dx = world.x - base_world.x;
        const double dy = world.y - base_world.y;
        const double c = std::cos(yaw_rad);
        const double s = std::sin(yaw_rad);
        heading = {c * dx + s * dy, -s * dx + c * dy,
                   world.z - base_world.z};
        return TerrainSwingFiniteVec3(heading);
    }

    bool HeadingToWorld(const go2::Vec3 &heading, go2::Vec3 &world) const
    {
        if (!valid || !TerrainSwingFiniteVec3(heading))
            return false;
        const double c = std::cos(yaw_rad);
        const double s = std::sin(yaw_rad);
        world = {base_world.x + c * heading.x - s * heading.y,
                 base_world.y + s * heading.x + c * heading.y,
                 base_world.z + heading.z};
        return TerrainSwingFiniteVec3(world);
    }

    bool HeadingToBody(const go2::Vec3 &heading, go2::Vec3 &body) const
    {
        go2::Vec3 world{};
        return HeadingToWorld(heading, world) && WorldToBody(world, body);
    }

    bool BodyToHeading(const go2::Vec3 &body, go2::Vec3 &heading) const
    {
        go2::Vec3 world{};
        return BodyToWorld(body, world) && WorldToHeading(world, heading);
    }

private:
    bool RotateQuaternion(const go2::Vec3 &input, go2::Vec3 &output,
                          bool inverse) const
    {
        double norm_squared = 0.0;
        for (double value : q_world_from_body)
            norm_squared += value * value;
        if (!(norm_squared > 1.0e-12) || !std::isfinite(norm_squared))
            return false;
        const double scale = 1.0 / std::sqrt(norm_squared);
        const double w = q_world_from_body[0] * scale;
        const double x = (inverse ? -1.0 : 1.0) *
            q_world_from_body[1] * scale;
        const double y = (inverse ? -1.0 : 1.0) *
            q_world_from_body[2] * scale;
        const double z = (inverse ? -1.0 : 1.0) *
            q_world_from_body[3] * scale;
        output = {
            (1.0 - 2.0 * (y * y + z * z)) * input.x +
                2.0 * (x * y - z * w) * input.y +
                2.0 * (x * z + y * w) * input.z,
            2.0 * (x * y + z * w) * input.x +
                (1.0 - 2.0 * (x * x + z * z)) * input.y +
                2.0 * (y * z - x * w) * input.z,
            2.0 * (x * z - y * w) * input.x +
                2.0 * (y * z + x * w) * input.y +
                (1.0 - 2.0 * (x * x + y * y)) * input.z};
        return TerrainSwingFiniteVec3(output);
    }
};

}  // namespace go2_terrain
