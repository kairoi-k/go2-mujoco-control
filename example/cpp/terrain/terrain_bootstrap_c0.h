#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "terrain_feasibility.h"
#include "terrain_model.h"
#include "velocity_command.h"

namespace go2_terrain
{

// Development-only readiness bits used by the runtime bootstrap gate. Keep
// this deliberately small: final ownership leases/acks and end-to-end latency
// certification remain separate acceptance work, not prerequisites to the
// first bounded C0/C1 simulation probe.
struct TerrainC0Readiness
{
    bool local_known_flat = false;
    bool swept_volume_clear = false;
    bool dstop_valid = false;

    bool valid() const noexcept
    {
        return local_known_flat && swept_volume_clear && dstop_valid;
    }
};

struct TerrainBootstrapStopEstimate
{
    bool valid = false;
    double distance_m = std::numeric_limits<double>::infinity();
    double time_s = std::numeric_limits<double>::infinity();
    std::size_t steps = 0;
};

// Mirrors the production VelocityCommandShaper's discrete jerk-limited
// braking recurrence, but starts from the live speed/acceleration state. The
// result intentionally excludes sensor/compute/actuation latency; C0 uses it
// only as a development bootstrap certificate, never as final safety proof.
inline TerrainBootstrapStopEstimate EstimateBootstrapStopDistance(
    double speed_mps,
    double acceleration_mps2,
    const go2_trot::VelocityCommandShaperParams &params,
    double dt_s) noexcept
{
    TerrainBootstrapStopEstimate out;
    if (!std::isfinite(speed_mps) || !std::isfinite(acceleration_mps2) ||
        !std::isfinite(dt_s) || speed_mps < 0.0 ||
        params.max_speed_mps <= 0.0 || params.max_decel_mps2 <= 0.0 ||
        params.max_jerk_mps3 <= 0.0)
        return out;

    out.distance_m = 0.0;
    out.time_s = 0.0;
    double speed = std::clamp(speed_mps, 0.0, params.max_speed_mps);
    double accel = std::clamp(
        acceleration_mps2, -params.max_decel_mps2, params.max_accel_mps2);
    const double dt = std::clamp(dt_s, 1.0e-4, 0.050);
    constexpr std::size_t kMaxSteps = 200000;
    for (std::size_t i = 0; i < kMaxSteps; ++i)
    {
        if (speed <= 0.0)
        {
            out.valid = true;
            return out;
        }
        const double desired_accel = -std::max(0.0, params.max_decel_mps2);
        const double max_accel_change =
            std::max(0.0, params.max_jerk_mps3) * dt;
        accel += std::clamp(
            desired_accel - accel, -max_accel_change, max_accel_change);
        speed = std::clamp(
            speed + accel * dt, 0.0, params.max_speed_mps);
        out.distance_m += speed * dt;
        out.time_s = static_cast<double>(i + 1) * dt;
        out.steps = i + 1;
    }
    out.distance_m = std::numeric_limits<double>::infinity();
    out.time_s = std::numeric_limits<double>::infinity();
    out.steps = 0;
    return out;
}

struct TerrainBootstrapC0Input
{
    const TerrainModel *terrain = nullptr;
    TerrainFeasibilityConfig feasibility{};
    std::array<go2::Vec3, go2::kLegCount> current_feet_base{};
    double forward_speed_mps = 0.0;
    double forward_acceleration_mps2 = 0.0;
    double forward_direction_sign = 1.0;
    go2_trot::VelocityCommandShaperParams shaper{};
    double dt_s = 0.002;
};

struct TerrainBootstrapC0Result
{
    TerrainC0Readiness readiness{};
    TerrainBootstrapStopEstimate stop{};
    bool development_only = true;
    bool latency_certified = false;
    std::size_t swept_patch_checks = 0;
    double minimum_map_edge_margin_m =
        std::numeric_limits<double>::infinity();
    std::string reason = "invalid_input";
};

inline bool BootstrapPatchIsKnownFlat(
    const TerrainPatch &patch,
    double reference_height_m,
    const TerrainFeasibilityConfig &config) noexcept
{
    if (!patch.valid || patch.HasUnknownInside() || patch.total_cells == 0)
        return false;
    const std::size_t in_grid = patch.total_cells - patch.outside_cells;
    if (in_grid == 0)
        return false;
    const double known_fraction = static_cast<double>(patch.known_cells) /
        static_cast<double>(in_grid);
    return known_fraction + 1.0e-12 >= config.min_known_fraction &&
        patch.map_edge_margin_m + 1.0e-12 >= config.min_edge_margin_m &&
        std::isfinite(patch.center_height_m) &&
        std::isfinite(patch.min_height_m) &&
        std::isfinite(patch.max_height_m) &&
        std::isfinite(patch.slope_rad) &&
        std::isfinite(patch.roughness_m) &&
        std::isfinite(patch.variance_m2) &&
        patch.slope_rad <= config.max_slope_rad &&
        patch.roughness_m <= config.max_roughness_m &&
        patch.variance_m2 <= config.max_variance_m2 &&
        patch.max_height_m - patch.min_height_m <= config.max_surface_step_m &&
        std::abs(patch.center_height_m - reference_height_m) <=
            config.max_surface_step_m &&
        std::abs(patch.min_height_m - reference_height_m) <=
            config.max_surface_step_m &&
        std::abs(patch.max_height_m - reference_height_m) <=
            config.max_surface_step_m;
}

// Generic C0 development certificate. It advances each currently loaded foot
// corridor by the discrete stop distance and requires every sampled patch to
// remain inside the existing terrain feasibility envelope. No obstacle x,
// step height, leg order, support topology, or relaxed threshold appears here.
inline TerrainBootstrapC0Result EvaluateTerrainBootstrapC0(
    const TerrainBootstrapC0Input &input)
{
    TerrainBootstrapC0Result out;
    const auto *terrain = input.terrain;
    if (terrain == nullptr || !terrain->valid())
    {
        out.reason = "terrain_invalid";
        return out;
    }
    if (terrain->frame_id != input.feasibility.required_frame)
    {
        out.reason = "frame_mismatch";
        return out;
    }
    if (!std::isfinite(terrain->age_s) ||
        terrain->age_s > input.feasibility.max_map_age_s)
    {
        out.reason = "map_stale";
        return out;
    }

    out.stop = EstimateBootstrapStopDistance(
        std::max(0.0, input.forward_speed_mps),
        input.forward_acceleration_mps2, input.shaper, input.dt_s);
    if (!out.stop.valid || !std::isfinite(out.stop.distance_m))
    {
        out.reason = "dstop_invalid";
        return out;
    }
    out.readiness.dstop_valid = true;

    std::array<double, go2::kLegCount> reference_height{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        TerrainPatch patch;
        const auto &foot = input.current_feet_base[leg];
        if (!terrain->CoversPatch(
                foot.x, foot.y, input.feasibility.foot_patch_radius_m))
        {
            out.reason = "local_patch_unavailable";
            return out;
        }
        if (!terrain->SamplePatch(
                foot.x, foot.y, input.feasibility.foot_patch_radius_m, patch) ||
            !patch.valid || !std::isfinite(patch.center_height_m))
        {
            // CoversPatch proved that the footprint lies inside the sensor
            // window. A failed sample therefore means in-grid terrain is
            // unknown/invalid, not that the observation window is absent.
            out.reason = "local_patch_not_known_flat";
            return out;
        }
        reference_height[leg] = patch.center_height_m;
        out.minimum_map_edge_margin_m = std::min(
            out.minimum_map_edge_margin_m, patch.map_edge_margin_m);
        ++out.swept_patch_checks;
        if (!BootstrapPatchIsKnownFlat(
                patch, reference_height[leg], input.feasibility))
        {
            out.reason = "local_patch_not_known_flat";
            return out;
        }
    }
    out.readiness.local_known_flat = true;

    const double spacing = std::max(
        1.0e-4,
        std::min(terrain->resolution_m,
                 std::max(1.0e-4, input.feasibility.foot_patch_radius_m)));
    const std::size_t samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil(out.stop.distance_m / spacing)));
    for (std::size_t sample = 1; sample <= samples; ++sample)
    {
        const double alpha = static_cast<double>(sample) /
            static_cast<double>(samples);
        const double direction = input.forward_direction_sign < 0.0 ? -1.0 : 1.0;
        const double dx = direction * alpha * out.stop.distance_m;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &foot = input.current_feet_base[leg];
            TerrainPatch patch;
            const double x = foot.x + dx;
            if (!terrain->CoversPatch(
                    x, foot.y, input.feasibility.foot_patch_radius_m))
            {
                out.reason = "swept_patch_unavailable";
                return out;
            }
            if (!terrain->SamplePatch(
                    x, foot.y, input.feasibility.foot_patch_radius_m, patch))
            {
                // Entirely unknown cells can make SamplePatch return false.
                // The footprint is still inside the observed window, so this
                // is a fail-closed known-flat failure rather than FOV loss.
                out.reason = "swept_patch_not_known_flat";
                return out;
            }
            out.minimum_map_edge_margin_m = std::min(
                out.minimum_map_edge_margin_m, patch.map_edge_margin_m);
            ++out.swept_patch_checks;
            if (!BootstrapPatchIsKnownFlat(
                    patch, reference_height[leg], input.feasibility))
            {
                out.reason = "swept_patch_not_known_flat";
                return out;
            }
        }
    }

    out.readiness.swept_volume_clear = true;
    out.reason = "c0_development_ready";
    return out;
}

} // namespace go2_terrain
