#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "go2_inverse_kinematics.h"
#include "terrain_model.h"

namespace go2_terrain
{

enum class FootholdRejectReason : std::uint8_t
{
    kNone = 0,
    kInvalidModel,
    kFrameMismatch,
    kStale,
    kUnknown,
    kSlope,
    kRoughness,
    kUncertainty,
    kEdge,
    kSurfaceStep,
    kReachability,
    kSwingClearance,
    kCollision,
    kSupport,
};

inline const char *FootholdRejectReasonName(FootholdRejectReason reason)
{
    switch (reason)
    {
    case FootholdRejectReason::kFrameMismatch: return "frame_mismatch";
    case FootholdRejectReason::kStale: return "stale";
    case FootholdRejectReason::kUnknown: return "unknown";
    case FootholdRejectReason::kSlope: return "slope";
    case FootholdRejectReason::kRoughness: return "roughness";
    case FootholdRejectReason::kUncertainty: return "uncertainty";
    case FootholdRejectReason::kEdge: return "edge";
    case FootholdRejectReason::kSurfaceStep: return "surface_step";
    case FootholdRejectReason::kReachability: return "reachability";
    case FootholdRejectReason::kSwingClearance: return "swing_clearance";
    case FootholdRejectReason::kCollision: return "collision";
    case FootholdRejectReason::kSupport: return "support";
    case FootholdRejectReason::kInvalidModel: return "invalid_model";
    default: return "none";
    }
}

struct TerrainFeasibilityConfig
{
    std::string required_frame = "base_link";
    double max_map_age_s = 0.20;
    double max_cell_age_s = 0.25;
    double min_known_fraction = 1.0;
    double foot_patch_radius_m = 0.025;
    double min_edge_margin_m = 0.040;
    double max_slope_rad = 20.0 * 3.14159265358979323846 / 180.0;
    double max_roughness_m = 0.025;
    double max_variance_m2 = 0.000625;
    double max_surface_step_m = 0.040;
    double min_reachability_margin_m = 0.010;
    double min_swing_clearance_m = 0.030;
    double region_half_extent_m = 0.035;
};

struct SafeFootholdRegion
{
    static constexpr std::size_t kMaxVertices = 4;
    go2::Leg leg = go2::Leg::FR;
    std::uint64_t map_epoch = 0;
    std::uint32_t region_id = 0;
    std::array<go2::Vec3, kMaxVertices> vertices{};
    std::size_t vertex_count = 0;
    go2::Vec3 center{};
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    double height_min_m = std::numeric_limits<double>::quiet_NaN();
    double height_max_m = std::numeric_limits<double>::quiet_NaN();
    double slope_rad = std::numeric_limits<double>::infinity();
    double roughness_m = std::numeric_limits<double>::infinity();
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    double swing_clearance_m = std::numeric_limits<double>::infinity();
    double uncertainty_m = std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct FootholdCandidate
{
    go2::Leg leg = go2::Leg::FR;
    go2::Vec3 foot_position{};
    std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
    std::uint64_t map_epoch = 0;
    std::uint32_t region_id = 0;
    double height_min_m = std::numeric_limits<double>::quiet_NaN();
    double height_max_m = std::numeric_limits<double>::quiet_NaN();
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    double swing_clearance_m = std::numeric_limits<double>::infinity();
    double support_margin_m = std::numeric_limits<double>::infinity();
    double collision_margin_m = std::numeric_limits<double>::infinity();
    double uncertainty_m = std::numeric_limits<double>::infinity();
    FootholdRejectReason reject_reason = FootholdRejectReason::kNone;
    bool hard_feasible = false;
};

inline double LegReachabilityMargin(
    go2::Leg leg, const go2::Vec3 &foot_position)
{
    const go2::LegGeometry geometry = go2::Geometry(leg);
    const double x = foot_position.x - geometry.hip_x;
    const double y = foot_position.y - geometry.hip_y;
    const double leg_z_squared = y * y + foot_position.z * foot_position.z -
        geometry.hip_link_y * geometry.hip_link_y;
    if (leg_z_squared < 0.0)
        return -std::numeric_limits<double>::infinity();
    const double leg_z = -std::sqrt(std::max(0.0, leg_z_squared));
    const double radial = std::hypot(x, leg_z);
    const double min_radial = std::abs(
        geometry.thigh_length - geometry.calf_length);
    const double max_radial = geometry.thigh_length + geometry.calf_length;
    return std::min(radial - min_radial, max_radial - radial);
}

inline bool CheckSwingClearance(
    const TerrainModel &model, const go2::Vec3 &start, const go2::Vec3 &end,
    double clearance_m, double &minimum_clearance_m)
{
    minimum_clearance_m = std::numeric_limits<double>::infinity();
    if (!model.valid())
        return false;
    const double distance = std::hypot(end.x - start.x, end.y - start.y);
    const int samples = std::max(2, static_cast<int>(std::ceil(distance /
        std::max(model.resolution_m * 0.5, 0.01))));
    const double lift = std::max(clearance_m, 0.050);
    for (int i = 0; i <= samples; ++i)
    {
        const double u = static_cast<double>(i) / samples;
        const double x = start.x + u * (end.x - start.x);
        const double y = start.y + u * (end.y - start.y);
        const double z = start.z + u * (end.z - start.z) +
            4.0 * u * (1.0 - u) * lift;
        TerrainPatch patch;
        if (!model.SamplePatch(x, y, 0.5 * model.resolution_m, patch) ||
            !patch.valid || !patch.all_known)
        {
            return false;
        }
        const double local_clearance = z - patch.max_height_m;
        minimum_clearance_m = std::min(minimum_clearance_m, local_clearance);
    }
    // Touchdown itself is expected to have zero terrain clearance.  The hard
    // swing gate applies to the interior of the trajectory only.
    if (samples > 1)
    {
        const double u = 1.0 / samples;
        const double z = start.z + u * (end.z - start.z) +
            4.0 * u * (1.0 - u) * lift;
        TerrainPatch patch;
        if (!model.SamplePatch(
                start.x + u * (end.x - start.x),
                start.y + u * (end.y - start.y),
                0.5 * model.resolution_m, patch) || !patch.valid)
            return false;
        minimum_clearance_m = std::min(minimum_clearance_m,
                                       z - patch.max_height_m);
    }
    // Recompute the interior minimum without the two touchdown endpoints.
    minimum_clearance_m = std::numeric_limits<double>::infinity();
    for (int i = 1; i < samples; ++i)
    {
        const double u = static_cast<double>(i) / samples;
        const double x = start.x + u * (end.x - start.x);
        const double y = start.y + u * (end.y - start.y);
        const double z = start.z + u * (end.z - start.z) +
            4.0 * u * (1.0 - u) * lift;
        TerrainPatch patch;
        if (!model.SamplePatch(x, y, 0.5 * model.resolution_m, patch) ||
            !patch.valid || !patch.all_known)
            return false;
        minimum_clearance_m = std::min(
            minimum_clearance_m, z - patch.max_height_m);
    }
    return samples <= 1 || minimum_clearance_m >= clearance_m;
}

inline FootholdCandidate EvaluateFoothold(
    const TerrainModel &model, go2::Leg leg, double x_m, double y_m,
    const TerrainFeasibilityConfig &config,
    const go2::Vec3 *swing_start = nullptr,
    double swing_clearance_m = std::numeric_limits<double>::infinity())
{
    FootholdCandidate candidate;
    candidate.leg = leg;
    candidate.map_epoch = model.epoch;
    if (!model.valid())
    {
        candidate.reject_reason = FootholdRejectReason::kInvalidModel;
        return candidate;
    }
    if (model.frame_id != config.required_frame)
    {
        candidate.reject_reason = FootholdRejectReason::kFrameMismatch;
        return candidate;
    }
    if (model.age_s > config.max_map_age_s)
    {
        candidate.reject_reason = FootholdRejectReason::kStale;
        return candidate;
    }
    TerrainPatch patch;
    if (!model.SamplePatch(x_m, y_m, config.foot_patch_radius_m, patch) ||
        !patch.valid ||
        static_cast<double>(patch.known_cells) /
            std::max<std::size_t>(1, patch.total_cells) <
            config.min_known_fraction)
    {
        candidate.reject_reason = FootholdRejectReason::kUnknown;
        return candidate;
    }
    if (model.age_s > config.max_cell_age_s)
    {
        candidate.reject_reason = FootholdRejectReason::kStale;
        return candidate;
    }
    candidate.foot_position = {x_m, y_m, patch.center_height_m};
    candidate.surface_normal = patch.normal;
    candidate.height_min_m = patch.min_height_m;
    candidate.height_max_m = patch.max_height_m;
    candidate.edge_margin_m = patch.map_edge_margin_m;
    candidate.uncertainty_m = std::sqrt(std::max(0.0, patch.variance_m2));
    candidate.reachability_margin_m = LegReachabilityMargin(
        leg, candidate.foot_position);
    if (candidate.edge_margin_m < config.min_edge_margin_m)
    {
        candidate.reject_reason = FootholdRejectReason::kEdge;
        return candidate;
    }
    if (patch.max_height_m - patch.min_height_m > config.max_surface_step_m)
    {
        candidate.reject_reason = FootholdRejectReason::kSurfaceStep;
        return candidate;
    }
    if (patch.slope_rad > config.max_slope_rad)
    {
        candidate.reject_reason = FootholdRejectReason::kSlope;
        return candidate;
    }
    if (patch.roughness_m > config.max_roughness_m)
    {
        candidate.reject_reason = FootholdRejectReason::kRoughness;
        return candidate;
    }
    if (candidate.uncertainty_m > std::sqrt(config.max_variance_m2))
    {
        candidate.reject_reason = FootholdRejectReason::kUncertainty;
        return candidate;
    }
    go2::LegJointPositions joints;
    if (!go2::LegInverseKinematics(leg, candidate.foot_position, joints) ||
        candidate.reachability_margin_m < config.min_reachability_margin_m)
    {
        candidate.reject_reason = FootholdRejectReason::kReachability;
        return candidate;
    }
    if (swing_start != nullptr && std::isfinite(swing_clearance_m))
    {
        if (!CheckSwingClearance(
                model, *swing_start, candidate.foot_position,
                swing_clearance_m, candidate.swing_clearance_m))
        {
            candidate.reject_reason = FootholdRejectReason::kSwingClearance;
            return candidate;
        }
    }
    candidate.hard_feasible = true;
    candidate.support_margin_m = candidate.edge_margin_m;
    candidate.collision_margin_m = candidate.swing_clearance_m;
    return candidate;
}

inline std::vector<SafeFootholdRegion> BuildSafeFootholdRegions(
    const TerrainModel &model, go2::Leg leg,
    const TerrainFeasibilityConfig &config)
{
    std::vector<SafeFootholdRegion> regions;
    if (!model.valid() || model.frame_id != config.required_frame)
        return regions;
    for (std::size_t iy = 0; iy < model.height; ++iy)
    {
        for (std::size_t ix = 0; ix < model.width; ++ix)
        {
            const TerrainCell *cell = model.CellAt(ix, iy);
            if (cell == nullptr || !cell->known)
                continue;
            const double x = model.origin_m[0] +
                (static_cast<double>(ix) + 0.5) * model.resolution_m;
            const double y = model.origin_m[1] +
                (static_cast<double>(iy) + 0.5) * model.resolution_m;
            const FootholdCandidate candidate = EvaluateFoothold(
                model, leg, x, y, config);
            if (!candidate.hard_feasible)
                continue;
            const double half = std::min(
                config.region_half_extent_m,
                0.5 * model.resolution_m - config.foot_patch_radius_m);
            if (!(half > 0.0))
                continue;
            SafeFootholdRegion region;
            region.leg = leg;
            region.map_epoch = model.epoch;
            region.region_id = static_cast<std::uint32_t>(regions.size());
            region.center = candidate.foot_position;
            region.normal = candidate.surface_normal;
            region.height_min_m = candidate.height_min_m;
            region.height_max_m = candidate.height_max_m;
            region.slope_rad = cell->slope_rad;
            region.roughness_m = cell->roughness_m;
            region.edge_margin_m = candidate.edge_margin_m;
            region.reachability_margin_m = candidate.reachability_margin_m;
            region.uncertainty_m = candidate.uncertainty_m;
            region.vertex_count = 4;
            region.vertices = {
                go2::Vec3{x - half, y - half, candidate.foot_position.z},
                go2::Vec3{x + half, y - half, candidate.foot_position.z},
                go2::Vec3{x + half, y + half, candidate.foot_position.z},
                go2::Vec3{x - half, y + half, candidate.foot_position.z}};
            region.valid = true;
            regions.push_back(region);
        }
    }
    return regions;
}

} // namespace go2_terrain
