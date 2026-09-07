#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include "terrain_model.h"
namespace go2_terrain
{
namespace stage_c
{
enum class WorldTerrainQueryFailure : std::uint8_t
{
    kNone = 0,
    kMetadata,
    kInvalidQuery,
    kOutside,
    kUnknownCells,
    kStaleCells,
    kNonfinitePatch,
    kConflictingHistory,
};
inline const char *WorldTerrainQueryFailureName(
    WorldTerrainQueryFailure failure)
{
    switch (failure)
    {
    case WorldTerrainQueryFailure::kNone: return "none";
    case WorldTerrainQueryFailure::kMetadata: return "metadata";
    case WorldTerrainQueryFailure::kInvalidQuery: return "invalid_query";
    case WorldTerrainQueryFailure::kOutside: return "outside";
    case WorldTerrainQueryFailure::kUnknownCells: return "unknown_cells";
    case WorldTerrainQueryFailure::kStaleCells: return "stale_cells";
    case WorldTerrainQueryFailure::kNonfinitePatch: return "nonfinite_patch";
    case WorldTerrainQueryFailure::kConflictingHistory:
        return "conflicting_history";
    default: return "unknown";
    }
}
struct WorldTerrainQueryDiagnostic
{
    WorldTerrainQueryFailure failure = WorldTerrainQueryFailure::kNone;
    double world_x = std::numeric_limits<double>::quiet_NaN();
    double world_y = std::numeric_limits<double>::quiet_NaN();
    double local_x = std::numeric_limits<double>::quiet_NaN();
    double local_y = std::numeric_limits<double>::quiet_NaN();
    double radius_m = std::numeric_limits<double>::quiet_NaN();
    double max_cell_age_s = std::numeric_limits<double>::quiet_NaN();
    std::size_t patch_known = 0;
    std::size_t patch_total = 0;
    std::size_t patch_outside = 0;
    double cell_age_max_s = std::numeric_limits<double>::quiet_NaN();
};
namespace world_terrain_view_detail
{
inline bool KnownSource(TerrainSource source)
{
    switch (source)
    {
    case TerrainSource::kLidar:
    case TerrainSource::kStateEstimator:
    case TerrainSource::kTestFixture:
        return true;
    default:
        return false;
    }
}
inline bool FiniteRegistrationPose(const TerrainModel &model)
{
    return std::isfinite(model.registration_position_world[0]) &&
        std::isfinite(model.registration_position_world[1]) &&
        std::isfinite(model.registration_position_world[2]) &&
        std::isfinite(model.registration_yaw_rad);
}
inline bool SafeModelShape(const TerrainModel &model)
{
    if (model.width == 0 || model.height == 0 ||
        model.width > std::numeric_limits<std::size_t>::max() / model.height)
        return false;
    if (model.cells.size() != model.width * model.height)
        return false;
    if (!std::isfinite(model.resolution_m) || model.resolution_m <= 0.0 ||
        !std::isfinite(model.origin_m[0]) ||
        !std::isfinite(model.origin_m[1]))
        return false;
    const double max_x = model.origin_m[0] +
        static_cast<double>(model.width) * model.resolution_m;
    const double max_y = model.origin_m[1] +
        static_cast<double>(model.height) * model.resolution_m;
    return std::isfinite(max_x) && std::isfinite(max_y) &&
        max_x > model.origin_m[0] && max_y > model.origin_m[1];
}
inline bool WorldToLocalXY(
    const TerrainModel &model, double world_x, double world_y,
    double &local_x, double &local_y, double &z_offset)
{
    if (model.frame_id == "world")
    {
        local_x = world_x;
        local_y = world_y;
        z_offset = 0.0;
        return std::isfinite(local_x) && std::isfinite(local_y);
    }
    if (model.frame_id != "base_link" || !model.registered ||
        !FiniteRegistrationPose(model))
        return false;
    const double dx = world_x - model.registration_position_world[0];
    const double dy = world_y - model.registration_position_world[1];
    if (!std::isfinite(dx) || !std::isfinite(dy))
        return false;
    const double c = std::cos(model.registration_yaw_rad);
    const double s = std::sin(model.registration_yaw_rad);
    local_x = c * dx + s * dy;
    local_y = -s * dx + c * dy;
    z_offset = model.registration_position_world[2];
    return std::isfinite(local_x) && std::isfinite(local_y) &&
        std::isfinite(z_offset);
}
enum class PatchCellQueryFailure : std::uint8_t
{
    kNone = 0,
    kUnknownCells,
    kStaleCells,
    kInvalidQuery,
};
inline PatchCellQueryFailure ScanFreshPatchCellsLocal(
    const TerrainModel &model, double local_x, double local_y,
    double radius_m, double max_cell_age_s, double &cell_age_max_s)
{
    cell_age_max_s = std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(radius_m) || radius_m < 0.0 ||
        !std::isfinite(max_cell_age_s) || max_cell_age_s < 0.0)
        return PatchCellQueryFailure::kInvalidQuery;
    const double half_resolution = 0.5 * model.resolution_m;
    const double r = std::max(radius_m, half_resolution);
    if (!std::isfinite(r) || !std::isfinite(local_x - r) ||
        !std::isfinite(local_x + r) || !std::isfinite(local_y - r) ||
        !std::isfinite(local_y + r))
        return PatchCellQueryFailure::kInvalidQuery;
    std::size_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    if (!model.CellIndex(local_x - r, local_y - r, x0, y0) ||
        !model.CellIndex(local_x + r, local_y + r, x1, y1) ||
        x1 < x0 || y1 < y0)
        return PatchCellQueryFailure::kInvalidQuery;
    PatchCellQueryFailure first_failure = PatchCellQueryFailure::kNone;
    for (std::size_t iy = y0; iy <= y1; ++iy)
    {
        for (std::size_t ix = x0; ix <= x1; ++ix)
        {
            const TerrainCell *cell = model.CellAt(ix, iy);
            if (cell == nullptr || !cell->known)
            {
                if (first_failure == PatchCellQueryFailure::kNone)
                    first_failure = PatchCellQueryFailure::kUnknownCells;
                continue;
            }
            if (std::isfinite(cell->age_s))
                cell_age_max_s = std::isfinite(cell_age_max_s)
                    ? std::max(cell_age_max_s, cell->age_s) : cell->age_s;
            if (!std::isfinite(cell->age_s) ||
                cell->age_s < -kTerrainMapTimeToleranceS ||
                cell->age_s > max_cell_age_s + kTerrainMapTimeToleranceS)
            {
                if (first_failure == PatchCellQueryFailure::kNone)
                    first_failure = PatchCellQueryFailure::kStaleCells;
            }
        }
    }
    return first_failure;
}
inline bool FreshPatchCellsLocal(
    const TerrainModel &model, double local_x, double local_y,
    double radius_m, double max_cell_age_s)
{
    double cell_age_max_s = std::numeric_limits<double>::quiet_NaN();
    return ScanFreshPatchCellsLocal(model, local_x, local_y, radius_m,
                                    max_cell_age_s, cell_age_max_s) ==
        PatchCellQueryFailure::kNone;
}
inline bool FinitePatch(const TerrainPatch &patch)
{
    if (!patch.valid || !patch.all_known ||
        !std::isfinite(patch.center_height_m) ||
        !std::isfinite(patch.min_height_m) ||
        !std::isfinite(patch.max_height_m) ||
        patch.min_height_m > patch.max_height_m ||
        !std::isfinite(patch.slope_rad) ||
        !std::isfinite(patch.roughness_m) ||
        !std::isfinite(patch.variance_m2) ||
        !std::isfinite(patch.map_edge_margin_m))
        return false;
    const double normal_norm = std::hypot(
        std::hypot(patch.normal[0], patch.normal[1]), patch.normal[2]);
    return std::isfinite(normal_norm) && normal_norm > 1.0e-9 &&
        std::abs(normal_norm - 1.0) <= 1.0e-6 &&
        std::isfinite(patch.normal[0]) && std::isfinite(patch.normal[1]) &&
        std::isfinite(patch.normal[2]);
}
inline bool AddHeight(double local_height, double z_offset, double &world_height)
{
    world_height = local_height + z_offset;
    return std::isfinite(world_height);
}
inline bool RotateNormal(
    const TerrainModel &model, const std::array<double, 3> &local,
    std::array<double, 3> &world)
{
    if (model.frame_id == "world")
    {
        world = local;
        return true;
    }
    const double c = std::cos(model.registration_yaw_rad);
    const double s = std::sin(model.registration_yaw_rad);
    world = {c * local[0] - s * local[1],
             s * local[0] + c * local[1], local[2]};
    return std::isfinite(world[0]) && std::isfinite(world[1]) &&
        std::isfinite(world[2]);
}
} // namespace world_terrain_view_detail
// Validate the immutable model metadata needed to interpret world queries.
// World-native models use frame_id="world" and need no registration pose.
// Registered heading grids use frame_id="base_link" plus a finite world pose.
inline bool WorldTerrainMetadataValid(const TerrainModel &model)
{
    using namespace world_terrain_view_detail;
    if (!SafeModelShape(model) || !model.valid() || model.epoch == 0 ||
        !KnownSource(model.source) || !std::isfinite(model.state_stamp_s) ||
        !std::isfinite(model.map_stamp_s) || !std::isfinite(model.age_s) ||
        model.age_s < -kTerrainMapTimeToleranceS)
        return false;
    const double expected_age = model.state_stamp_s - model.map_stamp_s;
    if (!std::isfinite(expected_age) ||
        expected_age < -kTerrainMapTimeToleranceS ||
        std::abs(model.age_s - std::max(0.0, expected_age)) >
            kTerrainMapTimeToleranceS)
        return false;
    if (model.frame_id == "world")
        return true;
    return model.frame_id == "base_link" && model.registered &&
        FiniteRegistrationPose(model);
}
// Sample a TerrainModel at a world point without changing the source model.
// For a registered base_link grid, the map is interpreted as a horizontal
// heading frame at registration_position_world. Heights are lifted by the
// registration z and normals are rotated by the registration yaw.
inline bool SampleWorldTerrainPatch(
    const TerrainModel &model, double world_x, double world_y,
    double radius_m, double max_cell_age_s, TerrainPatch &patch,
    WorldTerrainQueryDiagnostic *diagnostic = nullptr)
{
    using namespace world_terrain_view_detail;
    patch = TerrainPatch{};
    if (diagnostic != nullptr)
    {
        *diagnostic = WorldTerrainQueryDiagnostic{};
        diagnostic->world_x = world_x;
        diagnostic->world_y = world_y;
        diagnostic->radius_m = radius_m;
        diagnostic->max_cell_age_s = max_cell_age_s;
    }
    const auto fail = [diagnostic](WorldTerrainQueryFailure failure) {
        if (diagnostic != nullptr)
            diagnostic->failure = failure;
        return false;
    };
    if (!WorldTerrainMetadataValid(model))
        return fail(WorldTerrainQueryFailure::kMetadata);
    if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
        !std::isfinite(radius_m) || radius_m < 0.0 ||
        !std::isfinite(max_cell_age_s) || max_cell_age_s < 0.0)
        return fail(WorldTerrainQueryFailure::kInvalidQuery);
    double local_x = 0.0, local_y = 0.0, z_offset = 0.0;
    if (!WorldToLocalXY(model, world_x, world_y,
                        local_x, local_y, z_offset))
        return fail(WorldTerrainQueryFailure::kInvalidQuery);
    if (diagnostic != nullptr)
    {
        diagnostic->local_x = local_x;
        diagnostic->local_y = local_y;
    }
    const double r = std::max(radius_m, 0.5 * model.resolution_m);
    if (!std::isfinite(r) || !model.CoversPatch(local_x, local_y, r))
        return fail(WorldTerrainQueryFailure::kOutside);
    TerrainPatch local_patch;
    const bool sampled = model.SamplePatch(local_x, local_y, radius_m, local_patch);
    if (diagnostic != nullptr)
    {
        diagnostic->patch_known = local_patch.known_cells;
        diagnostic->patch_total = local_patch.total_cells;
        diagnostic->patch_outside = local_patch.outside_cells;
    }
    double cell_age_max_s = std::numeric_limits<double>::quiet_NaN();
    const auto freshness = ScanFreshPatchCellsLocal(
        model, local_x, local_y, radius_m, max_cell_age_s, cell_age_max_s);
    if (diagnostic != nullptr)
        diagnostic->cell_age_max_s = cell_age_max_s;
    if (!sampled)
    {
        if (local_patch.outside_cells != 0)
            return fail(WorldTerrainQueryFailure::kOutside);
        if (local_patch.total_cells >
            local_patch.known_cells + local_patch.outside_cells ||
            freshness == PatchCellQueryFailure::kUnknownCells)
            return fail(WorldTerrainQueryFailure::kUnknownCells);
        return fail(WorldTerrainQueryFailure::kNonfinitePatch);
    }
    if (local_patch.outside_cells != 0)
        return fail(WorldTerrainQueryFailure::kOutside);
    if (local_patch.total_cells >
        local_patch.known_cells + local_patch.outside_cells ||
        freshness == PatchCellQueryFailure::kUnknownCells)
        return fail(WorldTerrainQueryFailure::kUnknownCells);
    if (!FinitePatch(local_patch))
        return fail(WorldTerrainQueryFailure::kNonfinitePatch);
    if (freshness == PatchCellQueryFailure::kStaleCells)
        return fail(WorldTerrainQueryFailure::kStaleCells);
    if (freshness != PatchCellQueryFailure::kNone)
        return fail(WorldTerrainQueryFailure::kInvalidQuery);
    TerrainPatch world_patch = local_patch;
    if (!AddHeight(local_patch.center_height_m, z_offset,
                   world_patch.center_height_m) ||
        !AddHeight(local_patch.min_height_m, z_offset,
                   world_patch.min_height_m) ||
        !AddHeight(local_patch.max_height_m, z_offset,
                   world_patch.max_height_m) ||
        !RotateNormal(model, local_patch.normal, world_patch.normal) ||
        !FinitePatch(world_patch))
        return fail(WorldTerrainQueryFailure::kNonfinitePatch);
    patch = world_patch;
    if (diagnostic != nullptr)
        diagnostic->failure = WorldTerrainQueryFailure::kNone;
    return true;
}
} // namespace stage_c
} // namespace go2_terrain
