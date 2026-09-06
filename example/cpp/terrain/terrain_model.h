#pragma once

// Phase 2 local terrain representation.  This header deliberately contains
// no controller policy: a HeightMap becomes a timestamped, provenance-aware
// observation and nothing more.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <unitree/idl/go2/HeightMap_.hpp>

#include "terrain_map_envelope.h"

namespace go2_terrain
{

constexpr double kTerrainNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kTerrainInf = std::numeric_limits<double>::infinity();

enum class TerrainSource : std::uint8_t
{
    kNone = 0,
    kLidar = 1,
    kStateEstimator = 2,
    kTestFixture = 3,
    kOracleRejected = 255,
};

inline const char *TerrainSourceName(TerrainSource source)
{
    switch (source)
    {
    case TerrainSource::kLidar: return "lidar";
    case TerrainSource::kStateEstimator: return "state_estimator";
    case TerrainSource::kTestFixture: return "test_fixture";
    case TerrainSource::kOracleRejected: return "oracle_rejected";
    default: return "none";
    }
}

enum class TerrainModelError : std::uint8_t
{
    kNone = 0,
    kNullMessage,
    kInvalidStamp,
    kInvalidFrame,
    kInvalidResolution,
    kInvalidDimensions,
    kInvalidDataSize,
    kOracleSource,
    kFutureStamp,
    kInvalidCellAge,
    kUnregisteredMap,
    kStateStampMismatch,
    kInvalidHeightBounds,
};

struct TerrainCell
{
    // height_m is the scalar representative used by legacy callers.  V2
    // additionally carries an observed source-cell envelope; it is not a
    // claim about a continuous surface between sensor samples.
    double height_m = kTerrainNaN;
    double height_min_m = kTerrainNaN;
    double height_max_m = kTerrainNaN;
    bool has_height_bounds = false;
    double age_s = kTerrainInf;
    double slope_rad = kTerrainInf;
    double roughness_m = kTerrainInf;
    double variance_m2 = kTerrainInf;
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    bool known = false;
};

struct TerrainPatch
{
    bool valid = false;
    bool all_known = false;
    std::size_t known_cells = 0;
    std::size_t total_cells = 0;
    // Patch cells beyond the grid (sensor FOV edge), as opposed to
    // in-grid cells that were never observed (occlusion holes).
    std::size_t outside_cells = 0;
    double center_height_m = kTerrainNaN;
    double min_height_m = kTerrainNaN;
    double max_height_m = kTerrainNaN;
    double slope_rad = kTerrainInf;
    double roughness_m = kTerrainInf;
    double variance_m2 = kTerrainInf;
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    double map_edge_margin_m = 0.0;

    // True when an in-grid patch cell was never observed.  Out-of-grid
    // (FOV fringe) cells do not count: the window never observes its own
    // fringe at any time, so treating it as fatal unknown makes planning
    // structurally impossible whenever a foot drifts near the edge
    // (epoch17/18 crux starvation).
    bool HasUnknownInside() const
    {
        return total_cells > outside_cells + known_cells;
    }
};

struct TerrainModel
{
    std::string frame_id;
    double state_stamp_s = kTerrainNaN;
    double map_stamp_s = kTerrainNaN;
    double age_s = kTerrainInf;
    std::uint64_t epoch = 0;
    double resolution_m = 0.0;
    std::array<double, 2> origin_m{0.0, 0.0};
    std::size_t width = 0;
    std::size_t height = 0;
    TerrainSource source = TerrainSource::kNone;
    std::vector<TerrainCell> cells;
    bool registered = false;
    std::uint64_t map_sequence = 0;
    std::array<double, 3> capture_position_world{
        kTerrainNaN, kTerrainNaN, kTerrainNaN};
    double capture_yaw_rad = kTerrainNaN;
    std::array<double, 3> registration_position_world{
        kTerrainNaN, kTerrainNaN, kTerrainNaN};
    double registration_yaw_rad = kTerrainNaN;

    bool valid() const
    {
        return !frame_id.empty() && std::isfinite(state_stamp_s) &&
            std::isfinite(map_stamp_s) && std::isfinite(age_s) &&
            resolution_m > 0.0 && width > 0 && height > 0 &&
            cells.size() == width * height && source != TerrainSource::kNone &&
            source != TerrainSource::kOracleRejected;
    }

    bool InBounds(double x_m, double y_m) const
    {
        return valid() && x_m >= origin_m[0] && y_m >= origin_m[1] &&
            x_m < origin_m[0] + static_cast<double>(width) * resolution_m &&
            y_m < origin_m[1] + static_cast<double>(height) * resolution_m;
    }

    // True when a swept patch centered at (x_m, y_m) lies fully inside the
    // grid.  Points failing this have no terrain observation at all, as
    // opposed to observed-but-unknown cells inside the window.
    bool CoversPatch(double x_m, double y_m, double radius_m) const
    {
        return InBounds(x_m, y_m) &&
            x_m - radius_m >= origin_m[0] && y_m - radius_m >= origin_m[1] &&
            x_m + radius_m < origin_m[0] +
                static_cast<double>(width) * resolution_m &&
            y_m + radius_m < origin_m[1] +
                static_cast<double>(height) * resolution_m;
    }

    const TerrainCell *CellAt(std::size_t ix, std::size_t iy) const
    {
        if (ix >= width || iy >= height || cells.size() != width * height)
            return nullptr;
        return &cells[iy * width + ix];
    }

    TerrainCell *CellAt(std::size_t ix, std::size_t iy)
    {
        if (ix >= width || iy >= height || cells.size() != width * height)
            return nullptr;
        return &cells[iy * width + ix];
    }

    bool CellIndex(double x_m, double y_m,
                   std::size_t &ix, std::size_t &iy) const
    {
        if (!InBounds(x_m, y_m))
            return false;
        ix = static_cast<std::size_t>(
            std::floor((x_m - origin_m[0]) / resolution_m));
        iy = static_cast<std::size_t>(
            std::floor((y_m - origin_m[1]) / resolution_m));
        return ix < width && iy < height;
    }

    bool SamplePatch(double x_m, double y_m, double radius_m,
                     TerrainPatch &patch) const
    {
        patch = {};
        patch.total_cells = 0;
        if (!valid() || !std::isfinite(x_m) || !std::isfinite(y_m) ||
            !std::isfinite(radius_m) || radius_m < 0.0)
            return false;

        const double r = std::max(radius_m, 0.5 * resolution_m);
        const double min_x = x_m - r;
        const double max_x = x_m + r;
        const double min_y = y_m - r;
        const double max_y = y_m + r;
        patch.map_edge_margin_m = std::min({
            x_m - origin_m[0],
            y_m - origin_m[1],
            origin_m[0] + static_cast<double>(width) * resolution_m - x_m,
            origin_m[1] + static_cast<double>(height) * resolution_m - y_m});

        const int ix0 = static_cast<int>(std::floor(
            (min_x - origin_m[0]) / resolution_m));
        const int ix1 = static_cast<int>(std::floor(
            (max_x - origin_m[0]) / resolution_m));
        const int iy0 = static_cast<int>(std::floor(
            (min_y - origin_m[1]) / resolution_m));
        const int iy1 = static_cast<int>(std::floor(
            (max_y - origin_m[1]) / resolution_m));

        double sum_height = 0.0;
        double sum_slope = 0.0;
        double sum_roughness = 0.0;
        double sum_variance = 0.0;
        std::array<double, 3> normal_sum{0.0, 0.0, 0.0};
        for (int iy = iy0; iy <= iy1; ++iy)
        {
            for (int ix = ix0; ix <= ix1; ++ix)
            {
                ++patch.total_cells;
                if (ix < 0 || iy < 0 ||
                    ix >= static_cast<int>(width) ||
                    iy >= static_cast<int>(height))
                {
                    ++patch.outside_cells;
                    continue;
                }
                const TerrainCell *cell = CellAt(
                    static_cast<std::size_t>(ix), static_cast<std::size_t>(iy));
                if (cell == nullptr || !cell->known)
                    continue;
                const double cell_min = cell->has_height_bounds
                    ? cell->height_min_m : cell->height_m;
                const double cell_max = cell->has_height_bounds
                    ? cell->height_max_m : cell->height_m;
                if (!std::isfinite(cell_min) || !std::isfinite(cell_max) ||
                    cell_min > cell_max)
                    continue;
                ++patch.known_cells;
                sum_height += cell->height_m;
                sum_slope += cell->slope_rad;
                sum_roughness += cell->roughness_m;
                sum_variance += cell->variance_m2;
                patch.min_height_m = std::isfinite(patch.min_height_m)
                    ? std::min(patch.min_height_m, cell_min)
                    : cell_min;
                patch.max_height_m = std::isfinite(patch.max_height_m)
                    ? std::max(patch.max_height_m, cell_max)
                    : cell_max;
                for (int axis = 0; axis < 3; ++axis)
                    normal_sum[axis] += cell->normal[axis];
            }
        }
        if (patch.known_cells == 0)
            return false;
        const double count = static_cast<double>(patch.known_cells);
        patch.center_height_m = sum_height / count;
        patch.slope_rad = sum_slope / count;
        patch.roughness_m = sum_roughness / count;
        patch.variance_m2 = sum_variance / count;
        const double normal_norm = std::sqrt(
            normal_sum[0] * normal_sum[0] +
            normal_sum[1] * normal_sum[1] +
            normal_sum[2] * normal_sum[2]);
        if (normal_norm > 1.0e-9)
        {
            for (int axis = 0; axis < 3; ++axis)
                patch.normal[axis] = normal_sum[axis] / normal_norm;
        }
        patch.all_known = patch.known_cells == patch.total_cells &&
            patch.map_edge_margin_m >= radius_m;
        patch.valid = std::isfinite(patch.center_height_m) &&
            std::isfinite(patch.slope_rad) &&
            std::isfinite(patch.roughness_m) &&
            std::isfinite(patch.variance_m2);
        return patch.valid;
    }
};

struct TerrainModelBuildResult
{
    TerrainModel model;
    TerrainModelError error = TerrainModelError::kNone;

    bool ok() const { return error == TerrainModelError::kNone && model.valid(); }
};

inline TerrainModelBuildResult BuildTerrainModel(
    const unitree_go::msg::dds_::HeightMap_ *message,
    double state_stamp_s, std::uint64_t epoch, TerrainSource source)
{
    TerrainModelBuildResult result;
    if (message == nullptr)
    {
        result.error = TerrainModelError::kNullMessage;
        return result;
    }
    if (!std::isfinite(state_stamp_s) || !std::isfinite(message->stamp()))
    {
        result.error = TerrainModelError::kInvalidStamp;
        return result;
    }
    if (source == TerrainSource::kOracleRejected)
    {
        result.error = TerrainModelError::kOracleSource;
        return result;
    }
    if (message->frame_id().empty())
    {
        result.error = TerrainModelError::kInvalidFrame;
        return result;
    }
    if (!(message->resolution() > 0.0f) || !std::isfinite(message->resolution()))
    {
        result.error = TerrainModelError::kInvalidResolution;
        return result;
    }
    if (message->width() == 0 || message->height() == 0)
    {
        result.error = TerrainModelError::kInvalidDimensions;
        return result;
    }
    const std::size_t expected = static_cast<std::size_t>(message->width()) *
        static_cast<std::size_t>(message->height());
    if (message->data().size() != expected)
    {
        result.error = TerrainModelError::kInvalidDataSize;
        return result;
    }

    TerrainModel &model = result.model;
    model.frame_id = message->frame_id();
    model.state_stamp_s = state_stamp_s;
    model.map_stamp_s = message->stamp();
    model.age_s = std::max(0.0, state_stamp_s - message->stamp());
    model.epoch = epoch;
    model.resolution_m = message->resolution();
    model.origin_m = {message->origin()[0], message->origin()[1]};
    model.width = message->width();
    model.height = message->height();
    model.source = source;
    model.cells.assign(expected, TerrainCell{});

    for (std::size_t iy = 0; iy < model.height; ++iy)
    {
        for (std::size_t ix = 0; ix < model.width; ++ix)
        {
            TerrainCell &cell = model.cells[iy * model.width + ix];
            const double height = message->data()[iy * model.width + ix];
            cell.age_s = model.age_s;
            if (!std::isfinite(height))
                continue;
            cell.height_m = height;
            cell.known = true;
        }
    }

    // Estimate local plane and roughness from already observed neighbors.
    // Unknown cells are never imputed; a border cell simply has a larger
    // uncertainty and is rejected by the feasibility policy when required.
    const double r = model.resolution_m;
    for (std::size_t iy = 0; iy < model.height; ++iy)
    {
        for (std::size_t ix = 0; ix < model.width; ++ix)
        {
            TerrainCell &cell = model.cells[iy * model.width + ix];
            if (!cell.known)
                continue;
            const TerrainCell *left = ix > 0 ? model.CellAt(ix - 1, iy) : nullptr;
            const TerrainCell *right = ix + 1 < model.width
                ? model.CellAt(ix + 1, iy) : nullptr;
            const TerrainCell *down = iy > 0 ? model.CellAt(ix, iy - 1) : nullptr;
            const TerrainCell *up = iy + 1 < model.height
                ? model.CellAt(ix, iy + 1) : nullptr;
            const bool have_x = left && right && left->known && right->known;
            const bool have_y = down && up && down->known && up->known;
            const double dzdx = have_x ?
                (right->height_m - left->height_m) / (2.0 * r) : 0.0;
            const double dzdy = have_y ?
                (up->height_m - down->height_m) / (2.0 * r) : 0.0;
            const double slope = std::sqrt(dzdx * dzdx + dzdy * dzdy);
            cell.slope_rad = std::atan(slope);
            const double normal_norm = std::sqrt(
                dzdx * dzdx + dzdy * dzdy + 1.0);
            cell.normal = {-dzdx / normal_norm, -dzdy / normal_norm,
                           1.0 / normal_norm};
            double sum = 0.0;
            double sum_sq = 0.0;
            std::size_t count = 0;
            for (int oy = -1; oy <= 1; ++oy)
            {
                for (int ox = -1; ox <= 1; ++ox)
                {
                    const int nx = static_cast<int>(ix) + ox;
                    const int ny = static_cast<int>(iy) + oy;
                    if (nx < 0 || ny < 0 ||
                        nx >= static_cast<int>(model.width) ||
                        ny >= static_cast<int>(model.height))
                        continue;
                    const TerrainCell *neighbor = model.CellAt(
                        static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
                    if (neighbor == nullptr || !neighbor->known)
                        continue;
                    sum += neighbor->height_m;
                    sum_sq += neighbor->height_m * neighbor->height_m;
                    ++count;
                }
            }
            if (count > 0)
            {
                const double mean = sum / static_cast<double>(count);
                cell.variance_m2 = std::max(
                    0.0, sum_sq / static_cast<double>(count) - mean * mean);
                cell.roughness_m = std::sqrt(cell.variance_m2);
            }
        }
    }
    return result;
}

// Re-references base-relative cell heights in z.  The sim lidar publisher
// expresses each cell as world_z - base_z(snapshot); consumers compare
// against feet expressed relative to base_z(now).  During fast base-height
// transients (step-up crux) the stale reference masquerades as terrain
// penetration at the swing anchor.  dz_m = base_z(now) - base_z(snapshot)
// is subtracted from every finite cell; unknown (NaN) cells are preserved.
// On flat ground base_z changes slowly, dz_m ~= 0, behavior unchanged.
inline void RereferenceHeightMapZ(
    unitree_go::msg::dds_::HeightMap_ *message, double dz_m)
{
    if (message == nullptr || !std::isfinite(dz_m) || dz_m == 0.0)
        return;
    for (float &height : message->data())
    {
        if (std::isfinite(height))
            height = static_cast<float>(
                static_cast<double>(height) - dz_m);
    }
}

inline TerrainModelBuildResult BuildRegisteredTerrainModel(
    const RegisteredTerrainMap *registered_map, double state_stamp_s,
    std::uint64_t epoch, TerrainSource source)
{
    TerrainModelBuildResult result;
    if (registered_map == nullptr || !registered_map->registered)
    {
        result.error = TerrainModelError::kUnregisteredMap;
        return result;
    }
    if (!std::isfinite(registered_map->registration_state_stamp_s) ||
        std::abs(registered_map->registration_state_stamp_s -
                 state_stamp_s) > kTerrainMapTimeToleranceS)
    {
        result.error = TerrainModelError::kStateStampMismatch;
        return result;
    }
    if (!std::isfinite(state_stamp_s) ||
        !std::isfinite(registered_map->map_stamp_s) ||
        state_stamp_s - registered_map->map_stamp_s <
            -kTerrainMapTimeToleranceS)
    {
        result.error = TerrainModelError::kFutureStamp;
        return result;
    }
    const std::size_t expected = registered_map->map.data().size();
    const bool intervals_v2 = registered_map->registration_policy ==
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2;
    const bool scalar_v1 = registered_map->registration_policy ==
        TerrainMapRegistrationPolicy::kLegacyScalarV1;
    if (!intervals_v2 && !scalar_v1)
    {
        result.error = TerrainModelError::kInvalidHeightBounds;
        return result;
    }
    if (registered_map->cell_age_s.size() != expected ||
        (intervals_v2 && (registered_map->cell_min_height_m.size() != expected ||
                          registered_map->cell_max_height_m.size() != expected)) ||
        (scalar_v1 && (!registered_map->cell_min_height_m.empty() ||
                       !registered_map->cell_max_height_m.empty())))
    {
        result.error = registered_map->cell_age_s.size() != expected
            ? TerrainModelError::kInvalidCellAge
            : TerrainModelError::kInvalidHeightBounds;
        return result;
    }
    if (intervals_v2)
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            const float scalar = registered_map->map.data()[i];
            const bool have_scalar = std::isfinite(scalar);
            const double low = registered_map->cell_min_height_m[i];
            const double high = registered_map->cell_max_height_m[i];
            const bool have_low = std::isfinite(low);
            const bool have_high = std::isfinite(high);
            if (have_scalar != have_low || have_scalar != have_high ||
                (!have_scalar &&
                 (!std::isnan(scalar) || !std::isnan(low) ||
                  !std::isnan(high))) ||
                (have_scalar &&
                 (low > high || !std::isfinite(static_cast<float>(high)))))
            {
                result.error = TerrainModelError::kInvalidHeightBounds;
                return result;
            }
            if (have_scalar)
            {
                const double scalar_error =
                    std::abs(static_cast<double>(scalar) - high);
                const double scalar_tolerance =
                    2.0 * std::numeric_limits<float>::epsilon() *
                    std::max(1.0, std::abs(high));
                if (!std::isfinite(scalar_error) ||
                    scalar_error > scalar_tolerance)
                {
                    result.error = TerrainModelError::kInvalidHeightBounds;
                    return result;
                }
            }
        }
    }
    for (const double age : registered_map->cell_age_s)
    {
        if (std::isfinite(age) &&
            (age < -kTerrainMapTimeToleranceS ||
             age > kTerrainMapMaxAgeS + kTerrainMapTimeToleranceS))
        {
            result.error = TerrainModelError::kInvalidCellAge;
            return result;
        }
    }
    result = BuildTerrainModel(
        &registered_map->map, state_stamp_s, epoch, source);
    if (!result.ok())
        return result;
    result.model.registered = true;
    result.model.map_sequence = registered_map->sequence;
    result.model.capture_position_world =
        registered_map->capture_position_world;
    result.model.capture_yaw_rad = registered_map->capture_yaw_rad;
    result.model.registration_position_world =
        registered_map->current_position_world;
    result.model.registration_yaw_rad =
        registered_map->current_yaw_rad;
    for (std::size_t i = 0; i < result.model.cells.size(); ++i)
    {
        if (result.model.cells[i].known)
        {
            const double age = registered_map->cell_age_s[i];
            if (!std::isfinite(age))
            {
                result.error = TerrainModelError::kInvalidCellAge;
                result.model = {};
                return result;
            }
            result.model.cells[i].age_s = age;
            if (intervals_v2)
            {
                result.model.cells[i].height_min_m =
                    registered_map->cell_min_height_m[i];
                result.model.cells[i].height_max_m =
                    registered_map->cell_max_height_m[i];
                result.model.cells[i].has_height_bounds = true;
            }
        }
    }
    return result;
}
} // namespace go2_terrain
