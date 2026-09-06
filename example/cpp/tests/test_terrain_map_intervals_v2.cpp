#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include "terrain_feasibility.h"
#include "terrain_map_envelope.h"
#include "terrain_model.h"

namespace
{
bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << "\n";
    return condition;
}

go2_terrain::TerrainMapEnvelope MakeRiser(double low = -0.25,
                                          double high = -0.20)
{
    go2_terrain::TerrainMapEnvelope e;
    e.sequence = 101;
    e.map_stamp_s = 10.0;
    e.frame_id = "base_link";
    e.resolution_m = 0.05;
    e.width = 16;
    e.height = 20;
    e.origin_m = {-0.40, -0.50};
    e.capture_position_world = {0.0, 0.0, 0.0};
    e.capture_yaw_rad = 0.0;
    const std::size_t count = static_cast<std::size_t>(e.width) * e.height;
    e.heights_m.assign(count, low);
    e.observation_stamp_s.assign(count, e.map_stamp_s);
    // A vertical source-grid step at x=.20.  Under a pi/4 re-registration,
    // destination cells near x=.20,y=-.10 overlap both source heights.
    for (std::size_t iy = 0; iy < e.height; ++iy)
        for (std::size_t ix = 12; ix < e.width; ++ix)
            e.heights_m[iy * e.width + ix] = high;
    return e;
}

std::size_t FindMixed(const go2_terrain::RegisteredTerrainMap &map,
                      double target_x, double target_y)
{
    std::size_t best = std::numeric_limits<std::size_t>::max();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < map.cell_min_height_m.size(); ++i)
    {
        if (!std::isfinite(map.cell_min_height_m[i]) ||
            !std::isfinite(map.cell_max_height_m[i]) ||
            map.cell_max_height_m[i] - map.cell_min_height_m[i] <= 0.04)
            continue;
        const std::size_t ix = i % map.map.width();
        const std::size_t iy = i / map.map.width();
        const double x = map.map.origin()[0] +
            (static_cast<double>(ix) + 0.5) * map.map.resolution();
        const double y = map.map.origin()[1] +
            (static_cast<double>(iy) + 0.5) * map.map.resolution();
        const double distance = std::hypot(x - target_x, y - target_y);
        if (distance < best_distance)
        {
            best = i;
            best_distance = distance;
        }
    }
    return best;
}

bool BboxContainsSourceCell(
    const go2_terrain::TerrainMapEnvelope &source,
    const std::array<double, 3> &current, double current_yaw,
    std::size_t destination_index, std::size_t source_index)
{
    const std::size_t dx = destination_index % source.width;
    const std::size_t dy = destination_index / source.width;
    const double x0 = source.origin_m[0] +
        static_cast<double>(dx) * source.resolution_m;
    const double x1 = x0 + source.resolution_m;
    const double y0 = source.origin_m[1] +
        static_cast<double>(dy) * source.resolution_m;
    const double y1 = y0 + source.resolution_m;
    const std::array<go2_terrain::TerrainMapXY, 4> corners{
        go2_terrain::TerrainMapToCaptureLocal(source, current, current_yaw,
                                               {x0, y0}),
        go2_terrain::TerrainMapToCaptureLocal(source, current, current_yaw,
                                               {x1, y0}),
        go2_terrain::TerrainMapToCaptureLocal(source, current, current_yaw,
                                               {x0, y1}),
        go2_terrain::TerrainMapToCaptureLocal(source, current, current_yaw,
                                               {x1, y1})};
    double min_x = corners[0].x, max_x = corners[0].x;
    double min_y = corners[0].y, max_y = corners[0].y;
    for (const auto &corner : corners)
    {
        min_x = std::min(min_x, corner.x);
        max_x = std::max(max_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_y = std::max(max_y, corner.y);
    }
    const std::size_t sx = source_index % source.width;
    const std::size_t sy = source_index / source.width;
    const double sx0 = source.origin_m[0] +
        static_cast<double>(sx) * source.resolution_m;
    const double sy0 = source.origin_m[1] +
        static_cast<double>(sy) * source.resolution_m;
    return max_x > sx0 && min_x < sx0 + source.resolution_m &&
        max_y > sy0 && min_y < sy0 + source.resolution_m;
}
}

int main()
{
    using namespace go2_terrain;
    bool ok = true;
    const auto riser = MakeRiser();
    const std::array<double, 3> current{0.0, 0.0, 0.0};
    constexpr double kYaw = 0.7853981633974483;

    const auto v1 = RegisterTerrainMap(
        riser, 10.04, current, kYaw, kTerrainMapMaxAgeS,
        TerrainMapRegistrationPolicy::kLegacyScalarV1);
    ok &= Check(v1.ok() && v1.map.registered,
                "V1 riser registration failed");
    ok &= Check(v1.map.cell_min_height_m.empty() &&
                    v1.map.cell_max_height_m.empty(),
                "V1 unexpectedly carried interval vectors");

    const auto v2 = RegisterTerrainMap(
        riser, 10.04, current, kYaw, kTerrainMapMaxAgeS,
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    ok &= Check(v2.ok() && v2.map.registered,
                "V2 riser registration failed");
    const std::size_t mixed = FindMixed(v2.map, 0.20, -0.10);
    std::size_t interval_cells = 0;
    std::size_t mixed_count = 0;
    for (std::size_t i = 0; i < v2.map.cell_min_height_m.size(); ++i)
    {
        if (!std::isfinite(v2.map.cell_min_height_m[i]) ||
            !std::isfinite(v2.map.cell_max_height_m[i]))
            continue;
        ++interval_cells;
        if (v2.map.cell_max_height_m[i] - v2.map.cell_min_height_m[i] > 0.04)
            ++mixed_count;
    }
    const std::size_t mixed_ix = mixed == std::numeric_limits<std::size_t>::max()
        ? 0 : mixed % v2.map.map.width();
    const std::size_t mixed_iy = mixed == std::numeric_limits<std::size_t>::max()
        ? 0 : mixed / v2.map.map.width();
    const double mixed_x = v2.map.map.origin()[0] +
        (static_cast<double>(mixed_ix) + 0.5) * v2.map.map.resolution();
    const double mixed_y = v2.map.map.origin()[1] +
        (static_cast<double>(mixed_iy) + 0.5) * v2.map.map.resolution();
    std::cout << "interval_cells=" << interval_cells
              << " mixed_cells=" << mixed_count << " chosen=" << mixed
              << " xy=" << mixed_x << "," << mixed_y << "\n";
    ok &= Check(mixed != std::numeric_limits<std::size_t>::max(),
                "pi/4 riser did not produce an interval cell");
    if (mixed != std::numeric_limits<std::size_t>::max())
    {
        ok &= Check(!std::isfinite(v1.map.map.data()[mixed]),
                    "V1 mixed source cell was not unknown");
        ok &= Check(std::abs(v2.map.cell_min_height_m[mixed] + 0.25) < 1e-9 &&
                        std::abs(v2.map.cell_max_height_m[mixed] + 0.20) < 1e-9 &&
                        std::abs(static_cast<double>(v2.map.map.data()[mixed]) +
                                 0.20) < 1e-6,
                    "V2 interval or hi scalar was wrong");
    }

    const auto built = BuildRegisteredTerrainModel(
        &v2.map, 10.04, 9, TerrainSource::kLidar);
    ok &= Check(built.ok(), "V2 registered model did not build");
    if (built.ok() && mixed != std::numeric_limits<std::size_t>::max())
    {
        const auto &cell = built.model.cells[mixed];
        ok &= Check(cell.known && cell.has_height_bounds &&
                        std::abs(cell.height_min_m + 0.25) < 1e-9 &&
                        std::abs(cell.height_max_m + 0.20) < 1e-9 &&
                        std::abs(cell.height_m + 0.20) < 1e-6,
                    "BuildRegisteredTerrainModel lost V2 bounds");
    }

    TerrainPatch mixed_patch;
    // This reachable location is deliberately on the registered riser band.
    const bool sampled = built.ok() &&
        built.model.SamplePatch(mixed_x, mixed_y, 0.025, mixed_patch);
    ok &= Check(sampled && mixed_patch.valid &&
                    mixed_patch.max_height_m - mixed_patch.min_height_m > 0.04,
                "mixed landing patch did not preserve spread");
    if (built.ok())
    {
        const TerrainFeasibilityConfig config;
        const auto landing = EvaluateFoothold(
            built.model, go2::Leg::FR, mixed_x, mixed_y, config);
        ok &= Check(!landing.hard_feasible &&
                        landing.reject_reason == FootholdRejectReason::kSurfaceStep,
                    "mixed landing was not rejected by surface spread");
    }

    auto top = MakeRiser(-0.20, -0.20);
    const auto top_registered = RegisterTerrainMap(
        top, 10.04, current, kYaw, kTerrainMapMaxAgeS,
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    const auto top_model = BuildRegisteredTerrainModel(
        &top_registered.map, 10.04, 10, TerrainSource::kLidar);
    ok &= Check(top_registered.ok() && top_model.ok(),
                "uniform V2 top did not build");
    if (top_model.ok())
    {
        TerrainPatch top_patch;
        ok &= Check(top_model.model.SamplePatch(
                         0.20, -0.10, 0.025, top_patch) &&
                         std::abs(top_patch.max_height_m -
                                  top_patch.min_height_m) < 1e-9,
                     "uniform top did not collapse interval");
        const auto landing = EvaluateFoothold(
            top_model.model, go2::Leg::FR, 0.20, -0.10,
            TerrainFeasibilityConfig{});
        ok &= Check(landing.hard_feasible,
                    "uniform top foothold was rejected");
    }

    const std::size_t source_probe = 10 * riser.width + 12;
    auto unknown = riser;
    unknown.heights_m[source_probe] = kTerrainMapUnknown;
    unknown.observation_stamp_s[source_probe] = kTerrainMapUnknown;
    const auto unknown_registered = RegisterTerrainMap(
        unknown, 10.04, current, kYaw, kTerrainMapMaxAgeS,
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    const std::size_t no_destination =
        std::numeric_limits<std::size_t>::max();
    std::size_t unknown_destination = no_destination;
    if (unknown_registered.ok())
    {
        for (std::size_t i = 0; i < unknown_registered.map.map.data().size(); ++i)
        {
            if (BboxContainsSourceCell(unknown, current, kYaw, i, source_probe) &&
                std::isfinite(v2.map.map.data()[i]) &&
                !std::isfinite(unknown_registered.map.map.data()[i]))
            {
                unknown_destination = i;
                ok &= Check(!std::isfinite(
                                 unknown_registered.map.cell_min_height_m[i]) &&
                                 !std::isfinite(
                                     unknown_registered.map.cell_max_height_m[i]),
                             "unknown source fabricated V2 bounds");
                break;
            }
        }
    }
    ok &= Check(unknown_destination != no_destination,
                "unknown source did not turn the same baseline cell unknown");

    auto stale_source = riser;
    stale_source.observation_stamp_s[source_probe] = 9.70;
    const auto stale_registered = RegisterTerrainMap(
        stale_source, 10.04, current, kYaw, kTerrainMapMaxAgeS,
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    std::size_t stale_destination = no_destination;
    if (stale_registered.ok())
    {
        for (std::size_t i = 0; i < stale_registered.map.map.data().size(); ++i)
        {
            if (BboxContainsSourceCell(stale_source, current, kYaw, i, source_probe) &&
                std::isfinite(v2.map.map.data()[i]) &&
                !std::isfinite(stale_registered.map.map.data()[i]))
            {
                stale_destination = i;
                break;
            }
        }
    }
    ok &= Check(stale_destination != no_destination,
                "stale source did not turn the same baseline cell unknown");

    auto future_source = riser;
    future_source.map_stamp_s = 10.10;
    future_source.observation_stamp_s[source_probe] = 10.10;
    const auto future_registered = RegisterTerrainMap(
        future_source, 10.04, current, kYaw, kTerrainMapMaxAgeS,
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    ok &= Check(!future_registered.ok() &&
                    future_registered.error == TerrainMapRegistrationError::kFutureMap,
                "future source snapshot was not rejected");

    auto malformed_bounds = v2.map;
    malformed_bounds.cell_min_height_m[mixed] = -0.10;
    malformed_bounds.cell_max_height_m[mixed] = -0.20;
    const auto malformed_model = BuildRegisteredTerrainModel(
        &malformed_bounds, 10.04, 11, TerrainSource::kLidar);
    ok &= Check(!malformed_model.ok() &&
                    malformed_model.error == TerrainModelError::kInvalidHeightBounds,
                "invalid V2 bounds were not fail-closed");

    auto scalar_mismatch = v2.map;
    scalar_mismatch.map.data()[mixed] = -0.19f;
    const auto scalar_mismatch_model = BuildRegisteredTerrainModel(
        &scalar_mismatch, 10.04, 12, TerrainSource::kLidar);
    ok &= Check(!scalar_mismatch_model.ok() &&
                    scalar_mismatch_model.error == TerrainModelError::kInvalidHeightBounds,
                "scalar/high mismatch was not fail-closed");

    if (unknown_destination != no_destination)
    {
        auto half_finite_bounds = unknown_registered.map;
        half_finite_bounds.cell_min_height_m[unknown_destination] = -0.20;
        const auto half_finite_model = BuildRegisteredTerrainModel(
            &half_finite_bounds, 10.04, 13, TerrainSource::kLidar);
        ok &= Check(!half_finite_model.ok() &&
                        half_finite_model.error == TerrainModelError::kInvalidHeightBounds,
                    "half-finite unknown bounds were not fail-closed");
    }

    auto legacy = BuildTerrainModel(
        &v1.map.map, 10.04, 12, TerrainSource::kLidar);
    ok &= Check(legacy.ok(), "legacy scalar map did not build");
    if (legacy.ok())
    {
        TerrainPatch patch;
        ok &= Check(legacy.model.SamplePatch(
                         0.20, -0.10, 0.025, patch) && patch.valid,
                     "legacy scalar patch did not sample");
    }

    std::cout << (ok ? "Terrain map interval V2 checks passed.\n"
                     : "Terrain map interval V2 checks failed.\n");
    return ok ? 0 : 1;
}
