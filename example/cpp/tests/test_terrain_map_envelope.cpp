#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

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

go2_terrain::TerrainMapEnvelope MakeFlatEnvelope(
    std::uint32_t width = 16, std::uint32_t height = 20)
{
    go2_terrain::TerrainMapEnvelope e;
    e.sequence = 7;
    e.map_stamp_s = 10.0;
    e.frame_id = "base_link";
    e.resolution_m = 0.05;
    e.width = width;
    e.height = height;
    e.origin_m = {-0.40, -0.50};
    e.capture_position_world = {1.0, -2.0, 0.40};
    e.capture_yaw_rad = 0.25;
    const std::size_t count = static_cast<std::size_t>(width) * height;
    e.heights_m.assign(count, -0.40);
    e.observation_stamp_s.assign(count, 10.0);
    return e;
}
} // namespace

int main()
{
    using namespace go2_terrain;
    bool ok = true;

    auto flat = MakeFlatEnvelope();
    std::string wire;
    ok &= Check(SerializeTerrainMapEnvelope(flat, wire),
                "flat envelope did not serialize");
    const auto decoded = DeserializeTerrainMapEnvelope(wire);
    ok &= Check(decoded.ok() && decoded.envelope.heights_m.size() == 320,
                "flat envelope did not round-trip");
    ok &= Check(decoded.envelope.capture_position_world[2] == 0.40,
                "capture z did not round-trip");

    auto malformed = wire;
    const std::string marker = "heights=";
    const std::size_t start = malformed.find(marker);
    const std::size_t end = malformed.find("\n", start);
    malformed.insert(end, 1, ',');
    ok &= Check(!DeserializeTerrainMapEnvelope(malformed).ok(),
                "trailing comma was accepted");

    const std::array<double, 3> current{1.0, -2.0, 0.37};
    const auto registered = RegisterTerrainMap(
        flat, 10.04, current, flat.capture_yaw_rad);
    ok &= Check(registered.ok() && registered.map.registered,
                "flat map did not register");
    std::size_t registered_known = 0;
    for (const float h : registered.map.map.data())
        if (std::isfinite(h))
        {
            ++registered_known;
            ok &= Check(std::abs(static_cast<double>(h) + 0.37) < 1.0e-6,
                        "registered z did not use capture-current delta");
        }
    ok &= Check(registered_known == 320, "identity registration lost cells");

    const auto model = BuildRegisteredTerrainModel(
        &registered.map, 10.04, 11, TerrainSource::kLidar);
    ok &= Check(model.ok() && model.model.registered &&
                    std::abs(model.model.cells[0].age_s - 0.04) < 1.0e-12,
                "BuildTerrainModel lost real cell age");
    const auto mismatch = BuildRegisteredTerrainModel(
        &registered.map, 10.05, 12, TerrainSource::kLidar);
    ok &= Check(!mismatch.ok() &&
                    mismatch.error == TerrainModelError::kStateStampMismatch,
                "registered model accepted a different state stamp");

    auto riser = flat;
    riser.capture_position_world = {0.0, 0.0, 0.0};
    riser.capture_yaw_rad = 0.0;
    riser.origin_m = {-0.40, -0.50};
    riser.heights_m[10 * riser.width + 8] = 0.10;
    const auto rotated = RegisterTerrainMap(
        riser, 10.04, {0.0, 0.0, 0.0}, 0.20);
    ok &= Check(rotated.ok(), "riser map registration rejected valid pose");
    std::size_t rotated_known = 0;
    for (const float h : rotated.map.map.data())
        if (std::isfinite(h))
            ++rotated_known;
    ok &= Check(rotated_known > 0 && rotated_known < registered_known,
                "rotated riser was smoothed or retained across a boundary");

    auto stale_cell = flat;
    stale_cell.observation_stamp_s[0] = 9.70;
    const auto stale = RegisterTerrainMap(
        stale_cell, 10.04, current, flat.capture_yaw_rad);
    ok &= Check(stale.ok() && !std::isfinite(stale.map.map.data()[0]),
                "stale cell was not made unknown");
    auto tiny_resolution = flat;
    tiny_resolution.resolution_m = 1.0e-300;
    const auto tiny_result = RegisterTerrainMap(
        tiny_resolution, 10.04, current, flat.capture_yaw_rad);
    ok &= Check(!tiny_result.ok(),
                "tiny-resolution envelope was accepted");
    auto giant_pose = flat;
    giant_pose.capture_position_world[0] =
        std::numeric_limits<double>::max();
    const auto giant = RegisterTerrainMap(
        giant_pose, 10.04, {0.0, 0.0, 0.37}, giant_pose.capture_yaw_rad);
    std::size_t giant_known = 0;
    if (giant.ok())
        for (const float h : giant.map.map.data())
            giant_known += std::isfinite(h) ? 1U : 0U;
    ok &= Check(giant.ok() && giant_known == 0,
                "giant pose did not fail closed to unknown");

    auto future = flat;
    future.map_stamp_s = 10.10;
    ok &= Check(!RegisterTerrainMap(
                         future, 10.04, current, flat.capture_yaw_rad)
                         .ok(),
                "future map was accepted");

    constexpr int kWarmup = 20;
    constexpr int kRuns = 200;
    volatile std::size_t sink = 0;
    for (int i = 0; i < kWarmup; ++i)
    {
        const auto d = DeserializeTerrainMapEnvelope(wire);
        const auto r = d.ok() ? RegisterTerrainMap(
            d.envelope, 10.04, flat.capture_position_world,
            flat.capture_yaw_rad) : TerrainMapRegistrationResult{};
        if (r.ok())
        {
            const auto b = BuildRegisteredTerrainModel(
                &r.map, 10.04, static_cast<std::uint64_t>(i),
                TerrainSource::kLidar);
            if (b.ok())
                sink += b.model.cells.size();
        }
    }
    std::vector<double> elapsed_us;
    elapsed_us.reserve(kRuns);
    std::size_t last_known = 0;
    for (int i = 0; i < kRuns; ++i)
    {
        const auto begin = std::chrono::steady_clock::now();
        const auto d = DeserializeTerrainMapEnvelope(wire);
        const auto r = d.ok() ? RegisterTerrainMap(
            d.envelope, 10.04, flat.capture_position_world,
            flat.capture_yaw_rad) : TerrainMapRegistrationResult{};
        if (!r.ok())
        {
            ok = false;
            break;
        }
        const auto b = BuildRegisteredTerrainModel(
            &r.map, 10.04, static_cast<std::uint64_t>(i + 100),
            TerrainSource::kLidar);
        if (!b.ok())
        {
            ok = false;
            break;
        }
        last_known = 0;
        for (const auto &cell : b.model.cells)
            last_known += cell.known ? 1U : 0U;
        sink += wire.size() + last_known + b.model.map_sequence;
        const auto end = std::chrono::steady_clock::now();
        elapsed_us.push_back(std::chrono::duration<double, std::micro>(
            end - begin).count());
    }
    ok &= Check(elapsed_us.size() == kRuns && last_known == 320,
                "benchmark did not complete 320 known cells");
    if (!elapsed_us.empty())
    {
        std::sort(elapsed_us.begin(), elapsed_us.end());
        const std::size_t p95 = static_cast<std::size_t>(
            std::ceil(0.95 * elapsed_us.size())) - 1U;
        std::cout << "terrain_map_envelope bytes=" << wire.size()
                  << " known_cells=" << last_known
                  << " decode_register_build_us_p50="
                  << elapsed_us[elapsed_us.size() / 2]
                  << " p95=" << elapsed_us[p95]
                  << " max=" << elapsed_us.back()
                  << " sink=" << sink << "\n";
    }
    std::cout << (ok ? "Terrain map envelope checks passed.\n"
                     : "Terrain map envelope checks failed.\n");
    return ok ? 0 : 1;
}
