#include "stage_c/world_terrain_snapshot.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace go2_terrain;
using namespace go2_terrain::stage_c;
namespace
{
void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
constexpr std::size_t kWidth = 20;
constexpr std::size_t kHeight = 20;
constexpr double kResolution = 0.10;
constexpr double kOrigin = -1.0;
CaptureTerrainViewResult MakeView(
    std::uint64_t sequence, double map_stamp_s, double state_stamp_s,
    double capture_z, double capture_yaw, double world_height,
    double local_slope_x = 0.0, std::size_t unknown_index = kWidth * kHeight,
    double capture_x = 0.0, double capture_y = 0.0)
{
    TerrainMapEnvelope source;
    source.sequence = sequence;
    source.map_stamp_s = map_stamp_s;
    source.frame_id = "base_link";
    source.resolution_m = kResolution;
    source.width = static_cast<std::uint32_t>(kWidth);
    source.height = static_cast<std::uint32_t>(kHeight);
    source.origin_m = {kOrigin, kOrigin};
    source.capture_position_world = {capture_x, capture_y, capture_z};
    source.capture_yaw_rad = capture_yaw;
    const std::size_t count = kWidth * kHeight;
    source.heights_m.resize(count);
    source.observation_stamp_s.assign(count, map_stamp_s);
    for (std::size_t iy = 0; iy < kHeight; ++iy)
    {
        for (std::size_t ix = 0; ix < kWidth; ++ix)
        {
            const double local_x = kOrigin +
                (static_cast<double>(ix) + 0.5) * kResolution;
            source.heights_m[iy * kWidth + ix] = world_height - capture_z +
                local_slope_x * local_x;
        }
    }
    if (unknown_index < count)
    {
        source.heights_m[unknown_index] = kTerrainMapUnknown;
        source.observation_stamp_s[unknown_index] = kTerrainMapUnknown;
    }
    return BuildCaptureHeadingTerrainView(
        source, state_stamp_s, 7, TerrainSource::kTestFixture, 0.20);
}
WorldTerrainSnapshotOptions StationaryOptions()
{
    WorldTerrainSnapshotOptions options;
    options.max_captures = kWorldTerrainSnapshotMaxCaptures;
    options.stationary_terrain_assumption = true;
    return options;
}

bool SameModelCells(const TerrainModel &left, const TerrainModel &right)
{
    if (left.cells.size() != right.cells.size())
        return false;
    for (std::size_t i = 0; i < left.cells.size(); ++i)
    {
        const TerrainCell &a = left.cells[i];
        const TerrainCell &b = right.cells[i];
        if (a.height_m != b.height_m || a.height_min_m != b.height_min_m ||
            a.height_max_m != b.height_max_m ||
            a.has_height_bounds != b.has_height_bounds || a.age_s != b.age_s ||
            a.slope_rad != b.slope_rad || a.roughness_m != b.roughness_m ||
            a.variance_m2 != b.variance_m2 || a.normal != b.normal ||
            a.known != b.known)
            return false;
    }
    return true;
}

TerrainMapXY QueryWorld(const TerrainModel &model)
{
    return TerrainMapToWorld(
        model.registration_position_world, model.registration_yaw_rad,
        {0.05, 0.05});
}
} // namespace
int main()
{
    try
    {
        const auto history = MakeView(11, 9.98, 10.10, 0.30, 0.35, 0.80);
        const auto latest = MakeView(12, 10.05, 10.10, 0.55, -0.40, 0.80);
        Check(history.ok() && latest.ok(), "capture fixtures were not valid");
        const TerrainModel history_before = history.model;
        const auto query_xy = QueryWorld(latest.model);
        const auto missing_assumption = BuildWorldTerrainSnapshot(
            7, 10.10, std::vector<CaptureTerrainViewResult>{latest}, {});
        Check(!missing_assumption.ok() &&
                  missing_assumption.error ==
                      WorldTerrainSnapshotError::kStationaryAssumptionRequired,
              "historical assumption was implicit");
        const auto built = BuildWorldTerrainSnapshot(
            7, 10.10,
            std::vector<CaptureTerrainViewResult>{history, latest},
            StationaryOptions());
        Check(built.ok() && built.snapshot.captures.size() == 2 &&
                  built.snapshot.metadata.height_conflict_tolerance_m == 1.0e-3 &&
                  built.snapshot.metadata.minimum_normal_dot == 0.9998 &&
                  built.snapshot.latest_model() != nullptr &&
                  built.snapshot.latest_model()->map_sequence == 12 &&
                  built.snapshot.captures[0].model.map_sequence == 12 &&
                  built.snapshot.captures[1].model.map_sequence == 11,
              "snapshot was not bounded and source ordered");
        Check(SameModelCells(history.model, history_before),
              "fixture comparison changed source model");
        const auto latest_result = SampleWorldTerrainSnapshot(
            built.snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(latest_result.ok() && !latest_result.history_used &&
                  latest_result.selected_source_sequence == 12 &&
                  latest_result.selected_source_time_s == 10.05 &&
                  std::abs(latest_result.patch.center_height_m - 0.80) < 2.0e-6 &&
                  latest_result.selected_source_time == TimeNs::FromSeconds(10.05) &&
                  latest_result.diagnostic.failure == WorldTerrainQueryFailure::kNone,
              "latest complete capture was not preferred");
        Check(SameModelCells(history.model, history_before),
              "query mutated historical model");
        const std::size_t hole_index = 10 * kWidth + 10;
        const auto latest_partial = MakeView(
            12, 10.05, 10.10, 0.55, -0.40, 0.80, 0.0, hole_index);
        const auto fallback_snapshot = BuildWorldTerrainSnapshot(
            7, 10.10,
            std::vector<CaptureTerrainViewResult>{history, latest_partial},
            StationaryOptions());
        Check(fallback_snapshot.ok(), "partial capture snapshot rejected");
        const auto fallback = SampleWorldTerrainSnapshot(
            fallback_snapshot.snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(fallback.ok() && fallback.history_used &&
                  fallback.selected_source_sequence == 11 &&
                  fallback.selected_source_time_s == 9.98 &&
                  fallback.diagnostic.failure == WorldTerrainQueryFailure::kNone &&
                  fallback.latest_failure == WorldTerrainQueryFailure::kUnknownCells,
              "fresh complete history was not selected for latest hole");
        const auto raised_partial = MakeView(
            12, 10.05, 10.10, 0.55, -0.40, 1.00, 0.0, hole_index);
        const auto raised_snapshot = BuildWorldTerrainSnapshot(
            7, 10.10,
            std::vector<CaptureTerrainViewResult>{history, raised_partial},
            StationaryOptions());
        Check(raised_snapshot.ok(), "raised partial snapshot rejected");
        const auto raised = SampleWorldTerrainSnapshot(
            raised_snapshot.snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(!raised.ok() &&
                  raised.error == WorldTerrainSnapshotError::kConflictingHistory &&
                  raised.conflict_height_gap_m >= 0.0049 &&
                  raised.diagnostic.failure ==
                      WorldTerrainQueryFailure::kConflictingHistory &&
                  raised.conflict_normal_dot > 0.999,
              "partial newer height conflict was hidden by history");
        const auto edge_old = MakeView(
            21, 10.00, 10.10, 0.30, 0.0, 0.80);
        const auto edge_new = MakeView(
            22, 10.06, 10.10, 0.55, 0.0, 0.81, 0.0,
            kWidth * kHeight, 1.90, 0.0);
        Check(edge_old.ok() && edge_new.ok(),
              "edge conflict fixtures were not valid");
        const auto edge_snapshot = BuildWorldTerrainSnapshot(
            7, 10.10, std::vector<CaptureTerrainViewResult>{
                edge_old, edge_new}, StationaryOptions());
        Check(edge_snapshot.ok(), "edge conflict snapshot rejected");
        const auto edge_conflict = SampleWorldTerrainSnapshot(
            edge_snapshot.snapshot, 0.94, 0.05, 0.05, 0.20);
        Check(!edge_conflict.ok() &&
                  edge_conflict.error ==
                      WorldTerrainSnapshotError::kConflictingHistory &&
                  edge_conflict.conflict_newer_sequence == 22 &&
                  edge_conflict.conflict_older_sequence == 21 &&
                  edge_conflict.diagnostic.failure ==
                      WorldTerrainQueryFailure::kConflictingHistory,
              "partial edge overlap did not reject conflicting history");
        const auto far_query = SampleWorldTerrainSnapshot(
            edge_snapshot.snapshot, 1.0e300, -1.0e300, 0.02, 0.20);
        Check(!far_query.ok() &&
                  far_query.error == WorldTerrainSnapshotError::kOutside,
              "non-overlapping query was not rejected safely");
        const auto sloped_latest = MakeView(
            12, 10.05, 10.10, 0.55, -0.40, 0.80, 0.40);
        const auto sloped_snapshot = BuildWorldTerrainSnapshot(
            7, 10.10,
            std::vector<CaptureTerrainViewResult>{history, sloped_latest},
            StationaryOptions());
        Check(sloped_snapshot.ok(), "sloped snapshot rejected");
        const auto sloped = SampleWorldTerrainSnapshot(
            sloped_snapshot.snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(!sloped.ok() &&
                  sloped.error == WorldTerrainSnapshotError::kConflictingHistory,
              "normal conflict between flat and slope was hidden");
        const auto old_state = MakeView(11, 9.95, 10.00, 0.30, 0.35, 0.80);
        const auto stale_snapshot = BuildWorldTerrainSnapshot(
            7, 10.30, std::vector<CaptureTerrainViewResult>{old_state},
            StationaryOptions());
        Check(stale_snapshot.ok(), "stale history snapshot construction failed");
        const auto lagged_snapshot = BuildWorldTerrainSnapshot(
            7, 10.10, std::vector<CaptureTerrainViewResult>{old_state},
            StationaryOptions());
        const auto lagged = SampleWorldTerrainSnapshot(
            lagged_snapshot.snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(lagged.ok() && !lagged.history_used &&
                  lagged.selected_source_sequence == 11 &&
                  std::abs(lagged.diagnostic.cell_age_max_s - 0.15) < 1.0e-9,
              "freshness double-counted lagged model state time");
        const auto stale = SampleWorldTerrainSnapshot(
            stale_snapshot.snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(!stale.ok() &&
                  stale.error == WorldTerrainSnapshotError::kStaleCells &&
                  stale.latest_failure == WorldTerrainQueryFailure::kStaleCells &&
                  stale.diagnostic.failure == WorldTerrainQueryFailure::kStaleCells &&
                  stale.selected_source_sequence == 0,
              "history freshness was not evaluated at snapshot state time");
        const auto undefined_source = MakeView(
            13, 10.07, 10.10, 0.40, 0.0, 0.80);
        auto malformed = undefined_source;
        malformed.provenance.source = static_cast<TerrainSource>(99);
        const auto rejected = BuildWorldTerrainSnapshot(
            7, 10.10, std::vector<CaptureTerrainViewResult>{malformed},
            StationaryOptions());
        Check(!rejected.ok() &&
                  rejected.error == WorldTerrainSnapshotError::kCaptureInvalid,
              "capture source provenance was not validated");
        auto invalid_scope = latest;
        invalid_scope.scope = static_cast<CaptureTerrainViewScope>(0);
        const auto rejected_scope = BuildWorldTerrainSnapshot(
            7, 10.10, std::vector<CaptureTerrainViewResult>{invalid_scope},
            StationaryOptions());
        Check(!rejected_scope.ok() &&
                  rejected_scope.error ==
                      WorldTerrainSnapshotError::kCaptureInvalid,
              "capture scope was not validated");

        auto malformed_snapshot = built.snapshot;
        malformed_snapshot.captures[1].provenance.source_sequence = 999;
        const auto malformed_result = SampleWorldTerrainSnapshot(
            malformed_snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(!malformed_result.ok() &&
                  malformed_result.error ==
                      WorldTerrainSnapshotError::kInvalidInput,
              "all snapshot entry provenance was not validated");

        auto unsorted_snapshot = built.snapshot;
        const WorldTerrainSnapshotEntry first_entry =
            unsorted_snapshot.captures[0];
        unsorted_snapshot.captures[0] = unsorted_snapshot.captures[1];
        unsorted_snapshot.captures[1] = first_entry;
        const auto unsorted_result = SampleWorldTerrainSnapshot(
            unsorted_snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(!unsorted_result.ok() &&
                  unsorted_result.error ==
                      WorldTerrainSnapshotError::kInvalidInput,
              "unsorted snapshot history was accepted");

        auto duplicate_snapshot = built.snapshot;
        duplicate_snapshot.captures[1] = duplicate_snapshot.captures[0];
        const auto duplicate_result = SampleWorldTerrainSnapshot(
            duplicate_snapshot, query_xy.x, query_xy.y, 0.02, 0.20);
        Check(!duplicate_result.ok() &&
                  duplicate_result.error ==
                      WorldTerrainSnapshotError::kInvalidInput,
              "duplicate snapshot history was accepted");
        std::cout << "Stage C world terrain snapshot checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
