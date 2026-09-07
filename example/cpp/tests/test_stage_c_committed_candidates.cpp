#include "stage_c/terrain_candidates.h"
#include "stage_c/capture_terrain_view.h"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
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

TimeNs T(double seconds)
{
    return TimeNs::FromSeconds(seconds);
}

TerrainModel FlatTerrain(double height_m, std::uint64_t epoch)
{
    TerrainModel model;
    model.frame_id = "world";
    model.state_stamp_s = 1.0;
    model.map_stamp_s = 0.95;
    model.age_s = 0.05;
    model.epoch = epoch;
    model.map_sequence = epoch == 8 ? 208 : 107;
    model.resolution_m = 0.10;
    model.origin_m = {-1.0, -1.0};
    model.width = 40;
    model.height = 40;
    model.source = TerrainSource::kTestFixture;
    model.registered = true;
    model.cells.assign(model.width * model.height, TerrainCell{});
    for (TerrainCell &cell : model.cells)
    {
        cell.height_m = height_m;
        cell.height_min_m = height_m;
        cell.height_max_m = height_m;
        cell.has_height_bounds = true;
        cell.age_s = 0.05;
        cell.slope_rad = 0.0;
        cell.roughness_m = 0.0;
        cell.variance_m2 = 0.0;
        cell.normal = {0.0, 0.0, 1.0};
        cell.known = true;
    }
    return model;
}

TimedPoint CommittedTarget(double z)
{
    return {{0.30, 0.0, z}, Frame::kWorld, T(0.80), true,
            PointRole::kSurfaceContactPoint};
}

TouchdownEventTable Events()
{
    TouchdownEventTable table;
    TouchdownEvent committed;
    committed.id = {7, go2::Leg::FR, 1};
    committed.liftoff_time = T(0.90);
    committed.liftoff_valid = true;
    committed.touchdown_time = T(1.10);
    committed.contact_interval_end = T(1.20);
    committed.target_world = CommittedTarget(-0.25);
    committed.committed = true;
    table.events.push_back(committed);

    TouchdownEvent future;
    future.id = {7, go2::Leg::FL, 1};
    future.liftoff_time = T(1.00);
    future.liftoff_valid = true;
    future.touchdown_time = T(1.30);
    future.contact_interval_end = T(1.40);
    table.events.push_back(future);
    return table;
}

TerrainCandidateReference Reference()
{
    TerrainCandidateReference reference;
    reference.com_world_at_touchdown = {
        {0.0, 0.0, 0.50}, {0.40, 0.0, 0.50}};
    reference.com_world_at_touchdown_valid = {true, true};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        reference.nominal_foot_center_offset_world[leg] = {0.0, 0.0, -0.30};
        reference.nominal_offset_valid[leg] = true;
        reference.foot_radius_m[leg] = 0.022;
        reference.foot_radius_valid[leg] = true;
    }
    return reference;
}

TerrainCandidateConfig BoundConfig()
{
    TerrainCandidateConfig config;
    config.bind_committed_targets = true;
    return config;
}

WorldTerrainSnapshotBuildResult OldSnapshot()
{
    TerrainMapEnvelope source;
    source.sequence = 107;
    source.map_stamp_s = 0.95;
    source.frame_id = "base_link";
    source.resolution_m = 0.10;
    source.width = 40;
    source.height = 40;
    source.origin_m = {-1.0, -1.0};
    source.capture_position_world = {0.0, 0.0, 0.30};
    source.capture_yaw_rad = 0.0;
    source.heights_m.assign(40 * 40, -0.55);
    source.observation_stamp_s.assign(40 * 40, 0.95);
    const auto capture = BuildCaptureHeadingTerrainView(
        source, 1.0, 7, TerrainSource::kTestFixture, 0.20);
    WorldTerrainSnapshotOptions options;
    options.stationary_terrain_assumption = true;
    return BuildWorldTerrainSnapshot(
        7, 1.0, std::vector<CaptureTerrainViewResult>{capture}, options);
}

void TestOptInBindingAndDefault()
{
    const auto terrain = FlatTerrain(-0.25, 8);
    const auto events = Events();
    const auto reference = Reference();

    const auto legacy = GenerateTerrainCandidates(
        terrain, events, T(1.0), 8, reference, T(2.0));
    Check(legacy.valid && legacy.sets[0].event_set.candidates.size() == 5,
          "default config changed committed legacy behavior");

    const auto bound = GenerateTerrainCandidates(
        terrain, events, T(1.0), 8, reference, T(2.0), BoundConfig());
    Check(bound.valid && bound.failure == JointPlannerFailure::kNone &&
              bound.committed_targets_bound == 1 &&
              bound.sets[0].event_set.candidates.size() == 1 &&
              bound.sets[1].event_set.candidates.size() == 5,
          "opt-in binding did not isolate committed events");

    const auto &candidate = bound.sets[0].event_set.candidates.front();
    const auto &match = bound.sets[0].matched_surfaces.front();
    const auto target = CommittedTarget(-0.25);
    Check(candidate.target_world.value.x == target.value.x &&
              candidate.target_world.value.y == target.value.y &&
              candidate.target_world.value.z == target.value.z &&
              candidate.target_world.source_time == target.source_time &&
              candidate.target_world.role == target.role &&
              match.surface_world.value.x == target.value.x &&
              match.surface_world.value.y == target.value.y &&
              match.surface_world.value.z == target.value.z &&
              match.surface_world.source_time == target.source_time &&
              match.surface_world.role == target.role &&
              match.source_sequence == terrain.map_sequence,
          "committed target or source metadata was rewritten");
    Check(bound.sets[1].event_set.candidates.size() == 5 &&
              bound.sets[1].event_set.candidates[1].target_world.value.x !=
                  target.value.x,
          "uncommitted event was incorrectly target-bound");
}

void TestNewEpochAndFreshValidation()
{
    const auto terrain = FlatTerrain(-0.25, 8);
    const auto events = Events();
    const auto reference = Reference();
    const auto old_snapshot = OldSnapshot();
    Check(old_snapshot.ok(), "old observation snapshot was not valid");

    const auto mismatched_history = GenerateTerrainCandidates(
        terrain, events, T(1.0), 8, reference, T(2.0), BoundConfig(),
        &old_snapshot.snapshot);
    Check(!mismatched_history.valid &&
              mismatched_history.failure ==
                  JointPlannerFailure::kObservationUnavailable,
          "current terrain history epoch mismatch was accepted");

    // The old committed target has only a source timestamp and role. A new
    // current terrain epoch may revalidate it without inheriting old metadata.
    const auto new_epoch = GenerateTerrainCandidates(
        terrain, events, T(1.0), 8, reference, T(2.0), BoundConfig());
    Check(new_epoch.valid && new_epoch.sets[0].event_set.candidates.size() == 1 &&
              new_epoch.sets[0].matched_surfaces[0].contact_surface.map_epoch ==
                  terrain.epoch &&
              new_epoch.sets[0].matched_surfaces[0].source_sequence ==
                  terrain.map_sequence,
          "new current epoch was coupled to old observation epoch");
}

void TestUnknownAndHeightConflict()
{
    const auto reference = Reference();
    const auto events = Events();

    auto unknown = FlatTerrain(-0.25, 8);
    for (std::size_t iy = 9; iy <= 10; ++iy)
        for (std::size_t ix = 12; ix <= 13; ++ix)
            unknown.cells[iy * unknown.width + ix].known = false;
    const auto unknown_result = GenerateTerrainCandidates(
        unknown, events, T(1.0), 8, reference, T(2.0), BoundConfig());
    Check(!unknown_result.valid &&
              unknown_result.failure == JointPlannerFailure::kCoverageIncomplete,
          "unknown committed footprint was accepted");

    const auto raised_result = GenerateTerrainCandidates(
        FlatTerrain(-0.23, 8), events, T(1.0), 8, reference, T(2.0),
        BoundConfig());
    Check(!raised_result.valid && raised_result.commitment_conflict &&
              raised_result.failure == JointPlannerFailure::kCommitmentConflict &&
              raised_result.commitment_height_gap_m > 0.009 &&
              !raised_result.rejected_query_diagnostics.empty() &&
              raised_result.rejected_query_diagnostics.front().query.failure ==
                  WorldTerrainQueryFailure::kConflictingHistory,
          "height conflict against committed target was accepted");
}
}

int main()
{
    try
    {
        TestOptInBindingAndDefault();
        TestNewEpochAndFreshValidation();
        TestUnknownAndHeightConflict();
        std::cout << "committed terrain candidate fixtures passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
