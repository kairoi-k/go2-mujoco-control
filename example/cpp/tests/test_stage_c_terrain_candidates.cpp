#include "stage_c/event_schedule.h"
#include "stage_c/terrain_candidates.h"
#include <Eigen/Dense>
#include <array>
#include <cmath>
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
TerrainModel FlatTerrain()
{
    TerrainModel model;
    model.frame_id = "world";
    model.state_stamp_s = 1.0;
    model.map_stamp_s = 0.95;
    model.age_s = 0.05;
    model.epoch = 7;
    model.resolution_m = 0.1;
    model.origin_m = {-1.0, -1.0};
    model.width = 40;
    model.height = 40;
    model.source = TerrainSource::kTestFixture;
    model.registered = true;
    model.cells.assign(model.width * model.height, TerrainCell{});
    for (TerrainCell &cell : model.cells)
    {
        cell.height_m = -0.25;
        cell.height_min_m = -0.25;
        cell.height_max_m = -0.25;
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
TouchdownEventTable Events()
{
    TouchdownEventTable table;
    TouchdownEvent first;
    first.id = {7, go2::Leg::FR, 1};
    first.liftoff_time = T(0.90);
    first.liftoff_valid = true;
    first.touchdown_time = T(1.50);
    first.contact_interval_end = T(1.56);
    // A schedule preview deliberately has no terrain target yet.
    table.events.push_back(first);
    TouchdownEvent second;
    second.id = {7, go2::Leg::FL, 1};
    second.liftoff_time = T(0.98);
    second.liftoff_valid = true;
    second.touchdown_time = T(1.60);
    second.contact_interval_end = T(1.66);
    table.events.push_back(second);
    return table;
}
TerrainCandidateReference PerEventReference()
{
    TerrainCandidateReference reference;
    reference.com_world_at_touchdown = {{0.40, 0.00, 0.50},
                                        {0.80, 0.00, 0.50}};
    reference.com_world_at_touchdown_valid = {true, true};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        reference.nominal_foot_center_offset_world[leg] =
            {leg == 0 ? 0.10 : -0.10, 0.0, -0.30};
        reference.nominal_offset_valid[leg] = true;
        reference.foot_radius_m[leg] = 0.022;
        reference.foot_radius_valid[leg] = true;
    }
    return reference;
}
TerrainCandidateReference ConstantReference()
{
    TerrainCandidateReference reference = PerEventReference();
    reference.com_world_at_touchdown.clear();
    reference.com_world_at_touchdown_valid.clear();
    reference.com_world_valid = true;
    reference.com_world = {0.40, 0.0, 0.50};
    reference.com_velocity_world_valid = true;
    reference.com_velocity_world = {0.10, 0.0, 0.0};
    return reference;
}
TerrainCandidateGenerationResult Generate(
    const TerrainModel &terrain, const TouchdownEventTable &events,
    const TerrainCandidateReference &reference,
    const TerrainCandidateConfig &config = TerrainCandidateConfig{})
{
    return GenerateTerrainCandidates(
        terrain, events, T(1.0), 7, reference, T(2.0), config);
}
void CheckSurfacePair(const TerrainCandidateSurfaceMatch &match)
{
    Check(match.valid(), "surface match validity");
    Check(match.surface_world.source_time == T(0.95),
          "surface source time must be map time");
    Check(match.surface_world.source_time != T(1.50),
          "surface source time must not be touchdown time");
    Check(match.contact_surface.valid_until == T(2.0),
          "prediction horizon binding");
    const Eigen::Vector3d surface(
        match.surface_world.value.x, match.surface_world.value.y,
        match.surface_world.value.z);
    const Eigen::Vector3d center(
        match.sphere_center_world.value.x, match.sphere_center_world.value.y,
        match.sphere_center_world.value.z);
    Check((center - surface - match.collision_radius_m *
                             match.contact_surface.basis_world.col(2)).norm() <
              1.0e-12,
          "sphere center is surface plus radius normal");
}
void TestAlternativesAndProvenance()
{
    const TerrainModel terrain = FlatTerrain();
    const TouchdownEventTable events = Events();
    Check(!events.valid(), "unset schedule targets remain unset");
    const auto result = Generate(terrain, events, PerEventReference());
    Check(result.valid && result.failure == JointPlannerFailure::kNone,
          "valid terrain alternatives rejected");
    Check(result.sets.size() == events.events.size(),
          "one candidate set per event");
    Check(result.sets[0].event_set.event_id == events.events[0].id &&
              result.sets[1].event_set.event_id == events.events[1].id,
          "event identity preserved");
    Check(result.sets[0].event_set.candidates.size() == 5 &&
              result.sets[1].event_set.candidates.size() == 5,
          "deterministic bounded cross returned");
    Check(result.sets[0].matched_surfaces.size() == 5 &&
              result.sets[1].matched_surfaces.size() == 5,
          "surface matches are one-to-one");
    for (const TerrainCandidateSet &set : result.sets)
    {
        Check(set.valid(), "candidate set validity");
        for (std::size_t index = 0; index < set.event_set.candidates.size(); ++index)
        {
            Check(set.event_set.candidates[index].candidate_id == index + 1,
                  "candidate order is deterministic");
            Check(set.event_set.candidates[index].geometry_hard_feasible,
                  "terrain gates are represented as hard geometry status");
            CheckSurfacePair(set.matched_surfaces[index]);
        }
    }
    Check(std::abs(result.sets[0].matched_surfaces[0].surface_world.value.x -
                   result.sets[1].matched_surfaces[0].surface_world.value.x) >
              0.19,
          "per-event COM references must move future nominal centers");
}
void TestExplicitVelocityReference()
{
    auto events = Events();
    events.events[1].id = {7, go2::Leg::FR, 2};
    const auto result = Generate(FlatTerrain(), events, ConstantReference());
    Check(result.valid, "constant plus world velocity reference rejected");
    const double first_x = result.sets[0].matched_surfaces[0].surface_world.value.x;
    const double second_x = result.sets[1].matched_surfaces[0].surface_world.value.x;
    Check(std::abs((second_x - first_x) - 0.01) < 1.0e-9,
          "world command velocity was not applied at touchdown time");
}
void TestCoverageAndMetadataRejection()
{
    auto unknown = FlatTerrain();
    for (TerrainCell &cell : unknown.cells)
        cell.known = false;
    const auto events = Events();
    auto result = Generate(unknown, events, PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kCoverageIncomplete,
          "unknown terrain was accepted");
    Check(result.sets.size() == 2 && result.sets[0].event_set.candidates.empty() &&
              result.sets[1].event_set.candidates.empty(),
          "coverage loss was silently omitted");
    Check(result.rejected_query_diagnostics.size() == 10,
          "unknown query diagnostics did not retain each event candidate");
    for (std::size_t i = 0; i < result.rejected_query_diagnostics.size(); ++i)
    {
        const auto &diagnostic = result.rejected_query_diagnostics[i];
        Check(diagnostic.event_index == i / 5 &&
                  diagnostic.leg == events.events[i / 5].id.leg &&
                  diagnostic.candidate_index == i % 5 &&
                  diagnostic.query.failure ==
                      WorldTerrainQueryFailure::kUnknownCells &&
                  diagnostic.query.patch_total > diagnostic.query.patch_known &&
                  std::isfinite(diagnostic.query.world_x) &&
                  std::isfinite(diagnostic.query.local_x),
              "unknown query diagnostic lost event, leg, coordinates or reason");
    }
    auto nonfinite_height = FlatTerrain();
    for (TerrainCell &cell : nonfinite_height.cells)
        cell.height_m = std::numeric_limits<double>::quiet_NaN();
    result = Generate(nonfinite_height, Events(), PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kCoverageIncomplete,
          "nonfinite terrain was accepted as known");
    auto outside_reference = PerEventReference();
    outside_reference.com_world_at_touchdown[0] = {-1.15, 0.0, 0.50};
    outside_reference.com_world_at_touchdown[1] = {-1.15, 0.0, 0.50};
    result = Generate(FlatTerrain(), Events(), outside_reference);
    Check(!result.valid && result.failure == JointPlannerFailure::kCoverageIncomplete,
          "outside terrain patch was accepted");
    auto stale = FlatTerrain();
    stale.age_s = 0.21;
    result = Generate(stale, Events(), PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kObservationUnavailable,
          "stale map metadata was accepted");
    auto stale_cell = FlatTerrain();
    stale_cell.cells.front().age_s = 0.21;
    result = Generate(stale_cell, Events(), PerEventReference());
    Check(result.valid,"unrelated stale cell cannot invalidate known candidate patches");
    stale_cell.cells.front().known=false;stale_cell.cells.front().age_s=std::numeric_limits<double>::infinity();
    Check(Generate(stale_cell,Events(),PerEventReference()).valid,"unknown outside queried footprint is not local unknown");
    for(auto &cell:stale_cell.cells)cell.age_s=.21;
    const auto stale_result = Generate(stale_cell,Events(),PerEventReference());
    Check(!stale_result.valid,"stale cells in candidate footprints rejected");
    Check(!stale_result.rejected_query_diagnostics.empty() &&
              stale_result.rejected_query_diagnostics.front().query.failure ==
                  WorldTerrainQueryFailure::kStaleCells,
          "stale candidate query reason was lost");
    auto wrong_epoch = FlatTerrain();
    wrong_epoch.epoch = 8;
    result = Generate(wrong_epoch, Events(), PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kObservationUnavailable,
          "map epoch conflict was accepted");
}
void TestContactContinuationPolicy()
{
    FixedSchedulePreviewRequest request;
    request.start = T(1.0);
    request.end = T(1.30);
    request.phase_zero_time = T(0.0);
    request.period = T(0.24);
    request.max_interval = T(0.04);
    request.schedule_epoch = 7;
    request.duty = 0.80;
    request.leg_offsets.fill(0.0);
    const auto preview = BuildFixedSchedulePreview(request);
    Check(preview.complete && !preview.events.events.empty(),
          "tail stance schedule preview failed");
    const TimeNs horizon = T(1.30);
    bool has_tail_stance = false;
    for (const auto &event : preview.events.events)
        has_tail_stance |= event.contact_interval_end > horizon;
    Check(has_tail_stance, "tail stance was not represented with its real end");
    const auto reference = ConstantReference();
    const auto default_result = GenerateTerrainCandidates(
        FlatTerrain(), preview.events, T(1.0), 7, reference, horizon);
    Check(!default_result.valid &&
              default_result.failure == JointPlannerFailure::kCoverageIncomplete,
          "default candidate policy accepted a continuation beyond horizon");
    auto continuation_config = TerrainCandidateConfig{};
    continuation_config.allow_contact_continuation_beyond_horizon = true;
    const auto generated = GenerateTerrainCandidates(
        FlatTerrain(), preview.events, T(1.0), 7, reference, horizon,
        continuation_config);
    Check(generated.valid && generated.failure == JointPlannerFailure::kNone &&
              generated.sets.size() == preview.events.events.size(),
          "explicit contact continuation was rejected");
    for (const auto &set : generated.sets)
        for (const auto &match : set.matched_surfaces)
            Check(match.contact_surface.valid_until == horizon,
                  "continuation surface validity escaped the horizon");
    auto outside_td = preview.events;
    outside_td.events.back().touchdown_time = T(1.31);
    const auto outside_result = GenerateTerrainCandidates(
        FlatTerrain(), outside_td, T(1.0), 7, reference, horizon,
        continuation_config);
    Check(!outside_result.valid &&
              outside_result.failure == JointPlannerFailure::kCoverageIncomplete,
          "touchdown beyond horizon was accepted by continuation policy");
}
void TestExplicitTerrainGates()
{
    auto sloped = FlatTerrain();
    for (TerrainCell &cell : sloped.cells)
        cell.slope_rad = 0.60;
    auto result = Generate(sloped, Events(), PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kNoFeasibleCandidateInSet,
          "slope gate was ignored");
    auto rough = FlatTerrain();
    for (TerrainCell &cell : rough.cells)
        cell.roughness_m = 0.02;
    result = Generate(rough, Events(), PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kNoFeasibleCandidateInSet,
          "roughness gate was ignored");
    auto height = FlatTerrain();
    for (TerrainCell &cell : height.cells)
    {
        cell.height_min_m = -0.30;
        cell.height_max_m = -0.20;
    }
    result = Generate(height, Events(), PerEventReference());
    Check(!result.valid && result.failure == JointPlannerFailure::kNoFeasibleCandidateInSet,
          "height envelope gate was ignored");
    auto edge_config = TerrainCandidateConfig{};
    edge_config.minimum_edge_margin_m = 2.0;
    result = Generate(FlatTerrain(), Events(), PerEventReference(), edge_config);
    Check(!result.valid && result.failure == JointPlannerFailure::kNoFeasibleCandidateInSet,
          "edge margin gate was ignored");
}
void TestFailClosedInputs()
{
    auto result = Generate(FlatTerrain(), Events(), TerrainCandidateReference{});
    Check(!result.valid && result.failure == JointPlannerFailure::kObservationUnavailable,
          "missing reference was accepted");
    auto invalid_reference = PerEventReference();
    invalid_reference.com_world_at_touchdown[1].x =
        std::numeric_limits<double>::quiet_NaN();
    result = Generate(FlatTerrain(), Events(), invalid_reference);
    Check(!result.valid, "nonfinite COM reference was accepted");
    auto invalid_velocity = PerEventReference();
    invalid_velocity.com_velocity_world_valid = true;
    invalid_velocity.com_velocity_world.x =
        std::numeric_limits<double>::quiet_NaN();
    result = Generate(FlatTerrain(), Events(), invalid_velocity);
    Check(!result.valid, "nonfinite optional velocity was accepted");
    auto partial_reference = PerEventReference();
    partial_reference.com_world_at_touchdown.pop_back();
    result = Generate(FlatTerrain(), Events(), partial_reference);
    Check(!result.valid, "partial per-event COM reference was accepted");
    auto invalid_events = Events();
    invalid_events.events[1].touchdown_time = T(1.40);
    result = Generate(FlatTerrain(), invalid_events, PerEventReference());
    Check(!result.valid, "nonmonotonic event times were accepted");
    auto short_horizon = GenerateTerrainCandidates(
        FlatTerrain(), Events(), T(1.0), 7, PerEventReference(), T(1.55));
    Check(!short_horizon.valid &&
              short_horizon.failure == JointPlannerFailure::kCoverageIncomplete,
          "prediction horizon did not gate contact coverage");
    auto bad_config = TerrainCandidateConfig{};
    bad_config.assumed_friction_mu =
        std::numeric_limits<double>::quiet_NaN();
    result = Generate(FlatTerrain(), Events(), PerEventReference(), bad_config);
    Check(!result.valid && result.failure == JointPlannerFailure::kObservationUnavailable,
          "nonfinite assumption was accepted");
}
} // namespace
int main()
{
    try
    {
        TestAlternativesAndProvenance();
        TestExplicitVelocityReference();
        TestCoverageAndMetadataRejection();
        TestContactContinuationPolicy();
        TestExplicitTerrainGates();
        TestFailClosedInputs();
        std::cout << "terrain candidates: per-event/world-velocity references, unset targets, "
                     "full-known terrain, deterministic alternatives, provenance, "
                     "terrain gates and fail-closed inputs passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
