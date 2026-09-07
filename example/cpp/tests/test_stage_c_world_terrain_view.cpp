#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include "stage_c/world_terrain_view.h"
namespace
{
bool Check(bool value, const char *message)
{
    if (!value)
        std::cerr << "FAIL: " << message << "\n";
    return value;
}
go2_terrain::TerrainModel MakeModel()
{
    go2_terrain::TerrainModel model;
    model.frame_id = "base_link";
    model.state_stamp_s = 10.0;
    model.map_stamp_s = 10.0;
    model.age_s = 0.0;
    model.epoch = 17;
    model.resolution_m = 0.05;
    model.origin_m = {-1.0, -1.0};
    model.width = 40;
    model.height = 40;
    model.source = go2_terrain::TerrainSource::kTestFixture;
    model.registered = true;
    model.registration_position_world = {2.0, -3.0, 0.40};
    model.registration_yaw_rad = 0.60;
    model.cells.resize(model.width * model.height);
    const double normal_xy_norm = std::hypot(0.20, -0.30);
    const double normal_z = std::sqrt(1.0 - normal_xy_norm * normal_xy_norm);
    for (auto &cell : model.cells)
    {
        cell.height_m = 0.15;
        cell.height_min_m = 0.10;
        cell.height_max_m = 0.20;
        cell.has_height_bounds = true;
        cell.age_s = 0.01;
        cell.slope_rad = 0.20;
        cell.roughness_m = 0.001;
        cell.variance_m2 = 0.000001;
        cell.normal = {0.20, -0.30, normal_z};
        cell.known = true;
    }
    return model;
}
std::array<double, 2> LocalToWorldXY(
    const go2_terrain::TerrainModel &model, double x, double y)
{
    const double c = std::cos(model.registration_yaw_rad);
    const double s = std::sin(model.registration_yaw_rad);
    return {model.registration_position_world[0] + c * x - s * y,
            model.registration_position_world[1] + s * x + c * y};
}
std::array<double, 3> LocalNormalToWorld(
    const go2_terrain::TerrainModel &model,
    const std::array<double, 3> &normal)
{
    const double c = std::cos(model.registration_yaw_rad);
    const double s = std::sin(model.registration_yaw_rad);
    return {c * normal[0] - s * normal[1],
            s * normal[0] + c * normal[1], normal[2]};
}
} // namespace
int main()
{
    using namespace go2_terrain;
    using namespace go2_terrain::stage_c;
    bool ok = true;
    auto registered = MakeModel();
    ok &= Check(WorldTerrainMetadataValid(registered),
                "registered heading model metadata rejected");
    const auto source_before = registered.source;
    const auto epoch_before = registered.epoch;
    const double age_before = registered.age_s;
    const auto world_xy = LocalToWorldXY(registered, 0.10, -0.20);
    TerrainPatch patch;
    ok &= Check(SampleWorldTerrainPatch(
                    registered, world_xy[0], world_xy[1], 0.025, 0.20,
                    patch),
                "registered world patch rejected");
    ok &= Check(std::abs(patch.center_height_m - 0.55) < 1.0e-12,
                "registered center height did not add registration z");
    ok &= Check(std::abs(patch.min_height_m - 0.50) < 1.0e-12 &&
                    std::abs(patch.max_height_m - 0.60) < 1.0e-12,
                "registered height bounds did not add registration z");
    const auto expected_normal = LocalNormalToWorld(
        registered, {0.20, -0.30, std::sqrt(1.0 - 0.13)});
    ok &= Check(std::abs(patch.normal[0] - expected_normal[0]) < 1.0e-12 &&
                    std::abs(patch.normal[1] - expected_normal[1]) < 1.0e-12 &&
                    std::abs(patch.normal[2] - expected_normal[2]) < 1.0e-12,
                "tilted normal was not rotated by registration yaw");
    ok &= Check(registered.source == source_before &&
                    registered.epoch == epoch_before &&
                    registered.age_s == age_before,
                "sampling mutated source epoch or age");
    auto native = registered;
    native.frame_id = "world";
    native.registered = false;
    native.registration_position_world = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    native.registration_yaw_rad = std::numeric_limits<double>::quiet_NaN();
    TerrainPatch native_patch;
    ok &= Check(WorldTerrainMetadataValid(native),
                "world-native model metadata rejected");
    ok &= Check(SampleWorldTerrainPatch(native, 0.10, -0.20, 0.025, 0.20,
                                         native_patch),
                "world-native patch rejected");
    ok &= Check(std::abs(native_patch.center_height_m - 0.15) < 1.0e-12 &&
                    std::abs(native_patch.normal[0] - 0.20) < 1.0e-12,
                "world-native patch was transformed");
    auto native_registered = native;
    native_registered.registered = true;
    ok &= Check(WorldTerrainMetadataValid(native_registered),
                "registered world-native model rejected without pose");
    ok &= Check(SampleWorldTerrainPatch(
                    native_registered, 0.10, -0.20, 0.025, 0.20,
                    native_patch),
                "registered world-native patch rejected without pose");
    auto unknown = registered;
    unknown.cells[20 * unknown.width + 20].known = false;
    TerrainPatch rejected_patch;
    WorldTerrainQueryDiagnostic query;
    ok &= Check(!SampleWorldTerrainPatch(
                    unknown, registered.registration_position_world[0],
                    registered.registration_position_world[1], 0.025, 0.20,
                    rejected_patch, &query) &&
                    query.failure == WorldTerrainQueryFailure::kUnknownCells &&
                    query.patch_total > query.patch_known &&
                    std::abs(query.local_x) < 1.0e-12 &&
                    std::abs(query.local_y) < 1.0e-12,
                "unknown patch reason or local coordinates were lost");
    auto stale = registered;
    stale.cells[20 * stale.width + 20].age_s = 0.50;
    ok &= Check(!SampleWorldTerrainPatch(
                    stale, registered.registration_position_world[0],
                    registered.registration_position_world[1], 0.025, 0.20,
                    rejected_patch, &query) &&
                    query.failure == WorldTerrainQueryFailure::kStaleCells &&
                    query.cell_age_max_s >= 0.01,
                "stale patch reason was lost");
    auto first_stale = registered;
    first_stale.cells[19 * first_stale.width + 19].age_s = 0.50;
    ok &= Check(!SampleWorldTerrainPatch(
                    first_stale, registered.registration_position_world[0],
                    registered.registration_position_world[1], 0.025, 0.20,
                    rejected_patch, &query) &&
                    query.failure == WorldTerrainQueryFailure::kStaleCells &&
                    query.cell_age_max_s >= 0.50,
                "first stale cell was omitted from age maximum");
    const auto outside_xy = LocalToWorldXY(registered, 1.0, 0.0);
    ok &= Check(!SampleWorldTerrainPatch(
                    registered, outside_xy[0], outside_xy[1], 0.025, 0.20,
                    rejected_patch, &query) &&
                    query.failure == WorldTerrainQueryFailure::kOutside,
                "out-of-grid patch reason was lost");
    ok &= Check(!SampleWorldTerrainPatch(
                    registered, std::numeric_limits<double>::quiet_NaN(), 0.0,
                    0.025, 0.20, rejected_patch, &query) &&
                    query.failure == WorldTerrainQueryFailure::kInvalidQuery,
                "invalid query reason was lost");
    auto nonfinite = registered;
    for (TerrainCell &cell : nonfinite.cells)
        cell.height_m = std::numeric_limits<double>::quiet_NaN();
    ok &= Check(!SampleWorldTerrainPatch(
                    nonfinite, registered.registration_position_world[0],
                    registered.registration_position_world[1], 0.025, 0.20,
                    rejected_patch, &query) &&
                    query.failure == WorldTerrainQueryFailure::kNonfinitePatch,
                "nonfinite patch reason was lost");
    auto unregistered = registered;
    unregistered.registered = false;
    ok &= Check(!WorldTerrainMetadataValid(unregistered),
                "unregistered base_link model was accepted");
    auto bad_pose = registered;
    bad_pose.registration_position_world[0] =
        std::numeric_limits<double>::quiet_NaN();
    ok &= Check(!WorldTerrainMetadataValid(bad_pose),
                "NaN registration pose was accepted");
    auto bad_frame = registered;
    bad_frame.frame_id = "map";
    ok &= Check(!WorldTerrainMetadataValid(bad_frame),
                "unknown frame was accepted");
    auto bad_age = registered;
    bad_age.age_s = 0.25;
    bad_age.map_stamp_s = 9.0;
    ok &= Check(!WorldTerrainMetadataValid(bad_age),
                "inconsistent model age was accepted");
    return ok ? 0 : 1;
}
