#include "stage_c/capture_terrain_view.h"
#include "stage_c/world_terrain_view.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace go2_terrain;
using namespace go2_terrain::stage_c;
namespace
{
void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
TerrainMapEnvelope SourceEnvelope()
{
    TerrainMapEnvelope source;
    source.sequence = 318;
    source.map_stamp_s = 10.0;
    source.frame_id = "base_link";
    source.resolution_m = static_cast<double>(0.05f);
    source.width = 32;
    source.height = 10;
    source.origin_m = {static_cast<double>(-0.45f), static_cast<double>(-0.225f)};
    source.capture_position_world = {1.0, -2.0, 0.40};
    source.capture_yaw_rad = 0.25;
    const std::size_t count = static_cast<std::size_t>(source.width) *
        source.height;
    source.heights_m.assign(count, 0.10);
    source.observation_stamp_s.assign(count, 10.0);
    return source;
}
} // namespace
int main()
{
    try
    {
        const auto source = SourceEnvelope();
        const auto source_before = source;
        const auto view = BuildCaptureHeadingTerrainView(
            source, 10.04, 17, TerrainSource::kLidar);
        Check(view.ok() && view.scope ==
                  CaptureTerrainViewScope::kCaptureHeadingWorldQuery,
              "capture heading view was not built");
        Check(view.provenance.source_sequence == source.sequence &&
                  view.provenance.map_epoch == 17 &&
                  view.provenance.source == TerrainSource::kLidar &&
                  view.provenance.state_stamp_s == 10.04,
              "capture view provenance was not explicit");
        Check(view.model.registered && view.model.frame_id == "base_link" &&
                  view.model.registration_position_world ==
                      source.capture_position_world &&
                  view.model.registration_yaw_rad == source.capture_yaw_rad,
              "view registration used a current-body pose");
        Check(view.source_known_cells == 320 && view.view_known_cells == 320 &&
                  view.coverage_preserved && view.model.cells.size() == 320,
              "identity capture view lost known cells");
        Check(view.model.cells[0].has_height_bounds &&
                  std::abs(view.model.cells[0].height_m - 0.10) < 1.0e-6 &&
                  std::abs(view.model.cells[0].height_min_m - 0.10) < 1.0e-12 &&
                  std::abs(view.model.cells[0].height_max_m - 0.10) < 1.0e-12 &&
                  std::abs(view.model.cells[0].age_s - 0.04) < 1.0e-12,
              "source height, bounds or observation age changed");
        Check(source.sequence == source_before.sequence &&
                  source.map_stamp_s == source_before.map_stamp_s &&
                  source.capture_position_world ==
                      source_before.capture_position_world &&
                  source.capture_yaw_rad == source_before.capture_yaw_rad &&
                  source.heights_m == source_before.heights_m &&
                  source.observation_stamp_s == source_before.observation_stamp_s,
              "source envelope was mutated");
        const TerrainMapXY local_query{0.10, -0.10};
        const TerrainMapXY world_query = TerrainMapToWorld(
            source.capture_position_world, source.capture_yaw_rad,
            local_query);
        TerrainPatch patch;
        Check(SampleWorldTerrainPatch(
                  view.model, world_query.x, world_query.y, 0.01, 0.20,
                  patch),
              "capture view world query failed");
        Check(std::abs(patch.center_height_m - 0.50) < 2.0e-6 &&
                  std::abs(patch.min_height_m - 0.50) < 2.0e-6 &&
                  std::abs(patch.max_height_m - 0.50) < 2.0e-6 &&
                  std::abs(patch.normal[0]) < 1.0e-12 &&
                  std::abs(patch.normal[1]) < 1.0e-12 &&
                  std::abs(patch.normal[2] - 1.0) < 1.0e-12,
              "capture view world height or normal was transformed incorrectly");
        auto moved_pose = source.capture_position_world;
        moved_pose[0] += 1.0e-6;moved_pose[1] += 1.0e-6;
        const auto remapped = RegisterTerrainMap(source,10.04,moved_pose,
            source.capture_yaw_rad,.20,TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
        Check(remapped.ok(),"comparison registration rejected");
        std::size_t remapped_known=0;for(float z:remapped.map.map.data())remapped_known+=std::isfinite(z);
        Check(remapped_known<320 && view.view_known_cells==320,
            "capture view did not avoid current-body resampling loss");
        auto slope_source=source;
        for(std::size_t iy=0;iy<source.height;++iy)for(std::size_t ix=0;ix<source.width;++ix)
            slope_source.heights_m[iy*source.width+ix]=.1+.1*(source.origin_m[0]+(ix+.5)*source.resolution_m)+.05*(source.origin_m[1]+(iy+.5)*source.resolution_m);
        const auto slope_view=BuildCaptureHeadingTerrainView(slope_source,10.04,17,TerrainSource::kLidar);
        Check(slope_view.ok(),"sloped capture view rejected");
        TerrainPatch slope_patch;
        Check(SampleWorldTerrainPatch(slope_view.model,world_query.x,world_query.y,.01,.2,slope_patch),"sloped world query failed");
        const double norm=std::sqrt(1+.1*.1+.05*.05),c=std::cos(source.capture_yaw_rad),sn=std::sin(source.capture_yaw_rad);
        Check(std::abs(slope_patch.normal[0]-(-.1*c+.05*sn)/norm)<1e-6 &&
              std::abs(slope_patch.normal[1]-(-.1*sn-.05*c)/norm)<1e-6 &&
              std::abs(slope_patch.normal[2]-1/norm)<1e-6,"capture slope normal world rotation");
        auto unknown = source;
        const std::size_t unknown_index = 3 * source.width + 8;
        unknown.heights_m[unknown_index] = kTerrainMapUnknown;
        unknown.observation_stamp_s[unknown_index] = kTerrainMapUnknown;
        const auto unknown_view = BuildCaptureHeadingTerrainView(
            unknown, 10.04, 17, TerrainSource::kLidar);
        Check(unknown_view.ok() && unknown_view.source_known_cells == 319 &&
                  unknown_view.view_known_cells == 319 &&
                  !unknown_view.model.cells[unknown_index].known,
              "source unknown cell was filled or coverage accounting changed");
        const std::size_t unknown_x = unknown_index % source.width;
        const std::size_t unknown_y = unknown_index / source.width;
        const TerrainMapXY unknown_local{
            source.origin_m[0] + (static_cast<double>(unknown_x) + 0.5) *
                source.resolution_m,
            source.origin_m[1] + (static_cast<double>(unknown_y) + 0.5) *
                source.resolution_m};
        const TerrainMapXY unknown_world = TerrainMapToWorld(
            source.capture_position_world, source.capture_yaw_rad,
            unknown_local);
        TerrainPatch unknown_patch;
        Check(!SampleWorldTerrainPatch(
                  unknown_view.model, unknown_world.x, unknown_world.y, 0.01,
                  0.20, unknown_patch),
              "world query filled source unknown cell");
        auto stale = source;
        stale.observation_stamp_s[0] = 9.70;
        const auto stale_view = BuildCaptureHeadingTerrainView(
            stale, 10.04, 17, TerrainSource::kLidar);
        Check(!stale_view.ok() &&
                  stale_view.error == CaptureTerrainViewError::kStaleSource &&
                  stale_view.registration_error ==
                      TerrainMapRegistrationError::kInvalidCellAge,
              "stale source cell was accepted");
        const auto unknown_source = BuildCaptureHeadingTerrainView(
            source, 10.04, 17, static_cast<TerrainSource>(99));
        Check(!unknown_source.ok() &&
                  unknown_source.error == CaptureTerrainViewError::kInvalidInput,
              "undefined terrain source was accepted");
        std::cout << "Stage C capture terrain view checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
