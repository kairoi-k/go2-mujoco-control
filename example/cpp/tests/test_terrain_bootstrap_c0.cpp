#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

#include "terrain_bootstrap_c0.h"

namespace
{

go2_terrain::TerrainModel FlatModel()
{
    go2_terrain::TerrainModel model;
    model.frame_id = "base_link";
    model.state_stamp_s = 10.0;
    model.map_stamp_s = 9.99;
    model.age_s = 0.01;
    model.epoch = 7;
    model.resolution_m = 0.02;
    model.origin_m = {-0.60, -0.40};
    model.width = 60;
    model.height = 40;
    model.source = go2_terrain::TerrainSource::kTestFixture;
    model.cells.resize(model.width * model.height);
    for (auto &cell : model.cells)
    {
        cell.known = true;
        cell.height_m = 0.0;
        cell.age_s = 0.01;
        cell.slope_rad = 0.0;
        cell.roughness_m = 0.0;
        cell.variance_m2 = 0.0;
        cell.normal = {0.0, 0.0, 1.0};
    }
    return model;
}

std::array<go2::Vec3, go2::kLegCount> Feet()
{
    return {
        go2::Vec3{0.20, -0.13, -0.30},
        go2::Vec3{0.20, 0.13, -0.30},
        go2::Vec3{-0.20, -0.13, -0.30},
        go2::Vec3{-0.20, 0.13, -0.30}};
}

go2_terrain::TerrainBootstrapC0Input Input(
    const go2_terrain::TerrainModel &model)
{
    go2_terrain::TerrainBootstrapC0Input input;
    input.terrain = &model;
    input.current_feet_base = Feet();
    input.forward_speed_mps = 0.30;
    input.forward_acceleration_mps2 = 0.0;
    input.shaper.max_accel_mps2 = 0.8;
    input.shaper.max_decel_mps2 = 1.2;
    input.shaper.max_jerk_mps3 = 4.0;
    input.shaper.max_speed_mps = 0.30;
    input.dt_s = 0.002;
    return input;
}

void RaiseStep(go2_terrain::TerrainModel &model, double edge_x, double height)
{
    for (std::size_t iy = 0; iy < model.height; ++iy)
    {
        for (std::size_t ix = 0; ix < model.width; ++ix)
        {
            const double x = model.origin_m[0] +
                (static_cast<double>(ix) + 0.5) * model.resolution_m;
            if (x >= edge_x)
                model.cells[iy * model.width + ix].height_m = height;
        }
    }
}

void MakeUnknownStrip(go2_terrain::TerrainModel &model,
                      double min_x, double max_x)
{
    for (std::size_t iy = 0; iy < model.height; ++iy)
    {
        for (std::size_t ix = 0; ix < model.width; ++ix)
        {
            const double x = model.origin_m[0] +
                (static_cast<double>(ix) + 0.5) * model.resolution_m;
            if (x >= min_x && x <= max_x)
                model.cells[iy * model.width + ix].known = false;
        }
    }
}

} // namespace

int main()
{
    // Reproduce the production 2 ms jerk-limited stop from Order-116.
    go2_trot::VelocityCommandShaperParams shaper;
    shaper.max_accel_mps2 = 0.8;
    shaper.max_decel_mps2 = 1.2;
    shaper.max_jerk_mps3 = 4.0;
    shaper.max_speed_mps = 0.30;
    const auto stop = go2_terrain::EstimateBootstrapStopDistance(
        0.30, 0.0, shaper, 0.002);
    assert(stop.valid);
    assert(stop.steps == 200);
    assert(std::abs(stop.distance_m - 0.0774008) < 1.0e-9);

    // Fully known flat terrain covering every current-foot stopping corridor
    // is a valid development C0 envelope. The result explicitly remains
    // non-certifying until end-to-end latency is proven elsewhere.
    auto flat = FlatModel();
    auto result = go2_terrain::EvaluateTerrainBootstrapC0(Input(flat));
    assert(result.readiness.valid());
    assert(result.development_only);
    assert(!result.latency_certified);
    assert(result.reason == "c0_development_ready");
    assert(result.swept_patch_checks > go2::kLegCount);

    // A 5 cm height discontinuity entering the braking corridor must revoke
    // C0 before the foot can be authorized to advance through it. This uses
    // the unchanged 4 cm max-surface-step feasibility threshold.
    auto step = FlatModel();
    RaiseStep(step, 0.25, 0.05);
    result = go2_terrain::EvaluateTerrainBootstrapC0(Input(step));
    assert(result.readiness.dstop_valid);
    assert(result.readiness.local_known_flat);
    assert(!result.readiness.swept_volume_clear);
    assert(result.reason == "swept_patch_not_known_flat");

    // Unknown space is never explored by C0. It can become traversable only
    // after perception makes the same corridor known or C1 takes ownership.
    auto unknown = FlatModel();
    MakeUnknownStrip(unknown, 0.23, 0.27);
    result = go2_terrain::EvaluateTerrainBootstrapC0(Input(unknown));
    assert(result.readiness.dstop_valid);
    assert(!result.readiness.swept_volume_clear);
    assert(result.reason == "swept_patch_not_known_flat");

    // Stale or wrong-frame maps cannot authorize bootstrap motion.
    auto stale = FlatModel();
    stale.age_s = 1.0;
    result = go2_terrain::EvaluateTerrainBootstrapC0(Input(stale));
    assert(!result.readiness.valid());
    assert(result.reason == "map_stale");

    auto wrong_frame = FlatModel();
    wrong_frame.frame_id = "world";
    result = go2_terrain::EvaluateTerrainBootstrapC0(Input(wrong_frame));
    assert(!result.readiness.valid());
    assert(result.reason == "frame_mismatch");

    return 0;
}
