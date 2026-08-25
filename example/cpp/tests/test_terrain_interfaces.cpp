#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include "terrain_feasibility.h"
#include "terrain_motion_plan.h"
#include "terrain_planner.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

unitree_go::msg::dds_::HeightMap_ FlatMap()
{
    unitree_go::msg::dds_::HeightMap_ map;
    map.stamp(10.0);
    map.frame_id("base_link");
    map.resolution(0.05f);
    map.width(24);
    map.height(20);
    map.origin() = {-0.50f, -0.50f};
    map.data().assign(
        static_cast<std::size_t>(map.width()) * map.height(), -0.25f);
    return map;
}

} // namespace

int main()
{
    auto map = FlatMap();
    const auto built = go2_terrain::BuildTerrainModel(
        &map, 10.04, 1, go2_terrain::TerrainSource::kLidar);
    if (!Check(built.ok(), "flat lidar map did not build") ||
        !Check(built.model.valid(), "flat terrain model is invalid"))
        return 1;

    go2_terrain::TerrainPatch patch;
    if (!Check(built.model.SamplePatch(0.20, 0.0, 0.025, patch),
               "flat patch was not sampled") ||
        !Check(patch.all_known && std::abs(patch.slope_rad) < 1e-9,
               "flat patch lost known/plane information"))
        return 1;

    auto unknown_map = map;
    unknown_map.data()[10 * unknown_map.width() + 14] =
        std::numeric_limits<float>::quiet_NaN();
    const auto unknown_built = go2_terrain::BuildTerrainModel(
        &unknown_map, 10.04, 2, go2_terrain::TerrainSource::kLidar);
    go2_terrain::TerrainPatch unknown_patch;
    if (!Check(unknown_built.ok(), "unknown map did not build") ||
        !Check(unknown_built.model.SamplePatch(0.22, 0.0, 0.025,
                                               unknown_patch) &&
                   !unknown_patch.all_known,
               "unknown cell was silently imputed"))
        return 1;

    go2_terrain::TerrainFeasibilityConfig feasibility;
    const auto safe = go2_terrain::EvaluateFoothold(
        built.model, go2::Leg::FR, 0.20, -0.10, feasibility);
    if (!Check(safe.hard_feasible, "flat reachable foothold rejected"))
        return 1;

    auto step_map = map;
    for (std::size_t iy = 0; iy < step_map.height(); ++iy)
    {
        for (std::size_t ix = 16; ix < step_map.width(); ++ix)
            step_map.data()[iy * step_map.width() + ix] = -0.15f;
    }
    const auto step_built = go2_terrain::BuildTerrainModel(
        &step_map, 10.04, 3, go2_terrain::TerrainSource::kLidar);
    const auto edge = go2_terrain::EvaluateFoothold(
        step_built.model, go2::Leg::FR, 0.30, -0.10, feasibility);
    if (!Check(!edge.hard_feasible &&
                   edge.reject_reason ==
                       go2_terrain::FootholdRejectReason::kSurfaceStep,
               "step edge was accepted as one safe patch"))
        return 1;

    const go2::Vec3 swing_start{0.18, -0.10, -0.25};
    const go2::Vec3 swing_end{0.28, -0.10, -0.25};
    double clearance = 0.0;
    if (!Check(go2_terrain::CheckSwingClearance(
                   built.model, swing_start, swing_end, 0.03, clearance),
               "valid swept swing was rejected") ||
        !Check(!go2_terrain::CheckSwingClearance(
                   built.model, {0.18, -0.10, -0.32},
                   {0.28, -0.10, -0.32}, 0.03, clearance),
               "low swing was accepted"))
        return 1;

    go2_terrain::TerrainPlannerInput input;
    input.terrain = &built.model;
    input.state_stamp_s = 10.04;
    input.base_position_world = {0.0, 0.0, 0.0};
    input.base_height_m = 0.42;
    input.current_contact = {true, false, false, true};
    input.current_feet_base = {
        go2::Vec3{0.20, -0.10, -0.25},
        go2::Vec3{0.20, 0.10, -0.25},
        go2::Vec3{-0.20, -0.10, -0.25},
        go2::Vec3{-0.20, 0.10, -0.25}};
    input.nominal_feet_base = input.current_feet_base;
    for (std::size_t k = 0; k < 8; ++k)
    {
        input.planned_contact[k] = {true, false, false, true};
        if (k >= 2)
            input.planned_contact[k] = {true, true, true, true};
    }
    go2_terrain::TerrainPlannerConfig planner_config;
    planner_config.sensor_only = true;
    planner_config.allow_actuation = false;
    go2_terrain::TerrainPlanner planner(planner_config);
    const auto planned = planner.Build(input, 7);
    if (!Check(!planned.publishable &&
                   planned.plan.status ==
                       go2_terrain::TerrainPlanStatus::kDegraded,
               "sensor-only planner became actuation-capable") ||
        !Check(planned.candidate_counts[1] > 0 &&
                   planned.selected[1].hard_feasible,
               "planner did not select a feasible touchdown candidate"))
        return 1;

    planner_config.sensor_only = false;
    planner_config.allow_actuation = true;
    go2_terrain::TerrainPlanner actuation_planner(planner_config);
    const auto actuation_plan = actuation_planner.Build(input, 8);
    if (!Check(actuation_plan.publishable && actuation_plan.plan.valid(),
               "actuation planner did not publish a valid plan") ||
        !Check(actuation_plan.plan.committed_touchdowns > 0,
               "valid planner did not commit a touchdown") ||
        !Check(std::isfinite(actuation_plan.plan.min_edge_margin_m) &&
                   std::isfinite(actuation_plan.plan.min_support_margin_m),
               "planner validity metrics are not finite") ||
        !Check(actuation_plan.plan.body_reference[0].yaw_rad == 0.0,
               "planner did not preserve body yaw reference"))
        return 1;

    go2_terrain::TerrainMotionPlan atomic_plan;
    atomic_plan.plan_id = 1;
    atomic_plan.map_epoch = 1;
    atomic_plan.state_stamp_s = 1.0;
    atomic_plan.generated_at_s = 1.0;
    atomic_plan.valid_until_s = 2.0;
    atomic_plan.frame_id = "base_link";
    atomic_plan.status = go2_terrain::TerrainPlanStatus::kValid;
    atomic_plan.horizon_knots = 1;
    atomic_plan.body_reference[0].valid = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        atomic_plan.planned_contact[0][leg] = true;
        atomic_plan.predicted_foothold[0][leg].valid = true;
        atomic_plan.predicted_foothold[0][leg].position_world =
            input.current_feet_base[leg];
    }
    go2_terrain::TerrainPlanStore store;
    store.Publish(atomic_plan);
    const auto loaded = store.LoadUsable(1.5);
    if (!Check(loaded && loaded->plan_id == 1 && loaded->map_epoch == 1,
               "terrain plan was not atomically published"))
        return 1;

    std::cout << "Terrain model, feasibility, planner, and atomic plan checks passed.\n";
    return 0;
}
