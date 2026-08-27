#include <array>
#include <algorithm>
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
    double required_lift = 0.0;
    if (!Check(go2_terrain::CheckSwingClearance(
                   built.model, swing_start, swing_end, 0.03, clearance,
                   nullptr, go2::Leg::FR, &required_lift),
               "valid swept swing was rejected") ||
        !Check(required_lift >= 0.03,
               "swept clearance did not report required lift") ||
        !Check(!go2_terrain::CheckSwingClearance(
                   built.model, {0.18, -0.10, -0.32},
                   {0.28, -0.10, -0.32}, 0.03, clearance),
               "low swing was accepted"))
        return 1;

    double step_clearance = 0.0;
    double step_required_lift = 0.0;
    double step_peak_phase = 0.0;
    double step_leading_edge_phase = 0.0;
    bool step_leading_edge_phase_valid = false;
    go2_terrain::FootholdRejectReason step_swing_reason =
        go2_terrain::FootholdRejectReason::kNone;
    if (!Check(go2_terrain::CheckSwingClearance(
                   step_built.model, {0.18, -0.10, -0.25},
                   {0.425, -0.10, -0.15}, 0.03, step_clearance,
                   &step_swing_reason, go2::Leg::FL, &step_required_lift,
                   &step_peak_phase, &step_leading_edge_phase,
                   &step_leading_edge_phase_valid),
               "10cm sensor-derived swing clearance was rejected") ||
        !Check(step_required_lift >= 0.03 &&
                   step_required_lift < 0.30 &&
                   step_peak_phase >= 0.10 && step_peak_phase <= 0.90 &&
                   step_leading_edge_phase_valid &&
                   step_leading_edge_phase >= 0.10 &&
                   step_leading_edge_phase <= 0.75 &&
                   go2_terrain::TerrainSwingPathProgress(
                       step_leading_edge_phase, true,
                       step_leading_edge_phase) > 0.10 &&
                   go2_terrain::TerrainSwingPathProgress(
                       1.0, true, step_leading_edge_phase) > 0.999,
               "10cm swing clearance geometry was invalid"))
        return 1;

    auto high_step_map = map;
    for (std::size_t iy = 0; iy < high_step_map.height(); ++iy)
    {
        for (std::size_t ix = 14; ix < high_step_map.width(); ++ix)
            high_step_map.data()[iy * high_step_map.width() + ix] = -0.10f;
    }
    const auto high_step_built = go2_terrain::BuildTerrainModel(
        &high_step_map, 10.04, 4, go2_terrain::TerrainSource::kLidar);
    auto repeated_step_map = step_map;
    for (std::size_t iy = 0; iy < repeated_step_map.height(); ++iy)
    {
        for (std::size_t ix = 20; ix < repeated_step_map.width(); ++ix)
            repeated_step_map.data()[iy * repeated_step_map.width() + ix] =
                -0.10f;
    }
    const auto repeated_step_built = go2_terrain::BuildTerrainModel(
        &repeated_step_map, 10.04, 5, go2_terrain::TerrainSource::kLidar);
    double high_step_clearance = 0.0;
    double high_step_required_lift = 0.0;
    if (!Check(go2_terrain::CheckSwingClearance(
                   high_step_built.model, {0.18, -0.10, -0.25},
                   {0.425, -0.10, -0.10}, 0.03, high_step_clearance,
                   nullptr, go2::Leg::FL, &high_step_required_lift),
               "15cm sensor-derived swing clearance was rejected") ||
        !Check(high_step_required_lift >= 0.03 &&
                   high_step_required_lift < 0.40,
               "15cm swing clearance geometry was invalid"))
        return 1;

    go2_terrain::TerrainPlannerInput input;
    input.terrain = &built.model;
    input.state_stamp_s = 10.04;
    input.base_position_world = {0.0, 0.0, 0.0};
    input.base_height_m = 0.42;
    input.contact_schedule.measured_contact = {true, false, false, true};
    input.contact_schedule.measured_valid = true;
    input.current_feet_base = {
        go2::Vec3{0.20, -0.10, -0.25},
        go2::Vec3{0.20, 0.10, -0.25},
        go2::Vec3{-0.20, -0.10, -0.25},
        go2::Vec3{-0.20, 0.10, -0.25}};
    input.nominal_feet_base = input.current_feet_base;
    for (std::size_t k = 0; k < 8; ++k)
    {
        input.contact_schedule.planned_contact[k] = {true, false, false, true};
        if (k >= 2)
            input.contact_schedule.planned_contact[k] = {true, true, true, true};
    }
    input.contact_schedule.planned_valid = true;
    go2_terrain::TerrainPlannerConfig planner_config;
    planner_config.sensor_only = true;
    planner_config.allow_actuation = false;
    go2_terrain::TerrainPlanner planner(planner_config);
    const auto planned = planner.Build(input, 7);
    if (!Check(!planned.publishable &&
                   planned.plan.status ==
                       go2_terrain::TerrainPlanStatus::kDegraded,
               "sensor-only planner became actuation-capable") ||
        !Check(planned.candidate_counts[0] > 0 &&
                   planned.candidate_counts[1] > 0 &&
                   planned.candidate_counts[2] > 0 &&
                   planned.candidate_counts[3] > 0 &&
                   !planned.selected[1].hard_feasible,
               "sensor-only planner performed actuation selection"))
        return 1;

    planner_config.sensor_only = false;
    planner_config.allow_actuation = true;
    go2_terrain::TerrainPlanner actuation_planner(planner_config);

    auto forward_step_input = input;
    forward_step_input.terrain = &step_built.model;
    forward_step_input.commanded_vx_mps = 0.30;
    forward_step_input.next_touchdown_time_valid.fill(false);
    const auto forward_step_plan = actuation_planner.Build(
        forward_step_input, 12);
    if (!Check(forward_step_plan.publishable &&
                   forward_step_plan.selected[1].hard_feasible &&
                   forward_step_plan.selected[1].foot_position.z > -0.20,
               "forward sensor-elevated foothold was not selected"))
        return 1;
    if (!Check(forward_step_plan.plan.velocity_request.valid &&
                   forward_step_plan.plan.velocity_request.is_cap &&
                   forward_step_plan.plan.velocity_request.max_vx_mps == 0.0,
               "sensor-elevated foothold did not request a velocity cap"))
        return 1;
    if (!Check(forward_step_plan.plan.body_reference[4].position.z >
                   forward_step_plan.plan.body_reference[0].position.z +
                       1.0e-4,
               "terrain body reference did not rise with planned surface"))
        return 1;

    const auto actuation_plan = actuation_planner.Build(input, 8);
    if (!Check(actuation_plan.publishable && actuation_plan.plan.valid(),
               "actuation planner did not publish a valid plan") ||
        !Check(actuation_plan.plan.committed_touchdowns > 0,
               "valid planner did not commit a touchdown") ||
        !Check(std::isfinite(actuation_plan.plan.min_edge_margin_m) &&
                   std::isfinite(actuation_plan.plan.min_support_margin_m),
               "planner validity metrics are not finite") ||
        !Check(std::all_of(
                   actuation_plan.plan.current_terrain_height_valid.begin(),
                   actuation_plan.plan.current_terrain_height_valid.end(),
                   [](bool valid) { return valid; }),
               "planner did not preserve per-leg sensor terrain heights") ||
        !Check(actuation_plan.plan.current_support_surface_valid[0] &&
                   actuation_plan.plan.current_support_surface_valid[3] &&
                   !actuation_plan.plan.current_support_surface_valid[1] &&
                   !actuation_plan.plan.current_support_surface_valid[2],
               "sensor terrain height was promoted to measured support") ||
        !Check(actuation_plan.plan.body_reference[0].yaw_rad == 0.0,
               "planner did not preserve body yaw reference") ||
        !Check(std::abs(
                   actuation_plan.plan.body_reference[7].position.z -
                   actuation_plan.plan.body_reference[0].position.z) <
                   1.0e-9,
               "flat terrain changed the body height reference") ||
        !Check(actuation_plan.selected[1].region_id <
                   actuation_plan.regions[1].size(),
               "actuation planner did not consume a safe region") ||
        !Check(std::abs(actuation_plan.selected[1].foot_position.x -
                            input.nominal_feet_base[1].x) < 1.0e-9 &&
                   std::abs(actuation_plan.selected[1].foot_position.y -
                            input.nominal_feet_base[1].y) < 1.0e-9,
               "flat safe region displaced the nominal foothold") ||
        !Check(actuation_plan.selected[1].swing_lift_m >= 0.03,
               "actuation planner did not propagate swept lift"))
        return 1;

    auto moving_body_input = input;
    moving_body_input.base_velocity_world = {0.25, 0.0, 0.0};
    const auto moving_body_plan = actuation_planner.Build(
        moving_body_input, 13);
    if (!Check(moving_body_plan.publishable &&
                   moving_body_plan.plan.body_reference[4].position.x >
                       moving_body_plan.plan.body_reference[0].position.x,
               "planner did not advance its future body reference") ||
        !Check(moving_body_plan.plan.body_reference[4].position.x > 0.0,
               "future body reference did not use measured velocity"))
        return 1;

    auto measured_support_input = input;
    auto repeated_input = forward_step_input;
    repeated_input.terrain = &repeated_step_built.model;
    repeated_input.base_velocity_world = {0.30, 0.0, 0.0};
    repeated_input.contact_schedule.measured_contact =
        {true, false, false, true};
    for (std::size_t k = 0; k < 24; ++k)
    {
        if (k < 2)
            repeated_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else if (k < 6)
            repeated_input.contact_schedule.planned_contact[k] =
                {true, true, true, true};
        else if (k < 10)
            repeated_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else
            repeated_input.contact_schedule.planned_contact[k] =
                {true, true, true, true};
    }
    repeated_input.contact_schedule.planned_valid = true;
    auto repeated_planner_config = planner_config;
    repeated_planner_config.plan_validity_s = 0.50;
    go2_terrain::TerrainPlanner repeated_planner(
        repeated_planner_config);
    const auto repeated_plan = repeated_planner.Build(
        repeated_input, 14);
    int first_repeated_touchdown = -1;
    int second_repeated_touchdown = -1;
    for (std::size_t k = 0; k < 24; ++k)
    {
        if (!repeated_plan.plan.predicted_foothold[k][1].touchdown)
            continue;
        if (first_repeated_touchdown < 0)
            first_repeated_touchdown = static_cast<int>(k);
        else if (second_repeated_touchdown < 0)
            second_repeated_touchdown = static_cast<int>(k);
    }
    if (!Check(repeated_plan.publishable &&
                   repeated_plan.plan.valid(),
               "repeated terrain plan was not publishable") ||
        !Check(first_repeated_touchdown >= 0 &&
                   second_repeated_touchdown > first_repeated_touchdown,
               "plan did not preserve repeated touchdown events") ||
        !Check(repeated_plan.plan.predicted_foothold[
                   static_cast<std::size_t>(second_repeated_touchdown)][1].valid &&
                   repeated_plan.plan.predicted_foothold[
                       static_cast<std::size_t>(second_repeated_touchdown)][1].position_world.x >
                   repeated_plan.plan.predicted_foothold[
                       static_cast<std::size_t>(first_repeated_touchdown)][1].position_world.x,
               "future touchdown did not advance its sensor foothold") ||
        !Check(repeated_plan.plan.predicted_foothold[
                   static_cast<std::size_t>(second_repeated_touchdown)][1].
                       swing_start_position_valid &&
                   std::abs(repeated_plan.plan.predicted_foothold[
                       static_cast<std::size_t>(second_repeated_touchdown)][1].
                       swing_start_position_world.x -
                       repeated_plan.plan.predicted_foothold[
                           static_cast<std::size_t>(first_repeated_touchdown)][1].
                           position_world.x) < 1.0e-9,
               "future touchdown lost its checked swing start"))
        return 1;

    measured_support_input.contact_schedule.measured_contact =
        {true, true, true, true};
    measured_support_input.current_feet_base = {
        go2::Vec3{0.214, -0.135, -0.25},
        go2::Vec3{0.214, 0.135, -0.25},
        go2::Vec3{-0.174, -0.193, -0.25},
        go2::Vec3{-0.174, 0.193, -0.25}};
    measured_support_input.nominal_feet_base =
        measured_support_input.current_feet_base;
    for (std::size_t k = 0; k < 8; ++k)
        measured_support_input.contact_schedule.planned_contact[k] =
            {true, false, false, true};
    const auto measured_support_plan =
        actuation_planner.Build(measured_support_input, 11);
    if (!Check(measured_support_plan.publishable &&
                   measured_support_plan.plan.valid() &&
                   measured_support_plan.plan.current_support_count == 4,
               "measured support geometry was rejected as planned diagonal"))
        return 1;

    auto flight_input = input;
    for (std::size_t k = 0; k < 8; ++k)
    {
        if (k < 2)
            flight_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else if (k < 4)
            flight_input.contact_schedule.planned_contact[k] =
                {false, false, false, false};
        else
            flight_input.contact_schedule.planned_contact[k] =
                {false, true, true, false};
    }
    const auto flight_plan = actuation_planner.Build(flight_input, 9);
    if (!Check(flight_plan.publishable && flight_plan.plan.valid(),
               "running-trot flight knot was rejected") ||
        !Check(flight_plan.plan.committed_touchdowns > 0,
               "flight schedule did not retain touchdown planning"))
        return 1;

    std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
        plan_indices{};
    if (!Check(go2_terrain::BuildTerrainPlanHorizonIndices(
                   flight_plan.plan, 10.08, 0.02, 0.02, 4,
                   plan_indices) &&
                   plan_indices[0] == 2 && plan_indices[3] == 5,
               "terrain plan horizon was not time-aligned") ||
        !Check(go2_terrain::BuildTerrainPlanHorizonIndices(
                   flight_plan.plan, 10.10, 0.02, 0.02, 12,
                   plan_indices) &&
                   plan_indices[0] == 3 && plan_indices[11] == 14,
               "extended terrain horizon was not time-aligned") ||
        !Check(!go2_terrain::BuildTerrainPlanHorizonIndices(
                   flight_plan.plan, 10.30, 0.02, 0.02, 12,
                   plan_indices),
               "expired terrain plan was not rejected"))
        return 1;

    auto no_support_input = flight_input;
    for (auto &contact : no_support_input.contact_schedule.planned_contact)
        contact = {false, false, false, false};
    const auto no_support_plan = actuation_planner.Build(no_support_input, 10);
    if (!Check(!no_support_plan.publishable &&
                   no_support_plan.plan.failure ==
                       go2_terrain::TerrainPlanFailure::kSupportInfeasible,
               "support-free schedule was accepted"))
        return 1;

    go2_terrain::TerrainMotionPlan atomic_plan;
    atomic_plan.plan_id = 1;
    atomic_plan.plan_epoch = 1;
    atomic_plan.map_epoch = 1;
    atomic_plan.state_stamp_s = 1.0;
    atomic_plan.generated_at_s = 1.0;
    atomic_plan.valid_until_s = 2.0;
    atomic_plan.frame_id = "base_link";
    atomic_plan.status = go2_terrain::TerrainPlanStatus::kValid;
    atomic_plan.horizon_knots = 1;
    atomic_plan.body_reference[0].valid = true;
    atomic_plan.contact_schedule = input.contact_schedule;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        atomic_plan.contact_schedule.planned_contact[0][leg] = true;
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
    if (!Check(loaded->plan_epoch == 1 &&
                   loaded->contact_schedule.measured_contact[0] &&
                   !loaded->contact_schedule.measured_contact[1],
               "planned and measured contact state was not preserved"))
        return 1;

    std::cout << "Terrain model, feasibility, planner, and atomic plan checks passed.\n";
    return 0;
}
