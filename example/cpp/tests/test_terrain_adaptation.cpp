#include <cmath>
#include <iostream>
#include <limits>

#include "terrain/terrain_adaptation.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

go2_control::terrain::HeightMap MakeMap()
{
    go2_control::terrain::HeightMap map(-1.0, -0.8, 0.02, 200, 80);
    for (std::size_t iy = 0; iy < map.ny(); ++iy)
    {
        for (std::size_t ix = 0; ix < map.nx(); ++ix)
            map.SetCell(ix, iy, 0.0, true);
    }
    return map;
}

bool CheckFlatPlacement()
{
    using namespace go2_control::terrain;
    const HeightMap map = MakeMap();
    TerrainFootholdRequest request{};
    request.leg = go2::Leg::FR;
    request.base_world_x_m = 0.50;
    request.base_world_y_m = 0.0;
    request.base_world_z_m = 0.35;
    request.nominal_body_x_m = 0.1934;
    request.nominal_body_y_m = -0.14;
    request.nominal_body_z_m = -0.35;
    request.reference_foot_world_z_m = 0.0;
    TerrainFootholdPlannerParams params{};
    params.height_reference_m = 0.0;
    TerrainFootholdOutput output{};
    if (!PlanTerrainFoothold(params, map, request, output))
        return false;
    return output.status == TerrainPlanStatus::kValid &&
           Near(output.world_z_m, 0.0) &&
           Near(output.surface_height_range_m, 0.0) &&
           Near(output.slope_rad, 0.0);
}

bool CheckBarrierLandingUsesTopSurface()
{
    using namespace go2_control::terrain;
    HeightMap map = MakeMap();
    for (std::size_t iy = 0; iy < map.ny(); ++iy)
    {
        for (std::size_t ix = 0; ix < map.nx(); ++ix)
        {
            const double x = map.x_min_m() +
                (static_cast<double>(ix) + 0.5) * map.resolution_m();
            if (x >= 0.80 && x < 1.00)
                map.SetCell(ix, iy, 0.12, true);
        }
    }
    TerrainFootholdRequest request{};
    request.leg = go2::Leg::FR;
    request.base_world_x_m = 0.70;
    request.base_world_y_m = 0.0;
    request.base_world_z_m = 0.35;
    request.nominal_body_x_m = 0.1934;
    request.nominal_body_y_m = -0.14;
    request.nominal_body_z_m = -0.35;
    request.reference_foot_world_z_m = 0.0;
    TerrainFootholdPlannerParams params{};
    params.height_reference_m = 0.12;
    TerrainFootholdOutput output{};
    if (!PlanTerrainFoothold(params, map, request, output))
        return false;
    // The selected patch is inside the physical barrier top, not on its riser.
    return Near(output.world_z_m, 0.12, 1e-6) &&
           output.world_x_m > 0.835 && output.world_x_m < 0.965 &&
           output.surface_height_range_m <= params.max_surface_height_range_m;
}

bool CheckUnknownSurfaceIsRejected()
{
    using namespace go2_control::terrain;
    HeightMap map = MakeMap();
    for (std::size_t iy = 0; iy < map.ny(); ++iy)
    {
        for (std::size_t ix = 0; ix < map.nx(); ++ix)
            map.SetCell(ix, iy, 0.0, false);
    }
    TerrainFootholdRequest request{};
    request.leg = go2::Leg::FR;
    request.base_world_x_m = 0.5;
    request.base_world_y_m = 0.0;
    request.base_world_z_m = 0.35;
    request.nominal_body_x_m = 0.1934;
    request.nominal_body_y_m = -0.14;
    request.nominal_body_z_m = -0.35;
    TerrainFootholdOutput output{};
    return !PlanTerrainFoothold(
               TerrainFootholdPlannerParams{}, map, request, output) &&
           output.status == TerrainPlanStatus::kUnknownSurface;
}

bool CheckStaircaseSequenceIsReachable()
{
    using namespace go2_control::terrain;
    const StaircaseSpec spec{0.80, 0.24, 0.10, 4, 0.75, 0.0};
    HeightMap map(-1.0, -0.8, 0.02, 200, 80);
    if (!FillStaircaseHeightMap(spec, map))
        return false;

    const double base_x[] = {0.70, 0.95, 1.20, 1.45, 1.80};
    const double expected_z[] = {0.10, 0.20, 0.30, 0.40, 0.40};
    double previous_z = 0.0;
    for (std::size_t i = 0; i < 5; ++i)
    {
        TerrainFootholdRequest request{};
        request.leg = go2::Leg::FR;
        request.base_world_x_m = base_x[i];
        request.base_world_y_m = 0.0;
        request.base_world_z_m = 0.35 + previous_z;
        request.nominal_body_x_m = 0.1934;
        request.nominal_body_y_m = -0.14;
        request.nominal_body_z_m = -0.35;
        request.reference_foot_world_z_m = previous_z;
        TerrainFootholdPlannerParams params{};
        params.height_reference_m = expected_z[i];
        TerrainFootholdOutput output{};
        if (!PlanTerrainFoothold(params, map, request, output) ||
            !Near(output.world_z_m, expected_z[i], 1e-6) ||
            output.surface_height_range_m > params.max_surface_height_range_m)
        {
            return false;
        }
        previous_z = output.world_z_m;
    }
    return Near(previous_z, 0.40, 1e-6);
}

bool CheckStaircaseReferencePhasesAndSlew()
{
    using namespace go2_control::terrain;
    const StaircaseSpec spec{0.80, 0.24, 0.10, 4, 0.75, 0.0};
    TerrainMotionReference approach{};
    TerrainMotionReference climb{};
    TerrainMotionReference exit{};
    if (!PlanStaircaseReference(spec, 0.0, 0.20, 0.60, 0.12, 0.06, approach) ||
        !PlanStaircaseReference(spec, 0.70, 0.25, 0.60, 0.12, 0.06, climb) ||
        !PlanStaircaseReference(spec, 1.90, 0.10, 0.60, 0.12, 0.06, exit))
    {
        return false;
    }
    if (approach.phase != StaircasePhase::kApproach ||
        climb.phase != StaircasePhase::kClimb ||
        exit.phase != StaircasePhase::kExit ||
        climb.vx_mps > 0.35 + 1e-12 ||
        climb.pitch_rad <= 0.0 ||
        climb.foot_lift_m < 0.14 - 1e-12 ||
        exit.pitch_rad != 0.0 || exit.ground_height_m < 0.39)
    {
        return false;
    }

    TerrainMotionSlewLimits limits{};
    TerrainMotionReference blended{};
    if (!SlewTerrainMotionReference(
            approach, climb, limits, 0.01, blended))
    {
        return false;
    }
    return Near(blended.vx_mps, 0.592) &&
           Near(blended.ground_height_m, 0.002) &&
           blended.pitch_rad <= limits.pitch_radps * 0.01 + 1e-12 &&
           blended.foot_lift_m <=
               approach.foot_lift_m + limits.foot_lift_mps * 0.01 + 1e-12;
}

bool CheckParameterSweepAndSafetyRejection()
{
    using namespace go2_control::terrain;
    constexpr double nominal_x = 0.1934;
    constexpr double nominal_y = -0.14;
    for (const double tread : {0.20, 0.24, 0.28})
    {
        for (const double riser : {0.05, 0.08, 0.12})
        {
            const StaircaseSpec spec{0.80, tread, riser, 3, 0.75, 0.0};
            HeightMap map(-1.0, -0.8, 0.02, 200, 80);
            if (!FillStaircaseHeightMap(spec, map))
                return false;
            TerrainFootholdPlannerParams params{};
            params.height_reference_m = 0.0;
            double previous_z = 0.0;
            for (int step = 0; step < spec.step_count; ++step)
            {
                const double target_x = spec.start_x_m +
                    (static_cast<double>(step) + 0.5) * spec.tread_depth_m;
                TerrainFootholdRequest request{};
                request.leg = go2::Leg::FR;
                request.base_world_x_m = target_x - nominal_x;
                request.base_world_z_m = 0.35 + previous_z;
                request.nominal_body_x_m = nominal_x;
                request.nominal_body_y_m = nominal_y;
                request.nominal_body_z_m = -0.35;
                request.reference_foot_world_z_m = previous_z;
                params.height_reference_m =
                    riser * static_cast<double>(step + 1);
                TerrainFootholdOutput output{};
                if (!PlanTerrainFoothold(params, map, request, output) ||
                    !Near(output.world_z_m, params.height_reference_m, 1e-6))
                {
                    return false;
                }
                previous_z = output.world_z_m;
            }
        }
    }

    // A riser larger than the configured vertical reach must fail closed.
    const StaircaseSpec too_high{0.80, 0.24, 0.20, 1, 0.75, 0.0};
    HeightMap high_map(-1.0, -0.8, 0.02, 200, 80);
    if (!FillStaircaseHeightMap(too_high, high_map))
        return false;
    TerrainFootholdPlannerParams strict{};
    strict.candidate_dx_m.fill(0.0);
    strict.candidate_dy_m.fill(0.0);
    TerrainFootholdRequest high_request{};
    high_request.leg = go2::Leg::FR;
    high_request.base_world_x_m = 0.80 + 0.12 - nominal_x;
    high_request.base_world_z_m = 0.35;
    high_request.nominal_body_x_m = nominal_x;
    high_request.nominal_body_y_m = nominal_y;
    high_request.nominal_body_z_m = -0.35;
    high_request.reference_foot_world_z_m = 0.0;
    TerrainFootholdOutput high_output{};
    return !PlanTerrainFoothold(
               strict, high_map, high_request, high_output) &&
           high_output.status == TerrainPlanStatus::kStepTooHigh;
}

bool CheckApproachFsmSequence()
{
    using namespace go2_control::terrain;
    TerrainApproachFsm fsm;
    double scale = 1.0;
    double pitch = 0.0;
    if (!fsm.Step(false, false, false, 0.0, 0.0, 0.002, scale, pitch) ||
        fsm.phase != TerrainApproachPhase::kCruise || scale != 1.0)
        return false;
    if (!fsm.Step(true, false, false, 0.0, 0.0, 0.002, scale, pitch) ||
        fsm.phase != TerrainApproachPhase::kCreep ||
        std::abs(scale - 0.35) > 1e-12)
        return false;
    if (!fsm.Step(true, true, false, 0.05, 0.0, 0.002, scale, pitch) ||
        fsm.phase != TerrainApproachPhase::kMount)
        return false;
    if (!fsm.Step(true, true, true, 0.05, 0.05, 0.002, scale, pitch) ||
        fsm.phase != TerrainApproachPhase::kTraverse || scale != 1.0)
        return false;
    if (!fsm.Step(false, false, false, 0.0, 0.0, 0.002, scale, pitch) ||
        fsm.phase != TerrainApproachPhase::kCruise)
        return false;
    TerrainApproachFsm slew;
    slew.Step(false, true, true, 0.08, 0.0, 0.10, scale, pitch);
    const double expected = std::min(0.06, std::atan2(0.08, 0.40));
    return std::abs(pitch - expected) < 1e-9;
}

}  // namespace

int main()
{
    if (!CheckFlatPlacement() ||
        !CheckBarrierLandingUsesTopSurface() ||
        !CheckUnknownSurfaceIsRejected() ||
        !CheckStaircaseSequenceIsReachable() ||
        !CheckStaircaseReferencePhasesAndSlew() ||
        !CheckParameterSweepAndSafetyRejection() ||
        !CheckApproachFsmSequence())
    {
        std::cerr << "terrain adaptation checks failed\n";
        return 1;
    }
    std::cout << "terrain adaptation checks passed: flat, barrier, unknown, stairs, slew.\n";
    return 0;
}
