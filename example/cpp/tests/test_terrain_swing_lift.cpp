#include "terrain_swing_lift.h"
#include "raibert_trot_kernel.h"

#include <cmath>

namespace
{

bool Near(double actual, double expected)
{
    return std::isfinite(actual) &&
        std::abs(actual - expected) < 1.0e-12;
}

}  // namespace

int main()
{
    // Runtime scheduler lift is the kernel lift; the static CLI seed must not
    // suppress the planner's terrain clearance request.
    if (!Near(go2_trot::TerrainKernelNominalLift(
            true, 0.200, 0.035, 0.035), 0.035) ||
        !Near(go2_trot::TerrainExtraLift(0.080, 0.035), 0.045))
        return 1;

    // Non-runtime gait modes retain the historical max convention.
    if (!Near(go2_trot::TerrainKernelNominalLift(
            false, 0.200, 0.035, 0.035), 0.200) ||
        !Near(go2_trot::TerrainExtraLift(0.080, 0.200), 0.0) ||
        !Near(go2_trot::TerrainKernelNominalLift(
            false, 0.035, 0.080, 0.035), 0.080))
        return 1;

    // Invalid lift provenance is explicit and cannot add an unbounded offset.
    if (!std::isnan(go2_trot::TerrainExtraLift(-1.0, 0.035)) ||
        !std::isnan(go2_trot::TerrainExtraLift(NAN, 0.035)) ||
        !std::isnan(go2_trot::TerrainExtraLift(0.080, NAN)) ||
        !std::isnan(go2_trot::TerrainKernelNominalLift(
            true, 0.200, 0.035, NAN)) ||
        !std::isnan(go2_trot::TerrainKernelNominalLift(
            false, NAN, 0.035, 0.035)))
        return 1;

    // The kernel result carries the post-slew value used for this tick.
    go2_control::GaitKernelParams gait{};
    gait.period_s = 0.50;
    gait.duty_factor = 0.75;
    gait.step_length_m = 0.10;
    gait.foot_lift_m = 0.200;
    gait.blend_duration_s = 0.80;
    go2_control::RaibertTrotKernel kernel({gait, 0.20, 0.025, 0, false});
    kernel.SetGaitFootLift(0.035);
    go2_control::GaitKernelRequest request{};
    request.gait_time_s = 0.0;
    go2_control::GaitKernelResult result{};
    if (!kernel.Compute(request, result) ||
        !Near(result.effective_foot_lift_m, 0.035))
        return 1;
    kernel.SetGaitFootLift(0.050);
    request.gait_time_s = 0.50;
    if (!kernel.Compute(request, result) ||
        !Near(result.effective_foot_lift_m, 0.041))
        return 1;
    return 0;
}
