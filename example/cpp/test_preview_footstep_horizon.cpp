#include <cmath>
#include <iostream>

#include "preview_footstep_horizon.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckSingleStepMatchesRaibert()
{
    go2_control::PreviewFootstepHorizonParams params{};
    params.n_steps = 1;
    go2_control::PreviewFootstepHorizonOutput output{};
    if (!go2_control::PlanPreviewFootstepHorizon(
            params, {0.105, true}, output))
        return false;
    return output.n_steps == 1 &&
           Near(output.nominal_velocity_x_mps, 0.105) &&
           Near(output.touchdown_x_m[0], 0.042);
}

bool CheckNominalHorizonKeepsRaibertFoothold()
{
    go2_control::PreviewFootstepHorizonParams params{};
    params.n_steps = 4;
    go2_control::PreviewFootstepHorizonOutput output{};
    if (!go2_control::PlanPreviewFootstepHorizon(
            params, {0.105, true}, output))
        return false;
    if (output.n_steps != 4)
        return false;
    if (!Near(output.nominal_velocity_x_mps, 0.105))
        return false;
    for (int i = 0; i < 4; ++i)
    {
        if (!Near(output.touchdown_x_m[static_cast<std::size_t>(i)], 0.042))
            return false;
    }
    return Near(output.terminal_velocity_x_mps, 0.105, 1e-9);
}

bool CheckSlowFirstStepIsRearward()
{
    go2_control::PreviewFootstepHorizonParams params{};
    params.n_steps = 4;
    go2_control::RaibertFootstepPlannerOutput greedy{};
    if (!go2_control::PlanRaibertTouchdown(
            params.raibert, {0.05, true}, greedy))
        return false;
    go2_control::PreviewFootstepHorizonOutput output{};
    if (!go2_control::PlanPreviewFootstepHorizon(
            params, {0.05, true}, output))
        return false;
    return output.touchdown_x_m[0] < 0.042 &&
           output.touchdown_x_m[0] <= greedy.touchdown_x_m + 1e-12 &&
           std::abs(output.touchdown_x_m[0] - 0.042) <=
               params.raibert.max_adjustment_m + 1e-12;
}

bool CheckInvalidHorizonRejected()
{
    go2_control::PreviewFootstepHorizonParams params{};
    params.n_steps = 0;
    go2_control::PreviewFootstepHorizonOutput output{};
    return !go2_control::PlanPreviewFootstepHorizon(params, {}, output);
}

}  // namespace

int main()
{
    if (!CheckSingleStepMatchesRaibert() ||
        !CheckNominalHorizonKeepsRaibertFoothold() ||
        !CheckSlowFirstStepIsRearward() ||
        !CheckInvalidHorizonRejected())
    {
        std::cerr << "preview footstep horizon checks failed\n";
        return 1;
    }
    std::cout << "preview footstep horizon checks passed.\n";
    return 0;
}
