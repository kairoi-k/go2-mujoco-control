#include <cmath>
#include <iostream>
#include <limits>

#include "raibert_footstep_planner.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckNominalPlacement()
{
    const go2_control::RaibertFootstepPlannerParams params{};
    const go2_control::RaibertFootstepPlannerInput input{
        0.105, true};
    go2_control::RaibertFootstepPlannerOutput output{};

    if (!go2_control::PlanRaibertTouchdown(params, input, output))
        return false;
    return Near(output.nominal_velocity_x_mps, 0.105) &&
           Near(output.velocity_error_x_mps, 0.0) &&
           Near(output.adjustment_m, 0.0) &&
           Near(output.touchdown_x_m, 0.042);
}

bool CheckCorrectionSign()
{
    const go2_control::RaibertFootstepPlannerParams params{};
    go2_control::RaibertFootstepPlannerOutput too_slow{};
    go2_control::RaibertFootstepPlannerOutput too_fast{};

    if (!go2_control::PlanRaibertTouchdown(
            params, {0.05, true}, too_slow) ||
        !go2_control::PlanRaibertTouchdown(
            params, {0.15, true}, too_fast))
    {
        return false;
    }
    return too_slow.adjustment_m < 0.0 &&
           too_fast.adjustment_m > 0.0 &&
           too_slow.touchdown_x_m < 0.042 &&
           too_fast.touchdown_x_m > 0.042;
}

bool CheckCorrectionClamp()
{
    const go2_control::RaibertFootstepPlannerParams params{};
    go2_control::RaibertFootstepPlannerOutput output{};
    if (!go2_control::PlanRaibertTouchdown(
            params, {-1.0, true}, output))
    {
        return false;
    }
    return Near(output.adjustment_m, -params.max_adjustment_m);
}

bool CheckMissingVelocityIsDeterministic()
{
    const go2_control::RaibertFootstepPlannerParams params{};
    go2_control::RaibertFootstepPlannerOutput output{};
    return go2_control::PlanRaibertTouchdown(
               params, {}, output) &&
           Near(output.velocity_error_x_mps, 0.0) &&
           Near(output.adjustment_m, 0.0);
}

bool CheckInvalidInputIsRejected()
{
    const go2_control::RaibertFootstepPlannerParams params{};
    go2_control::RaibertFootstepPlannerOutput output{};
    const go2_control::RaibertFootstepPlannerInput invalid{
        std::numeric_limits<double>::quiet_NaN(), true};
    return !go2_control::PlanRaibertTouchdown(params, invalid, output);
}

} // namespace

int main()
{
    if (!CheckNominalPlacement() ||
        !CheckCorrectionSign() ||
        !CheckCorrectionClamp() ||
        !CheckMissingVelocityIsDeterministic() ||
        !CheckInvalidInputIsRejected())
    {
        std::cerr << "Raibert footstep planner checks failed\n";
        return 1;
    }

    std::cout << "Raibert footstep planner checks passed.\n";
    return 0;
}
