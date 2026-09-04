#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "contact_state_filter.h"

namespace
{

bool CheckHysteresisBand()
{
    const go2_control::HystereticContactParams params{5.0, 3.0};
    bool current = false;
    if (!go2_control::UpdateHystereticContact(
            false, 4.0, params, current) ||
        current)
    {
        return false;
    }
    if (!go2_control::UpdateHystereticContact(
            false, 5.0, params, current) ||
        !current)
    {
        return false;
    }
    if (!go2_control::UpdateHystereticContact(
            true, 4.0, params, current) ||
        !current)
    {
        return false;
    }
    return go2_control::UpdateHystereticContact(
               true, 3.0, params, current) &&
           !current;
}

bool CheckFusionGuard()
{
    go2_control::MeasuredContactFusion fusion;
    const std::array<bool, go2::kLegCount> stable{true, true, true, false};
    const std::array<bool, go2::kLegCount> dropout{true, true, false, false};
    auto result = fusion.Update(stable, true, true);
    if (!result.measured_valid || result.measured_count != 3 ||
        result.guard_active || result.fused_contact != stable)
        return false;
    // A single noisy-off sample retains the last robust support.
    result = fusion.Update(dropout, true, true);
    if (!result.guard_active || result.fallback_stage !=
            go2_control::ContactFusionFallbackStage::kN ||
        result.fused_contact != stable || result.measured_count != 2)
        return false;
    result = fusion.Update(stable, true, true);
    if (!result.recovery || result.guard_active)
        return false;
    // Consecutive loss expires the one-tick filter grace without using a
    // planned mask as a substitute.
    result = fusion.Update(dropout, true, true);
    result = fusion.Update(dropout, true, true);
    result = fusion.Update(dropout, true, true);
    if (result.fused_contact != dropout || result.grace_remaining_ticks != 0 ||
        result.fallback_stage !=
            go2_control::ContactFusionFallbackStage::kNPlus1)
        return false;
    for (int tick = 0; tick < 3; ++tick)
        result = fusion.Update(dropout, true, true);
    if (result.fallback_stage !=
            go2_control::ContactFusionFallbackStage::kNPlus5 ||
        result.fused_contact != dropout)
        return false;
    result = fusion.Update(stable, true, true);
    return result.recovery && !result.guard_active &&
        result.reason == std::string("measured-support-restored");
}

bool CheckInvalidInput()
{
    const go2_control::HystereticContactParams params{5.0, 3.0};
    bool current = true;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (go2_control::UpdateHystereticContact(
            true, nan, params, current))
    {
        return false;
    }

    const go2_control::HystereticContactParams invalid_params{3.0, 5.0};
    return !go2_control::UpdateHystereticContact(
        false, 4.0, invalid_params, current);
}

} // namespace

int main()
{
    if (!CheckHysteresisBand() || !CheckFusionGuard() || !CheckInvalidInput())
    {
        std::cerr << "Hysteretic contact filter checks failed" << std::endl;
        return 1;
    }
    std::cout << "Hysteretic contact filter checks passed." << std::endl;
    return 0;
}
