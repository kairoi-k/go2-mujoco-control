#include <array>
#include <cmath>
#include <iostream>

#include "terrain_plan_execution_adapter.h"

namespace {

class SpyKernel final : public go2_control::LocomotionKernel
{
public:
    const char *Name() const noexcept override { return "spy"; }
    bool Compute(const go2_control::GaitKernelRequest &,
                 go2_control::GaitKernelResult &) override
    {
        return true;
    }

    void SetGaitPattern(go2_control::GaitPattern value) override
    {
        pattern = value;
        pattern_set = true;
    }
    void SetGaitPeriod(double value) override
    {
        period_s = value;
        period_set = true;
    }
    void SetGaitDuty(double value) override
    {
        duty = value;
        duty_set = true;
    }
    void SetGaitStepLength(double value) override
    {
        step_m = value;
        step_set = true;
    }
    void SetGaitFootLift(double value) override
    {
        lift_m = value;
        lift_set = true;
    }

    go2_control::GaitPattern pattern = go2_control::GaitPattern::kCrawl;
    double period_s = -1.0;
    double duty = -1.0;
    double step_m = -1.0;
    double lift_m = -1.0;
    bool pattern_set = false;
    bool period_set = false;
    bool duty_set = false;
    bool step_set = false;
    bool lift_set = false;
};

bool Near(double a, double b)
{
    return std::abs(a - b) < 1.0e-12;
}

} // namespace

int main()
{
    go2_terrain::TerrainPlanExecutionAdapter adapter(true, 0.10);
    const std::array<bool, go2::kLegCount> measured_support{
        true, true, true, true};

    constexpr double kFallbackPeriodS = 0.50;
    constexpr double kFallbackDuty = 0.75;
    constexpr double kFallbackStepM = 0.15;
    constexpr double kFallbackLiftM = 0.08;
    const auto fallback = adapter.Update(
        nullptr, 1.0, true, measured_support,
        go2_control::GaitPattern::kDiagonalTrot,
        kFallbackPeriodS, kFallbackDuty, kFallbackStepM, kFallbackLiftM);

    if (fallback.using_plan || !fallback.request.valid ||
        !fallback.request.fallback)
    {
        std::cerr << "adapter did not produce a nominal fallback request\n";
        return 1;
    }

    SpyKernel kernel;
    go2_control::GaitKernelRequest request;
    std::array<go2::Vec3, go2::kLegCount> neutral_feet{};
    adapter.ApplyToKernel(kernel, request, 1.0, neutral_feet);

    if (request.has_execution_request)
    {
        std::cerr << "fallback leaked into timed terrain execution\n";
        return 1;
    }
    if (!kernel.pattern_set || !kernel.period_set || !kernel.duty_set ||
        !kernel.step_set || !kernel.lift_set)
    {
        std::cerr << "fallback did not overwrite stale terrain gait parameters\n";
        return 1;
    }
    if (kernel.pattern != go2_control::GaitPattern::kDiagonalTrot ||
        !Near(kernel.period_s, kFallbackPeriodS) ||
        !Near(kernel.duty, kFallbackDuty) ||
        !Near(kernel.step_m, kFallbackStepM) ||
        !Near(kernel.lift_m, kFallbackLiftM))
    {
        std::cerr << "fallback restored the wrong nominal gait parameters\n";
        return 1;
    }

    return 0;
}
