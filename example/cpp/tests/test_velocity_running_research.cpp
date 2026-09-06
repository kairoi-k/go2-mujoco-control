#include "velocity_command.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
namespace
{
constexpr double kTolerance = 1.0e-12;
void UnsetResearchEnvironment()
{
    ::unsetenv("TROT_RESEARCH_RUNNING_PERIOD_S");
    ::unsetenv("TROT_RESEARCH_RUNNING_LIFT_FLOOR_M");
}
void SetResearchEnvironment(const char *period, const char *lift_floor)
{
    if (period != nullptr)
        ::setenv("TROT_RESEARCH_RUNNING_PERIOD_S", period, 1);
    else
        ::unsetenv("TROT_RESEARCH_RUNNING_PERIOD_S");
    if (lift_floor != nullptr)
        ::setenv("TROT_RESEARCH_RUNNING_LIFT_FLOOR_M", lift_floor, 1);
    else
        ::unsetenv("TROT_RESEARCH_RUNNING_LIFT_FLOOR_M");
}
go2_trot::ContinuousVelocityGaitResearchConfig LoadConfig()
{
    go2_trot::ContinuousVelocityGaitResearchConfig config;
    std::string error;
    assert(go2_trot::LoadContinuousVelocityGaitResearchConfigFromEnvironment(
        config, &error));
    assert(error.empty());
    return config;
}
void AssertInvalid(const char *period, const char *lift_floor)
{
    SetResearchEnvironment(period, lift_floor);
    go2_trot::ContinuousVelocityGaitResearchConfig config;
    std::string error;
    assert(!go2_trot::LoadContinuousVelocityGaitResearchConfigFromEnvironment(
        config, &error));
    assert(!error.empty());
}
void AssertScheduleEqual(
    const go2_trot::ContinuousVelocityGaitSchedule &left,
    const go2_trot::ContinuousVelocityGaitSchedule &right)
{
    assert(std::abs(left.period_s - right.period_s) <= kTolerance);
    assert(std::abs(left.duty_factor - right.duty_factor) <= kTolerance);
    assert(std::abs(left.step_length_m - right.step_length_m) <= kTolerance);
    assert(std::abs(left.foot_lift_m - right.foot_lift_m) <= kTolerance);
    assert(std::string(left.regime) == std::string(right.regime));
}
}  // namespace
int main()
{
    // Unset research knobs preserve the old two-argument schedule exactly.
    UnsetResearchEnvironment();
    const auto baseline_config = LoadConfig();
    assert(std::abs(baseline_config.running_period_s - 0.14) <= kTolerance);
    assert(std::abs(baseline_config.running_lift_floor_m) <= kTolerance);
    go2_trot::ContinuousVelocityGaitScheduler baseline_scheduler;
    assert(baseline_scheduler.ConfigureResearch(baseline_config));
    const auto baseline = baseline_scheduler.Step(1.0, 0.002);
    const auto historical = go2_trot::ScheduleContinuousVelocityGait(1.0, false);
    AssertScheduleEqual(baseline, historical);
    // Period ablation preserves duty and speed while recomputing stride.
    SetResearchEnvironment("0.28", nullptr);
    const auto period_config = LoadConfig();
    go2_trot::ContinuousVelocityGaitScheduler period_scheduler;
    assert(period_scheduler.ConfigureResearch(period_config));
    const auto period_schedule = period_scheduler.Step(1.0, 0.002);
    assert(std::abs(period_schedule.period_s - 0.28) <= kTolerance);
    assert(std::abs(period_schedule.duty_factor - 0.44) <= kTolerance);
    assert(std::abs(period_schedule.step_length_m - (1.0 * 0.28 / 0.88)) <= kTolerance);
    assert(std::abs(period_schedule.foot_lift_m - historical.foot_lift_m) <= kTolerance);
    // Lift-floor ablation is independent of period and only raises lift.
    SetResearchEnvironment(nullptr, "0.10");
    const auto lift_config = LoadConfig();
    go2_trot::ContinuousVelocityGaitScheduler lift_scheduler;
    assert(lift_scheduler.ConfigureResearch(lift_config));
    const auto lift_schedule = lift_scheduler.Step(1.0, 0.002);
    assert(std::abs(lift_schedule.period_s - 0.14) <= kTolerance);
    assert(std::abs(lift_schedule.duty_factor - 0.44) <= kTolerance);
    assert(std::abs(lift_schedule.step_length_m - (1.0 * 0.14 / 0.88)) <= kTolerance);
    assert(std::abs(lift_schedule.foot_lift_m - 0.10) <= kTolerance);
    // Both knobs compose without changing the fixed running duty factor.
    SetResearchEnvironment("0.28", "0.10");
    const auto combined_config = LoadConfig();
    go2_trot::ContinuousVelocityGaitScheduler combined_scheduler;
    assert(combined_scheduler.ConfigureResearch(combined_config));
    const auto combined = combined_scheduler.Step(1.0, 0.002);
    assert(std::abs(combined.period_s - 0.28) <= kTolerance);
    assert(std::abs(combined.duty_factor - 0.44) <= kTolerance);
    assert(std::abs(combined.step_length_m - (1.0 * 0.28 / 0.88)) <= kTolerance);
    assert(std::abs(combined.foot_lift_m - 0.10) <= kTolerance);
    // Qualified low-speed support remains the historical 0.50/0.75/0.035
    // schedule even when running research overrides are configured.
    for (int i = 0; i < 499; ++i)
        assert(std::abs(combined_scheduler.Step(0.30, 0.002).period_s - 0.28) <= kTolerance);
    const auto low_speed = combined_scheduler.Step(0.30, 0.002);
    assert(std::abs(low_speed.period_s - 0.50) <= kTolerance);
    assert(std::abs(low_speed.duty_factor - 0.75) <= kTolerance);
    assert(std::abs(low_speed.foot_lift_m - 0.035) <= kTolerance);
    assert(std::abs(low_speed.step_length_m - 0.10) <= kTolerance);
    const auto running_again = combined_scheduler.Step(0.60, 0.002);
    assert(std::abs(running_again.period_s - 0.28) <= kTolerance);
    assert(std::abs(running_again.foot_lift_m - 0.10) <= kTolerance);
    // The public setter is fail-closed even when called without env parsing.
    const auto preserved = combined_scheduler.ResearchConfig();
    auto invalid_config = preserved;
    invalid_config.running_period_s = std::numeric_limits<double>::quiet_NaN();
    assert(!combined_scheduler.ConfigureResearch(invalid_config));
    assert(std::abs(combined_scheduler.ResearchConfig().running_period_s -
                    preserved.running_period_s) <= kTolerance);
    // Parsing is fail-closed for malformed, nonfinite, empty, or unsafe values.
    AssertInvalid("nan", nullptr);
    AssertInvalid("inf", nullptr);
    AssertInvalid("0.28trailing", nullptr);
    AssertInvalid("", nullptr);
    AssertInvalid("0", nullptr);
    AssertInvalid("0.04", nullptr);
    AssertInvalid("-0.1", nullptr);
    AssertInvalid(nullptr, "inf");
    AssertInvalid(nullptr, "-0.01");
    AssertInvalid(nullptr, "0.51");
    UnsetResearchEnvironment();
    return 0;
}
