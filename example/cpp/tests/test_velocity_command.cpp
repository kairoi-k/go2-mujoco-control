#include "velocity_command.h"
#include "locomotion_kernel.h"
#include <cassert>
#include <cmath>
int main()
{
    go2_trot::VelocityCommandProfile profile;
    profile.points = {{0.0, 0.0}, {2.0, 2.0}, {4.0, 1.0}};
    assert(profile.Validate());
    assert(std::abs(profile.Sample(1.0) - 1.0) < 1.0e-9);
    assert(std::abs(profile.Sample(3.0) - 1.5) < 1.0e-9);
    const auto stopped = go2_trot::ScheduleContinuousVelocityGait(0.0, true);
    const auto probe = go2_trot::ScheduleContinuousVelocityGait(0.30, true);
    const auto sprint = go2_trot::ScheduleContinuousVelocityGait(3.0, false);
    assert(stopped.step_length_m == 0.0);
    assert(std::abs(stopped.period_s - 0.50) < 1.0e-9);
    assert(std::abs(stopped.duty_factor - 0.75) < 1.0e-9);
    assert(std::abs(sprint.period_s - 0.14) < 1.0e-9);
    assert(std::abs(sprint.duty_factor - 0.44) < 1.0e-9);
    assert(sprint.step_length_m > 0.0);
    assert(std::abs(probe.period_s - 0.50) < 1.0e-9);
    assert(std::abs(probe.duty_factor - 0.75) < 1.0e-9);
    assert(std::abs(probe.foot_lift_m - 0.035) < 1.0e-9);
    assert(probe.step_length_m > 0.0);
    // Without low-speed qualification the schedule keeps the validated
    // high-speed timing even inside the low-speed band.
    const auto unqualified = go2_trot::ScheduleContinuousVelocityGait(0.30, false);
    assert(std::abs(unqualified.period_s - 0.14) < 1.0e-9);
    assert(std::abs(unqualified.duty_factor - 0.44) < 1.0e-9);
    // Qualification is time-based: ramping through the low band never
    // engages the support-rich schedule, sustaining it for one second does.
    go2_trot::ContinuousVelocityGaitScheduler scheduler;
    for (int i = 0; i < 250; ++i)
    {
        const auto schedule = scheduler.Step(0.30, 0.002);
        assert(std::abs(schedule.period_s - 0.14) < 1.0e-9);
    }
    for (int i = 0; i < 300; ++i)
    {
        const auto schedule = scheduler.Step(0.30, 0.002);
        assert(std::abs(schedule.period_s - 0.50) < 1.0e-9);
    }
    const auto ramped = scheduler.Step(0.60, 0.002);
    assert(std::abs(ramped.period_s - 0.14) < 1.0e-9);
    scheduler.Reset();
    const auto after_reset = scheduler.Step(0.30, 0.002);
    assert(std::abs(after_reset.period_s - 0.14) < 1.0e-9);
    go2_trot::VelocityCommandShaper shaper;
    double previous_accel = 0.0;
    double maximum_accel = 0.0;
    double maximum_jerk = 0.0;
    for (int i = 0; i < 2500; ++i)
    {
        const auto state = shaper.Step(3.0, 0.002);
        assert(state.shaped_mps <= 3.20 + 1.0e-9);
        assert(state.shaped_mps >= -1.0e-9);
        maximum_accel = std::max(maximum_accel, state.accel_mps2);
        maximum_jerk = std::max(maximum_jerk, std::abs(state.jerk_mps3));
        assert(state.accel_mps2 <= 1.50 + 1.0e-9);
        assert(state.jerk_mps3 <= 8.0 + 1.0e-6);
        assert(state.jerk_mps3 >= -8.0 - 1.0e-6);
        previous_accel = state.accel_mps2;
    }
    assert(std::abs(previous_accel) < 1.0e-6);
    for (int i = 0; i < 2500; ++i)
    {
        const auto state = shaper.Step(0.0, 0.002);
        assert(state.shaped_mps >= -1.0e-9);
        assert(state.accel_mps2 >= -2.0 - 1.0e-9);
        assert(std::abs(state.jerk_mps3) <= 8.0 + 1.0e-6);
    }
    assert(maximum_accel > 1.0);
    assert(maximum_jerk > 0.0);
    return 0;
}
