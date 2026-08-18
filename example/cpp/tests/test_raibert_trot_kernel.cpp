#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "raibert_trot_kernel.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-10)
{
    return std::abs(actual - expected) <= tolerance;
}
go2_control::GaitKernelRequest Request(
    double time_s,
    double velocity_x_mps)
{
    go2_control::GaitKernelRequest request{};
    request.gait_time_s = time_s;
    request.body_velocity_x_mps = velocity_x_mps;
    request.have_body_velocity = true;
    return request;
}

go2_control::RaibertTrotKernel MakeKernel()
{
    go2_control::GaitKernelParams gait{};
    gait.period_s = 0.8;
    gait.duty_factor = 0.75;
    gait.step_length_m = 0.084;
    gait.direction_sign = 1.0;
    gait.foot_lift_m = 0.035;
    gait.blend_duration_s = 0.8;
    return go2_control::RaibertTrotKernel(
        {gait, 0.20, 0.025});
}

bool CheckTargetIsFrozenWithinLegCycle()
{
    auto kernel = MakeKernel();
    go2_control::GaitKernelResult first{};
    go2_control::GaitKernelResult later{};
    if (!kernel.Compute(Request(0.0, 0.105), first) ||
        !kernel.Compute(Request(0.70, 0.05), later))
    {
        return false;
    }

    const std::size_t fr = static_cast<std::size_t>(go2::Leg::FR);
    return first.footstep_plan_valid &&
           Near(first.touchdown_target_x_m[fr], 0.0315) &&
           Near(later.touchdown_target_x_m[fr], 0.0315);
}

bool CheckCycleBoundaryIsContinuous()
{
    auto kernel = MakeKernel();
    go2_control::GaitKernelResult before{};
    go2_control::GaitKernelResult boundary{};
    if (!kernel.Compute(Request(0.0, 0.105), before) ||
        !kernel.Compute(Request(0.8, 0.05), boundary))
    {
        return false;
    }

    const std::size_t fr = static_cast<std::size_t>(go2::Leg::FR);
    const double expected_next_target =
        0.0315 + 0.20 * (0.05 - 0.105);
    return Near(boundary.touchdown_target_x_m[fr], expected_next_target) &&
           Near(boundary.feet[fr].x, 0.0315);
}

// [Fix 2026-08-13] gear shift 后 phase 连续 (旧实现 elapsed/period 在换挡时瞬移)
bool CheckGearShiftPhaseContinuity()
{
    auto kernel = MakeKernel();
    go2_control::GaitKernelResult prev{};
    if (!kernel.Compute(Request(0.0, 0.105), prev))
        return false;

    // 跑 8 帧 (period 0.8, 一个 cycle)
    double t = 0.1;
    for (int i = 0; i < 7; ++i)
    {
        go2_control::GaitKernelResult r{};
        if (!kernel.Compute(Request(t, 0.105), r))
            return false;
        prev = r;
        t += 0.1;
    }

    // 换挡: 目标 period 0.7 (kernel 每 cycle 渐变 0.02, 不会突变)
    kernel.SetGaitPeriod(0.7);

    // 继续跑 8 帧, 每帧 phase 增量应 ≈ dt/current_period (连续, 无瞬移)
    for (int i = 0; i < 8; ++i)
    {
        go2_control::GaitKernelResult r{};
        if (!kernel.Compute(Request(t, 0.105), r))
            return false;
        double d_phase = r.phase - prev.phase;
        // phase 跨 cycle 时回绕: |Δ|>0.5 说明跨了边界, ±1 调整
        if (d_phase < -0.5) d_phase += 1.0;
        if (d_phase > 0.5) d_phase -= 1.0;
        // 连续增量应为 dt/period (0.1/0.78 ~ 0.13) 附近; 旧实现会出现 >0.5 的瞬移
        if (std::abs(d_phase - 0.1 / 0.78) > 0.06)
        {
            std::cerr << "  gear shift phase jump: d_phase=" << d_phase
                      << " at t=" << t << "\n";
            return false;
        }
        prev = r;
        t += 0.1;
    }
    return true;
}

bool CheckResetAndInvalidInput()
{
    auto kernel = MakeKernel();
    go2_control::GaitKernelResult output{};
    if (!kernel.Compute(Request(0.0, 0.105), output))
        return false;

    auto invalid = Request(0.1, 0.105);
    invalid.body_velocity_x_mps =
        std::numeric_limits<double>::quiet_NaN();
    if (kernel.Compute(invalid, output))
        return false;

    kernel.Reset();
    return kernel.Compute(Request(0.0, 0.105), output) &&
           Near(output.touchdown_target_x_m[static_cast<std::size_t>(go2::Leg::FR)], 0.0315);
}

go2_control::RaibertTrotKernel MakePreviewKernel()
{
    go2_control::GaitKernelParams gait{};
    gait.period_s = 0.8;
    gait.duty_factor = 0.75;
    gait.step_length_m = 0.084;
    gait.direction_sign = 1.0;
    gait.foot_lift_m = 0.035;
    gait.blend_duration_s = 0.8;
    return go2_control::RaibertTrotKernel({gait, 0.20, 0.025, 4});
}

bool CheckPreviewHorizonClosesLoop()
{
    auto kernel = MakePreviewKernel();
    go2_control::GaitKernelResult nominal{};
    go2_control::GaitKernelResult slow{};
    if (!kernel.Compute(Request(0.0, 0.105), nominal))
        return false;
    const std::size_t fr = static_cast<std::size_t>(go2::Leg::FR);
    if (nominal.preview_n_steps != 4 ||
        !Near(nominal.touchdown_target_x_m[fr], 0.0315) ||
        !Near(nominal.preview_touchdown_x_m, 0.0315))
        return false;

    kernel.Reset();
    if (!kernel.Compute(Request(0.0, 0.05), slow))
        return false;
    return slow.preview_n_steps == 4 &&
           slow.touchdown_target_x_m[fr] < 0.0315 &&
           Near(slow.touchdown_target_x_m[fr], slow.preview_touchdown_x_m);
}

bool CheckPreviewPersistsWithinCycle()
{
    auto kernel = MakePreviewKernel();
    go2_control::GaitKernelResult first{};
    go2_control::GaitKernelResult later{};
    if (!kernel.Compute(Request(0.0, 0.05), first) ||
        !kernel.Compute(Request(0.10, 0.05), later))
        return false;
    return first.preview_n_steps == 4 &&
           later.preview_n_steps == 4 &&
           Near(later.preview_touchdown_x_m, first.preview_touchdown_x_m) &&
           Near(later.preview_terminal_velocity_x_mps,
                first.preview_terminal_velocity_x_mps);
}

bool CheckSpeedAdaptiveStanceUsesMeasuredTravel()
{
    go2_control::GaitKernelParams gait{};
    gait.period_s = 0.8;
    gait.duty_factor = 0.75;
    gait.step_length_m = 0.084;
    gait.direction_sign = 1.0;
    gait.foot_lift_m = 0.035;
    gait.blend_duration_s = 0.001;
    go2_control::RaibertTrotKernel kernel(
        {gait, 0.0, 0.0, 0, true});
    go2_control::GaitKernelResult mid{};
    if (!kernel.Compute(Request(0.0, 0.05), mid) ||
        !kernel.Compute(Request(0.30, 0.05), mid))
    {
        return false;
    }
    const std::size_t fr = static_cast<std::size_t>(go2::Leg::FR);
    const double commanded_travel = 0.084 * 0.75;
    const double v_nom = 0.084 / 0.8;
    const double v_stance = 0.85 * v_nom + 0.15 * 0.05;
    const double travel = std::clamp(
        v_stance * 0.8 * 0.75, 0.80 * commanded_travel, commanded_travel);
    const double expected = 0.5 * commanded_travel - travel * 0.5;
    return Near(mid.feet[fr].x, expected, 1e-6);
}

} // namespace

int main()
{
    if (!CheckTargetIsFrozenWithinLegCycle() ||
        !CheckCycleBoundaryIsContinuous() ||
        !CheckGearShiftPhaseContinuity() ||
        !CheckResetAndInvalidInput() ||
        !CheckPreviewHorizonClosesLoop() ||
        !CheckPreviewPersistsWithinCycle() ||
        !CheckSpeedAdaptiveStanceUsesMeasuredTravel())
    {
        std::cerr << "Raibert trot kernel checks failed\n";
        return 1;
    }

    std::cout << "Raibert trot kernel checks passed.\n";
    return 0;
}
