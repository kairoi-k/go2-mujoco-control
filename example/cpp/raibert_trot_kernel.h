// Raibert diagonal trot gait kernel (swing/stance targets, sin^2 foot height).
#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "locomotion_kernel.h"
#include "raibert_footstep_planner.h"

namespace go2_control
{

struct RaibertTrotKernelParams
{
    GaitKernelParams gait{};
    double velocity_gain_s = 0.20;
    double max_adjustment_m = 0.025;
};

class RaibertTrotKernel final : public LocomotionKernel
{
public:
    explicit RaibertTrotKernel(RaibertTrotKernelParams params)
        : params_(params)
    {
    }

    const char *Name() const noexcept override
    {
        return "raibert-trot";
    }

    void Reset() noexcept
    {
        leg_states_ = {};
        have_last_gait_time_ = false;
        last_gait_time_s_ = 0.0;
        phase_acc_ = 0.0;  // [Fix 2026-08-13] 连续相位累积
    }

    // [Phase3] 在线步长/周期变更(cycle 边界调用,渐变趋近防冲击)
    void SetGaitStepLength(double step_m)
    {
        if (step_m > 0.0 && std::isfinite(step_m))
            target_step_length_m_ = step_m;
    }
    void SetGaitPeriod(double period_s)
    {
        if (period_s > 0.0 && std::isfinite(period_s))
            target_period_s_ = period_s;
    }
    static constexpr double kMaxStepDeltaPerCycle = 0.002;
    static constexpr double kMaxPeriodDeltaPerCycle = 0.02;
    double target_step_length_m_ = -1.0;
    int last_ramp_cycle_index_ = -1;
    double target_period_s_ = -1.0;
    double phase_acc_ = 0.0;  // [Fix 2026-08-13] 连续相位累积器

    bool Compute(
        const GaitKernelRequest &request,
        GaitKernelResult &result) override
    {
        if (!ValidateRequest(request) ||
            !ValidateGaitParams(params_.gait) ||
            !(params_.velocity_gain_s >= 0.0) ||
            !(params_.max_adjustment_m >= 0.0) ||
            !std::isfinite(params_.velocity_gain_s) ||
            !std::isfinite(params_.max_adjustment_m))
        {
            return false;
        }

        // [Fix 2026-08-13] 连续相位累积: phase += dt / current_period
        // 旧实现 elapsed/period 在换挡(period 渐变)时 cycle_position 瞬移;
        // 新实现 period 变化只影响未来增量, 无跳变.
        if (have_last_gait_time_)
        {
            const double dt = request.gait_time_s - last_gait_time_s_;
            phase_acc_ += dt / params_.gait.period_s;
        }
        const double cycle_position = phase_acc_;
        if (!std::isfinite(cycle_position) ||
            cycle_position + 0.5 >
                static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }
        if (have_last_gait_time_ &&
            request.gait_time_s + 1e-9 < last_gait_time_s_)
        {
            return false;
        }
        last_gait_time_s_ = request.gait_time_s;

        const double phase = cycle_position - std::floor(cycle_position);
        const int cycle_index = static_cast<int>(std::floor(cycle_position));
        // 渐变趋近目标步长/周期(周期边界限幅,防换档冲击)
        if ((target_step_length_m_ > 0.0 ||
             target_period_s_ > 0.0) &&
            cycle_index != last_ramp_cycle_index_)
        {
            last_ramp_cycle_index_ = cycle_index;
            if (target_step_length_m_ > 0.0)
            {
                const double delta = std::clamp(
                    target_step_length_m_ - params_.gait.step_length_m,
                    -kMaxStepDeltaPerCycle, kMaxStepDeltaPerCycle);
                params_.gait.step_length_m += delta;
                std::cout << "STEPK cycle=" << cycle_index
                          << " step=" << params_.gait.step_length_m << "\n";
            }
            if (target_period_s_ > 0.0)
            {
                const double delta = std::clamp(
                    target_period_s_ - params_.gait.period_s,
                    -kMaxPeriodDeltaPerCycle, kMaxPeriodDeltaPerCycle);
                params_.gait.period_s += delta;
            }
        }
        const double nominal_velocity =
            params_.gait.direction_sign * params_.gait.step_length_m /
            params_.gait.period_s;
        const double measured_velocity = request.have_body_velocity
            ? request.body_velocity_x_mps
            : nominal_velocity;
        const double velocity_error = measured_velocity - nominal_velocity;

        result = GaitKernelResult{};
        result.phase = phase;
        result.cycle_index = cycle_index;
        result.feet = request.neutral_feet;
        result.velocity_error_x_mps = velocity_error;
        result.nominal_velocity_x_mps = nominal_velocity;  // [Fix 2026-08-13]
        result.footstep_plan_valid = true;

        const RaibertFootstepPlannerParams planner_params{
            params_.gait.period_s,
            params_.gait.step_length_m,
            params_.gait.direction_sign,
            params_.velocity_gain_s,
            params_.max_adjustment_m};
        const double blend = Smoothstep(
            request.gait_time_s / params_.gait.blend_duration_s);
        const double stance_duration = params_.gait.duty_factor;
        const double swing_duration = 1.0 - stance_duration;

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const bool diagonal_pair_b =
                leg == static_cast<std::size_t>(go2::Leg::FL) ||
                leg == static_cast<std::size_t>(go2::Leg::RR);
            const double leg_cycle_position =
                cycle_position + (diagonal_pair_b ? 0.5 : 0.0);
            const int leg_cycle_index =
                static_cast<int>(std::floor(leg_cycle_position));
            const double leg_phase =
                leg_cycle_position - std::floor(leg_cycle_position);
            LegState &state = leg_states_[leg];

            if (!state.initialized ||
                state.cycle_index != leg_cycle_index)
            {
                const RaibertFootstepPlannerInput planner_input{
                    measured_velocity, request.have_body_velocity};
                RaibertFootstepPlannerOutput planner_output{};
                if (!PlanRaibertTouchdown(
                        planner_params, planner_input, planner_output))
                {
                    return false;
                }

                if (!state.initialized)
                {
                    state.stance_start_x_m =
                        0.5 * params_.gait.direction_sign *
                        params_.gait.step_length_m;
                }
                else
                {
                    // The previous swing already ended at this target. Keep
                    // it at stance entry, then plan the following touchdown.
                    state.stance_start_x_m =
                        state.next_touchdown_x_m;
                }
                state.next_touchdown_x_m = planner_output.touchdown_x_m;
                state.cycle_index = leg_cycle_index;
                state.initialized = true;
            }

            double x_offset = 0.0;
            double z_offset = 0.0;
            if (leg_phase < stance_duration)
            {
                const double stance_phase = leg_phase / stance_duration;
                x_offset =
                    state.stance_start_x_m -
                    params_.gait.direction_sign *
                        params_.gait.step_length_m *
                        Smoothstep(stance_phase);
            }
            else
            {
                const double swing_phase =
                    (leg_phase - stance_duration) / swing_duration;
                const double swing_start_x =
                    state.stance_start_x_m -
                    params_.gait.direction_sign *
                        params_.gait.step_length_m;
                // [impulse] 摆动 x 提前到位: 相位 0.75 到达目标, 剩余
                // 悬停等触地。降低摆动末端加速度峰值, 让大步长摆动
                // 追得上目标(q_error 不再爆 0.3-0.5)。
                const double swing_x_phase =
                    std::min(1.0, swing_phase / 0.75);
                x_offset =
                    swing_start_x +
                    (state.next_touchdown_x_m - swing_start_x) *
                        SwingFast(swing_x_phase);
                // z 提前回落(0.75 相位降到 ~0.1 抬升高, 不拖地),
                // 相位 1.0 完全落地。与 x 到位匹配。
                const double swing_z_phase =
                    std::min(1.0, swing_phase / 0.75);
                z_offset =
                    params_.gait.foot_lift_m *
                    std::sin(kPi * swing_z_phase) *
                    std::sin(kPi * swing_z_phase) *
                    (1.0 - 0.9 * std::max(0.0, swing_phase - 0.75) / 0.25);
            }

            result.touchdown_target_x_m[leg] =
                state.next_touchdown_x_m;
            result.feet[leg].x += blend * x_offset;
            result.feet[leg].z += blend * z_offset;
        }

        have_last_gait_time_ = true;
        last_gait_time_s_ = request.gait_time_s;
        return true;
    }

private:
    struct LegState
    {
        int cycle_index = std::numeric_limits<int>::min();
        double stance_start_x_m = 0.0;
        double next_touchdown_x_m = 0.0;
        bool initialized = false;
    };

    static constexpr double kPi = 3.14159265358979323846;

    static double Smoothstep(double x)
    {
        if (x <= 0.0)
            return 0.0;
        if (x >= 1.0)
            return 1.0;
        return x * x * (3.0 - 2.0 * x);
    }

    // [impulse] 摆动快速到位: quintic 缓动, 相位 0.6 到达目标
    // (比 Smoothstep 更早到位), 给大步长摆动更多悬停余量。
    static double SwingFast(double x)
    {
        const double t = std::min(1.0, x / 0.60);
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    static bool ValidateRequest(const GaitKernelRequest &request)
    {
        if (!std::isfinite(request.gait_time_s) ||
            request.gait_time_s < 0.0)
        {
            return false;
        }
        if (request.have_body_velocity &&
            (!std::isfinite(request.body_velocity_x_mps) ||
             !std::isfinite(request.body_velocity_y_mps) ||
             !std::isfinite(request.body_velocity_z_mps)))
        {
            return false;
        }
        return true;
    }

    static bool ValidateGaitParams(const GaitKernelParams &params)
    {
        return std::isfinite(params.period_s) &&
               std::isfinite(params.duty_factor) &&
               std::isfinite(params.step_length_m) &&
               std::isfinite(params.direction_sign) &&
               std::isfinite(params.foot_lift_m) &&
               std::isfinite(params.blend_duration_s) &&
               params.period_s > 0.0 &&
               params.duty_factor > 0.0 &&
               params.duty_factor < 1.0 &&
               params.step_length_m >= 0.0 &&
               params.foot_lift_m >= 0.0 &&
               params.blend_duration_s > 0.0 &&
               std::abs(std::abs(params.direction_sign) - 1.0) < 1e-9;
    }

    RaibertTrotKernelParams params_;
    std::array<LegState, go2::kLegCount> leg_states_{};
    bool have_last_gait_time_ = false;
    double last_gait_time_s_ = 0.0;
};

} // namespace go2_control
