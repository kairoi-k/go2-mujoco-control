// Raibert diagonal trot gait kernel (swing/stance targets, sin^2 foot height).
#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include "locomotion_kernel.h"
#include "preview_footstep_horizon.h"
#include "raibert_footstep_planner.h"

namespace go2_control
{

struct RaibertTrotKernelParams
{
    GaitKernelParams gait{};
    double velocity_gain_s = 0.20;
    double max_adjustment_m = 0.025;
    int preview_horizon_steps = 0;
    bool speed_adaptive = false;
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
        stance_hold_ = false;
        stance_hold_blend_ = 0.0;
        have_last_gait_time_ = false;
        last_gait_time_s_ = 0.0;
        phase_acc_ = 0.0;  // [Fix 2026-08-13] 连续相位累积
        last_ramp_cycle_index_ = -1;
        last_preview_n_steps_ = 0;
        last_preview_touchdown_x_m_ = 0.0;
        last_preview_terminal_velocity_x_mps_ = 0.0;
        last_preview_planned_acc_x_mps2_ = 0.0;
        base_duty_factor_ = -1.0;
        base_foot_lift_m_ = -1.0;
    }

    // [Phase3] 在线步长/周期变更(cycle 边界调用,渐变趋近防冲击)
    void SetGaitStepLength(double step_m) override
    {
        if (step_m >= 0.0 && std::isfinite(step_m))
        {
            if (!have_last_gait_time_)
                params_.gait.step_length_m = step_m;
            else
                target_step_length_m_ = step_m;
        }
    }
    void SetGaitPeriod(double period_s) override
    {
        if (period_s > 0.0 && std::isfinite(period_s))
        {
            if (!have_last_gait_time_)
                params_.gait.period_s = period_s;
            else
                target_period_s_ = period_s;
        }
    }
    void SetGaitDuty(double duty) override
    {
        if (duty > 0.0 && duty < 1.0 && std::isfinite(duty))
        {
            if (!have_last_gait_time_)
                params_.gait.duty_factor = duty;
            else
                target_duty_factor_ = duty;
        }
    }
    void SetGaitFootLift(double lift_m) override
    {
        if (lift_m >= 0.0 && std::isfinite(lift_m))
        {
            if (!have_last_gait_time_)
                params_.gait.foot_lift_m = lift_m;
            else
                target_foot_lift_m_ = lift_m;
        }
    }
    void SetGaitSwingReachPhase(double phase) override
    {
        if (phase >= 0.5 && phase <= 1.0 && std::isfinite(phase))
            params_.gait.swing_reach_phase = phase;
    }
    void SetGaitEffectiveSpeedConvention(bool enabled) override
    {
        use_effective_speed_for_planner_ = enabled;
    }
    void SetStanceHold(bool hold, double gait_time_s) override
    {
        if (!hold && stance_hold_ && std::isfinite(gait_time_s))
        {
            phase_acc_ = 0.0;
            have_last_gait_time_ = false;
            leg_states_ = {};
            last_ramp_cycle_index_ = -1;
        }
        stance_hold_ = hold;
    }
    void SetGaitSlewLimits(double step, double period, double duty) override
    {
        if (step > 0.0 && std::isfinite(step))
            max_step_delta_ = step;
        if (period > 0.0 && std::isfinite(period))
            max_period_delta_ = period;
        if (duty > 0.0 && std::isfinite(duty))
            max_duty_delta_ = duty;
    }
    static constexpr double kMaxStepDeltaPerCycle = 0.010;
    static constexpr double kMaxPeriodDeltaPerCycle = 0.020;
    static constexpr double kMaxDutyDeltaPerCycle = 0.020;
    double max_step_delta_ = kMaxStepDeltaPerCycle;
    double max_period_delta_ = kMaxPeriodDeltaPerCycle;
    double max_duty_delta_ = kMaxDutyDeltaPerCycle;
    double target_duty_factor_ = -1.0;
    double target_foot_lift_m_ = -1.0;
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
            !std::isfinite(params_.max_adjustment_m) ||
            params_.preview_horizon_steps < 0 ||
            params_.preview_horizon_steps > kPreviewHorizonMaxSteps)
        {
            return false;
        }

        // [Fix 2026-08-13] 连续相位累积: phase += dt / current_period
        // 旧实现 elapsed/period 在换挡(period 渐变)时 cycle_position 瞬移;
        // 新实现 period 变化只影响未来增量, 无跳变.
        const double gait_dt = have_last_gait_time_
            ? std::max(0.0, request.gait_time_s - last_gait_time_s_)
            : 0.0;
        if (have_last_gait_time_ && !stance_hold_)
        {
            phase_acc_ += gait_dt / params_.gait.period_s;
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
        const double hold_step = std::clamp(gait_dt / 0.25, 0.0, 1.0);

        const double phase = cycle_position - std::floor(cycle_position);
        const int cycle_index = static_cast<int>(std::floor(cycle_position));
        // 渐变趋近目标步长/周期(周期边界限幅,防换档冲击)
        if (cycle_index != last_ramp_cycle_index_)
        {
            last_ramp_cycle_index_ = cycle_index;
            if (target_step_length_m_ >= 0.0)
            {
                const double delta = std::clamp(
                    target_step_length_m_ - params_.gait.step_length_m,
                    -max_step_delta_, max_step_delta_);
                params_.gait.step_length_m += delta;
                std::cout << "STEPK cycle=" << cycle_index
                          << " step=" << params_.gait.step_length_m << "\n";
            }
            if (target_period_s_ > 0.0)
            {
                const double delta = std::clamp(
                    target_period_s_ - params_.gait.period_s,
                    -max_period_delta_, max_period_delta_);
                params_.gait.period_s += delta;
            }
            if (target_duty_factor_ > 0.0)
            {
                const double delta = std::clamp(
                    target_duty_factor_ - params_.gait.duty_factor,
                    -max_duty_delta_, max_duty_delta_);
                params_.gait.duty_factor += delta;
            }
            if (target_foot_lift_m_ >= 0.0)
            {
                params_.gait.foot_lift_m += std::clamp(
                    target_foot_lift_m_ - params_.gait.foot_lift_m,
                    -0.006, 0.006);
            }
            if (params_.speed_adaptive)
            {
                if (base_duty_factor_ < 0.0)
                    base_duty_factor_ = params_.gait.duty_factor;
                if (base_foot_lift_m_ < 0.0)
                    base_foot_lift_m_ = params_.gait.foot_lift_m;
                const double v = std::abs(
                    params_.gait.direction_sign * params_.gait.step_length_m /
                    params_.gait.period_s);
                const double duty_des = std::clamp(
                    base_duty_factor_ - 0.06 * std::max(0.0, v - 0.60),
                    0.55, base_duty_factor_);
                const double duty_delta = std::clamp(
                    duty_des - params_.gait.duty_factor, -0.02, 0.02);
                if (std::abs(duty_delta) > 1.0e-6)
                {
                    params_.gait.duty_factor += duty_delta;
                    std::cout << "DUTY cycle=" << cycle_index
                              << " duty=" << params_.gait.duty_factor << "\n";
                }
                const double lift_des = std::clamp(
                    base_foot_lift_m_ +
                        0.014 * std::max(0.0, v - 0.20),
                    base_foot_lift_m_, 0.058);
                params_.gait.foot_lift_m += std::clamp(
                    lift_des - params_.gait.foot_lift_m, -0.004, 0.004);
            }
        }
        const double nominal_velocity = use_effective_speed_for_planner_
            ? params_.gait.direction_sign * params_.gait.step_length_m *
                  (2.0 * params_.gait.duty_factor) / params_.gait.period_s
            : params_.gait.direction_sign * params_.gait.step_length_m /
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
        result.preview_n_steps = last_preview_n_steps_;
        result.preview_touchdown_x_m = last_preview_touchdown_x_m_;
        result.preview_terminal_velocity_x_mps =
            last_preview_terminal_velocity_x_mps_;
        result.preview_planned_acc_x_mps2 =
            last_preview_planned_acc_x_mps2_;
        result.period_s = params_.gait.period_s;
        result.duty_factor = params_.gait.duty_factor;
        result.step_length_m = params_.gait.step_length_m;

        const RaibertFootstepPlannerParams planner_params{
            params_.gait.period_s,
            params_.gait.step_length_m,
            params_.gait.direction_sign,
            params_.velocity_gain_s,
            params_.max_adjustment_m,
            params_.gait.duty_factor,
            nominal_velocity};
        const double blend = Smoothstep(
            request.gait_time_s / params_.gait.blend_duration_s);
        const double hold_target = stance_hold_ ? 1.0 : 0.0;
        stance_hold_blend_ +=
            (hold_target - stance_hold_blend_) * hold_step;
        const double stance_duration = params_.gait.duty_factor;
        const double gait_blend = blend * (1.0 - stance_hold_blend_);
        const double swing_duration = 1.0 - stance_duration;
        const double commanded_travel_m =
            params_.gait.step_length_m * stance_duration;

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const double leg_cycle_position =
                cycle_position + GaitLegPhase(
                    leg, 0.0, params_.gait.pattern);
            const int leg_cycle_index =
                static_cast<int>(std::floor(leg_cycle_position));
            const double leg_phase =
                leg_cycle_position - std::floor(leg_cycle_position);
            LegState &state = leg_states_[leg];

            if (!state.initialized ||
                state.cycle_index != leg_cycle_index)
            {
                const RaibertFootstepPlannerInput planner_input{
                    measured_velocity, request.have_body_velocity,
                    request.body_velocity_y_mps, request.have_body_velocity};
                double next_touchdown_x_m = 0.0;
                double next_touchdown_y_m = 0.0;
                if (params_.preview_horizon_steps > 0)
                {
                    PreviewFootstepHorizonParams preview_params;
                    preview_params.raibert = planner_params;
                    preview_params.n_steps = params_.preview_horizon_steps;
                    PreviewFootstepHorizonOutput preview_output{};
                    if (!PlanPreviewFootstepHorizon(
                            preview_params, planner_input, preview_output))
                    {
                        return false;
                    }
                    next_touchdown_x_m = preview_output.touchdown_x_m[0];
                    next_touchdown_y_m = preview_output.touchdown_y_m[0];
                    last_preview_n_steps_ = preview_output.n_steps;
                    last_preview_touchdown_x_m_ = next_touchdown_x_m;
                    last_preview_terminal_velocity_x_mps_ =
                        preview_output.terminal_velocity_x_mps;
                    last_preview_planned_acc_x_mps2_ =
                        preview_output.planned_acc_x_mps2;
                    result.preview_n_steps = last_preview_n_steps_;
                    result.preview_touchdown_x_m = last_preview_touchdown_x_m_;
                    result.preview_terminal_velocity_x_mps =
                        last_preview_terminal_velocity_x_mps_;
                    result.preview_planned_acc_x_mps2 =
                        last_preview_planned_acc_x_mps2_;
                }
                else
                {
                    RaibertFootstepPlannerOutput planner_output{};
                    if (!PlanRaibertTouchdown(
                            planner_params, planner_input, planner_output))
                    {
                        return false;
                    }
                    next_touchdown_x_m = planner_output.touchdown_x_m;
                }

                if (!state.initialized)
                {
                    state.stance_start_x_m =
                        0.5 * params_.gait.direction_sign * commanded_travel_m;
                    state.stance_start_y_m = 0.0;
                }
                else
                {
                    // The previous swing already ended at this target. Keep
                    // it at stance entry, then plan the following touchdown.
                    state.stance_start_x_m =
                        state.next_touchdown_x_m;
                    state.stance_start_y_m = state.next_touchdown_y_m;
                }
                state.stance_travel_m = commanded_travel_m;
                if (params_.speed_adaptive && request.have_body_velocity)
                {
                    // Stay close to commanded travel so PD actually
                    // demands the gait speed. The speed governor keeps
                    // v_cmd near v_meas, so slip stays small.
                    const double v_stance =
                        0.85 * std::abs(nominal_velocity) +
                        0.15 * std::abs(measured_velocity);
                    state.stance_travel_m = std::clamp(
                        v_stance * params_.gait.period_s * stance_duration,
                        0.80 * commanded_travel_m,
                        commanded_travel_m);
                }
                state.next_touchdown_x_m = next_touchdown_x_m;
                state.next_touchdown_y_m = next_touchdown_y_m;
                state.cycle_index = leg_cycle_index;
                state.initialized = true;
            }

            double x_offset = 0.0;
            double y_offset = 0.0;
            double z_offset = 0.0;
            if (leg_phase < stance_duration)
            {
                const double stance_phase = leg_phase / stance_duration;
                x_offset =
                    state.stance_start_x_m -
                    params_.gait.direction_sign *
                        state.stance_travel_m *
                        Smoothstep(stance_phase);
                y_offset = state.stance_start_y_m;
            }
            else
            {
                const double swing_phase =
                    (leg_phase - stance_duration) / swing_duration;
                const double swing_start_x =
                    state.stance_start_x_m -
                    params_.gait.direction_sign *
                        state.stance_travel_m;
                const double swing_progress_end =
                    params_.gait.swing_reach_phase >= 0.5 ? 1.0 : 0.75;
                const double swing_x_phase = std::min(
                    1.0, swing_phase / swing_progress_end);
                x_offset =
                    swing_start_x +
                    (state.next_touchdown_x_m - swing_start_x) *
                        SwingFast(
                            swing_x_phase,
                            params_.gait.pattern,
                            params_.gait.swing_reach_phase);
                y_offset =
                    state.stance_start_y_m +
                    (state.next_touchdown_y_m - state.stance_start_y_m) *
                        SwingFast(
                            swing_x_phase,
                            params_.gait.pattern,
                            params_.gait.swing_reach_phase);
                const double swing_z_phase = std::min(
                    1.0, swing_phase / swing_progress_end);
                z_offset =
                    params_.gait.foot_lift_m *
                    std::sin(kPi * swing_z_phase) *
                    std::sin(kPi * swing_z_phase) *
                    (1.0 - 0.9 * std::max(0.0, swing_phase - 0.75) / 0.25);
            }

            result.touchdown_target_x_m[leg] =
                state.next_touchdown_x_m;
            result.feet[leg].x += gait_blend * x_offset;
            result.feet[leg].y += gait_blend * y_offset;
            result.feet[leg].z += gait_blend * z_offset;
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
        double stance_start_y_m = 0.0;
        double next_touchdown_y_m = 0.0;
        double stance_travel_m = 0.0;
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

    // [impulse] 摆动快速到位: quintic 缓动。低占空比跑态把目标
    // 略晚落到，避免在极短摆动窗内先猛冲到 touchdown 再长时间悬停；
    // 传统对角小跑保留原来的 0.60 相位。
    static double SwingFast(
        double x,
        GaitPattern pattern,
        double reach_phase_override)
    {
        const double reach_phase = reach_phase_override >= 0.5
            ? std::clamp(reach_phase_override, 0.5, 1.0)
            : (pattern == GaitPattern::kDiagonalTrot ? 0.60 : 0.72);
        const double t = std::min(1.0, x / reach_phase);
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
    bool stance_hold_ = false;
    double stance_hold_blend_ = 0.0;
    std::array<LegState, go2::kLegCount> leg_states_{};
    bool have_last_gait_time_ = false;
    double last_gait_time_s_ = 0.0;
    int last_preview_n_steps_ = 0;
    double last_preview_touchdown_x_m_ = 0.0;
    double last_preview_terminal_velocity_x_mps_ = 0.0;
    double last_preview_planned_acc_x_mps2_ = 0.0;
    double base_duty_factor_ = -1.0;
    double base_foot_lift_m_ = -1.0;
    bool use_effective_speed_for_planner_ = false;
};

} // namespace go2_control
