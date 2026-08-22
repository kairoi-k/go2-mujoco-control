// Quasi-static crawl gait kernel: one leg swings at a time, three feet
// always planted.  Swing sequence FL -> RR -> FR -> RL (classic lateral
// sequence crawl) maximises the support polygon during each transfer.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "locomotion_kernel.h"

namespace go2_control
{

class CrawlKernel final : public LocomotionKernel
{
public:
    explicit CrawlKernel(GaitKernelParams params)
        : params_(params)
    {
    }

    const char *Name() const noexcept override
    {
        return "crawl";
    }

    void SetGaitStepLength(double step_m) override
    {
        if (step_m >= 0.0 && std::isfinite(step_m))
            params_.step_length_m = step_m;
    }
    void SetGaitPeriod(double period_s) override
    {
        if (period_s > 0.0 && std::isfinite(period_s))
            params_.period_s = period_s;
    }
    void SetGaitFootLift(double lift_m) override
    {
        if (lift_m >= 0.0 && std::isfinite(lift_m))
            params_.foot_lift_m = lift_m;
    }
    void SetStanceHold(bool hold, double gait_time_s) override
    {
        if (!hold && stance_hold_ && std::isfinite(gait_time_s))
            phase_origin_gait_time_s_ = gait_time_s;
        stance_hold_ = hold;
    }

    bool Compute(
        const GaitKernelRequest &request,
        GaitKernelResult &result) override
    {
        if (!std::isfinite(request.gait_time_s) ||
            request.gait_time_s < 0.0 ||
            !(params_.period_s > 0.0) ||
            !(params_.step_length_m >= 0.0) ||
            !(params_.foot_lift_m >= 0.0) ||
            !(params_.blend_duration_s > 0.0) ||
            !(std::abs(std::abs(params_.direction_sign) - 1.0) < 1e-9))
        {
            return false;
        }

        // One full cycle swings all four legs once; each swing owns 1/4 of
        // the cycle and every leg is in stance the other 3/4.
        constexpr int kSwingOrder[go2::kLegCount] = {
            static_cast<int>(go2::Leg::FL),
            static_cast<int>(go2::Leg::RR),
            static_cast<int>(go2::Leg::FR),
            static_cast<int>(go2::Leg::RL)};
        // Swing slots start every 1/4 cycle, but each swing only lasts 70%
        // of its slot.  The remaining 30% keeps all four feet planted as an
        // explicit weight-transfer settle before the next lift-off.
        constexpr double kSlotFraction = 0.25;
        constexpr double kSwingFraction = kSlotFraction * 0.70;

        const double effective_gait_time = std::max(
            0.0, request.gait_time_s - phase_origin_gait_time_s_);
        const double cycle_position = effective_gait_time / params_.period_s;
        if (!std::isfinite(cycle_position))
            return false;

        result.phase = cycle_position - std::floor(cycle_position);
        result.cycle_index = static_cast<int>(std::floor(cycle_position));
        result.feet = request.neutral_feet;
        result.touchdown_target_x_m.fill(0.0);
        result.velocity_error_x_mps = 0.0;
        result.nominal_velocity_x_mps =
            params_.direction_sign * params_.step_length_m /
            params_.period_s;
        result.footstep_plan_valid = false;
        result.period_s = params_.period_s;
        result.duty_factor = 1.0 - kSwingFraction;
        result.step_length_m = params_.step_length_m;
        result.has_swing_schedule = true;

        const double blend =
            Smoothstep(effective_gait_time / params_.blend_duration_s);
        const double half_step = 0.5 * params_.step_length_m;

        for (std::size_t slot = 0; slot < go2::kLegCount; ++slot)
        {
            const std::size_t leg =
                static_cast<std::size_t>(kSwingOrder[slot]);
            const double swing_start =
                static_cast<double>(slot) * kSlotFraction;
            double x_offset = 0.0;
            double z_offset = 0.0;
            const double leg_phase =
                (result.phase - swing_start + 1.0) -
                std::floor(result.phase - swing_start + 1.0);
            result.scheduled_swing[leg] = leg_phase < kSwingFraction;
            if (leg_phase < kSwingFraction)
            {
                // Swing: foot travels -half -> +half step while lifting on
                // a smoothstep-shaped bump.
                const double s = leg_phase / kSwingFraction;
                x_offset = -half_step +
                           params_.step_length_m * Smoothstep(s);
                z_offset = params_.foot_lift_m *
                           std::sin(kPi * Smoothstep(s));
                result.touchdown_target_x_m[leg] =
                    params_.direction_sign * half_step;
            }
            else
            {
                // Stance: foot sweeps +half -> -half relative to the body.
                const double s = (leg_phase - kSwingFraction) /
                                 (1.0 - kSwingFraction);
                x_offset = half_step -
                           params_.step_length_m * Smoothstep(s);
            }
            x_offset *= params_.direction_sign;
            result.feet[leg].x += blend * x_offset;
            result.feet[leg].z += blend * z_offset;
        }
        return true;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    static double Smoothstep(double x)
    {
        if (x <= 0.0)
            return 0.0;
        if (x >= 1.0)
            return 1.0;
        return x * x * (3.0 - 2.0 * x);
    }

    GaitKernelParams params_;
    bool stance_hold_ = false;
    double phase_origin_gait_time_s_ = 0.0;
};

}  // namespace go2_control
