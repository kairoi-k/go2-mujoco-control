// Abstract locomotion gait kernel interface used by real_trot_go2.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "go2_forward_kinematics.h"

namespace go2_control
{

struct GaitKernelParams
{
    double period_s = 0.8;
    double duty_factor = 0.58;
    double step_length_m = 0.12;
    double direction_sign = 1.0;
    double foot_lift_m = 0.05;
    double blend_duration_s = 0.8;
};

struct GaitKernelRequest
{
    double gait_time_s = 0.0;
    std::array<go2::Vec3, go2::kLegCount> neutral_feet{};
    // State inputs reserved for feedback-driven classical kernels.
    // Body-frame velocity: x forward, y left, z up.
    // The caller must convert sensor/world velocity before filling it.
    double body_velocity_x_mps = 0.0;
    double body_velocity_y_mps = 0.0;
    double body_velocity_z_mps = 0.0;
    bool have_body_velocity = false;
};

struct GaitKernelResult
{
    double phase = 0.0;
    int cycle_index = 0;
    std::array<go2::Vec3, go2::kLegCount> feet{};
    std::array<double, go2::kLegCount> touchdown_target_x_m{};
    double velocity_error_x_mps = 0.0;
    double nominal_velocity_x_mps = 0.0;  // [Fix 2026-08-13] kernel 当前生效目标速度 (换挡后 world 同步用)
    bool footstep_plan_valid = false;
    int preview_n_steps = 0;
    double preview_touchdown_x_m = 0.0;
    double preview_terminal_velocity_x_mps = 0.0;
};

class LocomotionKernel
{
public:
    virtual ~LocomotionKernel() = default;
    virtual const char *Name() const noexcept = 0;
    virtual bool Compute(
        const GaitKernelRequest &request,
        GaitKernelResult &result) = 0;
    virtual void SetGaitStepLength(double) {}
    virtual void SetGaitPeriod(double) {}
};

class HandCodedTrotKernel final : public LocomotionKernel
{
public:
    explicit HandCodedTrotKernel(GaitKernelParams params)
        : params_(params)
    {
    }

    const char *Name() const noexcept override
    {
        return "hand-coded-trot";
    }

    bool Compute(
        const GaitKernelRequest &request,
        GaitKernelResult &result) override
    {
        if (!std::isfinite(request.gait_time_s) ||
            request.gait_time_s < 0.0 ||
            !(params_.period_s > 0.0) ||
            !(params_.duty_factor > 0.0 &&
              params_.duty_factor < 1.0) ||
            !(params_.step_length_m >= 0.0) ||
            !(params_.foot_lift_m >= 0.0) ||
            !(params_.blend_duration_s > 0.0) ||
            !(std::abs(std::abs(params_.direction_sign) - 1.0) < 1e-9))
        {
            return false;
        }

        const double cycle_position =
            request.gait_time_s / params_.period_s;
        if (!std::isfinite(cycle_position) ||
            cycle_position >
                static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }

        result.phase = cycle_position - std::floor(cycle_position);
        result.cycle_index = static_cast<int>(std::floor(cycle_position));
        result.feet = request.neutral_feet;
        result.touchdown_target_x_m.fill(0.0);
        result.velocity_error_x_mps = 0.0;
        result.footstep_plan_valid = false;

        const double blend =
            Smoothstep(request.gait_time_s / params_.blend_duration_s);
        const double half_step = 0.5 * params_.step_length_m;
        const double stance_duration = params_.duty_factor;
        const double swing_duration = 1.0 - params_.duty_factor;

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const bool diagonal_pair_b =
                leg == static_cast<std::size_t>(go2::Leg::FL) ||
                leg == static_cast<std::size_t>(go2::Leg::RR);
            const double leg_phase = std::fmod(
                result.phase + (diagonal_pair_b ? 0.5 : 0.0), 1.0);
            double x_offset = 0.0;
            double z_offset = 0.0;
            if (leg_phase < stance_duration)
            {
                const double stance_phase = leg_phase / stance_duration;
                x_offset =
                    half_step -
                    params_.step_length_m * Smoothstep(stance_phase);
            }
            else
            {
                const double swing_phase =
                    (leg_phase - stance_duration) / swing_duration;
                x_offset =
                    -half_step +
                    params_.step_length_m * Smoothstep(swing_phase);
                z_offset =
                    params_.foot_lift_m * std::sin(kPi * swing_phase);
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
};

} // namespace go2_control
