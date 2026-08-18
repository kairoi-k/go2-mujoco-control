#pragma once

#include <algorithm>
#include <cmath>

namespace go2_control
{

// Pure Raibert-style touchdown placement. This is a planning brick, not a
// complete MPC or whole-body controller.
struct RaibertFootstepPlannerParams
{
    double period_s = 0.8;
    double step_length_m = 0.084;
    double direction_sign = 1.0;
    double velocity_gain_s = 0.20;
    double max_adjustment_m = 0.025;
    double duty_factor = 1.0;
};

inline double RaibertStanceTravelM(const RaibertFootstepPlannerParams &params)
{
    double duty = params.duty_factor;
    if (!(duty > 0.0 && duty <= 1.0))
        duty = 1.0;
    return params.step_length_m * duty;
}

inline double RaibertNominalTouchdownX(const RaibertFootstepPlannerParams &params)
{
    return 0.5 * params.direction_sign * RaibertStanceTravelM(params);
}

struct RaibertFootstepPlannerInput
{
    double measured_velocity_x_mps = 0.0;
    bool measured_velocity_valid = false;
    double measured_velocity_y_mps = 0.0;
    bool measured_velocity_y_valid = false;
};

struct RaibertFootstepPlannerOutput
{
    double nominal_velocity_x_mps = 0.0;
    double velocity_error_x_mps = 0.0;
    double touchdown_x_m = 0.0;
    double adjustment_m = 0.0;
};

inline bool PlanRaibertTouchdown(
    const RaibertFootstepPlannerParams &params,
    const RaibertFootstepPlannerInput &input,
    RaibertFootstepPlannerOutput &output)
{
    if (!(params.period_s > 0.0) ||
        !(params.step_length_m >= 0.0) ||
        !(params.velocity_gain_s >= 0.0) ||
        !(params.max_adjustment_m >= 0.0) ||
        !(params.duty_factor > 0.0 && params.duty_factor <= 1.0) ||
        !(std::abs(std::abs(params.direction_sign) - 1.0) < 1e-9) ||
        !std::isfinite(params.period_s) ||
        !std::isfinite(params.step_length_m) ||
        !std::isfinite(params.direction_sign) ||
        !std::isfinite(params.velocity_gain_s) ||
        !std::isfinite(params.max_adjustment_m) ||
        !std::isfinite(params.duty_factor) ||
        (input.measured_velocity_valid &&
         !std::isfinite(input.measured_velocity_x_mps)))
    {
        return false;
    }

    const double nominal_velocity =
        params.direction_sign * params.step_length_m / params.period_s;
    const double measured_velocity = input.measured_velocity_valid
        ? input.measured_velocity_x_mps
        : nominal_velocity;
    // A slow body needs a rearward foot placement to accelerate.
    const double velocity_error = measured_velocity - nominal_velocity;
    const double adjustment = std::clamp(
        params.velocity_gain_s * velocity_error,
        -params.max_adjustment_m,
        params.max_adjustment_m);

    output.nominal_velocity_x_mps = nominal_velocity;
    output.velocity_error_x_mps = velocity_error;
    output.adjustment_m = adjustment;
    output.touchdown_x_m =
        RaibertNominalTouchdownX(params) + adjustment;
    return std::isfinite(output.touchdown_x_m);
}

} // namespace go2_control
