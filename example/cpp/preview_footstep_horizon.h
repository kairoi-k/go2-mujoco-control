#pragma once

// N-step receding-horizon foothold MPC. Jointly solves all preview
// adjustments; the gait kernel applies the first foothold.

#include <array>
#include <cmath>
#include <cstddef>

#include "raibert_footstep_planner.h"

namespace go2_control
{

constexpr int kPreviewHorizonMaxSteps = 8;
constexpr int kPreviewFirstStepSamples = 21;
constexpr double kPreviewFirstStepRegularization = 1.0e-3;

struct PreviewFootstepHorizonParams
{
    RaibertFootstepPlannerParams raibert{};
    int n_steps = 4;
};

struct PreviewFootstepHorizonOutput
{
    int n_steps = 0;
    double nominal_velocity_x_mps = 0.0;
    double terminal_velocity_x_mps = 0.0;
    double planned_acc_x_mps2 = 0.0;
    int qp_iterations = 0;
    std::array<double, kPreviewHorizonMaxSteps> touchdown_x_m{};
    std::array<double, kPreviewHorizonMaxSteps> predicted_velocity_x_mps{};
    std::array<double, kPreviewHorizonMaxSteps> touchdown_y_m{};
    std::array<double, kPreviewHorizonMaxSteps> predicted_velocity_y_mps{};
    double terminal_velocity_y_mps = 0.0;
    double planned_acc_y_mps2 = 0.0;
};

inline double PreviewNeutralTouchdownX(
    const RaibertFootstepPlannerParams &params)
{
    return 0.5 * params.direction_sign * params.step_length_m;
}

inline double PreviewAdjustmentFromTouchdown(
    const RaibertFootstepPlannerParams &params,
    double touchdown_x_m)
{
    return touchdown_x_m - PreviewNeutralTouchdownX(params);
}

// Rearward (negative) placement increases forward speed.
inline double PreviewVelocityAfterAdjustment(
    double velocity_x_mps,
    double adjustment_m,
    double period_s)
{
    if (!(period_s > 0.0))
        return velocity_x_mps;
    return velocity_x_mps - (2.0 / period_s) * adjustment_m;
}

inline bool SimulatePreviewHorizon(
    const PreviewFootstepHorizonParams &params,
    double first_touchdown_x_m,
    double velocity_x_mps,
    PreviewFootstepHorizonOutput &output)
{
    output = PreviewFootstepHorizonOutput{};
    if (params.n_steps <= 0 || params.n_steps > kPreviewHorizonMaxSteps)
        return false;
    if (!std::isfinite(first_touchdown_x_m) || !std::isfinite(velocity_x_mps))
        return false;

    const double nominal_velocity =
        params.raibert.direction_sign * params.raibert.step_length_m /
        params.raibert.period_s;
    double velocity = velocity_x_mps;
    for (int step = 0; step < params.n_steps; ++step)
    {
        double touchdown_x_m = first_touchdown_x_m;
        if (step > 0)
        {
            const RaibertFootstepPlannerInput input{velocity, true};
            RaibertFootstepPlannerOutput planned{};
            if (!PlanRaibertTouchdown(params.raibert, input, planned))
                return false;
            touchdown_x_m = planned.touchdown_x_m;
        }
        output.touchdown_x_m[static_cast<std::size_t>(step)] = touchdown_x_m;
        output.predicted_velocity_x_mps[static_cast<std::size_t>(step)] =
            velocity;
        const double adjustment = PreviewAdjustmentFromTouchdown(
            params.raibert, touchdown_x_m);
        velocity = PreviewVelocityAfterAdjustment(
            velocity, adjustment, params.raibert.period_s);
        if (!std::isfinite(velocity))
            return false;
    }
    output.n_steps = params.n_steps;
    output.nominal_velocity_x_mps = nominal_velocity;
    output.terminal_velocity_x_mps = velocity;
    return true;
}

inline double PreviewFirstStepCost(
    const PreviewFootstepHorizonOutput &horizon,
    double first_touchdown_x_m,
    const RaibertFootstepPlannerParams &params)
{
    const double velocity_error =
        horizon.terminal_velocity_x_mps - horizon.nominal_velocity_x_mps;
    const double adjustment =
        PreviewAdjustmentFromTouchdown(params, first_touchdown_x_m);
    return velocity_error * velocity_error +
           kPreviewFirstStepRegularization * adjustment * adjustment;
}

}  // namespace go2_control

#include "footstep_mpc.h"

namespace go2_control
{

inline bool PlanPreviewFootstepHorizon(
    const PreviewFootstepHorizonParams &params,
    const RaibertFootstepPlannerInput &input,
    PreviewFootstepHorizonOutput &output)
{
    output = PreviewFootstepHorizonOutput{};
    if (params.n_steps <= 0 || params.n_steps > kPreviewHorizonMaxSteps)
        return false;

    RaibertFootstepPlannerOutput greedy{};
    if (!PlanRaibertTouchdown(params.raibert, input, greedy))
        return false;

    const double measured_velocity = input.measured_velocity_valid
        ? input.measured_velocity_x_mps
        : greedy.nominal_velocity_x_mps;
    const double measured_velocity_y = input.measured_velocity_y_valid
        ? input.measured_velocity_y_mps
        : 0.0;

    if (SolveFootstepMpc(
            params, measured_velocity, output, measured_velocity_y))
        return true;

    if (params.n_steps == 1)
        return SimulatePreviewHorizon(
            params, greedy.touchdown_x_m, measured_velocity, output);

    const double max_adjustment = params.raibert.max_adjustment_m;
    const double neutral = PreviewNeutralTouchdownX(params.raibert);
    bool have_best = false;
    double best_cost = 0.0;
    PreviewFootstepHorizonOutput best{};

    auto consider = [&](double touchdown_x_m) {
        PreviewFootstepHorizonOutput candidate{};
        if (!SimulatePreviewHorizon(
                params, touchdown_x_m, measured_velocity, candidate))
            return;
        const double cost = PreviewFirstStepCost(
            candidate, touchdown_x_m, params.raibert);
        if (!have_best || cost < best_cost)
        {
            have_best = true;
            best_cost = cost;
            best = candidate;
        }
    };

    consider(greedy.touchdown_x_m);
    if (max_adjustment > 0.0)
    {
        for (int sample = 0; sample < kPreviewFirstStepSamples; ++sample)
        {
            const double fraction =
                static_cast<double>(sample) /
                static_cast<double>(kPreviewFirstStepSamples - 1);
            const double adjustment =
                -max_adjustment + 2.0 * max_adjustment * fraction;
            consider(neutral + adjustment);
        }
    }
    if (!have_best)
        return false;
    output = best;
    return true;
}

inline bool PreviewTerminalAcceleration(
    double nominal_velocity_x_mps,
    double terminal_velocity_x_mps,
    int n_steps,
    double period_s,
    double &acc_x_mps2)
{
    acc_x_mps2 = 0.0;
    if (n_steps <= 0 ||
        !(period_s > 0.0) ||
        !std::isfinite(nominal_velocity_x_mps) ||
        !std::isfinite(terminal_velocity_x_mps))
        return false;
    const double horizon_s =
        0.5 * period_s * static_cast<double>(n_steps);
    acc_x_mps2 =
        (nominal_velocity_x_mps - terminal_velocity_x_mps) / horizon_s;
    return std::isfinite(acc_x_mps2);
}

}  // namespace go2_control
