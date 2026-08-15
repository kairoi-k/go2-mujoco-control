#pragma once

// Receding-horizon foothold MPC. Jointly optimizes all N adjustments:
//   v_{k+1} = v_k - (2/T) adj_k
//   min sum_k (v_k - v*)^2 + w ||adj||^2
//   s.t. |adj_k| <= adj_max
// First foothold is applied; the rest is the preview.

#include <cmath>

#include <Eigen/Dense>

#include "dense_qp.h"
#include "preview_footstep_horizon.h"

namespace go2_control
{

inline bool SolveFootstepMpc(
    const PreviewFootstepHorizonParams &params,
    double measured_velocity_x_mps,
    PreviewFootstepHorizonOutput &output)
{
    output = PreviewFootstepHorizonOutput{};
    if (params.n_steps <= 0 || params.n_steps > kPreviewHorizonMaxSteps)
        return false;
    if (!(params.raibert.period_s > 0.0) ||
        !std::isfinite(measured_velocity_x_mps) ||
        !std::isfinite(params.raibert.max_adjustment_m) ||
        params.raibert.max_adjustment_m < 0.0)
    {
        return false;
    }

    const int n = params.n_steps;
    const double period = params.raibert.period_s;
    const double alpha = 2.0 / period;
    const double nominal =
        params.raibert.direction_sign * params.raibert.step_length_m / period;
    const double velocity_weight = 1.0;
    const double adjustment_weight = kPreviewFirstStepRegularization;

    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);
    for (int row = 0; row < n; ++row)
    {
        for (int col = 0; col <= row; ++col)
            S(row, col) = 1.0;
    }
    const Eigen::VectorXd offset =
        Eigen::VectorXd::Constant(n, measured_velocity_x_mps - nominal);
    Eigen::MatrixXd H =
        2.0 * velocity_weight * alpha * alpha * (S.transpose() * S);
    H.diagonal().array() += 2.0 * adjustment_weight;
    const Eigen::VectorXd gvec =
        -2.0 * velocity_weight * alpha * (S.transpose() * offset);

    Eigen::MatrixXd A(2 * n, n);
    A.setZero();
    Eigen::VectorXd b(2 * n);
    const double max_adj = params.raibert.max_adjustment_m;
    A.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    A.block(n, 0, n, n) = -Eigen::MatrixXd::Identity(n, n);
    b.head(n).setConstant(max_adj);
    b.tail(n).setConstant(max_adj);

    Eigen::VectorXd adj;
    int iterations = 0;
    DenseQpSettings settings;
    settings.max_iterations = 80;
    if (!SolveDenseQp(H, gvec, A, b, adj, iterations, settings) ||
        adj.size() != n)
    {
        return false;
    }

    const double neutral = PreviewNeutralTouchdownX(params.raibert);
    double velocity = measured_velocity_x_mps;
    for (int step = 0; step < n; ++step)
    {
        const double adjustment = adj[step];
        if (!std::isfinite(adjustment))
            return false;
        output.touchdown_x_m[static_cast<std::size_t>(step)] =
            neutral + adjustment;
        output.predicted_velocity_x_mps[static_cast<std::size_t>(step)] =
            velocity;
        if (step == 0)
        {
            output.planned_acc_x_mps2 =
                -alpha * adjustment / period;
        }
        velocity = PreviewVelocityAfterAdjustment(velocity, adjustment, period);
    }
    output.n_steps = n;
    output.nominal_velocity_x_mps = nominal;
    output.terminal_velocity_x_mps = velocity;
    output.qp_iterations = iterations;
    return std::isfinite(output.terminal_velocity_x_mps) &&
           std::isfinite(output.planned_acc_x_mps2);
}

}  // namespace go2_control
