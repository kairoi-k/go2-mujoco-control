#pragma once

// Receding-horizon foothold MPC. Jointly optimizes all N adjustments
// on x and y:
//   v_{k+1} = v_k - (2/T) adj_k
//   min sum_k (v_k - v*)^2 + w ||adj||^2
//   s.t. |adj_k| <= adj_max
// First foothold is applied; the rest is the preview.

#include <array>
#include <cmath>

#include <Eigen/Dense>

#include "dense_qp.h"
#include "preview_footstep_horizon.h"

namespace go2_control
{

inline bool SolveFootstepMpcAxis(
    int n_steps,
    double period_s,
    double measured_velocity_mps,
    double nominal_velocity_mps,
    double max_adjustment_m,
    std::array<double, kPreviewHorizonMaxSteps> &touchdown_m,
    std::array<double, kPreviewHorizonMaxSteps> &predicted_velocity_mps,
    double neutral_m,
    double &terminal_velocity_mps,
    double &planned_acc_mps2,
    int &qp_iterations)
{
    if (n_steps <= 0 || n_steps > kPreviewHorizonMaxSteps)
        return false;
    if (!(period_s > 0.0) ||
        !std::isfinite(measured_velocity_mps) ||
        !std::isfinite(nominal_velocity_mps) ||
        !std::isfinite(max_adjustment_m) ||
        max_adjustment_m < 0.0)
    {
        return false;
    }

    const int n = n_steps;
    const double alpha = 2.0 / period_s;
    const double velocity_weight = 1.0;
    const double adjustment_weight = kPreviewFirstStepRegularization;

    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);
    for (int row = 0; row < n; ++row)
    {
        for (int col = 0; col <= row; ++col)
            S(row, col) = 1.0;
    }
    const Eigen::VectorXd offset = Eigen::VectorXd::Constant(
        n, measured_velocity_mps - nominal_velocity_mps);
    Eigen::MatrixXd H =
        2.0 * velocity_weight * alpha * alpha * (S.transpose() * S);
    H.diagonal().array() += 2.0 * adjustment_weight;
    const Eigen::VectorXd gvec =
        -2.0 * velocity_weight * alpha * (S.transpose() * offset);

    Eigen::MatrixXd A(2 * n, n);
    A.setZero();
    Eigen::VectorXd b(2 * n);
    A.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    A.block(n, 0, n, n) = -Eigen::MatrixXd::Identity(n, n);
    b.head(n).setConstant(max_adjustment_m);
    b.tail(n).setConstant(max_adjustment_m);

    Eigen::VectorXd adj;
    int iterations = 0;
    DenseQpSettings settings;
    settings.max_iterations = 80;
    if (!SolveDenseQp(H, gvec, A, b, adj, iterations, settings) ||
        adj.size() != n)
    {
        return false;
    }

    double velocity = measured_velocity_mps;
    planned_acc_mps2 = 0.0;
    for (int step = 0; step < n; ++step)
    {
        const double adjustment = adj[step];
        if (!std::isfinite(adjustment))
            return false;
        touchdown_m[static_cast<std::size_t>(step)] = neutral_m + adjustment;
        predicted_velocity_mps[static_cast<std::size_t>(step)] = velocity;
        if (step == 0)
            planned_acc_mps2 = -alpha * adjustment / period_s;
        velocity = PreviewVelocityAfterAdjustment(
            velocity, adjustment, period_s);
    }
    terminal_velocity_mps = velocity;
    qp_iterations += iterations;
    return std::isfinite(terminal_velocity_mps) &&
           std::isfinite(planned_acc_mps2);
}

inline bool SolveFootstepMpc(
    const PreviewFootstepHorizonParams &params,
    double measured_velocity_x_mps,
    PreviewFootstepHorizonOutput &output,
    double measured_velocity_y_mps = 0.0)
{
    output = PreviewFootstepHorizonOutput{};
    const double nominal_x =
        params.raibert.direction_sign * params.raibert.step_length_m /
        params.raibert.period_s;
    int iterations = 0;
    if (!SolveFootstepMpcAxis(
            params.n_steps,
            params.raibert.period_s,
            measured_velocity_x_mps,
            nominal_x,
            params.raibert.max_adjustment_m,
            output.touchdown_x_m,
            output.predicted_velocity_x_mps,
            PreviewNeutralTouchdownX(params.raibert),
            output.terminal_velocity_x_mps,
            output.planned_acc_x_mps2,
            iterations))
    {
        return false;
    }
    if (!SolveFootstepMpcAxis(
            params.n_steps,
            params.raibert.period_s,
            measured_velocity_y_mps,
            0.0,
            params.raibert.max_adjustment_m,
            output.touchdown_y_m,
            output.predicted_velocity_y_mps,
            0.0,
            output.terminal_velocity_y_mps,
            output.planned_acc_y_mps2,
            iterations))
    {
        return false;
    }
    output.n_steps = params.n_steps;
    output.nominal_velocity_x_mps = nominal_x;
    output.qp_iterations = iterations;
    return true;
}

}  // namespace go2_control
