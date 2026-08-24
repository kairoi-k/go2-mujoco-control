#pragma once

// Hierarchical inverse-dynamics WBC:
//   M qdd + h = S^T tau + J^T f
// Floating-base rows are equalities. Friction / swing-zero / torque limits
// are inequalities. Motion tasks are weighted: CoM/orientation from MPC,
// then swing, then posture. Output is joint tau* and qdd*.

#include <array>
#include <cmath>

#include <Eigen/Dense>

#include "dense_qp.h"
#include "go2_forward_kinematics.h"
#include "go2_rigid_body.h"

namespace go2_control
{

struct IdWbcParams
{
    double friction_mu = 0.8;
    double min_normal_n = 1.0;
    double max_normal_n = 180.0;
    double tau_limit_nm = 35.0;
    double w_base_lin = 80.0;
    // Optional anisotropic sprint weights.  A negative value keeps the
    // isotropic w_base_lin/w_swing setting used by the validated trot.
    double w_base_lin_x = -1.0;
    double w_base_ang = 40.0;
    double w_swing = 80.0;
    double w_swing_x = -1.0;
    double w_stance_no_slip = 8.0;
    // <0 means use w_stance_no_slip on that axis (isotropic).
    double w_stance_no_slip_x = -1.0;
    double w_stance_no_slip_y = -1.0;
    double w_stance_no_slip_z = -1.0;
    double w_posture = 0.2;
    double w_force = 1.0e-5;
    double w_force_track = 0.0;
    double w_tau = 1.0e-4;
    bool hard_stance_no_slip = false;
};

struct IdWbcInput
{
    RigidBodyDynamics dynamics{};
    Eigen::Vector3d desired_linear_acc_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_angular_acc_body = Eigen::Vector3d::Zero();
    std::array<bool, go2::kLegCount> contact{};
    std::array<Eigen::Vector3d, go2::kLegCount> swing_acc_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> stance_acc_world{};
    bool have_stance_acc = false;
    bool have_force_ref = false;
    Eigen::Matrix<double, 12, 1> force_ref =
        Eigen::Matrix<double, 12, 1>::Zero();
};

struct IdWbcOutput
{
    bool ok = false;
    int iterations = 0;
    double eq_residual = 0.0;
    double rne_residual = 0.0;
    Eigen::Matrix<double, kGo2Nv, 1> qdd =
        Eigen::Matrix<double, kGo2Nv, 1>::Zero();
    Eigen::Matrix<double, 12, 1> force =
        Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, go2::kJointCount, 1> tau =
        Eigen::Matrix<double, go2::kJointCount, 1>::Zero();
};

inline Eigen::Matrix<double, 12, kGo2Nv> StackFootJacobian(
    const RigidBodyDynamics &dyn)
{
    Eigen::Matrix<double, 12, kGo2Nv> J = Eigen::Matrix<double, 12, kGo2Nv>::Zero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        J.block<3, kGo2Nv>(static_cast<int>(3 * leg), 0) = dyn.foot_jac_world[leg];
    return J;
}

inline bool SolveInverseDynamicsWbc(
    const IdWbcParams &params,
    const IdWbcInput &input,
    IdWbcOutput &output)
{
    output = IdWbcOutput{};
    if (!input.dynamics.valid)
        return false;
    const auto &M = input.dynamics.mass_matrix;
    const auto &h = input.dynamics.bias;
    const auto J = StackFootJacobian(input.dynamics);
    if (!M.allFinite() || !h.allFinite() || !J.allFinite())
        return false;

    constexpr int nqdd = kGo2Nv;
    constexpr int nf = 12;
    constexpr int n = nqdd + nf;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);

    Eigen::Matrix<double, 6, 1> a_des;
    a_des << input.desired_linear_acc_world, input.desired_angular_acc_body;
    Eigen::Matrix<double, 6, 6> Wb = Eigen::Matrix<double, 6, 6>::Zero();
    const double w_base_x = params.w_base_lin_x >= 0.0
        ? params.w_base_lin_x : params.w_base_lin;
    Wb.diagonal() << w_base_x, params.w_base_lin, params.w_base_lin,
        params.w_base_ang, params.w_base_ang, params.w_base_ang;
    H.topLeftCorner<6, 6>() += 2.0 * Wb;
    g.head<6>() += -2.0 * Wb * a_des;

    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto Jl = input.dynamics.foot_jac_world[leg];
        const int col_f = nqdd + static_cast<int>(3 * leg);
        if (input.contact[leg])
        {
            if (!params.hard_stance_no_slip)
            {
                const double wx = params.w_stance_no_slip_x >= 0.0
                    ? params.w_stance_no_slip_x
                    : params.w_stance_no_slip;
                const double wy = params.w_stance_no_slip_y >= 0.0
                    ? params.w_stance_no_slip_y
                    : params.w_stance_no_slip;
                const double wz = params.w_stance_no_slip_z >= 0.0
                    ? params.w_stance_no_slip_z
                    : params.w_stance_no_slip;
                const Eigen::Matrix3d Wns =
                    Eigen::Vector3d(wx, wy, wz).asDiagonal();
                H.topLeftCorner(nqdd, nqdd) +=
                    2.0 * Jl.transpose() * Wns * Jl;
                const Eigen::Vector3d jdot_qvel =
                    input.dynamics.foot_jac_dot_world[leg] *
                    input.dynamics.qvel;
                const Eigen::Vector3d a_des = input.have_stance_acc
                    ? input.stance_acc_world[leg]
                    : Eigen::Vector3d::Zero();
                g.head(nqdd) +=
                    2.0 * Jl.transpose() * Wns * (jdot_qvel - a_des);
            }
        }
        else
        {
            Eigen::Matrix3d Wswing = Eigen::Matrix3d::Identity() * params.w_swing;
            if (params.w_swing_x >= 0.0)
                Wswing(0, 0) = params.w_swing_x;
            H.topLeftCorner(nqdd, nqdd) +=
                2.0 * Jl.transpose() * Wswing * Jl;
            g.head(nqdd) +=
                -2.0 * Jl.transpose() * Wswing * input.swing_acc_world[leg];
        }
        H(col_f, col_f) += 2.0 * params.w_force;
        H(col_f + 1, col_f + 1) += 2.0 * params.w_force;
        H(col_f + 2, col_f + 2) += 2.0 * params.w_force;
        if (params.w_force_track > 0.0 && input.have_force_ref)
        {
            H(col_f, col_f) += 2.0 * params.w_force_track;
            H(col_f + 1, col_f + 1) += 2.0 * params.w_force_track;
            H(col_f + 2, col_f + 2) += 2.0 * params.w_force_track;
            g[col_f] +=
                -2.0 * params.w_force_track * input.force_ref[3 * static_cast<int>(leg) + 0];
            g[col_f + 1] +=
                -2.0 * params.w_force_track * input.force_ref[3 * static_cast<int>(leg) + 1];
            g[col_f + 2] +=
                -2.0 * params.w_force_track * input.force_ref[3 * static_cast<int>(leg) + 2];
        }
    }
    H.block(6, 6, 12, 12).diagonal().array() += 2.0 * params.w_posture;

    // tau = Mj qdd + hj - Jj^T f,  w_tau ||tau||^2
    const auto Mj = M.bottomRows<12>();
    const auto hj = h.tail<12>();
    const auto Jj_t = J.rightCols<12>().transpose();  // 12 x 12
    Eigen::MatrixXd tau_map = Eigen::MatrixXd::Zero(12, n);
    tau_map.block<12, 18>(0, 0) = Mj;
    tau_map.block<12, 12>(0, 18) = -Jj_t;
    H += 2.0 * params.w_tau * tau_map.transpose() * tau_map;
    g += 2.0 * params.w_tau * tau_map.transpose() * hj;
    H.diagonal().array() += 1.0e-8;

    int n_hard = 0;
    if (params.hard_stance_no_slip)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (input.contact[leg])
                n_hard += 3;
        }
    }
    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(6 + n_hard, n);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(6 + n_hard);
    Aeq.block(0, 0, 6, nqdd) = M.topRows<6>();
    Aeq.block(0, nqdd, 6, nf) = -J.leftCols<6>().transpose();
    beq.head<6>() = -h.head<6>();
    if (n_hard > 0)
    {
        int row_eq = 6;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!input.contact[leg])
                continue;
            const auto Jl = input.dynamics.foot_jac_world[leg];
            const Eigen::Vector3d jdot_qvel =
                input.dynamics.foot_jac_dot_world[leg] * input.dynamics.qvel;
            const Eigen::Vector3d a_des = input.have_stance_acc
                ? input.stance_acc_world[leg]
                : Eigen::Vector3d::Zero();
            Aeq.block(row_eq, 0, 3, nqdd) = Jl;
            beq.segment<3>(row_eq) = a_des - jdot_qvel;
            row_eq += 3;
        }
    }

    const double mu = params.friction_mu / std::sqrt(2.0);
    const int mineq = 6 * 4 + 24;
    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(mineq, n);
    Eigen::VectorXd bineq = Eigen::VectorXd::Zero(mineq);
    int row = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const int c = nqdd + static_cast<int>(3 * leg);
        if (!input.contact[leg])
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                Aineq(row, c + axis) = 1.0;
                bineq[row] = 0.05;
                ++row;
                Aineq(row, c + axis) = -1.0;
                bineq[row] = 0.05;
                ++row;
            }
            continue;
        }
        Aineq(row, c + 2) = -1.0;
        bineq[row] = -params.min_normal_n;
        ++row;
        Aineq(row, c + 2) = 1.0;
        bineq[row] = params.max_normal_n;
        ++row;
        Aineq(row, c + 0) = 1.0;
        Aineq(row, c + 2) = -mu;
        ++row;
        Aineq(row, c + 0) = -1.0;
        Aineq(row, c + 2) = -mu;
        ++row;
        Aineq(row, c + 1) = 1.0;
        Aineq(row, c + 2) = -mu;
        ++row;
        Aineq(row, c + 1) = -1.0;
        Aineq(row, c + 2) = -mu;
        ++row;
    }
    for (int i = 0; i < 12; ++i)
    {
        Aineq.row(row).head(n) = tau_map.row(i);
        bineq[row] = params.tau_limit_nm - hj[i];
        ++row;
        Aineq.row(row).head(n) = -tau_map.row(i);
        bineq[row] = params.tau_limit_nm + hj[i];
        ++row;
    }

    Eigen::VectorXd x;
    int iters = 0;
    DenseQpSettings settings;
    settings.max_iterations = 120;
    settings.rho = 8.0;
    settings.abs_tol = 1e-5;
    settings.rel_tol = 1e-4;
    settings.feasibility_tol = 1e-4;
    const bool qp_ok =
        SolveDenseQpEq(H, g, Aineq, bineq, Aeq, beq, x, iters, settings);
    if (x.size() != n || !x.allFinite())
        return false;

    output.ok = true;
    output.iterations = iters;
    output.qdd = x.head<nqdd>();
    output.force = x.tail<nf>();
    output.tau = Mj * output.qdd + hj - Jj_t * output.force;
    output.eq_residual = (Aeq * x - beq).norm();
    output.rne_residual =
        (M * output.qdd + h - J.transpose() * output.force).head<6>().norm();
    if (output.eq_residual >= 5.0)
        return false;
    output.ok = qp_ok || output.eq_residual < 1.0e-2;
    return output.ok;
}

}  // namespace go2_control
