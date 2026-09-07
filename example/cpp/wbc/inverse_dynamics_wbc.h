#pragma once

// Hierarchical inverse-dynamics WBC:
//   M qdd + h = S^T tau + J^T f
// Floating-base rows are equalities. Only support-leg forces are optimized;
// swing forces are identically zero. Friction / normal / torque limits
// are inequalities. Motion tasks are weighted: CoM/orientation from MPC,
// then swing, then posture. Output is joint tau* and qdd*.

#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "dense_qp.h"
#include "dense_qp_active_set.h"
#include "go2_forward_kinematics.h"
#include "terrain_control_interface.h"
#include "go2_rigid_body.h"

namespace go2_control
{

struct IdWbcParams
{
    double friction_mu = 0.8;
    double min_normal_n = 1.0;
    // Optional per-leg terrain handoff floors. Zero keeps min_normal_n.
    std::array<double, go2::kLegCount> min_normal_n_by_leg{};
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
    bool use_primal_active_set = false;
    bool prioritize_body_and_stance = false;
};

using IdWbcFootJacobian = Eigen::Matrix<double, 3, kGo2Nv>;
using IdWbcFootJacobianArray =
    std::array<IdWbcFootJacobian, go2::kLegCount>;

struct IdWbcInput
{
    RigidBodyDynamics dynamics{};
    Eigen::Vector3d desired_linear_acc_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_angular_acc_body = Eigen::Vector3d::Zero();
    // Optional articulated COM / angular-momentum objective. These are world
    // [cddot, Ldot] rows, not base translational / angular accelerations.
    // Caller supplies the SAME model's [Jcom; Amom], map_dot*qvel and explicit
    // weights with the appropriate physical units. Dynamics constraints stay
    // unchanged. The legacy base objective is used only when this is absent.
    bool have_centroidal_motion_task = false;
    // Independent body angular-acceleration feedback in the same QP. COM/L
    // tracking does not by itself regulate absolute body attitude. This flag
    // is valid only with the centroidal task and leaves legacy defaults alone.
    bool have_centroidal_orientation_task = false;
    Eigen::Matrix<double, 6, kGo2Nv> centroidal_motion_map =
        Eigen::Matrix<double, 6, kGo2Nv>::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 6, 1> centroidal_motion_bias =
        Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 6, 1> desired_centroidal_derivative =
        Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 6, 1> centroidal_motion_weights =
        Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    // contact is the mask actually applied to this WBC solve.  The following
    // fields carry terrain plan provenance without silently replacing it.
    bool has_terrain_plan = false;
    go2_terrain::TerrainPlanIdentity terrain_plan{};
    std::array<bool, go2::kLegCount> measured_contact{};
    bool measured_contact_valid = false;
    // contact may retain bounded robust support during filter grace. Keep
    // that fused safety mask explicit so it cannot be confused with planned.
    std::array<bool, go2::kLegCount> fused_contact{};
    bool fused_contact_valid = false;
    std::array<bool, go2::kLegCount> planned_contact{};
    bool planned_contact_valid = false;
    std::array<bool, go2::kLegCount> contact{};
    // Optional support-surface normals. Invalid entries retain the flat
    // world-Z friction cone, preserving the established flat WBC path.
    std::array<Eigen::Vector3d, go2::kLegCount> contact_normal{};
    std::array<bool, go2::kLegCount> contact_normal_valid{};
    // Physical acceleration at the foot collision-geom center, world frame.
    // The QP accounts for Jdot*qvel; callers must not pre-subtract it.
    std::array<Eigen::Vector3d, go2::kLegCount> swing_acc_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> stance_acc_world{};
    bool have_stance_acc = false;
    // Optional force-application Jacobians. Motion tasks continue to use
    // dynamics.foot_jac_world (the named geom center). When enabled, these
    // Jacobians are used only for J^T*f and the corresponding joint torque
    // map; callers must provide every leg and keep all entries finite.
    bool have_force_application_jacobian = false;
    IdWbcFootJacobianArray force_application_jac_world{
        IdWbcFootJacobian::Constant(std::numeric_limits<double>::quiet_NaN()),
        IdWbcFootJacobian::Constant(std::numeric_limits<double>::quiet_NaN()),
        IdWbcFootJacobian::Constant(std::numeric_limits<double>::quiet_NaN()),
        IdWbcFootJacobian::Constant(std::numeric_limits<double>::quiet_NaN())};
    bool have_force_ref = false;
    Eigen::Matrix<double, 12, 1> force_ref =
        Eigen::Matrix<double, 12, 1>::Zero();
};

struct IdWbcCostTerms
{
    double centroidal_motion = 0.0;
    double base_linear = 0.0;
    double base_angular = 0.0;
    double stance_no_slip = 0.0;
    double swing = 0.0;
    double force_regularization = 0.0;
    double force_tracking = 0.0;
    double posture = 0.0;
    double torque = 0.0;
};

struct IdWbcOutput
{
    bool ok = false;
    // Preserve solver-attempt status even when the caller falls back to the
    // last accepted command.  This makes a rare invalid tick diagnosable
    // without changing plant authority or relaxing a safety constraint.
    bool body_stance_priority_used = false;
    double priority_preservation_residual = 0.0;
    bool centroidal_motion_task_used = false;
    bool centroidal_orientation_task_used = false;
    bool qp_converged = false;
    bool qp_recovery_used = false;
    bool solution_finite = false;
    bool terrain_plan_consumed = false;
    go2_terrain::TerrainPlanIdentity terrain_plan{};
    std::array<bool, go2::kLegCount> measured_contact{};
    std::array<bool, go2::kLegCount> fused_contact{};
    std::array<bool, go2::kLegCount> planned_contact{};
    bool measured_contact_valid = false;
    bool fused_contact_valid = false;
    bool planned_contact_valid = false;
    int iterations = 0;
    int primary_iterations = 0;
    int recovery_iterations = 0;
    double recovery_correction_norm = 0.0;
    double recovery_max_inequality_violation = 0.0;
    double eq_residual = 0.0;
    double rne_residual = 0.0;
    Eigen::Matrix<double, kGo2Nv, 1> qdd =
        Eigen::Matrix<double, kGo2Nv, 1>::Zero();
    Eigen::Matrix<double, 12, 1> force =
        Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, go2::kJointCount, 1> tau =
        Eigen::Matrix<double, go2::kJointCount, 1>::Zero();
    // The torque inequalities are safety constraints, not merely a QP
    // objective. Retain the measured violation for callers/telemetry.
    double max_tau_violation_nm = 0.0;
    IdWbcCostTerms cost_terms{};
    // Per-contact cone activity is retained for terrain forensic telemetry.
    std::array<double, go2::kLegCount> friction_ratio{};
    std::array<double, go2::kLegCount> normal_force{};
};

inline Eigen::Matrix<double, 12, kGo2Nv> StackFootJacobian(
    const IdWbcFootJacobianArray &foot_jacobians)
{
    Eigen::Matrix<double, 12, kGo2Nv> J =
        Eigen::Matrix<double, 12, kGo2Nv>::Zero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        J.block<3, kGo2Nv>(static_cast<int>(3 * leg), 0) =
            foot_jacobians[leg];
    return J;
}

inline Eigen::Matrix<double, 12, kGo2Nv> StackFootJacobian(
    const RigidBodyDynamics &dyn)
{
    return StackFootJacobian(dyn.foot_jac_world);
}

// Select one force Jacobian convention for all dynamics terms. The default
// preserves the established geom-center path; an explicit request is
// fail-closed so a missing/invalid point Jacobian cannot silently fall back.
inline bool SelectIdWbcForceJacobians(
    const IdWbcInput &input, IdWbcFootJacobianArray &selected)
{
    selected = input.dynamics.foot_jac_world;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (!selected[leg].allFinite())
            return false;
    if (!input.have_force_application_jacobian)
        return true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!input.force_application_jac_world[leg].allFinite())
            return false;
        selected[leg] = input.force_application_jac_world[leg];
    }
    return true;
}
inline bool ValidateIdWbcTerrainReference(const IdWbcInput &input)
{
    if (!input.has_terrain_plan)
        return true;
    if (!input.terrain_plan.valid() || !input.measured_contact_valid ||
        !input.planned_contact_valid)
        return false;
    // C-005 callers explicitly provide the fused safety mask. Legacy terrain
    // callers retain their pre-C-005 interface until they opt into it.
    if (input.fused_contact_valid)
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            if (input.contact[leg] && !input.fused_contact[leg])
                return false;
    return true;
}


struct IdWbcQpSnapshot {
    Eigen::MatrixXd H,Aineq,Aeq;
    Eigen::VectorXd g,bineq,beq,iterate,seed;
    std::string stage, failure;
};
inline bool SolveInverseDynamicsWbc(
    const IdWbcParams &params,
    const IdWbcInput &input,
    IdWbcOutput &output, IdWbcQpSnapshot *snapshot = nullptr)
{
    output = IdWbcOutput{};
    if (!input.dynamics.valid || !ValidateIdWbcTerrainReference(input))
        return false;
    output.terrain_plan_consumed = input.has_terrain_plan;
    output.terrain_plan = input.terrain_plan;
    output.measured_contact = input.measured_contact;
    output.fused_contact = input.fused_contact_valid
        ? input.fused_contact : input.contact;
    output.planned_contact = input.planned_contact;
    output.measured_contact_valid = input.measured_contact_valid;
    output.fused_contact_valid = input.fused_contact_valid;
    output.planned_contact_valid = input.planned_contact_valid;
    const auto &M = input.dynamics.mass_matrix;
    const auto &h = input.dynamics.bias;
    const auto J = StackFootJacobian(input.dynamics);
    IdWbcFootJacobianArray selected_force_foot_jacobians;
    if (!SelectIdWbcForceJacobians(input, selected_force_foot_jacobians))
        return false;
    const auto J_force = StackFootJacobian(selected_force_foot_jacobians);
    if (!M.allFinite() || !h.allFinite() || !J.allFinite() ||
        !J_force.allFinite())
        return false;

    constexpr int nqdd = kGo2Nv;
    // Contact mode defines the force variables. An unconverged iterate cannot
    // borrow force from a swing leg because that variable does not exist.
    std::array<int, go2::kLegCount> force_offset;
    force_offset.fill(-1);
    int nf = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (input.contact[leg])
        {
            force_offset[leg] = nf;
            nf += 3;
        }
    Eigen::MatrixXd force_map = Eigen::MatrixXd::Zero(12, nf);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (input.contact[leg])
            force_map.block<3, 3>(3 * static_cast<int>(leg), force_offset[leg]).setIdentity();
    const int n = nqdd + nf;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd priority_map(0,n);Eigen::VectorXd priority_target(0);
    const auto add_priority=[&](const auto &map,const auto &target,const auto &weights) {
        if(!params.prioritize_body_and_stance)return;
        for(int r=0;r<map.rows();++r)if(weights[r]>0) {
            const int k=priority_map.rows();priority_map.conservativeResize(k+1,n);
            priority_map.row(k).setZero();priority_map.row(k).head(nqdd)=std::sqrt(weights[r])*map.row(r);
            priority_target.conservativeResize(k+1);priority_target[k]=std::sqrt(weights[r])*target[r];
        }
    };
    if(params.prioritize_body_and_stance && !params.use_primal_active_set)return false;

    if (input.have_centroidal_orientation_task && !input.have_centroidal_motion_task)
        return false;
    Eigen::Matrix<double, 6, 1> a_des;
    a_des << input.desired_linear_acc_world, input.desired_angular_acc_body;
    Eigen::Matrix<double, 6, 6> Wb = Eigen::Matrix<double, 6, 6>::Zero();
    const double w_base_x = params.w_base_lin_x >= 0.0
        ? params.w_base_lin_x : params.w_base_lin;
    Wb.diagonal() << w_base_x, params.w_base_lin, params.w_base_lin,
        params.w_base_ang, params.w_base_ang, params.w_base_ang;
    if (input.have_centroidal_motion_task)
    {
        if (!input.centroidal_motion_map.allFinite() ||
            !input.centroidal_motion_bias.allFinite() ||
            !input.desired_centroidal_derivative.allFinite() ||
            !input.centroidal_motion_weights.allFinite() ||
            (input.centroidal_motion_weights.array() < 0.0).any() ||
            !(input.centroidal_motion_weights.maxCoeff() > 0.0))
            return false;
        const auto weights = input.centroidal_motion_weights.asDiagonal();
        const auto &map = input.centroidal_motion_map;
        H.topLeftCorner<nqdd, nqdd>() += 2.0 * map.transpose() * weights * map;
        g.head<nqdd>() += 2.0 * map.transpose() * weights *
            (input.centroidal_motion_bias - input.desired_centroidal_derivative);
        add_priority(map.topRows<3>(),
            (input.desired_centroidal_derivative-input.centroidal_motion_bias).head<3>(),
            input.centroidal_motion_weights.head<3>());
        output.centroidal_motion_task_used = true;
        if (input.have_centroidal_orientation_task)
        {
            if (!input.desired_angular_acc_body.allFinite() ||
                !std::isfinite(params.w_base_ang) || params.w_base_ang <= 0.0)
                return false;
            H.block<3,3>(3,3).diagonal().array() += 2.0 * params.w_base_ang;
            g.segment<3>(3) -= 2.0 * params.w_base_ang * input.desired_angular_acc_body;
            Eigen::Matrix<double,3,nqdd> orientation_map=Eigen::Matrix<double,3,nqdd>::Zero();
            orientation_map.block<3,3>(0,3).setIdentity();
            add_priority(orientation_map,input.desired_angular_acc_body,Eigen::Vector3d::Constant(params.w_base_ang));
            output.centroidal_orientation_task_used = true;
        }
    }
    else
    {
        H.topLeftCorner<6, 6>() += 2.0 * Wb;
        g.head<6>() += -2.0 * Wb * a_des;
        Eigen::Matrix<double,6,nqdd> body_map=Eigen::Matrix<double,6,nqdd>::Zero();body_map.leftCols<6>().setIdentity();
        add_priority(body_map,a_des,Wb.diagonal());
    }

    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto Jl = input.dynamics.foot_jac_world[leg];
        const int col_f = nqdd + force_offset[leg];
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
                add_priority(Jl,a_des-jdot_qvel,Wns.diagonal());
            }
        }
        else
        {
            Eigen::Matrix3d Wswing = Eigen::Matrix3d::Identity() * params.w_swing;
            if (params.w_swing_x >= 0.0)
                Wswing(0, 0) = params.w_swing_x;
            H.topLeftCorner(nqdd, nqdd) +=
                2.0 * Jl.transpose() * Wswing * Jl;
            const Eigen::Vector3d jdot_qvel =
                input.dynamics.foot_jac_dot_world[leg] * input.dynamics.qvel;
            g.head(nqdd) +=
                2.0 * Jl.transpose() * Wswing *
                (jdot_qvel - input.swing_acc_world[leg]);
        }
        if (!input.contact[leg])
            continue;
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
    const auto Jj_t = J_force.rightCols<12>().transpose();  // 12 x 12
    Eigen::MatrixXd tau_map = Eigen::MatrixXd::Zero(12, n);
    tau_map.block<12, 18>(0, 0) = Mj;
    if (nf > 0)
        tau_map.block(0, nqdd, 12, nf) = -Jj_t * force_map;
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
    if (nf > 0)
        Aeq.block(0, nqdd, 6, nf) =
            -J_force.leftCols<6>().transpose() * force_map;
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
    const int mineq = 2 * nf + 24;
    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(mineq, n);
    Eigen::VectorXd bineq = Eigen::VectorXd::Zero(mineq);
    int row = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!input.contact[leg])
            continue;
        const int c = nqdd + force_offset[leg];
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        if (input.contact_normal_valid[leg] &&
            input.contact_normal[leg].allFinite() &&
            input.contact_normal[leg].norm() > 1.0e-6)
        {
            normal = input.contact_normal[leg].normalized();
            if (normal.z() < 0.0)
                normal = -normal;
        }
        Aineq.row(row).segment<3>(c) = -normal.transpose();
        const double per_leg_floor = params.min_normal_n_by_leg[leg];
        bineq[row] = -std::max(
            params.min_normal_n,
            std::isfinite(per_leg_floor) && per_leg_floor > 0.0
                ? per_leg_floor : params.min_normal_n);
        ++row;
        Aineq.row(row).segment<3>(c) = normal.transpose();
        bineq[row] = params.max_normal_n;
        ++row;
        // Choose a deterministic tangent basis. For the flat default this is
        // exactly world X/Y; terrain contacts instead use the measured
        // support-plane normal, so the cone is not spuriously narrowed by a
        // tilted three-foot stance.
        Eigen::Vector3d tangent_x =
            std::abs(normal.z()) < 0.9
                ? normal.cross(Eigen::Vector3d::UnitZ()).normalized()
                : Eigen::Vector3d::UnitX();
        Eigen::Vector3d tangent_y = normal.cross(tangent_x).normalized();
        const auto add_cone_row = [&](const Eigen::Vector3d &axis) {
            Aineq.row(row).segment<3>(c) = axis.transpose();
            Aineq.row(row).segment<3>(c) -= mu * normal.transpose();
            ++row;
        };
        add_cone_row(tangent_x);
        add_cone_row(-tangent_x);
        add_cone_row(tangent_y);
        add_cone_row(-tangent_y);
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

    if(snapshot) { snapshot->H=H; snapshot->g=g; snapshot->Aineq=Aineq;
        snapshot->bineq=bineq; snapshot->Aeq=Aeq; snapshot->beq=beq; }
    Eigen::VectorXd x;
    int iters = 0;
    DenseQpSettings settings;
    settings.max_iterations = 120;
    settings.rho = 8.0;
    settings.abs_tol = 1e-5;
    settings.rel_tol = 1e-4;
    settings.feasibility_tol = 1e-4;
    bool qp_ok=false;
    if(params.use_primal_active_set) {
        Eigen::VectorXd seed=Eigen::VectorXd::Zero(n);
        seed.head(nqdd)=M.ldlt().solve(-h);
        if(params.prioritize_body_and_stance && priority_map.rows()>0) {
            Eigen::MatrixXd Hp=2.0*priority_map.transpose()*priority_map;
            Hp.diagonal().array()+=1e-6;
            const Eigen::VectorXd gp=-2.0*priority_map.transpose()*priority_target;
            Eigen::VectorXd primary;int first_iters=0;
            DenseQpActiveSetDiagnostics primary_diagnostic;
            if(snapshot){snapshot->stage="body_stance_primary";snapshot->H=Hp;snapshot->g=gp;snapshot->seed=seed;}
            qp_ok=SolveDenseQpPrimalActiveSet(Hp,gp,Aineq,bineq,Aeq,beq,seed,primary,first_iters,
                snapshot ? &primary_diagnostic : nullptr);
            if(snapshot) snapshot->failure=primary_diagnostic.failure;
            if(qp_ok) {
                Eigen::MatrixXd E(Aeq.rows()+priority_map.rows(),n);
                Eigen::VectorXd d(beq.size()+priority_map.rows());
                E<<Aeq,priority_map;d<<beq,priority_map*primary;
                DenseQpActiveSetDiagnostics secondary_diagnostic;
                if(snapshot){snapshot->stage="remaining_tasks_secondary";snapshot->H=H;snapshot->g=g;
                    snapshot->Aeq=E;snapshot->beq=d;snapshot->seed=primary;}
                qp_ok=SolveDenseQpPrimalActiveSet(H,g,Aineq,bineq,E,d,primary,x,iters,
                    snapshot ? &secondary_diagnostic : nullptr);
                if(snapshot) snapshot->failure=secondary_diagnostic.failure;
                if(qp_ok) {
                    output.priority_preservation_residual=(priority_map*(x-primary)).lpNorm<Eigen::Infinity>();
                    output.body_stance_priority_used=true;
                }
            }
            iters+=first_iters;
        } else {
            qp_ok=SolveDenseQpPrimalActiveSet(H,g,Aineq,bineq,Aeq,beq,seed,x,iters);
        }
    } else {
        qp_ok=SolveDenseQpEq(H, g, Aineq, bineq, Aeq, beq, x, iters, settings);
    }
    output.primary_iterations = iters;
    const Eigen::VectorXd primary_x = x;
    const auto torque_violation = [&](const Eigen::VectorXd &candidate) {
        if (candidate.size() != n || !candidate.allFinite())
            return std::numeric_limits<double>::infinity();
        const Eigen::Matrix<double, 12, 1> tau =
            tau_map * candidate + hj;
        return (tau.cwiseAbs().array() - params.tau_limit_nm)
            .max(0.0).maxCoeff();
    };
    if (!params.use_primal_active_set && primary_x.size() == n && primary_x.allFinite() &&
        torque_violation(primary_x) > 5.0e-2)
    {
        DenseQpSettings recovery_settings = settings;
        recovery_settings.max_iterations = 600;
        recovery_settings.rho = 10.0;
        int recovery_iters = 0;
        output.qp_recovery_used = true;
        qp_ok = SolveDenseQpEqNullspace(
            H, g, Aineq, bineq, Aeq, beq, x, recovery_iters,
            recovery_settings);
        output.recovery_iterations = recovery_iters;
        iters += recovery_iters;
        if (x.size() == n && x.allFinite())
        {
            output.recovery_correction_norm = (x - primary_x).norm();
            output.recovery_max_inequality_violation =
                (Aineq * x - bineq).cwiseMax(0.0).maxCoeff();
        }
    }
    if(snapshot) snapshot->iterate=x;
    output.qp_converged = qp_ok;
    output.iterations = iters;
    if (x.size() != n || !x.allFinite())
        return false;

    output.solution_finite = true;
    output.ok = !params.use_primal_active_set || qp_ok;
    output.qdd = x.head<nqdd>();
    output.force.setZero();
    if (nf > 0)
        output.force.noalias() = force_map * x.tail(nf);
    output.tau = Mj * output.qdd + hj - Jj_t * output.force;
    output.eq_residual = (Aeq * x - beq).norm();
    output.max_tau_violation_nm =
        (output.tau.cwiseAbs().array() - params.tau_limit_nm).max(0.0).maxCoeff();
    // DenseQpEq may return an equality-accurate iterate before ADMM has
    // driven every inequality to tolerance. Never pass such an iterate to
    // the plant: the caller will retain its last validated solution.
    if (!std::isfinite(output.max_tau_violation_nm) ||
        output.max_tau_violation_nm > 5.0e-2)
    {
        output.ok = false;
        return false;
    }
    output.rne_residual =
        (M * output.qdd + h - J_force.transpose() * output.force)
            .head<6>().norm();

    // Keep the objective decomposition alongside the solution.  These are
    // diagnostic terms only; the solver objective remains exactly the same.
    const Eigen::Vector3d base_lin_error =
        output.qdd.segment<3>(0) - input.desired_linear_acc_world;
    const Eigen::Vector3d base_ang_error =
        output.qdd.segment<3>(3) - input.desired_angular_acc_body;
    output.cost_terms.base_linear =
        (params.w_base_lin_x >= 0.0 ? params.w_base_lin_x
                                    : params.w_base_lin) * base_lin_error.x() * base_lin_error.x() +
        params.w_base_lin * (base_lin_error.y() * base_lin_error.y() +
                             base_lin_error.z() * base_lin_error.z());
    output.cost_terms.base_angular = params.w_base_ang * base_ang_error.squaredNorm();
    if (input.have_centroidal_motion_task)
    {
        const Eigen::Matrix<double, 6, 1> error = input.centroidal_motion_map *
            output.qdd + input.centroidal_motion_bias - input.desired_centroidal_derivative;
        output.cost_terms.centroidal_motion =
            (input.centroidal_motion_weights.array() * error.array().square()).sum();
        output.cost_terms.base_linear = 0.0;
        if (!input.have_centroidal_orientation_task)
            output.cost_terms.base_angular = 0.0;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto Jl = input.dynamics.foot_jac_world[leg];
        const Eigen::Vector3d jdot_qvel =
            input.dynamics.foot_jac_dot_world[leg] * input.dynamics.qvel;
        if (input.contact[leg])
        {
            const double wx = params.w_stance_no_slip_x >= 0.0
                ? params.w_stance_no_slip_x : params.w_stance_no_slip;
            const double wy = params.w_stance_no_slip_y >= 0.0
                ? params.w_stance_no_slip_y : params.w_stance_no_slip;
            const double wz = params.w_stance_no_slip_z >= 0.0
                ? params.w_stance_no_slip_z : params.w_stance_no_slip;
            const Eigen::Vector3d stance_error = Jl * output.qdd + jdot_qvel -
                (input.have_stance_acc ? input.stance_acc_world[leg]
                                        : Eigen::Vector3d::Zero());
            output.cost_terms.stance_no_slip += wx * stance_error.x() * stance_error.x() +
                wy * stance_error.y() * stance_error.y() + wz * stance_error.z() * stance_error.z();
        }
        else
        {
            const Eigen::Vector3d swing_error =
                Jl * output.qdd + jdot_qvel - input.swing_acc_world[leg];
            const double wx = params.w_swing_x >= 0.0 ? params.w_swing_x : params.w_swing;
            output.cost_terms.swing += wx * swing_error.x() * swing_error.x() +
                params.w_swing * (swing_error.y() * swing_error.y() + swing_error.z() * swing_error.z());
        }
        const Eigen::Vector3d force = output.force.segment<3>(3 * static_cast<int>(leg));
        output.cost_terms.force_regularization += params.w_force * force.squaredNorm();
        if (params.w_force_track > 0.0 && input.have_force_ref)
            output.cost_terms.force_tracking += params.w_force_track *
                (force - input.force_ref.segment<3>(3 * static_cast<int>(leg))).squaredNorm();
    }
    output.cost_terms.posture = params.w_posture * output.qdd.tail<12>().squaredNorm();
    output.cost_terms.torque = params.w_tau * output.tau.squaredNorm();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        if (input.contact_normal_valid[leg] &&
            input.contact_normal[leg].allFinite() &&
            input.contact_normal[leg].norm() > 1.0e-6)
        {
            normal = input.contact_normal[leg].normalized();
            if (normal.z() < 0.0)
                normal = -normal;
        }
        if (!input.contact[leg])
            continue;
        const Eigen::Vector3d force =
            output.force.segment<3>(3 * static_cast<int>(leg));
        const double fn = normal.dot(force);
        const Eigen::Vector3d tangent_x =
            std::abs(normal.z()) < 0.9
                ? normal.cross(Eigen::Vector3d::UnitZ()).normalized()
                : Eigen::Vector3d::UnitX();
        const Eigen::Vector3d tangent_y = normal.cross(tangent_x).normalized();
        const double axis_limit = params.friction_mu / std::sqrt(2.0) * fn;
        output.normal_force[leg] = fn;
        output.friction_ratio[leg] = axis_limit > 1.0e-9
            ? std::max(std::abs(tangent_x.dot(force)),
                       std::abs(tangent_y.dot(force))) / axis_limit : 0.0;
    }
    if (output.eq_residual >= 5.0)
        return false;
    output.ok = qp_ok || output.eq_residual < 1.0e-2;
    if (input.has_terrain_plan)
    {
        output.terrain_plan_consumed = true;
        output.terrain_plan = input.terrain_plan;
    }
    return output.ok;
}

}  // namespace go2_control
