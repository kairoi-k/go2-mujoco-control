#include "trot_experiment.h"
#include "trot_true_dynamics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "centroidal_wbc.h"
#include "contact_wrench_lexicographic_allocator.h"
#include "contact_wrench_projected_allocator.h"
#include "contact_wrench_qp.h"
#include "contact_state_filter.h"
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "inverse_dynamics_wbc.h"
#include "motion_frame_utils.h"
#include "preview_footstep_horizon.h"
#include "srbd_mpc.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

namespace
{

go2_control::RigidBodyState MakeRigidBodyState(
    const unitree_go::msg::dds_::LowState_ &low,
    const unitree_go::msg::dds_::SportModeState_ &high,
    const Eigen::Vector3d &linear_vel_world)
{
    const WorldPose pose = ComputeWorldPose(low, high);
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(pose.base.x, pose.base.y, pose.base.z);
    state.quat_world_from_body = Eigen::Quaterniond(
        pose.quaternion[0], pose.quaternion[1],
        pose.quaternion[2], pose.quaternion[3]);
    state.linear_vel_world = linear_vel_world;
    state.angular_vel_body = Eigen::Vector3d(
        low.imu_state().gyroscope()[0],
        low.imu_state().gyroscope()[1],
        low.imu_state().gyroscope()[2]);
    for (int i = 0; i < kMotorCount; ++i)
    {
        state.q[i] = low.motor_state()[i].q();
        state.dq[i] = low.motor_state()[i].dq();
    }
    return state;
}

Eigen::Vector3d ClampVec3(const Eigen::Vector3d &v, double lim)
{
    Eigen::Vector3d out = v;
    for (int i = 0; i < 3; ++i)
        out[i] = std::clamp(out[i], -lim, lim);
    return out;
}

}  // namespace

void TrotExperiment::UpdateWbcFull(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot)
{
    wbc_shadow_diagnostics_.enabled = true;
    if (!rigid_body_ || !rigid_body_->loaded())
        return;
    const double pitch_abs = std::abs(
        static_cast<double>(state_snapshot.imu_state().rpy()[1]));
    const double roll_abs = std::abs(
        static_cast<double>(state_snapshot.imu_state().rpy()[0]));
    const double attitude_fade = std::clamp(
        1.0 - std::max(0.0, std::max(pitch_abs, roll_abs) - 0.06) / 0.10,
        0.0, 1.0);
    const double v_body = have_filtered_body_velocity_
        ? std::abs(latest_filtered_body_velocity_[0])
        : 0.0;
    const double speed_lock = Smoothstep((v_body - 0.40) / 0.60);
    const double cycle_lock =
        Smoothstep((static_cast<double>(completed_cycles_) - 24.0) / 20.0);
    const double cart_lock = params_.cartesian_world
        ? std::min(cycle_lock, speed_lock) * attitude_fade
        : 0.0;
    Eigen::Vector3d linear_vel_world(
        high_state_snapshot.velocity()[0],
        high_state_snapshot.velocity()[1],
        high_state_snapshot.velocity()[2]);
    go2_control::RigidBodyDynamics dyn;
    if (!rigid_body_->Evaluate(
            MakeRigidBodyState(
                state_snapshot, high_state_snapshot, linear_vel_world),
            dyn))
    {
        return;
    }
    if (!dynamics_logged_)
    {
        dynamics_logged_ = true;
        std::cout << "WBC-FULL rigid body mass=" << dyn.mass_kg
                  << " com_z=" << dyn.com_world.z()
                  << " bias_z=" << dyn.bias[2] << "\n";
    }

    std::array<bool, go2::kLegCount> measured_contact{};
    const go2_control::HystereticContactParams contact_params{
        kShadowContactOnForceN, kShadowContactOffForceN};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const double force = state_snapshot.foot_force()[leg];
        bool next_contact = false;
        if (!go2_control::UpdateHystereticContact(
                wbc_shadow_contact_state_[leg], force, contact_params,
                next_contact))
            return;
        wbc_shadow_contact_state_[leg] = next_contact;
        measured_contact[leg] = next_contact;
    }
    // During trot the force sensors stay high through lift-off, so measured
    // 3/4-contact at a scheduled diagonal. The QP uses the gait schedule.
    std::array<bool, go2::kLegCount> qp_contact = measured_contact;
    const double gait_period =
        kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
    const double gait_duty =
        kernel_duty_factor_ > 0.05 ? kernel_duty_factor_ : params_.duty_factor;
    if (gait_started_ && motion_stage_ == 2)
    {
        std::array<std::array<bool, go2::kLegCount>, go2_control::kSrbdMaxHorizon>
            scheduled{};
        go2_control::FillTrotContactSchedulePhase(
            current_phase_, gait_period, gait_duty, 1, 0.0, scheduled);
        qp_contact = scheduled[0];
    }
    int contact_mask = 0;
    int active = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (qp_contact[leg])
        {
            ++active;
            contact_mask |= 1 << static_cast<int>(leg);
        }
    }
    wbc_shadow_diagnostics_.active_contacts = active;
    wbc_shadow_diagnostics_.contact_mask = contact_mask;

    go2_control::SrbdMpcParams mpc_params;
    mpc_params.horizon = 8;
    mpc_params.dt_s = std::clamp(gait_period / 8.0, 0.020, 0.05);
    mpc_params.mass_kg = dyn.mass_kg;
    mpc_params.inertia_com_world = dyn.inertia_com_world;
    mpc_params.friction_mu = kShadowWbcFrictionCoefficient;
    if (params_.cartesian_world)
    {
        mpc_params.w_vel_xy = 60.0 + 30.0 * cart_lock;
        mpc_params.w_pos_xy = 6.0;
        mpc_params.w_ori = 120.0;
        mpc_params.w_vel_z = 12.0;
        mpc_params.w_omega = 8.0;
        mpc_params.w_force = 3.0e-4;
    }
    else if (!params_.step_plan.empty())
    {
        mpc_params.w_vel_xy = 80.0;
        mpc_params.w_pos_xy = 20.0;
    }
    const bool run_mpc =
        (wbc_full_ticks_ % (params_.cartesian_world ? 10 : 25)) == 0 ||
        !last_srbd_.ok;
    if (run_mpc)
    {
        go2_control::SrbdMpcInput mpc_in;
        mpc_in.state[0] = state_snapshot.imu_state().rpy()[0];
        mpc_in.state[1] = state_snapshot.imu_state().rpy()[1];
        mpc_in.state[2] = state_snapshot.imu_state().rpy()[2];
        mpc_in.state[3] = dyn.com_world.x();
        mpc_in.state[4] = dyn.com_world.y();
        mpc_in.state[5] = dyn.com_world.z();
        mpc_in.state[6] = state_snapshot.imu_state().gyroscope()[0];
        mpc_in.state[7] = state_snapshot.imu_state().gyroscope()[1];
        mpc_in.state[8] = state_snapshot.imu_state().gyroscope()[2];
        mpc_in.state[9] = linear_vel_world.x();
        mpc_in.state[10] = linear_vel_world.y();
        mpc_in.state[11] = linear_vel_world.z();
        mpc_in.reference = mpc_in.state;
        mpc_in.reference[0] = 0.0;
        mpc_in.reference[1] = 0.0;
        mpc_in.reference[4] = 0.0;
        mpc_in.reference[5] = kWbcPrimaryBaseHeightM;
        mpc_in.reference[6] = 0.0;
        mpc_in.reference[7] = 0.0;
        mpc_in.reference[8] = params_.turn_rate_radps;
        const double v_cmd =
            gait_started_ && motion_stage_ == 2
                ? (std::isfinite(kernel_nominal_velocity_x_mps_) &&
                           std::abs(kernel_nominal_velocity_x_mps_) > 1.0e-6
                       ? kernel_nominal_velocity_x_mps_
                       : params_.direction_sign * params_.step_length_m /
                             params_.period_s)
                : 0.0;
        const double yaw =
            static_cast<double>(state_snapshot.imu_state().rpy()[2]);
        if (params_.cartesian_world)
        {
            mpc_in.reference[2] = world_reference_yaw_rad_;
            mpc_in.reference[4] = world_reference_y_m_;
            const double yaw_err = WrapAngle(
                yaw - world_reference_yaw_rad_);
            mpc_in.reference[8] = Clamp(-1.2 * yaw_err, -0.30, 0.30);
            mpc_in.reference[9] =
                v_cmd * std::cos(world_reference_yaw_rad_);
            mpc_in.reference[10] =
                v_cmd * std::sin(world_reference_yaw_rad_);
            mpc_in.reference[3] =
                dyn.com_world.x() + mpc_in.reference[9] * mpc_params.dt_s *
                                        0.5 * mpc_params.horizon;
        }
        else
        {
            mpc_in.reference[9] = v_cmd;
            mpc_in.reference[10] = 0.0;
        }
        mpc_in.reference[11] = 0.0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            mpc_in.foot_from_com_world[leg] =
                dyn.foot_pos_world[leg] - dyn.com_world;
        if (gait_started_ && motion_stage_ == 2)
        {
            go2_control::FillTrotContactSchedulePhase(
                current_phase_, gait_period, gait_duty,
                mpc_params.horizon, mpc_params.dt_s, mpc_in.contact);
        }
        else
        {
            for (int k = 0; k < mpc_params.horizon; ++k)
                mpc_in.contact[k] = qp_contact;
        }
        go2_control::SrbdMpcOutput mpc_out;
        if (go2_control::SolveSrbdMpc(mpc_params, mpc_in, mpc_out) && mpc_out.ok)
            last_srbd_ = mpc_out;
    }
    ++wbc_full_ticks_;
    wbc_shadow_diagnostics_.srbd_ok = last_srbd_.ok;

    go2_control::IdWbcInput wbc_in;
    wbc_in.dynamics = dyn;
    wbc_in.contact = qp_contact;
    if (last_srbd_.ok)
    {
        wbc_in.desired_linear_acc_world = last_srbd_.first_linear_acc;
        const Eigen::Quaterniond quat(
            state_snapshot.imu_state().quaternion()[0],
            state_snapshot.imu_state().quaternion()[1],
            state_snapshot.imu_state().quaternion()[2],
            state_snapshot.imu_state().quaternion()[3]);
        wbc_in.desired_angular_acc_body =
            quat.normalized().toRotationMatrix().transpose() *
            last_srbd_.first_angular_acc;
        if (have_filtered_body_velocity_ && gait_started_ &&
            motion_stage_ == 2 &&
            (params_.cartesian_world || !params_.step_plan.empty()))
        {
            const double v_des =
                std::isfinite(kernel_nominal_velocity_x_mps_)
                    ? kernel_nominal_velocity_x_mps_
                    : 0.0;
            const double v_err =
                v_des - latest_filtered_body_velocity_[0];
            const double yaw =
                static_cast<double>(state_snapshot.imu_state().rpy()[2]);
            const double pitch =
                static_cast<double>(state_snapshot.imu_state().rpy()[1]);
            const double gyro_y =
                static_cast<double>(
                    state_snapshot.imu_state().gyroscope()[1]);
            const double pitch_fade = Clamp(
                1.0 - std::max(0.0, std::abs(pitch) - 0.06) / 0.12,
                0.15, 1.0);
            const double ax_lim = params_.cartesian_world ? 1.0 : 3.0;
            const double ax_gain = params_.cartesian_world ? 2.0 : 4.0;
            const double ax_body =
                pitch_fade * Clamp(ax_gain * v_err, -ax_lim, ax_lim);
            wbc_in.desired_linear_acc_world.x() += ax_body * std::cos(yaw);
            wbc_in.desired_linear_acc_world.y() += ax_body * std::sin(yaw);
            const double roll =
                static_cast<double>(state_snapshot.imu_state().rpy()[0]);
            const double gyro_x =
                static_cast<double>(
                    state_snapshot.imu_state().gyroscope()[0]);
            wbc_in.desired_angular_acc_body.x() +=
                (params_.cartesian_world ? -40.0 : -20.0) * roll -
                (params_.cartesian_world ? 5.0 : 2.5) * gyro_x;
            wbc_in.desired_angular_acc_body.y() +=
                -12.0 * pitch - 1.5 * gyro_y - 0.25 * ax_body;
            if (params_.cartesian_world)
            {
                const double yaw_err = WrapAngle(
                    yaw - world_reference_yaw_rad_);
                const double gyro_z = static_cast<double>(
                    state_snapshot.imu_state().gyroscope()[2]);
                wbc_in.desired_angular_acc_body.z() +=
                    -8.0 * yaw_err - 2.0 * gyro_z;
            }
        }
    }
    if (have_commanded_body_feet_)
    {
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        const Eigen::Quaterniond quat(
            pose.quaternion[0], pose.quaternion[1],
            pose.quaternion[2], pose.quaternion[3]);
        const Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
        Eigen::Matrix<double, go2_control::kGo2Nv, 1> qvel =
            Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
        qvel.head<3>() = linear_vel_world;
        qvel.segment<3>(3) = Eigen::Vector3d(
            state_snapshot.imu_state().gyroscope()[0],
            state_snapshot.imu_state().gyroscope()[1],
            state_snapshot.imu_state().gyroscope()[2]);
        for (int i = 0; i < kMotorCount; ++i)
        {
            const int dof = rigid_body_->MotorDof(i);
            if (dof >= 6 && dof < go2_control::kGo2Nv)
                qvel[dof] = state_snapshot.motor_state()[i].dq();
        }
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const Eigen::Vector3d v = dyn.foot_jac_world[leg] * qvel;
            if (qp_contact[leg])
            {
                if (params_.cartesian_world && have_commanded_world_feet_)
                {
                    const Eigen::Vector3d p_des(
                        commanded_world_feet_[leg].x,
                        commanded_world_feet_[leg].y,
                        commanded_world_feet_[leg].z);
                    wbc_in.stance_acc_world[leg] = ClampVec3(
                        (40.0 + 120.0 * cart_lock) *
                                (p_des - dyn.foot_pos_world[leg]) -
                            (10.0 + 10.0 * cart_lock) * v,
                        8.0 + 12.0 * cart_lock);
                    wbc_in.have_stance_acc = true;
                }
                continue;
            }
            Eigen::Vector3d p_des =
                Eigen::Vector3d(pose.base.x, pose.base.y, pose.base.z) +
                R * Eigen::Vector3d(
                        commanded_body_feet_[leg].x,
                        commanded_body_feet_[leg].y,
                        commanded_body_feet_[leg].z);
            Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
            if (params_.cartesian_world && have_commanded_world_feet_)
            {
                p_des = Eigen::Vector3d(
                    commanded_world_feet_[leg].x,
                    commanded_world_feet_[leg].y,
                    commanded_world_feet_[leg].z);
                v_des = Eigen::Vector3d(
                    cartesian_state_.target_world_vel[leg].x,
                    cartesian_state_.target_world_vel[leg].y,
                    cartesian_state_.target_world_vel[leg].z);
            }
            const Eigen::Vector3d p = dyn.foot_pos_world[leg];
            wbc_in.swing_acc_world[leg] = ClampVec3(
                180.0 * (p_des - p) + 16.0 * (v_des - v),
                50.0);
        }
    }

    go2_control::IdWbcOutput wbc_out;
    go2_control::IdWbcParams id_params;
    const int n_contact =
        (qp_contact[0] ? 1 : 0) + (qp_contact[1] ? 1 : 0) +
        (qp_contact[2] ? 1 : 0) + (qp_contact[3] ? 1 : 0);
    id_params.w_stance_no_slip =
        params_.cartesian_world ? (50.0 + 90.0 * cart_lock) : 8.0;
    id_params.w_base_lin = params_.cartesian_world ? 80.0 : 80.0;
    id_params.w_base_ang = params_.cartesian_world ? (80.0 + 30.0 * cart_lock) : 40.0;
    id_params.w_swing = params_.cartesian_world ? 80.0 : 80.0;
    id_params.tau_limit_nm = 35.0;
    if (params_.cartesian_world)
    {
        id_params.hard_stance_no_slip = false;
        id_params.w_force = 1.0e-6;
        id_params.w_force_track = cart_lock * 0.008;
        if (last_srbd_.ok)
        {
            wbc_in.have_force_ref = true;
            wbc_in.force_ref = last_srbd_.first_force;
        }
    }
    bool solved =
        go2_control::SolveInverseDynamicsWbc(id_params, wbc_in, wbc_out) &&
        wbc_out.ok;
    if (!solved && id_params.hard_stance_no_slip)
    {
        id_params.hard_stance_no_slip = false;
        solved =
            go2_control::SolveInverseDynamicsWbc(id_params, wbc_in, wbc_out) &&
            wbc_out.ok;
    }
    if (solved)
    {
        last_id_wbc_ = wbc_out;
        have_last_id_wbc_ = true;
    }
    else if (have_last_id_wbc_)
    {
        wbc_out = last_id_wbc_;
    }
    else
    {
        return;
    }

    if (params_.cartesian_world && have_commanded_world_feet_ &&
        gait_started_ && motion_stage_ == 2 && cart_lock > 0.05)
    {
        Eigen::Matrix<double, 12, 1> tau_pd =
            Eigen::Matrix<double, 12, 1>::Zero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!qp_contact[leg])
                continue;
            const Eigen::Vector3d v = dyn.foot_jac_world[leg] * dyn.qvel;
            const Eigen::Vector3d dp =
                Eigen::Vector3d(
                    commanded_world_feet_[leg].x,
                    commanded_world_feet_[leg].y,
                    commanded_world_feet_[leg].z) -
                dyn.foot_pos_world[leg];
            Eigen::Vector3d f_pd = Eigen::Vector3d::Zero();
            f_pd.x() = std::clamp(
                cart_lock * (80.0 * dp.x() - 28.0 * v.x()), -18.0, 18.0);
            f_pd.y() = std::clamp(
                cart_lock * (80.0 * dp.y() - 28.0 * v.y()), -18.0, 18.0);
            tau_pd +=
                dyn.foot_jac_world[leg].rightCols<12>().transpose() * f_pd;
        }
        wbc_out.tau += tau_pd;
        for (int i = 0; i < 12; ++i)
            wbc_out.tau[i] = std::clamp(wbc_out.tau[i], -35.0, 35.0);
    }

    wbc_shadow_diagnostics_.solver_ok = true;
    wbc_shadow_diagnostics_.mapping_ok = true;
    wbc_shadow_diagnostics_.id_wbc_ok = solved;
    wbc_shadow_diagnostics_.wrench_satisfied = wbc_out.eq_residual < 1.0;
    wbc_shadow_diagnostics_.constraint_feasible = true;
    wbc_shadow_diagnostics_.task_satisfied = wbc_out.eq_residual < 1.0;
    wbc_shadow_diagnostics_.residual_norm = wbc_out.eq_residual;
    wbc_shadow_diagnostics_.id_eq_residual = wbc_out.eq_residual;
    wbc_shadow_diagnostics_.task_residual_norm = wbc_out.rne_residual;
    wbc_shadow_diagnostics_.iterations = wbc_out.iterations;
    wbc_shadow_diagnostics_.active_contacts = active;
    wbc_shadow_diagnostics_.contact_mask = contact_mask;
    wbc_shadow_diagnostics_.max_abs_tau = wbc_out.tau.cwiseAbs().maxCoeff();
    wbc_shadow_diagnostics_.desired_force_x_n =
        last_srbd_.ok ? last_srbd_.first_force[0] + last_srbd_.first_force[3] +
                            last_srbd_.first_force[6] + last_srbd_.first_force[9]
                      : 0.0;
    double min_fz = 1.0e9;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d f = wbc_out.force.segment<3>(3 * static_cast<int>(leg));
        for (int j = 0; j < 3; ++j)
        {
            const int motor = static_cast<int>(3 * leg + j);
            const int dof = rigid_body_->MotorDof(motor);
            const int joint_row = dof - 6;
            wbc_shadow_candidate_torques_[leg][j] =
                (joint_row >= 0 && joint_row < 12) ? wbc_out.tau[joint_row] : 0.0;
        }
        if (qp_contact[leg])
            min_fz = std::min(min_fz, f.z());
    }
    wbc_shadow_diagnostics_.min_contact_normal_force_n =
        std::isfinite(min_fz) ? min_fz : 0.0;
}

// --- TrotExperiment::UpdateWbcShadow ---
void TrotExperiment::UpdateWbcShadow(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state)
{
    wbc_shadow_diagnostics_ = WbcShadowDiagnostics{};
    const auto shadow_start = std::chrono::steady_clock::now();
    const auto finish_shadow_timing = [&]() {
        wbc_shadow_diagnostics_.elapsed_us =
            std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - shadow_start).count();
        wbc_shadow_diagnostics_.within_budget =
            wbc_shadow_diagnostics_.elapsed_us <= kShadowWbcBudgetUs;
    };
    if (params_.wbc_full && have_state && have_high_state)
    {
        UpdateWbcFull(state_snapshot, high_state_snapshot);
        finish_shadow_timing();
        return;
    }
    const TrueDynamics true_dyn = ExtractTrueDynamics(state_snapshot);
    if (true_dyn.valid && !dynamics_logged_)
    {
        dynamics_logged_ = true;
        std::cout << "TrueDynamics: M00=" << true_dyn.base_mass_matrix[0]
                  << " M05=" << true_dyn.base_mass_matrix[5]
                  << " bias_z=" << true_dyn.base_qfrc_bias[2]
                  << " bias_pitch=" << true_dyn.base_qfrc_bias[4]
                  << "\n";
    }
    wbc_shadow_candidate_torques_ = {};
    wbc_shadow_diagnostics_.enabled = params_.wbc_shadow;
    if (!params_.wbc_shadow || !have_state)
    {
        finish_shadow_timing();
        return;
    }

    go2_control::ProjectedContactWrenchRequest request;
    request.wrench.contact.fill(false);
    request.force_constraints.friction_coefficient =
        kShadowWbcFrictionCoefficient;
    request.force_constraints.max_normal_force =
        kShadowWbcMaxNormalForce;

    go2_control::JointAngles joint_angles{};
    int active_contacts = 0;
    int contact_mask = 0;
    const go2_control::HystereticContactParams shadow_contact_params{
        kShadowContactOnForceN, kShadowContactOffForceN};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const int motor = static_cast<int>(3 * leg);
        joint_angles[leg] = {
            static_cast<double>(state_snapshot.motor_state()[motor + 0].q()),
            static_cast<double>(state_snapshot.motor_state()[motor + 1].q()),
            static_cast<double>(state_snapshot.motor_state()[motor + 2].q())};
        request.wrench.contact_positions_body[leg] =
            go2::FootPosition(
                static_cast<go2::Leg>(leg),
                joint_angles[leg][0],
                joint_angles[leg][1],
                joint_angles[leg][2]);
        const double foot_force =
            static_cast<double>(state_snapshot.foot_force()[leg]);
        bool next_contact = false;
        if (!go2_control::UpdateHystereticContact(
                wbc_shadow_contact_state_[leg],
                foot_force,
                shadow_contact_params,
                next_contact))
        {
            finish_shadow_timing();
            return;
        }
        wbc_shadow_contact_state_[leg] = next_contact;
        request.wrench.contact[leg] = wbc_shadow_contact_state_[leg];
        if (request.wrench.contact[leg])
        {
            ++active_contacts;
            contact_mask |= 1 << static_cast<int>(leg);
        }
    }
    wbc_shadow_diagnostics_.active_contacts = active_contacts;
    const bool reduced_contact_task =
        params_.wbc_reduced_contact_task &&
        !params_.wbc_full &&
        active_contacts < go2::kLegCount;
    if (reduced_contact_task)
        request.wrench.task_weights = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
    else if (params_.wbc_full)
        request.wrench.task_weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    else if (params_.impulse)
    {
        // [wrench-fix] 冲量模式: 姿态力矩权重优先于推力,
        // 防止推力产生的前倾力矩压倒姿态任务(分配器优先保姿态)。
        request.wrench.task_weights = {0.5, 0.5, 1.0, 2.0, 2.0, 1.0};
    }
    wbc_shadow_diagnostics_.reduced_contact_task = reduced_contact_task;

    double desired_force_x_n = 0.0;
    if (params_.wbc_velocity_wrench &&
        motion_stage_ == 2 &&
        gait_started_ &&
        !stop_requested_ &&
        have_filtered_body_velocity_)
    {
        const double target_velocity_x_mps =
            params_.direction_sign * params_.step_length_m /
            params_.period_s;
        const double velocity_error_x_mps =
            target_velocity_x_mps - latest_filtered_body_velocity_[0];
        desired_force_x_n = Clamp(
            kShadowWbcMassKg * params_.wbc_velocity_gain_s_inv *
                velocity_error_x_mps,
            -params_.wbc_max_forward_force_n,
            params_.wbc_max_forward_force_n);
    }
    wbc_shadow_diagnostics_.desired_force_x_n = desired_force_x_n;
    double desired_force_y_n = 0.0;
    double desired_force_z_n = 0.0;  // [增量式] z 力交给位置伺服(避免重复补偿)
    double desired_tau_x_nm = 0.0;
    double desired_tau_y_nm = 0.0;
    double desired_tau_z_nm = 0.0;
    std::array<double, 3> desired_force{
        desired_force_x_n, desired_force_y_n, desired_force_z_n};
    std::array<double, 3> desired_torque{
        desired_tau_x_nm, desired_tau_y_nm, desired_tau_z_nm};
    const bool enhanced_wrench_active =
        params_.wbc_primary && have_high_state &&
        gait_started_ &&
        (running_time_ - gait_start_time_s_) >=
            kWbcPrimaryWrenchEnableS;
    if (enhanced_wrench_active)
    {
        // 基座高度 PD(目标 0.42 m)
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        const double height_error_m =
            kWbcPrimaryBaseHeightM - pose.base.z;
        const double base_vel_z_mps =
            static_cast<double>(high_state_snapshot.velocity()[2]);
        // [wrench-fix] z 目标 = 纯重力基底(支撑归一化)。
        // 位置环(kp=63)已扛重力+高度, wrench 再要全重力会双倍补偿;
        // 但 z=0 又让摩擦锥无法产生水平力。折中: 只给重力基底,
        // 高度修正完全交给位置环, wrench z 明确=需要的法向支撑。
        if (params_.impulse)
        {
            // [wrench-fix] 重力基底加到 desired_force[2](数组), 不是标量
            // (数组在块外拷贝, 标量改动不会反映到 wrench)。
            const double gravity_base_n =
                kShadowWbcMassKg * kShadowWbcGravityMps2 *
                (static_cast<double>(active_contacts) /
                 static_cast<double>(go2::kLegCount));
            desired_force[2] += gravity_base_n;
        }
        else
        {
            desired_force[2] +=
                kWbcPrimaryHeightKp * height_error_m -
                kWbcPrimaryHeightKd * base_vel_z_mps;
        }
        // 姿态 PD + 角速度阻尼(imu rpy 与 gyro)
        const double roll_rad =
            static_cast<double>(state_snapshot.imu_state().rpy()[0]);
        const double pitch_rad =
            static_cast<double>(state_snapshot.imu_state().rpy()[1]);
        const double gyro_x_radps =
            static_cast<double>(state_snapshot.imu_state().gyroscope()[0]);
        const double gyro_y_radps =
            static_cast<double>(state_snapshot.imu_state().gyroscope()[1]);
        const double gyro_z_radps =
            static_cast<double>(state_snapshot.imu_state().gyroscope()[2]);
        desired_tau_x_nm =
            -kWbcPrimaryRollKp * roll_rad -
            kWbcPrimaryRollKd * gyro_x_radps;
        desired_tau_y_nm =
            -kWbcPrimaryPitchKp * pitch_rad -
            kWbcPrimaryPitchKd * gyro_y_radps;
        // [Phase6] 真动力学基座任务层:wrench = M_base x a_desired + 0*bias(增量式)
        const TrueDynamics dyn = ExtractTrueDynamics(state_snapshot);
        if (dyn.valid)
        {
            std::array<double, 6> a_desired{};
            if (params_.impulse)
            {
                // 冲量主控 v1: 线动量任务(加速度域)
                // a = Kp*(v_target - v) + Kd*(-a_meas)
                const double target_vx =
                    params_.direction_sign * params_.step_length_m /
                    params_.period_s;
                const double target_vy = 0.0;
                if (have_filtered_body_velocity_)
                {
                    const double v_err_x =
                        target_vx - latest_filtered_body_velocity_[0];
                    const double v_err_y =
                        target_vy - latest_filtered_body_velocity_[1];
                    double damp_x = 0.0;
                    double damp_y = 0.0;
                    if (have_body_acceleration_)
                    {
                        damp_x = latest_body_acceleration_[0];
                        damp_y = latest_body_acceleration_[1];
                    }
                    a_desired[0] = Clamp(
                        kImpulseLinVelKpS * v_err_x -
                            kImpulseLinVelKd * damp_x,
                        -kImpulseLinAccMaxMps2, kImpulseLinAccMaxMps2);
                    // [impulse] y 向弱增益(直线保持): 高速时 y 扰动
                    // 被强增益放大导致侧偏/roll 累积, y 只需弱纠正。
                    a_desired[1] = Clamp(
                        kImpulseLinVelKpS * 0.3 * v_err_y -
                            kImpulseLinVelKd * 0.5 * damp_y,
                        -kImpulseLinAccMaxMps2 * 0.5,
                        kImpulseLinAccMaxMps2 * 0.5);
                }
            }
            else
            {
                a_desired[0] = Clamp(
                    kWbcPrimaryVelGain1S * desired_force_x_n /
                        kShadowWbcMassKg,
                    -3.0, 3.0);
                a_desired[1] = 0.0;
                if (params_.wbc_full && have_preview_terminal_velocity_)
                {
                    if (std::isfinite(preview_planned_acc_x_mps2_))
                        a_desired[0] = Clamp(
                            preview_planned_acc_x_mps2_, -3.0, 3.0);
                    else
                    {
                        double preview_acc_x = 0.0;
                        if (go2_control::PreviewTerminalAcceleration(
                                params_.direction_sign *
                                    params_.step_length_m /
                                    params_.period_s,
                                preview_terminal_velocity_x_mps_,
                                preview_n_steps_,
                                params_.period_s,
                                preview_acc_x))
                        {
                            a_desired[0] = Clamp(preview_acc_x, -3.0, 3.0);
                        }
                    }
                }
            }
            const double bounce_phase =
                2.0 * kPi * 2.0 / params_.period_s *
                (running_time_ - gait_start_time_s_);
            const double bounce_acc =
                params_.bounce_acc_amp * std::sin(bounce_phase);
            a_desired[2] = params_.impulse
                ? 0.0  // [wrench-fix] 高度完全交给位置环, wrench z=纯重力
                : Clamp(
                      kWbcPrimaryHeightAccKp * height_error_m -
                          kWbcPrimaryHeightAccKd * base_vel_z_mps +
                          bounce_acc,
                      -3.0, 3.0);
            const double attitude_acc_lim =
                params_.impulse ? 8.0 : 4.0;
            // [impulse] pitch 前倾参考 3°: 动态 trot 自然前倾,
            // 推力产生的前倾不被姿态任务强行拉回(减少对抗),
            // 但限幅防过度前倾。
            const double pitch_ref_rad =
                params_.impulse ? 3.0 * kPi / 180.0 : 0.0;
            a_desired[3] = Clamp(
                -kWbcPrimaryRollAccKp * roll_rad -
                    kWbcPrimaryRollAccKd * gyro_x_radps,
                -attitude_acc_lim, attitude_acc_lim);
            a_desired[4] = Clamp(
                kWbcPrimaryPitchAccKp * (pitch_ref_rad - pitch_rad) -
                    kWbcPrimaryPitchAccKd * gyro_y_radps,
                -attitude_acc_lim, attitude_acc_lim);
            const double turn_enable_w = Clamp(
                (running_time_ - gait_start_time_s_ - 4.0) / 2.0,
                0.0, 1.0);
            a_desired[5] = Clamp(
                kWbcPrimaryTurnYawAccKp *
                        (turn_enable_w * params_.turn_rate_radps -
                         gyro_z_radps) -
                    kWbcPrimaryYawAccKd * gyro_z_radps,
                -4.0, 4.0);
            if (params_.wbc_full)
            {
                go2_control::CentroidalMass mass;
                mass.mass_matrix = dyn.base_mass_matrix;
                mass.bias = dyn.base_qfrc_bias;
                mass.include_bias = true;
                go2_control::CentroidalTask task;
                task.desired_acc = a_desired;
                go2_control::CentroidalWrench built;
                if (go2_control::BuildCentroidalWrench(mass, task, built) &&
                    built.valid)
                {
                    desired_force = {
                        built.wrench[0], built.wrench[1], built.wrench[2]};
                    desired_torque = {
                        built.wrench[3], built.wrench[4], built.wrench[5]};
                }
            }
            else
            {
                for (int i = 0; i < 6; ++i)
                {
                    double w = 0.0;
                    for (int j = 0; j < 6; ++j)
                        w += dyn.base_mass_matrix[i * 6 + j] * a_desired[j];
                    if (i < 3)
                        desired_force[static_cast<std::size_t>(i)] += w;
                    else
                        desired_torque[static_cast<std::size_t>(i - 3)] += w;
                }
            }
        }
    }
    wbc_shadow_diagnostics_.desired_force_x_n = desired_force_x_n;
    request.wrench.desired_wrench = {
        desired_force[0], desired_force[1], desired_force[2],
        desired_torque[0], desired_torque[1], desired_torque[2]};
    wbc_shadow_diagnostics_.contact_mask = contact_mask;

    go2_control::ContactForces contact_forces{};
    if (params_.wbc_full)
    {
        go2_control::ContactWrenchQpAllocator qp_allocator;
        go2_control::ProjectedContactWrenchSolution qp_solution;
        if (!qp_allocator.Solve(request, qp_solution))
        {
            finish_shadow_timing();
            return;
        }
        contact_forces = qp_solution.forces;
        wbc_shadow_diagnostics_.solver_ok = true;
        wbc_shadow_diagnostics_.wrench_satisfied = qp_solution.wrench_satisfied;
        wbc_shadow_diagnostics_.constraint_feasible =
            qp_solution.constraint_report.feasible;
        wbc_shadow_diagnostics_.iterations = qp_solution.iterations;
        wbc_shadow_diagnostics_.residual_norm = qp_solution.residual_norm;
        wbc_shadow_diagnostics_.task_satisfied = qp_solution.task_satisfied;
        wbc_shadow_diagnostics_.task_residual_norm =
            qp_solution.task_residual_norm;
        wbc_shadow_diagnostics_.max_axis_friction_ratio =
            qp_solution.max_axis_friction_ratio;
        wbc_shadow_diagnostics_.max_radial_friction_ratio =
            qp_solution.max_radial_friction_ratio;
        wbc_shadow_diagnostics_.min_contact_normal_force_n =
            qp_solution.min_contact_normal_force;
    }
    else
    {
        go2_control::ContactWrenchProjectedAllocator allocator;
        go2_control::ProjectedContactWrenchSolution wrench_solution;
        if (!allocator.Solve(request, wrench_solution))
        {
            finish_shadow_timing();
            return;
        }
        contact_forces = wrench_solution.forces;
        wbc_shadow_diagnostics_.solver_ok = true;
        wbc_shadow_diagnostics_.wrench_satisfied =
            wrench_solution.wrench_satisfied;
        wbc_shadow_diagnostics_.constraint_feasible =
            wrench_solution.constraint_report.feasible;
        wbc_shadow_diagnostics_.iterations = wrench_solution.iterations;
        wbc_shadow_diagnostics_.residual_norm =
            wrench_solution.residual_norm;
        wbc_shadow_diagnostics_.task_satisfied =
            wrench_solution.task_satisfied;
        wbc_shadow_diagnostics_.task_residual_norm =
            wrench_solution.task_residual_norm;
        wbc_shadow_diagnostics_.max_axis_friction_ratio =
            wrench_solution.max_axis_friction_ratio;
        wbc_shadow_diagnostics_.max_radial_friction_ratio =
            wrench_solution.max_radial_friction_ratio;
        wbc_shadow_diagnostics_.min_contact_normal_force_n =
            wrench_solution.min_contact_normal_force;
    }

    go2_control::ContactTorqueMapRequest torque_request;
    torque_request.joint_angles = joint_angles;
    torque_request.contact_forces = contact_forces;
    torque_request.contact = request.wrench.contact;
    go2_control::ContactTorqueMapSolution torque_solution;
    if (!go2_control::MapContactForcesToJointTorques(
            torque_request, torque_solution))
    {
        finish_shadow_timing();
        return;
    }

    wbc_shadow_diagnostics_.mapping_ok = true;
    wbc_shadow_diagnostics_.max_abs_tau =
        torque_solution.max_abs_torque;
    wbc_shadow_candidate_torques_ = torque_solution.torques;
    finish_shadow_timing();
}

// --- TrotExperiment::PrepareWbcTorqueFeedforward ---
bool TrotExperiment::PrepareWbcTorqueFeedforward(
    std::array<double, kMotorCount> &torque_ff)
{
    torque_ff.fill(0.0);
    wbc_shadow_diagnostics_.feedforward_ready = false;
    wbc_shadow_diagnostics_.feedforward_applied = false;
    wbc_shadow_diagnostics_.feedforward_reduced_task_gate = false;
    wbc_shadow_diagnostics_.feedforward_max_abs_tau = 0.0;
    wbc_shadow_diagnostics_.feedforward_gate_code =
        static_cast<int>(go2_control::WbcFeedforwardGateCode::kDisabled);

    const bool reduced_task_gate =
        params_.wbc_task_torque_feedforward &&
        params_.wbc_reduced_contact_task &&
        wbc_shadow_diagnostics_.reduced_contact_task &&
        wbc_shadow_diagnostics_.task_satisfied;
    wbc_shadow_diagnostics_.feedforward_reduced_task_gate =
        reduced_task_gate && !wbc_shadow_diagnostics_.wrench_satisfied;

    go2_control::WbcFeedforwardGateInput gate_input;
    gate_input.requested = params_.wbc_torque_feedforward;
    gate_input.shadow_enabled =
        params_.wbc_shadow && wbc_shadow_diagnostics_.enabled;
    gate_input.locomotion_active =
        motion_stage_ == 2 && gait_started_ && !stop_requested_;
    gate_input.solver_ok = wbc_shadow_diagnostics_.solver_ok;
    gate_input.mapping_ok = wbc_shadow_diagnostics_.mapping_ok;
    gate_input.wrench_satisfied =
        wbc_shadow_diagnostics_.wrench_satisfied;
    gate_input.reduced_task_gate = reduced_task_gate;
    gate_input.constraint_feasible =
        wbc_shadow_diagnostics_.constraint_feasible;
    gate_input.active_contacts = wbc_shadow_diagnostics_.active_contacts;
    gate_input.minimum_contacts = kMinimumSupportContacts;
    gate_input.within_budget = wbc_shadow_diagnostics_.within_budget;
    gate_input.torque_scale = params_.wbc_torque_scale;
    gate_input.max_torque_scale = kWbcTorqueFeedforwardMaxScale;

    auto gate_result =
        go2_control::EvaluateWbcFeedforwardGate(gate_input);
    wbc_shadow_diagnostics_.feedforward_gate_code =
        static_cast<int>(gate_result.code);
    if (!gate_result.ready)
        return false;

    double max_abs_torque = 0.0;
    bool candidate_values_finite = true;
    bool scaled_candidate_within_limit = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        for (std::size_t joint = 0; joint < 3; ++joint)
        {
            const double candidate =
                wbc_shadow_candidate_torques_[leg][joint];
            const double scaled = params_.wbc_torque_scale * candidate;
            if (!std::isfinite(candidate))
                candidate_values_finite = false;
            if (!std::isfinite(scaled) ||
                std::abs(scaled) > kWbcTorqueFeedforwardMaxAbsNm)
                scaled_candidate_within_limit = false;
            if (!candidate_values_finite ||
                !scaled_candidate_within_limit)
                break;
            torque_ff[3 * leg + joint] = scaled;
            max_abs_torque = std::max(max_abs_torque, std::abs(scaled));
        }
        if (!candidate_values_finite ||
            !scaled_candidate_within_limit)
            break;
    }

    if (!candidate_values_finite || !scaled_candidate_within_limit)
    {
        gate_input.candidate_values_finite = candidate_values_finite;
        gate_input.scaled_candidate_within_limit =
            scaled_candidate_within_limit;
        gate_result =
            go2_control::EvaluateWbcFeedforwardGate(gate_input);
        wbc_shadow_diagnostics_.feedforward_gate_code =
            static_cast<int>(gate_result.code);
        torque_ff.fill(0.0);
        return false;
    }

    wbc_shadow_diagnostics_.feedforward_gate_code =
        static_cast<int>(go2_control::WbcFeedforwardGateCode::kReady);
    wbc_shadow_diagnostics_.feedforward_ready = true;
    wbc_shadow_diagnostics_.feedforward_max_abs_tau = max_abs_torque;
    return true;
}
