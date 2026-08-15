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
#include "contact_state_filter.h"
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "motion_frame_utils.h"
#include "preview_footstep_horizon.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

// --- TrotExperiment::UpdateWbcShadow ---
void TrotExperiment::UpdateWbcShadow(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state)
{
    wbc_shadow_diagnostics_ = WbcShadowDiagnostics{};
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
    const auto shadow_start = std::chrono::steady_clock::now();
    const auto finish_shadow_timing = [&]() {
        wbc_shadow_diagnostics_.elapsed_us =
            std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - shadow_start).count();
        wbc_shadow_diagnostics_.within_budget =
            wbc_shadow_diagnostics_.elapsed_us <= kShadowWbcBudgetUs;
    };
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
                    double preview_acc_x = 0.0;
                    if (go2_control::PreviewTerminalAcceleration(
                            params_.direction_sign * params_.step_length_m /
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
        go2_control::LexicographicContactWrenchRequest lex_request;
        lex_request.wrench = request.wrench;
        lex_request.force_constraints = request.force_constraints;
        lex_request.moment_task_active = true;
        go2_control::ContactWrenchLexicographicSlackAllocator lex_allocator;
        go2_control::LexicographicContactWrenchSolution lex_solution;
        if (!lex_allocator.Solve(lex_request, lex_solution))
        {
            finish_shadow_timing();
            return;
        }
        contact_forces = lex_solution.forces;
        wbc_shadow_diagnostics_.solver_ok = true;
        wbc_shadow_diagnostics_.wrench_satisfied = lex_solution.wrench_satisfied;
        wbc_shadow_diagnostics_.constraint_feasible =
            lex_solution.constraint_report.feasible;
        wbc_shadow_diagnostics_.iterations = lex_solution.iterations;
        wbc_shadow_diagnostics_.residual_norm = lex_solution.residual_norm;
        wbc_shadow_diagnostics_.task_satisfied = lex_solution.policy_satisfied;
        wbc_shadow_diagnostics_.task_residual_norm = lex_solution.residual_norm;
        wbc_shadow_diagnostics_.max_axis_friction_ratio =
            lex_solution.max_axis_friction_ratio;
        wbc_shadow_diagnostics_.max_radial_friction_ratio =
            lex_solution.max_radial_friction_ratio;
        wbc_shadow_diagnostics_.min_contact_normal_force_n =
            lex_solution.min_contact_normal_force;
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
