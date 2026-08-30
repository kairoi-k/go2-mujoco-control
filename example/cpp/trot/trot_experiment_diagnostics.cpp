#include "trot_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "contact_wrench_projected_allocator.h"
#include "contact_state_filter.h"
#include "full2_campaign_env.h"
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "motion_frame_utils.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

// DIAGNOSTICS — CSV, cycle quality, hard limits (see docs/CODE_GUIDE.md)

// --- TrotExperiment::WriteCsvHeader ---  [SECTION: csv-header]
void TrotExperiment::WriteCsvHeader()
{
    csv_ << "cmd_time_s,state_tick_s,has_state,motion_dt_s"
         << ",state_clock_paused,state_tick_gap_s,state_clock_pause_count"
         << ",motion_clock_wall_mode,motion_clock_wall_dt_s"
         << ",motion_clock_paused,motion_clock_pause_count"
         << ",motion_stage,cycle_index,phase"
         << ",event_active,event_priority,event_type,event_source,event_hold_stance,event_ref_vx_mps,event_ref_vy_mps,event_ref_yaw_rate_radps,event_target_vx_mps,event_target_vy_mps,event_target_yaw_rate_radps"
         << ",velocity_command_requested_mps,velocity_command_shaped_mps,velocity_command_applied_mps"
         << ",velocity_command_measured_mps,velocity_command_tracking_error_mps"
         << ",velocity_command_accel_mps2,velocity_command_jerk_mps3"
         << ",velocity_command_active,velocity_command_gait_period_s"
         << ",velocity_command_gait_duty,velocity_command_gait_step_length_m"
         << ",velocity_command_gait_foot_lift_m,velocity_command_gait_regime"
         << ",environment_map_valid,environment_map_age_s"
         << ",obstacle_center_distance_m,obstacle_left_distance_m,obstacle_right_distance_m"
         << ",obstacle_center_height_m,obstacle_left_height_m,obstacle_right_height_m"
         << ",terrain_enabled,terrain_sensor_only,terrain_actuation"
         << ",terrain_map_valid,terrain_map_source,terrain_map_epoch"
         << ",terrain_map_age_s,terrain_known_cells,terrain_feasible_regions"
         << ",terrain_plan_status,terrain_plan_id,terrain_plan_epoch,terrain_plan_valid"
         << ",terrain_planner_updates,terrain_planner_rejections"
         << ",terrain_planner_deadline_misses,terrain_solver_elapsed_us"
         << ",terrain_safe_stop_requested,terrain_velocity_cap_mps"
         << ",terrain_plan_published,terrain_plan_consumed"
         << ",terrain_gait_target_overrides,terrain_mpc_plan_consumed"
         << ",terrain_plan_failure,terrain_committed_touchdowns"
         << ",terrain_dominant_foothold_reject"
         << ",terrain_failed_leg,terrain_failed_leg_reject"
         << ",terrain_support_failure_knot"
         << ",terrain_support_failure_contact_mask"
         << ",terrain_support_failure_margin_m";
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const char *name = kLegNames[leg];
        csv_ << ",terrain_" << name << "_candidate_count"
             << ",terrain_" << name << "_swing_candidate_count"
             << ",terrain_" << name << "_candidate_required"
             << ",terrain_" << name << "_touchdown_knot";
    }
    csv_
         << ",terrain_min_edge_margin_m,terrain_min_uncertainty_edge_margin_m"
         << ",terrain_min_slope_rad,terrain_max_roughness_m,terrain_min_reachability_margin_m,terrain_min_swing_clearance_m"
         << ",terrain_min_support_margin_m,terrain_min_uncertainty_support_margin_m"
         << ",terrain_plan_contact_rejections"
         << ",terrain_execution_plan_id,terrain_execution_plan_epoch"
         << ",terrain_execution_map_epoch,terrain_execution_plan_generated_at_s"
         << ",terrain_execution_plan_valid_until_s,terrain_execution_plan_age_s"
         << ",terrain_execution_plan_usable,terrain_execution_planned_contact_mask"
         << ",terrain_transfer_hold_active,terrain_transfer_hold_mask"
         << ",terrain_transfer_window_active,terrain_transfer_window_release_s"
         << ",terrain_crawl_state,terrain_crawl_active_leg"
         << ",terrain_event_sequencer_state,terrain_event_sequencer_active_leg"
         << ",terrain_event_sequencer_control_authority"
         << ",terrain_event_sequencer_authority_trot_full_contact_able"
         << ",terrain_event_sequencer_authority_contacts_ready"
         << ",terrain_event_sequencer_authority_velocity_ready"
         << ",terrain_event_sequencer_authority_posture_ready"
         << ",terrain_event_sequencer_authority_stand_transition_seen"
         << ",terrain_event_sequencer_authority_block_reason"
         << ",terrain_event_sequencer_authority_velocity_mps"
         << ",terrain_event_sequencer_authority_roll_rad"
         << ",terrain_event_sequencer_authority_pitch_rad"
         << ",terrain_event_sequencer_stage_abort_reason"
         << ",terrain_event_sequencer_stand_transition_requested"
         << ",terrain_event_sequencer_com_reference_valid"
         << ",terrain_event_sequencer_com_reference_x_m"
         << ",terrain_event_sequencer_com_reference_y_m"
         << ",terrain_event_sequencer_com_reference_z_m"
         << ",terrain_event_sequencer_measured_contacts"
         << ",terrain_event_sequencer_flat_ground_mode"
         << ",terrain_event_sequencer_committed_mask"
         << ",terrain_event_sequencer_contact_schedule_mask"
         << ",terrain_event_sequencer_swing_target_x_m"
         << ",terrain_event_sequencer_swing_target_y_m"
         << ",terrain_event_sequencer_swing_target_z_m"
         << ",terrain_event_sequencer_target_world_x_m"
         << ",terrain_event_sequencer_target_world_y_m"
         << ",terrain_event_sequencer_target_world_z_m"
         << ",terrain_event_sequencer_swing_start_world_x_m"
         << ",terrain_event_sequencer_swing_start_world_y_m"
         << ",terrain_event_sequencer_swing_start_world_z_m"
         << ",terrain_staging_target_valid,terrain_staging_error_m"
         << ",terrain_staging_target_world_x_m"
         << ",terrain_crawl_retry_count,terrain_crawl_state_enter_s"
         << ",terrain_crawl_shift_duration_s,terrain_crawl_shift_recovery_count"
         << ",terrain_crawl_min_contacts,terrain_crawl_step_commits"
         << ",terrain_crawl_com_x_m,terrain_crawl_com_y_m"
         << ",terrain_crawl_com_margin_m,terrain_crawl_com_target_x_m"
         << ",terrain_crawl_com_target_y_m,terrain_crawl_com_target_valid"
         << ",terrain_crawl_shift_support_lifted_leg"
         << ",terrain_crawl_shift_triangle_v0_x_m,terrain_crawl_shift_triangle_v0_y_m,terrain_crawl_shift_triangle_v0_z_m"
         << ",terrain_crawl_shift_triangle_v1_x_m,terrain_crawl_shift_triangle_v1_y_m,terrain_crawl_shift_triangle_v1_z_m"
         << ",terrain_crawl_shift_triangle_v2_x_m,terrain_crawl_shift_triangle_v2_y_m,terrain_crawl_shift_triangle_v2_z_m"
         << ",terrain_crawl_stage_margin_kind"
         << ",terrain_crawl_stage_basin_margin_m"
         << ",terrain_crawl_stage_target_margin_m"
         << ",terrain_crawl_stage_target_x_m"
         << ",terrain_crawl_stage_target_y_m"
         << ",terrain_crawl_stage_target_valid"
         << ",terrain_crawl_stage_retry_count"
         << ",terrain_crawl_stage_probe_active"
         << ",terrain_crawl_stage_probe_direction"
         << ",terrain_crawl_stage_servo_acc_x_mps2"
         << ",terrain_crawl_stage_servo_acc_y_mps2"
         << ",terrain_crawl_shift_servo_acc_x_mps2"
         << ",terrain_crawl_shift_servo_acc_y_mps2"
         << ",terrain_crawl_stage_servo_saturated"
         << ",terrain_stance_reference_valid"
         << ",terrain_stance_reference_roll_rad"
         << ",terrain_stance_reference_pitch_rad"
         << ",terrain_surface_transition_active"
         << ",terrain_surface_transition_required_mask"
         << ",terrain_surface_transition_original_required_mask"
         << ",terrain_surface_transition_cancelled_mask"
         << ",terrain_surface_transition_committed_mask"
         << ",terrain_surface_transition_completions"
         << ",terrain_surface_transition_last_required_mask"
         << ",terrain_surface_transition_last_committed_mask"
         << ",terrain_target_prepare_attempts,terrain_target_prepared"
         << ",terrain_target_prepare_rejections,terrain_target_last_prepare_failure";
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const char *name = kLegNames[leg];
        csv_ << ",terrain_exec_" << name << "_valid"
             << ",terrain_exec_" << name << "_in_flight"
             << ",terrain_exec_" << name << "_endpoint_held"
             << ",terrain_exec_" << name << "_measured_touchdown"
             << ",terrain_exec_" << name << "_wbc_endpoint_error_m"
             << ",terrain_exec_" << name << "_wbc_at_endpoint"
             << ",terrain_exec_" << name << "_wbc_measured_contact"
             << ",terrain_exec_" << name << "_target_required"
             << ",terrain_exec_" << name << "_height_change"
             << ",terrain_exec_" << name << "_time_rebased"
             << ",terrain_exec_" << name << "_plan_id"
             << ",terrain_exec_" << name << "_map_epoch"
             << ",terrain_exec_" << name << "_swing_start_time_s"
             << ",terrain_exec_" << name << "_trajectory_start_time_s"
             << ",terrain_exec_" << name << "_nominal_touchdown_time_s"
             << ",terrain_exec_" << name << "_touchdown_time_s"
             << ",terrain_exec_" << name << "_swing_duration_s"
             << ",terrain_exec_" << name << "_terrain_swing_duration_s"
             << ",terrain_exec_" << name << "_planned_swing_duration_s"
             << ",terrain_exec_" << name << "_window_margin_s"
             << ",terrain_exec_" << name << "_phase"
             << ",terrain_exec_" << name << "_start_world_x_m"
             << ",terrain_exec_" << name << "_start_world_y_m"
             << ",terrain_exec_" << name << "_start_world_z_m"
             << ",terrain_exec_" << name << "_target_world_x_m"
             << ",terrain_exec_" << name << "_target_world_y_m"
             << ",terrain_exec_" << name << "_target_world_z_m"
             << ",terrain_exec_" << name << "_foot_world_x_m"
             << ",terrain_exec_" << name << "_foot_world_y_m"
             << ",terrain_exec_" << name << "_foot_world_z_m"
             << ",terrain_pending_" << name << "_valid"
             << ",terrain_pending_" << name << "_plan_id"
             << ",terrain_pending_" << name << "_touchdown_time_s"
             << ",terrain_pending_" << name << "_target_world_x_m"
             << ",terrain_pending_" << name << "_target_world_y_m"
             << ",terrain_pending_" << name << "_target_world_z_m"
             << ",terrain_" << name << "_last_prepare_failure";
    }
    csv_
         << ",support_foot_kinematics_valid,support_foot_count,support_foot_speed_mps"
         << ",support_low_friction_evidence"
         << ",low_friction_accumulation"
         << ",world_base_x_m,world_base_y_m,world_base_z_m,world_yaw_error_rad"
         << ",world_feedback_x_m,world_feedback_y_m"
         << ",body_velocity_x_mps,body_velocity_y_mps,body_velocity_z_mps"
         << ",world_velocity_x_mps,world_velocity_y_mps,world_velocity_z_mps"
         << ",raw_body_velocity_x_mps,raw_body_velocity_y_mps,raw_body_velocity_z_mps"
         << ",imu_accel_x_mps2,imu_accel_y_mps2,imu_accel_z_mps2"
         << ",imu_gyro_x_radps,imu_gyro_y_radps,imu_gyro_z_radps"
         << ",imu_gyro_body_x_radps,imu_gyro_body_y_radps,imu_gyro_body_z_radps"
         << ",imu_roll_rad,imu_pitch_rad,imu_yaw_rad"
         << ",velocity_filter_alpha,velocity_filter_initialized"
         << ",kernel_footstep_plan_valid,kernel_velocity_error_x_mps"
         << ",kernel_touchdown_target_fr_x_m,kernel_touchdown_target_fl_x_m"
         << ",kernel_touchdown_target_rr_x_m,kernel_touchdown_target_rl_x_m"
         << ",attitude_feedback_x_m,attitude_feedback_y_m"
         << ",contact_count"
         << ",foot_force_FR,foot_force_FL,foot_force_RR,foot_force_RL"
         << ",contact_FR,contact_FL,contact_RR,contact_RL"
         << ",touchdown_event_count,touchdown_leg,touchdown_command_x_m"
         << ",touchdown_actual_x_m,touchdown_x_error_m,touchdown_y_error_m"
         << ",wbc_shadow_enabled,wbc_shadow_desired_force_x_n"
         << ",wbc_shadow_reduced_contact_task,wbc_shadow_task_satisfied"
         << ",wbc_shadow_task_residual_norm"
         << ",wbc_shadow_solver_ok,wbc_shadow_mapping_ok"
         << ",wbc_shadow_active_contacts,wbc_shadow_contact_mask"
         << ",wbc_shadow_iterations"
         << ",wbc_shadow_wrench_satisfied,wbc_shadow_constraint_feasible"
         << ",wbc_shadow_max_axis_friction_ratio"
         << ",wbc_shadow_max_radial_friction_ratio"
         << ",wbc_shadow_min_contact_normal_force_n"
         << ",wbc_shadow_id_wbc_normal_force_fr_n"
         << ",wbc_shadow_id_wbc_normal_force_fl_n"
         << ",wbc_shadow_id_wbc_normal_force_rr_n"
         << ",wbc_shadow_id_wbc_normal_force_rl_n"
         << ",terrain_hold_force_telemetry";
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        csv_ << ",terrain_hold_" << kLegNames[leg]
             << "_raw_normal_force_n"
             << ",terrain_hold_" << kLegNames[leg]
             << "_wbc_normal_force_n";
    csv_
         << ",terrain_hold_cost_base_linear"
         << ",terrain_hold_cost_base_angular"
         << ",terrain_hold_cost_stance_no_slip"
         << ",terrain_hold_cost_swing"
         << ",terrain_hold_cost_force_regularization"
         << ",terrain_hold_cost_force_tracking"
         << ",terrain_hold_cost_posture"
         << ",terrain_hold_cost_torque"
         << ",wbc_shadow_residual_norm,wbc_shadow_max_abs_tau"
         << ",wbc_shadow_elapsed_us,wbc_shadow_within_budget"
         << ",wbc_shadow_feedforward_ready,wbc_shadow_feedforward_applied"
         << ",wbc_shadow_feedforward_reduced_task_gate"
         << ",wbc_shadow_feedforward_gate_code"
         << ",wbc_shadow_feedforward_gate_reason"
         << ",wbc_shadow_feedforward_max_abs_tau"
         << ",wbc_full_srbd_ok,wbc_full_id_ok,wbc_full_eq_residual"
         << ",wbc_full_velocity_target_x_mps,wbc_full_requested_acc_x_mps2"
         << ",wbc_full_srbd_acc_x_mps2,wbc_full_id_qdd_x_mps2"
         << ",wbc_full_id_contact_force_x_n"
         << ",wbc_measured_contact_mask,wbc_scheduled_contact_mask"
         << ",wbc_terrain_planned_contact_mask,wbc_mpc_update_count"
         << ",wbc_mpc_contact_mask_k0"
         << ",wbc_mpc_min_contact_count,wbc_mpc_reference_x_first_m"
         << ",wbc_mpc_reference_x_last_m,wbc_mpc_reference_vx_first_mps"
         << ",wbc_mpc_reference_vx_last_mps,wbc_terrain_contact_coherent"
         << ",wbc_terrain_plan_id";
    for (int i = 0; i < kMotorCount; ++i)
    {
        csv_ << "," << kMotorNames[i] << "_q_target"
             << "," << kMotorNames[i] << "_dq_target"
             << "," << kMotorNames[i] << "_kp"
             << "," << kMotorNames[i] << "_kd"
             << "," << kMotorNames[i] << "_tau_ff"
             << "," << kMotorNames[i] << "_q_state"
             << "," << kMotorNames[i] << "_dq_state"
             << "," << kMotorNames[i] << "_tau_est"
             << "," << kMotorNames[i] << "_q_error";
    }
    csv_ << "\n";
}

// --- TrotExperiment::ResetCycleDiagnostics ---
void TrotExperiment::ResetCycleDiagnostics()
{
    cycle_diagnostics_ = CycleDiagnostics{};
    cycle_vx_sum_ = 0.0;
    cycle_vx_count_ = 0;
}

// [真动力学] 从 lowstate 空槽位(motor 12-17)解析基座质量矩阵与 qfrc_bias

// --- TrotExperiment::UpdateCycleDiagnostics ---  [SECTION: update-cycle-diagnostics]
void TrotExperiment::UpdateCycleDiagnostics(
    double phase,
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const std::array<double, kMotorCount> &joint_targets,
    bool have_world_feet,
    const std::array<go2::Vec3, go2::kLegCount> &world_feet,
    const WorldPose &world_pose)
{
    if (!have_state)
        return;

    const double roll = state_snapshot.imu_state().rpy()[0];
    const double pitch = state_snapshot.imu_state().rpy()[1];
    cycle_diagnostics_.max_abs_roll_rad = std::max(
        cycle_diagnostics_.max_abs_roll_rad, std::abs(roll));
    cycle_diagnostics_.max_abs_pitch_rad = std::max(
        cycle_diagnostics_.max_abs_pitch_rad, std::abs(pitch));
    if (have_filtered_body_velocity_)
    {
        cycle_vx_sum_ += latest_filtered_body_velocity_[0];
        ++cycle_vx_count_;
    }

    int support_contacts = 0;
    const double diagnostic_duty =
        kernel_duty_factor_ > 0.05
            ? kernel_duty_factor_
            : params_.duty_factor;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const double leg_phase = go2_control::GaitLegPhase(
            leg, phase, runtime_gait_pattern_);
        const bool swing = leg_phase >= diagnostic_duty;
        const double force = state_snapshot.foot_force()[leg];
        const bool contact = force >= kContactForceThreshold;
        if (!have_leg_phase_history_)
        {
            previous_leg_swing_[leg] = swing;
            touchdown_recorded_[leg] = !swing;
            touchdown_waiting_contact_[leg] = false;
        }
        else
        {
            const bool entering_swing =
                swing && !previous_leg_swing_[leg];
            const bool entering_stance =
                !swing && previous_leg_swing_[leg];
            if (entering_swing)
            {
                touchdown_recorded_[leg] = false;
                touchdown_waiting_contact_[leg] = false;
            }
            if (entering_stance && !touchdown_recorded_[leg])
            {
                touchdown_waiting_contact_[leg] = true;
            }

            // A force that persists while the leg enters swing is not a
            // touchdown. Count only after the leg enters stance, allowing
            // delayed contact sensing to complete the event.
            const bool contact_event =
                contact &&
                !swing &&
                !touchdown_recorded_[leg] &&
                (entering_stance || touchdown_waiting_contact_[leg]);
            if (contact_event &&
                have_world_feet &&
                have_commanded_body_feet_)
            {
                const std::array<double, 4> inverse_quaternion = {
                    world_pose.quaternion[0],
                    -world_pose.quaternion[1],
                    -world_pose.quaternion[2],
                    -world_pose.quaternion[3]};
                const go2::Vec3 delta = {
                    world_feet[leg].x - world_pose.base.x,
                    world_feet[leg].y - world_pose.base.y,
                    world_feet[leg].z - world_pose.base.z};
                const go2::Vec3 actual_body =
                    RotateByQuaternion(inverse_quaternion, delta);
                const double x_error =
                    actual_body.x - commanded_body_feet_[leg].x;
                const double y_error =
                    actual_body.y - commanded_body_feet_[leg].y;
                cycle_diagnostics_.touchdown_events++;
                cycle_diagnostics_.max_abs_touchdown_x_error_m =
                    std::max(
                        cycle_diagnostics_.max_abs_touchdown_x_error_m,
                        std::abs(x_error));
                cycle_diagnostics_.max_abs_touchdown_y_error_m =
                    std::max(
                        cycle_diagnostics_.max_abs_touchdown_y_error_m,
                        std::abs(y_error));
                touchdown_event_count_++;
                last_touchdown_leg_ = static_cast<int>(leg);
                last_touchdown_command_x_m_ =
                    commanded_body_feet_[leg].x;
                last_touchdown_actual_x_m_ = actual_body.x;
                last_touchdown_x_error_m_ = x_error;
                last_touchdown_y_error_m_ = y_error;
                touchdown_recorded_[leg] = true;
                touchdown_waiting_contact_[leg] = false;
            }
            previous_leg_swing_[leg] = swing;
        }
        if (swing)
        {
            // A new stance must get a new world reference. Keeping the
            // previous reference through swing would count the intentional
            // foot transfer as support drift after touchdown.
            cycle_diagnostics_.support_reference_valid[leg] = false;
        }
        else
        {
            if (force >= kContactForceThreshold)
            {
                ++support_contacts;
                if (have_world_feet &&
                    !cycle_diagnostics_.support_reference_valid[leg])
                {
                    cycle_diagnostics_.support_reference_world_feet[leg] =
                        world_feet[leg];
                    cycle_diagnostics_.support_reference_valid[leg] = true;
                }
            }
            if (have_world_feet &&
                cycle_diagnostics_.support_reference_valid[leg])
            {
                const double dx =
                    world_feet[leg].x -
                    cycle_diagnostics_.support_reference_world_feet[leg].x;
                const double dy =
                    world_feet[leg].y -
                    cycle_diagnostics_.support_reference_world_feet[leg].y;
                cycle_diagnostics_.max_support_drift_m = std::max(
                    cycle_diagnostics_.max_support_drift_m,
                    std::hypot(dx, dy));
            }
        }
    }

    have_leg_phase_history_ = true;
    cycle_diagnostics_.min_support_contacts = std::min(
        cycle_diagnostics_.min_support_contacts, support_contacts);
    ++cycle_diagnostics_.support_contact_samples;
    if (support_contacts >= kMinimumSupportContacts)
    {
        ++cycle_diagnostics_.support_contact_good_samples;
        cycle_diagnostics_.consecutive_low_support_samples = 0;
    }
    else
    {
        ++cycle_diagnostics_.consecutive_low_support_samples;
        cycle_diagnostics_.max_consecutive_low_support_samples = std::max(
            cycle_diagnostics_.max_consecutive_low_support_samples,
            cycle_diagnostics_.consecutive_low_support_samples);
    }

    double sample_max_abs_tau_est = 0.0;
    for (int i = 0; i < kMotorCount; ++i)
    {
        cycle_diagnostics_.max_abs_joint_error_rad = std::max(
            cycle_diagnostics_.max_abs_joint_error_rad,
            std::abs(
                joint_targets[i] -
                static_cast<double>(state_snapshot.motor_state()[i].q())));
        if (have_commanded_body_feet_)
        {
            const int leg_i = i / 3;
            const int base = 3 * leg_i;
            const go2::Vec3 actual = go2::FootPosition(
                static_cast<go2::Leg>(leg_i),
                static_cast<double>(state_snapshot.motor_state()[base + 0].q()),
                static_cast<double>(state_snapshot.motor_state()[base + 1].q()),
                static_cast<double>(state_snapshot.motor_state()[base + 2].q()));
            const go2::Vec3 target =
                commanded_body_feet_[static_cast<std::size_t>(leg_i)];
            const double dx = target.x - actual.x;
            const double dy = target.y - actual.y;
            const double dz = target.z - actual.z;
            cycle_diagnostics_.max_foot_error_m = std::max(
                cycle_diagnostics_.max_foot_error_m,
                std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        const double abs_tau_est = std::abs(
            static_cast<double>(state_snapshot.motor_state()[i].tau_est()));
        sample_max_abs_tau_est = std::max(sample_max_abs_tau_est, abs_tau_est);
        if (abs_tau_est > cycle_diagnostics_.max_abs_tau_est)
        {
            cycle_diagnostics_.max_abs_tau_est = abs_tau_est;
            cycle_diagnostics_.max_tau_motor_index = i;
        }
    }
    if (sample_max_abs_tau_est > params_.tau_limit_nm)
    {
        ++cycle_diagnostics_.tau_over_limit_samples;
        ++cycle_diagnostics_.consecutive_tau_over_limit_samples;
        cycle_diagnostics_.max_consecutive_tau_over_limit_samples =
            std::max(
                cycle_diagnostics_.max_consecutive_tau_over_limit_samples,
                cycle_diagnostics_.consecutive_tau_over_limit_samples);
    }
    else
    {
        cycle_diagnostics_.consecutive_tau_over_limit_samples = 0;
    }
}

// --- TrotExperiment::ValidateCycle ---  [SECTION: validate-cycle]
bool TrotExperiment::ValidateCycle(int cycle_index)
{
    const double roll_deg =
        cycle_diagnostics_.max_abs_roll_rad * 180.0 / kPi;
    const double pitch_deg =
        cycle_diagnostics_.max_abs_pitch_rad * 180.0 / kPi;
    const double drift_mm =
        cycle_diagnostics_.max_support_drift_m * 1000.0;
    const double support_fraction =
        cycle_diagnostics_.support_contact_samples == 0
            ? 0.0
            : static_cast<double>(
                  cycle_diagnostics_.support_contact_good_samples) /
                  cycle_diagnostics_.support_contact_samples;

    // 起步豁免: 前 8 个 cycle 是加速期, tau 峰值需求高(大步长起步
    // 冲击可达 23.5Nm), 不执行 tau 质量门, 避免误杀稳态。硬限仍保护。
    const bool startup_tau_exempt =
        cycle_index <= (params_.wbc_full ? 60 : 8);
    const double tau_burst_nm =
        params_.wbc_full ? 48.0 : kSafetyMaxTauBurstEst;
    const int tau_over_limit =
        params_.wbc_full ? 80 : kSafetyMaxTauOverLimitSamples;
    const int tau_over_consecutive =
        params_.wbc_full ? 30 : kSafetyMaxConsecutiveTauOverLimitSamples;
    const bool tau_quality_ok =
        startup_tau_exempt ||
        cycle_diagnostics_.max_abs_tau_est <= params_.tau_limit_nm ||
        (cycle_diagnostics_.max_abs_tau_est <= tau_burst_nm &&
         cycle_diagnostics_.tau_over_limit_samples <= tau_over_limit &&
         cycle_diagnostics_.max_consecutive_tau_over_limit_samples <=
             tau_over_consecutive);

    std::cout << "Trot cycle " << cycle_index
              << " health: roll=" << roll_deg
              << " deg, pitch=" << pitch_deg
              << " deg, support_drift=" << drift_mm
              << " mm, q_error="
              << cycle_diagnostics_.max_abs_joint_error_rad
              << " rad, foot_error="
              << cycle_diagnostics_.max_foot_error_m
              << " m, tau_est="
              << cycle_diagnostics_.max_abs_tau_est
              << ", tau_motor="
              << (cycle_diagnostics_.max_tau_motor_index >= 0
                      ? kMotorNames[cycle_diagnostics_.max_tau_motor_index]
                      : "none")
              << ", tau_over_samples="
              << cycle_diagnostics_.tau_over_limit_samples
              << ", tau_over_max_consecutive="
              << cycle_diagnostics_.max_consecutive_tau_over_limit_samples
              << ", min_support_contacts="
              << cycle_diagnostics_.min_support_contacts
              << ", max_low_support_samples="
              << cycle_diagnostics_.max_consecutive_low_support_samples
              << ", support_contact_fraction="
              << support_fraction
              << ", touchdown_events="
              << cycle_diagnostics_.touchdown_events
              << ", touchdown_max_abs_x_error_m="
              << cycle_diagnostics_.max_abs_touchdown_x_error_m
              << ", touchdown_max_abs_y_error_m="
              << cycle_diagnostics_.max_abs_touchdown_y_error_m << "\n";

    // 冲量模式(dynamic trot): 允许更大的腾空/对角支撑相,
    // 放宽支撑分数与低支撑容忍(动态步态天然有腾空)。
    const double effective_duty =
        kernel_duty_factor_ > 0.05
            ? kernel_duty_factor_
            : params_.duty_factor;
    const double min_support_fraction =
        params_.cartesian_world ? 0.28
        : (params_.wbc_full
               ? (effective_duty < 0.48 ? 0.25 : 0.35)
               : (params_.impulse ? 0.78 : kSafetyMinSupportFraction));
    const int max_consecutive_low_support =
        params_.wbc_full ? 250
                         : (params_.impulse ? 40 : kSafetyMaxConsecutiveLowSupport);
    // Position-control q_error 0.28. ID-WBC stance tracks tau*, not IK.
    const double max_joint_error_rad =
        params_.cartesian_world ? 1.15
        : (params_.wbc_full ? 0.80 : 0.28);
    const double max_roll_rad =
        params_.wbc_full ? (16.0 * kPi / 180.0) : kSafetyMaxRollRad;
    const double max_pitch_rad =
        params_.wbc_full ? (16.0 * kPi / 180.0) : kSafetyMaxPitchRad;
    const bool safe =
        cycle_diagnostics_.support_contact_samples > 0 &&
        cycle_diagnostics_.max_abs_roll_rad <= max_roll_rad &&
        cycle_diagnostics_.max_abs_pitch_rad <= max_pitch_rad &&
        cycle_diagnostics_.max_abs_joint_error_rad <=
            max_joint_error_rad &&
        tau_quality_ok &&
        support_fraction >= min_support_fraction &&
        cycle_diagnostics_.max_consecutive_low_support_samples <=
            max_consecutive_low_support;
    // V2 crawl deliberately violates the running-trot support fraction and
    // cycle timing envelope while the declared transfer window is active.
    // Keep instantaneous hard posture limits below, but do not let this
    // trot-tuned cycle-quality verdict abort the terrain transaction.
    if (!safe && terrain_transfer_window_active_)
    {
        std::cerr << "Trot crawl cycle quality bypassed during transfer window "
                  << cycle_index << "\n";
        return true;
    }
    const bool high_speed_health_governor =
        params_.wbc_full && !params_.cartesian_world &&
        Full2EnvDouble("TROT_HS_STABILITY_GOV", 0.0) > 0.5;
    if (!safe && high_speed_health_governor)
    {
        std::cerr << "Trot health governor: degraded cycle "
                  << cycle_index << "; speed cap remains active\n";
        return true;
    }
    if (!safe && Full2EnvDouble("TROT_EXPLORATORY_CONTINUE", 0.0) > 0.5)
    {
        std::cerr << "Trot exploratory continuation: cycle quality rejected "
                  << cycle_index << " (hard limits remain active)\n";
        return true;
    }
    if (!safe)
    {
        std::cerr << "Trot cycle quality guard rejected cycle "
                  << cycle_index << "\n";
        if (!tau_quality_ok)
        {
            std::cerr << "  tau quality: max="
                      << cycle_diagnostics_.max_abs_tau_est
                      << ", over_samples="
                      << cycle_diagnostics_.tau_over_limit_samples
                      << ", max_consecutive="
                      << cycle_diagnostics_.max_consecutive_tau_over_limit_samples
                      << "\n";
        }
    }
    return safe;
}

// --- TrotExperiment::CheckInstantaneousHardLimits ---  [SECTION: hard-limits]
bool TrotExperiment::CheckInstantaneousHardLimits(
    const std::array<double, kMotorCount> &joint_targets,
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    bool primary_active) const
{
    if (!have_state)
        return true;
    const double roll = state_snapshot.imu_state().rpy()[0];
    const double pitch = state_snapshot.imu_state().rpy()[1];
    const bool crawl_stance_reference = terrain_transfer_window_active_ &&
        terrain_stance_reference_valid_ &&
        (terrain_crawl_state_machine_.state() ==
             go2_terrain::TerrainCrawlState::kShiftCom ||
         terrain_crawl_state_machine_.state() ==
             go2_terrain::TerrainCrawlState::kCrawlStep);
    const double roll_error = crawl_stance_reference
        ? roll - terrain_stance_reference_roll_rad_ : roll;
    const double pitch_error = crawl_stance_reference
        ? pitch - terrain_stance_reference_pitch_rad_ : pitch;
    // Crawl deliberately tilts with its measured support plane. Only the
    // deviation from that deliberate reference is a posture hard stop.
    const double hard_roll = crawl_stance_reference
        ? go2_terrain::TerrainCrawlStateMachine::kStancePostureDeviationLimitRad
        : (params_.wbc_full ? (22.0 * kPi / 180.0) : kHardMaxRollRad);
    const double hard_pitch = crawl_stance_reference
        ? go2_terrain::TerrainCrawlStateMachine::kStancePostureDeviationLimitRad
        : (params_.wbc_full ? (22.0 * kPi / 180.0) : kHardMaxPitchRad);
    if (std::abs(roll_error) > hard_roll ||
        std::abs(pitch_error) > hard_pitch)
    {
        std::cerr << "Trot hard posture limit: roll="
                  << roll * 180.0 / kPi << " deg, pitch="
                  << pitch * 180.0 / kPi << " deg, reference_roll="
                  << (crawl_stance_reference
                          ? terrain_stance_reference_roll_rad_ : 0.0)
                  << " rad, reference_pitch="
                  << (crawl_stance_reference
                          ? terrain_stance_reference_pitch_rad_ : 0.0)
                  << " rad, deviation_limit=" << hard_roll << " rad" << std::endl;
        return false;
    }
    // WBC 主控激活时:q 目标=实际位置且命令扭矩已限幅,
    // q_error/tau_est 门只用于回退(位置控制)状态。
    // --wbc-full ID-WBC 的 tau* 可达 35 N·m; 瞬时 tau_est 尖峰不是摔倒。
    if (primary_active || params_.wbc_full)
        return true;
    for (int i = 0; i < kMotorCount; ++i)
    {
        const double q_error =
            joint_targets[i] -
            static_cast<double>(state_snapshot.motor_state()[i].q());
        if (std::abs(q_error) > kHardMaxJointErrorRad ||
            std::abs(
                static_cast<double>(
                    state_snapshot.motor_state()[i].tau_est())) >
                kHardMaxTauEst)
        {
            std::cerr << "Trot hard joint limit: motor=" << i
                      << ", q_error=" << q_error
                      << ", tau_est="
                      << state_snapshot.motor_state()[i].tau_est()
                      << "\n";
            return false;
        }
    }
    return true;
}

// --- TrotExperiment::LogSample ---  [SECTION: log-sample]
void TrotExperiment::LogSample(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state)
{
    const go2_control::Vector3 world_velocity =
        have_world_velocity_ ? latest_world_velocity_ : go2_control::Vector3{};
    const go2_control::Vector3 raw_body_velocity =
        have_raw_body_velocity_ ? latest_raw_body_velocity_ : go2_control::Vector3{};
    const go2_control::Vector3 body_velocity =
        have_filtered_body_velocity_ ? latest_filtered_body_velocity_ : go2_control::Vector3{};
    const bool have_body_velocity = have_filtered_body_velocity_;
    const double state_tick_s =
        have_state ? static_cast<double>(state_snapshot.tick()) * 0.001 : 0.0;
    const WorldPose pose = have_world_reference_ && have_high_state
        ? ComputeWorldPose(state_snapshot, high_state_snapshot)
        : WorldPose{};
    std::array<double, 3> imu_acceleration{};
    std::array<double, 3> imu_gyro{};
    std::array<double, 3> imu_gyro_body{};
    std::array<double, 3> imu_rpy{};
    if (have_state)
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            imu_acceleration[axis] =
                static_cast<double>(
                    state_snapshot.imu_state().accelerometer()[axis]);
            imu_gyro[axis] = static_cast<double>(
                state_snapshot.imu_state().gyroscope()[axis]);
            imu_rpy[axis] = static_cast<double>(
                state_snapshot.imu_state().rpy()[axis]);
        }
    }

    imu_gyro_body = ConvertImuGyroToBody(imu_gyro);
    int contact_count = 0;
    const bool terrain_force_telemetry =
        Full2EnvDouble("TROT_TERRAIN_DEBUG_FORCE", 0.0) > 0.5 &&
        (terrain_transfer_hold_active_ || terrain_surface_transition_active_);
    std::array<double, go2::kLegCount> foot_forces{};
    std::array<int, go2::kLegCount> contact_flags{};
    if (have_state)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            foot_forces[leg] =
                static_cast<double>(state_snapshot.foot_force()[leg]);
            contact_flags[leg] =
                foot_forces[leg] >= kContactForceThreshold ? 1 : 0;
            contact_count += contact_flags[leg];
        }
    }
    double terrain_last_failure = 0.0;
    std::string terrain_dominant_foothold_reject = "none";
    double terrain_min_edge_margin_m = 0.0;
    double terrain_min_uncertainty_edge_margin_m = 0.0;
    double terrain_min_slope_rad = 0.0;
    double terrain_max_roughness_m = 0.0;
    double terrain_min_reachability_margin_m = 0.0;
    double terrain_min_swing_clearance_m = 0.0;
    double terrain_min_support_margin_m = 0.0;
    double terrain_min_uncertainty_support_margin_m = 0.0;
    std::uint64_t terrain_committed_touchdowns = 0;
    std::uint64_t terrain_plan_contact_rejections = 0;
    const auto terrain_execution_plan = terrain_execution_plan_;
    const bool terrain_execution_plan_usable = terrain_execution_plan &&
        terrain_execution_plan->usable_at(state_tick_s);
    const double terrain_execution_plan_age_s = terrain_execution_plan
        ? state_tick_s - terrain_execution_plan->generated_at_s
        : 0.0;
    int terrain_execution_planned_contact_mask = 0;
    if (terrain_execution_plan_usable)
    {
        std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
            terrain_plan_indices{};
        if (go2_terrain::BuildTerrainPlanHorizonIndices(
                *terrain_execution_plan, state_tick_s,
                terrain_planner_.config().knot_dt_s,
                terrain_planner_.config().knot_dt_s, 1,
                terrain_plan_indices))
        {
            const auto &planned = terrain_execution_plan->contact_schedule
                .planned_contact[terrain_plan_indices[0]];
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                if (planned[leg])
                    terrain_execution_planned_contact_mask |=
                        1 << static_cast<int>(leg);
        }
    }
    int terrain_transfer_hold_mask = 0;
    int terrain_surface_transition_required_mask = 0;
    int terrain_surface_transition_original_required_mask = 0;
    int terrain_surface_transition_cancelled_mask = 0;
    int terrain_surface_transition_committed_mask = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (terrain_transfer_hold_contact_[leg])
            terrain_transfer_hold_mask |= 1 << static_cast<int>(leg);
        if (terrain_surface_transition_required_[leg])
            terrain_surface_transition_required_mask |=
                1 << static_cast<int>(leg);
        if (terrain_surface_transition_original_required_[leg])
            terrain_surface_transition_original_required_mask |=
                1 << static_cast<int>(leg);
        if (terrain_surface_transition_cancelled_[leg])
            terrain_surface_transition_cancelled_mask |=
                1 << static_cast<int>(leg);
        if (terrain_surface_transition_committed_[leg])
            terrain_surface_transition_committed_mask |=
                1 << static_cast<int>(leg);
    }
    std::array<go2::Vec3, go2::kLegCount> terrain_actual_world_feet{};
    bool terrain_actual_world_feet_valid = false;
    if (have_state && have_high_state)
    {
        std::array<double, kMotorCount> joint_positions{};
        for (std::size_t motor = 0; motor < joint_positions.size(); ++motor)
            joint_positions[motor] = state_snapshot.motor_state()[motor].q();
        const auto body_feet = go2::AllFootPositions(joint_positions);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            terrain_actual_world_feet[leg] = go2_control::BodyToWorld(
                pose.base, pose.quaternion, body_feet[leg]);
        terrain_actual_world_feet_valid = true;
    }

    const auto terrain_crawl_state = terrain_crawl_state_machine_.state();
    const int terrain_crawl_active_leg = static_cast<int>(
        terrain_crawl_state_machine_.ActiveLeg());
    const int terrain_crawl_min_contacts = terrain_crawl_min_contact_count_ ==
            go2::kLegCount ? 0 : terrain_crawl_min_contact_count_;
    int terrain_event_sequencer_committed_mask = 0;
    int terrain_event_sequencer_contact_schedule_mask = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (terrain_crawl_sequencer_output_.committed[leg])
            terrain_event_sequencer_committed_mask |= 1 << static_cast<int>(leg);
        if (terrain_crawl_sequencer_output_.contact_schedule[leg])
            terrain_event_sequencer_contact_schedule_mask |=
                1 << static_cast<int>(leg);
    }
    std::shared_ptr<const go2_terrain::TerrainModel> terrain_model;
    double terrain_last_map_age_s = std::numeric_limits<double>::infinity();
    double terrain_last_solver_us = 0.0;
    double terrain_last_plan_status = 0.0;
    std::size_t terrain_known_cells = 0;
    std::size_t terrain_feasible_regions = 0;
    std::uint64_t terrain_planner_updates = 0;
    std::uint64_t terrain_planner_rejections = 0;
    std::uint64_t terrain_planner_deadline_misses = 0;
    bool terrain_latest_plan_valid = false;
    std::uint64_t terrain_plan_published = 0;
    int terrain_failed_leg = -1;
    std::string terrain_failed_leg_reject_reason = "none";
    std::array<std::size_t, go2::kLegCount> terrain_candidate_counts{};
    std::array<std::size_t, go2::kLegCount>
        terrain_swing_candidate_counts{};
    std::array<bool, go2::kLegCount> terrain_candidate_required{};
    std::array<int, go2::kLegCount> terrain_touchdown_knots{};
    int terrain_support_failure_knot = -1;
    int terrain_support_failure_contact_mask = 0;
    double terrain_support_failure_margin_m = 0.0;
    if (params_.terrain_enabled)
    {
        std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
        terrain_model = terrain_model_;
        terrain_last_map_age_s = terrain_last_map_age_s_;
        terrain_last_solver_us = terrain_last_solver_us_;
        terrain_last_failure = terrain_last_failure_;
        terrain_dominant_foothold_reject = terrain_dominant_foothold_reject_reason_;
        terrain_failed_leg = terrain_failed_leg_;
        terrain_failed_leg_reject_reason = terrain_failed_leg_reject_reason_;
        terrain_candidate_counts = terrain_candidate_counts_;
        terrain_swing_candidate_counts = terrain_swing_candidate_counts_;
        terrain_candidate_required = terrain_candidate_required_;
        terrain_touchdown_knots = terrain_touchdown_knots_;
        terrain_support_failure_knot = terrain_support_failure_knot_;
        terrain_support_failure_contact_mask =
            terrain_support_failure_contact_mask_;
        terrain_support_failure_margin_m = terrain_support_failure_margin_m_;
        terrain_min_edge_margin_m = terrain_min_edge_margin_m_;
        terrain_min_uncertainty_edge_margin_m =
            terrain_min_uncertainty_edge_margin_m_;
        terrain_min_slope_rad = terrain_min_slope_rad_;
        terrain_max_roughness_m = terrain_max_roughness_m_;
        terrain_min_reachability_margin_m =
            terrain_min_reachability_margin_m_;
        terrain_min_swing_clearance_m = terrain_min_swing_clearance_m_;
        terrain_min_support_margin_m = terrain_min_support_margin_m_;
        terrain_min_uncertainty_support_margin_m =
            terrain_min_uncertainty_support_margin_m_;
        terrain_last_plan_status = terrain_last_plan_status_;
        terrain_committed_touchdowns = terrain_committed_touchdowns_;
        terrain_plan_contact_rejections = terrain_plan_contact_rejections_.load();
        terrain_known_cells = terrain_known_cells_;
        terrain_feasible_regions = terrain_feasible_regions_;
        terrain_planner_updates = terrain_planner_updates_;
        terrain_planner_rejections = terrain_planner_rejections_;
        terrain_planner_deadline_misses = terrain_planner_deadline_misses_;
        terrain_latest_plan_valid = terrain_latest_plan_valid_;
        terrain_plan_published = terrain_plan_published_count_;
    }
    const bool terrain_safe_stop_requested = params_.terrain_enabled &&
        terrain_safe_stop_requested_.load();
    const double terrain_velocity_cap_mps = params_.terrain_enabled
        ? terrain_velocity_cap_mps_.load()
        : std::numeric_limits<double>::infinity();

    // SECTION: log-state-summary (time, clock, pose, velocity, imu)
    csv_ << running_time_ << "," << state_tick_s << ","
         << (have_state ? 1 : 0) << "," << last_motion_dt_s_ << ","
         << (last_clock_paused_ ? 1 : 0) << "," << last_state_tick_gap_s_
         << "," << clock_pause_count_
         << "," << (params_.wall_clock_motion ? 1 : 0)
         << "," << last_wall_motion_dt_s_
         << "," << (last_motion_clock_paused_ ? 1 : 0)
         << "," << motion_clock_pause_count_
         << "," << task_.motion_stage_ << ","
         << active_cycle_index_ << "," << current_phase_ << ","
         << (motion_event_state_.event_active ? 1 : 0)
         << "," << motion_event_state_.active_priority
         << "," << static_cast<int>(motion_event_state_.active_event)
         << "," << static_cast<int>(motion_event_state_.active_source)
         << "," << (motion_reference_.hold_stance ? 1 : 0)
         << "," << motion_reference_.vx_mps
         << "," << motion_reference_.vy_mps
         << "," << motion_reference_.yaw_rate_radps
         << "," << motion_event_state_.target.vx_mps
         << "," << motion_event_state_.target.vy_mps
         << "," << motion_event_state_.target.yaw_rate_radps
         << "," << velocity_command_state_.requested_mps
         << "," << velocity_command_state_.shaped_mps
         << "," << velocity_command_state_.applied_mps
         << "," << (have_filtered_body_velocity_
                        ? latest_filtered_body_velocity_[0] : 0.0)
         << "," << (velocity_command_state_.active
                        ? velocity_command_state_.shaped_mps -
                              (have_filtered_body_velocity_
                                   ? latest_filtered_body_velocity_[0] : 0.0)
                        : 0.0)
         << "," << velocity_command_state_.accel_mps2
         << "," << velocity_command_state_.jerk_mps3
         << "," << (velocity_command_state_.active ? 1 : 0)
         << "," << kernel_period_s_
         << "," << kernel_duty_factor_
         << "," << runtime_gait_step_length_m_
         << "," << runtime_gait_foot_lift_m_
         << "," << runtime_gait_regime_
         << "," << (latest_motion_sensor_.have_obstacle_scan ? 1 : 0)
         << "," << (std::isfinite(latest_motion_sensor_.obstacle_scan_age_s)
                        ? latest_motion_sensor_.obstacle_scan_age_s : -1.0)
         << "," << (std::isfinite(latest_motion_sensor_.obstacle_center_distance_m)
                        ? latest_motion_sensor_.obstacle_center_distance_m : -1.0)
         << "," << (std::isfinite(latest_motion_sensor_.obstacle_left_distance_m)
                        ? latest_motion_sensor_.obstacle_left_distance_m : -1.0)
         << "," << (std::isfinite(latest_motion_sensor_.obstacle_right_distance_m)
                        ? latest_motion_sensor_.obstacle_right_distance_m : -1.0)
         << "," << latest_motion_sensor_.obstacle_center_height_m
         << "," << latest_motion_sensor_.obstacle_left_height_m
         << "," << latest_motion_sensor_.obstacle_right_height_m
         << "," << (params_.terrain_enabled ? 1 : 0)
         << "," << (params_.terrain_sensor_only ? 1 : 0)
         << "," << (params_.terrain_actuation ? 1 : 0)
         << "," << ((terrain_model && terrain_model->valid()) ? 1 : 0)
         << "," << (terrain_model
                           ? go2_terrain::TerrainSourceName(
                                 terrain_model->source)
                           : "none")
         << "," << (terrain_model ? terrain_model->epoch : 0)
         << "," << terrain_last_map_age_s
         << "," << terrain_known_cells
         << "," << terrain_feasible_regions
         << "," << static_cast<int>(terrain_last_plan_status)
         << "," << terrain_plan_id_.load()
         << "," << terrain_plan_epoch_.load()
         << "," << (terrain_latest_plan_valid ? 1 : 0)
         << "," << terrain_planner_updates
         << "," << terrain_planner_rejections
         << "," << terrain_planner_deadline_misses
         << "," << terrain_last_solver_us
         << "," << (terrain_safe_stop_requested ? 1 : 0)
         << "," << terrain_velocity_cap_mps
         << "," << terrain_plan_published
         << "," << terrain_plan_consumed_count_
         << "," << terrain_gait_target_override_count_
         << "," << terrain_mpc_plan_consumed_count_
         << "," << terrain_last_failure
         << "," << terrain_committed_touchdowns
         << "," << terrain_dominant_foothold_reject
         << "," << terrain_failed_leg
         << "," << terrain_failed_leg_reject_reason
         << "," << terrain_support_failure_knot
         << "," << terrain_support_failure_contact_mask
         << "," << terrain_support_failure_margin_m;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        csv_ << "," << terrain_candidate_counts[leg]
             << "," << terrain_swing_candidate_counts[leg]
             << "," << (terrain_candidate_required[leg] ? 1 : 0)
             << "," << terrain_touchdown_knots[leg];
    }
    csv_
         << "," << terrain_min_edge_margin_m
         << "," << terrain_min_uncertainty_edge_margin_m
         << "," << terrain_min_slope_rad
         << "," << terrain_max_roughness_m
         << "," << terrain_min_reachability_margin_m
         << "," << terrain_min_swing_clearance_m
         << "," << terrain_min_support_margin_m
         << "," << terrain_min_uncertainty_support_margin_m
         << "," << terrain_plan_contact_rejections
         << "," << (terrain_execution_plan
                          ? terrain_execution_plan->plan_id : 0)
         << "," << (terrain_execution_plan
                          ? terrain_execution_plan->plan_epoch : 0)
         << "," << (terrain_execution_plan
                          ? terrain_execution_plan->map_epoch : 0)
         << "," << (terrain_execution_plan
                          ? terrain_execution_plan->generated_at_s : 0.0)
         << "," << (terrain_execution_plan
                          ? terrain_execution_plan->valid_until_s : 0.0)
         << "," << terrain_execution_plan_age_s
         << "," << (terrain_execution_plan_usable ? 1 : 0)
         << "," << terrain_execution_planned_contact_mask
         << "," << (terrain_transfer_hold_active_ ? 1 : 0)
         << "," << terrain_transfer_hold_mask
         << "," << (terrain_transfer_window_active_ ? 1 : 0)
         << "," << terrain_transfer_window_release_s_
         << "," << go2_terrain::TerrainCrawlStateName(terrain_crawl_state)
         << "," << terrain_crawl_active_leg
         << "," << go2_terrain::TerrainCrawlSequencerStateName(
                terrain_crawl_sequencer_output_.state)
         << "," << (terrain_crawl_sequencer_output_.active_leg < go2::kLegCount
                ? static_cast<int>(terrain_crawl_sequencer_output_.active_leg) : -1)
         << "," << (terrain_crawl_sequencer_output_.control_authority_active ? 1 : 0)
         << "," << (terrain_crawl_sequencer_output_.authority_trot_full_contact_able ? 1 : 0)
         << "," << (terrain_crawl_sequencer_output_.authority_measured_contacts_ready ? 1 : 0)
         << "," << (terrain_crawl_sequencer_output_.authority_velocity_ready ? 1 : 0)
         << "," << (terrain_crawl_sequencer_output_.authority_posture_ready ? 1 : 0)
         << "," << (terrain_crawl_sequencer_output_.authority_stand_transition_seen ? 1 : 0)
         << "," << terrain_crawl_sequencer_output_.authority_block_reason
         << "," << terrain_crawl_sequencer_output_.authority_velocity_mps
         << "," << terrain_crawl_sequencer_output_.authority_roll_rad
         << "," << terrain_crawl_sequencer_output_.authority_pitch_rad
         << "," << terrain_crawl_sequencer_output_.stage_abort_reason
         << "," << (terrain_crawl_sequencer_output_.stand_transition_requested ? 1 : 0)
         << "," << (terrain_crawl_sequencer_output_.com_reference_valid ? 1 : 0)
         << "," << terrain_crawl_sequencer_output_.com_reference_world.x
         << "," << terrain_crawl_sequencer_output_.com_reference_world.y
         << "," << terrain_crawl_sequencer_output_.com_reference_world.z
         << "," << terrain_crawl_sequencer_output_.measured_contact_count
         << "," << (terrain_crawl_sequencer_output_.flat_ground_mode ? 1 : 0)
         << "," << terrain_event_sequencer_committed_mask
         << "," << terrain_event_sequencer_contact_schedule_mask
         << "," << terrain_crawl_sequencer_output_.swing_position_world.x
         << "," << terrain_crawl_sequencer_output_.swing_position_world.y
         << "," << terrain_crawl_sequencer_output_.swing_position_world.z
         << "," << terrain_crawl_sequencer_output_.target_world.x
         << "," << terrain_crawl_sequencer_output_.target_world.y
         << "," << terrain_crawl_sequencer_output_.target_world.z
         << "," << terrain_crawl_sequencer_output_.swing_start_world.x
         << "," << terrain_crawl_sequencer_output_.swing_start_world.y
         << "," << terrain_crawl_sequencer_output_.swing_start_world.z
         << "," << (terrain_execution_plan &&
                          terrain_execution_plan->staging_target_valid ? 1 : 0)
         << "," << terrain_staging_error_m_
         << "," << (terrain_execution_plan
                          ? terrain_execution_plan->staging_target_world_x_m : 0.0)
         << "," << terrain_crawl_state_machine_.retry_count()
         << "," << terrain_crawl_state_machine_.state_enter_time_s()
         << "," << terrain_crawl_state_machine_.com_shift_duration_s()
         << "," << terrain_crawl_state_machine_.shift_recovery_count()
         << "," << terrain_crawl_min_contacts
         << "," << terrain_crawl_step_commit_count_
         << "," << (have_measured_com_world_ ? measured_com_world_.x : 0.0)
         << "," << (have_measured_com_world_ ? measured_com_world_.y : 0.0)
         << "," << terrain_crawl_state_machine_.com_margin_m()
         << "," << terrain_crawl_state_machine_.com_target_world().x
         << "," << terrain_crawl_state_machine_.com_target_world().y
         << "," << (terrain_crawl_state_machine_.com_target_valid() ? 1 : 0)
         << "," << terrain_crawl_state_machine_.com_support_lifted_leg()
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[0].x
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[0].y
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[0].z
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[1].x
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[1].y
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[1].z
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[2].x
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[2].y
         << "," << terrain_crawl_state_machine_.com_support_triangle().vertex[2].z
         << "," << (terrain_crawl_state_machine_.state() ==
                          go2_terrain::TerrainCrawlState::kStage ? "4-contact-polygon" :
                          "FL-lifted-triangle")
         << "," << terrain_crawl_state_machine_.stage_basin_margin_m()
         << "," << terrain_crawl_state_machine_.stage_target_margin_m()
         << "," << terrain_crawl_state_machine_.com_target_world().x
         << "," << terrain_crawl_state_machine_.com_target_world().y
         << "," << (terrain_crawl_state_machine_.stage_com_target_valid() ? 1 : 0)
         << "," << terrain_crawl_state_machine_.stage_retry_count()
         << "," << (terrain_crawl_state_machine_.stage_micro_adjust_active(running_time_) ? 1 : 0)
         << "," << terrain_crawl_state_machine_.stage_micro_adjust_direction()
         << "," << terrain_stage_servo_acc_x_mps2_
         << "," << terrain_stage_servo_acc_y_mps2_
         << "," << terrain_shift_servo_acc_x_mps2_
         << "," << terrain_shift_servo_acc_y_mps2_
         << "," << (terrain_stage_servo_saturated_ ? 1 : 0)
         << "," << (terrain_stance_reference_valid_ ? 1 : 0)
         << "," << terrain_stance_reference_roll_rad_
         << "," << terrain_stance_reference_pitch_rad_
         << "," << (terrain_surface_transition_active_ ? 1 : 0)
         << "," << terrain_surface_transition_required_mask
         << "," << terrain_surface_transition_original_required_mask
         << "," << terrain_surface_transition_cancelled_mask
         << "," << terrain_surface_transition_committed_mask
         << "," << terrain_surface_transition_completions_
         << "," << terrain_surface_transition_last_required_mask_
         << "," << terrain_surface_transition_last_committed_mask_
         << "," << terrain_target_prepare_attempt_count_
         << "," << terrain_target_prepared_count_
         << "," << terrain_target_prepare_rejection_count_
         << "," << terrain_target_last_prepare_failure_;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &execution = terrain_swing_execution_[leg];
        const auto &pending = terrain_swing_pending_[leg];
        const double execution_phase = execution.valid &&
                std::isfinite(execution.trajectory_start_time_s) &&
                std::isfinite(execution.swing_duration_s) &&
                execution.swing_duration_s > 1.0e-9
            ? std::clamp(
                  (state_tick_s - execution.trajectory_start_time_s) /
                      execution.swing_duration_s,
                  0.0, 1.0)
            : 0.0;
        const auto actual_foot = terrain_actual_world_feet_valid
            ? terrain_actual_world_feet[leg] : go2::Vec3{};
        csv_ << "," << (execution.valid ? 1 : 0)
             << "," << (execution.in_flight ? 1 : 0)
             << "," << (execution.endpoint_held ? 1 : 0)
             << "," << (execution.measured_touchdown ? 1 : 0)
             << "," << execution.wbc_endpoint_error_m
             << "," << (execution.wbc_at_endpoint ? 1 : 0)
             << "," << (execution.wbc_measured_contact ? 1 : 0)
             << "," << (execution.terrain_target_required ? 1 : 0)
             << "," << (execution.terrain_height_change ? 1 : 0)
             << "," << (execution.time_rebased_at_handoff ? 1 : 0)
             << "," << execution.plan_id
             << "," << execution.map_epoch
             << "," << execution.swing_start_time_s
             << "," << execution.trajectory_start_time_s
             << "," << execution.nominal_touchdown_time_s
             << "," << execution.touchdown_time_s
             << "," << execution.swing_duration_s
             << "," << execution.terrain_swing_duration_s
             << "," << execution.planned_swing_duration_s
             << "," << (execution.terrain_swing_duration_s -
                              execution.planned_swing_duration_s)
             << "," << execution_phase
             << "," << execution.start_world.x
             << "," << execution.start_world.y
             << "," << execution.start_world.z
             << "," << execution.target_world.x
             << "," << execution.target_world.y
             << "," << execution.target_world.z
             << "," << actual_foot.x
             << "," << actual_foot.y
             << "," << actual_foot.z
             << "," << (pending.valid ? 1 : 0)
             << "," << pending.plan_id
             << "," << pending.touchdown_time_s
             << "," << pending.target_world.x
             << "," << pending.target_world.y
             << "," << pending.target_world.z
             << "," << terrain_target_last_prepare_failure_by_leg_[leg];
    }
    csv_
         << "," << (latest_motion_sensor_.have_support_foot_kinematics ? 1 : 0)
         << "," << latest_motion_sensor_.support_foot_count
         << "," << latest_motion_sensor_.support_foot_speed_mps
         << "," << latest_motion_sensor_.support_low_friction_evidence
         << "," << latest_motion_sensor_.low_friction_accumulation
         << "," << pose.base.x << "," << pose.base.y << "," << pose.base.z << ","
         << world_yaw_error_rad_
         << "," << world_feedback_x_m_ << "," << world_feedback_y_m_
         << "," << (have_body_velocity ? body_velocity[0] : 0.0)
         << "," << (have_body_velocity ? body_velocity[1] : 0.0)
         << "," << (have_body_velocity ? body_velocity[2] : 0.0)
         << "," << world_velocity[0]
         << "," << world_velocity[1]
         << "," << world_velocity[2]
         << "," << raw_body_velocity[0]
         << "," << raw_body_velocity[1]
         << "," << raw_body_velocity[2]
         << "," << imu_acceleration[0]
         << "," << imu_acceleration[1]
         << "," << imu_acceleration[2]
         << "," << imu_gyro[0]
         << "," << imu_gyro[1]
         << "," << imu_gyro[2]
         << "," << imu_gyro_body[0]
         << "," << imu_gyro_body[1]
         << "," << imu_gyro_body[2]
         << "," << imu_rpy[0]
         << "," << imu_rpy[1]
         << "," << imu_rpy[2]
         << "," << velocity_filter_alpha_
         << "," << (have_filtered_body_velocity_ ? 1 : 0)
         << "," << (kernel_footstep_plan_valid_ ? 1 : 0)
         << "," << kernel_velocity_error_x_mps_
         << "," << kernel_touchdown_target_x_m_[0]
         << "," << kernel_touchdown_target_x_m_[1]
         << "," << kernel_touchdown_target_x_m_[2]
         << "," << kernel_touchdown_target_x_m_[3]
         << "," << attitude_feedback_x_m_ << ","
         << attitude_feedback_y_m_ << "," << contact_count
         << "," << foot_forces[0] << "," << foot_forces[1]
         << "," << foot_forces[2] << "," << foot_forces[3]
         << "," << contact_flags[0] << "," << contact_flags[1]
         << "," << contact_flags[2] << "," << contact_flags[3]
         << "," << touchdown_event_count_
         << "," << last_touchdown_leg_
         << "," << last_touchdown_command_x_m_
         << "," << last_touchdown_actual_x_m_
         << "," << last_touchdown_x_error_m_
         << "," << last_touchdown_y_error_m_
         << "," << (wbc_shadow_diagnostics_.enabled ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.desired_force_x_n
         << "," << (wbc_shadow_diagnostics_.reduced_contact_task ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.task_satisfied ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.task_residual_norm
         << "," << (wbc_shadow_diagnostics_.solver_ok ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.mapping_ok ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.active_contacts
         << "," << wbc_shadow_diagnostics_.contact_mask
         << "," << wbc_shadow_diagnostics_.iterations
         << "," << (wbc_shadow_diagnostics_.wrench_satisfied ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.constraint_feasible ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.max_axis_friction_ratio
         << "," << wbc_shadow_diagnostics_.max_radial_friction_ratio
         << "," << wbc_shadow_diagnostics_.min_contact_normal_force_n
         << "," << wbc_shadow_diagnostics_.id_wbc_normal_force_n[0]
         << "," << wbc_shadow_diagnostics_.id_wbc_normal_force_n[1]
         << "," << wbc_shadow_diagnostics_.id_wbc_normal_force_n[2]
         << "," << wbc_shadow_diagnostics_.id_wbc_normal_force_n[3]
         << "," << (terrain_force_telemetry ? 1 : 0);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const bool held_leg = terrain_transfer_hold_contact_[leg];
        csv_ << "," << (terrain_force_telemetry && held_leg
                              ? foot_forces[leg] : 0.0)
             << "," << (terrain_force_telemetry && held_leg
                              ? wbc_shadow_diagnostics_.terrain_hold_wbc_normal_force_n[leg]
                              : 0.0);
    }
    csv_
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_base_linear : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_base_angular : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_stance_no_slip : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_swing : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_force_regularization : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_force_tracking : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_posture : 0.0)
         << "," << (terrain_force_telemetry
                              ? wbc_shadow_diagnostics_.terrain_hold_cost_torque : 0.0)
         << "," << wbc_shadow_diagnostics_.residual_norm
         << "," << wbc_shadow_diagnostics_.max_abs_tau
         << "," << wbc_shadow_diagnostics_.elapsed_us
         << "," << (wbc_shadow_diagnostics_.within_budget ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.feedforward_ready ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.feedforward_applied ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.feedforward_reduced_task_gate ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.feedforward_gate_code
         << "," << go2_control::WbcFeedforwardGateReasonName(
                static_cast<go2_control::WbcFeedforwardGateCode>(
                    wbc_shadow_diagnostics_.feedforward_gate_code))
         << "," << wbc_shadow_diagnostics_.feedforward_max_abs_tau
         << "," << (wbc_shadow_diagnostics_.srbd_ok ? 1 : 0)
         << "," << (wbc_shadow_diagnostics_.id_wbc_ok ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.id_eq_residual
         << "," << wbc_shadow_diagnostics_.full_velocity_target_x_mps
         << "," << wbc_shadow_diagnostics_.full_requested_acc_x_mps2
         << "," << wbc_shadow_diagnostics_.full_srbd_acc_x_mps2
         << "," << wbc_shadow_diagnostics_.full_id_qdd_x_mps2
         << "," << wbc_shadow_diagnostics_.full_id_contact_force_x_n
         << "," << wbc_shadow_diagnostics_.measured_contact_mask
         << "," << wbc_shadow_diagnostics_.scheduled_contact_mask
         << "," << wbc_shadow_diagnostics_.terrain_planned_contact_mask
         << "," << wbc_shadow_diagnostics_.mpc_update_count
         << "," << wbc_shadow_diagnostics_.mpc_contact_mask_k0
         << "," << wbc_shadow_diagnostics_.mpc_min_contact_count
         << "," << wbc_shadow_diagnostics_.mpc_reference_x_first_m
         << "," << wbc_shadow_diagnostics_.mpc_reference_x_last_m
         << "," << wbc_shadow_diagnostics_.mpc_reference_vx_first_mps
         << "," << wbc_shadow_diagnostics_.mpc_reference_vx_last_mps
         << "," << (wbc_shadow_diagnostics_.terrain_contact_coherent ? 1 : 0)
         << "," << wbc_shadow_diagnostics_.terrain_plan_id;

    // SECTION: log-joint-cmds (cmd vs state per joint)
    for (int i = 0; i < kMotorCount; ++i)
    {
        const auto &motor_cmd = low_cmd_.motor_cmd()[i];
        const double q_state = have_state
            ? static_cast<double>(state_snapshot.motor_state()[i].q())
            : 0.0;
        const double dq_state = have_state
            ? static_cast<double>(state_snapshot.motor_state()[i].dq())
            : 0.0;
        const double tau_state = have_state
            ? static_cast<double>(state_snapshot.motor_state()[i].tau_est())
            : 0.0;
        csv_ << "," << motor_cmd.q()
             << "," << motor_cmd.dq()
             << "," << motor_cmd.kp()
             << "," << motor_cmd.kd()
             << "," << motor_cmd.tau()
             << "," << q_state
             << "," << dq_state
             << "," << tau_state
             << "," << (motor_cmd.q() - q_state);
    }
    csv_ << "\n";
}
