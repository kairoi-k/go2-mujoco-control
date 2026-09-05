#include "trot_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

#include "contact_wrench_projected_allocator.h"
#include "contact_state_filter.h"
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "motion_frame_utils.h"
#include "cartesian_world_trot.h"
#include "full2_campaign_env.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

// --- TrotExperiment::UpdateVelocityEstimate ---
void TrotExperiment::UpdateVelocityEstimate(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state,
    double motion_dt)
{
    have_world_velocity_ = false;
    have_raw_body_velocity_ = false;
    have_filtered_body_velocity_ = false;
    velocity_filter_alpha_ = 0.0;
    if (!have_high_state)
    {
        have_filtered_body_velocity_ = false;
        return;
    }

    const auto &velocity = high_state_snapshot.velocity();
    latest_world_velocity_ = {velocity[0], velocity[1], velocity[2]};
    have_world_velocity_ =
        std::isfinite(velocity[0]) &&
        std::isfinite(velocity[1]) &&
        std::isfinite(velocity[2]);
    if (!have_world_velocity_)
    {
        have_filtered_body_velocity_ = false;
        return;
    }

    const go2_control::Quaternion orientation{
        state_snapshot.imu_state().quaternion()[0],
        state_snapshot.imu_state().quaternion()[1],
        state_snapshot.imu_state().quaternion()[2],
        state_snapshot.imu_state().quaternion()[3]};
    go2_control::Vector3 raw_body_velocity{};
    if (!go2_control::WorldToBodyVelocity(
            orientation, latest_world_velocity_, raw_body_velocity))
    {
        have_filtered_body_velocity_ = false;
        return;
    }

    latest_raw_body_velocity_ = raw_body_velocity;
    have_raw_body_velocity_ = true;
    if (params_.velocity_filter_cutoff_hz == 0.0)
    {
        latest_filtered_body_velocity_ = raw_body_velocity;
        have_filtered_body_velocity_ = true;
        velocity_filter_alpha_ = 1.0;
        return;
    }

    go2_control::Vector3 filtered_body_velocity{};
    if (velocity_filter_.Update(
            raw_body_velocity, motion_dt, filtered_body_velocity))
    {
        latest_filtered_body_velocity_ = filtered_body_velocity;
        have_filtered_body_velocity_ = true;
        velocity_filter_alpha_ = velocity_filter_.last_alpha();
    }
    // 冲量主控: body 系速度一阶差分 -> 加速度估计(供线动量任务阻尼)
    if (have_filtered_body_velocity_ &&
        have_body_acceleration_ &&
        std::isfinite(motion_dt) &&
        motion_dt > 0.0)
    {
        latest_body_acceleration_[0] =
            (latest_filtered_body_velocity_[0] -
             latest_body_acceleration_prev_v_[0]) / motion_dt;
        latest_body_acceleration_[1] =
            (latest_filtered_body_velocity_[1] -
             latest_body_acceleration_prev_v_[1]) / motion_dt;
        latest_body_acceleration_[2] =
            (latest_filtered_body_velocity_[2] -
             latest_body_acceleration_prev_v_[2]) / motion_dt;
        latest_body_acceleration_prev_v_ = latest_filtered_body_velocity_;
    }
    else if (have_filtered_body_velocity_)
    {
        latest_body_acceleration_ = {};
        latest_body_acceleration_prev_v_ = latest_filtered_body_velocity_;
        have_body_acceleration_ = true;
    }
    else
    {
        have_body_acceleration_ = false;
    }
}

// --- TrotExperiment::BuildGaitTargets ---
void TrotExperiment::UpdateRuntimeVelocityCommand(double gait_time_s)
{
    if (!params_.runtime_velocity_command)
    {
        velocity_command_state_ = {};
        runtime_gait_regime_ = "inactive";
        return;
    }
    if (high_speed_stop_brake_active_ || high_speed_stop_hold_active_ ||
        stop_brake_active_ || task_.stop_requested_)
    {
        velocity_command_state_.requested_mps = 0.0;
        velocity_command_state_.shaped_mps = 0.0;
        velocity_command_state_.applied_mps = 0.0;
        velocity_command_state_.accel_mps2 = 0.0;
        velocity_command_state_.jerk_mps3 = 0.0;
        velocity_command_state_.active = true;
        runtime_gait_regime_ = "stop-brake";
        wbc_speed_cmd_mps_ = 0.0;
        return;
    }
    if (!velocity_command_initialized_ || gait_time_s <= 1.0e-9)
    {
        velocity_command_shaper_.Reset(0.0);
        velocity_gait_scheduler_.Reset();
        velocity_stance_hold_gate_.Reset();
        velocity_command_initialized_ = true;
    }
    const double dt = (std::isfinite(last_motion_dt_s_) &&
                       last_motion_dt_s_ > 1.0e-4)
        ? last_motion_dt_s_
        : dt_;
    const double requested_mps =
        params_.velocity_command_profile.Sample(gait_time_s);
    velocity_command_state_ = velocity_command_shaper_.Step(
        requested_mps, dt);
    const bool zero_command_profile_finished =
        !params_.velocity_command_profile.points.empty() &&
        params_.velocity_command_profile.points.back().velocity_mps <= 1.0e-6 &&
        gait_time_s >= params_.velocity_command_profile.points.back().time_s &&
        velocity_command_state_.shaped_mps <= 0.05;
    const double measured_command_speed = have_filtered_body_velocity_
        ? std::max(0.0, params_.direction_sign * latest_filtered_body_velocity_[0])
        : 0.0;
    const bool command_stop_ready = zero_command_profile_finished &&
        (!have_filtered_body_velocity_ || measured_command_speed <= 0.25);
    if (command_stop_ready && !task_.stop_requested_)
    {
        task_.stop_requested_ = true;
        task_.motion_stage_ = 3;
        task_.task_completion_requested_ = false;
        std::cout << "Runtime velocity profile reached zero; stopping in place\n";
    }
    double applied_mps = go2_trot::GovernAppliedVelocity(
        velocity_command_state_.shaped_mps,
        have_filtered_body_velocity_
            ? params_.direction_sign * latest_filtered_body_velocity_[0]
            : velocity_command_state_.shaped_mps,
        params_.velocity_command_shaper.max_tracking_lead_mps);
    const bool runtime_health_governor = params_.wbc_full &&
        !params_.cartesian_world &&
        Full2EnvDouble("TROT_HS_STABILITY_GOV", 0.0) > 0.5;
    if (runtime_health_governor && high_speed_health_cap_mps_ > 0.0 &&
        applied_mps > high_speed_health_cap_mps_)
    {
        applied_mps = high_speed_health_cap_mps_;
    }
    velocity_command_state_.applied_mps = applied_mps;
    runtime_velocity_stance_hold_active_ =
        velocity_stance_hold_gate_.Step(
            requested_mps,
            velocity_command_state_.shaped_mps,
            have_filtered_body_velocity_
                ? latest_filtered_body_velocity_[0] : 0.0,
            have_filtered_body_velocity_);
    const auto schedule = velocity_gait_scheduler_.Step(applied_mps, dt);
    locomotion_kernel_->SetGaitEffectiveSpeedConvention(true);
    locomotion_kernel_->SetGaitSlewLimits(0.060, 0.020, 0.020);
    locomotion_kernel_->SetGaitPeriod(schedule.period_s);
    locomotion_kernel_->SetGaitDuty(schedule.duty_factor);
    locomotion_kernel_->SetGaitStepLength(schedule.step_length_m);
    locomotion_kernel_->SetGaitFootLift(schedule.foot_lift_m);
    wbc_speed_cmd_mps_ =
        std::abs(params_.direction_sign) * applied_mps;
    runtime_gait_step_length_m_ = schedule.step_length_m;
    runtime_gait_foot_lift_m_ = schedule.foot_lift_m;
    runtime_gait_regime_ = schedule.regime;
}
bool TrotExperiment::BuildGaitTargets(
    double gait_time_s,
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state,
    std::array<double, kMotorCount> &joint_targets)
{
    auto feet = go2::AllFootPositions(task_.stand_up_joint_pos_);
    go2_control::GaitKernelResult gait_result{};
    go2_control::GaitKernelRequest gait_request{};
    const double phase_period_s = params_.period_s > 0.0
        ? params_.period_s : kDefaultPeriodS;
    gait_request.gait_time_s =
        gait_time_s + params_.gait_phase_offset * phase_period_s;
    gait_request.neutral_feet = feet;
    if (have_filtered_body_velocity_)
    {
        gait_request.body_velocity_x_mps =
            latest_filtered_body_velocity_[0];
        gait_request.body_velocity_y_mps =
            latest_filtered_body_velocity_[1];
        gait_request.body_velocity_z_mps =
            latest_filtered_body_velocity_[2];
        gait_request.have_body_velocity = true;
    }
    const bool runtime_velocity_command = params_.runtime_velocity_command;
    if (runtime_velocity_command)
        UpdateRuntimeVelocityCommand(gait_time_s);
    const double requested_speed = runtime_velocity_command
        ? velocity_command_state_.shaped_mps
        : std::abs(
              2.0 * params_.duty_factor * params_.direction_sign *
              params_.step_length_m /
              std::max(1.0e-3, params_.period_s));
    const bool high_speed_curriculum =
        !runtime_velocity_command &&
        Full2EnvDouble("TROT_HS_DISABLE", 0.0) <= 0.5 &&
        (params_.gait_pattern != go2_control::GaitPattern::kDiagonalTrot ||
         requested_speed > 1.25);
    locomotion_kernel_->SetGaitEffectiveSpeedConvention(
        params_.wbc_full && (high_speed_curriculum || runtime_velocity_command));
    const double swing_reach_override = Full2EnvDouble(
        "TROT_HS_SWING_REACH", -1.0);
    if (swing_reach_override >= 0.5)
        locomotion_kernel_->SetGaitSwingReachPhase(
            std::clamp(swing_reach_override, 0.5, 1.0));
    if (params_.wbc_full && high_speed_curriculum &&
        wbc_speed_cmd_mps_ < 0.0)
    {
        const double target_period = params_.period_s;
        const double requested_target_speed = std::abs(
            2.0 * params_.duty_factor * params_.direction_sign *
            params_.step_length_m /
            std::max(1.0e-3, target_period));
        const double target_speed_override = Full2EnvDouble(
            "TROT_HS_TARGET_SPEED", -1.0);
        const double target_speed = target_speed_override >= 0.0
            ? std::clamp(target_speed_override, 0.20, 3.50)
            : requested_target_speed;
        const double seed_speed = high_speed_curriculum
            ? std::min(target_speed, 0.90)
            : std::min(target_speed, 0.50);
        wbc_speed_cmd_mps_ = seed_speed;
        // Use the running plant as the seed for low-duty patterns.  The old
        // universal 0.40 s seed was a walk-like transition that changed the
        // contact timing again one tick later, exactly when a bound/gallop
        // is trying to build forward momentum.
        const double start_period = std::clamp(
            Full2EnvDouble(
                "TROT_HS_START_PERIOD",
                high_speed_curriculum ? 0.26 : 0.40),
            0.08, 1.0);
        const double start_duty = std::clamp(
            Full2EnvDouble(
                "TROT_HS_START_DUTY",
                high_speed_curriculum ? 0.45 : 0.60),
            0.25, 0.90);
        locomotion_kernel_->SetGaitPeriod(start_period);
        locomotion_kernel_->SetGaitDuty(start_duty);
        locomotion_kernel_->SetGaitStepLength(
            wbc_speed_cmd_mps_ * start_period / (2.0 * start_duty));
        locomotion_kernel_->SetGaitFootLift(
            std::max(params_.foot_lift_m, 0.08));
        const double hs_step_slew = std::clamp(Full2EnvDouble("TROT_HS_STEP_SLEW", 0.012), 0.008, 0.10);
        locomotion_kernel_->SetGaitSlewLimits(hs_step_slew, 0.020, 0.020);
    }
    // A high-speed brake is still the same locomotion plant, but its foot
    // offsets must converge to the neutral four-foot support before WBC is
    // asked to hold. Keep the kernel's stance-hold blend latched through
    // both the brake and the post-brake WBC hold.
    // Keep the running contact exchange alive while braking.  Freezing the
    // feet at the brake trigger would turn a still-fast two-contact gait into
    // an inconsistent four-contact plant before the body's momentum is gone.
    // Enter stance hold only after the measured-speed/attitude gate below
    // has accepted the brake completion.
    if (params_.runtime_velocity_command)
        locomotion_kernel_->SetStanceHold(
            runtime_velocity_stance_hold_active_, gait_time_s);
    if (high_speed_stop_hold_active_)
        locomotion_kernel_->SetStanceHold(true, gait_time_s);
    if (!locomotion_kernel_->Compute(gait_request, gait_result))
    {
        std::cerr << "Locomotion kernel failed at gait_time="
                  << gait_time_s << "\n";
        return false;
    }
    kernel_footstep_plan_valid_ = gait_result.footstep_plan_valid;
    kernel_touchdown_target_feet_base_ = gait_result.touchdown_target_feet_base;
    have_kernel_touchdown_target_feet_ = gait_result.touchdown_target_feet_valid;
    kernel_velocity_error_x_mps_ = gait_result.velocity_error_x_mps;
    kernel_nominal_velocity_x_mps_ = gait_result.nominal_velocity_x_mps;
    kernel_touchdown_target_x_m_ = gait_result.touchdown_target_x_m;
    if (params_.runtime_velocity_command)
        kernel_nominal_velocity_x_mps_ =
            params_.direction_sign * velocity_command_state_.applied_mps;
    else if (params_.cartesian_world && wbc_speed_cmd_mps_ > 0.0)
        kernel_nominal_velocity_x_mps_ =
            params_.direction_sign * wbc_speed_cmd_mps_;
    if (params_.wbc_full && high_speed_curriculum &&
        wbc_speed_cmd_mps_ > 0.0)
        kernel_nominal_velocity_x_mps_ =
            params_.direction_sign * wbc_speed_cmd_mps_;
    if (motion_event_response_enabled_)
        kernel_nominal_velocity_x_mps_ = motion_reference_.vx_mps;
    if (gait_result.period_s > 0.0)
        kernel_period_s_ = gait_result.period_s;
    if (gait_result.duty_factor > 0.0)
        kernel_duty_factor_ = gait_result.duty_factor;
    preview_n_steps_ = gait_result.preview_n_steps;
    preview_touchdown_x_m_ = gait_result.preview_touchdown_x_m;
    preview_terminal_velocity_x_mps_ =
        gait_result.preview_terminal_velocity_x_mps;
    preview_planned_acc_x_mps2_ = gait_result.preview_planned_acc_x_mps2;
    have_preview_terminal_velocity_ = preview_n_steps_ > 0;
    const double phase = gait_result.phase;
    current_phase_ = phase;
    const int cycle_index = gait_result.cycle_index;
    if (active_cycle_index_ < 0)
    {
        active_cycle_index_ = cycle_index;
        ResetCycleDiagnostics();
        if (gait_result.preview_n_steps > 0)
        {
            std::cout << "WBC-FULL preview n=" << gait_result.preview_n_steps
                      << " x0=" << gait_result.preview_touchdown_x_m << "\n";
        }
    }
    else if (cycle_index > active_cycle_index_)
    {
        const double cycle_roll =
            cycle_diagnostics_.max_abs_roll_rad;
        const double cycle_pitch =
            cycle_diagnostics_.max_abs_pitch_rad;
        const double cycle_drift =
            cycle_diagnostics_.max_support_drift_m;
        const double cycle_contact =
            cycle_diagnostics_.support_contact_samples == 0
                ? 1.0
                : static_cast<double>(
                      cycle_diagnostics_.support_contact_good_samples) /
                      static_cast<double>(
                          cycle_diagnostics_.support_contact_samples);
        const double v_meas =
            cycle_vx_count_ > 0
                ? params_.direction_sign * cycle_vx_sum_ /
                      static_cast<double>(cycle_vx_count_)
                : (have_filtered_body_velocity_
                       ? params_.direction_sign *
                             latest_filtered_body_velocity_[0]
                       : 0.0);
        const double cycle_foot = cycle_diagnostics_.max_foot_error_m;
        const bool high_speed_health_governor =
            params_.wbc_full && !params_.cartesian_world &&
            Full2EnvDouble("TROT_HS_STABILITY_GOV", 0.0) > 0.5;
        const bool cycle_valid = ValidateCycle(active_cycle_index_);
        if (!cycle_valid && !high_speed_health_governor)
        {
            // A high-speed cycle rejection is still a locomotion-plant
            // failure, not permission to jump directly to the old
            // stand-return phase.  Latching the bounded brake here keeps the
            // same gait/WBC plant in charge until measured speed and attitude
            // are safe.  Previously this path set stop_requested_ directly;
            // the next tick then skipped the health guard and a rare late
            // touchdown could roll the body before the stand phase began.
            if (HighSpeedStopBrakeEnabled() &&
                !high_speed_stop_brake_active_ &&
                !high_speed_stop_hold_active_ &&
                !task_.stop_requested_ &&
                !external_stop_requested_.load())
            {
                high_speed_stop_brake_active_ = true;
                high_speed_stop_brake_duration_s_ = std::clamp(
                    Full2EnvDouble(
                        "TROT_HS_GOV_HEALTH_BRAKE_DURATION_S", 1.20),
                    0.25, 2.00);
                const double health_brake_u = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_HEALTH_BRAKE_U", 0.35),
                    0.0, 0.90);
                high_speed_stop_brake_start_time_s_ =
                    running_time_ -
                    health_brake_u * high_speed_stop_brake_duration_s_;
                high_speed_stop_brake_base_speed_mps_ = std::max(
                    std::abs(v_meas), std::abs(wbc_speed_cmd_mps_));
                if (!(high_speed_stop_brake_base_speed_mps_ > 0.05) &&
                    have_world_velocity_)
                    high_speed_stop_brake_base_speed_mps_ = std::abs(
                        params_.direction_sign * latest_world_velocity_[0]);
                if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
                    high_speed_stop_brake_base_speed_mps_ = std::abs(
                        params_.direction_sign * params_.step_length_m /
                        std::max(0.08, params_.period_s));
                high_speed_stop_brake_base_period_s_ =
                    kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
                high_speed_stop_brake_base_duty_ =
                    kernel_duty_factor_ > 0.25 ? kernel_duty_factor_ : params_.duty_factor;
                std::cout << "High-speed cycle rejection: braking"
                          << " cycle=" << active_cycle_index_
                          << " v=" << high_speed_stop_brake_base_speed_mps_
                          << " roll=" << cycle_roll
                          << " pitch=" << cycle_pitch
                          << " foot=" << cycle_foot << "\n";
            }
            else if (!HighSpeedStopBrakeEnabled())
                task_.stop_requested_ = true;
        }
        ++completed_cycles_;
        if (max_cycles_ > 0 && completed_cycles_ >= max_cycles_ &&
            !emergency_stop_latched_)
        {
            if (!stop_brake_active_)
            {
                stop_brake_active_ = true;
                stop_brake_start_time_s_ = running_time_;
                stop_brake_base_step_m_ =
                    std::abs(gait_result.step_length_m);
                std::cout << "Trot pre-stop brake: reducing gait reference\n";
            }
        }
        if (high_speed_health_governor)
        {
            const double target_speed_override =
                Full2EnvDouble("TROT_HS_TARGET_SPEED", -1.0);
            const double requested_speed = std::abs(
                2.0 * params_.duty_factor * params_.step_length_m /
                std::max(1.0e-3, params_.period_s));
            const double target_speed = target_speed_override >= 0.0
                ? std::clamp(target_speed_override, 0.20, 3.50)
                : requested_speed;
            if (high_speed_health_cap_mps_ < 0.0)
                high_speed_health_cap_mps_ = target_speed;
            const double gov_joint_limit = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_JOINT_ERROR", 0.72),
                0.30, 1.20);
            const double gov_foot_limit = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_FOOT_ERROR", 0.16),
                0.05, 0.40);
            const double gov_drift_limit = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_DRIFT", 0.075),
                0.010, 0.20);
            const double gov_angle_limit = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_ANGLE_RAD", 0.16),
                0.04, 0.40);
            const double gov_contact_limit = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_CONTACT", 0.28),
                0.05, 0.80);
            const double gov_min_speed = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_MIN_SPEED", 1.80),
                0.80, 3.20);
            const double gov_min_gait_s = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_MIN_GAIT_S", 10.0),
                0.0, 60.0);
            // The cycle-average measured speed can already be falling when a
            // bad touchdown is first visible.  Keep the governor armed while
            // the commanded sprint reference is still high; otherwise the
            // very cycle that needs the brake is classified as low-speed and
            // the guard arrives one cycle too late.
            // Once a run has genuinely entered the sprint regime, keep the
            // health gate armed even if an earlier degraded cycle has already
            // pulled the reference below gov_min_speed.  Requiring the speed
            // threshold on every cycle created a blind spot exactly when the
            // bad touchdown was slowing the body and posture was worsening.
            const bool health_eval = gait_time_s >= gov_min_gait_s &&
                (high_speed_guard_armed_ ||
                 std::max(std::abs(v_meas), std::abs(wbc_speed_cmd_mps_)) >=
                     gov_min_speed);
            // A single large tracking residual is common during a healthy
            // aerial stride: the swing leg is intentionally far from its
            // touchdown target while the stance pair carries the body.  Do
            // not throttle a good 3 m/s run for that alone.  Declare a
            // degraded cycle only when tracking, foothold drift, attitude,
            // or support loss agree that recovery is actually needed.
            const bool tracking_degraded =
                cycle_diagnostics_.max_abs_joint_error_rad > gov_joint_limit &&
                cycle_foot > gov_foot_limit &&
                cycle_drift > 0.50 * gov_drift_limit;
            const bool attitude_degraded =
                std::max(cycle_roll, cycle_pitch) > gov_angle_limit;
            // A low contact fraction is expected near an aerial hand-off.
            // Treat it as a fault only when the same cycle also shows
            // attitude or foothold drift, rather than using contact count as
            // a standalone speed limiter.
            const bool support_degraded =
                cycle_contact < gov_contact_limit &&
                (std::max(cycle_roll, cycle_pitch) > gov_angle_limit ||
                 cycle_drift > gov_drift_limit);
            // A sprint can lose a touchdown without immediately tilting. A
            // very low contact fraction plus a long low-support gap is the
            // measurable precursor to the late posture failures seen in
            // the 3 m/s runs. Treat it as a soft reference degradation so
            // the same WBC plant gets more support time before braking.
            const double support_gap_fraction = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_SUPPORT_GAP_FRACTION", 0.18),
                0.05, 0.50);
            const double support_gap_samples = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_SUPPORT_GAP_SAMPLES", 45.0),
                5.0, 200.0);
            const double support_gap_drift = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_SUPPORT_GAP_DRIFT", 0.040),
                0.010, 0.20);
            const bool support_gap_degraded =
                cycle_contact < support_gap_fraction &&
                cycle_diagnostics_.max_consecutive_low_support_samples >=
                    support_gap_samples &&
                cycle_drift >= support_gap_drift;
            const bool degraded = health_eval &&
                (tracking_degraded || attitude_degraded || support_degraded ||
                 support_gap_degraded);
            const double gov_brake_joint = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_BRAKE_JOINT_ERROR", 1.00),
                0.70, 1.40);
            const double gov_brake_foot = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_BRAKE_FOOT_ERROR", 0.20),
                0.10, 0.40);
            const double gov_brake_contact = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_BRAKE_CONTACT", 0.12),
                0.05, 0.80);
            const int gov_brake_streak = static_cast<int>(std::clamp(
                Full2EnvDouble("TROT_HS_GOV_BRAKE_STREAK", 0.0),
                0.0, 100.0));
            const bool severe_degradation = health_eval &&
                ((cycle_diagnostics_.max_abs_joint_error_rad >
                      gov_brake_joint &&
                  cycle_foot > gov_brake_foot) ||
                 (cycle_diagnostics_.max_abs_joint_error_rad >
                      0.95 * gov_brake_joint &&
                  cycle_contact < gov_brake_contact));
            if (gov_brake_streak > 0 && severe_degradation)
                ++high_speed_health_severe_streak_;
            else
                high_speed_health_severe_streak_ = 0;
            if (gov_brake_streak > 0 &&
                high_speed_health_severe_streak_ >= gov_brake_streak &&
                !high_speed_stop_brake_active_ &&
                !high_speed_stop_hold_active_ && !task_.stop_requested_)
            {
                high_speed_stop_brake_active_ = true;
                high_speed_stop_brake_duration_s_ = std::clamp(
                    Full2EnvDouble(
                        "TROT_HS_GOV_HEALTH_BRAKE_DURATION_S", 1.20),
                    0.25, 2.00);
                const double health_brake_u = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_HEALTH_BRAKE_U", 0.35),
                    0.0, 0.90);
                high_speed_stop_brake_start_time_s_ =
                    running_time_ -
                    health_brake_u * high_speed_stop_brake_duration_s_;
                high_speed_stop_brake_base_speed_mps_ = std::max(
                    std::abs(v_meas), std::abs(wbc_speed_cmd_mps_));
                if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
                    high_speed_stop_brake_base_speed_mps_ =
                        std::abs(kernel_nominal_velocity_x_mps_);
                high_speed_stop_brake_base_period_s_ =
                    kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
                high_speed_stop_brake_base_duty_ =
                    kernel_duty_factor_ > 0.25 ? kernel_duty_factor_ : params_.duty_factor;
                std::cout << "HIGH-SPEED-HEALTH-GOV brake"
                          << " cycle=" << active_cycle_index_
                          << " v=" << v_meas
                          << " q=" << cycle_diagnostics_.max_abs_joint_error_rad
                          << " foot=" << cycle_foot
                          << " contact=" << cycle_contact << "\n";
            }
            const double gov_stop_angle = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_STOP_ANGLE_RAD", 0.12),
                0.05, 0.60);
            const double gov_stop_joint = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_STOP_JOINT_ERROR", 1.00),
                0.60, 1.80);
            const double gov_stop_foot = std::clamp(
                Full2EnvDouble("TROT_HS_GOV_STOP_FOOT_ERROR", 0.22),
                0.12, 0.60);
            const bool auto_brake =
                Full2EnvDouble("TROT_HS_GOV_AUTO_BRAKE", 0.0) > 0.5;
            const bool auto_brake_condition = health_eval &&
                (std::max(cycle_roll, cycle_pitch) > gov_stop_angle ||
                 cycle_diagnostics_.max_abs_joint_error_rad > gov_stop_joint ||
                 cycle_foot > gov_stop_foot ||
                 (cycle_contact < gov_brake_contact &&
                  cycle_drift > 0.70 * gov_drift_limit));
            if (auto_brake && auto_brake_condition &&
                !high_speed_stop_brake_active_ &&
                !high_speed_stop_hold_active_ && !task_.stop_requested_)
            {
                high_speed_stop_brake_active_ = true;
                high_speed_stop_brake_duration_s_ = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_BRAKE_DURATION_S", 2.00),
                    0.25, 2.00);
                high_speed_stop_brake_start_time_s_ = running_time_;
                high_speed_stop_brake_base_speed_mps_ = std::max(
                    std::abs(v_meas), std::abs(wbc_speed_cmd_mps_));
                if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
                    high_speed_stop_brake_base_speed_mps_ =
                        std::abs(kernel_nominal_velocity_x_mps_);
                high_speed_stop_brake_base_period_s_ =
                    kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
                high_speed_stop_brake_base_duty_ =
                    kernel_duty_factor_ > 0.25 ? kernel_duty_factor_ : params_.duty_factor;
                std::cout << "HIGH-SPEED-HEALTH-GOV auto-brake"
                          << " cycle=" << active_cycle_index_
                          << " angle=" << std::max(cycle_roll, cycle_pitch)
                          << " q=" << cycle_diagnostics_.max_abs_joint_error_rad
                          << " foot=" << cycle_foot
                          << " contact=" << cycle_contact << "\n";
            }
            const bool critical_degradation =
                Full2EnvDouble("TROT_HS_GOV_CRITICAL", 0.0) > 0.5 &&
                (std::max(cycle_roll, cycle_pitch) > gov_stop_angle ||
                 cycle_diagnostics_.max_abs_joint_error_rad > gov_stop_joint ||
                 cycle_foot > gov_stop_foot);
            if (critical_degradation)
            {
                task_.stop_requested_ = true;
                task_.motion_stage_ = 3;
                task_.task_completion_requested_ = false;
                high_speed_stop_hold_active_ = true;
                high_speed_stop_hold_start_time_s_ = running_time_;
                high_speed_stop_hold_targets_ = joint_targets;
                have_high_speed_stop_hold_targets_ = true;
                std::cout << "HIGH-SPEED-HEALTH-GOV critical; WBC hold"
                          << " cycle=" << active_cycle_index_
                          << " angle=" << std::max(cycle_roll, cycle_pitch)
                          << " q=" << cycle_diagnostics_.max_abs_joint_error_rad
                          << " foot=" << cycle_foot << "\n";
            }
            const double gov_min_cap = std::min(
                target_speed,
                std::clamp(Full2EnvDouble("TROT_HS_GOV_MIN_CAP", 2.50),
                           1.20, 3.50));
            const int gov_hold_cycles = static_cast<int>(std::clamp(
                Full2EnvDouble("TROT_HS_GOV_HOLD_CYCLES", 10.0),
                1.0, 100.0));
            // A low-support cycle with a clearly diverging attitude is no
            // longer a soft quality fluctuation: waiting for another 0.2 m/s
            // cap step leaves the body enough momentum to cross the hard
            // posture limit before the gait shape changes.  Drop directly to
            // the configured safe cap, but keep the same WBC/MPC plant.
            const bool emergency_support_degraded =
                health_eval &&
                ((attitude_degraded && cycle_contact < gov_contact_limit) ||
                 support_gap_degraded);
            if (high_speed_health_hold_cycles_ > 0)
                --high_speed_health_hold_cycles_;
            if (degraded && high_speed_health_hold_cycles_ == 0)
            {
                const double degrade_step = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_DEGRADE_STEP", 0.20),
                    0.020, 0.50);
                if (emergency_support_degraded)
                {
                    high_speed_health_cap_mps_ = gov_min_cap;
                    high_speed_health_hold_cycles_ = std::max(
                        gov_hold_cycles,
                        static_cast<int>(std::clamp(
                            Full2EnvDouble(
                                "TROT_HS_GOV_EMERGENCY_HOLD_CYCLES", 25.0),
                            5.0, 100.0)));
                    std::cout << "HIGH-SPEED-HEALTH-GOV emergency support cap"
                              << " cycle=" << active_cycle_index_
                              << " cap=" << high_speed_health_cap_mps_
                              << " angle="
                              << std::max(cycle_roll, cycle_pitch)
                              << " contact=" << cycle_contact << "\n";
                }
                else
                {
                    high_speed_health_cap_mps_ = std::max(
                        gov_min_cap, high_speed_health_cap_mps_ - degrade_step);
                    high_speed_health_hold_cycles_ = gov_hold_cycles;
                }
                std::cout << "HIGH-SPEED-HEALTH-GOV degraded cycle="
                          << active_cycle_index_
                          << " cap=" << high_speed_health_cap_mps_
                          << " q=" << cycle_diagnostics_.max_abs_joint_error_rad
                          << " foot=" << cycle_foot
                          << " drift=" << cycle_drift << "\n";
            }
            if (!degraded && high_speed_health_hold_cycles_ == 0)
            {
                const double recover_step = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_RECOVER_STEP", 0.008),
                    0.0, 0.10);
                high_speed_health_cap_mps_ = std::min(
                    target_speed, high_speed_health_cap_mps_ + recover_step);
            }
        }
        cartesian_last_foot_error_m_ = cycle_foot;
        active_cycle_index_ = cycle_index;
        ResetCycleDiagnostics();
        if (!params_.runtime_velocity_command && params_.cartesian_world)
        {
            if (wbc_speed_cmd_mps_ < 0.0)
            {
                wbc_speed_cmd_mps_ = std::abs(
                    params_.direction_sign * params_.step_length_m /
                    params_.period_s);
                locomotion_kernel_->SetGaitSlewLimits(0.12, 0.20, 0.20);
            }
            const double pitch =
                std::abs(static_cast<double>(
                    state_snapshot.imu_state().rpy()[1]));
            const double roll =
                std::abs(static_cast<double>(
                    state_snapshot.imu_state().rpy()[0]));
            constexpr double kDeg = go2_trot::kPi / 180.0;
            // Keep peak at c124 roll=8.9°: attitude_ok is 5°, then slam
            // at 8/7 dumps cmd to v_meas+0.04 and last-8s sags. Unset=5.
            // ATT=10 from cycle 0 sat (N6EA50ATT10). Gate with ATT_V.
            double att = 5.0;
            const double att_ov = Full2EnvDouble("FULL2_ATT", -1.0);
            const double att_v = Full2EnvDouble("FULL2_ATT_V", 0.0);
            if (att_ov > 0.0 && (att_v <= 0.0 || v_meas >= att_v))
                att = att_ov;
            const bool attitude_ok = pitch < att * kDeg && roll < att * kDeg;
            const int hold_from = static_cast<int>(
                Full2EnvDouble("FULL2_HOLD_CYCLES", 8.0));
            if (cycle_index >= hold_from)
            {
                if (attitude_ok)
                {
                    double inc = v_meas >= 0.40 ? 0.06 : 0.03;
                    const double inc_override =
                        Full2EnvDouble("FULL2_INC", -1.0);
                    if (inc_override >= 0.0)
                        inc = inc_override;
                    double lead =
                        (pitch < 2.5 * kDeg && roll < 2.5 * kDeg &&
                         v_meas >= 0.40)
                            ? 0.20
                            : 0.12;
                    const double lead_override =
                        Full2EnvDouble("FULL2_LEAD", -1.0);
                    if (lead_override > 0.0)
                        lead = lead_override;
                    const double cmd_floor = wbc_speed_cmd_mps_;
                    // X3 c44→c50: roll/pitch <2.5° while v_meas 0.55→0.04
                    // because lead follows a one-cycle dip. Hold cmd (and
                    // gait schedule) across dips >0.08 m/s; still follow a
                    // real slowdown that steps down gradually.
                    const bool stumble_hold =
                        Full2EnvDouble("FULL2_STUMBLE", 0.0) > 0.5 &&
                        cartesian_last_v_meas_ >= 0.0 &&
                        cartesian_last_v_meas_ - v_meas > 0.08;
                    wbc_speed_cmd_mps_ = std::min(
                        2.45, wbc_speed_cmd_mps_ + inc);
                    double follow = std::max(0.15, v_meas + lead);
                    if (stumble_hold)
                        follow = std::max(follow, cmd_floor);
                    wbc_speed_cmd_mps_ = std::min(
                        wbc_speed_cmd_mps_, follow);
                    const double foot_hold =
                        Full2EnvDouble("FULL2_FOOT_HOLD", 0.0);
                    const double foot_v =
                        Full2EnvDouble("FULL2_FOOT_V", 0.0);
                    if (foot_hold > 0.0 && cycle_foot > foot_hold &&
                        (foot_v <= 0.0 || v_meas >= foot_v))
                        wbc_speed_cmd_mps_ = cmd_floor;
                    if (Full2EnvDouble("FULL2_RATCHET", 0.0) > 0.5)
                    {
                        // 233312 last-8s 0.686 was still climbing at 280.
                        // G2 completes 400 but cmd follows v_meas down
                        // after a 0.70 peak (212138 c200→c280).
                        wbc_speed_cmd_mps_ = std::max(
                            cmd_floor, wbc_speed_cmd_mps_);
                    }
                    const double hold_floor =
                        Full2EnvDouble("FULL2_CMD_FLOOR", 0.0);
                    if (hold_floor > 0.0 && cmd_floor >= hold_floor)
                        wbc_speed_cmd_mps_ = std::max(
                            hold_floor, wbc_speed_cmd_mps_);
                }
                else if (pitch > (att + 3.0) * kDeg ||
                         roll > (att + 2.0) * kDeg)
                {
                    wbc_speed_cmd_mps_ = std::max(
                        0.15, std::min(wbc_speed_cmd_mps_, v_meas + 0.04));
                }
            }
            wbc_speed_cmd_mps_ = Clamp(wbc_speed_cmd_mps_, 0.15, 2.45);
            const double sched_lead = Full2EnvDouble("FULL2_SCHED_LEAD", 0.12);
            double v_sched = std::min(
                wbc_speed_cmd_mps_,
                std::max(0.15, v_meas + sched_lead));
            if (Full2EnvDouble("FULL2_STUMBLE", 0.0) > 0.5 &&
                cartesian_last_v_meas_ >= 0.0 &&
                cartesian_last_v_meas_ - v_meas > 0.08)
                v_sched = wbc_speed_cmd_mps_;
            if (cycle_index >= 12)
                v_sched = std::max(0.32, v_sched);
            cartesian_last_v_meas_ = v_meas;
            const auto sched0 = Full2UseRunGait()
                ? go2_control::ScheduleCheetahTrotEarly(v_sched)
                : Full2UseCheetahTable()
                    ? go2_control::ScheduleCheetahTrot(v_sched)
                    : go2_control::ScheduleRunningTrot(v_sched);
            auto sched = sched0;
            // Unset Early span is 0.85, so t=1 at v=1.00 and keep is already
            // at duty 0.40 by cycle 29 (v_meas~0.90). Span 1.85 → t=1 at 2.00.
            const double early_span = Full2EnvDouble("FULL2_EARLY_SPAN", 0.0);
            if (Full2UseRunGait() && early_span > 0.0)
            {
                const double v = std::clamp(v_sched, 0.10, 2.60);
                const double t = std::clamp((v - 0.15) / early_span, 0.0, 1.0);
                sched.duty_factor = 0.50 + t * (0.40 - 0.50);
                sched.period_s = 0.30 + t * (0.26 - 0.30);
                sched.stance_time_s = sched.duty_factor * sched.period_s;
                sched.step_length_m = v * sched.period_s;
                sched.foot_lift_m = 0.040 + t * 0.012;
            }
            // Early saturates at v=1.00 (duty 0.40 / T=0.26). Keep then
            // lengthens step at a frozen gait, tau-sats, and rolls at ~1.04.
            // FAST_SPAN continues after FAST_FROM (unset FROM=1.00). Unset
            // SPAN = freeze. FAST1 never crossed 1.00 so SPAN never fired;
            // FROM=0.85 starts the second lerp during the keep climb.
            const double fast_span = Full2EnvDouble("FULL2_FAST_SPAN", 0.0);
            const double fast_from = Full2EnvDouble("FULL2_FAST_FROM", 1.00);
            if (Full2UseRunGait() && fast_span > 0.0 && fast_from > 0.0 &&
                v_sched > fast_from)
            {
                const double v = std::clamp(v_sched, fast_from, 2.60);
                const double t2 = std::clamp((v - fast_from) / fast_span, 0.0, 1.0);
                const double t0 = std::clamp((fast_from - 0.15) / 0.85, 0.0, 1.0);
                const double duty0 = 0.50 + t0 * (0.40 - 0.50);
                const double period0 = 0.30 + t0 * (0.26 - 0.30);
                sched.duty_factor = duty0 + t2 * (0.36 - duty0);
                sched.period_s = period0 + t2 * (0.22 - period0);
                sched.stance_time_s = sched.duty_factor * sched.period_s;
                sched.step_length_m = v * sched.period_s;
                sched.foot_lift_m = 0.052 + t2 * 0.008;
            }
            const double duty_floor = Full2EnvDouble("FULL2_DUTY_FLOOR", 0.0);
            const double duty_v = Full2EnvDouble("FULL2_DUTY_V", 0.0);
            if (duty_floor > 0.0 && sched.duty_factor < duty_floor &&
                (duty_v <= 0.0 || v_sched >= duty_v))
            {
                sched.duty_factor = duty_floor;
                sched.stance_time_s = duty_floor * sched.period_s;
            }
            const double period_floor = Full2EnvDouble("FULL2_PERIOD_FLOOR", 0.0);
            if (period_floor > 0.0 && sched.period_s < period_floor)
            {
                sched.period_s = period_floor;
                sched.stance_time_s = sched.duty_factor * sched.period_s;
            }
            double period_s = sched.period_s;
            double duty = sched.duty_factor;
            double tst = sched.stance_time_s;
            double step_m = wbc_speed_cmd_mps_ * sched.period_s;
            if (Full2UseSlew())
            {
                const double slew = 0.008;
                if (cartesian_stance_s_ > sched.stance_time_s)
                    cartesian_stance_s_ = std::max(
                        sched.stance_time_s, cartesian_stance_s_ - slew);
                else
                    cartesian_stance_s_ = std::min(
                        sched.stance_time_s, cartesian_stance_s_ + slew);
                duty = sched.duty_factor;
                period_s = std::clamp(
                    cartesian_stance_s_ / std::max(0.35, duty),
                    0.22, 0.60);
                tst = duty * period_s;
                cartesian_stance_s_ = tst;
                step_m = wbc_speed_cmd_mps_ * period_s;
            }
            else
                cartesian_stance_s_ = tst;
            locomotion_kernel_->SetGaitPeriod(period_s);
            locomotion_kernel_->SetGaitDuty(duty);
            locomotion_kernel_->SetGaitStepLength(step_m);
            double lift = sched.foot_lift_m;
            const double lift_ov = Full2EnvDouble("FULL2_LIFT", -1.0);
            if (lift_ov > 0.0)
                lift = lift_ov;
            locomotion_kernel_->SetGaitFootLift(lift);
            const double v_cap = go2_control::CartesianWorkspaceSpeedCap(
                kernel_duty_factor_ > 0.05 ? kernel_duty_factor_
                                           : params_.duty_factor,
                kernel_period_s_ > 0.05 ? kernel_period_s_
                                        : params_.period_s);
            if (wbc_speed_cmd_mps_ > v_cap)
                wbc_speed_cmd_mps_ = v_cap;
            std::cout << "CART-GOV cycle=" << cycle_index
                      << " v_cmd=" << wbc_speed_cmd_mps_
                      << " v_meas=" << v_meas
                      << " v_cap=" << v_cap
                      << " period=" << period_s
                      << " duty=" << duty
                      << " Tst=" << tst
                      << " step=" << step_m
                      << " motor_kp=" << low_cmd_.motor_cmd()[1].kp()
                      << "\n";
        }
        else if (!params_.runtime_velocity_command && params_.wbc_full && high_speed_curriculum)
        {
            // High-speed bound/gallop uses a deliberately slow curriculum.
            // Starting directly at the terminal step length asks the first
            // swing to move faster than the leg and makes the MPC plant fall
            // forward before the body has acquired momentum.
            double target_period = params_.period_s;
            double target_duty = params_.duty_factor;
            const double requested_target_speed = std::abs(
                2.0 * target_duty * params_.direction_sign *
                params_.step_length_m /
                std::max(1.0e-3, target_period));
            const double target_speed_override = Full2EnvDouble(
                "TROT_HS_TARGET_SPEED", -1.0);
            const double target_speed = target_speed_override >= 0.0
                ? std::clamp(target_speed_override, 0.20, 3.50)
                : requested_target_speed;
            // Start from the already-validated running-trot plant.  The
            // previous 0.40 s / 0.60 duty seed was a walk-like gait and
            // injected a second, unvalidated transition before acceleration.
            const double kStartPeriod = std::clamp(
                Full2EnvDouble("TROT_HS_START_PERIOD", 0.26),
                0.08, 1.0);
            const double kStartDuty = std::clamp(
                Full2EnvDouble("TROT_HS_START_DUTY", 0.45),
                0.25, 0.90);
            constexpr double kStartSpeed = 0.90;
            if (wbc_speed_cmd_mps_ < 0.0)
            {
                wbc_speed_cmd_mps_ = std::min(target_speed, kStartSpeed);
                const double hs_step_slew = std::clamp(Full2EnvDouble("TROT_HS_STEP_SLEW", 0.012), 0.008, 0.10);
                locomotion_kernel_->SetGaitSlewLimits(hs_step_slew, 0.020, 0.020);
            }
            const double v_meas_abs = have_filtered_body_velocity_
                ? std::max(0.0, params_.direction_sign *
                                   latest_filtered_body_velocity_[0])
                : 0.0;
            const double ramp_step_override = Full2EnvDouble("TROT_HS_RAMP_STEP", -1.0);
            const double ramp_step = ramp_step_override >= 0.0 ? std::clamp(ramp_step_override, 0.010, 0.250) : (target_speed > 2.0 ? 0.060 : 0.045);
            wbc_speed_cmd_mps_ = std::min(
                target_speed, wbc_speed_cmd_mps_ + ramp_step);
            // Keep the commanded reference close to the realized speed for
            // the entire curriculum.  Letting it outrun the body after the
            // first ten cycles creates a force/foothold demand the two-leg
            // support pair cannot sustain.
            const double speed_lead_override = Full2EnvDouble("TROT_HS_SPEED_LEAD", -1.0);
            const double speed_lead = speed_lead_override >= 0.0 ? std::clamp(speed_lead_override, 0.0, 3.0) : (target_speed > 2.0 ? 0.38 : 0.30);
            wbc_speed_cmd_mps_ = std::max(
                kStartSpeed,
                std::min(wbc_speed_cmd_mps_, v_meas_abs + speed_lead));
            if (high_speed_health_governor &&
                high_speed_health_cap_mps_ > 0.0)
                wbc_speed_cmd_mps_ = std::min(
                    wbc_speed_cmd_mps_, high_speed_health_cap_mps_);
            // When health feedback lowers the speed cap, change the gait
            // shape at the same time. Keeping the terminal low-duty pattern
            // while asking for a much lower speed leaves too little support
            // time to recover from a degraded cycle. This remains the same
            // WBC/MPC plant; only its continuous reference is changed.
            if (high_speed_health_governor &&
                high_speed_health_cap_mps_ > 0.0 &&
                high_speed_health_cap_mps_ < target_speed)
            {
                const double guard_start = std::clamp(
                    Full2EnvDouble("TROT_HS_GUARD_START_SPEED", 3.0),
                    1.20, 3.50);
                const double guard_period = std::clamp(
                    Full2EnvDouble("TROT_HS_GUARD_PERIOD", 0.24),
                    0.12, 0.60);
                const double guard_duty = std::clamp(
                    Full2EnvDouble("TROT_HS_GUARD_DUTY", 0.52),
                    0.35, 0.85);
                const double guard_alpha = std::clamp(
                    (high_speed_health_cap_mps_ - kStartSpeed) /
                        std::max(0.10, guard_start - kStartSpeed),
                    0.0, 1.0);
                target_period = guard_period + guard_alpha *
                    (target_period - guard_period);
                target_duty = guard_duty + guard_alpha *
                    (target_duty - guard_duty);
                std::cout << "HIGH-SPEED-HEALTH-GOV shape cap="
                          << high_speed_health_cap_mps_
                          << " period_target=" << target_period
                          << " duty_target=" << target_duty << "\n";
            }
            const double alpha = target_speed > kStartSpeed
                ? std::clamp(
                      (wbc_speed_cmd_mps_ - kStartSpeed) /
                          (target_speed - kStartSpeed),
                      0.0, 1.0)
                : 1.0;
            const double period_alpha = std::clamp(
                (wbc_speed_cmd_mps_ - kStartSpeed) / 1.0, 0.0, 1.0);
            const double duty_alpha = std::clamp(
                (wbc_speed_cmd_mps_ - kStartSpeed) / 0.80, 0.0, 1.0);
            const double period = kStartPeriod +
                period_alpha * (target_period - kStartPeriod);
            const double duty = kStartDuty +
                duty_alpha * (target_duty - kStartDuty);
            // The kernel advances the body twice per gait period (one
            // support pair at each half-cycle).  Treat the CLI step as a
            // per-leg stance stroke, so v = 2*duty*step/period.  The old
            // step=v*period made the MPC velocity reference outrun the
            // actual foot stroke by 1/(2*duty), which is fatal in sprint
            // mode and also mislabeled the requested speed.
            double step = wbc_speed_cmd_mps_ * period /
                std::max(0.20, 2.0 * duty);
            const double step_cap = Full2EnvDouble(
                "TROT_HS_STEP_CAP", -1.0);
            if (step_cap > 0.0)
                step = std::min(step, std::clamp(step_cap, 0.08, 1.20));
            const double target_lift = std::max(params_.foot_lift_m, 0.08);
            const double lift = 0.08 + alpha * (target_lift - 0.08);
            locomotion_kernel_->SetGaitPeriod(period);
            locomotion_kernel_->SetGaitDuty(duty);
            locomotion_kernel_->SetGaitStepLength(step);
            locomotion_kernel_->SetGaitFootLift(lift);
            std::cout << "HIGH-SPEED-GOV pattern="
                      << go2_control::GaitPatternName(params_.gait_pattern)
                      << " cycle=" << cycle_index
                      << " v_cmd=" << wbc_speed_cmd_mps_
                      << " v_meas=" << v_meas_abs
                      << " period=" << period
                      << " duty=" << duty
                      << " step=" << step << "\n";
        }
        else if (params_.wbc_full && !params_.step_plan.empty())
        {
            if (wbc_speed_cmd_mps_ < 0.0)
            {
                wbc_speed_cmd_mps_ = std::abs(
                    params_.direction_sign * params_.step_length_m /
                    params_.period_s);
            }
            const double pitch =
                std::abs(static_cast<double>(
                    state_snapshot.imu_state().rpy()[1]));
            const double roll =
                std::abs(static_cast<double>(
                    state_snapshot.imu_state().rpy()[0]));
            constexpr double kDeg = go2_trot::kPi / 180.0;
            const double period = cycle_index >= 12 ? 0.34 : 0.52;
            double step = std::clamp((v_meas + 0.10) * period, 0.091, 0.52);
            if (v_meas >= 0.90 * (step / period) &&
                pitch < 6.0 * kDeg && roll < 6.0 * kDeg)
                step = std::min(0.52, step + 0.012);
            if (pitch > 10.0 * kDeg || roll > 10.0 * kDeg)
                step = std::max(0.091, step - 0.010);
            wbc_speed_cmd_mps_ = step / period;
            locomotion_kernel_->SetGaitPeriod(period);
            locomotion_kernel_->SetGaitStepLength(step);
            std::cout << "SPEED-GOV cycle=" << cycle_index
                      << " v_cmd=" << wbc_speed_cmd_mps_
                      << " v_meas=" << v_meas
                      << " period=" << period
                      << " step=" << step << "\n";
        }
        else
        {
            for (const auto &plan : params_.step_plan)
            {
                if (cycle_index == plan.first)
                {
                    locomotion_kernel_->SetGaitStepLength(plan.second);
                    std::cout << "STEP-PLAN cycle=" << cycle_index
                              << " step=" << plan.second << "\n";
                }
            }
            for (const auto &plan : params_.period_plan)
            {
                if (cycle_index == plan.first)
                {
                    locomotion_kernel_->SetGaitPeriod(plan.second);
                    std::cout << "PERIOD-PLAN cycle=" << cycle_index
                              << " period=" << plan.second << "\n";
                }
            }
        }
        if (high_speed_stop_brake_active_)
        {
            const double brake_elapsed =
                running_time_ - high_speed_stop_brake_start_time_s_;
            const double brake_duration = std::clamp(
                high_speed_stop_brake_duration_s_, 0.25, 2.00);
            const double brake_u = std::clamp(
                brake_elapsed / brake_duration, 0.0, 1.0);
            const double smooth_u = Smoothstep(brake_u);
            const double speed =
                high_speed_stop_brake_base_speed_mps_ * (1.0 - smooth_u);
            const double period = std::clamp(
                high_speed_stop_brake_base_period_s_, 0.16, 0.60);
            const double duty = std::clamp(
                high_speed_stop_brake_base_duty_ +
                    smooth_u * (0.90 - high_speed_stop_brake_base_duty_),
                0.40, 0.90);
            const double step = std::max(
                0.004, speed * period / std::max(0.20, 2.0 * duty));
            wbc_speed_cmd_mps_ = speed;
            kernel_nominal_velocity_x_mps_ =
                params_.direction_sign * speed;
            kernel_period_s_ = period;
            kernel_duty_factor_ = duty;
            // The normal curriculum slew (0.02 duty per cycle) is meant for
            // gentle gait morphing.  During a sprint brake it would leave
            // the controller on a two-contact schedule for most of the
            // 2-second brake, so raise the bounded slew just for this
            // deceleration phase.  This is still continuous, but reaches a
            // support-rich duty before the body loses its speed.
            locomotion_kernel_->SetGaitSlewLimits(
                std::clamp(Full2EnvDouble("TROT_HS_BRAKE_STEP_SLEW", 0.20),
                           0.08, 0.40),
                std::clamp(Full2EnvDouble("TROT_HS_BRAKE_PERIOD_SLEW", 0.10),
                           0.04, 0.30),
                std::clamp(Full2EnvDouble("TROT_HS_BRAKE_DUTY_SLEW", 0.20),
                           0.08, 0.40));
            locomotion_kernel_->SetGaitPeriod(period);
            locomotion_kernel_->SetGaitDuty(duty);
            locomotion_kernel_->SetGaitStepLength(step);
            locomotion_kernel_->SetGaitFootLift(
                std::max(params_.foot_lift_m, 0.08));
            const double measured_speed = have_world_velocity_
                ? std::abs(params_.direction_sign * latest_world_velocity_[0])
                : std::numeric_limits<double>::infinity();
            constexpr double kBrakeReadyAngleRad = 10.0 * kPi / 180.0;
            const double measured_roll = std::abs(static_cast<double>(
                state_snapshot.imu_state().rpy()[0]));
            const double measured_pitch = std::abs(static_cast<double>(
                state_snapshot.imu_state().rpy()[1]));
            const bool brake_ready =
                brake_elapsed >= brake_duration &&
                measured_speed <= 0.55 &&
                measured_roll <= kBrakeReadyAngleRad &&
                measured_pitch <= kBrakeReadyAngleRad;
            if (brake_ready)
            {
                high_speed_stop_brake_active_ = false;
                high_speed_stop_hold_active_ = true;
                high_speed_stop_hold_start_time_s_ = running_time_;
                high_speed_stop_hold_targets_ = joint_targets;
                have_high_speed_stop_hold_targets_ = true;
                task_.stop_requested_ = true;
                task_.motion_stage_ = 3;
                task_.task_completion_requested_ = false;
                std::cout << "High-speed stop: brake complete;"
                          << " entering WBC four-contact hold\n";
            }
        }
        std::cout << "Trot cycle " << cycle_index << " started"
                  << " v_cmd=" << gait_result.nominal_velocity_x_mps
                  << " v_meas="
                  << (have_filtered_body_velocity_
                          ? latest_filtered_body_velocity_[0]
                          : 0.0)
                  << " period=" << gait_result.period_s
                  << " step=" << gait_result.step_length_m
                  << " duty=" << gait_result.duty_factor;
        if (task_.goal_enabled_ && have_high_state)
        {
            const WorldPose goal_pose =
                ComputeWorldPose(state_snapshot, high_state_snapshot);
            std::cout << " pose=(" << goal_pose.base.x << ","
                      << goal_pose.base.y << "," << goal_pose.base.z
                      << ") remaining="
                      << task_.RemainingXy(goal_pose.base.x, goal_pose.base.y);
        }
        std::cout << "\n";
    }

    feet = gait_result.feet;
    const double stance_duration =
        gait_result.duty_factor > 0.05 ? gait_result.duty_factor
                                       : params_.duty_factor;

    WorldPose pose{};
    std::array<go2::Vec3, go2::kLegCount> actual_world_feet{};
    bool have_actual_world_feet = false;
    if (have_high_state)
    {
        pose = ComputeWorldPose(state_snapshot, high_state_snapshot);
        actual_world_feet = ComputeWorldFeet(state_snapshot, pose);
        have_actual_world_feet = true;
    }
    if (params_.cartesian_world && have_actual_world_feet)
    {
        go2_control::CartesianWorldInput cart;
        cart.base = pose.base;
        cart.quaternion = pose.quaternion;
        cart.actual_world_feet = actual_world_feet;
        cart.stand_body_feet = go2::AllFootPositions(task_.stand_up_joint_pos_);
        cart.phase = phase;
        cart.duty_factor = stance_duration;
        cart.period_s =
            gait_result.period_s > 0.05 ? gait_result.period_s
                                        : params_.period_s;
        cart.v_cmd_mps =
            wbc_speed_cmd_mps_ > 0.0
                ? wbc_speed_cmd_mps_
                : std::abs(gait_result.nominal_velocity_x_mps);
        if (have_filtered_body_velocity_)
        {
            const double c = std::cos(pose.yaw_rad);
            const double s = std::sin(pose.yaw_rad);
            cart.vx_world =
                latest_filtered_body_velocity_[0] * c -
                latest_filtered_body_velocity_[1] * s;
            cart.vy_world =
                latest_filtered_body_velocity_[0] * s +
                latest_filtered_body_velocity_[1] * c;
        }
        else if (have_world_velocity_)
        {
            cart.vx_world = latest_world_velocity_[0];
            cart.vy_world = latest_world_velocity_[1];
        }
        cart.yaw_rad = pose.yaw_rad;
        cart.roll_rad = static_cast<double>(state_snapshot.imu_state().rpy()[0]);
        cart.gyro_x = static_cast<double>(
            state_snapshot.imu_state().gyroscope()[0]);
        cart.raibert_gain_s = 0.03;
        const double raibert_override = Full2EnvDouble("FULL2_RAIBERT", -1.0);
        if (raibert_override > 0.0)
            cart.raibert_gain_s = raibert_override;
        cart.raibert_gain_y = cart.raibert_gain_s;
        const double raibert_y = Full2EnvDouble("FULL2_RAIBERT_Y", -1.0);
        if (raibert_y > 0.0)
            cart.raibert_gain_y = raibert_y;
        const double raibert_x = Full2EnvDouble("FULL2_RAIBERT_X", -1.0);
        if (raibert_x > 0.0)
            cart.raibert_gain_s = raibert_x;
        cart.raibert_max_adj_m = params_.raibert_max_adjustment_m;
        cart.ypos_gain = Full2EnvDouble("FULL2_YPOS", 0.0);
        cart.world_heading = Full2EnvDouble("FULL2_WORLD_HEADING", 0.0) > 0.5;
        cart.yaw_gain = Full2EnvDouble("FULL2_YAW_GAIN", 0.0);
        cart.pattern = params_.gait_pattern;
        cart.foot_lift_m =
            gait_result.duty_factor > 0.0
                ? std::max(params_.foot_lift_m, 0.022)
                : params_.foot_lift_m;
        const double lift_ov = Full2EnvDouble("FULL2_LIFT", -1.0);
        if (lift_ov > 0.0)
            cart.foot_lift_m = lift_ov;
        cart.blend = Smoothstep(gait_time_s / 0.60);
        // 223300: Cheetah vWorld placement at high kp marched.
        // 085348 rolled with extra ax + MPC chasing v_cmd.
        // 104319/105123: sagittal vWorld after paper kp, Y stays mixed.
        // 105455 mix + MPC lead 0.35 rolled 180; 105123 X-only completed 400.
        cart.measured_placement =
            cartesian_kp_frozen_ && cartesian_latched_kp_ <= 5.0;
        // 230456: Cheetah duty 0.49 / period 0.30 still held v_meas 0.31.
        // Missing ConvexMPCLocomotion hip-at-TD lead (vWorld * t_sw).
        // 233312 freeze turned this on; G2 freeze is off so the foot
        // plants at current hip and brakes. One-factor: FULL2_HIP_TD=1.
        cart.predict_hip_at_td =
            cartesian_kp_frozen_ ||
            Full2EnvDouble("FULL2_HIP_TD", 0.0) > 0.5;
        go2_control::ApplyCartesianWorldTrot(cart, cartesian_state_, feet);
        commanded_world_feet_ = cartesian_state_.target_world;
        have_commanded_world_feet_ = true;
        if (wbc_speed_cmd_mps_ < 0.0)
        {
            wbc_speed_cmd_mps_ = cart.v_cmd_mps;
            locomotion_kernel_->SetGaitSlewLimits(0.12, 0.20, 0.20);
        }
    }
    if (params_.world_feedback && have_world_reference_ && have_high_state &&
        !params_.cartesian_world)
    {
        pose = ComputeWorldPose(state_snapshot, high_state_snapshot);
        double path_yaw = world_reference_yaw_rad_;
        if (task_.goal_enabled_ && !task_.reached_goal_)
        {
            path_yaw = std::atan2(
                task_.goal_y_ - world_reference_y_m_,
                task_.goal_x_ - world_reference_x_m_);
        }
        const double ref_cos = std::cos(path_yaw);
        const double ref_sin = std::sin(path_yaw);
        const double actual_x =
            ref_cos * (pose.base.x - world_reference_x_m_) +
            ref_sin * (pose.base.y - world_reference_y_m_);
        const double actual_y =
            -ref_sin * (pose.base.x - world_reference_x_m_) +
            ref_cos * (pose.base.y - world_reference_y_m_);
        const double cruise_v =
            gait_result.nominal_velocity_x_mps > 0.0
                ? gait_result.nominal_velocity_x_mps
                : params_.direction_sign * params_.step_length_m /
                      params_.period_s;
        const double v_scale = task_.goal_enabled_
            ? task_.CommandedStepScale(pose.base.x, pose.base.y)
            : 1.0;
        double target_x = cruise_v * v_scale * gait_time_s;
        if (task_.goal_enabled_)
        {
            const double path_length_m = task_.RemainingXy(
                world_reference_x_m_, world_reference_y_m_);
            target_x = std::min(target_x, std::max(0.0, path_length_m));
        }
        const double target_y = 0.0;
        const double error_x = actual_x - target_x;
        const double error_y = actual_y - target_y;
        const double requested_x = Clamp(
            -params_.world_feedback_gain * error_x,
            -params_.world_feedback_max_m,
            params_.world_feedback_max_m);
        const double requested_y = Clamp(
            -params_.world_feedback_gain * error_y,
            -params_.world_feedback_max_m,
            params_.world_feedback_max_m);
        world_feedback_x_m_ += Clamp(
            requested_x - world_feedback_x_m_,
            -params_.world_feedback_slew_m,
            params_.world_feedback_slew_m);
        world_feedback_y_m_ += Clamp(
            requested_y - world_feedback_y_m_,
            -params_.world_feedback_slew_m,
            params_.world_feedback_slew_m);
        const double heading_ref =
            task_.goal_enabled_ && !task_.reached_goal_
                ? task_.DesiredHeading(pose.base.x, pose.base.y)
                : world_reference_yaw_rad_;
        world_yaw_error_rad_ = WrapAngle(pose.yaw_rad - heading_ref);
        const bool turn_active =
            motion_event_response_enabled_
                ? std::abs(motion_reference_.yaw_rate_radps) > 1.0e-4
                : (task_.goal_enabled_
                       ? task_.TurnEnable(running_time_) > 0.0 &&
                             std::abs(task_.commanded_turn_rate_radps_) > 1.0e-4
                       : params_.turn_rate_radps != 0.0);
        const double yaw_trim = turn_active
            ? 0.0
            : Clamp(
                  kYawFeedbackGain * world_yaw_error_rad_,
                  -kYawFeedbackMaxRad,
                  kYawFeedbackMaxRad);
        const double cos_trim = std::cos(yaw_trim);
        const double sin_trim = std::sin(yaw_trim);
        for (auto &foot : feet)
        {
            foot.x -= world_feedback_x_m_;
            foot.y -= world_feedback_y_m_;
            const double x = foot.x;
            const double y = foot.y;
            foot.x = cos_trim * x - sin_trim * y;
            foot.y = sin_trim * x + cos_trim * y;
        }
    }

    if (params_.support_anchor_feedback && have_actual_world_feet &&
        !params_.cartesian_world)
    {
        const std::array<double, 4> inverse_quaternion = {
            pose.quaternion[0],
            -pose.quaternion[1],
            -pose.quaternion[2],
            -pose.quaternion[3]};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const double leg_phase = go2_control::GaitLegPhase(
                leg, phase, params_.gait_pattern);
            const bool support = leg_phase < stance_duration;
            if (!support)
            {
                support_anchor_valid_[leg] = false;
                continue;
            }
            if (!support_anchor_valid_[leg])
            {
                support_anchor_world_feet_[leg] = actual_world_feet[leg];
                support_anchor_valid_[leg] = true;
                support_anchor_start_time_s_[leg] = gait_time_s;
            }
            const go2::Vec3 delta = {
                support_anchor_world_feet_[leg].x - pose.base.x,
                support_anchor_world_feet_[leg].y - pose.base.y,
                support_anchor_world_feet_[leg].z - pose.base.z};
            const go2::Vec3 base_anchor =
                RotateByQuaternion(inverse_quaternion, delta);
            const double anchor_blend =
                params_.support_anchor_gain * Smoothstep(
                (gait_time_s - support_anchor_start_time_s_[leg]) /
                kSupportAnchorBlendDuration);
            feet[leg].x =
                (1.0 - anchor_blend) * feet[leg].x +
                anchor_blend * base_anchor.x;
            feet[leg].y =
                (1.0 - anchor_blend) * feet[leg].y +
                anchor_blend * base_anchor.y;
        }
    }
    if (params_.attitude_feedback && !params_.cartesian_world)
    {
        const double imu_roll = state_snapshot.imu_state().rpy()[0];
        const double imu_pitch = state_snapshot.imu_state().rpy()[1];
        attitude_roll_rad_ +=
            kAttitudeFilterAlpha * (imu_roll - attitude_roll_rad_);
        attitude_pitch_rad_ +=
            kAttitudeFilterAlpha * (imu_pitch - attitude_pitch_rad_);
        attitude_feedback_x_m_ = Clamp(
            -kAttitudeFeedbackGainMPerRad * attitude_pitch_rad_,
            -kAttitudeFeedbackMaxM,
            kAttitudeFeedbackMaxM);
        attitude_feedback_y_m_ = Clamp(
            kAttitudeFeedbackGainMPerRad * attitude_roll_rad_,
            -kAttitudeFeedbackMaxM,
            kAttitudeFeedbackMaxM);
        if (params_.wbc_full && high_speed_curriculum)
        {
            const double y_gain = Full2EnvDouble(
                "TROT_HS_ATTITUDE_Y_GAIN", -1.0);
            const double y_max = Full2EnvDouble(
                "TROT_HS_ATTITUDE_Y_MAX", -1.0);
            if (y_gain > 0.0 && y_max > 0.0)
                attitude_feedback_y_m_ = Clamp(
                    y_gain * attitude_roll_rad_, -y_max, y_max);
            const double x_gain = Full2EnvDouble(
                "TROT_HS_ATTITUDE_X_GAIN", -1.0);
            const double x_max = Full2EnvDouble(
                "TROT_HS_ATTITUDE_X_MAX", -1.0);
            if (x_gain > 0.0 && x_max > 0.0)
                attitude_feedback_x_m_ = Clamp(
                    -x_gain * attitude_pitch_rad_, -x_max, x_max);
        }
        for (auto &foot : feet)
        {
            foot.x -= attitude_feedback_x_m_;
            foot.y -= attitude_feedback_y_m_;
        }
    }
    else
    {
        attitude_feedback_x_m_ = 0.0;
        attitude_feedback_y_m_ = 0.0;
    }

    // Inner/outer-leg y offset (Raibert turn). Goal heading uses the same
    // plant; the old 4 s delay is gone so A→B can yaw immediately.
    double turn_rate = params_.turn_rate_radps;
    if (task_.goal_enabled_)
        turn_rate = task_.commanded_turn_rate_radps_;
    if (turn_rate != 0.0)
    {
        const double turn_enable = task_.TurnEnable(running_time_);
        const double delta = turn_enable * std::clamp(
            turn_rate * params_.period_s * 0.08,
            -0.015, 0.015);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const bool left = feet[leg].y > 0.0;
            feet[leg].y += (left ? -delta : delta);
        }
    }
    const double sprint_foot_slope = std::clamp(
        Full2EnvDouble("TROT_HS_FOOT_SLOPE", 0.0), -0.40, 0.40);
    if (params_.wbc_full && high_speed_curriculum &&
        std::abs(sprint_foot_slope) > 1.0e-9)
    {
        // Match the body-frame foot-height slope used by the validated
        // open-loop sprint probe.  It gives the stance pair a bounded
        // pitch moment through the normal joint targets, while the WBC
        // still enforces contact dynamics and torque limits.
        for (auto &foot : feet)
            foot.z -= sprint_foot_slope * foot.x;
    }

    // Terrain execution is a bounded endpoint transaction on top of the
    // running-trot phase.  The kernel remains the sole timing/topology source;
    // the planner-selected world foothold is only latched during the existing
    // swing and is blended to its endpoint before touchdown.
    if (params_.terrain_actuation && terrain_tick_plan_ && have_high_state)
    {
        const double terrain_now_s =
            static_cast<double>(state_snapshot.tick()) * 1.0e-3;
        const std::size_t terrain_k0 = go2_terrain::TerrainPlanCurrentKnot(
            *terrain_tick_plan_, terrain_now_s);
        int required_mask = 0;
        int committed_mask = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const double leg_phase = go2_control::GaitLegPhase(
                leg, phase, params_.gait_pattern);
            const bool in_swing = leg_phase >= stance_duration;
            if (terrain_execution_target_valid_[leg])
                required_mask |= 1 << static_cast<int>(leg);
            if (!in_swing)
            {
                terrain_execution_in_flight_[leg] = false;
                if (terrain_execution_target_valid_[leg])
                {
                    const bool measured = state_snapshot.foot_force()[leg] >=
                        kContactForceThreshold;
                    if (measured)
                    {
                        terrain_execution_measured_touchdown_[leg] = true;
                        committed_mask |= 1 << static_cast<int>(leg);
                        if (!terrain_execution_completion_recorded_[leg])
                        {
                            terrain_execution_completion_recorded_[leg] = true;
                            ++terrain_surface_transition_completions_;
                        }
                    }
                }
                continue;
            }

            if (!terrain_execution_in_flight_[leg])
            {
                terrain_execution_target_valid_[leg] = false;
                terrain_execution_measured_touchdown_[leg] = false;
                terrain_execution_completion_recorded_[leg] = false;
                bool has_planner_touchdown = false;
                for (std::size_t k = terrain_k0;
                     k < terrain_tick_plan_->horizon_knots; ++k)
                {
                    const auto &foot = terrain_tick_plan_->predicted_foothold[k][leg];
                    has_planner_touchdown = has_planner_touchdown || foot.touchdown;
                    if (!foot.valid || !foot.touchdown)
                        continue;
                    terrain_execution_target_world_[leg] =
                        go2::ContactPatchToFootSite(foot.position_world);
                    terrain_execution_target_lift_[leg] = foot.swing_lift_m;
                    terrain_execution_swing_start_phase_[leg] = leg_phase;
                    terrain_execution_target_plan_id_[leg] =
                        terrain_tick_plan_->plan_id;
                    terrain_execution_target_valid_[leg] = true;
                    terrain_execution_in_flight_[leg] = true;
                    {
                        std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
                        ++terrain_target_prepare_attempts_;
                        ++terrain_target_prepared_;
                    }
                    break;
                }
                if (!terrain_execution_target_valid_[leg] && has_planner_touchdown)
                {
                    std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
                    ++terrain_target_prepare_attempts_;
                    ++terrain_target_prepare_rejections_;
                }
            }

            if (!terrain_execution_target_valid_[leg])
                continue;
            required_mask |= 1 << static_cast<int>(leg);
            const go2::Vec3 target_body = go2_control::WorldToBody(
                pose.base, pose.quaternion,
                terrain_execution_target_world_[leg]);
            const double swing_u = std::clamp(
                (leg_phase - stance_duration) /
                    std::max(1.0e-6, 1.0 - stance_duration),
                0.0, 1.0);
            const double start_u = std::clamp(
                (terrain_execution_swing_start_phase_[leg] - stance_duration) /
                    std::max(1.0e-6, 1.0 - stance_duration),
                0.0, 0.95);
            const double handoff_u = Smoothstep(
                (swing_u - start_u) / std::max(1.0e-3, 1.0 - start_u));
            feet[leg].x += (target_body.x - feet[leg].x) * handoff_u;
            feet[leg].y += (target_body.y - feet[leg].y) * handoff_u;
            feet[leg].z += (target_body.z - feet[leg].z) * handoff_u;
            const double nominal_lift = std::max(
                params_.foot_lift_m, runtime_gait_foot_lift_m_);
            const double extra_lift = std::max(
                0.0, terrain_execution_target_lift_[leg] - nominal_lift);
            feet[leg].z += extra_lift * go2_terrain::TerrainSwingProfile(
                swing_u, 0.5);
            if (handoff_u > 1.0e-6 || extra_lift > 1.0e-6)
            {
                std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
                ++terrain_gait_target_override_count_;
            }
        }
        if (required_mask != 0)
        {
            terrain_surface_transition_required_mask_ = required_mask;
            terrain_surface_transition_committed_mask_ = committed_mask;
            terrain_surface_transition_last_required_mask_ = required_mask;
            terrain_surface_transition_last_committed_mask_ = committed_mask;
        }
    }
    if (have_commanded_body_feet_ && last_motion_dt_s_ > 1.0e-5)
    {
        const double inv_dt = 1.0 / last_motion_dt_s_;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            commanded_body_feet_velocity_[leg].x =
                std::clamp((feet[leg].x - commanded_body_feet_[leg].x) * inv_dt,
                           -12.0, 12.0);
            commanded_body_feet_velocity_[leg].y =
                std::clamp((feet[leg].y - commanded_body_feet_[leg].y) * inv_dt,
                           -12.0, 12.0);
            commanded_body_feet_velocity_[leg].z =
                std::clamp((feet[leg].z - commanded_body_feet_[leg].z) * inv_dt,
                           -12.0, 12.0);
        }
        have_commanded_body_feet_velocity_ = true;
    }
    else
    {
        commanded_body_feet_velocity_.fill({0.0, 0.0, 0.0});
        have_commanded_body_feet_velocity_ = false;
    }
    commanded_body_feet_ = feet;
    have_commanded_body_feet_ = true;
    if (params_.wbc_full)
    {
        if (!go2::AllLegInverseKinematicsClamped(feet, joint_targets))
        {
            std::cerr << "Trot IK failed at gait_time=" << gait_time_s << "\n";
            return false;
        }
    }
    else if (!go2::AllLegInverseKinematics(feet, joint_targets))
    {
        std::cerr << "Trot IK failed at gait_time=" << gait_time_s << "\n";
        return false;
    }
    return true;
}
