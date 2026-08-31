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
        velocity_command_initialized_ = true;
    }
    const double dt = (std::isfinite(last_motion_dt_s_) &&
                       last_motion_dt_s_ > 1.0e-4)
        ? last_motion_dt_s_
        : dt_;
    double requested_mps = params_.velocity_command_profile.Sample(gait_time_s);
    const bool flat_crawl_debug =
        Full2EnvDouble("TROT_TERRAIN_DEBUG_FLAT_CRAWL", 0.0) > 0.5;
    const bool staged_start_debug =
        Full2EnvDouble("TROT_TERRAIN_DEBUG_STAGED_START", 0.0) > 0.5;
    // Harness-only isolation: arm the sequencer at gait start without a
    // terrain window or lidar map. The default remains entirely inactive.
    if ((flat_crawl_debug ||
         (staged_start_debug && params_.terrain_actuation &&
          !params_.terrain_sensor_only)) &&
        task_.gait_started_ && task_.motion_stage_ == 2 &&
        !terrain_transfer_window_active_)
    {
        terrain_transfer_window_active_ = true;
        terrain_approach_braking_active_ = false;
        terrain_transfer_window_release_s_ =
            -std::numeric_limits<double>::infinity();
        terrain_crawl_sequencer_.Reset();
    }
    // V2-A: the declared terrain window owns a bounded creep request. Keep
    // the Phase 1 shaper as the sole command writer, but do not let an
    // asynchronous planner snapshot restore the approach command while the
    // body is crossing the riser. This is a creep, not a full brake.
    const double absolute_gait_time_s = gait_time_s +
        task_.gait_start_time_s_;
    if (terrain_transfer_window_active_ && !flat_crawl_debug &&
        std::isfinite(terrain_transfer_window_release_s_) &&
        absolute_gait_time_s >= terrain_transfer_window_release_s_)
    {
        terrain_transfer_window_active_ = false;
        terrain_approach_braking_active_ = false;
        terrain_approach_staging_error_m_ =
            std::numeric_limits<double>::quiet_NaN();
        terrain_approach_speed_cap_mps_ = 0.0;
        terrain_transfer_window_release_s_ =
            -std::numeric_limits<double>::infinity();
        // A commit ledger is scoped to one terrain transfer window. Clear it
        // only after the post-crossing stable passage, never at transaction
        // completion, so recovery and the next leg observe measured commits.
        terrain_surface_transition_committed_.fill(false);
        terrain_surface_transition_committed_surface_valid_.fill(false);
        terrain_surface_transition_committed_surface_world_z_.fill(0.0);
        terrain_crawl_state_machine_.Reset();
        terrain_crawl_sequencer_.Reset();
        terrain_crawl_sequencer_output_ = {};
        terrain_staged_target_valid_ = false;
        terrain_staged_target_world_ = {};
        terrain_crawl_min_contact_count_ = go2::kLegCount;
        terrain_crawl_step_commit_count_ = 0;
    }
    const bool terrain_window_active = terrain_transfer_window_active_;
    const auto terrain_state = terrain_crawl_state_machine_.state();
    // Use crawl support as soon as DECELERATE starts, while retaining the
    // continuous velocity ramp. SHIFT_COM remains gated on settled posture.
    const bool event_crawl_active =
        terrain_crawl_sequencer_output_.control_authority_active &&
        terrain_crawl_sequencer_output_.state !=
            go2_terrain::TerrainCrawlSequencerState::kStage;
    const bool terrain_crawl_active = terrain_window_active &&
        (event_crawl_active ||
         (terrain_crawl_sequencer_output_.control_authority_active &&
          terrain_crawl_sequencer_output_.state !=
              go2_terrain::TerrainCrawlSequencerState::kStage &&
          terrain_crawl_state_machine_.UsesCrawlExecution()) ||
         (terrain_crawl_sequencer_output_.control_authority_active &&
          terrain_state == go2_terrain::TerrainCrawlState::kDecelerateToCreep &&
          terrain_crawl_sequencer_output_.state !=
              go2_terrain::TerrainCrawlSequencerState::kStage));
    if ((params_.terrain_actuation && !params_.terrain_sensor_only) ||
        flat_crawl_debug)
    {
        if (terrain_window_active)
        {
            const auto crawl_state = terrain_crawl_state_machine_.state();
            const bool approach_braking =
                terrain_approach_braking_active_ &&
                !terrain_crawl_sequencer_output_.control_authority_active &&
                !flat_crawl_debug && !staged_start_debug;
            if (approach_braking)
            {
                // V2-A is an adaptive stopping profile, not a passive wait
                // for the seizure predicates. Keep trot authority and shape
                // the requested speed toward the live staging target.
                terrain_approach_speed_cap_mps_ =
                    go2_terrain::TerrainCrawlSequencer::ApproachSpeedCapMps(
                        terrain_approach_staging_error_m_);
                requested_mps = std::min(
                    std::max(0.0, requested_mps),
                    terrain_approach_speed_cap_mps_);
                terrain_deceleration_active_ = true;
                terrain_deceleration_target_mps_ =
                    terrain_approach_speed_cap_mps_;
                runtime_gait_regime_ = "terrain-approach-brake";
            }
            else
            {
            const bool sequencer_staging =
                terrain_crawl_sequencer_output_.control_authority_active &&
                terrain_crawl_sequencer_output_.state ==
                    go2_terrain::TerrainCrawlSequencerState::kStage;
            if (sequencer_staging ||
                terrain_crawl_sequencer_output_.stand_transition_requested)
            {
                // STAGE retains the trot contact schedule, but once the
                // legacy state machine is in STAGE it actively servos the
                // measured edge-minus-base basin instead of parking at the
                // old nominal standoff.
                if ((sequencer_staging ||
                     terrain_crawl_state_machine_.state() ==
                         go2_terrain::TerrainCrawlState::kStage) &&
                    !terrain_crawl_sequencer_output_.flat_ground_mode)
                {
                    // The measured four-contact COM target is the primary
                    // STAGE servo. Base creep remains only as a fallback
                    // while geometry is unavailable, avoiding a body command
                    // that fights the support-polygon correction.
                    if (terrain_crawl_state_machine_.stage_com_target_valid())
                    {
                        requested_mps = 0.0;
                        terrain_stage_direction_ = 1.0;
                    }
                    else
                    {
                    if (terrain_crawl_state_machine_.stage_micro_adjust_active(
                            absolute_gait_time_s))
                    {
                        terrain_stage_direction_ =
                            terrain_crawl_state_machine_.stage_micro_adjust_direction();
                        requested_mps =
                            go2_terrain::TerrainCrawlStateMachine::
                                kStageMicroAdjustSpeedMps;
                    }
                    else if (std::isfinite(terrain_staging_error_m_) &&
                             std::abs(terrain_staging_error_m_) >
                                 go2_terrain::TerrainCrawlStateMachine::
                                     kStageBasinHalfWidthM)
                    {
                        terrain_stage_direction_ =
                            std::copysign(1.0, terrain_staging_error_m_);
                        requested_mps =
                            go2_terrain::TerrainCrawlStateMachine::kCreepSpeedMps;
                    }
                    else
                        requested_mps = 0.0;
                    }
                }
                else
                    requested_mps = 0.0;
                terrain_deceleration_active_ = false;
            }
            else if (terrain_crawl_state_machine_.aborted())
            {
                // Abort is a controlled v2 window stop. Let the existing
                // shaper ramp to zero; do not enter the v1 stop-brake path.
                requested_mps = 0.0;
                terrain_velocity_cap_mps_.store(0.0);
                terrain_deceleration_active_ = false;
                runtime_gait_regime_ = "terrain-crawl-abort";
            }
            else if (crawl_state == go2_terrain::TerrainCrawlState::kDecelerateToCreep)
            {
                // Keep trot timing unchanged while spreading the handoff:
                // the longer ramp removes residual stride momentum before
                // the measured settle dwell permits SHIFT_COM.
                constexpr double kDecelRampS = 1.20;
                constexpr double kApproachSpeedMps = 0.30;
                constexpr double kCreepSpeedMps =
                    go2_terrain::TerrainCrawlStateMachine::kCreepSpeedMps;
                if (!terrain_deceleration_active_)
                {
                    terrain_deceleration_active_ = true;
                    terrain_deceleration_target_mps_ = kApproachSpeedMps;
                }
                terrain_deceleration_target_mps_ = std::max(
                    kCreepSpeedMps, terrain_deceleration_target_mps_ -
                        (kApproachSpeedMps - kCreepSpeedMps) *
                        dt / kDecelRampS);
                requested_mps = std::min(
                    std::max(0.0, requested_mps),
                    terrain_deceleration_target_mps_);
            }
            else if (crawl_state == go2_terrain::TerrainCrawlState::kApproach ||
                     crawl_state == go2_terrain::TerrainCrawlState::kInactive)
            {
                // Approach is the unchanged 0.30 m/s trot; do not apply the
                // planner's eventual crawl cap before DECELERATE starts.
                terrain_deceleration_active_ = false;
                terrain_deceleration_target_mps_ = 0.30;
            }
            else if (terrain_crawl_active)
            {
                terrain_velocity_cap_mps_.store(
                    go2_terrain::TerrainCrawlStateMachine::kAdvanceBodySpeedMps);
                if (crawl_state == go2_terrain::TerrainCrawlState::kStage)
                {
                    // Stage regulates the body to the sensor-derived edge
                    // reference. It never starts a crawl swing; stopping at
                    // the target lets the subsequent settle dwell establish
                    // a repeatable phase and posture.
                    if (terrain_crawl_state_machine_.stage_micro_adjust_active(
                            absolute_gait_time_s))
                    {
                        // Once the edge-relative basin target is reached,
                        // probe a bounded forward/backward creep while the
                        // measured four-contact margin is still below +20 mm.
                        requested_mps =
                            terrain_crawl_state_machine_.stage_micro_adjust_direction() *
                            go2_terrain::TerrainCrawlStateMachine::
                                kStageMicroAdjustSpeedMps;
                    }
                    else if (std::isfinite(terrain_staging_error_m_) &&
                             std::abs(terrain_staging_error_m_) >
                                 go2_terrain::TerrainCrawlStateMachine::
                                     kStagePositionToleranceM)
                        requested_mps = std::copysign(
                            go2_terrain::TerrainCrawlStateMachine::kCreepSpeedMps,
                            terrain_staging_error_m_);
                    else
                        requested_mps = 0.0;
                }
                else if (crawl_state == go2_terrain::TerrainCrawlState::kAdvanceBody)
                {
                    // Rear targets are checked at the measured pose. A bounded
                    // creep is required to bring them inside the FK envelope;
                    // stopping here would make ADVANCE_BODY self-blocking.
                    requested_mps =
                        go2_terrain::TerrainCrawlStateMachine::kAdvanceBodySpeedMps;
                }
                else
                    requested_mps = 0.0;
            }
            }
        }
        else
        {
            terrain_deceleration_active_ = false;
            terrain_deceleration_target_mps_ = 0.30;
            if (std::isfinite(terrain_velocity_cap_mps_.load()))
            {
                requested_mps = std::min(
                    std::max(0.0, requested_mps),
                    std::max(0.0, terrain_velocity_cap_mps_.load()));
            }
        }
    }
    // Flat isolation uses a small, harness-local body creep in addition to
    // the measured swing/commit loop. The default stays zero so the debug
    // switch cannot alter ordinary gait behavior.
    if (flat_crawl_debug)
    {
        const auto flat_state = terrain_crawl_sequencer_output_.state;
        const bool flat_body_creep_allowed =
            terrain_crawl_sequencer_output_.control_authority_active &&
            flat_state != go2_terrain::TerrainCrawlSequencerState::kStage &&
            flat_state != go2_terrain::TerrainCrawlSequencerState::kResume;
        requested_mps = flat_body_creep_allowed
            ? std::clamp(Full2EnvDouble(
                "TROT_FLAT_CRAWL_BODY_SPEED", 0.05), 0.0, 0.12)
            : 0.0;
    }
    if (terrain_window_active && !flat_crawl_debug &&
        std::isfinite(terrain_staging_error_m_) &&
        std::abs(terrain_staging_error_m_) >
            go2_terrain::TerrainCrawlStateMachine::kStageBasinHalfWidthM)
    {
        terrain_stage_direction_ =
            std::copysign(1.0, terrain_staging_error_m_);
        requested_mps =
            go2_terrain::TerrainCrawlStateMachine::kStageMicroAdjustSpeedMps;
    }
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
    double applied_mps = velocity_command_state_.shaped_mps;
    if (have_filtered_body_velocity_ &&
        velocity_command_state_.shaped_mps > 0.90)
    {
        const double measured_mps = std::max(
            0.0, params_.direction_sign * latest_filtered_body_velocity_[0]);
        applied_mps = std::min(
            velocity_command_state_.shaped_mps,
            measured_mps +
                params_.velocity_command_shaper.max_tracking_lead_mps);
        // If the measured body is already ahead of the shaped command by
        // more than the allowed lead, lower the applied reference smoothly
        // so the gait schedule and MPC see a meaningful braking demand.
        // Merely capping at measured+lead leaves an overspeeding body at the
        // original command and cannot recover the tracking error.
        const double overspeed = measured_mps -
            velocity_command_state_.shaped_mps;
        if (overspeed >
            params_.velocity_command_shaper.max_tracking_lead_mps)
        {
            applied_mps = std::max(
                0.0,
                velocity_command_state_.shaped_mps -
                    (overspeed -
                     params_.velocity_command_shaper.max_tracking_lead_mps));
        }
    }
    const bool runtime_health_governor = params_.wbc_full &&
        !params_.cartesian_world &&
        Full2EnvDouble("TROT_HS_STABILITY_GOV", 0.0) > 0.5;
    if (runtime_health_governor && high_speed_health_cap_mps_ > 0.0 &&
        applied_mps > high_speed_health_cap_mps_)
    {
        applied_mps = high_speed_health_cap_mps_;
    }
    velocity_command_state_.applied_mps = applied_mps;
    const auto schedule = terrain_crawl_active
        ? ScheduleTerrainCrawl(applied_mps)
        : velocity_gait_scheduler_.Step(applied_mps, dt);
    runtime_gait_pattern_ = terrain_crawl_active
        ? go2_control::GaitPattern::kCrawl
        : params_.gait_pattern;
    locomotion_kernel_->SetGaitPattern(runtime_gait_pattern_);
    locomotion_kernel_->SetGaitEffectiveSpeedConvention(true);
    locomotion_kernel_->SetGaitSlewLimits(0.060, 0.020, 0.020);
    locomotion_kernel_->SetGaitPeriod(schedule.period_s);
    locomotion_kernel_->SetGaitDuty(schedule.duty_factor);
    locomotion_kernel_->SetGaitStepLength(schedule.step_length_m);
    locomotion_kernel_->SetGaitFootLift(schedule.foot_lift_m);
    const bool terrain_stage_servo = terrain_window_active &&
        !flat_crawl_debug && std::isfinite(terrain_staging_error_m_);
    wbc_speed_cmd_mps_ = terrain_stage_servo
        ? terrain_stage_direction_ * applied_mps
        : (terrain_crawl_active &&
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kStage
            ? params_.direction_sign * applied_mps
            : std::abs(params_.direction_sign) * applied_mps);
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
    const auto terrain_state = terrain_crawl_state_machine_.state();
    // Keep the crawl support pattern through DECELERATE while the state
    // machine holds SHIFT_COM behind the measured speed/posture dwell.
    const bool event_crawl_active =
        terrain_crawl_sequencer_output_.control_authority_active &&
        terrain_crawl_sequencer_output_.state !=
            go2_terrain::TerrainCrawlSequencerState::kStage;
    const bool terrain_crawl_active = terrain_transfer_window_active_ &&
        (event_crawl_active ||
         (terrain_crawl_sequencer_output_.control_authority_active &&
          terrain_crawl_sequencer_output_.state !=
              go2_terrain::TerrainCrawlSequencerState::kStage &&
          terrain_crawl_state_machine_.UsesCrawlExecution()) ||
         (terrain_crawl_sequencer_output_.control_authority_active &&
          terrain_state == go2_terrain::TerrainCrawlState::kDecelerateToCreep &&
          terrain_crawl_sequencer_output_.state !=
              go2_terrain::TerrainCrawlSequencerState::kStage));
    runtime_gait_pattern_ = terrain_crawl_active
        ? go2_control::GaitPattern::kCrawl
        : params_.gait_pattern;
    locomotion_kernel_->SetGaitPattern(runtime_gait_pattern_);
    const bool sequencer_staging =
        terrain_crawl_sequencer_output_.control_authority_active &&
        terrain_crawl_sequencer_output_.state ==
            go2_terrain::TerrainCrawlSequencerState::kStage;
    locomotion_kernel_->SetStanceHold(sequencer_staging, gait_time_s);
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
    if (high_speed_stop_hold_active_)
        locomotion_kernel_->SetStanceHold(true, gait_time_s);
    if (!locomotion_kernel_->Compute(gait_request, gait_result))
    {
        std::cerr << "Locomotion kernel failed at gait_time="
                  << gait_time_s << std::endl;
        return false;
    }
    kernel_footstep_plan_valid_ = gait_result.footstep_plan_valid;
    kernel_velocity_error_x_mps_ = gait_result.velocity_error_x_mps;
    kernel_nominal_velocity_x_mps_ = gait_result.nominal_velocity_x_mps;
    kernel_touchdown_target_x_m_ = gait_result.touchdown_target_x_m;
    kernel_touchdown_target_feet_base_ =
        gait_result.touchdown_target_feet_base;
    have_kernel_touchdown_target_feet_ =
        gait_result.touchdown_target_feet_valid;
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
    const double terrain_now_s = static_cast<double>(state_snapshot.tick()) *
        1.0e-3;
    const auto latest_terrain_plan =
        params_.terrain_actuation && !params_.terrain_sensor_only
            ? terrain_plan_store_.LoadUsable(terrain_now_s)
            : nullptr;
    const bool flat_crawl_debug =
        Full2EnvDouble("TROT_TERRAIN_DEBUG_FLAT_CRAWL", 0.0) > 0.5;
    const bool staged_start_debug =
        Full2EnvDouble("TROT_TERRAIN_DEBUG_STAGED_START", 0.0) > 0.5;
    const bool terrain_execution_allowed =
        (params_.terrain_actuation && !params_.terrain_sensor_only) ||
        flat_crawl_debug;
    const bool gait_execution_started =
        task_.gait_started_ && task_.motion_stage_ == 2;
    std::shared_ptr<const go2_terrain::TerrainModel> live_terrain_model;
    {
        std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
        live_terrain_model = terrain_model_;
    }
    const auto early_pose = ComputeWorldPose(
        state_snapshot, high_state_snapshot);
    // Arm the v2 window from the measured map edge, before the trot
    // transition intent. This is deliberately early: STAGE must be able to
    // settle at the edge-relative standoff rather than inherit an overshoot.
    if (terrain_execution_allowed && gait_execution_started &&
        !terrain_transfer_window_active_ &&
        (flat_crawl_debug || (live_terrain_model && have_high_state)))
    {
        bool activate = flat_crawl_debug || staged_start_debug;
        if (!activate)
        {
            const auto nominal_feet = go2::AllFootPositions(
                task_.stand_up_joint_pos_);
            const double nominal_front_x =
                0.5 * (nominal_feet[0].x + nominal_feet[1].x);
            activate = go2_terrain::TerrainCrawlSequencer::TransferActivationReady(
                *live_terrain_model, early_pose.base, early_pose.yaw_rad,
                nominal_front_x);
        }
        if (activate)
        {
            terrain_transfer_window_active_ = true;
            terrain_approach_braking_active_ =
                !flat_crawl_debug && !staged_start_debug;
            terrain_approach_staging_error_m_ = std::numeric_limits<double>::quiet_NaN();
            terrain_approach_speed_cap_mps_ =
                go2_terrain::TerrainCrawlSequencer::kApproachMaxSpeedMps;
            if (!flat_crawl_debug && live_terrain_model)
            {
                const auto staging = staged_start_debug
                    ? go2_terrain::MeasureTerrainBasinStagingReference(
                          *live_terrain_model, early_pose.base,
                          early_pose.yaw_rad)
                    : go2_terrain::MeasureTerrainStagingReference(
                        *live_terrain_model, early_pose.base, early_pose.yaw_rad,
                        0.5 * (go2::AllFootPositions(task_.stand_up_joint_pos_)[0].x +
                               go2::AllFootPositions(task_.stand_up_joint_pos_)[1].x),
                        go2_terrain::TerrainCrawlSequencer::kStandoffM);
                if (staging.valid)
                    terrain_approach_staging_error_m_ = staging.error_m;
            }
            terrain_transfer_window_release_s_ =
                -std::numeric_limits<double>::infinity();
            terrain_staged_target_valid_ = false;
            terrain_staged_target_world_ = {};
            terrain_crawl_sequencer_.Reset();
        }
    }
    go2_terrain::TerrainCrawlSequencerOutput sequencer_output{};
    bool terrain_crawl_control_authority_active = false;
    // A prepared-but-not-started target is not yet an execution snapshot.
    // Permit a newer planner publication to replace it, because that is where
    // S1's retimed touchdown knot becomes visible to the execution rebase.
    const bool terrain_swing_transaction_active =
        std::any_of(
            terrain_swing_execution_.begin(), terrain_swing_execution_.end(),
            [](const TerrainSwingExecution &execution) {
                return execution.valid &&
                    (execution.in_flight || execution.endpoint_held);
            }) ||
        std::any_of(
            terrain_swing_pending_.begin(), terrain_swing_pending_.end(),
            [](const TerrainSwingExecution &execution) {
                return execution.valid;
            });
    if (!terrain_execution_allowed)
    {
        terrain_execution_plan_.reset();
        terrain_swing_execution_ = {};
        terrain_swing_pending_ = {};
        terrain_transfer_hold_contact_.fill(false);
        terrain_transfer_hold_active_ = false;
        terrain_surface_transition_active_ = false;
        terrain_transfer_window_active_ = false;
        terrain_approach_braking_active_ = false;
        terrain_approach_staging_error_m_ =
            std::numeric_limits<double>::quiet_NaN();
        terrain_approach_speed_cap_mps_ = 0.0;
        terrain_transfer_window_release_s_ =
            -std::numeric_limits<double>::infinity();
        terrain_crawl_sequencer_.Reset();
        terrain_crawl_sequencer_output_ = {};
        runtime_gait_pattern_ = params_.gait_pattern;
        terrain_surface_transition_required_.fill(false);
        terrain_surface_transition_original_required_.fill(false);
        terrain_surface_transition_cancelled_.fill(false);
        terrain_surface_transition_committed_.fill(false);
        terrain_surface_transition_committed_surface_valid_.fill(false);
        terrain_surface_transition_committed_surface_world_z_.fill(0.0);
        terrain_surface_transition_source_valid_.fill(false);
    }
    else if (!terrain_swing_transaction_active)
    {
        if (terrain_execution_plan_ != latest_terrain_plan)
        {
            // Do not carry a plan-time touchdown into the boundary when a
            // newer S1-retimed knot has arrived. In-flight/held targets are
            // immutable; only the not-yet-started handoff is replaceable.
            // Once CRAWL_STEP has accepted the fixed script handoff, keep
            // its prepared endpoint across asynchronous planner refreshes.
            // Replacing it here reintroduces the old short gait deadline
            // before the next tick can launch the scripted swing.
            // Once the transfer window opens, the prepared endpoint is a
            // scripted transaction even while entry is still settling. Keep
            // it across map refreshes; the fixed CRAWL_STEP handoff retimes
            // it before any swing is launched.
            const bool crawl_script_handoff_active =
                terrain_transfer_window_active_;
            if (!crawl_script_handoff_active)
            {
                for (auto &execution : terrain_swing_execution_)
                    if (execution.valid && !execution.in_flight &&
                        !execution.endpoint_held)
                        execution = {};
                terrain_swing_pending_ = {};
            }
        }
        // During a crawl shift, a transient planner rejection must not erase
        // the last usable snapshot before the selected leg is prepared. Keep
        // it until its immutable touchdown window expires or a newer usable
        // snapshot replaces it; flat/out-of-window behavior is unchanged.
        if (latest_terrain_plan || !terrain_transfer_window_active_)
            terrain_execution_plan_ = latest_terrain_plan;
    }
    const auto active_terrain_plan = terrain_execution_plan_;
    std::array<bool, go2::kLegCount> terrain_contact_now{};
    bool terrain_contact_now_valid = false;
    if (active_terrain_plan)
    {
        std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
            terrain_now_index{};
        terrain_contact_now_valid =
            go2_terrain::BuildTerrainPlanHorizonIndices(
                *active_terrain_plan, terrain_now_s,
                terrain_planner_.config().knot_dt_s,
                terrain_planner_.config().knot_dt_s, 1, terrain_now_index);
        if (terrain_contact_now_valid)
            terrain_contact_now = active_terrain_plan->contact_schedule
                .planned_contact[terrain_now_index[0]];
    }
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
                      << go2_control::GaitPatternName(runtime_gait_pattern_)
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
                leg, phase, runtime_gait_pattern_);
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

    // Refresh the approach target from the live map while the trot still
    // owns authority; this keeps the adaptive envelope tied to the measured
    // edge rather than an arming-time snapshot.
    if (terrain_transfer_window_active_ && !flat_crawl_debug &&
        live_terrain_model && have_high_state &&
        (!terrain_crawl_sequencer_output_.control_authority_active ||
         terrain_crawl_sequencer_output_.state ==
             go2_terrain::TerrainCrawlSequencerState::kStage))
    {
        const auto approach_pose = ComputeWorldPose(
            state_snapshot, high_state_snapshot);
        const auto staging = go2_terrain::MeasureTerrainStagingReference(
            *live_terrain_model, approach_pose.base, approach_pose.yaw_rad,
            0.5 * (go2::AllFootPositions(task_.stand_up_joint_pos_)[0].x +
                   go2::AllFootPositions(task_.stand_up_joint_pos_)[1].x),
            go2_terrain::TerrainCrawlSequencer::kStandoffM);
        if (staging.valid)
            terrain_approach_staging_error_m_ = staging.error_m;
    }

    // Drive terrain execution from the explicit v2 sequencer. All inputs
    // here are observations from the force filter, lidar plan, current FK,
    // and measured body/foot pose; no scene or XML truth is consulted.
    if (terrain_transfer_window_active_)
    {
        go2_terrain::TerrainCrawlSequencerInput sequencer_input;
        sequencer_input.transfer_window_active = true;
        sequencer_input.flat_ground_mode = flat_crawl_debug;
        sequencer_input.flat_step_length_m =
            params_.direction_sign * std::clamp(Full2EnvDouble(
                "TROT_FLAT_CRAWL_STEP_M", 0.04), 0.02, 0.08);
        const double trot_period = gait_result.period_s > 0.05
            ? gait_result.period_s : params_.period_s;
        const double trot_duty = gait_result.duty_factor > 0.05
            ? gait_result.duty_factor : params_.duty_factor;
        std::array<std::array<bool, go2::kLegCount>,
                   go2_control::kSrbdMaxHorizon> trot_contacts{};
        go2_control::FillTrotContactSchedulePhase(
            gait_result.phase, trot_period, trot_duty, 1, 0.0,
            trot_contacts, params_.gait_pattern);
        sequencer_input.trot_full_contact_able = std::all_of(
            trot_contacts[0].begin(), trot_contacts[0].end(),
            [](bool contact) { return contact; });
        sequencer_input.legacy_stage_ready = staged_start_debug ||
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kShiftCom ||
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kCrawlStep ||
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kAdvanceBody ||
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kClear ||
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kResume;
        // Gate terrain SWING on the legacy SHIFT_COM witness itself, not a
        // fixed sequencer dwell. This mirrors the state machine's measured
        // COM-margin and three-leg force-balance predicates while avoiding a
        // circular wait for CRAWL_STEP (which is entered after this update).
        bool legacy_shift_ready = terrain_crawl_state_machine_.state() ==
            go2_terrain::TerrainCrawlState::kCrawlStep;
        if (!legacy_shift_ready && !flat_crawl_debug &&
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kShiftCom &&
            terrain_crawl_state_machine_.com_target_valid() &&
            terrain_crawl_state_machine_.com_margin_m() >= 0.02 &&
            have_high_state)
        {
            const std::size_t lifted_leg =
                terrain_crawl_state_machine_.com_target_leg();
            double force_total = 0.0;
            double force_min = std::numeric_limits<double>::infinity();
            double force_max = 0.0;
            bool force_ready = lifted_leg < go2::kLegCount;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (leg == lifted_leg)
                    continue;
                const double force = state_snapshot.foot_force()[leg];
                force_ready = force_ready && std::isfinite(force) &&
                    force >= 10.0;
                force_total += force;
                force_min = std::min(force_min, force);
                force_max = std::max(force_max, force);
            }
            const double shift_elapsed = terrain_now_s -
                terrain_crawl_state_machine_.state_enter_time_s();
            const bool shift_geometry_ready = std::isfinite(shift_elapsed) &&
                shift_elapsed + 1.0e-9 >=
                    terrain_crawl_state_machine_.com_shift_duration_s() + 0.20;
            legacy_shift_ready = force_ready && force_total >= 50.0 &&
                force_min > 0.0 && force_max <= 4.0 * force_min &&
                shift_geometry_ready;
        }
        sequencer_input.com_margin_m =
            terrain_crawl_state_machine_.com_margin_m();
        sequencer_input.legacy_shift_ready = flat_crawl_debug ||
            legacy_shift_ready;
        // Recheck the exact measured triangle at the sequencer handoff. The
        // legacy state can remain CRAWL_STEP after its prior tick's witness
        // has decayed; force balance alone must not launch an unsafe swing.
        if (!flat_crawl_debug && sequencer_input.legacy_shift_ready)
            sequencer_input.legacy_shift_ready =
                std::isfinite(sequencer_input.com_margin_m) &&
                sequencer_input.com_margin_m >= 0.02;
        sequencer_input.terrain = live_terrain_model.get();
        if (staged_start_debug &&
            terrain_crawl_state_machine_.com_target_leg() < go2::kLegCount)
        {
            const auto leg = terrain_crawl_state_machine_.com_target_leg();
            // The planner may have prepared a future-pose foothold before
            // staged authority. Do not freeze that stale endpoint: at the
            // event boundary derive the fixed target from the measured foot
            // and current lidar edge, then retain that measured world target.
            if (!terrain_staged_target_valid_ && live_terrain_model &&
                have_actual_world_feet)
            {
                const auto current_base = go2_control::WorldToBody(
                    pose.base, pose.quaternion, actual_world_feet[leg]);
                const auto measured = go2_terrain::MeasureTerrainScriptTarget(
                    *live_terrain_model, static_cast<go2::Leg>(leg),
                    current_base);
                if (measured.valid)
                {
                    const auto target_base = go2::ContactPatchToFootSite(
                        measured.position_base);
                    terrain_staged_target_world_ = go2_control::BodyToWorld(
                        pose.base, pose.quaternion, target_base);
                    // Preserve the measured-map stand-off and elevation
                    // target.  The planner has already checked the same
                    // endpoint for FK reachability; a second fixed clamp or
                    // site-height override would change the contact patch.
                    terrain_staged_target_valid_ =
                        std::isfinite(terrain_staged_target_world_.x) &&
                        std::isfinite(terrain_staged_target_world_.y) &&
                        std::isfinite(terrain_staged_target_world_.z);
                }
            }
            sequencer_input.staged_target_valid = terrain_staged_target_valid_;
            sequencer_input.staged_target_world = terrain_staged_target_world_;
        }
        sequencer_input.base_position_world = pose.base;
        sequencer_input.base_yaw_rad = pose.yaw_rad;
        const auto nominal_feet = go2::AllFootPositions(
            task_.stand_up_joint_pos_);
        sequencer_input.nominal_front_foot_x_m =
            0.5 * (nominal_feet[0].x + nominal_feet[1].x);
        sequencer_input.measured_feet_world = actual_world_feet;
        sequencer_input.measured_feet_valid = have_actual_world_feet;
        sequencer_input.measured_contact_valid =
            wbc_shadow_contact_state_valid_;
        sequencer_input.measured_contact = wbc_shadow_contact_state_;
        sequencer_input.measured_force_valid = have_high_state;
        if (sequencer_input.measured_force_valid)
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                sequencer_input.measured_normal_force_n[leg] =
                    state_snapshot.foot_force()[leg];
        sequencer_input.measured_com_valid = have_measured_com_world_;
        sequencer_input.measured_com_world = measured_com_world_;
        sequencer_input.measured_velocity_mps = have_filtered_body_velocity_
            ? std::abs(latest_filtered_body_velocity_[0]) : 0.0;
        const auto sequencer_rpy = state_snapshot.imu_state().rpy();
        sequencer_input.measured_posture_valid =
            std::isfinite(sequencer_rpy[0]) && std::isfinite(sequencer_rpy[1]);
        sequencer_input.measured_roll_rad = sequencer_rpy[0];
        sequencer_input.measured_pitch_rad = sequencer_rpy[1];
        // Flat mode has no terrain planner to provide the rear-workspace and
        // clear witnesses. Supply only harness-local measured predicates so
        // the event sequence can complete all four legs.
        if (flat_crawl_debug)
        {
            sequencer_input.rear_targets_fk_reachable = true;
            sequencer_input.base_clear = true;
            sequencer_input.all_feet_clear = true;
            sequencer_input.stable =
                sequencer_input.measured_contact_valid &&
                std::count(sequencer_input.measured_contact.begin(),
                           sequencer_input.measured_contact.end(), true) >= 3 &&
                sequencer_input.measured_posture_valid &&
                std::abs(sequencer_input.measured_roll_rad) <= 0.08 &&
                std::abs(sequencer_input.measured_pitch_rad) <= 0.08 &&
                sequencer_input.measured_velocity_mps <= 0.12;
        }
        sequencer_input.now_s = terrain_now_s;
        (void)terrain_crawl_sequencer_.Update(sequencer_input);
        sequencer_output = terrain_crawl_sequencer_.output();
        terrain_crawl_sequencer_output_ = sequencer_output;
        terrain_crawl_control_authority_active =
            sequencer_output.control_authority_active;
        if (sequencer_output.state ==
                go2_terrain::TerrainCrawlSequencerState::kAbort)
            terrain_safe_stop_requested_.store(true);

        if (terrain_crawl_control_authority_active &&
            terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kInactive)
        {
            if (staged_start_debug)
                terrain_crawl_state_machine_.EnterStaged(terrain_now_s);
            else
                terrain_crawl_state_machine_.Enter(terrain_now_s);
        }
        go2_terrain::TerrainCrawlSignals crawl_signals;
        crawl_signals.transfer_window_active =
            terrain_crawl_control_authority_active;
        crawl_signals.scripted_execution =
            terrain_crawl_control_authority_active;
        crawl_signals.plan_valid = flat_crawl_debug
            ? sequencer_output.target_valid
            : ((active_terrain_plan && active_terrain_plan->valid()) ||
               sequencer_output.target_valid);
        // Staged isolation owns one fixed target prepared at SHIFT entry;
        // do not let the asynchronous planner expiry hide that immutable
        // handoff before the legacy state machine can enter CRAWL_STEP.
        if (staged_start_debug &&
            terrain_crawl_state_machine_.com_target_leg() < go2::kLegCount)
        {
            const auto leg = terrain_crawl_state_machine_.com_target_leg();
            crawl_signals.plan_valid = crawl_signals.plan_valid ||
                terrain_swing_execution_[leg].valid;
        }
        crawl_signals.sequencer_stage_pending = !flat_crawl_debug &&
            sequencer_output.state ==
                go2_terrain::TerrainCrawlSequencerState::kStage;
        crawl_signals.sequencer_pre_swing_pending = !flat_crawl_debug &&
            sequencer_output.state ==
                go2_terrain::TerrainCrawlSequencerState::kStage;
        crawl_signals.sequencer_swing_active = !flat_crawl_debug &&
            (sequencer_output.state ==
                 go2_terrain::TerrainCrawlSequencerState::kSwing ||
             sequencer_output.state ==
                 go2_terrain::TerrainCrawlSequencerState::kCommit) &&
            sequencer_output.target_valid &&
            sequencer_output.active_leg < go2::kLegCount;
        crawl_signals.measured_contact_valid = wbc_shadow_contact_state_valid_;
        crawl_signals.measured_contact = wbc_shadow_contact_state_;
        crawl_signals.measured_com_valid = have_measured_com_world_;
        crawl_signals.measured_com_world = measured_com_world_;
        crawl_signals.measured_foot_valid = have_actual_world_feet;
        crawl_signals.measured_foot_world = actual_world_feet;
        crawl_signals.measured_velocity_mps = have_filtered_body_velocity_
            ? std::abs(latest_filtered_body_velocity_[0]) : 0.0;
        const auto measured_rpy = state_snapshot.imu_state().rpy();
        crawl_signals.measured_posture_valid =
            std::isfinite(measured_rpy[0]) && std::isfinite(measured_rpy[1]);
        crawl_signals.measured_roll_rad = measured_rpy[0];
        crawl_signals.measured_pitch_rad = measured_rpy[1];
        if (terrain_crawl_state_machine_.state() ==
                go2_terrain::TerrainCrawlState::kStage &&
            have_high_state)
        {
            const auto staging_pose = ComputeWorldPose(
                state_snapshot, high_state_snapshot);
            if (live_terrain_model)
            {
                // STAGE is the map/target handoff boundary. Keep this dump
                // opt-in and one-shot so evidence records the exact model
                // consumed at that boundary without changing planning.
                static bool terrain_stage_map_dumped = false;
                if (!terrain_stage_map_dumped &&
                    Full2EnvDouble("TROT_TERRAIN_DEBUG_MAP", 0.0) > 0.5)
                {
                    const auto &map = *live_terrain_model;
                    std::size_t known_cells = 0;
                    std::size_t elevated_cells = 0;
                    double min_height = std::numeric_limits<double>::infinity();
                    double max_height = -std::numeric_limits<double>::infinity();
                    for (const auto &cell : map.cells)
                    {
                        if (!cell.known || !std::isfinite(cell.height_m))
                            continue;
                        ++known_cells;
                        min_height = std::min(min_height, cell.height_m);
                        max_height = std::max(max_height, cell.height_m);
                    }
                    for (std::size_t iy = 0; iy < map.height; ++iy)
                    {
                        for (std::size_t ix = 0; ix < map.width; ++ix)
                        {
                            const auto *cell = map.CellAt(ix, iy);
                            if (cell == nullptr || !cell->known ||
                                !std::isfinite(cell->height_m))
                                continue;
                            const double x = map.origin_m[0] +
                                (static_cast<double>(ix) + 0.5) * map.resolution_m;
                            if (x > 0.70 && std::isfinite(min_height) &&
                                cell->height_m >= min_height + 0.030)
                                ++elevated_cells;
                        }
                    }
                    std::cout << "Terrain STAGE map dump state="
                              << terrain_now_s << " epoch=" << map.epoch
                              << " stamp=" << map.map_stamp_s
                              << " age=" << map.age_s << " frame="
                              << map.frame_id << " origin=(" << map.origin_m[0]
                              << "," << map.origin_m[1] << ") resolution="
                              << map.resolution_m << " dims=" << map.width
                              << "x" << map.height << " known=" << known_cells
                              << " height_range=[" << min_height << ","
                              << max_height << "] plateau_cells_x_gt_070="
                              << elevated_cells << "\n";
                    for (std::size_t iy = 0; iy < map.height; ++iy)
                    {
                        std::cout << "Terrain STAGE map row iy=" << iy << ":";
                        for (std::size_t ix = 0; ix < map.width; ++ix)
                        {
                            const auto *cell = map.CellAt(ix, iy);
                            const double x = map.origin_m[0] +
                                (static_cast<double>(ix) + 0.5) * map.resolution_m;
                            std::cout << " " << x << ":" <<
                                (cell != nullptr && cell->known &&
                                 std::isfinite(cell->height_m)
                                     ? cell->height_m
                                     : std::numeric_limits<double>::quiet_NaN());
                        }
                        std::cout << "\n";
                    }
                    terrain_stage_map_dumped = true;
                }
                const auto staging =
                    terrain_crawl_state_machine_.state() ==
                            go2_terrain::TerrainCrawlState::kStage
                        ? go2_terrain::MeasureTerrainBasinStagingReference(
                              *live_terrain_model, staging_pose.base,
                              staging_pose.yaw_rad)
                        : go2_terrain::MeasureTerrainStagingReference(
                              *live_terrain_model, staging_pose.base,
                              staging_pose.yaw_rad,
                              0.5 * (go2::AllFootPositions(
                                  task_.stand_up_joint_pos_)[0].x +
                                  go2::AllFootPositions(
                                      task_.stand_up_joint_pos_)[1].x));
                crawl_signals.staging_target_valid = staging.valid;
                crawl_signals.staging_error_m = staging.error_m;
            }
            else if (active_terrain_plan &&
                     active_terrain_plan->staging_target_valid)
            {
                crawl_signals.staging_target_valid = true;
                crawl_signals.staging_error_m =
                    active_terrain_plan->staging_target_world_x_m -
                    staging_pose.base.x;
            }
            terrain_staging_error_m_ = crawl_signals.staging_target_valid
                ? crawl_signals.staging_error_m
                : std::numeric_limits<double>::quiet_NaN();
        }
        else
            terrain_staging_error_m_ = flat_crawl_debug
                ? 0.0 : std::numeric_limits<double>::quiet_NaN();
        if (flat_crawl_debug)
        {
            crawl_signals.staging_target_valid = true;
            crawl_signals.staging_error_m = 0.0;
        }
        crawl_signals.committed = flat_crawl_debug
            ? sequencer_output.committed
            : terrain_surface_transition_committed_;
        crawl_signals.measured_force_valid = true;
        if (crawl_signals.measured_force_valid)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                crawl_signals.measured_normal_force_n[leg] =
                    state_snapshot.foot_force()[leg];
        }
        crawl_signals.measured_com_velocity_valid = have_filtered_body_velocity_ &&
            std::isfinite(latest_filtered_body_velocity_[0]) &&
            std::isfinite(latest_filtered_body_velocity_[1]) &&
            std::isfinite(latest_filtered_body_velocity_[2]);
        if (crawl_signals.measured_com_velocity_valid)
            crawl_signals.measured_com_velocity_mps = std::sqrt(
                latest_filtered_body_velocity_[0] * latest_filtered_body_velocity_[0] +
                latest_filtered_body_velocity_[1] * latest_filtered_body_velocity_[1] +
                latest_filtered_body_velocity_[2] * latest_filtered_body_velocity_[2]);
        const double crawl_target_time_guard_s = 0.5 * std::max(
            1.0e-4, terrain_planner_.config().knot_dt_s);
        const auto crawl_target_is_live = [&](const TerrainSwingExecution &target) {
            if (!target.valid)
                return false;
            if (!target.terrain_target_required)
                return true;
            return std::isfinite(target.touchdown_time_s) &&
                target.touchdown_time_s > terrain_now_s +
                    crawl_target_time_guard_s;
        };
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            crawl_signals.target_valid[leg] =
                crawl_target_is_live(terrain_swing_execution_[leg]) ||
                crawl_target_is_live(terrain_swing_pending_[leg]);
        if (staged_start_debug &&
            terrain_crawl_state_machine_.com_target_leg() < go2::kLegCount)
        {
            const auto leg = terrain_crawl_state_machine_.com_target_leg();
            crawl_signals.target_valid[leg] =
                crawl_signals.target_valid[leg] ||
                terrain_swing_execution_[leg].valid;
        }
        // The event owner samples the live lidar map directly. Do not make
        // the planner's candidate publication a prerequisite for a swing.
        if (sequencer_output.target_valid &&
            terrain_crawl_state_machine_.ActiveLeg() ==
                sequencer_output.active_leg)
            crawl_signals.target_valid[sequencer_output.active_leg] = true;
        crawl_signals.rear_targets_fk_reachable = flat_crawl_debug;
        if (!flat_crawl_debug && active_terrain_plan && have_high_state)
        {
            const WorldPose crawl_pose = ComputeWorldPose(
                state_snapshot, high_state_snapshot);
            bool rear_targets_found = true;
            for (std::size_t leg = 2; leg < go2::kLegCount; ++leg)
            {
                const go2_terrain::TerrainFootholdPrediction *rear = nullptr;
                for (std::size_t k = 0;
                     k < active_terrain_plan->horizon_knots; ++k)
                {
                    const auto &foot = active_terrain_plan->predicted_foothold[k][leg];
                    if (foot.valid && foot.touchdown &&
                        foot.surface_transition_required)
                    {
                        rear = &foot;
                        break;
                    }
                }
                if (rear == nullptr)
                {
                    rear_targets_found = false;
                    break;
                }
                const go2::Vec3 target_base = go2_control::WorldToBody(
                    crawl_pose.base, crawl_pose.quaternion, rear->position_world);
                go2::LegJointPositions joints;
                const double radial = std::hypot(target_base.x, target_base.y);
                if (radial > 0.40 || !go2::LegInverseKinematics(
                        static_cast<go2::Leg>(leg), target_base, joints))
                {
                    rear_targets_found = false;
                    break;
                }
            }
            crawl_signals.rear_targets_fk_reachable = rear_targets_found;
        }
        crawl_signals.step_failed =
            terrain_crawl_state_machine_.ActiveLeg() < go2::kLegCount &&
            terrain_target_last_prepare_failure_by_leg_[
                terrain_crawl_state_machine_.ActiveLeg()] == 7;
        // CLEAR is based on measured world pose, not a scene edge constant.
        // A committed endpoint supplies the sensor-derived final-surface
        // reference; require all measured feet to be at that endpoint and
        // the measured base to be within the 0.40 m FK radial envelope.
        bool committed_targets_observed = true;
        double minimum_committed_target_x =
            std::numeric_limits<double>::infinity();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!crawl_signals.committed[leg])
            {
                committed_targets_observed = false;
                continue;
            }
            const auto &execution = terrain_swing_execution_[leg];
            if (!execution.valid || !execution.measured_touchdown ||
                !have_actual_world_feet ||
                !std::isfinite(execution.target_world.x) ||
                !std::isfinite(actual_world_feet[leg].x) ||
                actual_world_feet[leg].x < execution.target_world.x - 0.045)
            {
                committed_targets_observed = false;
                continue;
            }
            minimum_committed_target_x = std::min(
                minimum_committed_target_x, execution.target_world.x);
        }
        crawl_signals.all_feet_clear = flat_crawl_debug ||
            (committed_targets_observed &&
             std::isfinite(minimum_committed_target_x));
        crawl_signals.base_clear = flat_crawl_debug ||
            (crawl_signals.all_feet_clear && have_high_state &&
            std::isfinite(ComputeWorldPose(state_snapshot, high_state_snapshot).base.x) &&
            ComputeWorldPose(state_snapshot, high_state_snapshot).base.x >=
                minimum_committed_target_x - 0.40);
        crawl_signals.now_s = terrain_now_s;
        const auto crawl_state = terrain_crawl_state_machine_.Update(crawl_signals);
        if (crawl_state == go2_terrain::TerrainCrawlState::kResume &&
            !std::isfinite(terrain_transfer_window_release_s_))
            terrain_transfer_window_release_s_ = terrain_now_s + 0.45;
        if (crawl_state == go2_terrain::TerrainCrawlState::kAbort)
        {
            terrain_safe_stop_requested_.store(true);
            terrain_velocity_cap_mps_.store(0.0);
        }
        terrain_crawl_min_contact_count_ = std::min(
            terrain_crawl_min_contact_count_,
            crawl_signals.measured_contact_valid
                ? go2_terrain::TerrainCrawlContactCount(
                      crawl_signals.measured_contact)
                : 0);
    }

    // Consume one complete planner snapshot.  A selected foothold is a
    // trajectory transaction: prepare it in stance, execute it through the
    // actual gait swing boundary, then hold its world endpoint.  Planner
    // refreshes cannot make an in-flight target disappear mid-swing.
    // Flat isolation bypasses the terrain planner entirely. Keep every
    // support foot at its measured anchor and drive only the sequencer target.
    if (flat_crawl_debug && terrain_crawl_sequencer_output_.control_authority_active &&
        have_actual_world_feet)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            feet[leg] = go2_control::WorldToBody(
                pose.base, pose.quaternion, actual_world_feet[leg]);
        const auto sequencer_state =
            terrain_crawl_sequencer_output_.state;
        if ((sequencer_state ==
                 go2_terrain::TerrainCrawlSequencerState::kSwing ||
             sequencer_state ==
                 go2_terrain::TerrainCrawlSequencerState::kCommit) &&
            terrain_crawl_sequencer_output_.active_leg < go2::kLegCount &&
            terrain_crawl_sequencer_output_.target_valid)
        {
            const std::size_t leg = terrain_crawl_sequencer_output_.active_leg;
            const bool landing = sequencer_state ==
                go2_terrain::TerrainCrawlSequencerState::kCommit;
            const auto &target = landing
                ? terrain_crawl_sequencer_output_.target_world
                : terrain_crawl_sequencer_output_.swing_position_world;
            const auto &velocity = terrain_crawl_sequencer_output_.swing_velocity_world;
            feet[leg] = go2_control::WorldToBody(
                pose.base, pose.quaternion, target);
            if (params_.cartesian_world && have_commanded_world_feet_)
            {
                commanded_world_feet_[leg] = target;
                cartesian_state_.target_world_vel[leg] = landing
                    ? go2::Vec3{}
                    : velocity;
            }
        }
    }

    const bool terrain_target_adapter_enabled =
        Full2EnvDouble("TROT_TERRAIN_TARGET_ADAPTER", 1.0) > 0.5;
    if (terrain_target_adapter_enabled && active_terrain_plan &&
        have_high_state &&
        task_.gait_started_ && task_.motion_stage_ == 2)
    {
        ++terrain_plan_consumed_count_;
        bool terrain_target_overridden = false;
        const WorldPose terrain_pose = ComputeWorldPose(
            state_snapshot, high_state_snapshot);
        const bool terrain_target_xy_enabled =
            Full2EnvDouble("TROT_TERRAIN_TARGET_XY", 1.0) > 0.5;
        const bool terrain_target_z_enabled =
            Full2EnvDouble("TROT_TERRAIN_TARGET_Z", 1.0) > 0.5;
        std::array<double, go2::kLegCount> support_surface_heights{};
        std::size_t support_surface_count = 0;
        for (std::size_t support_leg = 0;
             support_leg < go2::kLegCount; ++support_leg)
        {
            const auto &anchor =
                active_terrain_plan->current_support_anchor[support_leg];
            if (anchor.valid &&
                active_terrain_plan->current_support_surface_valid[
                    support_leg] &&
                std::isfinite(active_terrain_plan->
                    current_support_surface_height_world[support_leg]))
            {
                support_surface_heights[support_surface_count++] =
                    active_terrain_plan->
                        current_support_surface_height_world[support_leg];
            }
        }
        double support_surface_z = 0.0;
        if (support_surface_count > 0)
        {
            std::sort(support_surface_heights.begin(),
                      support_surface_heights.begin() +
                          support_surface_count);
            support_surface_z = support_surface_heights[
                support_surface_count / 2];
        }
        const double duty = std::clamp(
            gait_result.duty_factor, 0.35, 0.90);
        const double terrain_plan_period_s =
            active_terrain_plan->gait_period_s;
        const double terrain_plan_duty = std::clamp(
            active_terrain_plan->duty_factor, 0.35, 0.90);
        const double terrain_swing_duration_s =
            (1.0 - terrain_plan_duty) * terrain_plan_period_s;
        const double terrain_plan_knot_dt_s =
            terrain_planner_.config().knot_dt_s;
        const double terrain_time_tolerance_s = 0.5 * std::max(
            1.0e-4, terrain_plan_knot_dt_s);
        const bool terrain_timeline_valid =
            std::isfinite(terrain_plan_period_s) &&
            terrain_plan_period_s > 0.0 &&
            std::isfinite(active_terrain_plan->duty_factor) &&
            std::isfinite(terrain_swing_duration_s) &&
            terrain_swing_duration_s > terrain_time_tolerance_s;


        // In CRAWL_STEP use the map-measured script target and a fixed
        // deadline. The surrounding plan remains the MPC/WBC validation
        // snapshot; this removes asynchronous candidate/timing handoff.
        go2_terrain::TerrainFootholdPrediction scripted_target{};

        const auto find_planned_foothold =
            [&](std::size_t leg,
                bool leg_in_swing,
                const go2_terrain::TerrainFootholdPrediction *&planned,
                double &swing_start_time_s) {
                planned = nullptr;
                swing_start_time_s =
                    std::numeric_limits<double>::infinity();
                if (leg >= go2::kLegCount || !terrain_timeline_valid)
                    return false;
                // The sequencer is the event owner. Its measured target is
                // authoritative as soon as SWING/COMMIT publishes it; do not
                // gate this handoff on the legacy state machine's one-tick
                // COM/SWING phase, or an old planner target can win.
                const auto sequencer_state = sequencer_output.state;
                const bool sequencer_swing =
                    sequencer_state ==
                        go2_terrain::TerrainCrawlSequencerState::kSwing ||
                    sequencer_state ==
                        go2_terrain::TerrainCrawlSequencerState::kCommit;
                const bool scripted_step =
                    terrain_transfer_window_active_ &&
                    sequencer_swing && sequencer_output.target_valid &&
                    sequencer_output.active_leg == leg;
                if (scripted_step)
                {
                    // Direct measured-map target; planner candidates are not
                    // consulted for this event-driven crawl leg.
                    scripted_target.valid = true;
                    scripted_target.touchdown = true;
                    // This event was selected by the terrain sequencer as a
                    // raised-surface transfer. Preserve that transaction
                    // witness when adapting the site-frame target back into
                    // the planner prediction type; otherwise preparation
                    // rejects the valid target as a non-transition foothold.
                    scripted_target.surface_transition_required = true;
                    // TerrainFootholdPrediction carries contact-patch
                    // coordinates, while the sequencer publishes the
                    // foot-site center consumed by FK/WBC. Convert back at
                    // this typed boundary so prepare_terrain_target applies
                    // the site offset exactly once.
                    scripted_target.position_world =
                        go2::FootSiteToContactPatch(
                            sequencer_output.target_world);
                    scripted_target.swing_lift_m = 0.03;
                    scripted_target.edge_margin_m = 0.03;
                    scripted_target.touchdown_time_s = terrain_now_s +
                        go2_terrain::TerrainCrawlScript::kSwingDurationS;
                    scripted_target.swing_duration_s =
                        go2_terrain::TerrainCrawlScript::kSwingDurationS;
                    scripted_target.swing_peak_phase =
                        go2_terrain::TerrainCrawlScript::kSwingApexPhase;
                    scripted_target.swing_leading_edge_phase =
                        go2_terrain::TerrainCrawlScript::kSwingApexPhase;
                    scripted_target.swing_leading_edge_phase_valid = true;
                    swing_start_time_s = terrain_now_s;
                    planned = &scripted_target;
                    return true;
                }
                double best_touchdown_time_s =
                    std::numeric_limits<double>::infinity();
                for (std::size_t k = 0;
                     k < active_terrain_plan->horizon_knots; ++k)
                {
                    const auto &foot =
                        active_terrain_plan->predicted_foothold[k][leg];
                    if (!foot.valid || !foot.touchdown ||
                        !std::isfinite(foot.touchdown_time_s) ||
                        !std::isfinite(active_terrain_plan->valid_until_s) ||
                        foot.touchdown_time_s >
                            active_terrain_plan->valid_until_s)
                        continue;
                    const double planned_swing_duration_s =
                        std::isfinite(foot.swing_duration_s) &&
                                foot.swing_duration_s >
                                    terrain_time_tolerance_s
                            ? foot.swing_duration_s
                            : terrain_swing_duration_s;
                    const double candidate_swing_start_s =
                        foot.touchdown_time_s - planned_swing_duration_s;
                    // A past swing start may still be adopted while the
                    // touchdown remains ahead: the stored planner start
                    // anchors this late handoff.  A stance leg must still
                    // have a future swing window.
                    if (foot.touchdown_time_s + terrain_time_tolerance_s <
                            terrain_now_s ||
                        (!leg_in_swing && candidate_swing_start_s +
                            terrain_time_tolerance_s < terrain_now_s))
                        continue;
                    if (foot.touchdown_time_s < best_touchdown_time_s)
                    {
                        planned = &foot;
                        best_touchdown_time_s = foot.touchdown_time_s;
                        swing_start_time_s = candidate_swing_start_s;
                    }
                }
                return planned != nullptr;
            };

        const auto terrain_height_change_for =
            [&](std::size_t leg,
                const go2_terrain::TerrainFootholdPrediction &planned) {
                if (terrain_surface_transition_active_ &&
                    terrain_surface_transition_required_[leg] &&
                    !terrain_surface_transition_committed_[leg] &&
                    terrain_surface_transition_source_valid_[leg])
                {
                    return planned.position_world.z -
                            terrain_surface_transition_source_world_z_[leg] >
                        terrain_surface_transition_deadband_m_;
                }
                const bool measured_leg_surface_valid =
                    active_terrain_plan->current_support_surface_valid[leg] &&
                    std::isfinite(active_terrain_plan->
                        current_support_surface_height_world[leg]);
                const double terrain_surface_reference_z =
                    measured_leg_surface_valid
                        ? active_terrain_plan->
                              current_support_surface_height_world[leg]
                        : support_surface_z;
                const bool terrain_surface_reference_valid =
                    measured_leg_surface_valid || support_surface_count > 0;
                const double terrain_surface_delta =
                    terrain_surface_reference_valid
                        ? planned.position_world.z -
                            terrain_surface_reference_z
                        : 0.0;
                const double terrain_height_deadband = std::max(
                    2.0 * std::max(0.0, planned.uncertainty_m),
                    0.5 * terrain_planner_.config().feasibility.
                        foot_patch_radius_m);
                return terrain_surface_reference_valid &&
                    terrain_surface_delta > terrain_height_deadband;
            };

        const auto begin_terrain_surface_transition =
            [&](std::size_t transition_leg,
                double target_world_z, double uncertainty_m) {
                if (transition_leg >= go2::kLegCount)
                    return;
                const double deadband_m = std::max(
                    2.0 * std::max(0.0, uncertainty_m),
                    0.5 * terrain_planner_.config().feasibility.
                        foot_patch_radius_m);
                const bool previously_committed =
                    terrain_surface_transition_committed_surface_valid_[
                        transition_leg] &&
                    std::isfinite(
                        terrain_surface_transition_committed_surface_world_z_[
                            transition_leg]) &&
                    std::abs(
                        terrain_surface_transition_committed_surface_world_z_[
                            transition_leg] - target_world_z) <= deadband_m;
                if (previously_committed)
                {
                    if (terrain_surface_transition_active_)
                    {
                        terrain_surface_transition_source_valid_[
                            transition_leg] = true;
                        terrain_surface_transition_source_world_z_[
                            transition_leg] =
                            terrain_surface_transition_committed_surface_world_z_[
                                transition_leg];
                        terrain_surface_transition_required_[transition_leg] =
                            false;
                        terrain_surface_transition_committed_[transition_leg] =
                            true;
                    }
                    return;
                }
                if (!terrain_surface_transition_active_)
                {
                    terrain_surface_transition_active_ = true;
                    // V2 window starts from the first sensor-derived surface
                    // transition intent and remains active through the
                    // stable post-crossing passage.
                    terrain_transfer_window_active_ = true;
                    if (terrain_crawl_state_machine_.state() ==
                        go2_terrain::TerrainCrawlState::kInactive)
                        terrain_crawl_state_machine_.Enter(terrain_now_s);
                    terrain_crawl_min_contact_count_ = go2::kLegCount;
                    terrain_crawl_step_commit_count_ = 0;
                    terrain_transfer_window_release_s_ =
                        -std::numeric_limits<double>::infinity();
                    terrain_surface_transition_target_world_z_ = target_world_z;
                    terrain_surface_transition_deadband_m_ = deadband_m;
                    terrain_surface_transition_required_.fill(false);
                    terrain_surface_transition_original_required_.fill(false);
                    terrain_surface_transition_cancelled_.fill(false);
                    // The measured commit ledger spans successive atomic
                    // transactions in this window. Do not clear it here:
                    // FL's commit must remain visible while FR is prepared.
                    terrain_surface_transition_source_valid_.fill(false);
                }
                else
                {
                    terrain_surface_transition_target_world_z_ = std::max(
                        terrain_surface_transition_target_world_z_,
                        target_world_z);
                    terrain_surface_transition_deadband_m_ = std::max(
                        terrain_surface_transition_deadband_m_, deadband_m);
                }
                double source_world_z = support_surface_z;
                bool source_valid = support_surface_count > 0;
                if (active_terrain_plan->current_support_surface_valid[
                        transition_leg] &&
                    std::isfinite(active_terrain_plan->
                        current_support_surface_height_world[transition_leg]))
                {
                    source_world_z = active_terrain_plan->
                        current_support_surface_height_world[transition_leg];
                    source_valid = true;
                }
                terrain_surface_transition_source_valid_[transition_leg] =
                    source_valid;
                terrain_surface_transition_source_world_z_[transition_leg] =
                    source_world_z;
                terrain_surface_transition_required_[transition_leg] =
                    !source_valid ||
                    std::abs(source_world_z - target_world_z) >
                        terrain_surface_transition_deadband_m_;
                if (terrain_surface_transition_required_[transition_leg])
                    terrain_surface_transition_original_required_[transition_leg] =
                        true;
            };

        const auto prepare_terrain_target =
            [&](std::size_t leg,
                const go2_terrain::TerrainFootholdPrediction &planned,
                double swing_start_time_s,
                bool current_leg_in_swing,
                TerrainSwingExecution &execution) {
                ++terrain_target_prepare_attempt_count_;
                const auto reject_prepare = [&](int failure) {
                    execution = {};
                    ++terrain_target_prepare_rejection_count_;
                    terrain_target_last_prepare_failure_ = failure;
                    if (leg < go2::kLegCount)
                        terrain_target_last_prepare_failure_by_leg_[leg] =
                            failure;
                    return false;
                };
                if (!std::isfinite(swing_start_time_s) ||
                    !std::isfinite(planned.touchdown_time_s) ||
                    !std::isfinite(planned.position_world.x) ||
                    !std::isfinite(planned.position_world.y) ||
                    !std::isfinite(planned.position_world.z))
                    return reject_prepare(1);
                go2::Vec3 start_world{};
                bool start_valid = false;
                bool time_rebased_at_handoff = false;
                if (current_leg_in_swing && have_actual_world_feet &&
                    std::isfinite(actual_world_feet[leg].x) &&
                    std::isfinite(actual_world_feet[leg].y) &&
                    std::isfinite(actual_world_feet[leg].z))
                {
                    // A late sensor plan must attach to the measured foot at
                    // the handoff instant.  Reusing the planner's old phase
                    // would create a position jump before the first lift.
                    start_world = actual_world_feet[leg];
                    start_valid = true;
                    time_rebased_at_handoff = true;
                }
                if (!start_valid &&
                    planned.swing_start_position_valid &&
                    std::isfinite(planned.swing_start_position_world.x) &&
                    std::isfinite(planned.swing_start_position_world.y) &&
                    std::isfinite(planned.swing_start_position_world.z))
                {
                    start_world = planned.swing_start_position_world;
                    start_valid = true;
                }
                const double plan_leg_phase = go2_control::GaitLegPhase(
                    leg, active_terrain_plan->gait_phase,
                    runtime_gait_pattern_);
                const bool plan_captured_in_stance =
                    std::isfinite(plan_leg_phase) &&
                    plan_leg_phase < terrain_plan_duty;
                const auto &plan_start =
                    active_terrain_plan->swing_start_position_world[leg];
                if (!start_valid && plan_captured_in_stance &&
                    active_terrain_plan->swing_start_position_valid[leg] &&
                    std::isfinite(plan_start.x) &&
                    std::isfinite(plan_start.y) &&
                    std::isfinite(plan_start.z))
                {
                    start_world = plan_start;
                    start_valid = true;
                }
                // A deferred target cannot use an in-flight foot as the
                // origin of its next swing.  Capture a measured support
                // anchor only while the leg is actually in stance.
                if (!start_valid && !current_leg_in_swing)
                {
                    if (support_anchor_valid_[leg] &&
                        std::isfinite(support_anchor_world_feet_[leg].x) &&
                        std::isfinite(support_anchor_world_feet_[leg].y) &&
                        std::isfinite(support_anchor_world_feet_[leg].z))
                    {
                        start_world = support_anchor_world_feet_[leg];
                        start_valid = true;
                    }
                    else if (have_actual_world_feet &&
                             std::isfinite(actual_world_feet[leg].x) &&
                             std::isfinite(actual_world_feet[leg].y) &&
                             std::isfinite(actual_world_feet[leg].z))
                    {
                        start_world = actual_world_feet[leg];
                        start_valid = true;
                    }
                }
                if (!start_valid)
                    return reject_prepare(2);
                execution = {};
                execution.valid = true;
                execution.plan_id = active_terrain_plan->plan_id;
                execution.map_epoch = active_terrain_plan->map_epoch;
                execution.start_world = start_world;
                // Planner targets are contact-patch coordinates; execution
                // and FK/WBC use the calibrated foot-site coordinate.
                execution.target_world = go2::ContactPatchToFootSite(
                    planned.position_world);
                execution.nominal_touchdown_time_s =
                    planned.touchdown_time_s;
                execution.trajectory_start_time_s =
                    time_rebased_at_handoff ? terrain_now_s :
                                               swing_start_time_s;
                execution.swing_lift_m = std::max(0.0, planned.swing_lift_m);
                const double committed_swing_duration_s =
                    std::isfinite(planned.swing_duration_s) &&
                            planned.swing_duration_s >
                                terrain_time_tolerance_s
                        ? planned.swing_duration_s
                        : planned.touchdown_time_s - swing_start_time_s;
                const double available_swing_duration_s =
                    planned.touchdown_time_s -
                    execution.trajectory_start_time_s;
                const double required_swing_duration_s =
                    go2_terrain::TerrainSwingDurationForPath(
                        available_swing_duration_s,
                        start_world, execution.target_world,
                        execution.swing_lift_m,
                        terrain_planner_.config().feasibility.
                            max_swing_speed_mps);
                if (!std::isfinite(committed_swing_duration_s) ||
                    committed_swing_duration_s <= terrain_time_tolerance_s ||
                    !std::isfinite(available_swing_duration_s) ||
                    available_swing_duration_s <= terrain_time_tolerance_s ||
                    !std::isfinite(required_swing_duration_s) ||
                    required_swing_duration_s >
                        available_swing_duration_s +
                            terrain_time_tolerance_s)
                {
                    return reject_prepare(3);
                }
                // The planner committed this absolute touchdown time.  Do
                // not replace it with a consumer-side duration extension:
                // the contact schedule, MPC preview, WBC transaction, and
                // swing trajectory must all end at this same event.
                execution.planned_swing_duration_s =
                    committed_swing_duration_s;
                execution.terrain_swing_duration_s =
                    available_swing_duration_s;
                execution.swing_duration_s = available_swing_duration_s;
                execution.swing_start_time_s =
                    execution.trajectory_start_time_s;
                execution.touchdown_time_s = planned.touchdown_time_s;
                execution.swing_peak_phase = std::clamp(
                    planned.swing_peak_phase, 0.10, 0.90);
                execution.swing_leading_edge_phase = std::clamp(
                    planned.swing_leading_edge_phase, 0.10, 0.75);
                execution.swing_leading_edge_phase_valid =
                    planned.swing_leading_edge_phase_valid;
                execution.time_rebased_at_handoff = time_rebased_at_handoff;
                execution.terrain_height_change =
                    terrain_height_change_for(leg, planned);
                // A safe-region center is a sensor-grid representative, not
                // automatically a terrain-induced foothold displacement.  A
                // plan may carry future flat-surface representatives for
                // MPC/support prediction, but only an observed surface-height
                // transition may retarget the actuation trajectory.  This
                // keeps map quantization from hijacking the Phase 1 gait and
                // prevents an early world endpoint from becoming an
                // unreachable held stance before the riser.
                const bool sequencer_owned_target =
                    terrain_transfer_window_active_ &&
                    (terrain_crawl_sequencer_output_.state ==
                         go2_terrain::TerrainCrawlSequencerState::kSwing ||
                     terrain_crawl_sequencer_output_.state ==
                         go2_terrain::TerrainCrawlSequencerState::kCommit) &&
                    terrain_crawl_sequencer_output_.target_valid &&
                    terrain_crawl_sequencer_output_.active_leg == leg;
                execution.terrain_target_required =
                    sequencer_owned_target ||
                    planned.surface_transition_required ||
                    execution.terrain_height_change;
                if (!execution.terrain_target_required)
                    return reject_prepare(4);
                begin_terrain_surface_transition(
                    leg, go2::FootSiteToContactPatch(execution.target_world).z,
                    planned.uncertainty_m);
                // The planner's reachability test is made at its predicted
                // touchdown body pose.  A committed snapshot can outlive a
                // body-height/pose change, however; applying its world
                // endpoint against the current pose can make the following
                // AllLegInverseKinematicsClamped call fail.  Revalidate an
                // in-flight endpoint in the measured current frame and let
                // the nominal gait continue if the old transaction no longer
                // has a realizable endpoint.
                if (current_leg_in_swing)
                {
                    const go2::Vec3 target_base = go2_control::WorldToBody(
                        terrain_pose.base, terrain_pose.quaternion,
                        planned.position_world);
                    go2::LegJointPositions joints;
                    (void)go2::LegInverseKinematics(
                        static_cast<go2::Leg>(leg), target_base, joints);
                    // Reachability was checked by the planner at its
                    // predicted handoff pose. The measured crawl pose may be
                    // marginally outside that nominal envelope; retain the
                    // immutable endpoint and let the clamped WBC apply it
                    // rather than discarding the live transaction.
                }
                ++terrain_target_prepared_count_;
                terrain_target_last_prepare_failure_ = 0;
                terrain_target_last_prepare_failure_by_leg_[leg] = 0;
                return true;
            };

        const auto apply_world_target =
            [&](std::size_t leg,
                const go2::Vec3 &target_world,
                const go2::Vec3 &target_world_velocity) {
                // Cartesian WBC owns the actual swing task. Keep its world
                // target and velocity on the exact terrain trajectory that
                // is also converted to joint targets below; changing only
                // `feet` would leave WBC tracking the old nominal swing.
                terrain_command_velocity_world_[leg] = target_world_velocity;
                if (params_.cartesian_world && have_commanded_world_feet_)
                {
                    if (terrain_target_xy_enabled)
                    {
                        commanded_world_feet_[leg].x = target_world.x;
                        commanded_world_feet_[leg].y = target_world.y;
                        cartesian_state_.target_world_vel[leg].x =
                            target_world_velocity.x;
                        cartesian_state_.target_world_vel[leg].y =
                            target_world_velocity.y;
                    }
                    if (terrain_target_z_enabled)
                    {
                        commanded_world_feet_[leg].z = target_world.z;
                        cartesian_state_.target_world_vel[leg].z =
                            target_world_velocity.z;
                    }
                }
                const go2::Vec3 target_base = go2_control::WorldToBody(
                    terrain_pose.base, terrain_pose.quaternion, target_world);
                bool overridden = false;
                if (terrain_target_xy_enabled)
                {
                    feet[leg].x = target_base.x;
                    feet[leg].y = target_base.y;
                    overridden = true;
                }
                if (terrain_target_z_enabled)
                {
                    feet[leg].z = target_base.z;
                    overridden = true;
                }
                terrain_target_overridden =
                    terrain_target_overridden || overridden;
            };

        const auto begin_terrain_transfer_hold =
            [&](std::size_t target_leg) {
            std::array<bool, go2::kLegCount> scheduled_support{};
            if (terrain_contact_now_valid)
                scheduled_support = terrain_contact_now;
            else
            {
                for (std::size_t support_leg = 0;
                     support_leg < go2::kLegCount; ++support_leg)
                {
                    scheduled_support[support_leg] =
                        go2_control::GaitLegPhase(
                            support_leg, phase, runtime_gait_pattern_) < duty;
                }
            }
            const int support_count = static_cast<int>(std::count(
                scheduled_support.begin(), scheduled_support.end(), true));
            if (terrain_transfer_hold_active_)
            {
                // Do not replace the captured support with the next nominal
                // diagonal: that would reintroduce a two-contact plant while
                // the terrain target is still endpoint-held/uncommitted.
                terrain_transfer_hold_contact_ =
                    go2_terrain::TerrainTransferHoldSupport(
                        terrain_transfer_hold_contact_, scheduled_support, true);
                return;
            }
            if (support_count < 2)
                return;
            terrain_transfer_hold_contact_ =
                go2_terrain::TerrainTransferHoldSupport(
                    terrain_transfer_hold_contact_, scheduled_support, false);
            terrain_transfer_hold_active_ = true;
        };

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const double leg_phase = go2_control::GaitLegPhase(
                leg, phase, runtime_gait_pattern_);
            const bool nominal_leg_in_swing = leg_phase >= duty;
            const bool leg_in_swing = terrain_contact_now_valid
                ? !terrain_contact_now[leg]
                : nominal_leg_in_swing;
            const bool terrain_crawl_execution =
                terrain_crawl_control_authority_active &&
                terrain_crawl_state_machine_.UsesCrawlExecution();
            const bool terrain_crawl_staging = terrain_crawl_execution &&
                terrain_crawl_state_machine_.state() ==
                    go2_terrain::TerrainCrawlState::kStage;
            const bool explicit_crawl_step = terrain_crawl_execution &&
                terrain_crawl_state_machine_.state() ==
                    go2_terrain::TerrainCrawlState::kCrawlStep &&
                (terrain_crawl_sequencer_output_.state ==
                     go2_terrain::TerrainCrawlSequencerState::kSwing ||
                 terrain_crawl_sequencer_output_.state ==
                     go2_terrain::TerrainCrawlSequencerState::kCommit);
            const bool crawl_shift_leg =
                terrain_crawl_execution && !explicit_crawl_step &&
                terrain_crawl_state_machine_.com_target_leg() == leg;
            auto &execution = terrain_swing_execution_[leg];
            auto &pending = terrain_swing_pending_[leg];
            const bool sequencer_direct_target =
                terrain_transfer_window_active_ &&
                (terrain_crawl_sequencer_output_.state ==
                     go2_terrain::TerrainCrawlSequencerState::kSwing ||
                 terrain_crawl_sequencer_output_.state ==
                     go2_terrain::TerrainCrawlSequencerState::kCommit) &&
                terrain_crawl_sequencer_output_.target_valid &&
                terrain_crawl_sequencer_output_.active_leg == leg;
            if (sequencer_direct_target && execution.valid)
            {
                const auto &sequencer_target =
                    terrain_crawl_sequencer_output_.target_world;
                const double target_frame_error = std::hypot(
                    std::hypot(execution.target_world.x - sequencer_target.x,
                               execution.target_world.y - sequencer_target.y),
                    execution.target_world.z - sequencer_target.z);
                const auto &sequencer_start =
                    terrain_crawl_sequencer_output_.swing_start_world;
                const double start_frame_error = std::hypot(
                    std::hypot(execution.start_world.x - sequencer_start.x,
                               execution.start_world.y - sequencer_start.y),
                    execution.start_world.z - sequencer_start.z);
                if (target_frame_error > 1.0e-6 ||
                    start_frame_error > 1.0e-3)
                {
                    // The sequencer owns this event. A planner foothold may
                    // have been prepared before the event boundary; retaining
                    // even an equal endpoint with an old start would make the
                    // WBC corner-catch test evaluate the old timeline and
                    // reject the first event tick as a false early touchdown.
                    // Rebind both endpoint and origin at the explicit event
                    // boundary so the target is converted exactly once below.
                    execution = {};
                    pending = {};
                }
            }
            // A scripted timeout owns a bounded retract/retry. Clear only
            // the uncommitted active endpoint; committed feet remain anchors.
            if (terrain_crawl_control_authority_active &&
                terrain_crawl_state_machine_.state() ==
                    go2_terrain::TerrainCrawlState::kShiftCom &&
                terrain_crawl_state_machine_.retry_count() > 0 &&
                terrain_crawl_state_machine_.com_target_leg() == leg &&
                execution.valid && !execution.measured_touchdown)
            {
                execution = {};
                pending = {};
            }
            // A target prepared during SHIFT_COM belongs to the emergent
            // gait timeline. Once the state machine grants CRAWL_STEP, keep
            // its already validated endpoint but retime the handoff to the
            // fixed script deadline. Dropping it would make the next map
            // snapshot's optional script target a flaky launch dependency.
            if (explicit_crawl_step && execution.valid &&
                !execution.in_flight && !execution.endpoint_held)
            {
                execution.trajectory_start_time_s = terrain_now_s;
                execution.swing_start_time_s = terrain_now_s;
                execution.nominal_touchdown_time_s = terrain_now_s +
                    go2_terrain::TerrainCrawlScript::kSwingDurationS;
                execution.touchdown_time_s =
                    execution.nominal_touchdown_time_s;
                execution.swing_duration_s =
                    go2_terrain::TerrainCrawlScript::kSwingDurationS;
                execution.terrain_swing_duration_s =
                    execution.swing_duration_s;
                execution.planned_swing_duration_s =
                    execution.swing_duration_s;
                execution.swing_peak_phase =
                    go2_terrain::TerrainCrawlScript::kSwingApexPhase;
                execution.swing_leading_edge_phase =
                    go2_terrain::TerrainCrawlScript::kSwingApexPhase;
                execution.swing_leading_edge_phase_valid = true;
                execution.time_rebased_at_handoff = true;
                pending = {};
            }
            const bool explicit_active_leg =
                go2_terrain::TerrainCrawlSwingStillInFlight(
                    explicit_crawl_step,
                    terrain_crawl_state_machine_.ActiveLeg(), leg,
                    execution.valid, execution.in_flight, terrain_now_s,
                    execution.touchdown_time_s, terrain_time_tolerance_s);
            const auto crawl_state = terrain_crawl_state_machine_.state();
            if (terrain_crawl_staging)
                continue;
            if (terrain_crawl_execution && !explicit_crawl_step &&
                !crawl_shift_leg)
            {
                if (have_actual_world_feet)
                    feet[leg] = go2_control::WorldToBody(
                        terrain_pose.base, terrain_pose.quaternion,
                        actual_world_feet[leg]);
                continue;
            }
            // Before the creep handoff, leave all terrain transactions alone
            // and let the configured trot generate its ordinary swing.
            if (terrain_transfer_window_active_ &&
                !terrain_crawl_control_authority_active)
                continue;
            if (explicit_crawl_step &&
                terrain_crawl_state_machine_.ActiveLeg() != leg)
            {
                if (have_actual_world_feet)
                    feet[leg] = go2_control::WorldToBody(
                        terrain_pose.base, terrain_pose.quaternion,
                        actual_world_feet[leg]);
                continue;
            }
            const bool terrain_extended_swing =
                execution.valid &&
                execution.terrain_target_required &&
                execution.in_flight &&
                !execution.endpoint_held &&
                std::isfinite(execution.touchdown_time_s) &&
                terrain_now_s + terrain_time_tolerance_s <
                    execution.touchdown_time_s;
            const bool effective_leg_in_swing = sequencer_direct_target
                ? true
                : (explicit_crawl_step
                       ? explicit_active_leg
                       : (crawl_shift_leg ? false
                                           : (leg_in_swing ||
                                              terrain_extended_swing)));
            const bool terrain_target_pending = execution.valid &&
                execution.terrain_target_required &&
                !execution.measured_touchdown;
            if ((terrain_surface_transition_active_ ||
                 terrain_transfer_hold_active_) &&
                execution.valid && execution.measured_touchdown)
            {
                apply_world_target(leg, execution.target_world, go2::Vec3{});
                continue;
            }
            if (terrain_transfer_hold_active_ &&
                !terrain_target_pending &&
                terrain_transfer_hold_contact_[leg] &&
                !crawl_shift_leg &&
                !sequencer_direct_target &&
                have_actual_world_feet)
            {
                // A support foot that was loaded when the terrain transfer
                // began remains an anchor even if the nominal phase reaches
                // its swing interval.  Defer that leg's next terrain target
                // until the current target has been measured.
                feet[leg] = go2_control::WorldToBody(
                    terrain_pose.base, terrain_pose.quaternion,
                    actual_world_feet[leg]);
                continue;
            }
            if (!effective_leg_in_swing)
            {
                if (execution.valid && execution.in_flight)
                {
                    // The terrain-conditioned trajectory reached its own
                    // feasible touchdown time.  This is still not a
                    // measured-contact assertion.
                    execution.in_flight = false;
                    execution.endpoint_held = true;
                }
                if (execution.valid && execution.endpoint_held)
                {
                    if (execution.terrain_target_required)
                        apply_world_target(
                            leg, execution.target_world, go2::Vec3{});
                    if (!pending.valid)
                    {
                        const go2_terrain::TerrainFootholdPrediction *planned =
                            nullptr;
                        double swing_start_time_s = 0.0;
                        if (find_planned_foothold(
                                leg, false, planned, swing_start_time_s) &&
                            planned != nullptr)
                        {
                            prepare_terrain_target(
                                leg, *planned, swing_start_time_s, false,
                                pending);
                        }
                    }
                    continue;
                }
                if (!execution.valid)
                {
                    const go2_terrain::TerrainFootholdPrediction *planned =
                        nullptr;
                    double swing_start_time_s = 0.0;
                    if (find_planned_foothold(
                            leg, false, planned, swing_start_time_s) &&
                        planned != nullptr)
                    {
                        prepare_terrain_target(
                            leg, *planned, swing_start_time_s, false,
                            execution);
                    }
                }
                continue;
            }

            if (execution.endpoint_held &&
                terrain_transfer_hold_active_ &&
                !explicit_crawl_step &&
                execution.terrain_target_required &&
                !execution.measured_touchdown)
            {
                // Do not start the next nominal swing while this target is
                // still waiting for live touchdown confirmation.  The
                // endpoint remains the swing task until the WBC transaction
                // promotes it to measured support.
                apply_world_target(
                    leg, execution.target_world, go2::Vec3{});
                continue;
            }
            if (execution.endpoint_held)
            {
                if (pending.valid)
                {
                    execution = pending;
                    pending = {};
                }
                else
                {
                    execution = {};
                }
            }
            if (!execution.valid)
            {
                const go2_terrain::TerrainFootholdPrediction *planned = nullptr;
                double swing_start_time_s = 0.0;
                if (find_planned_foothold(
                        leg, true, planned, swing_start_time_s) &&
                    planned != nullptr)
                {
                    prepare_terrain_target(
                        leg, *planned, swing_start_time_s, true, execution);
                }
            }
            if (execution.valid && !execution.endpoint_held &&
                !execution.in_flight &&
                !execution.time_rebased_at_handoff &&
                !terrain_transfer_window_active_)
            {
                // A plan may be prepared while the leg is in stance, but
                // the body and the loaded foot can move before the actual
                // gait swing boundary.  The touchdown target is immutable;
                // the swing origin must be the measured foot at that
                // boundary, otherwise the first terrain sample jumps from
                // a stale planner-time anchor.
                if (have_actual_world_feet &&
                    std::isfinite(actual_world_feet[leg].x) &&
                    std::isfinite(actual_world_feet[leg].y) &&
                    std::isfinite(actual_world_feet[leg].z))
                {
                    execution.start_world = actual_world_feet[leg];
                    execution.trajectory_start_time_s = terrain_now_s;
                    execution.swing_start_time_s = terrain_now_s;
                    const double available_swing_duration_s =
                        execution.nominal_touchdown_time_s - terrain_now_s;
                    const double required_path_duration_s =
                        go2_terrain::TerrainSwingDurationForPath(
                            available_swing_duration_s,
                            execution.start_world,
                            execution.target_world,
                            execution.swing_lift_m,
                            terrain_planner_.config().feasibility.
                                max_swing_speed_mps);
                    const double required_swing_duration_s = std::max(
                        execution.planned_swing_duration_s,
                        required_path_duration_s);
                    if (!std::isfinite(available_swing_duration_s) ||
                        available_swing_duration_s <=
                            terrain_time_tolerance_s ||
                        !std::isfinite(required_swing_duration_s) ||
                        required_swing_duration_s >
                            available_swing_duration_s +
                                terrain_time_tolerance_s)
                    {
                        int required_mask_before = 0;
                        for (std::size_t required_leg = 0;
                             required_leg < go2::kLegCount; ++required_leg)
                            if (terrain_surface_transition_required_[
                                    required_leg])
                                required_mask_before |=
                                    1 << static_cast<int>(required_leg);
                        const bool cancelled_requirement =
                            go2_terrain::MarkTerrainTransitionLegCancelled(
                                terrain_surface_transition_required_,
                                terrain_surface_transition_committed_,
                                terrain_surface_transition_cancelled_,
                                terrain_surface_transition_source_valid_,
                                leg);
                        int required_mask_after = 0;
                        for (std::size_t required_leg = 0;
                             required_leg < go2::kLegCount; ++required_leg)
                            if (terrain_surface_transition_required_[
                                    required_leg])
                                required_mask_after |=
                                    1 << static_cast<int>(required_leg);
                        if (Full2EnvDouble(
                                "TROT_TERRAIN_DEBUG_TRANSACTION", 0.0) >
                                0.5)
                        {
                            std::cout << "Terrain transaction event=cancelled"
                                      << " t=" << terrain_now_s
                                      << " leg=" << leg
                                      << " touchdown="
                                      << execution.nominal_touchdown_time_s
                                      << " available="
                                      << available_swing_duration_s
                                      << " required_swing="
                                      << required_swing_duration_s
                                      << " required_before="
                                      << required_mask_before
                                      << " required_after="
                                      << required_mask_after
                                      << " original_required="
                                      << required_mask_before
                                      << " cancelled="
                                      << (cancelled_requirement ? 1 : 0)
                                      << " failure=6\n";
                        }
                        execution = {};
                        ++terrain_target_prepare_rejection_count_;
                        terrain_target_last_prepare_failure_ = 6;
                        terrain_target_last_prepare_failure_by_leg_[leg] = 6;
                        continue;
                    }
                    execution.terrain_swing_duration_s =
                        available_swing_duration_s;
                    execution.swing_duration_s = available_swing_duration_s;
                    execution.touchdown_time_s =
                        execution.nominal_touchdown_time_s;
                    execution.time_rebased_at_handoff = true;
                }
            }
            if (!execution.valid || !execution.terrain_target_required ||
                terrain_now_s + terrain_time_tolerance_s <
                    execution.swing_start_time_s)
            {
                // The explicit sequencer trajectory is already a validated
                // foot-site/world target. Do not silently fall back to the
                // nominal gait while its bookkeeping transaction is waiting
                // for a planner snapshot; that was the immobile-leg failure.
                if (sequencer_direct_target &&
                    terrain_crawl_sequencer_output_.state ==
                        go2_terrain::TerrainCrawlSequencerState::kSwing)
                {
                    apply_world_target(
                        leg, terrain_crawl_sequencer_output_.swing_position_world,
                        terrain_crawl_sequencer_output_.swing_velocity_world);
                }
                continue;
            }

            if (terrain_crawl_control_authority_active)
                begin_terrain_transfer_hold(leg);
            if (terrain_crawl_control_authority_active &&
                !explicit_crawl_step && !sequencer_direct_target)
            {
                // Scripted targets are prepared ahead of the fixed swing, but
                // must not inherit a nominal trot flight during entry or COM
                // shift. Preserve the validated endpoint for the explicit
                // CRAWL_STEP handoff instead of invalidating it at the riser.
                execution.in_flight = false;
                execution.endpoint_held = false;
                continue;
            }
            if (terrain_transfer_hold_active_ &&
                terrain_transfer_hold_contact_[leg])
            {
                int alternate_support_count = 0;
                for (std::size_t support_leg = 0;
                     support_leg < go2::kLegCount; ++support_leg)
                {
                    if (support_leg != leg &&
                        terrain_transfer_hold_contact_[support_leg])
                        ++alternate_support_count;
                }
                if (alternate_support_count < 2)
                {
                    if (have_actual_world_feet)
                        feet[leg] = go2_control::WorldToBody(
                            terrain_pose.base, terrain_pose.quaternion,
                            actual_world_feet[leg]);
                    continue;
                }
            }
            execution.in_flight = true;
            const double swing_phase = execution.time_rebased_at_handoff
                ? std::clamp(
                      (terrain_now_s - execution.trajectory_start_time_s) /
                          std::max(1.0e-6, execution.swing_duration_s),
                      0.0, 1.0)
                : std::clamp(
                      (leg_phase - duty) / std::max(1.0e-6, 1.0 - duty),
                      0.0, 1.0);
            // A riser is not safely crossed by a centered bell swing when
            // the nominal foot reaches its leading edge late in the swing.
            // The feasibility solver records that sensor-derived edge phase
            // and moves the vertical peak to it; horizontal progress stays
            // continuous so the immutable touchdown endpoint is reachable.
            const double path_progress =
                go2_terrain::TerrainSwingPathProgress(
                    swing_phase,
                    execution.swing_leading_edge_phase_valid,
                    execution.swing_leading_edge_phase);
            const double terrain_peak_phase =
                execution.swing_leading_edge_phase_valid
                    ? std::min(
                          execution.swing_peak_phase,
                          execution.swing_leading_edge_phase)
                    : execution.swing_peak_phase;
            const double swing_arch = execution.swing_lift_m *
                go2_terrain::TerrainSwingProfile(
                    swing_phase, terrain_peak_phase);
            const go2::Vec3 path_world{
                execution.start_world.x + path_progress *
                    (execution.target_world.x - execution.start_world.x),
                execution.start_world.y + path_progress *
                    (execution.target_world.y - execution.start_world.y),
                execution.start_world.z + path_progress *
                    (execution.target_world.z - execution.start_world.z) +
                    swing_arch};
            const double actual_swing_duration_s =
                std::max(
                    terrain_time_tolerance_s,
                    execution.swing_duration_s);
            const double phase_rate = 1.0 / std::max(
                1.0e-6, actual_swing_duration_s);
            const double path_progress_rate =
                go2_terrain::TerrainSwingPathProgressDerivative(
                    swing_phase,
                    execution.swing_leading_edge_phase_valid,
                    execution.swing_leading_edge_phase) * phase_rate;
            const double swing_arch_rate = execution.swing_lift_m *
                go2_terrain::TerrainSwingProfileDerivative(
                    swing_phase, terrain_peak_phase) * phase_rate;
            const go2::Vec3 path_velocity{
                path_progress_rate *
                    (execution.target_world.x - execution.start_world.x),
                path_progress_rate *
                    (execution.target_world.y - execution.start_world.y),
                path_progress_rate *
                    (execution.target_world.z - execution.start_world.z) +
                    swing_arch_rate};
            if (sequencer_output.state ==
                    go2_terrain::TerrainCrawlSequencerState::kSwing &&
                sequencer_output.active_leg == leg)
                apply_world_target(leg, sequencer_output.swing_position_world,
                                   sequencer_output.swing_velocity_world);
            else
                apply_world_target(leg, path_world, path_velocity);
        }
        if (terrain_transfer_hold_active_ && have_actual_world_feet)
        {
            // The phase continues to advance while the terrain foothold is
            // being confirmed. Keep the support feet at measured anchors
            // instead of feeding their nominal swing targets to the servo.
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!terrain_transfer_hold_contact_[leg])
                    continue;
                const auto &execution = terrain_swing_execution_[leg];
                if (execution.valid && execution.in_flight &&
                    execution.terrain_target_required)
                    continue;
                feet[leg] = go2_control::WorldToBody(
                    terrain_pose.base, terrain_pose.quaternion,
                    actual_world_feet[leg]);
            }
        }
        if (terrain_target_overridden)
            ++terrain_gait_target_override_count_;
    }

    if (terrain_transfer_window_active_ &&
        terrain_crawl_state_machine_.state() ==
            go2_terrain::TerrainCrawlState::kShiftCom &&
        !(flat_crawl_debug && terrain_crawl_sequencer_output_.state ==
          go2_terrain::TerrainCrawlSequencerState::kSwing) &&
        have_actual_world_feet)
    {
        // SHIFT_COM has no active swing. Freeze every commanded foot at its
        // measured world anchor, otherwise the newly selected crawl gait
        // can move feet while WBC simultaneously declares them stance.
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            feet[leg] = go2_control::WorldToBody(
                pose.base, pose.quaternion, actual_world_feet[leg]);
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
    // Keep the WBC swing task consistent with the joint target. Clamped IK
    // is needed for a map target beyond the current body pose; if the
    // unclamped terrain point is stored first, WBC pursues an unreachable
    // foot while the actuator command silently retracts it.
    if (params_.wbc_full)
    {
        if (!go2::AllLegInverseKinematicsClamped(feet, joint_targets))
        {
            std::cerr << "Trot IK failed at gait_time=" << gait_time_s << std::endl;
            return false;
        }
    }
    else if (!go2::AllLegInverseKinematics(feet, joint_targets))
    {
        std::cerr << "Trot IK failed at gait_time=" << gait_time_s << std::endl;
        return false;
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
    return true;
}
