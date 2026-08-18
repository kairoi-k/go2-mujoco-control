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
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "motion_frame_utils.h"
#include "full2_campaign_env.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

static_assert(TrotTask::kStandUpDuration == kStandUpDuration);
static_assert(TrotTask::kStandSettleDuration == kStandSettleDuration);
static_assert(TrotTask::kStandDownDuration == kStandDownDuration);
static_assert(TrotTask::kStopTransitionDuration == kStopTransitionDuration);
static_assert(TrotTask::kFinalHoldDuration == kFinalHoldDuration);
static_assert(TrotTask::kGaitBlendDuration == kGaitBlendDuration);

// CONTROL LOOP — 500Hz LowCmdWrite state machine (see docs/CODE_GUIDE.md)

// --- TrotExperiment::LowCmdWrite ---
void TrotExperiment::LowCmdWrite()
{
    // ROADMAP LowCmdWrite: stand-up -> gait -> wbc -> limits -> lie-down/stop -> publish
    // Jump via SECTION: markers below.
    // AUTO-TOC (line numbers drift if edited; search SECTION:)
    //   L60: lie-down (see PhaseLieDown)
    //   L68: stand-settle
    //   L72: start-gait
    //   L78: gait-run
    //   L85: stop-to-stand
    //   L92: hard-limits
    //   L131: publish-lowcmd
    //   L133: log-sample

    if (finished_.load())
        return;

    unitree_go::msg::dds_::LowState_ state_snapshot{};
    unitree_go::msg::dds_::SportModeState_ high_state_snapshot{};
    bool have_state = false;
    bool have_high_state = false;
    if (!SnapshotState(state_snapshot, high_state_snapshot,
                       have_state, have_high_state))
        return;

    bool motion_clock_paused = false;
    const double motion_dt = MotionClockStep(state_snapshot, motion_clock_paused);

    std::array<double, kMotorCount> joint_targets = task_.stand_up_joint_pos_;
    UpdateVelocityEstimate(state_snapshot, high_state_snapshot, have_high_state, motion_dt);
    std::array<double, kMotorCount> joint_velocities{};
    if (PhaseLieDown(joint_targets))
    {
        // SECTION: lie-down (see PhaseLieDown)
    }
    else if (PhaseStandUp(joint_targets))
    {
        // see PhaseStandUp
    }
    else if (PhaseStandSettle(joint_targets))
    {
        // SECTION: stand-settle
    }
    else if (PhaseStartGait(joint_targets))
    {
        // SECTION: start-gait
    }

    if (PhaseRunGait(state_snapshot, high_state_snapshot,
                     have_high_state, joint_targets))
    {
        // SECTION: gait-run
    }

    if (external_stop_requested_.load())
        task_.stop_requested_ = true;
    if (PhaseStopToStand(joint_targets))
    {
        // SECTION: stop-to-stand
    }
    if (task_.sequence_finished_)
        finished_.store(true);

    double gait_elapsed_s = 0.0;
    const bool wbc_primary_active =
        ComputeWbcPrimaryActive(gait_elapsed_s);

    // SECTION: hard-limits
    if (!CheckInstantaneousHardLimits(
            joint_targets, state_snapshot, have_state,
            wbc_primary_active))
    {
        std::cerr << "Trot hard safety limit reached; stopping\n";
        task_.stop_requested_ = true;
        task_.task_completion_requested_ = false;
        if (task_.stop_start_time_s_ == 0.0)
        {
            task_.stop_start_time_s_ = running_time_;
            task_.stop_origin_joint_targets_ = joint_targets;
            task_.have_stop_origin_joint_targets_ = true;
        }
        joint_targets = task_.stand_up_joint_pos_;
        task_.motion_stage_ = 3;
    }
    std::array<double, kMotorCount> wbc_torque_ff{};
    const bool apply_wbc_torque_ff =
        UpdateWbcShadowAndTorqueFf(
            state_snapshot, have_state,
            high_state_snapshot, have_high_state,
            wbc_torque_ff);

    UpdateJointVelocityFeedforward(
        joint_targets, motion_dt, motion_clock_paused,
        joint_velocities);

    // WBC 主控模式:求解成功且扭矩有效时,扭矩直接作为主命令,
    // 位置伺服降为柔顺约束(kp/kd 小值);否则回退位置控制。
    WriteMotorCommands(
        wbc_primary_active, gait_elapsed_s,
        joint_targets, joint_velocities,
        wbc_torque_ff, apply_wbc_torque_ff);
    UpdateGaitWorldDiagnostics(
        state_snapshot, have_state,
        high_state_snapshot, have_high_state,
        joint_targets);

    // SECTION: publish-lowcmd
    PublishLowCmdWithCrc();
    // SECTION: log-sample
        LogSample(state_snapshot, have_state, high_state_snapshot, have_high_state);
}

void TrotExperiment::PublishLowCmdWithCrc()
{
    low_cmd_.crc() = crc32_core(
        (uint32_t *)&low_cmd_,
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher_->Write(low_cmd_);
}

bool TrotExperiment::PhaseStandUp(std::array<double, go2_trot::kMotorCount> &joint_targets)
{
    return task_.PhaseStandUp(running_time_, joint_targets);
}

bool TrotExperiment::PhaseLieDown(std::array<double, go2_trot::kMotorCount> &joint_targets)
{
    return task_.PhaseLieDown(running_time_, joint_targets);
}

bool TrotExperiment::PhaseStandSettle(std::array<double, go2_trot::kMotorCount> &joint_targets)
{
    (void)joint_targets;
    return task_.PhaseStandSettle(running_time_);
}

bool TrotExperiment::PhaseStopToStand(std::array<double, go2_trot::kMotorCount> &joint_targets)
{
    const bool active = task_.PhaseStopToStand(running_time_, joint_targets);
    if (task_.lie_down_started_ &&
        task_.lie_down_start_time_s_ == running_time_)
    {
        active_cycle_index_ = -1;
        completed_cycles_ = 0;
    }
    return active;
}

bool TrotExperiment::PhaseStartGait(std::array<double, go2_trot::kMotorCount> &joint_targets)
{
    if (!task_.BeginGait(running_time_))
        return false;
    support_anchor_valid_.fill(false);
    cartesian_state_ = {};
    have_commanded_world_feet_ = false;
    previous_leg_swing_.fill(false);
    touchdown_recorded_.fill(false);
    touchdown_waiting_contact_.fill(false);
    have_leg_phase_history_ = false;
    std::cout << "Starting diagonal trot: period="
              << params_.period_s
              << " s, duty=" << params_.duty_factor
              << ", step=" << params_.step_length_m
              << " m, lift=" << params_.foot_lift_m << " m"
              << (params_.cartesian_world ? " cartesian-world" : "")
              << "\n";
    if (task_.goal_enabled_)
    {
        std::cout << "Task world goal x=" << task_.goal_x_
                  << " y=" << task_.goal_y_
                  << " tol=" << task_.goal_tol_ << " m\n";
    }
    if (params_.cartesian_world)
        locomotion_kernel_->SetGaitSlewLimits(0.12, 0.20, 0.20);
    if (params_.cartesian_world)
    {
        locomotion_kernel_->SetGaitPeriod(params_.period_s);
        locomotion_kernel_->SetGaitDuty(params_.duty_factor);
        locomotion_kernel_->SetGaitStepLength(params_.step_length_m);
        locomotion_kernel_->SetGaitFootLift(params_.foot_lift_m);
        wbc_speed_cmd_mps_ = 0.15;
        cartesian_cruise_latched_ = false;
        cartesian_force_blend_ = 0.0;
        cartesian_yield_hold_ = 0.0;
        cartesian_yield_v_ref_ = 0.0;
        cartesian_plateau_cycles_ = 0;
        cartesian_force_cycles_ = 0;
        cartesian_latched_kp_ = 26.0;
        cartesian_kp_frozen_ = false;
        cartesian_paper_cycles_ = 0;
        cartesian_paper_latched_ = false;
        cartesian_open_latched_ = false;
        cartesian_open_t_latched_ = 0.0;
        cartesian_duty_ = 0.75;
        cartesian_stance_s_ = 0.45;
        cartesian_step_m_ = params_.step_length_m;
    }

    (void)joint_targets;
    return true;
}
bool TrotExperiment::SnapshotState(
    unitree_go::msg::dds_::LowState_ &state_snapshot,
    unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool &have_state,
    bool &have_high_state)
{
    have_state = false;
    have_high_state = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        have_state = have_low_state_;
        have_high_state = have_high_state_;
        if (have_state)
            state_snapshot = low_state_;
        if (have_high_state)
            high_state_snapshot = high_state_;
    }
    return have_state;
}
double TrotExperiment::MotionClockStep(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool &motion_clock_paused)
{
    const auto writer_now = std::chrono::steady_clock::now();
    double wall_dt_s = 0.0;
    if (!have_last_writer_time_)
    {
    have_last_writer_time_ = true;
    last_writer_time_ = writer_now;
    }
    else
    {
    wall_dt_s = std::chrono::duration<double>(
        writer_now - last_writer_time_).count();
    last_writer_time_ = writer_now;
    }

    double motion_dt = 0.0;
    double state_tick_gap = 0.0;
    bool state_clock_paused = false;
    const double state_tick_s =
    static_cast<double>(state_snapshot.tick()) * 0.001;
    if (!have_last_state_tick_)
    {
    have_last_state_tick_ = true;
    last_state_tick_s_ = state_tick_s;
    state_clock_paused = true;
    }
    else
    {
    state_tick_gap = state_tick_s - last_state_tick_s_;
    last_state_tick_s_ = state_tick_s;
    if (state_tick_gap > 1e-6 &&
        state_tick_gap <= kStateClockMaxGapS)
        motion_dt = state_tick_gap;
    else
        state_clock_paused = true;
    }

    motion_clock_paused = state_clock_paused;
    if (params_.wall_clock_motion)
    {
    if (wall_dt_s > 1e-6 &&
        wall_dt_s <= kStateClockMaxGapS)
    {
        motion_dt = wall_dt_s;
        motion_clock_paused = false;
    }
    else
    {
        motion_dt = 0.0;
        motion_clock_paused = true;
    }
    }

    last_motion_dt_s_ = motion_dt;
    last_state_tick_gap_s_ = state_tick_gap;
    last_clock_paused_ = state_clock_paused;
    last_wall_motion_dt_s_ = wall_dt_s;
    last_motion_clock_paused_ = motion_clock_paused;
    if (state_clock_paused)
    ++clock_pause_count_;
    if (motion_clock_paused)
    ++motion_clock_pause_count_;
    running_time_ += motion_dt;
    return motion_dt;
}
bool TrotExperiment::ComputeWbcPrimaryActive(double &gait_elapsed_s)
{
    bool wbc_primary_active = false;
    gait_elapsed_s = task_.gait_started_ ? running_time_ - task_.gait_start_time_s_ : 0.0;
    task_.gait_started_ ? running_time_ - task_.gait_start_time_s_ : 0.0;
    if (params_.wbc_primary &&
    task_.motion_stage_ == 2 && task_.gait_started_ && !task_.stop_requested_ &&
    gait_elapsed_s >= (params_.wbc_full ? 0.0 : kWbcPrimaryEnterDelayS) &&
    wbc_shadow_diagnostics_.solver_ok &&
    wbc_shadow_diagnostics_.mapping_ok)
    {
    wbc_primary_active = true;
    // [impulse] 大步长时 wrench 力矩需求大, 放宽 primary 力矩上限,
    // 避免退回位置控制后 q_error 硬限误杀。
            const double max_abs_torque_nm =
        params_.impulse ? 40.0
                        : (params_.wbc_full ? 35.0 : kWbcPrimaryMaxAbsTorqueNm);
    for (int i = 0; i < kMotorCount; ++i)
    {
        const double candidate =
            wbc_shadow_candidate_torques_[i / 3][i % 3];
        if (!std::isfinite(candidate) ||
            std::abs(candidate) > max_abs_torque_nm)
        {
            wbc_primary_active = false;
            break;
        }
    }
    }
    return wbc_primary_active;
}
bool TrotExperiment::PhaseRunGait(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state,
    std::array<double, kMotorCount> &joint_targets)
{
    if (task_.InLocomotion())
    {
    task_.motion_stage_ = 2;
    const double gait_time_s = running_time_ - task_.gait_start_time_s_;
    if (have_high_state)
    {
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        if (task_.goal_enabled_)
        {
            const double scale =
                task_.CommandedStepScale(pose.base.x, pose.base.y);
            if (scale < 0.999)
                locomotion_kernel_->SetGaitStepLength(
                    params_.step_length_m * scale);
            task_.UpdateTurnFromPose(
                pose.base.x, pose.base.y, pose.yaw_rad);
        }
    }
    // SECTION: gait-targets
    if (!BuildGaitTargets(
            gait_time_s,
            state_snapshot,
            high_state_snapshot,
            have_high_state,
            joint_targets))
    {
        task_.stop_requested_ = true;
    }
    if (have_high_state && !task_.stop_requested_)
    {
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        if (task_.MaybeReachWorldGoal(pose.base.x, pose.base.y))
        {
            std::cout << "Task reached world goal x=" << pose.base.x
                      << " y=" << pose.base.y
                      << " remaining="
                      << task_.RemainingXy(pose.base.x, pose.base.y)
                      << " m; RETURN_TO_STAND\n";
        }
    }
    if (!continuous_mode_ && !task_.stop_requested_ &&
        gait_time_s >= duration_s_ &&
        std::isfinite(duration_s_))
    {
        task_.stop_requested_ = true;
        if (task_.task_mode_)
        {
            task_.task_completion_requested_ = true;
            std::cout << "Task transition: LOCOMOTION -> RETURN_TO_STAND\n";
        }
    }
    }
    return true;
}

double TrotExperiment::UpdateCartesianForceBlend()
{
    if (!params_.cartesian_world)
    {
        cartesian_force_blend_ = 0.0;
        return 0.0;
    }
    const double vabs = have_filtered_body_velocity_
        ? std::abs(latest_filtered_body_velocity_[0])
        : 0.0;
    const double tst_blend = Smoothstep(
        (0.40 - cartesian_stance_s_) / 0.12);
    const double speed_gate = Smoothstep((vabs - 0.28) / 0.15);
    const double target = tst_blend * speed_gate;
    if (Full2EnvDouble("FULL2_NO_BLEND", 0.0) > 0.5)
    {
        cartesian_force_blend_ = 0.0;
        return 0.0;
    }
    // 210225 completed 90 at 0.32 with blend tracking speed_gate through
    // the Tst slew. 210548 floored blend at Tst<=0.18 and reversed.
    // After Tst settles, cartesian_yield_hold_ is a one-way kp-yield
    // floor so we do not re-stiffen when v_meas sags.
    cartesian_force_blend_ = std::max(cartesian_yield_hold_, target);
    return cartesian_force_blend_;
}

void TrotExperiment::WriteMotorCommands(
    bool wbc_primary_active,
    double gait_elapsed_s,
    const std::array<double, go2_trot::kMotorCount> &joint_targets,
    const std::array<double, go2_trot::kMotorCount> &joint_velocities,
    const std::array<double, go2_trot::kMotorCount> &wbc_torque_ff,
    bool apply_wbc_torque_ff)
{
    if (wbc_primary_active)
    {
    // 扭矩渐变注入:激活后 0.5s 内从 0 线性升到 1,避免跳变冲击
    const double primary_ramp = params_.wbc_full
        ? 1.0
        : Clamp(
            (gait_elapsed_s - kWbcPrimaryEnterDelayS) /
                kWbcPrimaryRampS,
            0.0, 1.0);
    // 混合控制:支撑腿 = WBC 扭矩(力控制)+ 弱位置;摆动腿 = 位置控制。
    // 接触状态经一阶平滑(swing<->stance 渐变),避免命令跳变冲击。
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const bool leg_in_stance =
            (wbc_shadow_diagnostics_.contact_mask &
             (1 << leg)) != 0;
        const double target_blend = leg_in_stance ? 1.0 : 0.0;
        wbc_stance_blend_[leg] +=
            (target_blend - wbc_stance_blend_[leg]) *
            (dt_ / kWbcPrimaryBlendTauS);
        if (wbc_stance_blend_[leg] < 0.0)
            wbc_stance_blend_[leg] = 0.0;
        if (wbc_stance_blend_[leg] > 1.0)
            wbc_stance_blend_[leg] = 1.0;
        for (int joint = 0; joint < 3; ++joint)
        {
            const int i = 3 * static_cast<int>(leg) + joint;
            low_cmd_.motor_cmd()[i].q() = joint_targets[i];
            low_cmd_.motor_cmd()[i].dq() = joint_velocities[i];
            // Mini Cheetah split after latch: joint PD yields, ID-WBC
            // keeps the world plant and J^T f pushes the body.
            const double vabs = have_filtered_body_velocity_
                ? std::abs(latest_filtered_body_velocity_[0])
                : 0.0;
            const double force_blend = UpdateCartesianForceBlend();
            const double speed_kp =
                24.0 + 28.0 * Clamp(vabs / 1.20, 0.0, 1.0);
            const bool use_latched =
                cartesian_cruise_latched_ &&
                (cartesian_yield_hold_ > 0.08 ||
                 cartesian_kp_frozen_ ||
                 cartesian_plateau_cycles_ >= 16);
            if (!use_latched)
                cartesian_latched_kp_ = speed_kp;
            const double high_kp =
                use_latched ? cartesian_latched_kp_ : speed_kp;
            const double plant = Smoothstep(
                (0.40 - cartesian_stance_s_) / 0.12);
            const double yield = plant * force_blend;
            // Kim et al. 2019: joint kp=3, kd=0.3 in locomotion.
            // After freeze, latched_kp is the command. Mixing 3
            // through force_blend (142839 blend 0.25–0.55 at Tst=0.15)
            // dropped motor sagittal to ~10 while CART-GOV printed 18.
            // Campaign baseline: do not mix toward paper kp=3.
            // That mix made CART-GOV kp lie; paper kp is a closed cell.
            const double sagittal_kp = high_kp;
            (void)plant;
            (void)yield;
            const double hip_floor =
                cartesian_stance_s_ <= 0.36 ? 48.0 : 36.0;
            const double hip_kp = std::max(high_kp, hip_floor);
            const double stance_kp = params_.cartesian_world
                ? (joint == 0 ? hip_kp : sagittal_kp)
                : (params_.wbc_full
                    ? kWbcFullStanceKp
                    : (params_.impulse ? kImpulseStanceKp : kWbcPrimaryCommandKp));
            const double kd_cheetah =
                cartesian_kp_frozen_
                    ? Smoothstep((8.0 - cartesian_latched_kp_) / 5.0)
                    : 0.0;
            const double stance_kd = params_.cartesian_world
                ? (joint == 0 ? 2.6
                   : (2.5 * (1.0 - kd_cheetah) + 0.3 * kd_cheetah))
                : (params_.wbc_full
                    ? kWbcFullStanceKd
                    : kWbcPrimaryCommandKd);
            // 142128: CART-GOV printed kp=3 but FR_thigh motor kp
            // median was 33 because swing mixed params_.kp=63 through
            // wbc_stance_blend. Cheetah uses 3/0.3 on all sagittal
            // joints once latched. Do not mix 63 after freeze.
            double cmd_kp = stance_kp * wbc_stance_blend_[leg] +
                params_.kp * (1.0 - wbc_stance_blend_[leg]);
            double cmd_kd = stance_kd * wbc_stance_blend_[leg] +
                params_.kd * (1.0 - wbc_stance_blend_[leg]);
            if (params_.cartesian_world && cartesian_kp_frozen_)
            {
                cmd_kp = (joint == 0 ? hip_kp : sagittal_kp);
                cmd_kd = stance_kd;
            }
            low_cmd_.motor_cmd()[i].kp() = cmd_kp;
            low_cmd_.motor_cmd()[i].kd() = cmd_kd;
            // 低接触数(过渡/对角支撑)时减弱 WBC 扭矩,避免力分配
            // 在支撑切换瞬间扰动姿态
            const double contact_scale = params_.wbc_full
                ? 1.0
                : (wbc_shadow_diagnostics_.active_contacts < 3 ? 0.5 : 1.0);
            low_cmd_.motor_cmd()[i].tau() =
                primary_ramp * contact_scale *
                (params_.wbc_full ? 1.0 : wbc_stance_blend_[leg]) *
                wbc_shadow_candidate_torques_[leg][joint];
        }
    }
    wbc_shadow_diagnostics_.feedforward_applied = true;
    }
    else
    {
    for (int i = 0; i < kMotorCount; ++i)
    {
        low_cmd_.motor_cmd()[i].q() = joint_targets[i];
        low_cmd_.motor_cmd()[i].dq() = joint_velocities[i];
        low_cmd_.motor_cmd()[i].kp() =
            task_.motion_stage_ == 0 ? 100.0 : params_.kp;
        low_cmd_.motor_cmd()[i].kd() =
            task_.motion_stage_ == 0 ? 3.5 : params_.kd;
        low_cmd_.motor_cmd()[i].tau() =
            apply_wbc_torque_ff ? wbc_torque_ff[i] : 0.0;
    }
    if (apply_wbc_torque_ff)
        wbc_shadow_diagnostics_.feedforward_applied = true;
    }
}
bool TrotExperiment::UpdateWbcShadowAndTorqueFf(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state,
    std::array<double, kMotorCount> &wbc_torque_ff)
{
    // SECTION: wbc-shadow
    if (params_.wbc_shadow)
        UpdateWbcShadow(
            state_snapshot, have_state,
            high_state_snapshot, have_high_state);
    else
    {
        wbc_shadow_diagnostics_ = WbcShadowDiagnostics{};
        wbc_shadow_candidate_torques_ = {};
    }
    // SECTION: wbc-torque-ff
    return params_.wbc_torque_feedforward &&
           PrepareWbcTorqueFeedforward(wbc_torque_ff);
}
void TrotExperiment::UpdateJointVelocityFeedforward(
    const std::array<double, kMotorCount> &joint_targets,
    double motion_dt,
    bool motion_clock_paused,
    std::array<double, kMotorCount> &joint_velocities)
{
    if (params_.velocity_feedforward &&
        have_previous_joint_targets_ &&
        motion_dt > 1e-6 &&
        !motion_clock_paused)
    {
        for (int i = 0; i < kMotorCount; ++i)
        {
            joint_velocities[i] = Clamp(
                (joint_targets[i] - previous_joint_targets_[i]) / motion_dt,
                -10.0,
                10.0);
        }
    }
    previous_joint_targets_ = joint_targets;
    have_previous_joint_targets_ = true;
}
void TrotExperiment::UpdateGaitWorldDiagnostics(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state,
    const std::array<double, kMotorCount> &joint_targets)
{
    std::array<go2::Vec3, go2::kLegCount> world_feet{};
    bool have_world_feet = false;
    WorldPose world_pose{};
    if (have_high_state)
    {
        world_pose = ComputeWorldPose(state_snapshot, high_state_snapshot);
        world_feet = ComputeWorldFeet(state_snapshot, world_pose);
        have_world_feet = true;
    }

    touchdown_event_count_ = 0;
    last_touchdown_leg_ = -1;
    last_touchdown_command_x_m_ = 0.0;
    last_touchdown_actual_x_m_ = 0.0;
    last_touchdown_x_error_m_ = 0.0;
    last_touchdown_y_error_m_ = 0.0;
    if (task_.gait_started_ && !task_.stop_requested_)
    {
        UpdateCycleDiagnostics(
            current_phase_,
            state_snapshot,
            have_state,
            joint_targets,
            have_world_feet,
            world_feet,
            world_pose);
    }
}
