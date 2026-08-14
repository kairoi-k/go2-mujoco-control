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
bool TrotExperiment::BuildGaitTargets(
    double gait_time_s,
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state,
    std::array<double, kMotorCount> &joint_targets)
{
    auto feet = go2::AllFootPositions(stand_up_joint_pos_);
    go2_control::GaitKernelResult gait_result{};
    go2_control::GaitKernelRequest gait_request{};
    gait_request.gait_time_s = gait_time_s;
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
    if (!locomotion_kernel_->Compute(gait_request, gait_result))
    {
        std::cerr << "Locomotion kernel failed at gait_time="
                  << gait_time_s << "\n";
        return false;
    }
    kernel_footstep_plan_valid_ = gait_result.footstep_plan_valid;
    kernel_velocity_error_x_mps_ = gait_result.velocity_error_x_mps;
    kernel_touchdown_target_x_m_ = gait_result.touchdown_target_x_m;
    const double phase = gait_result.phase;
    current_phase_ = phase;
    const int cycle_index = gait_result.cycle_index;
    if (active_cycle_index_ < 0)
    {
        active_cycle_index_ = cycle_index;
        ResetCycleDiagnostics();
    }
    else if (cycle_index > active_cycle_index_)
    {
        if (!ValidateCycle(active_cycle_index_))
            stop_requested_ = true;
        ++completed_cycles_;
        if (max_cycles_ > 0 && completed_cycles_ >= max_cycles_)
            stop_requested_ = true;
        active_cycle_index_ = cycle_index;
        ResetCycleDiagnostics();
        // [Phase3] online gear shift: apply step-length plan at cycle boundary
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
        std::cout << "Trot cycle " << cycle_index << " started\n";
    }

    feet = gait_result.feet;
    const double stance_duration = params_.duty_factor;

    WorldPose pose{};
    std::array<go2::Vec3, go2::kLegCount> actual_world_feet{};
    bool have_actual_world_feet = false;
    if (have_high_state)
    {
        pose = ComputeWorldPose(state_snapshot, high_state_snapshot);
        actual_world_feet = ComputeWorldFeet(state_snapshot, pose);
        have_actual_world_feet = true;
    }
    if (params_.world_feedback && have_world_reference_ && have_high_state)
    {
        pose = ComputeWorldPose(state_snapshot, high_state_snapshot);
        const double ref_cos = std::cos(world_reference_yaw_rad_);
        const double ref_sin = std::sin(world_reference_yaw_rad_);
        const double actual_x =
            ref_cos * (pose.base.x - world_reference_x_m_) +
            ref_sin * (pose.base.y - world_reference_y_m_);
        const double actual_y =
            -ref_sin * (pose.base.x - world_reference_x_m_) +
            ref_cos * (pose.base.y - world_reference_y_m_);
        // [Fix 2026-08-13] world target 用 kernel 当前生效速度 (换挡同步)
        const double target_x =
            gait_result.nominal_velocity_x_mps > 0.0
                ? gait_result.nominal_velocity_x_mps * gait_time_s
                : params_.direction_sign * gait_time_s *
                      params_.step_length_m / params_.period_s;
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
        world_yaw_error_rad_ = WrapAngle(
            pose.yaw_rad - world_reference_yaw_rad_);
        const double yaw_trim = params_.turn_rate_radps != 0.0
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

    if (params_.support_anchor_feedback && have_actual_world_feet)
    {
        const std::array<double, 4> inverse_quaternion = {
            pose.quaternion[0],
            -pose.quaternion[1],
            -pose.quaternion[2],
            -pose.quaternion[3]};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const bool diagonal_pair_b =
                leg == static_cast<std::size_t>(go2::Leg::FL) ||
                leg == static_cast<std::size_t>(go2::Leg::RR);
            const double leg_phase = std::fmod(
                phase + (diagonal_pair_b ? 0.5 : 0.0), 1.0);
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
    if (params_.attitude_feedback)
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

    // [Phase5] 转向(内/外腿 y 偏移,经典 Raibert):内侧腿 y 收,
    // 外侧腿 y 放,产生 yaw 力矩;偏移小(<=8mm)不超工作空间。
    if (params_.turn_rate_radps != 0.0)
    {
        // 起步 4s 内不转向,4-6s 渐变启用(避免转向启动冲击)
        const double turn_enable = Clamp(
            (running_time_ - gait_start_time_s_ - 4.0) / 2.0,
            0.0, 1.0);
        const double delta = turn_enable * std::clamp(
            params_.turn_rate_radps * params_.period_s * 0.08,
            -0.015, 0.015);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            // 正 turn-rate = 左转:y 正侧(左腿)内收,右腿外放
            const bool left = feet[leg].y > 0.0;
            feet[leg].y += (left ? -delta : delta);
        }
        std::cout << "TURN delta=" << delta << " m\n";
    }
    commanded_body_feet_ = feet;
    have_commanded_body_feet_ = true;
    if (!go2::AllLegInverseKinematics(feet, joint_targets))
    {
        std::cerr << "Trot IK failed at gait_time=" << gait_time_s << "\n";
        return false;
    }
    return true;
}
