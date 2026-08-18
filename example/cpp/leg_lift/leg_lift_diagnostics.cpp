#include "leg_lift_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "go2_forward_kinematics.h"
#include "go2_inverse_kinematics.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_leg;

// DIAGNOSTICS — CSV, step validation, motion checks (see docs/CODE_GUIDE.md)

// --- TrackingExperiment::WriteCsvHeader ---  [SECTION: csv-header]
void TrackingExperiment::WriteCsvHeader()
{
    csv_ << "cmd_time_s,state_tick_s,has_state,motion_clock_dt_s"
         << ",state_clock_paused,state_tick_gap_s,state_clock_pause_count"
         << ",motion_stage,cycle_index,active_leg_index"
         << ",tempo_governor_scale"
         << ",body_shift_x_target_m,body_shift_y_target_m"
         << ",body_advance_x_target_m,body_advance_y_target_m"
         << ",world_feedback_x_m,world_feedback_y_m"
         << ",world_yaw_error_rad,yaw_feedback_y_m,yaw_feedback_swing_x_m"
         << ",yaw_feedback_body_rotation_rad"
         << ",attitude_feedback_x_m,attitude_feedback_y_m";
    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        csv_ << "," << kLegNames[i] << "_foot_lift_target_m"
             << "," << kLegNames[i] << "_foot_swing_x_target_m"
             << "," << kLegNames[i] << "_foot_swing_y_target_m";
    }
    for (int i = 0; i < kMotorCount; i++)
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

    csv_ << ",imu_roll_rad,imu_pitch_rad,imu_yaw_rad";
    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        csv_ << "," << kLegNames[i] << "_foot_force"
             << "," << kLegNames[i] << "_foot_force_est";
    }
    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        for (const char *axis : {"x", "y", "z"})
        {
            csv_ << "," << kLegNames[i] << "_foot_" << axis << "_target_m"
                 << "," << kLegNames[i] << "_foot_" << axis << "_state_m"
                 << "," << kLegNames[i] << "_foot_" << axis << "_error_m";
        }
    }
    csv_ << ",base_world_x_m,base_world_y_m,base_world_z_m";
    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        csv_ << "," << kLegNames[i] << "_foot_world_x_m"
             << "," << kLegNames[i] << "_foot_world_y_m"
             << "," << kLegNames[i] << "_foot_world_z_m"
             << "," << kLegNames[i] << "_foot_ground_clearance_m";
    }
    csv_ << "\n";
}

// --- TrackingExperiment::ResetStepDiagnostics ---
void TrackingExperiment::ResetStepDiagnostics()
{
    step_diagnostics_ = StepDiagnostics{};
}

// --- TrackingExperiment::UpdateStepDiagnostics ---  [SECTION: update-step-diagnostics]
void TrackingExperiment::UpdateStepDiagnostics(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const std::array<double, kMotorCount> &target_joint_positions,
    const std::array<double, kMotorCount> &state_joint_positions,
    const std::array<go2::Vec3, go2::kLegCount> &world_feet,
    bool have_world_feet)
{
    if (!have_state)
        return;

    step_diagnostics_.have_sample = true;
    const double roll = state_snapshot.imu_state().rpy()[0];
    const double pitch = state_snapshot.imu_state().rpy()[1];
    step_diagnostics_.max_abs_roll_rad = std::max(
        step_diagnostics_.max_abs_roll_rad, std::abs(roll));
    step_diagnostics_.max_abs_pitch_rad = std::max(
        step_diagnostics_.max_abs_pitch_rad, std::abs(pitch));
    if (have_world_reference_)
    {
        const double yaw_error = std::remainder(
            state_snapshot.imu_state().rpy()[2] -
                world_reference_yaw_rad_,
            2.0 * kPi);
        step_diagnostics_.max_abs_yaw_error_rad = std::max(
            step_diagnostics_.max_abs_yaw_error_rad, std::abs(yaw_error));
    }

    for (int i = 0; i < kMotorCount; ++i)
    {
        step_diagnostics_.max_abs_joint_error_rad = std::max(
            step_diagnostics_.max_abs_joint_error_rad,
            std::abs(target_joint_positions[i] - state_joint_positions[i]));
        step_diagnostics_.max_abs_tau_est = std::max(
            step_diagnostics_.max_abs_tau_est,
            std::abs(static_cast<double>(
                state_snapshot.motor_state()[i].tau_est())));
    }

    int support_contacts = 0;
    const int active_leg_index = static_cast<int>(lift_leg_);
    for (int leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (leg != active_leg_index &&
            state_snapshot.foot_force()[leg] >= kSupportContactThreshold)
        {
            ++support_contacts;
        }
    }
    step_diagnostics_.min_support_contacts = std::min(
        step_diagnostics_.min_support_contacts, support_contacts);
    if (support_contacts < kSafetyMinSupportContacts)
    {
        ++step_diagnostics_.consecutive_low_support_samples;
        step_diagnostics_.max_consecutive_low_support_samples = std::max(
            step_diagnostics_.max_consecutive_low_support_samples,
            step_diagnostics_.consecutive_low_support_samples);
    }
    else
    {
        step_diagnostics_.consecutive_low_support_samples = 0;
    }
    ++step_diagnostics_.support_contact_samples;
    if (support_contacts >= kSafetyMinSupportContacts)
    {
        ++step_diagnostics_.support_contact_good_samples;
    }
    step_diagnostics_.support_contact_fraction =
        static_cast<double>(step_diagnostics_.support_contact_good_samples) /
        static_cast<double>(step_diagnostics_.support_contact_samples);

    if (!have_world_feet)
        return;
    if (!step_diagnostics_.support_reference_valid)
    {
        step_diagnostics_.support_reference_world_feet = world_feet;
        step_diagnostics_.support_reference_valid = true;
    }
    for (int leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (leg == active_leg_index)
            continue;
        const double dx = world_feet[leg].x -
            step_diagnostics_.support_reference_world_feet[leg].x;
        const double dy = world_feet[leg].y -
            step_diagnostics_.support_reference_world_feet[leg].y;
        step_diagnostics_.max_support_drift_m = std::max(
            step_diagnostics_.max_support_drift_m, std::hypot(dx, dy));
    }
}

// --- TrackingExperiment::ValidateStepDiagnostics ---  [SECTION: validate-step]
bool TrackingExperiment::ValidateStepDiagnostics() const
{
    const double radians_to_degrees = 180.0 / kPi;
    std::cout << "Step " << current_cycle_number_ << " health: roll="
              << step_diagnostics_.max_abs_roll_rad * radians_to_degrees
              << " deg, pitch="
              << step_diagnostics_.max_abs_pitch_rad * radians_to_degrees
              << " deg, max_yaw_error="
              << step_diagnostics_.max_abs_yaw_error_rad * radians_to_degrees
              << " deg, support_drift="
              << step_diagnostics_.max_support_drift_m * 1000.0
              << " mm, q_error="
              << step_diagnostics_.max_abs_joint_error_rad
              << " rad, tau_est=" << step_diagnostics_.max_abs_tau_est
              << ", min_support_contacts="
              << step_diagnostics_.min_support_contacts
              << ", max_low_support_samples="
              << step_diagnostics_.max_consecutive_low_support_samples
              << ", support_contact_fraction="
              << step_diagnostics_.support_contact_fraction << std::endl;
    const bool support_contacts_healthy =
        step_diagnostics_.support_contact_samples > 0 &&
        step_diagnostics_.support_contact_fraction >=
            kSafetyMinSupportContactFraction &&
        step_diagnostics_.max_consecutive_low_support_samples <=
            kSafetyMaxConsecutiveLowSupportSamples;
    const bool safe =
        step_diagnostics_.have_sample &&
        step_diagnostics_.max_abs_roll_rad <= kSafetyMaxAbsRollRad &&
        step_diagnostics_.max_abs_pitch_rad <= kSafetyMaxAbsPitchRad &&
        step_diagnostics_.max_support_drift_m <= kSafetyMaxSupportDriftM &&
        step_diagnostics_.max_abs_joint_error_rad <= kSafetyMaxJointErrorRad &&
        step_diagnostics_.max_abs_tau_est <= kSafetyMaxTauEst &&
        (!yaw_feedback_enabled_ ||
         step_diagnostics_.max_abs_yaw_error_rad <=
             kSafetyMaxAbsYawErrorRad) &&
        support_contacts_healthy;
    if (!safe)
    {
        std::cerr << "Safety guard rejected step " << current_cycle_number_
                  << ". Stop before applying the next step." << std::endl;
    }
    return safe;
}

// --- TrackingExperiment::LogSample ---  [SECTION: log-sample]
void TrackingExperiment::LogSample()
{
    unitree_go::msg::dds_::LowState_ state_snapshot{};
    unitree_go::msg::dds_::SportModeState_ high_state_snapshot{};
    bool have_state = false;
    bool have_high_state = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_snapshot = low_state_;
        high_state_snapshot = high_state_;
        have_state = have_low_state_;
        have_high_state = have_high_state_;
    }

    const double state_tick_s = have_state ? state_snapshot.tick() * 0.001 : 0.0;
    const int active_leg_index =
        motion_stage_ >= 2 ? static_cast<int>(lift_leg_) : -1;
    // SECTION: log-state-summary (time, clock, stage, commanded shifts)
    csv_ << running_time_
         << "," << state_tick_s
         << "," << (have_state ? 1 : 0)
         << "," << last_motion_clock_dt_s_
         << "," << (last_motion_clock_paused_ ? 1 : 0)
         << "," << last_state_tick_gap_s_
         << "," << state_clock_pause_count_
         << "," << motion_stage_
         << "," << current_cycle_number_
         << "," << active_leg_index
         << "," << last_tempo_governor_scale_
         << "," << commanded_body_shift_x_m_
         << "," << commanded_body_shift_y_m_
         << "," << body_advance_x_m_
         << "," << body_advance_y_m_
         << "," << world_feedback_x_m_ << "," << world_feedback_y_m_
         << "," << world_yaw_error_rad_ << "," << yaw_feedback_y_m_
         << "," << yaw_feedback_swing_x_m_
         << "," << yaw_feedback_body_rotation_rad_
         << "," << attitude_feedback_x_m_
         << "," << attitude_feedback_y_m_;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const bool active = static_cast<int>(leg) == active_leg_index;
        csv_ << "," << (active ? commanded_foot_lift_height_m_ : 0.0)
             << "," << (active ? commanded_foot_swing_x_m_ : 0.0)
             << "," << (active ? commanded_foot_swing_y_m_ : 0.0);
    }

    // SECTION: log-joint-cmds (cmd vs state per joint)
    std::array<double, kMotorCount> target_joint_positions{};
    std::array<double, kMotorCount> state_joint_positions{};
    for (int i = 0; i < kMotorCount; i++)
    {
        const double q_target = low_cmd_.motor_cmd()[i].q();
        const double dq_target = low_cmd_.motor_cmd()[i].dq();
        const double kp = low_cmd_.motor_cmd()[i].kp();
        const double kd = low_cmd_.motor_cmd()[i].kd();
        const double tau_ff = low_cmd_.motor_cmd()[i].tau();
        const double q_state = have_state ? state_snapshot.motor_state()[i].q() : 0.0;
        const double dq_state = have_state ? state_snapshot.motor_state()[i].dq() : 0.0;
        const double tau_est = have_state ? state_snapshot.motor_state()[i].tau_est() : 0.0;
        const double q_error = have_state ? q_target - q_state : 0.0;

        target_joint_positions[i] = q_target;
        state_joint_positions[i] = q_state;

        csv_ << "," << q_target
             << "," << dq_target
             << "," << kp
             << "," << kd
             << "," << tau_ff
             << "," << q_state
             << "," << dq_state
             << "," << tau_est
             << "," << q_error;
    }

    const double imu_roll = have_state ? state_snapshot.imu_state().rpy()[0] : 0.0;
    const double imu_pitch = have_state ? state_snapshot.imu_state().rpy()[1] : 0.0;
    const double imu_yaw = have_state ? state_snapshot.imu_state().rpy()[2] : 0.0;
    csv_ << "," << imu_roll
         << "," << imu_pitch
         << "," << imu_yaw;

    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        csv_ << ","
             << (have_state ? state_snapshot.foot_force()[i] : 0)
             << ","
             << (have_state ? state_snapshot.foot_force_est()[i] : 0);
    }

    // SECTION: log-feet (target/state/error per axis, world feet)
    const auto target_feet = go2::AllFootPositions(target_joint_positions);
    const auto state_feet = go2::AllFootPositions(state_joint_positions);
    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        const std::array<double, 3> target = {
            target_feet[i].x, target_feet[i].y, target_feet[i].z};
        const std::array<double, 3> state = {
            state_feet[i].x, state_feet[i].y, state_feet[i].z};

        for (std::size_t axis = 0; axis < target.size(); ++axis)
        {
            csv_ << "," << target[axis]
                 << "," << state[axis]
                 << "," << (have_state ? target[axis] - state[axis] : 0.0);
        }
    }

    go2::Vec3 base_world{};
    std::array<go2::Vec3, go2::kLegCount> world_feet{};
    if (have_state && have_high_state)
    {
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        base_world = pose.base;
        for (std::size_t i = 0; i < go2::kLegCount; ++i)
        {
            const auto foot_offset_world =
                RotateByQuaternion(pose.quaternion, state_feet[i]);
            world_feet[i] = {
                base_world.x + foot_offset_world.x,
                base_world.y + foot_offset_world.y,
                base_world.z + foot_offset_world.z};
        }
    }
    UpdateStepDiagnostics(state_snapshot, have_state, target_joint_positions,
                         state_joint_positions, world_feet,
                         have_state && have_high_state);
    csv_ << "," << base_world.x
         << "," << base_world.y
         << "," << base_world.z;
    for (const auto &foot : world_feet)
    {
        csv_ << "," << foot.x
             << "," << foot.y
             << "," << foot.z
             << "," << (foot.z - kFootCollisionRadiusM);
    }
    csv_ << "\n";
}

// --- TrackingExperiment::CheckMotionTargets ---  [SECTION: check-motion-targets]
bool TrackingExperiment::CheckMotionTargets() const
{
    const auto stand_feet = go2::AllFootPositions(stand_up_joint_pos_);
    const auto check_target =
        [](const auto &target_feet)
    {
        std::array<double, kMotorCount> joint_targets{};
        if (!go2::AllLegInverseKinematics(target_feet, joint_targets))
        {
            std::cerr
                << "Requested body shift or foot lift is unreachable\n";
            return false;
        }

        for (int leg = 0; leg < 4; ++leg)
        {
            const int joint = leg * 3;
            const bool front = leg == 0 || leg == 1;
            const double thigh_min = front ? -1.5708 : -0.5236;
            const double thigh_max = front ? 3.4907 : 4.5379;
            if (joint_targets[joint] < -1.0472 ||
                joint_targets[joint] > 1.0472 ||
                joint_targets[joint + 1] < thigh_min ||
                joint_targets[joint + 1] > thigh_max ||
                joint_targets[joint + 2] < -2.7227 ||
                joint_targets[joint + 2] > -0.83776)
            {
                std::cerr
                    << "Joint limit at " << kLegNames[leg]
                    << ": hip=" << joint_targets[joint]
                    << ", thigh=" << joint_targets[joint + 1]
                    << ", calf=" << joint_targets[joint + 2]
                    << std::endl;
                std::cerr
                    << "Requested motion exceeds a Go2 joint limit\n";
                return false;
            }
        }
        return true;
    };
    std::array<double, go2::kLegCount> placed_x{};
    std::array<double, go2::kLegCount> placed_y{};
    double body_start_x = 0.0;
    double body_start_y = 0.0;
    for (std::size_t step_index = 0;
         step_index < steps_.size();
         ++step_index)
    {
        const auto &step = steps_[step_index];
        const double body_x_min = std::min(
            body_start_x,
            std::min(step.body_shift_x_m, step.body_advance_x_m));
        const double body_x_max = std::max(
            body_start_x,
            std::max(step.body_shift_x_m, step.body_advance_x_m));
        const double body_y_min = std::min(
            body_start_y,
            std::min(step.body_shift_y_m, step.body_advance_y_m));
        const double body_y_max = std::max(
            body_start_y,
            std::max(step.body_shift_y_m, step.body_advance_y_m));
        for (int shift_sample = 0; shift_sample <= 100; ++shift_sample)
        {
            const double shift_phase = shift_sample / 100.0;
            const double commanded_body_x =
                body_x_min + shift_phase * (body_x_max - body_x_min);
            const double commanded_body_y =
                body_y_min + shift_phase * (body_y_max - body_y_min);
            for (int lift_sample = 0; lift_sample <= 100; ++lift_sample)
            {
                const double lift_phase = lift_sample / 100.0;
                auto target_feet = stand_feet;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    target_feet[leg].x +=
                        placed_x[leg] - commanded_body_x;
                    target_feet[leg].y +=
                        placed_y[leg] - commanded_body_y;
                }
                const std::size_t active_leg =
                    static_cast<std::size_t>(step.lift_leg);
                target_feet[active_leg].z +=
                    lift_phase * step.foot_lift_height_m;
                target_feet[active_leg].x +=
                    lift_phase * step.swing_x_m;
                target_feet[active_leg].y +=
                    lift_phase * step.swing_y_m;
                if (!check_target(target_feet))
                {
                    std::cerr
                        << "Sequence step " << step_index + 1
                        << " target rejected: body=("
                        << commanded_body_x << ", "
                        << commanded_body_y << "), active leg="
                        << kLegNames[static_cast<std::size_t>(step.lift_leg)]
                        << ", lift phase=" << lift_phase << std::endl;
                    return false;
                }
            }
        }
        const std::size_t completed_leg =
            static_cast<std::size_t>(step.lift_leg);
        placed_x[completed_leg] += step.swing_x_m;
        placed_y[completed_leg] += step.swing_y_m;
        body_start_x = step.body_advance_x_m;
        body_start_y = step.body_advance_y_m;
    }
    const double body_x_min =
        std::min(0.0, std::min(body_shift_x_m_, body_advance_x_m_));
    const double body_x_max =
        std::max(0.0, std::max(body_shift_x_m_, body_advance_x_m_));
    const double body_y_min =
        std::min(0.0, std::min(body_shift_y_m_, body_advance_y_m_));
    const double body_y_max =
        std::max(0.0, std::max(body_shift_y_m_, body_advance_y_m_));
    for (int shift_sample = 0; shift_sample <= 100; ++shift_sample)
    {
        const double shift_phase = shift_sample / 100.0;
        const double commanded_body_x =
            body_x_min + shift_phase * (body_x_max - body_x_min);
        const double commanded_body_y =
            body_y_min + shift_phase * (body_y_max - body_y_min);
        for (int lift_sample = 0; lift_sample <= 100; ++lift_sample)
        {
            const double lift_phase = lift_sample / 100.0;
            auto target_feet = stand_feet;
            for (auto &foot : target_feet)
            {
                foot.x -= commanded_body_x;
                foot.y -= commanded_body_y;
            }
            target_feet[static_cast<std::size_t>(lift_leg_)].z +=
                shift_phase * lift_phase * foot_lift_height_m_;
            target_feet[static_cast<std::size_t>(lift_leg_)].x +=
                lift_phase * swing_x_m_;
            target_feet[static_cast<std::size_t>(lift_leg_)].y +=
                lift_phase * swing_y_m_;

            std::array<double, kMotorCount> joint_targets{};
            if (!go2::AllLegInverseKinematics(target_feet, joint_targets))
            {
                std::cerr
                    << "Requested body shift or foot lift is unreachable\n";
                return false;
            }

            for (int leg = 0; leg < 4; ++leg)
            {
                const int joint = leg * 3;
                const bool front = leg == 0 || leg == 1;
                const double thigh_min = front ? -1.5708 : -0.5236;
                const double thigh_max = front ? 3.4907 : 4.5379;
                if (joint_targets[joint] < -1.0472 ||
                    joint_targets[joint] > 1.0472 ||
                    joint_targets[joint + 1] < thigh_min ||
                    joint_targets[joint + 1] > thigh_max ||
                    joint_targets[joint + 2] < -2.7227 ||
                    joint_targets[joint + 2] > -0.83776)
                {
                    std::cerr
                        << "Requested motion exceeds a Go2 joint limit\n";
                    return false;
                }
            }
        }
    }
    return true;
}
