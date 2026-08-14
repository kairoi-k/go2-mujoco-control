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

// CONTROL LOOP — phase machine + LowCmd write (see CODEMAP)

// --- TrackingExperiment::LowCmdWrite ---

void TrackingExperiment::LowCmdWrite()
{
    LowCmdScratch scratch;
    // ROADMAP LowCmdWrite: stand-up -> settle -> weight-shift -> foot-lift/swing/lower -> body-return -> terminal -> publish
    // Jump via SECTION: markers below.
    // AUTO-TOC (line numbers drift if edited; search SECTION:)
    //   L122: stand-up (see PhaseStandUp)
    //   L126: stand-settle
    //   L130: terminal-correction (see PhaseTerminalCorrection)
    //   L134: weight-shift (see PhaseWeightShift)
    //   L155: foot-lift (see PhaseFootLift)
    //   L159: foot-swing (see PhaseFootSwing)
    //   L173: foot-lower-active (see PhaseFootLowerActive)
    //   L183: landing-hold (see PhaseLandingHold)
    //   L191: body-return (see PhaseBodyReturn)
    //   L207: publish-lowcmd

    if (finished_.load())
    {
        return;
    }
    if (!have_start_joint_pos_)
    {
        return;
    }

    // Keep command generation at 500 Hz, but let motion time follow the
    // simulator tick. If rendering or physics stalls, hold the current gait
    // phase instead of advancing targets against a stale LowState snapshot.
    double motion_clock_dt_s = dt_;
    double state_tick_gap_s = 0.0;
    bool motion_clock_paused = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_low_state_)
        {
            const double state_tick_s =
                static_cast<double>(low_state_.tick()) * 0.001;
            if (!have_last_state_tick_for_clock_)
            {
                last_state_tick_s_for_clock_ = state_tick_s;
                have_last_state_tick_for_clock_ = true;
            }
            else
            {
                state_tick_gap_s =
                    state_tick_s - last_state_tick_s_for_clock_;
                last_state_tick_s_for_clock_ = state_tick_s;
                if (state_tick_gap_s > 1e-6 &&
                    state_tick_gap_s <= kMaxStateClockGapS)
                {
                    motion_clock_dt_s = state_tick_gap_s;
                }
                else
                {
                    motion_clock_dt_s = 0.0;
                    motion_clock_paused = true;
                }
            }
        }
    }
    last_motion_clock_dt_s_ = motion_clock_dt_s;
    last_state_tick_gap_s_ = state_tick_gap_s;
    last_motion_clock_paused_ = motion_clock_paused;
    if (motion_clock_paused)
    {
        ++state_clock_pause_count_;
    }

    const bool balance_phase_active = motion_stage_ >= 2;
    const bool swing_phase_active =
        balance_phase_active &&
        (motion_stage_ == 4 || motion_stage_ == 5);
    const bool governor_phase_active =
        balance_phase_active &&
        adaptive_tempo_;
    const double tempo_governor_scale =
        TempoGovernorScale(governor_phase_active);

    const double phase_scale =
        !balance_phase_active
            ? 1.0
            : (swing_phase_active ? tempo_scale_ : support_scale_);
    const double applied_tempo_governor_scale =
        governor_phase_active ? tempo_governor_scale : 1.0;
    const double motion_clock_rate = 1.0 / phase_scale;
    last_tempo_governor_scale_ = applied_tempo_governor_scale;
    running_time_ +=
        motion_clock_dt_s * motion_clock_rate *
        applied_tempo_governor_scale;
    std::array<double, kMotorCount> joint_targets = stand_up_joint_pos_;
    commanded_body_shift_x_m_ = 0.0;
    commanded_body_shift_y_m_ = 0.0;
    commanded_foot_lift_height_m_ = 0.0;
    commanded_foot_swing_x_m_ = 0.0;
    commanded_foot_swing_y_m_ = 0.0;
    current_cycle_number_ = 0;

    if (PhaseStandUp(joint_targets))
    {
        // SECTION: stand-up (see PhaseStandUp)
    }
    else if (PhaseStandSettle(joint_targets))
    {
        // SECTION: stand-settle
    }
    else if (PhaseTerminalCorrection(joint_targets))
    {
        // SECTION: terminal-correction (see PhaseTerminalCorrection)
    }
    else if (PhaseWeightShift(joint_targets))
    {
        // SECTION: weight-shift (see PhaseWeightShift)
    }
    else
    {
        if (!cycle_started_)
        {
            cycle_started_ = true;
            ResetStepDiagnostics();
            cycle_start_time_s_ = running_time_;
            landing_detected_ = false;
            landing_time_s_ = 0.0;
            landing_lift_height_m_ = 0.0;
        }

        current_cycle_number_ = repeat_sequence_ ? completed_step_count_ + 1 : cycle_index_ + 1;
        scratch.cycle_time = running_time_ - cycle_start_time_s_;
        commanded_body_shift_x_m_ = body_shift_x_m_;
        commanded_body_shift_y_m_ = body_shift_y_m_;

        if (PhaseFootLift(joint_targets, scratch))
        {
            // SECTION: foot-lift (see PhaseFootLift)
        }
        else if (PhaseFootSwing(joint_targets, scratch))
        {
            // SECTION: foot-swing (see PhaseFootSwing)
        }
        else
        {
            scratch.lower_time =
                scratch.cycle_time -
                kFootLiftDuration -
                kFootSwingDuration;

            commanded_foot_swing_x_m_ = swing_x_m_ + yaw_feedback_swing_x_m_;
            commanded_foot_swing_y_m_ = swing_y_m_;

            if (PhaseFootLowerActive(joint_targets, scratch))
            {
                // SECTION: foot-lower-active (see PhaseFootLowerActive)
            }

            if (landing_detected_)
            {
                scratch.since_landing =
                    running_time_ - landing_time_s_;
                // post-touchdown chain
                if (PhaseLandingHold(joint_targets, scratch))
                {
                    // SECTION: landing-hold (see PhaseLandingHold)
                }
                else if (PhaseBetweenCycles(joint_targets, scratch))
                {
                    // see PhaseBetweenCycles
                }
                else if (PhaseBodyReturn(joint_targets, scratch))
                {
                    // SECTION: body-return (see PhaseBodyReturn)
                }
                else if (PhaseNeutralSettle(joint_targets, scratch))
                {
                    return;
                }
            }
        }
    }

    UpdateAttitudeFeedback();

    if (!ApplyTaskSpaceIk(joint_targets))
        return;
    WriteMotorCommands(joint_targets);

    // SECTION: publish-lowcmd
    PublishLowCmdWithCrc();
    LogSample();

    if (all_cycles_complete_ &&
        terminal_correction_started_ &&
        running_time_ - all_cycles_complete_time_s_ >=
            (world_feedback_enabled_ ? kTerminalCorrectionDuration : 0.0) +
            kFinalHoldDuration)
    {
        if (!ValidateStepDiagnostics())
        {
            finished_.store(true);
            return;
        }
        LogTerminalWorldError();
        finished_.store(true);
        csv_.flush();
        std::cout << "Finished. CSV saved to " << csv_path_ << std::endl;
    }
    else if (running_time_ >= duration_s_)
    {
        finished_.store(true);
        csv_.flush();
        const std::string progress = repeat_sequence_
            ? std::to_string(completed_step_count_) +
                  " steps (repeat sequence length " +
                  std::to_string(cycle_count_) + ")"
            : std::to_string(cycle_index_) + "/" +
                  std::to_string(cycle_count_) + " cycles";
        std::cerr << "Timed out after completing " << progress
            << ". CSV saved to "
            << csv_path_ << std::endl;
    }
}

void TrackingExperiment::PublishLowCmdWithCrc()
{
    low_cmd_.crc() = crc32_core(
        (uint32_t *)&low_cmd_,
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher_->Write(low_cmd_);
}

bool TrackingExperiment::PhaseStandUp(std::array<double, kMotorCount> &joint_targets)
{
    if (!(running_time_ < kStandUpDuration))
        return false;

    motion_stage_ = 0;
    // SECTION: stand-up
    phase_ = smoothstep(running_time_ / kStandUpDuration);
    for (int i = 0; i < kMotorCount; i++)
    {
        joint_targets[i] =
            phase_ * stand_up_joint_pos_[i] +
            (1.0 - phase_) * start_joint_pos_[i];
    }

    return true;
}

bool TrackingExperiment::PhaseStandSettle(
    std::array<double, kMotorCount> &joint_targets)
{
    if (!(running_time_ >= kStandUpDuration &&
          running_time_ < kStandUpDuration + kStandSettleDuration))
        return false;

    motion_stage_ = 1;
    phase_ = 0.0;

    (void)joint_targets;
    return true;
}

bool TrackingExperiment::PhaseWeightShift(
    std::array<double, kMotorCount> &joint_targets)
{
    if (!(!initial_weight_shift_complete_))
        return false;

    current_cycle_number_ = repeat_sequence_ ? completed_step_count_ + 1 : cycle_index_ + 1;
    const double shift_time = running_time_ - weight_shift_start_time_s_;
    if (shift_time < kWeightShiftDuration)
    {
        // SECTION: weight-shift
        phase_ = smoothstep(shift_time / kWeightShiftDuration);
        motion_stage_ = 2;
        commanded_body_shift_x_m_ =
            body_shift_start_x_m_ +
            phase_ * (body_shift_x_m_ - body_shift_start_x_m_);
        commanded_body_shift_y_m_ =
            body_shift_start_y_m_ +
            phase_ * (body_shift_y_m_ - body_shift_start_y_m_);
    }
    else
    {
        motion_stage_ = 3;
        commanded_body_shift_x_m_ = body_shift_x_m_;
        commanded_body_shift_y_m_ = body_shift_y_m_;
        if (shift_time >=
            kWeightShiftDuration + kWeightShiftSettleDuration)
        {
            initial_weight_shift_complete_ = true;
            cycle_started_ = true;
            ResetStepDiagnostics();
            cycle_start_time_s_ = running_time_;
        }
    }

    return true;
}

bool TrackingExperiment::PhaseFootLift(
    std::array<double, kMotorCount> &joint_targets,
    LowCmdScratch &scratch)
{
    if (!(scratch.cycle_time < kFootLiftDuration))
        return false;

    motion_stage_ = 4;
    // SECTION: foot-lift
    commanded_foot_lift_height_m_ =
        cycle_start_lift_height_m_ +
        smoothstep(scratch.cycle_time / kFootLiftDuration) *
            (foot_lift_height_m_ -
             cycle_start_lift_height_m_);

    return true;
}

bool TrackingExperiment::PhaseLandingHold(
    std::array<double, kMotorCount> &joint_targets,
    LowCmdScratch &scratch)
{
    if (!(scratch.since_landing < kFootLowerSettleDuration))
        return false;

    motion_stage_ = 7;
    commanded_foot_lift_height_m_ = landing_lift_height_m_;
    (void)joint_targets;
    return true;
}

bool TrackingExperiment::PhaseFootSwing(
    std::array<double, kMotorCount> &joint_targets,
    LowCmdScratch &scratch)
{
    if (!(scratch.cycle_time >= kFootLiftDuration &&
          scratch.cycle_time < kFootLiftDuration + kFootSwingDuration))
        return false;

    motion_stage_ = 5;
    commanded_foot_lift_height_m_ = foot_lift_height_m_;
    // SECTION: foot-swing
    const double swing_phase = smoothstep(
        (scratch.cycle_time - kFootLiftDuration) / kFootSwingDuration);
    commanded_foot_swing_x_m_ =
        swing_phase * (swing_x_m_ + yaw_feedback_swing_x_m_);
    commanded_foot_swing_y_m_ = swing_phase * swing_y_m_;
    (void)joint_targets;
    return true;
}

bool TrackingExperiment::PhaseBodyReturn(
    std::array<double, kMotorCount> &joint_targets,
    LowCmdScratch &scratch)
{
    if (!(scratch.since_landing >= kFootLowerSettleDuration &&
          scratch.since_landing <
              kFootLowerSettleDuration + kBodyReturnDuration))
        return false;

    motion_stage_ = 8;
    scratch.return_time =
        scratch.since_landing - kFootLowerSettleDuration;
    // SECTION: body-return
    const double return_phase = smoothstep(
        scratch.return_time / kBodyReturnDuration);
    const double return_scale = 1.0 - return_phase;
    commanded_foot_lift_height_m_ =
        return_scale * landing_lift_height_m_;
    commanded_body_shift_x_m_ =
        return_scale * body_shift_x_m_ +
        return_phase * body_advance_x_m_;
    commanded_body_shift_y_m_ =
        return_scale * body_shift_y_m_ +
        return_phase * body_advance_y_m_;

    (void)joint_targets;
    return true;
}

bool TrackingExperiment::PhaseFootLowerActive(
    std::array<double, kMotorCount> &joint_targets,
    LowCmdScratch &scratch)
{
    if (landing_detected_)
        return false;

    motion_stage_ = 6;
    commanded_foot_lift_height_m_ =
        (1.0 - smoothstep(
                   scratch.lower_time / kFootLowerDuration)) *
        foot_lift_height_m_;

    double lift_leg_force = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_low_state_)
        {
            lift_leg_force = low_state_.foot_force()[
                static_cast<std::size_t>(lift_leg_)];
        }
    }
    if (lift_leg_force >= kLandingForceThreshold ||
        scratch.lower_time >= kFootLowerDuration)
    {
        landing_detected_ = true;
        landing_time_s_ = running_time_;
        landing_lift_height_m_ =
            commanded_foot_lift_height_m_;
        std::cout
            << "Cycle " << current_cycle_number_
            << " landing detected at t=" << running_time_
            << " s, " << lift_leg_name_ << " force="
            << lift_leg_force << std::endl;
    }

    (void)joint_targets;
    return true;
}

bool TrackingExperiment::PhaseTerminalCorrection(
    std::array<double, kMotorCount> &joint_targets)
{
    if (!(all_cycles_complete_))
        return false;

    motion_stage_ = 10;
    current_cycle_number_ = cycle_count_;
    BeginTerminalCorrection();
    // SECTION: terminal-correction
    const double terminal_phase = smoothstep(
        (running_time_ - terminal_correction_start_time_s_) /
        kTerminalCorrectionDuration);
    commanded_body_shift_x_m_ =
        body_advance_x_m_ + terminal_phase * terminal_correction_x_m_;
    commanded_body_shift_y_m_ =
        body_advance_y_m_ + terminal_phase * terminal_correction_y_m_;

    return true;
}

bool TrackingExperiment::PhaseBetweenCycles(
    std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch)
{
    if (!(!sequence_mode_ &&
                         cycle_index_ + 1 < cycle_count_))
        return false;

        motion_stage_ = 11;
        current_cycle_number_ = repeat_sequence_ ? completed_step_count_ + 2 : cycle_index_ + 2;
        scratch.between_cycle_time =
            scratch.since_landing - kFootLowerSettleDuration;
        commanded_foot_lift_height_m_ =
            landing_lift_height_m_;
        if (scratch.between_cycle_time >=
            kBetweenCycleSettleDuration)
        {
            cycle_start_lift_height_m_ =
                landing_lift_height_m_;
            ++cycle_index_;
            landing_detected_ = false;
            landing_time_s_ = 0.0;
            landing_lift_height_m_ = 0.0;
            cycle_start_time_s_ = running_time_;
        }

    return true;
}

bool TrackingExperiment::ApplyTaskSpaceIk(
    std::array<double, kMotorCount> &joint_targets)
{
    if (motion_stage_ < 2)
        return true;

    auto target_feet = go2::AllFootPositions(stand_up_joint_pos_);
    for (std::size_t i = 0; i < go2::kLegCount; ++i)
    {
        target_feet[i].x +=
            placed_foot_offset_x_m_[i] + yaw_placed_offset_x_m_[i];
        target_feet[i].y += placed_foot_offset_y_m_[i];
    }
    for (auto &foot : target_feet)
    {
        foot.x -= commanded_body_shift_x_m_;
        foot.y -= commanded_body_shift_y_m_;
    }
    if (yaw_feedback_enabled_ && have_world_reference_)
    {
        const double yaw_trim = yaw_feedback_body_rotation_rad_;
        const double trim_cos = std::cos(yaw_trim);
        const double trim_sin = std::sin(yaw_trim);
        for (auto &foot : target_feet)
        {
            const double x = foot.x;
            const double y = foot.y;
            foot.x = trim_cos * x - trim_sin * y;
            foot.y = trim_sin * x + trim_cos * y;
        }
    }
    target_feet[static_cast<std::size_t>(lift_leg_)].x +=
        commanded_foot_swing_x_m_;
    target_feet[static_cast<std::size_t>(lift_leg_)].y +=
        commanded_foot_swing_y_m_;
    target_feet[static_cast<std::size_t>(lift_leg_)].z +=
        commanded_foot_lift_height_m_;

    if (!go2::AllLegInverseKinematics(target_feet, joint_targets))
    {
        std::cerr << "IK failed during weight shift or foot lift\n";
        finished_.store(true);
        return false;
    }

    return true;
}

void TrackingExperiment::WriteMotorCommands(
    const std::array<double, kMotorCount> &joint_targets)
{
    for (int i = 0; i < kMotorCount; i++)
    {
        low_cmd_.motor_cmd()[i].q() = joint_targets[i];
        low_cmd_.motor_cmd()[i].dq() = 0;
        if (motion_stage_ == 0)
        {
            low_cmd_.motor_cmd()[i].kp() =
                phase_ * 100.0 + (1.0 - phase_) * 20.0;
        }
        else
        {
            low_cmd_.motor_cmd()[i].kp() = 100.0;
        }
        low_cmd_.motor_cmd()[i].kd() = 3.5;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}

void TrackingExperiment::UpdateAttitudeFeedback()
{
    if (motion_stage_ < 2)
        return;

    attitude_feedback_x_m_ = 0.0;
    attitude_feedback_y_m_ = 0.0;
    double imu_roll = 0.0;
    double imu_pitch = 0.0;
    bool have_imu = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_low_state_)
        {
            imu_roll = low_state_.imu_state().rpy()[0];
            imu_pitch = low_state_.imu_state().rpy()[1];
            have_imu = true;
        }
    }
    if (have_imu)
    {
        constexpr double alpha = kAttitudeFeedbackFilterAlpha;
        attitude_feedback_roll_rad_ +=
            alpha * (imu_roll - attitude_feedback_roll_rad_);
        attitude_feedback_pitch_rad_ +=
            alpha * (imu_pitch - attitude_feedback_pitch_rad_);
        attitude_feedback_x_m_ = std::max(
            -kAttitudeFeedbackMaxCorrectionM,
            std::min(
                kAttitudeFeedbackMaxCorrectionM,
                -kPitchFeedbackBodyShiftGainMPerRad *
                    attitude_feedback_pitch_rad_));
        attitude_feedback_y_m_ = std::max(
            -kAttitudeFeedbackMaxCorrectionM,
            std::min(
                kAttitudeFeedbackMaxCorrectionM,
                kRollFeedbackBodyShiftGainMPerRad *
                    attitude_feedback_roll_rad_));
        commanded_body_shift_x_m_ += attitude_feedback_x_m_;
        commanded_body_shift_y_m_ += attitude_feedback_y_m_;
    }

}

bool TrackingExperiment::PhaseNeutralSettle(
    std::array<double, kMotorCount> &joint_targets,
    LowCmdScratch &scratch)
{
    // Post-return phase: place footholds and advance/loop/finish cycles.
    // Returns true when LowCmdWrite must stop this tick (terminal conditions).

    motion_stage_ = 9;
    commanded_foot_lift_height_m_ = 0.0;
    commanded_body_shift_x_m_ = body_advance_x_m_;
    commanded_body_shift_y_m_ = body_advance_y_m_;

    const double neutral_settle_time =
        scratch.since_landing -
        kFootLowerSettleDuration -
        kBodyReturnDuration;
    if (neutral_settle_time >=
        kBetweenCycleSettleDuration)
    {
        const std::size_t completed_leg =
            static_cast<std::size_t>(lift_leg_);
        placed_foot_offset_x_m_[completed_leg] +=
            swing_x_m_;
        const double yaw_offset_max_m =
            kWorldYawPlacedOffsetMaxM;
        yaw_placed_offset_x_m_[completed_leg] = std::max(
            -yaw_offset_max_m,
            std::min(
                yaw_offset_max_m,
                yaw_placed_offset_x_m_[completed_leg] +
                    yaw_feedback_swing_x_m_));
        // Yaw correction is a transient placement bias for
        // the current step. Do not integrate it into the
        // periodic nominal foothold, or it will accumulate
        // into a leg-specific roll bias over long runs.
        placed_foot_offset_y_m_[completed_leg] +=
            swing_y_m_;
        const double next_body_start_x =
            body_advance_x_m_;
        const double next_body_start_y =
            body_advance_y_m_;
        cycle_start_lift_height_m_ =
            sequence_mode_ ? 0.0 : landing_lift_height_m_;
        if (cycle_index_ + 1 < cycle_count_)
        {
            if (!ValidateStepDiagnostics())
            {
                finished_.store(true);
                return true;
            }
            if (!UpdateWorldFeedbackForNextStep())
            {
                finished_.store(true);
                return true;
            }
            ++completed_step_count_;
            if (max_completed_steps_ > 0 &&
                completed_step_count_ >= max_completed_steps_)
            {
                std::cout << "Stopped after completing "
                          << completed_step_count_
                          << " steps in repeat mode"
                          << std::endl;
                finished_.store(true);
                csv_.flush();
                return true;
            }
            ++cycle_index_;
            ApplyStepConfig(
                steps_[static_cast<std::size_t>(
                    cycle_index_)]);
            body_shift_start_x_m_ = next_body_start_x;
            body_shift_start_y_m_ = next_body_start_y;
            weight_shift_start_time_s_ = running_time_;
            initial_weight_shift_complete_ = !sequence_mode_;
            cycle_started_ = false;
            landing_detected_ = false;
            landing_time_s_ = 0.0;
            landing_lift_height_m_ = 0.0;
            commanded_foot_swing_x_m_ = 0.0;
            commanded_foot_swing_y_m_ = 0.0;
            commanded_body_shift_x_m_ =
                next_body_start_x;
            commanded_body_shift_y_m_ =
                next_body_start_y;
            motion_stage_ = 11;
            current_cycle_number_ = cycle_index_ + 1;
        }
        else if (repeat_sequence_)
        {
            if (!ValidateStepDiagnostics())
            {
                finished_.store(true);
                return true;
            }
            if (!UpdateWorldFeedbackForNextStep())
            {
                finished_.store(true);
                return true;
            }
            ++completed_step_count_;
            if (max_completed_steps_ > 0 &&
                completed_step_count_ >= max_completed_steps_)
            {
                std::cout << "Repeated sequence: completed "
                          << completed_step_count_
                          << " steps at t=" << running_time_
                          << " s" << std::endl;
                std::cout << "Stopped after completing "
                          << completed_step_count_
                          << " steps in repeat mode"
                          << std::endl;
                finished_.store(true);
                csv_.flush();
                return true;
            }
            sequence_offset_x_m_ += steps_.back().body_advance_x_m;
            sequence_offset_y_m_ += steps_.back().body_advance_y_m;
            cycle_index_ = 0;
            ApplyStepConfig(steps_.front());
            body_shift_start_x_m_ = next_body_start_x;
            body_shift_start_y_m_ = next_body_start_y;
            weight_shift_start_time_s_ = running_time_;
            initial_weight_shift_complete_ = false;
            cycle_started_ = false;
            landing_detected_ = false;
            landing_time_s_ = 0.0;
            landing_lift_height_m_ = 0.0;
            commanded_foot_swing_x_m_ = 0.0;
            commanded_foot_swing_y_m_ = 0.0;
            commanded_body_shift_x_m_ = next_body_start_x;
            commanded_body_shift_y_m_ = next_body_start_y;
            motion_stage_ = 11;
            current_cycle_number_ = completed_step_count_ + 1;
            std::cout << "Repeated sequence: completed "
                      << completed_step_count_
                      << " steps at t=" << running_time_
                      << " s" << std::endl;
        }
        else
        {
            if (!ValidateStepDiagnostics())
            {
                finished_.store(true);
                return true;
            }
            ++completed_step_count_;
            all_cycles_complete_ = true;
            all_cycles_complete_time_s_ = running_time_;
            motion_stage_ = 10;
            current_cycle_number_ = cycle_count_;
            commanded_foot_swing_x_m_ = 0.0;
            commanded_foot_swing_y_m_ = 0.0;
            std::cout << "Completed " << cycle_count_
                      << " steps at t=" << running_time_
                      << " s" << std::endl;
        }
    }
    return false;
}

double TrackingExperiment::TempoGovernorScale(bool governor_phase_active)
{
    double tempo_governor_scale = 1.0;
    if (governor_phase_active)
    {
        double max_abs_q_error = 0.0;
        double max_abs_tau_est = 0.0;
        double max_abs_attitude = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_low_state_)
            {
                for (int i = 0; i < kMotorCount; ++i)
                {
                    max_abs_q_error = std::max(
                        max_abs_q_error,
                        std::abs(
                            static_cast<double>(
                                low_cmd_.motor_cmd()[i].q()) -
                            static_cast<double>(
                                low_state_.motor_state()[i].q())));
                    max_abs_tau_est = std::max(
                        max_abs_tau_est,
                        std::abs(static_cast<double>(
                            low_state_.motor_state()[i].tau_est())));
                }
                max_abs_attitude = std::max(
                    std::abs(static_cast<double>(
                        low_state_.imu_state().rpy()[0])),
                    std::abs(static_cast<double>(
                        low_state_.imu_state().rpy()[1])));
            }
        }
        const auto margin_scale = [](double value, double soft, double hard)
        {
            if (value <= soft)
                return 1.0;
            if (value >= hard)
                return kTempoGovernorMinScale;
            return std::max(
                kTempoGovernorMinScale,
                (hard - value) / (hard - soft));
        };
        tempo_governor_scale = std::min(
            margin_scale(
                max_abs_q_error,
                kTempoGovernorSoftJointErrorRad,
                kSafetyMaxJointErrorRad),
            std::min(
                margin_scale(
                    max_abs_tau_est,
                    kTempoGovernorSoftTauEst,
                    kSafetyMaxTauEst),
                margin_scale(
                    max_abs_attitude,
                    kTempoGovernorSoftAttitudeRad,
                    kSafetyMaxAbsPitchRad)));
    }

    return tempo_governor_scale;
}
