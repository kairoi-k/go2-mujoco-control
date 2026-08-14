#pragma once
// Quasi-static multi-step leg lift / walk-sequence controller.

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "leg_lift_types.h"

using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriberPtr;
using unitree::common::ThreadPtr;
using namespace go2_leg;

#ifndef GO2_LEG_TOPIC_LOWCMD
#define GO2_LEG_TOPIC_LOWCMD "rt/lowcmd"
#define GO2_LEG_TOPIC_LOWSTATE "rt/lowstate"
#define GO2_LEG_TOPIC_HIGHSTATE "rt/sportmodestate"
#endif

class TrackingExperiment
{
public:
    TrackingExperiment(
        double duration_s,
        const std::string &csv_path,
        std::vector<StepConfig> steps,
        bool sequence_mode,
        bool repeat_sequence,
        int max_completed_steps,
        bool world_feedback_enabled,
        bool yaw_feedback_enabled,
        double tempo_scale,
        double support_scale,
        bool adaptive_tempo)
        : duration_s_(duration_s),
          world_feedback_enabled_(world_feedback_enabled),
          yaw_feedback_enabled_(yaw_feedback_enabled),
          tempo_scale_(tempo_scale),
          support_scale_(support_scale),
          adaptive_tempo_(adaptive_tempo),
          csv_path_(csv_path),
          steps_(std::move(steps)),
          sequence_mode_(sequence_mode),
          repeat_sequence_(repeat_sequence),
          max_completed_steps_(max_completed_steps),
          cycle_count_(static_cast<int>(steps_.size())),
          body_shift_x_m_(steps_.front().body_shift_x_m),
          body_shift_y_m_(steps_.front().body_shift_y_m),
          foot_lift_height_m_(steps_.front().foot_lift_height_m),
          lift_leg_(steps_.front().lift_leg),
          lift_leg_name_(kLegNames[static_cast<std::size_t>(steps_.front().lift_leg)]),
          swing_x_m_(steps_.front().swing_x_m),
          swing_y_m_(steps_.front().swing_y_m),
          body_advance_x_m_(steps_.front().body_advance_x_m),
          body_advance_y_m_(steps_.front().body_advance_y_m)
    {
        ResetStepDiagnostics();
    }

    bool Init();
    bool Finished() const { return finished_.load(); }
    void Shutdown();

private:
    void InitLowCmd();
    void WriteCsvHeader();
    bool WaitForNaturalSettle(double timeout_s);
    void LowStateMessageHandler(const void *message);
    void HighStateMessageHandler(const void *message);
    struct LowCmdScratch {
        double cycle_time = 0.0;
        double since_landing = 0.0;
        double lower_time = 0.0;
        double return_time = 0.0;
        double between_cycle_time = 0.0;
    };

    void LowCmdWrite();
    double MotionClockStep();
    double TempoGovernorScale(bool governor_phase_active);
    bool PhaseFootLift(std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch);
    bool PhaseFootSwing(std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch);
    bool PhaseFootLowerActive(std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch);
    bool PhaseLandingHold(std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch);
    bool PhaseBodyReturn(std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch);
    bool PhaseBetweenCycles(std::array<double, kMotorCount> &joint_targets, LowCmdScratch &scratch);
    void PublishLowCmdWithCrc();
    void UpdateAttitudeFeedback();
    bool PhaseNeutralSettle(std::array<double, kMotorCount> &joint_targets,
                            LowCmdScratch &scratch);
    bool ApplyTaskSpaceIk(std::array<double, kMotorCount> &joint_targets);
    void WriteMotorCommands(const std::array<double, kMotorCount> &joint_targets);
    bool PhaseStandUp(std::array<double, kMotorCount> &joint_targets);
    bool PhaseTerminalCorrection(std::array<double, kMotorCount> &joint_targets);
    bool PhaseStandSettle(std::array<double, kMotorCount> &joint_targets);
    bool PhaseWeightShift(std::array<double, kMotorCount> &joint_targets);

    void LogSample();
    bool CheckMotionTargets() const;
    void ApplyStepConfig(const StepConfig &step);
    bool CaptureWorldReference();
    bool ReadWorldPose(WorldPose &pose);
    bool UpdateWorldFeedbackForNextStep();
    void ResetStepDiagnostics();
    void UpdateStepDiagnostics(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        const std::array<double, kMotorCount> &target_joint_positions,
        const std::array<double, kMotorCount> &state_joint_positions,
        const std::array<go2::Vec3, go2::kLegCount> &world_feet,
        bool have_world_feet);
    bool ValidateStepDiagnostics() const;
    void LogTerminalWorldError();
    void BeginTerminalCorrection();

private:
    const std::array<double, kMotorCount> stand_up_joint_pos_ = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};

    std::array<double, kMotorCount> start_joint_pos_{};

    const double dt_ = 0.002;
    const double duration_s_;
    const bool world_feedback_enabled_;
    const bool yaw_feedback_enabled_;
    const double tempo_scale_;
    const double support_scale_;
    const bool adaptive_tempo_;
    const std::string csv_path_;
    const std::vector<StepConfig> steps_;
    const bool sequence_mode_;
    const bool repeat_sequence_;
    const int max_completed_steps_;
    const int cycle_count_;
    double sequence_offset_x_m_ = 0.0;
    double sequence_offset_y_m_ = 0.0;
    int completed_step_count_ = 0;
    double body_shift_x_m_;
    double body_shift_y_m_;
    double foot_lift_height_m_;
    go2::Leg lift_leg_;
    std::string lift_leg_name_;
    double swing_x_m_;
    double swing_y_m_;
    double body_advance_x_m_;
    double body_advance_y_m_;
    double world_feedback_x_m_ = 0.0;
    double world_feedback_y_m_ = 0.0;
    double world_yaw_error_rad_ = 0.0;
    double yaw_feedback_swing_x_m_ = 0.0;
    double yaw_feedback_body_rotation_rad_ = 0.0;
    double yaw_feedback_y_m_ = 0.0;
    double world_position_error_x_m_ = 0.0;
    double world_position_error_y_m_ = 0.0;
    double world_reference_x_m_ = 0.0;
    double world_reference_y_m_ = 0.0;
    double world_reference_yaw_rad_ = 0.0;
    bool have_world_reference_ = false;
    int world_feedback_update_count_ = 0;
    double terminal_world_position_error_x_m_ = 0.0;
    double terminal_world_position_error_y_m_ = 0.0;
    bool terminal_correction_started_ = false;
    double terminal_correction_start_time_s_ = 0.0;
    double terminal_correction_x_m_ = 0.0;
    double terminal_correction_y_m_ = 0.0;
    StepDiagnostics step_diagnostics_{};
    double last_tempo_governor_scale_ = 1.0;
    double attitude_feedback_roll_rad_ = 0.0;
    double attitude_feedback_pitch_rad_ = 0.0;
    double attitude_feedback_x_m_ = 0.0;
    double attitude_feedback_y_m_ = 0.0;

    double running_time_ = 0.0;
    double last_state_tick_s_for_clock_ = 0.0;
    bool have_last_state_tick_for_clock_ = false;
    double last_motion_clock_dt_s_ = 0.0;
    double last_state_tick_gap_s_ = 0.0;
    bool last_motion_clock_paused_ = false;
    int state_clock_pause_count_ = 0;
    double phase_ = 0.0;
    double commanded_body_shift_x_m_ = 0.0;
    double commanded_body_shift_y_m_ = 0.0;
    double commanded_foot_lift_height_m_ = 0.0;
    double commanded_foot_swing_x_m_ = 0.0;
    double commanded_foot_swing_y_m_ = 0.0;
    double landing_time_s_ = 0.0;
    double landing_lift_height_m_ = 0.0;
    double cycle_start_lift_height_m_ = 0.0;
    bool landing_detected_ = false;
    int motion_stage_ = 0;
    int cycle_index_ = 0;
    int current_cycle_number_ = 0;
    bool cycle_started_ = false;
    double cycle_start_time_s_ = 0.0;
    double weight_shift_start_time_s_ =
        kStandUpDuration + kStandSettleDuration;
    double body_shift_start_x_m_ = 0.0;
    double body_shift_start_y_m_ = 0.0;
    std::array<double, go2::kLegCount> placed_foot_offset_x_m_{};
    std::array<double, go2::kLegCount> placed_foot_offset_y_m_{};
    std::array<double, go2::kLegCount> yaw_placed_offset_x_m_{};
    bool initial_weight_shift_complete_ = false;
    bool all_cycles_complete_ = false;
    double all_cycles_complete_time_s_ = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    unitree_go::msg::dds_::SportModeState_ high_state_{};
    bool have_low_state_ = false;
    bool have_high_state_ = false;
    bool have_start_joint_pos_ = false;

    std::mutex state_mutex_;
    std::ofstream csv_;
    std::atomic<bool> finished_{false};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
        highstate_subscriber_;
    ThreadPtr low_cmd_write_thread_;
};
