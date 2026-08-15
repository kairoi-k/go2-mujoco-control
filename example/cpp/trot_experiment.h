#pragma once
// Diagonal-trot experiment controller (DDS in/out, gait, WBC, CSV).

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "go2_contact_torque_mapping.h"
#include "locomotion_kernel.h"
#include "trot_types.h"
#include "velocity_filter.h"

using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriberPtr;

// DDS topics (shared by lifecycle Init).
#ifndef GO2_TROT_TOPIC_LOWCMD
#define GO2_TROT_TOPIC_LOWCMD "rt/lowcmd"
#define GO2_TROT_TOPIC_LOWSTATE "rt/lowstate"
#define GO2_TROT_TOPIC_HIGHSTATE "rt/sportmodestate"
#endif

class TrotExperiment
{
public:
    TrotExperiment(
        double duration_s,
        const std::string &csv_path,
        go2_trot::TrotParams params,
        int max_cycles,
        bool continuous_mode,
        const std::string &stop_file_path,
        bool task_mode)
        : duration_s_(duration_s),
          csv_path_(csv_path),
          params_(std::move(params)),
          max_cycles_(max_cycles),
          continuous_mode_(continuous_mode),
          stop_file_path_(stop_file_path),
          task_mode_(task_mode),
          locomotion_kernel_(go2_trot::CreateLocomotionKernel(params_)),
          velocity_filter_({params_.velocity_filter_cutoff_hz})
    {
    }

    bool Init();
    bool Finished() const { return finished_.load(); }
    void RequestStop();
    bool StopFileRequested() const;
    void Shutdown();

private:
    void InitLowCmd();
    void WriteCsvHeader();
    bool WaitForNaturalSettle(double timeout_s);
    bool CaptureWorldReference();
    void LowStateMessageHandler(const void *message);
    void HighStateMessageHandler(const void *message);
    void LowCmdWrite();
    bool UpdateWbcShadowAndTorqueFf(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state,
        std::array<double, go2_trot::kMotorCount> &wbc_torque_ff);
    void UpdateJointVelocityFeedforward(
        const std::array<double, go2_trot::kMotorCount> &joint_targets,
        double motion_dt, bool motion_clock_paused,
        std::array<double, go2_trot::kMotorCount> &joint_velocities);
    void UpdateGaitWorldDiagnostics(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state,
        const std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseRunGait(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state,
        std::array<double, go2_trot::kMotorCount> &joint_targets);
    void WriteMotorCommands(
        bool wbc_primary_active, double gait_elapsed_s,
        const std::array<double, go2_trot::kMotorCount> &joint_targets,
        const std::array<double, go2_trot::kMotorCount> &joint_velocities,
        const std::array<double, go2_trot::kMotorCount> &wbc_torque_ff,
        bool apply_wbc_torque_ff);
    bool SnapshotState(
        unitree_go::msg::dds_::LowState_ &state_snapshot,
        unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool &have_state, bool &have_high_state);
    double MotionClockStep(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool &motion_clock_paused);
    bool ComputeWbcPrimaryActive(double &gait_elapsed_s);
    bool PhaseStandUp(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseStandSettle(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseStartGait(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseStopToStand(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseLieDown(std::array<double, go2_trot::kMotorCount> &joint_targets);
    void PublishLowCmdWithCrc();
    void LogSample(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state);
    bool BuildGaitTargets(
        double gait_time_s,
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state,
        std::array<double, go2_trot::kMotorCount> &joint_targets);
    void UpdateWbcShadow(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state);
    bool PrepareWbcTorqueFeedforward(
        std::array<double, go2_trot::kMotorCount> &torque_ff);
    void UpdateVelocityEstimate(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state,
        double motion_dt);
    void UpdateCycleDiagnostics(
        double phase,
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        const std::array<double, go2_trot::kMotorCount> &joint_targets,
        bool have_world_feet,
        const std::array<go2::Vec3, go2::kLegCount> &world_feet,
        const go2_trot::WorldPose &world_pose);
    bool ValidateCycle(int cycle_index);
    bool CheckInstantaneousHardLimits(
        const std::array<double, go2_trot::kMotorCount> &joint_targets,
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        bool have_state,
        bool primary_active) const;
    void ResetCycleDiagnostics();

private:
    const std::array<double, go2_trot::kMotorCount> stand_up_joint_pos_ = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};
    const std::array<double, go2_trot::kMotorCount> stand_down_joint_pos_ = {
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375,
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375};

    std::array<double, go2_trot::kMotorCount> start_joint_pos_{};
    std::array<double, go2_trot::kMotorCount> previous_joint_targets_{};
    bool have_start_joint_pos_ = false;
    bool have_previous_joint_targets_ = false;

    const double dt_ = go2_trot::kDt;
    const double duration_s_;
    const std::string csv_path_;
    const go2_trot::TrotParams params_;
    const int max_cycles_;
    const bool continuous_mode_;
    const std::string stop_file_path_;
    const bool task_mode_;
    std::unique_ptr<go2_control::LocomotionKernel> locomotion_kernel_;
    go2_control::FirstOrderVelocityFilter velocity_filter_;
    go2_control::Vector3 latest_world_velocity_{};
    go2_control::Vector3 latest_raw_body_velocity_{};
    go2_control::Vector3 latest_filtered_body_velocity_{};
    bool have_world_velocity_ = false;
    bool have_raw_body_velocity_ = false;
    bool have_filtered_body_velocity_ = false;
    go2_control::Vector3 latest_body_acceleration_{};
    go2_control::Vector3 latest_body_acceleration_prev_v_{};
    bool have_body_acceleration_ = false;
    double velocity_filter_alpha_ = 0.0;
    std::array<go2::Vec3, go2::kLegCount> commanded_body_feet_{};
    bool have_commanded_body_feet_ = false;
    std::array<bool, go2::kLegCount> previous_leg_swing_{};
    std::array<bool, go2::kLegCount> touchdown_recorded_{};
    std::array<bool, go2::kLegCount> touchdown_waiting_contact_{};
    bool have_leg_phase_history_ = false;
    int touchdown_event_count_ = 0;
    int last_touchdown_leg_ = -1;
    double last_touchdown_command_x_m_ = 0.0;
    double last_touchdown_actual_x_m_ = 0.0;
    double last_touchdown_x_error_m_ = 0.0;
    double last_touchdown_y_error_m_ = 0.0;

    double running_time_ = 0.0;
    double gait_start_time_s_ = 0.0;
    double last_state_tick_s_ = 0.0;
    bool have_last_state_tick_ = false;
    double last_motion_dt_s_ = 0.0;
    double last_state_tick_gap_s_ = 0.0;
    bool last_clock_paused_ = false;
    int clock_pause_count_ = 0;
    std::chrono::steady_clock::time_point last_writer_time_{};
    bool have_last_writer_time_ = false;
    double last_wall_motion_dt_s_ = 0.0;
    bool last_motion_clock_paused_ = false;
    int motion_clock_pause_count_ = 0;

    int active_cycle_index_ = -1;
    int completed_cycles_ = 0;
    bool gait_started_ = false;
    bool stop_requested_ = false;
    bool task_completion_requested_ = false;
    bool lie_down_started_ = false;
    double lie_down_start_time_s_ = 0.0;
    double stop_start_time_s_ = 0.0;
    std::array<double, go2_trot::kMotorCount> stop_origin_joint_targets_{};
    bool have_stop_origin_joint_targets_ = false;
    std::array<bool, go2::kLegCount> wbc_shadow_contact_state_{};
    std::array<double, go2::kLegCount> wbc_stance_blend_{};
    go2_trot::WbcShadowDiagnostics wbc_shadow_diagnostics_{};
    go2_control::JointTorques wbc_shadow_candidate_torques_{};
    bool dynamics_logged_ = false;
    int motion_stage_ = 0;
    double current_phase_ = 0.0;
    bool kernel_footstep_plan_valid_ = false;
    double kernel_velocity_error_x_mps_ = 0.0;
    std::array<double, go2::kLegCount> kernel_touchdown_target_x_m_{};
    int preview_n_steps_ = 0;
    double preview_touchdown_x_m_ = 0.0;
    double preview_terminal_velocity_x_mps_ = 0.0;
    bool have_preview_terminal_velocity_ = false;

    double world_reference_x_m_ = 0.0;
    double world_reference_y_m_ = 0.0;
    double world_reference_yaw_rad_ = 0.0;
    bool have_world_reference_ = false;
    double world_feedback_x_m_ = 0.0;
    double world_feedback_y_m_ = 0.0;
    double world_yaw_error_rad_ = 0.0;
    double attitude_roll_rad_ = 0.0;
    double attitude_pitch_rad_ = 0.0;
    double attitude_feedback_x_m_ = 0.0;
    double attitude_feedback_y_m_ = 0.0;
    std::array<go2::Vec3, go2::kLegCount> support_anchor_world_feet_{};
    std::array<bool, go2::kLegCount> support_anchor_valid_{};
    std::array<double, go2::kLegCount> support_anchor_start_time_s_{};

    go2_trot::CycleDiagnostics cycle_diagnostics_{};

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    unitree_go::msg::dds_::SportModeState_ high_state_{};
    bool have_low_state_ = false;
    bool have_high_state_ = false;

    std::mutex state_mutex_;
    std::ofstream csv_;
    std::atomic<bool> finished_{false};
    std::atomic<bool> external_stop_requested_{false};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
        highstate_subscriber_;
    std::thread low_cmd_write_thread_;
    std::atomic<bool> writer_stop_{false};
};
