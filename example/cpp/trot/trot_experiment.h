#pragma once
// Diagonal-trot experiment controller (DDS in/out, gait, WBC, CSV).

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/Error_.hpp>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "go2_contact_torque_mapping.h"
#include "lockstep_motion_clock.h"
#include "lockstep_writer_gate.h"
#include "locomotion_kernel.h"
#include "trot_task.h"
#include "trot_types.h"
#include "velocity_filter.h"
#include "go2_rigid_body.h"
#include "srbd_mpc.h"
#include "inverse_dynamics_wbc.h"
#include "cartesian_world_trot.h"
#include "terrain_model.h"
#include "terrain_execution_consistency.h"
#include "terrain_motion_plan.h"
#include "terrain_planner.h"

using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriberPtr;

// DDS topics (shared by lifecycle Init).
#ifndef GO2_TROT_TOPIC_LOWCMD
#define GO2_TROT_TOPIC_LOWCMD "rt/lowcmd"
#define GO2_TROT_TOPIC_LOWSTATE "rt/lowstate"
#define GO2_TROT_TOPIC_HIGHSTATE "rt/sportmodestate"
#endif
// Order-107 verification-only causal handshake topic: the adapter
// publishes ack{state_seq, command_seq} (unitree Error_ repurposed as a
// sequence-metadata carrier; Error_.source()/state() are uint32_t and carry
// the full-width frozen-state tick and the controller's exact command_seq)
// after every LowCmd write when TROT_LOCKSTEP_ACK=1.
#ifndef GO2_TROT_TOPIC_LOCKSTEP_ACK
#define GO2_TROT_TOPIC_LOCKSTEP_ACK "rt/lockstep/ack"
#endif
#ifndef GO2_TROT_TOPIC_ENVIRONMENT_MAP
#define GO2_TROT_TOPIC_ENVIRONMENT_MAP "rt/go2/environment_heightmap"
#endif
#ifndef GO2_TROT_TOPIC_LIDAR_MAP
#define GO2_TROT_TOPIC_LIDAR_MAP "rt/go2/lidar_heightmap"
#define GO2_TROT_TOPIC_LIDAR_MAP_ENVELOPE "rt/go2/lidar_heightmap_capture_v1"
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
        bool task_mode,
        TrotGoalConfig goal = {})
        : duration_s_(duration_s),
          csv_path_(csv_path),
          params_(std::move(params)),
          max_cycles_(max_cycles),
          continuous_mode_(continuous_mode),
          stop_file_path_(stop_file_path),
          locomotion_kernel_(go2_trot::CreateLocomotionKernel(params_)),
          runtime_event_schedule_(params_.event_schedule),
          velocity_filter_({params_.velocity_filter_cutoff_hz}),
          velocity_command_shaper_(params_.velocity_command_shaper)
    {
        task_.Configure(task_mode, goal);
        motion_event_response_enabled_ =
            params_.reactive_events || params_.auto_environment ||
            !params_.event_schedule.empty() ||
            params_.impact_to_emergency_stop_delay_s >= 0.0;
    }

    bool Init();
    bool Finished() const { return finished_.load(); }
    void RequestStop();
    bool StopFileRequested() const;
    void Shutdown();

#ifdef GO2_TROT_TESTING
    struct TestMotionClockSample
    {
        double motion_dt_s = 0.0;
        double cmd_time_s = 0.0;
        double gait_time_s = 0.0;
        double ramp_time_s = 0.0;
        double governor_time_s = 0.0;
        double stop_time_s = 0.0;
    };

    void TestPrepareMotionClock(std::uint32_t handoff_tick);
    bool TestRunWallClockTick(const unitree_go::msg::dds_::LowState_ &state);
    bool TestRunLockstepTick(const unitree_go::msg::dds_::LowState_ &state);
    TestMotionClockSample TestLastMotionClockSample() const;
#endif

private:
    void EnvironmentHeightMapMessageHandler(const void *message);
    void LidarHeightMapMessageHandler(const void *message);
    void LidarTerrainEnvelopeMessageHandler(const void *message);
    void InitLowCmd();
    void WriteCsvHeader();
    bool WaitForNaturalSettle(double timeout_s);
    bool CaptureWorldReference();
    void LowStateMessageHandler(const void *message);
    void HighStateMessageHandler(const void *message);
    bool LowCmdWrite(
        std::uint32_t expected_state_tick = 0,
        bool enforce_state_tick = false);
    void EngageLockstepWriterIfNeeded();
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
    bool HighSpeedStopBrakeEnabled() const;
    bool WbcStopHoldActive() const;
    bool EmergencyStopStanceBlendActive() const;
    bool EmergencyStopHoldReady() const;
    bool PhaseStandUp(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseStandSettle(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseStartGait(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        std::array<double, go2_trot::kMotorCount> &joint_targets);
    void UpdateMotionEventResponse(
        double gait_elapsed_s, double motion_dt_s,
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state);
    void UpdateRuntimeVelocityCommand(double gait_time_s);
    bool PhaseStopToStand(std::array<double, go2_trot::kMotorCount> &joint_targets);
    bool PhaseLieDown(std::array<double, go2_trot::kMotorCount> &joint_targets);
    double UpdateCartesianForceBlend();
    void PublishLowCmdWithCrc();
    void PublishLockstepAck(std::uint32_t state_seq);
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
    void UpdateWbcFull(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot);
    bool PrepareWbcTorqueFeedforward(
        std::array<double, go2_trot::kMotorCount> &torque_ff);
    void UpdateVelocityEstimate(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state,
        double motion_dt);
    void UpdateTerrainRuntime();
    void TerrainPlannerWorker();
    void PublishTerrainControlSnapshot(
        const unitree_go::msg::dds_::LowState_ &state_snapshot,
        const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
        bool have_high_state);
    void PinCurrentThreadToEnv(const char *env_name);
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

    double WallClockTelemetryTimeS() const
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() -
                   telemetry_start_time_)
            .count();
    }

private:
    struct TerrainPlannerWork
    {
        bool have_map = false;
        bool have_base_pose = false;
        go2_terrain::TerrainMapEnvelope map_envelope{};
        std::uint64_t map_epoch = 0;
        std::uint64_t plan_id = 0;
        go2_terrain::TerrainPlannerInput input{};
    };

    struct TerrainControlSnapshot
    {
        bool valid = false;
        double state_stamp_s = 0.0;
        double base_yaw_rad = 0.0;
        double base_roll_rad = 0.0;
        double base_pitch_rad = 0.0;
        std::array<double, 4> base_quaternion{};
        go2::Vec3 imu_position_world{};
        go2::Vec3 base_velocity_world{};
        go2::Vec3 model_com_world{};
        double model_com_state_stamp_s = 0.0;
        bool model_com_valid = false;
        bool have_base_position_world = false;
        std::array<go2::Vec3, go2::kLegCount> measured_support_anchor_world{};
        std::array<bool, go2::kLegCount> measured_support_anchor_valid{};
        double gait_phase = 0.0;
        double gait_period_s = 0.0;
        double duty_factor = 0.0;
        double commanded_vx_mps = 0.0;
        std::array<double, go2_trot::kMotorCount> joint_positions{};
        std::array<go2::Vec3, go2::kLegCount> nominal_feet_base{};
        std::array<go2::Vec3, go2::kLegCount> touchdown_target_feet_base{};
        bool touchdown_target_feet_valid = false;
        std::array<bool, go2::kLegCount> measured_contact{};
        bool have_commanded_body_feet = false;
        bool measured_valid = false;
    };

    static constexpr double kEmergencyStopPostHoldDurationS = 1.50;

    TrotTask task_;
    std::array<double, go2_trot::kMotorCount> previous_joint_targets_{};
    bool stop_brake_active_ = false;
    double stop_brake_start_time_s_ = 0.0;
    double stop_brake_base_step_m_ = 0.0;
    static constexpr double kStopBrakeDurationS = 0.80;
    // Keep a high-speed stop inside the locomotion/WBC plant before the
    // legacy joint-target return-to-stand interpolation is allowed to run.
    bool high_speed_stop_brake_active_ = false;
    double high_speed_stop_brake_start_time_s_ = 0.0;
    double high_speed_stop_brake_base_speed_mps_ = 0.0;
    double high_speed_stop_brake_base_period_s_ = 0.22;
    double high_speed_stop_brake_base_duty_ = 0.44;
    // Health-triggered braking must be shorter than the normal timed stop:
    // once posture/foothold quality is degrading, waiting two seconds keeps
    // the robot in the failing aerial plant for too long.  The default stays
    // at the validated timed-stop value; sprint profiles may request a
    // bounded faster guard brake.
    double high_speed_stop_brake_duration_s_ = 2.00;
    bool high_speed_stop_hold_active_ = false;
    double high_speed_stop_hold_start_time_s_ = 0.0;
    std::array<double, go2_trot::kMotorCount>
        high_speed_stop_hold_targets_{};
    bool have_high_speed_stop_hold_targets_ = false;
    double high_speed_stop_speed_candidate_start_s_ = -1.0;
    static constexpr double kHighSpeedStopBrakeDurationS = 2.00;
    // Keep the ID-WBC four-contact hold alive long enough for the body and
    // contact forces to settle before ending the bounded sprint process.
    static constexpr double kHighSpeedStopHoldDurationS = 2.00;
    bool have_previous_joint_targets_ = false;

    const double dt_ = go2_trot::kDt;
    const double duration_s_;
    const std::string csv_path_;
    const go2_trot::TrotParams params_;
    const int max_cycles_;
    const bool continuous_mode_;
    const std::string stop_file_path_;
    go2_control::MotionSensorSample latest_motion_sensor_{};
    std::unique_ptr<go2_control::LocomotionKernel> locomotion_kernel_;
    go2_control::MotionEventResponseLayer motion_event_layer_;
    go2_control::MotionEventDetector motion_event_detector_;
    go2_control::MotionEventResponse motion_event_state_{};
    go2_control::MotionReference motion_reference_{};
    go2_control::MotionEvent auto_motion_event_{};
    go2_control::MotionEvent auto_emergency_stop_event_{};
    std::vector<go2_control::MotionEvent> runtime_event_schedule_;
    bool auto_emergency_stop_scheduled_ = false;
    bool motion_event_response_enabled_ = false;
    go2_control::MotionEventType last_motion_event_type_ =
        go2_control::MotionEventType::kNone;
    bool emergency_stop_latched_ = false;
    // Let a running trot finish its support exchange before switching the
    // MPC/WBC contact horizon to four-foot stance.
    double emergency_stop_latch_gait_time_s_ = 0.0;
    double emergency_stop_finish_time_s_ = 0.0;
    go2_control::FirstOrderVelocityFilter velocity_filter_;
    go2_trot::VelocityCommandShaper velocity_command_shaper_;
    go2_trot::ContinuousVelocityGaitScheduler velocity_gait_scheduler_;
    go2_trot::RuntimeVelocityStanceHoldGate velocity_stance_hold_gate_;
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
    std::array<go2::Vec3, go2::kLegCount> commanded_body_feet_velocity_{};
    bool have_commanded_body_feet_ = false;
    bool have_commanded_body_feet_velocity_ = false;
    std::array<go2::Vec3, go2::kLegCount> commanded_world_feet_{};
    bool have_commanded_world_feet_ = false;
    std::array<go2::Vec3, go2::kLegCount> previous_support_foot_world_{};
    std::array<bool, go2::kLegCount> previous_support_foot_valid_{};
    go2_control::CartesianWorldState cartesian_state_{};
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
    std::array<bool, go2::kLegCount> wbc_shadow_contact_state_{};
    std::array<double, go2::kLegCount> wbc_stance_blend_{};
    go2_trot::WbcShadowDiagnostics wbc_shadow_diagnostics_{};
    go2_control::JointTorques wbc_shadow_candidate_torques_{};
    std::unique_ptr<go2_control::Go2RigidBody> rigid_body_;
    go2_control::SrbdMpcOutput last_srbd_{};
    go2_control::IdWbcOutput last_id_wbc_{};
    bool have_last_id_wbc_ = false;
    int wbc_full_ticks_ = 0;
    bool dynamics_logged_ = false;
    double current_phase_ = 0.0;
    bool kernel_footstep_plan_valid_ = false;
    double kernel_velocity_error_x_mps_ = 0.0;
    double kernel_nominal_velocity_x_mps_ = 0.0;
    double kernel_period_s_ = 0.0;
    double kernel_duty_factor_ = 0.0;
    std::size_t step_plan_index_ = 0;
    go2_trot::VelocityCommandState velocity_command_state_{};
    bool velocity_command_initialized_ = false;
    bool runtime_velocity_stance_hold_active_ = false;
    double runtime_gait_step_length_m_ = 0.0;
    double runtime_gait_foot_lift_m_ = 0.0;
    double kernel_effective_foot_lift_m_ = std::numeric_limits<double>::quiet_NaN();
    std::string runtime_gait_regime_ = "inactive";
    std::size_t period_plan_index_ = 0;
    double wbc_speed_cmd_mps_ = -1.0;
    // Optional health-aware cap for the sprint curriculum. It is disabled
    // unless TROT_HS_STABILITY_GOV is set, so legacy profiles keep their
    // exact command trajectory.
    double high_speed_health_cap_mps_ = -1.0;
    int high_speed_health_hold_cycles_ = 0;
    // Temporary support-rich shape hold used by the health governor.  It
    // keeps the commanded speed reference intact while buying contact time
    // after a measurable touchdown-quality degradation.
    int high_speed_health_severe_streak_ = 0;
    // Once a sprint has reached its high-speed regime, keep the per-tick
    // safety guard armed while the health governor is already reducing the
    // reference.  Using only the instantaneous (now falling) speed can make
    // the guard miss the short transition from a bad touchdown to a hard
    // posture excursion.
    bool high_speed_guard_armed_ = false;
    int high_speed_guard_bad_ticks_ = 0;
    int high_speed_preflight_stable_cycles_ = 0;
    double cartesian_last_v_meas_ = -1.0;
    double cartesian_last_foot_error_m_ = 0.0;
    bool cartesian_cruise_latched_ = false;
    double cartesian_force_blend_ = 0.0;
    double cartesian_yield_hold_ = 0.0;
    double cartesian_yield_v_ref_ = 0.0;
    int cartesian_plateau_cycles_ = 0;
    int cartesian_force_cycles_ = 0;
    double cartesian_latched_kp_ = 26.0;
    bool cartesian_kp_frozen_ = false;
    int cartesian_paper_cycles_ = 0;
    bool cartesian_paper_latched_ = false;
    bool cartesian_open_latched_ = false;
    double cartesian_open_t_latched_ = 0.0;
    double cartesian_duty_ = 0.75;
    double cartesian_stance_s_ = 0.45;
    double cartesian_step_m_ = 0.091;
    std::array<go2::Vec3, go2::kLegCount>
        kernel_touchdown_target_feet_base_{};
    bool have_kernel_touchdown_target_feet_ = false;
    double cycle_vx_sum_ = 0.0;
    int cycle_vx_count_ = 0;
    std::array<double, go2::kLegCount> kernel_touchdown_target_x_m_{};
    int preview_n_steps_ = 0;
    double preview_touchdown_x_m_ = 0.0;
    double preview_terminal_velocity_x_mps_ = 0.0;
    double preview_planned_acc_x_mps2_ = 0.0;
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
    std::array<go2::Vec3, go2::kLegCount> measured_support_anchor_world_feet_{};
    std::array<bool, go2::kLegCount> measured_support_anchor_valid_{};
    std::array<double, go2::kLegCount> support_anchor_start_time_s_{};

    go2_trot::CycleDiagnostics cycle_diagnostics_{};

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    unitree_go::msg::dds_::SportModeState_ high_state_{};
    unitree_go::msg::dds_::HeightMap_ environment_heightmap_{};
    unitree_go::msg::dds_::HeightMap_ lidar_heightmap_{};
    go2_terrain::TerrainMapEnvelope lidar_map_envelope_{};
    bool have_lidar_map_envelope_ = false;
    bool have_low_state_ = false;
    bool have_high_state_ = false;
    go2_terrain::TerrainPlanStore terrain_plan_store_{};
    std::shared_ptr<const go2_terrain::TerrainMotionPlan>
        terrain_tick_plan_;
    bool have_environment_heightmap_ = false;
    bool have_lidar_heightmap_ = false;

    go2_terrain::TerrainPlanner terrain_planner_{};
    std::shared_ptr<const go2_terrain::TerrainModel> terrain_model_;
    go2::Vec3 terrain_model_com_world_{};
    double terrain_model_com_state_stamp_s_ = 0.0;
    bool terrain_model_com_valid_ = false;
    std::atomic<std::uint64_t> terrain_map_epoch_{0};
    std::atomic<std::uint64_t> terrain_plan_epoch_{0};
    std::atomic<std::uint64_t> terrain_plan_id_{0};
    double terrain_last_update_s_ = -1.0e9;
    double terrain_last_map_age_s_ = std::numeric_limits<double>::infinity();
    double terrain_last_solver_us_ = 0.0;
    double terrain_last_plan_status_ = 0.0;
    double terrain_last_failure_ = 0.0;
    double terrain_min_edge_margin_m_ = 0.0;
    double terrain_min_uncertainty_edge_margin_m_ = 0.0;
    double terrain_min_slope_rad_ = 0.0;
    double terrain_max_roughness_m_ = 0.0;
    double terrain_min_reachability_margin_m_ = 0.0;
    std::uint64_t terrain_plan_published_count_ = 0;
    std::uint64_t terrain_plan_consumed_count_ = 0;
    std::uint64_t terrain_gait_target_override_count_ = 0;
    int terrain_execution_applied_mask_ = 0;
    std::uint64_t terrain_mpc_update_count_ = 0;
    std::uint64_t terrain_mpc_plan_consumed_count_ = 0;
    std::uint64_t terrain_target_prepare_attempts_ = 0;
    std::uint64_t terrain_target_prepared_ = 0;
    std::uint64_t terrain_target_prepare_rejections_ = 0;
    std::uint64_t terrain_surface_transition_completions_ = 0;
    int terrain_surface_transition_required_mask_ = 0;
    int terrain_surface_transition_committed_mask_ = 0;
    int terrain_surface_transition_last_required_mask_ = 0;
    int terrain_surface_transition_last_committed_mask_ = 0;
    std::array<std::size_t, go2::kLegCount> terrain_candidate_counts_{};
    std::array<std::size_t, go2::kLegCount> terrain_swing_candidate_counts_{};
    std::array<int, go2::kLegCount> terrain_touchdown_knots_{};
    double terrain_min_swing_clearance_m_ = 0.0;
    double terrain_min_support_margin_m_ = 0.0;
    double terrain_min_uncertainty_support_margin_m_ = 0.0;
    int terrain_support_failure_knot_ = -1;
    int terrain_support_failure_contact_mask_ = 0;
    double terrain_support_failure_margin_m_ = 0.0;
    std::uint64_t terrain_committed_touchdowns_ = 0;
    std::size_t terrain_known_cells_ = 0;
    std::size_t terrain_feasible_regions_ = 0;
    std::uint64_t terrain_planner_updates_ = 0;
    std::uint64_t terrain_planner_rejections_ = 0;
    std::uint64_t terrain_planner_deadline_misses_ = 0;
    bool terrain_latest_plan_valid_ = false;

    std::array<go2::Vec3, go2::kLegCount>
        terrain_execution_target_world_{};
    std::array<double, go2::kLegCount>
        terrain_execution_target_time_s_{};
    std::array<double, go2::kLegCount>
        terrain_execution_last_touchdown_time_s_{};
    std::array<double, go2::kLegCount> terrain_execution_target_lift_{};
    std::array<double, go2::kLegCount>
        terrain_execution_swing_start_phase_{};
    std::array<std::uint64_t, go2::kLegCount>
        terrain_execution_target_plan_id_{};
    std::array<std::uint64_t, go2::kLegCount>
        terrain_execution_target_plan_epoch_{};
    std::array<bool, go2::kLegCount>
        terrain_execution_target_valid_{};
    std::array<bool, go2::kLegCount>
        terrain_execution_in_flight_{};
    std::array<bool, go2::kLegCount>
        terrain_execution_measured_touchdown_{};
    std::array<bool, go2::kLegCount>
        terrain_execution_completion_recorded_{};
    std::array<go2_terrain::TerrainExecutionCommitment, go2::kLegCount>
        terrain_shadow_commitments_{};

    // Read-only wall-clock provenance for the exact state snapshot consumed
    // by each LowCmdWrite.  These fields are diagnostic only; they never
    // gate or alter controller decisions.
    std::chrono::steady_clock::time_point telemetry_start_time_ =
        std::chrono::steady_clock::now();
    std::uint64_t lowstate_arrival_count_ = 0;
    std::uint32_t lowstate_arrival_tick_ = 0;
    std::uint32_t lowstate_arrival_tick_delta_ = 0;
    bool lowstate_arrival_repeated_ = false;
    bool lowstate_arrival_jumped_ = false;
    bool lowstate_arrival_reordered_ = false;
    bool have_lowstate_arrival_tick_ = false;
    std::uint32_t previous_lowstate_arrival_tick_ = 0;
    double lowstate_arrival_wall_time_s_ = -1.0;
    std::uint64_t highstate_arrival_count_ = 0;
    double highstate_arrival_wall_time_s_ = -1.0;
    double highstate_stamp_s_ = -1.0;
    std::uint64_t lidar_arrival_count_ = 0;
    double lidar_arrival_wall_time_s_ = -1.0;
    double lidar_stamp_s_ = -1.0;

    double telemetry_controller_wall_time_s_ = -1.0;
    double telemetry_lowstate_consumed_wall_time_s_ = -1.0;
    std::uint64_t telemetry_lowstate_consumed_count_ = 0;
    std::uint32_t telemetry_lowstate_consumed_tick_ = 0;
    std::uint32_t telemetry_lowstate_consumed_tick_delta_ = 0;
    bool telemetry_lowstate_consumed_new_tick_ = false;
    bool telemetry_lowstate_consumed_repeated_ = false;
    bool telemetry_lowstate_consumed_jumped_ = false;
    bool telemetry_lowstate_consumed_reordered_ = false;
    bool have_lowstate_consumed_tick_ = false;
    std::uint32_t previous_lowstate_consumed_tick_ = 0;
    double telemetry_lowstate_arrival_wall_time_s_ = -1.0;
    std::uint32_t telemetry_lowstate_arrival_tick_ = 0;
    std::uint32_t telemetry_lowstate_arrival_tick_delta_ = 0;
    bool telemetry_lowstate_arrival_repeated_ = false;
    bool telemetry_lowstate_arrival_jumped_ = false;
    bool telemetry_lowstate_arrival_reordered_ = false;
    std::uint64_t telemetry_lowstate_arrival_count_ = 0;
    double telemetry_highstate_arrival_wall_time_s_ = -1.0;
    double telemetry_highstate_stamp_s_ = -1.0;
    std::uint64_t telemetry_highstate_arrival_count_ = 0;
    double telemetry_lidar_arrival_wall_time_s_ = -1.0;
    double telemetry_lidar_stamp_s_ = -1.0;
    std::uint64_t telemetry_lidar_arrival_count_ = 0;

    std::mutex terrain_diagnostics_mutex_;
    std::mutex terrain_control_mutex_;
    TerrainControlSnapshot terrain_control_snapshot_{};
    std::atomic<std::uint64_t> terrain_control_generation_{0};
    double terrain_last_control_snapshot_s_ = -1.0e9;
    std::mutex terrain_work_mutex_;
    std::condition_variable terrain_work_cv_;
    TerrainPlannerWork terrain_pending_work_{};
    bool terrain_work_pending_ = false;
    std::atomic<bool> terrain_worker_stop_{false};
    std::thread terrain_planner_thread_;

    std::mutex terrain_map_mutex_;
    std::mutex state_mutex_;
    std::ofstream csv_;
    std::atomic<bool> finished_{false};
    ChannelSubscriberPtr<unitree_go::msg::dds_::HeightMap_>
        environment_heightmap_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::HeightMap_>
        lidar_heightmap_subscriber_;
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_>
        lidar_map_envelope_subscriber_;
    std::atomic<bool> external_stop_requested_{false};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelPublisherPtr<unitree_go::msg::dds_::Error_> lockstep_ack_publisher_;
    bool lockstep_ack_enabled_ = false;
#ifdef GO2_TROT_TESTING
    bool suppress_lowcmd_publish_for_test_ = false;
#endif
    // Order-107: lockstep-local sequence epoch established at the first
    // lockstep state consumed after the controller's lifecycle barrier
    // (start-gait); the command sequence counts every LowCmd write 1:1 from
    // the adapter's first ack (uint32, wraps after 2^32 writes) so the sim's
    // exchange-local arrival ordinals match exactly.
    bool lockstep_epoch_valid_ = false;
    std::uint32_t lockstep_epoch_state_seq_ = 0;
    std::uint32_t lockstep_cmd_seq_ = 0;
    // Order-108 verification-only tick gate: once the writer handoff has
    // completed the lowcmd writer consumes exactly ONE new physics tick per
    // loop iteration (see lockstep_writer_gate.h); last_consumed_state_tick_
    // is the exact tick the most recent control update consumed (the same
    // state_seq the Order-107 ack carried). Flag-off: never engaged, so the
    // wall-clock writer loop is unchanged.
    lockstep_writer::WriterGate lockstep_writer_gate_;
    // Order-109: authoritative motion elapsed time after lockstep handoff.
    lockstep_motion::StateSynchronousClock lockstep_motion_clock_;
    std::uint32_t last_consumed_state_tick_ = 0;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
        highstate_subscriber_;
    std::thread low_cmd_write_thread_;
    std::atomic<bool> writer_stop_{false};
};
