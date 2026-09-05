#include "trot_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <iomanip>
#include <iostream>
#if defined(__linux__)
#include <sched.h>
#endif
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

namespace
{
void FillObstacleScan(
    const unitree_go::msg::dds_::HeightMap_ &map,
    double now_s,
    go2_control::MotionSensorSample &sensor)
{
    sensor.have_obstacle_scan = false;
    sensor.obstacle_scan_age_s = std::numeric_limits<double>::infinity();
    if (map.width() == 0 || map.height() == 0 ||
        map.resolution() <= 0.0f ||
        (map.frame_id() != "" && map.frame_id() != "base_link"))
        return;
    const std::size_t expected =
        static_cast<std::size_t>(map.width()) * map.height();
    if (map.data().size() < expected || !std::isfinite(now_s) ||
        !std::isfinite(map.stamp()))
        return;
    sensor.obstacle_scan_age_s = std::max(0.0, now_s - map.stamp());
    sensor.have_obstacle_scan = true;
    for (std::size_t iy = 0; iy < map.height(); ++iy)
    {
        for (std::size_t ix = 0; ix < map.width(); ++ix)
        {
            const float height = map.data()[iy * map.width() + ix];
            if (!std::isfinite(height) || height <= 0.0f)
                continue;
            const double x = map.origin()[0] +
                (static_cast<double>(ix) + 0.5) * map.resolution();
            const double y = map.origin()[1] +
                (static_cast<double>(iy) + 0.5) * map.resolution();
            if (!(x > 0.10))
                continue;
            const double distance = std::hypot(x, y);
            auto update = [&](double &distance_out, double &height_out) {
                if (distance < distance_out)
                    distance_out = distance;
                height_out = std::max(height_out, static_cast<double>(height));
            };
            if (std::abs(y) <= 0.20)
                update(sensor.obstacle_center_distance_m,
                       sensor.obstacle_center_height_m);
            if (y >= 0.16)
                update(sensor.obstacle_left_distance_m,
                       sensor.obstacle_left_height_m);
            if (y <= -0.16)
                update(sensor.obstacle_right_distance_m,
                       sensor.obstacle_right_height_m);
        }
    }
}
} // namespace

using namespace go2_trot;

void TrotExperiment::PinCurrentThreadToEnv(const char *env_name)
{
#if defined(__linux__)
    const char *value = std::getenv(env_name);
    if (value == nullptr || value[0] == 0)
        return;
    char *end = nullptr;
    const long cpu = std::strtol(value, &end, 10);
    if (end == value || *end != 0 || cpu < 0 || cpu >= CPU_SETSIZE)
        return;
    cpu_set_t cpu_set{};
    CPU_ZERO(&cpu_set);
    CPU_SET(static_cast<int>(cpu), &cpu_set);
    if (sched_setaffinity(0, sizeof(cpu_set), &cpu_set) != 0)
        std::cerr << "Unable to pin " << env_name << " to CPU "
                  << cpu << "\n";
#else
    (void)env_name;
#endif
}

void TrotExperiment::PublishTerrainControlSnapshot(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state)
{
    if (!params_.terrain_enabled ||
        !std::isfinite(running_time_) ||
        running_time_ - terrain_last_control_snapshot_s_ < 0.050)
        return;

    TerrainControlSnapshot snapshot;
    snapshot.valid = true;
    const double gait_period_s = kernel_period_s_ > 0.05
        ? kernel_period_s_ : params_.period_s;
    const double duty_factor = kernel_duty_factor_ > 0.1
        ? kernel_duty_factor_ : params_.duty_factor;
    snapshot.state_stamp_s =
        static_cast<double>(state_snapshot.tick()) * 1.0e-3;
    if (!std::isfinite(snapshot.state_stamp_s))
        return;
    snapshot.gait_phase = current_phase_;
    snapshot.gait_period_s = gait_period_s;
    snapshot.duty_factor = duty_factor;
    snapshot.commanded_vx_mps = kernel_nominal_velocity_x_mps_;
    snapshot.base_velocity_world = have_world_velocity_
        ? go2::Vec3{latest_world_velocity_[0], latest_world_velocity_[1],
                    latest_world_velocity_[2]}
        : go2::Vec3{};
    snapshot.base_roll_rad = state_snapshot.imu_state().rpy()[0];
    snapshot.base_pitch_rad = state_snapshot.imu_state().rpy()[1];
    snapshot.base_yaw_rad = state_snapshot.imu_state().rpy()[2];
    for (std::size_t axis = 0; axis < snapshot.base_quaternion.size(); ++axis)
        snapshot.base_quaternion[axis] =
            state_snapshot.imu_state().quaternion()[axis];
    snapshot.have_base_position_world = have_high_state;
    if (have_high_state)
    {
        snapshot.imu_position_world = {
            high_state_snapshot.position()[0],
            high_state_snapshot.position()[1],
            high_state_snapshot.position()[2]};
    }

    for (std::size_t i = 0; i < kMotorCount; ++i)
        snapshot.joint_positions[i] = state_snapshot.motor_state()[i].q();
    snapshot.have_commanded_body_feet = have_commanded_body_feet_;
    if (snapshot.have_commanded_body_feet)
        snapshot.nominal_feet_base = commanded_body_feet_;
    snapshot.touchdown_target_feet_base = kernel_touchdown_target_feet_base_;
    snapshot.touchdown_target_feet_valid = have_kernel_touchdown_target_feet_;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        snapshot.measured_contact[leg] =
            state_snapshot.foot_force()[leg] >= kContactForceThreshold;
    snapshot.measured_valid = true;

    {
        std::lock_guard<std::mutex> lock(terrain_control_mutex_);
        terrain_control_snapshot_ = snapshot;
    }
    terrain_control_generation_.fetch_add(1, std::memory_order_release);
    terrain_last_control_snapshot_s_ = running_time_;
    terrain_work_cv_.notify_one();
}

void TrotExperiment::UpdateTerrainRuntime()
{
    if (!params_.terrain_enabled)
        return;

    TerrainControlSnapshot control;
    {
        std::lock_guard<std::mutex> lock(terrain_control_mutex_);
        control = terrain_control_snapshot_;
    }
    if (!control.valid || !std::isfinite(control.state_stamp_s) ||
        control.state_stamp_s - terrain_last_update_s_ < 0.050)
        return;

    TerrainPlannerWork work;
    work.map_epoch = ++terrain_map_epoch_;
    work.plan_id = ++terrain_plan_id_;
    auto &input = work.input;
    input.state_stamp_s = control.state_stamp_s;
    input.base_yaw_rad = control.base_yaw_rad;
    const go2::Vec3 imu_offset_world = RotateByQuaternion(
        control.base_quaternion, {-0.02557, 0.0, 0.04232});
    if (control.have_base_position_world)
    {
        input.base_position_world = {
            control.imu_position_world.x - imu_offset_world.x,
            control.imu_position_world.y - imu_offset_world.y,
            control.imu_position_world.z - imu_offset_world.z};
    }
    input.base_velocity_world = control.base_velocity_world;
    input.base_roll_rad = control.base_roll_rad;
    input.base_pitch_rad = control.base_pitch_rad;
    input.base_height_m = input.base_position_world.z;
    input.gait_phase = control.gait_phase;
    input.gait_period_s = control.gait_period_s;
    input.duty_factor = control.duty_factor;
    input.commanded_vx_mps = control.commanded_vx_mps;
    input.current_feet_base = go2::AllFootPositions(control.joint_positions);
    input.nominal_feet_base = control.have_commanded_body_feet
        ? control.nominal_feet_base
        : go2::AllFootPositions(task_.stand_up_joint_pos_);
    input.touchdown_target_feet_base = control.touchdown_target_feet_base;
    input.touchdown_target_feet_valid = control.touchdown_target_feet_valid;
    input.contact_schedule.measured_contact = control.measured_contact;
    input.contact_schedule.measured_valid = control.measured_valid;
    go2_control::FillTrotContactSchedulePhase(
        input.gait_phase, input.gait_period_s, input.duty_factor,
        static_cast<int>(terrain_planner_.config().horizon_knots),
        terrain_planner_.config().knot_dt_s,
        input.contact_schedule.planned_contact,
        params_.gait_pattern);
    // The gait helper fills contact bits only; validity is an explicit
    // planned-vs-measured interface contract.
    input.contact_schedule.planned_valid = true;

    {
        std::lock_guard<std::mutex> lock(terrain_map_mutex_);
        work.have_map = have_lidar_heightmap_;
        if (work.have_map)
            work.map = lidar_heightmap_;
    }
    terrain_last_update_s_ = control.state_stamp_s;

    {
        std::lock_guard<std::mutex> lock(terrain_work_mutex_);
        terrain_pending_work_ = std::move(work);
        terrain_work_pending_ = true;
    }
}

void TrotExperiment::TerrainPlannerWorker()
{
#if defined(__linux__)
    // Sensor-only terrain is an observer. Keep its best-effort work from
    // preempting the accepted 500 Hz Phase 1 command writer when the runner
    // pins the controller process to one CPU.
    if (params_.terrain_sensor_only)
    {
        sched_param scheduler_params{};
        (void)sched_setscheduler(0, SCHED_IDLE, &scheduler_params);
    }
#endif
    PinCurrentThreadToEnv("TROT_TERRAIN_CPU");
    // Diagnostic-only: exercise worker creation, scheduling setup, and
    // shutdown without consuming terrain work. Production never parks it.
    if (Full2EnvDouble("TROT_TERRAIN_WORKER_PARK", 0.0) > 0.5)
    {
        std::unique_lock<std::mutex> lock(terrain_work_mutex_);
        terrain_work_cv_.wait(lock, [this]() {
            return terrain_worker_stop_.load();
        });
        return;
    }
    std::uint64_t consumed_generation = 0;
    for (;;)
    {
        {
            std::unique_lock<std::mutex> lock(terrain_work_mutex_);
            terrain_work_cv_.wait(lock, [this, &consumed_generation]() {
                return terrain_worker_stop_.load() ||
                    terrain_control_generation_.load(
                        std::memory_order_acquire) > consumed_generation;
            });
            if (terrain_worker_stop_.load())
                return;
            consumed_generation = terrain_control_generation_.load(
                std::memory_order_acquire);
        }

        UpdateTerrainRuntime();

        TerrainPlannerWork work;
        {
            std::lock_guard<std::mutex> lock(terrain_work_mutex_);
            if (!terrain_work_pending_)
                continue;
            work = std::move(terrain_pending_work_);
            terrain_work_pending_ = false;
        }

        std::shared_ptr<const go2_terrain::TerrainModel> model;
        if (work.have_map)
        {
            const auto built = go2_terrain::BuildTerrainModel(
                &work.map, work.input.state_stamp_s, work.map_epoch,
                go2_terrain::TerrainSource::kLidar);
            if (built.ok())
                model = std::make_shared<const go2_terrain::TerrainModel>(
                    built.model);
        }
        work.input.terrain = model.get();
        const auto result = terrain_planner_.Build(work.input, work.plan_id);
        if (result.publishable)
            terrain_plan_store_.Publish(result.plan);

        std::size_t known_cells = 0;
        std::size_t feasible_regions = 0;
        if (model)
        {
            for (const auto &cell : model->cells)
                if (cell.known)
                    ++known_cells;
            for (const auto &regions : result.regions)
                feasible_regions += regions.size();
        }
        {
            std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
            terrain_plan_epoch_.store(result.plan.plan_epoch);
            terrain_model_ = model;
            terrain_last_map_age_s_ = model
                ? model->age_s : std::numeric_limits<double>::infinity();
            terrain_known_cells_ = known_cells;
            terrain_feasible_regions_ = feasible_regions;
            terrain_last_solver_us_ = result.plan.solver.elapsed_us;
            terrain_last_plan_status_ = static_cast<double>(
                static_cast<int>(result.plan.status));
            terrain_last_failure_ = static_cast<double>(
                static_cast<int>(result.plan.failure));
            terrain_min_edge_margin_m_ = result.plan.min_edge_margin_m;
            terrain_min_uncertainty_edge_margin_m_ =
                result.plan.min_uncertainty_inflated_edge_margin_m;
            terrain_min_slope_rad_ = result.plan.min_slope_rad;
            terrain_max_roughness_m_ = result.plan.max_roughness_m;
            terrain_min_reachability_margin_m_ =
                result.plan.min_reachability_margin_m;
            terrain_min_swing_clearance_m_ =
                result.plan.min_swing_clearance_m;
            terrain_min_support_margin_m_ = result.plan.min_support_margin_m;
            terrain_min_uncertainty_support_margin_m_ =
                result.plan.min_uncertainty_inflated_support_margin_m;
            terrain_plan_published_count_ += result.publishable ? 1 : 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                terrain_candidate_counts_[leg] = result.candidate_counts[leg];
                terrain_swing_candidate_counts_[leg] = 0;
                terrain_touchdown_knots_[leg] = -1;
                bool previous = work.input.contact_schedule.measured_contact[leg];
                for (std::size_t k = 0; k < terrain_planner_.config().horizon_knots; ++k)
                {
                    const bool planned =
                        work.input.contact_schedule.planned_contact[k][leg];
                    if (planned && !previous && terrain_touchdown_knots_[leg] < 0)
                        terrain_touchdown_knots_[leg] = static_cast<int>(k);
                    previous = planned;
                }
                for (const auto &region : result.regions[leg])
                    if (region.valid && region.swing_clearance_m >=
                        terrain_planner_.config().feasibility.min_swing_clearance_m)
                        ++terrain_swing_candidate_counts_[leg];
            }
            terrain_committed_touchdowns_ = result.plan.committed_touchdowns;
            ++terrain_planner_updates_;
            if (result.plan.solver.deadline_miss)
                ++terrain_planner_deadline_misses_;
            if (!result.publishable)
                ++terrain_planner_rejections_;
            terrain_latest_plan_valid_ = result.plan.valid();
        }

    }
}

static_assert(TrotTask::kStandUpDuration == kStandUpDuration);
static_assert(TrotTask::kStandSettleDuration == kStandSettleDuration);
static_assert(TrotTask::kStandDownDuration == kStandDownDuration);
static_assert(TrotTask::kStopTransitionDuration == kStopTransitionDuration);
static_assert(TrotTask::kFinalHoldDuration == kFinalHoldDuration);
static_assert(TrotTask::kGaitBlendDuration == kGaitBlendDuration);

// CONTROL LOOP — 500Hz LowCmdWrite state machine (see docs/CODE_GUIDE.md)

// --- TrotExperiment::LowCmdWrite ---
bool TrotExperiment::LowCmdWrite(
    std::uint32_t expected_state_tick,
    bool enforce_state_tick)
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
        return false;

    unitree_go::msg::dds_::LowState_ state_snapshot{};
    unitree_go::msg::dds_::SportModeState_ high_state_snapshot{};
    bool have_state = false;
    bool have_high_state = false;
    if (!SnapshotState(state_snapshot, high_state_snapshot,
                       have_state, have_high_state))
        return false;
    if (enforce_state_tick &&
        state_snapshot.tick() != expected_state_tick)
    {
        lockstep_writer_gate_.FailSnapshotMismatch();
        return false;
    }

    bool motion_clock_paused = false;
    const double motion_dt = MotionClockStep(state_snapshot, motion_clock_paused);

    terrain_tick_plan_.reset();
    if (params_.terrain_actuation && params_.terrain_enabled)
    {
        const double terrain_now_s =
            static_cast<double>(state_snapshot.tick()) * 1.0e-3;
        terrain_tick_plan_ = terrain_plan_store_.LoadUsable(terrain_now_s);
        if (terrain_tick_plan_)
        {
            std::lock_guard<std::mutex> lock(terrain_diagnostics_mutex_);
            ++terrain_plan_consumed_count_;
        }
    }

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
    else if (PhaseStartGait(state_snapshot, joint_targets))
    {
        // SECTION: start-gait
    }

    if (PhaseRunGait(state_snapshot, high_state_snapshot,
                     have_high_state, joint_targets))
    {
        // SECTION: gait-run
    }

    if (external_stop_requested_.load())
    {
        if (emergency_stop_latched_)
            task_.sequence_finished_ = true;
        else if (HighSpeedStopBrakeEnabled() && task_.InLocomotion())
        {
            if (!high_speed_stop_brake_active_)
            {
                high_speed_stop_brake_active_ = true;
                high_speed_stop_brake_duration_s_ = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_BRAKE_DURATION_S", 2.00),
                    0.25, 2.00);
                high_speed_stop_brake_start_time_s_ = running_time_;
                high_speed_stop_brake_base_speed_mps_ =
                    wbc_speed_cmd_mps_ > 0.0
                        ? std::abs(wbc_speed_cmd_mps_)
                        : std::abs(kernel_nominal_velocity_x_mps_);
                if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
                    high_speed_stop_brake_base_speed_mps_ =
                        std::abs(params_.direction_sign * params_.step_length_m /
                                 std::max(0.08, params_.period_s));
                high_speed_stop_brake_base_period_s_ =
                    kernel_period_s_ > 0.05
                        ? kernel_period_s_ : params_.period_s;
                high_speed_stop_brake_base_duty_ =
                    kernel_duty_factor_ > 0.25
                        ? kernel_duty_factor_ : params_.duty_factor;
                std::cout << "High-speed stop: braking in locomotion plant"
                          << " v0=" << high_speed_stop_brake_base_speed_mps_
                          << " period=" << high_speed_stop_brake_base_period_s_
                          << " duty=" << high_speed_stop_brake_base_duty_
                          << "\n";
            }
        }
        else
            task_.stop_requested_ = true;
    }
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
    PublishTerrainControlSnapshot(
        state_snapshot, high_state_snapshot, have_high_state);

    // SECTION: publish-lowcmd
    PublishLowCmdWithCrc();
    // Order-107: after the LowCmd is published in this cycle, increment the
    // local command sequence and emit the verification-only
    // ack{state_seq, command_seq} for the exact state snapshot consumed by
    // this control period (no-op when the adapter is off).
    PublishLockstepAck(state_snapshot.tick());
    // SECTION: log-sample
        LogSample(state_snapshot, have_state, high_state_snapshot, have_high_state);
    return true;
}

void TrotExperiment::PublishLowCmdWithCrc()
{
    low_cmd_.crc() = crc32_core(
        (uint32_t *)&low_cmd_,
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
#ifdef GO2_TROT_TESTING
    if (suppress_lowcmd_publish_for_test_)
        return;
#endif
    lowcmd_publisher_->Write(low_cmd_);
}

#ifdef GO2_TROT_TESTING
void TrotExperiment::TestPrepareMotionClock(std::uint32_t handoff_tick)
{
    InitLowCmd();
    suppress_lowcmd_publish_for_test_ = true;
    lockstep_ack_enabled_ = true;
    last_consumed_state_tick_ = handoff_tick;
    EngageLockstepWriterIfNeeded();
    // Keep the production gait/timer consumers active for the call-chain
    // probe; these are existing controller fields, not test-time clocks.
    task_.gait_started_ = true;
    task_.motion_stage_ = 2;
    stop_brake_start_time_s_ = running_time_;
    high_speed_stop_brake_start_time_s_ = running_time_;
    high_speed_stop_hold_start_time_s_ = running_time_;
}

bool TrotExperiment::TestRunWallClockTick(
    const unitree_go::msg::dds_::LowState_ &state)
{
    suppress_lowcmd_publish_for_test_ = true;
    LowStateMessageHandler(&state);
    if (!lockstep_writer_gate_.Engaged())
        return LowCmdWrite();
    else
        return false;
}

bool TrotExperiment::TestRunLockstepTick(
    const unitree_go::msg::dds_::LowState_ &state)
{
    LowStateMessageHandler(&state);
    if (!lockstep_writer_gate_.HasPendingTick())
        return false; // duplicate publication: writer does not run
    std::uint32_t pending_tick = 0;
    if (lockstep_writer_gate_.WaitForTick(
            []() { return false; }, &pending_tick) !=
        lockstep_writer::WaitResult::kTick)
        return false;
    if (!LowCmdWrite(pending_tick, true))
        return false;
    lockstep_writer_gate_.RecordConsumed(pending_tick);
    return true;
}

TrotExperiment::TestMotionClockSample
TrotExperiment::TestLastMotionClockSample() const
{
    TestMotionClockSample sample;
    sample.motion_dt_s = last_motion_dt_s_;
    sample.cmd_time_s = running_time_;
    sample.gait_time_s = running_time_ - task_.gait_start_time_s_;
    // These are the production elapsed-time consumers used by gait ramp,
    // health governor, and timed stop paths, observed from their anchors.
    sample.ramp_time_s = running_time_ - task_.gait_start_time_s_;
    sample.governor_time_s = running_time_ - high_speed_stop_brake_start_time_s_;
    sample.stop_time_s = running_time_ - stop_brake_start_time_s_;
    return sample;
}
#endif

// Order-107 verification-only ack: ack{state_seq, command_seq} published
// only after the LowCmd write of the same control cycle, only when the
// adapter is enabled. `state_seq` is the tick side-channel of the LowState
// snapshot the cycle consumed (Error_.source(), uint32_t; wraps at 2^32 ms
// ~ 49.7 days at 1 kHz). The lockstep-local sequence epoch is established at
// the first lockstep state consumed after the controller's lifecycle
// barrier (start-gait); every subsequent LowCmd write increments the local
// command_seq (Error_.state(), uint32_t) and the ack carries the exact pair,
// so the simulator can bind the ack to the acked cycle's own LowCmd arrival.
// No control math or message payload changes.
void TrotExperiment::PublishLockstepAck(std::uint32_t state_seq)
{
    if (!lockstep_ack_enabled_ || !lockstep_ack_publisher_)
        return;
    // Order-108: record the exact tick this control update consumed so the
    // writer gate can detect the next strictly-new tick (and so Engage()
    // clears old events without missing the first lockstep tick).
    last_consumed_state_tick_ = state_seq;
    if (!lockstep_epoch_valid_ && task_.gait_started_)
    {
        // First lockstep state consumed after the controller's lifecycle
        // barrier anchors the local epoch; the command sequence keeps
        // counting 1:1 with every LowCmd write from the adapter's first ack.
        lockstep_epoch_state_seq_ = state_seq;
        lockstep_epoch_valid_ = true;
    }
    ++lockstep_cmd_seq_;
    unitree_go::msg::dds_::Error_ ack;
    ack.source(state_seq);
    ack.state(lockstep_cmd_seq_);
    lockstep_ack_publisher_->Write(ack);
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
    if (high_speed_stop_hold_active_ && task_.stop_requested_)
    {
        const double hold_elapsed =
            running_time_ - high_speed_stop_hold_start_time_s_;
        const double hold_u = Smoothstep(std::clamp(
            hold_elapsed / kHighSpeedStopHoldDurationS, 0.0, 1.0));
        if (have_high_speed_stop_hold_targets_)
        {
            // Keep the transition inside the four-contact WBC plant: first
            // preserve the touchdown posture, then continuously settle the
            // joint target toward the validated stand configuration.  A
            // direct jump to either target would recreate a hidden mode
            // switch at the exact moment the brake has just finished.
            for (std::size_t i = 0; i < kMotorCount; ++i)
                joint_targets[i] =
                    (1.0 - hold_u) * high_speed_stop_hold_targets_[i] +
                    hold_u * task_.stand_up_joint_pos_[i];
        }
        if (hold_elapsed < kHighSpeedStopHoldDurationS)
            return true;
        high_speed_stop_hold_active_ = false;
        std::cout << "High-speed stop: WBC four-contact hold complete;"
                  << " finished in WBC stance\n";
        // A sprint acceptance is already in the same full-WBC plant after
        // braking.  Handing the body to the old joint interpolation here
        // would create a second, unrelated plant switch and can reintroduce
        // the very fall the controlled stop just prevented.  Finish while
        // the four-contact WBC hold is valid; a separate task may explicitly
        // request the low-speed stand-down transition.
        task_.motion_stage_ = 3;
        task_.sequence_finished_ = true;
        task_.task_completion_requested_ = false;
        return true;
    }
    const bool active = task_.PhaseStopToStand(running_time_, joint_targets);
    if (task_.lie_down_started_ &&
        task_.lie_down_start_time_s_ == running_time_)
    {
        active_cycle_index_ = -1;
        completed_cycles_ = 0;
    }
    return active;
}

bool TrotExperiment::HighSpeedStopBrakeEnabled() const
{
    if (!params_.wbc_full ||
        Full2EnvDouble("TROT_HS_DISABLE", 0.0) > 0.5)
        return false;
    const double requested_speed = std::abs(
        2.0 * params_.duty_factor * params_.direction_sign *
        params_.step_length_m / std::max(1.0e-3, params_.period_s));
    const bool high_speed_curriculum =
        params_.gait_pattern != go2_control::GaitPattern::kDiagonalTrot ||
        requested_speed > 1.25;
    if (!high_speed_curriculum)
        return false;
    const double override = Full2EnvDouble("TROT_HS_STOP_BRAKE", -1.0);
    if (override >= 0.0)
        return override > 0.5;
    return requested_speed >= 2.0;
}

bool TrotExperiment::PhaseStartGait(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    std::array<double, go2_trot::kMotorCount> &joint_targets)
{
    if (params_.wbc_full && !task_.gait_started_ && !task_.stop_requested_)
    {
        const double extra_settle_s = std::clamp(
            Full2EnvDouble("TROT_HS_EXTRA_SETTLE_S", 0.8), 0.0, 3.0);
        const double nominal_gait_start_s =
            TrotTask::kStandUpDuration + TrotTask::kStandSettleDuration;
        const double requested_speed = std::abs(
            2.0 * params_.duty_factor * params_.direction_sign *
            params_.step_length_m /
            std::max(1.0e-3, params_.period_s));
        const bool high_speed_curriculum =
            !params_.cartesian_world &&
            (params_.gait_pattern != go2_control::GaitPattern::kDiagonalTrot ||
             requested_speed > 1.25);
        const double preflight_s = std::clamp(
            Full2EnvDouble("TROT_HS_PREFLIGHT_STABLE_S", 0.30),
            0.0, 2.0);
        const double preflight_angle = std::clamp(
            Full2EnvDouble("TROT_HS_PREFLIGHT_ANGLE_RAD", 0.08),
            0.03, 0.25);
        const int min_contacts = static_cast<int>(std::clamp(
            Full2EnvDouble("TROT_HS_PREFLIGHT_MIN_CONTACTS", 3.0),
            2.0, 4.0));
        int contact_count = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (state_snapshot.foot_force()[leg] >= kContactForceThreshold)
                ++contact_count;
        }
        const double roll = std::abs(static_cast<double>(
            state_snapshot.imu_state().rpy()[0]));
        const double pitch = std::abs(static_cast<double>(
            state_snapshot.imu_state().rpy()[1]));
        const bool preflight_stable =
            !high_speed_curriculum ||
            (std::max(roll, pitch) <= preflight_angle &&
             contact_count >= min_contacts);
        if (preflight_stable)
            ++high_speed_preflight_stable_cycles_;
        else
            high_speed_preflight_stable_cycles_ = 0;
        const int required_preflight_cycles = static_cast<int>(std::ceil(
            preflight_s / 0.002));
        if (running_time_ < nominal_gait_start_s + extra_settle_s ||
            (high_speed_curriculum &&
             high_speed_preflight_stable_cycles_ < required_preflight_cycles))
        {
            task_.motion_stage_ = 1;
            joint_targets = task_.stand_up_joint_pos_;
            return true;
        }
    }
    if (!task_.BeginGait(running_time_))
        return false;
    velocity_command_initialized_ = false;
    velocity_command_state_ = {};
    velocity_stance_hold_gate_.Reset();
    runtime_velocity_stance_hold_active_ = false;
    runtime_gait_regime_ = params_.runtime_velocity_command
        ? "continuous-trot"
        : "inactive";
    if (motion_event_response_enabled_)
    {
        motion_event_layer_.Reset();
        motion_event_detector_.Reset();
        motion_event_state_ = {};
        auto_motion_event_ = {};
        auto_emergency_stop_event_ = {};
        runtime_event_schedule_ = params_.event_schedule;
        auto_emergency_stop_scheduled_ = false;
        motion_reference_ = {};
        last_motion_event_type_ = go2_control::MotionEventType::kNone;
        emergency_stop_latched_ = false;
        emergency_stop_finish_time_s_ = 0.0;
    }
    support_anchor_valid_.fill(false);
    high_speed_stop_speed_candidate_start_s_ = -1.0;
    cartesian_state_ = {};
    have_commanded_body_feet_ = false;
    have_commanded_body_feet_velocity_ = false;
    have_commanded_world_feet_ = false;
    previous_leg_swing_.fill(false);
    touchdown_recorded_.fill(false);
    touchdown_waiting_contact_.fill(false);
    previous_support_foot_valid_.fill(false);
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
void TrotExperiment::UpdateMotionEventResponse(
    double gait_elapsed_s, double motion_dt_s,
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state)
{
    if (!motion_event_response_enabled_)
        return;

    go2_control::MotionReference nominal;
    nominal.vx_mps = params_.direction_sign *
        (params_.step_length_m / params_.period_s);
    if (params_.cartesian_world && wbc_speed_cmd_mps_ > 0.0)
        nominal.vx_mps = params_.direction_sign * wbc_speed_cmd_mps_;
    nominal.yaw_rate_radps = task_.goal_enabled_
        ? task_.commanded_turn_rate_radps_
        : params_.turn_rate_radps;
    nominal.step_scale = 1.0;
    nominal.duty_factor = params_.duty_factor;
    nominal.foot_lift_m = params_.foot_lift_m;

    go2_control::MotionSensorSample sensor;
    sensor.velocity_x_mps = latest_filtered_body_velocity_[0];
    sensor.velocity_y_mps = latest_filtered_body_velocity_[1];
    sensor.have_velocity = have_filtered_body_velocity_;
    sensor.raw_velocity_x_mps = latest_raw_body_velocity_[0];
    sensor.raw_velocity_y_mps = latest_raw_body_velocity_[1];
    sensor.have_raw_velocity = have_raw_body_velocity_;
    sensor.angular_velocity_z_radps = static_cast<double>(
        state_snapshot.imu_state().gyroscope()[2]);
    sensor.have_angular_velocity_z = true;
    sensor.accel_x_mps2 = static_cast<double>(
        state_snapshot.imu_state().accelerometer()[0]);
    sensor.accel_y_mps2 = static_cast<double>(
        state_snapshot.imu_state().accelerometer()[1]);
    sensor.accel_z_mps2 = static_cast<double>(
        state_snapshot.imu_state().accelerometer()[2]);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (state_snapshot.foot_force()[leg] >= kContactForceThreshold)
            ++sensor.contact_count;
    }
    // Estimate support-foot motion in the world frame.  A scheduled stance
    // foot should remain nearly fixed on the ground; sustained motion of at
    // least two support feet is a direct kinematic slip/low-friction signal.
    if (have_high_state && motion_dt_s > 1.0e-6 &&
        std::isfinite(motion_dt_s))
    {
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        const auto world_feet = ComputeWorldFeet(state_snapshot, pose);
        const double period = std::max(params_.period_s, 1.0e-3);
        const double phase = std::fmod(
            std::max(0.0, gait_elapsed_s) / period, 1.0);
        const double duty = std::clamp(
            params_.duty_factor, 0.45, 0.90);
        std::array<double, go2::kLegCount> support_speeds{};
        int support_speed_count = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const double leg_phase = go2_control::GaitLegPhase(
                leg, phase, params_.gait_pattern);
            const bool scheduled_stance = leg_phase < duty;
            if (!scheduled_stance)
            {
                previous_support_foot_valid_[leg] = false;
                continue;
            }
            if (previous_support_foot_valid_[leg] &&
                state_snapshot.foot_force()[leg] >= kContactForceThreshold)
            {
                const go2::Vec3 delta = {
                    world_feet[leg].x - previous_support_foot_world_[leg].x,
                    world_feet[leg].y - previous_support_foot_world_[leg].y,
                    world_feet[leg].z - previous_support_foot_world_[leg].z};
                const double speed =
                    std::hypot(delta.x, delta.y) / motion_dt_s;
                if (std::isfinite(speed))
                    support_speeds[support_speed_count++] = speed;
            }
            previous_support_foot_world_[leg] = world_feet[leg];
            previous_support_foot_valid_[leg] = true;
        }
        if (support_speed_count >= 2)
        {
            std::sort(
                support_speeds.begin(),
                support_speeds.begin() + support_speed_count);
            sensor.have_support_foot_kinematics = true;
            sensor.support_foot_count = support_speed_count;
            sensor.support_foot_speed_mps =
                support_speed_count % 2 == 0
                    ? 0.5 * (support_speeds[support_speed_count / 2 - 1] +
                             support_speeds[support_speed_count / 2])
                    : support_speeds[support_speed_count / 2];
        }
    }
    if (params_.auto_environment)
    {
        unitree_go::msg::dds_::HeightMap_ height_map;
        bool have_height_map = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            height_map = environment_heightmap_;
            have_height_map = have_environment_heightmap_;
        }
        if (have_height_map)
        {
            const double now_s =
                static_cast<double>(state_snapshot.tick()) * 1.0e-3;
            FillObstacleScan(height_map, now_s, sensor);
        }
    }
    latest_motion_sensor_ = sensor;
    auto_motion_event_ = {};
    if (params_.reactive_events || params_.auto_environment ||
        params_.impact_to_emergency_stop_delay_s >= 0.0)
    {
        auto_motion_event_ = motion_event_detector_.Observe(
            gait_elapsed_s, motion_dt_s, sensor, nominal);
        sensor.support_low_friction_evidence =
            motion_event_detector_.SupportLowFrictionEvidence();
        sensor.low_friction_accumulation =
            motion_event_detector_.LowFrictionAccumulation();
        latest_motion_sensor_ = sensor;
    }
    if (auto_motion_event_.type == go2_control::MotionEventType::kImpact &&
        params_.impact_to_emergency_stop_delay_s >= 0.0 &&
        !auto_emergency_stop_scheduled_)
    {
        auto_emergency_stop_event_ = {
            go2_control::MotionEventType::kEmergencyStop,
            auto_motion_event_.start_time_s +
                params_.impact_to_emergency_stop_delay_s,
            0.80,
            0.0};
        runtime_event_schedule_.push_back(auto_emergency_stop_event_);
        auto_emergency_stop_scheduled_ = true;
        std::cout << "Auto emergency stop scheduled after impact at gait t="
                  << auto_emergency_stop_event_.start_time_s << " s\n";
    }
    const go2_control::MotionEvent *sensor_event =
        (params_.reactive_events || params_.auto_environment ||
         params_.impact_to_emergency_stop_delay_s >= 0.0) &&
                auto_motion_event_.IsActive(gait_elapsed_s)
            ? &auto_motion_event_
            : nullptr;

    motion_event_state_ = motion_event_layer_.Update(
        gait_elapsed_s, motion_dt_s, nominal, runtime_event_schedule_,
        sensor_event, &sensor);
    motion_reference_ = motion_event_state_.reference;

    // An emergency stop is terminal for this bounded experiment.  Once the
    // event is seen, keep the same WBC/MPC plant in four-foot stance instead
    // of letting the scheduled event expire and restarting the trot.
    if (motion_event_state_.active_event ==
            go2_control::MotionEventType::kEmergencyStop &&
        !emergency_stop_latched_)
    {
        emergency_stop_latched_ = true;
        motion_event_layer_.SetEmergencyStopLatched(true);
        emergency_stop_latch_gait_time_s_ = gait_elapsed_s;
        emergency_stop_finish_time_s_ =
            motion_event_state_.active_event_end_time_s +
            kEmergencyStopPostHoldDurationS;
        std::cout << "Emergency stop latched; holding WBC stance until t="
                  << emergency_stop_finish_time_s_ << " s\n";
    }
    if (emergency_stop_latched_)
    {
        // Keep the safety target and stance hold latched, but do not overwrite
        // the slew-limited velocity reference. The common transition layer
        // continues braking toward zero after the scheduled event expires.
        motion_event_state_.target.vx_mps = 0.0;
        motion_event_state_.target.vy_mps = 0.0;
        motion_event_state_.target.yaw_rate_radps = 0.0;
        motion_event_state_.target.hold_stance = true;
        // Keep the running contact phase alive while the velocity reference
        // brakes.  Enter stance blending only after this support exchange has
        // had time to settle.
        motion_reference_.hold_stance = EmergencyStopStanceBlendActive();
        motion_reference_.step_scale =
            std::min(motion_reference_.step_scale, 0.45);
        motion_reference_.duty_factor =
            std::max(motion_reference_.duty_factor, 0.82);
        motion_reference_.foot_lift_m =
            std::max(motion_reference_.foot_lift_m, 0.024);
        if (!task_.sequence_finished_ &&
            gait_elapsed_s >= emergency_stop_finish_time_s_)
        {
            task_.sequence_finished_ = true;
            std::cout << "Emergency stop hold complete; ending in WBC stance\n";
        }
    }

    const double velocity_step =
        std::abs(motion_reference_.vx_mps) * params_.period_s;
    const double scaled_nominal_step =
        std::abs(params_.step_length_m) * motion_reference_.step_scale;
    const double event_step = std::min(velocity_step, scaled_nominal_step);
    locomotion_kernel_->SetStanceHold(motion_reference_.hold_stance, gait_elapsed_s);
    locomotion_kernel_->SetGaitStepLength(event_step);
    locomotion_kernel_->SetGaitDuty(motion_reference_.duty_factor);
    locomotion_kernel_->SetGaitFootLift(motion_reference_.foot_lift_m);

    if (motion_event_state_.active_event != last_motion_event_type_)
    {
        std::cout << "Motion event "
                  << go2_control::MotionEventName(
                         motion_event_state_.active_event)
                  << " priority=" << motion_event_state_.active_priority
                  << " ref_vx=" << motion_reference_.vx_mps
                  << " ref_yaw=" << motion_reference_.yaw_rate_radps
                  << "\n";
        last_motion_event_type_ = motion_event_state_.active_event;
    }
}
bool TrotExperiment::SnapshotState(
    unitree_go::msg::dds_::LowState_ &state_snapshot,
    unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool &have_state,
    bool &have_high_state)
{
    have_state = false;
    have_high_state = false;
    const double consumed_wall_time_s = WallClockTelemetryTimeS();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        have_state = have_low_state_;
        have_high_state = have_high_state_;
        if (have_state)
        {
            state_snapshot = low_state_;
            telemetry_controller_wall_time_s_ = consumed_wall_time_s;
            telemetry_lowstate_consumed_wall_time_s_ = consumed_wall_time_s;
            telemetry_lowstate_consumed_tick_ = state_snapshot.tick();
            telemetry_lowstate_arrival_wall_time_s_ =
                lowstate_arrival_wall_time_s_;
            telemetry_lowstate_arrival_tick_ = lowstate_arrival_tick_;
            telemetry_lowstate_arrival_tick_delta_ =
                lowstate_arrival_tick_delta_;
            telemetry_lowstate_arrival_repeated_ =
                lowstate_arrival_repeated_;
            telemetry_lowstate_arrival_jumped_ =
                lowstate_arrival_jumped_;
            telemetry_lowstate_arrival_reordered_ =
                lowstate_arrival_reordered_;
            telemetry_lowstate_arrival_count_ = lowstate_arrival_count_;

            telemetry_lowstate_consumed_tick_delta_ = 0;
            telemetry_lowstate_consumed_new_tick_ = true;
            telemetry_lowstate_consumed_repeated_ = false;
            telemetry_lowstate_consumed_jumped_ = false;
            telemetry_lowstate_consumed_reordered_ = false;
            if (have_lowstate_consumed_tick_)
            {
                const std::uint32_t delta =
                    telemetry_lowstate_consumed_tick_ -
                    previous_lowstate_consumed_tick_;
                telemetry_lowstate_consumed_tick_delta_ = delta;
                telemetry_lowstate_consumed_new_tick_ = delta != 0;
                telemetry_lowstate_consumed_repeated_ = delta == 0;
                const std::uint32_t expected_tick_delta = std::max<
                    std::uint32_t>(1u, static_cast<std::uint32_t>(
                        std::llround(dt_ * 1000.0)));
                telemetry_lowstate_consumed_jumped_ =
                    delta > expected_tick_delta &&
                    delta < 0x80000000u;
                telemetry_lowstate_consumed_reordered_ =
                    delta >= 0x80000000u;
            }
            previous_lowstate_consumed_tick_ =
                telemetry_lowstate_consumed_tick_;
            have_lowstate_consumed_tick_ = true;
            ++telemetry_lowstate_consumed_count_;
        }
        if (have_high_state)
        {
            high_state_snapshot = high_state_;
            telemetry_highstate_arrival_wall_time_s_ =
                highstate_arrival_wall_time_s_;
            telemetry_highstate_stamp_s_ = highstate_stamp_s_;
            telemetry_highstate_arrival_count_ = highstate_arrival_count_;
        }
    }
    {
        std::lock_guard<std::mutex> lock(terrain_map_mutex_);
        telemetry_lidar_arrival_wall_time_s_ = lidar_arrival_wall_time_s_;
        telemetry_lidar_stamp_s_ = lidar_stamp_s_;
        telemetry_lidar_arrival_count_ = lidar_arrival_count_;
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

    // Order-109: after the established writer handoff, simulator state time
    // is authoritative even when wall_clock_motion remains configured. The
    // handoff rebase makes this one state delta continuous with running_time_;
    // WriterGate has already rejected missing, reordered, or gapped ticks.
    if (lockstep_ack_enabled_ && lockstep_writer_gate_.Engaged())
    {
        double state_synchronous_dt = 0.0;
        if (lockstep_motion_clock_.Step(
                state_snapshot.tick(), state_synchronous_dt))
        {
            motion_dt = state_synchronous_dt;
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
bool TrotExperiment::WbcStopHoldActive() const
{
    return params_.wbc_full && params_.wbc_primary &&
           task_.gait_started_ && task_.motion_stage_ == 3 &&
           task_.stop_requested_ && !emergency_stop_latched_;
}

bool TrotExperiment::EmergencyStopStanceBlendActive() const
{
    if (!emergency_stop_latched_ || !task_.gait_started_)
        return false;
    const double gait_elapsed_s = running_time_ - task_.gait_start_time_s_;
    return gait_elapsed_s >= emergency_stop_latch_gait_time_s_ + 0.45;
}

bool TrotExperiment::EmergencyStopHoldReady() const
{
    if (!emergency_stop_latched_ || !task_.gait_started_)
        return false;
    const double gait_elapsed_s = running_time_ - task_.gait_start_time_s_;
    // RaibertTrotKernel blends stance hold over about 0.25 s.  The extra
    // margin lets the running trot finish that blend before changing the MPC
    // contact horizon to four-foot support.
    return gait_elapsed_s >= emergency_stop_latch_gait_time_s_ + 0.95;
}
bool TrotExperiment::ComputeWbcPrimaryActive(double &gait_elapsed_s)
{
    bool wbc_primary_active = false;
    gait_elapsed_s = task_.gait_started_ ? running_time_ - task_.gait_start_time_s_ : 0.0;
    // Explicit diagnostic escape hatch for comparing the high-speed
    // kinematic/PD plant against ID-WBC. It is opt-in and never changes the
    // normal --wbc-full path.
    if (Full2EnvDouble("TROT_PD_ONLY", 0.0) > 0.5)
        return false;
    task_.gait_started_ ? running_time_ - task_.gait_start_time_s_ : 0.0;
    const bool regular_wbc =
        task_.motion_stage_ == 2 && task_.gait_started_ &&
        !task_.stop_requested_ &&
        // A commanded zero is the existing stand controller's domain. Keep
        // ID-WBC solving in shadow for the frozen validity gate, but do not
        // feed its locomotion torques into a phase-frozen neutral stance.
        !runtime_velocity_stance_hold_active_ &&
        gait_elapsed_s >= (params_.wbc_full ? 0.0 : kWbcPrimaryEnterDelayS);
    const bool wbc_active = regular_wbc || WbcStopHoldActive();
    if (params_.wbc_primary && wbc_active &&
        wbc_shadow_diagnostics_.solver_ok &&
        wbc_shadow_diagnostics_.mapping_ok)
    {
    wbc_primary_active = true;
    // [impulse] 大步长时 wrench 力矩需求大, 放宽 primary 力矩上限,
    // 避免退回位置控制后 q_error 硬限误杀。
            const double max_abs_torque_nm =
        params_.impulse ? 40.0
                        : (params_.wbc_full
                               ? std::min(params_.tau_limit_nm, 45.0)
                               : kWbcPrimaryMaxAbsTorqueNm);
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
    auto start_high_speed_brake = [&](const char *reason) {
        if (high_speed_stop_brake_active_)
            return;
        const bool health_guard = std::string(reason) == "health guard";
        const double brake_duration = std::clamp(
            Full2EnvDouble(
                health_guard ? "TROT_HS_GOV_HEALTH_BRAKE_DURATION_S"
                             : "TROT_HS_GOV_BRAKE_DURATION_S",
                health_guard ? 0.80 : 2.00),
            0.25, 2.00);
        const double immediate_u = health_guard
            ? std::clamp(
                  Full2EnvDouble("TROT_HS_GOV_HEALTH_BRAKE_U", 0.35),
                  0.0, 0.90)
            : 0.0;
        high_speed_stop_brake_active_ = true;
        high_speed_stop_brake_duration_s_ = brake_duration;
        high_speed_stop_brake_start_time_s_ =
            running_time_ - immediate_u * brake_duration;
        high_speed_stop_brake_base_speed_mps_ =
            wbc_speed_cmd_mps_ > 0.0
                ? std::abs(wbc_speed_cmd_mps_)
                : std::abs(kernel_nominal_velocity_x_mps_);
        if (!(high_speed_stop_brake_base_speed_mps_ > 0.05) &&
            have_world_velocity_)
            high_speed_stop_brake_base_speed_mps_ =
                std::abs(params_.direction_sign * latest_world_velocity_[0]);
        if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
            high_speed_stop_brake_base_speed_mps_ =
                std::abs(params_.direction_sign * params_.step_length_m /
                         std::max(0.08, params_.period_s));
        high_speed_stop_brake_base_period_s_ =
            kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
        high_speed_stop_brake_base_duty_ =
            kernel_duty_factor_ > 0.25 ? kernel_duty_factor_ : params_.duty_factor;
        std::cout << "High-speed " << reason
                  << ": braking in locomotion plant"
                  << " gait_t=" << gait_time_s
                  << " v0=" << high_speed_stop_brake_base_speed_mps_
                  << "\n";
    };
    // Check the sprint guard every control tick, not only at a completed
    // gait cycle.  A pitch/roll excursion can grow from safe to unrecoverable
    // inside one 0.14 s cycle; waiting for cycle diagnostics would be late.
    if (Full2EnvDouble("TROT_HS_GOV_AUTO_BRAKE", 0.0) > 0.5 &&
        !high_speed_stop_brake_active_ && !high_speed_stop_hold_active_ &&
        !task_.stop_requested_ &&
        !motion_event_response_enabled_)
    {
        const double speed = have_world_velocity_
            ? std::abs(params_.direction_sign * latest_world_velocity_[0])
            : 0.0;
        const double roll = std::abs(static_cast<double>(
            state_snapshot.imu_state().rpy()[0]));
        const double pitch = std::abs(static_cast<double>(
            state_snapshot.imu_state().rpy()[1]));
        const double guard_min_speed = std::clamp(
            Full2EnvDouble("TROT_HS_GOV_AUTO_BRAKE_MIN_SPEED", 1.80),
            1.20, 3.50);
        const bool high_speed_observed = speed >= guard_min_speed ||
            std::abs(wbc_speed_cmd_mps_) >= guard_min_speed;
        if (high_speed_observed)
            high_speed_guard_armed_ = true;
        const bool high_speed = high_speed_guard_armed_;
        const double max_angle = std::max(roll, pitch);
        const double guard_angle_limit = std::clamp(
            Full2EnvDouble("TROT_HS_GOV_GUARD_ANGLE_RAD", 0.14),
            0.08, 0.40);
        const double guard_gyro_limit = std::clamp(
            Full2EnvDouble("TROT_HS_GOV_GUARD_GYRO_RADPS", 2.50),
            0.50, 8.0);
        const double gyro_x = std::abs(static_cast<double>(
            state_snapshot.imu_state().gyroscope()[0]));
        const double gyro_y = std::abs(static_cast<double>(
            state_snapshot.imu_state().gyroscope()[1]));
        const double max_gyro = std::max(gyro_x, gyro_y);
        const bool posture_danger = max_angle > guard_angle_limit;
        const bool dynamic_danger = max_angle > 0.08 &&
            max_gyro > guard_gyro_limit &&
            wbc_shadow_diagnostics_.active_contacts <= 1;
        const double guard_min_gait_s = std::clamp(
            Full2EnvDouble("TROT_HS_GOV_GUARD_MIN_GAIT_S", 4.0),
            0.0, 60.0);
        const int guard_hold_ticks = static_cast<int>(std::clamp(
            Full2EnvDouble("TROT_HS_GOV_GUARD_HOLD_TICKS", 2.0),
            1.0, 20.0));
        // The cycle governor already handles slow tracking/contact quality
        // degradation.  This fast path only handles a clearly unsafe
        // instantaneous posture, or a rapidly growing excursion while the
        // measured support set has collapsed.
        // A posture excursion is safety-critical even before the filtered
        // velocity/command has reached the sprint threshold.  Keep this
        // direct path armed for every high-speed campaign after the initial
        // two-second gait hand-off; otherwise a run can roll during ramp-up
        // while the speed latch is still false.
        const bool direct_posture_guard = HighSpeedStopBrakeEnabled() &&
            gait_time_s >= 2.0 && posture_danger;
        const bool tick_degraded = direct_posture_guard ||
            (high_speed && gait_time_s >= guard_min_gait_s &&
             (posture_danger || dynamic_danger));
        if (tick_degraded)
            ++high_speed_guard_bad_ticks_;
        else
            high_speed_guard_bad_ticks_ = 0;
        if (high_speed_guard_bad_ticks_ >= guard_hold_ticks)
        {
            high_speed_guard_bad_ticks_ = 0;
            start_high_speed_brake("health guard");
        }
    }
    else
    {
        high_speed_guard_armed_ = false;
        high_speed_guard_bad_ticks_ = 0;
    }
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
    if (motion_event_response_enabled_)
        UpdateMotionEventResponse(
            gait_time_s, last_motion_dt_s_, state_snapshot,
            high_state_snapshot, have_high_state);
    const double stop_at_speed_mps = Full2EnvDouble(
        "TROT_HS_STOP_AT_SPEED_MPS", -1.0);
    const double stop_speed_hold_s = std::clamp(
        Full2EnvDouble("TROT_HS_STOP_SPEED_HOLD_S", 0.35),
        0.05, 2.0);
    if (HighSpeedStopBrakeEnabled() &&
        !motion_event_response_enabled_ &&
        std::isfinite(stop_at_speed_mps) && stop_at_speed_mps > 0.0 &&
        !task_.stop_requested_ && !emergency_stop_latched_ &&
        !high_speed_stop_brake_active_)
    {
        const double speed = have_world_velocity_
            ? std::abs(params_.direction_sign * latest_world_velocity_[0])
            : 0.0;
        const double roll = std::abs(static_cast<double>(
            state_snapshot.imu_state().rpy()[0]));
        const double pitch = std::abs(static_cast<double>(
            state_snapshot.imu_state().rpy()[1]));
        if (speed >= stop_at_speed_mps &&
            std::max(roll, pitch) <= 10.0 * kPi / 180.0)
        {
            if (high_speed_stop_speed_candidate_start_s_ < 0.0)
                high_speed_stop_speed_candidate_start_s_ = running_time_;
            if (running_time_ - high_speed_stop_speed_candidate_start_s_ >=
                stop_speed_hold_s)
                start_high_speed_brake("speed-threshold stop");
        }
        else
            high_speed_stop_speed_candidate_start_s_ = -1.0;
    }
    else if (!high_speed_stop_brake_active_)
        high_speed_stop_speed_candidate_start_s_ = -1.0;
    // A high-speed acceptance run may request its stop from the controller
    // clock rather than from the wall-clock wrapper or the motion-event
    // layer.  This keeps the sprint reference untouched until the measured
    // gait has reached the requested plateau, then uses the same locomotion
    // plant brake as an external stop.
    const double scheduled_high_speed_stop_s = Full2EnvDouble(
        "TROT_HS_STOP_AFTER_GAIT_S", -1.0);
    if (HighSpeedStopBrakeEnabled() &&
        std::isfinite(scheduled_high_speed_stop_s) &&
        scheduled_high_speed_stop_s >= 0.0 &&
        gait_time_s >= scheduled_high_speed_stop_s &&
        !high_speed_stop_brake_active_ &&
        !task_.stop_requested_ && !emergency_stop_latched_)
    {
        high_speed_stop_brake_active_ = true;
        high_speed_stop_brake_duration_s_ = std::clamp(
            Full2EnvDouble("TROT_HS_GOV_BRAKE_DURATION_S", 2.00),
            0.25, 2.00);
        high_speed_stop_brake_start_time_s_ = running_time_;
        high_speed_stop_brake_base_speed_mps_ =
            wbc_speed_cmd_mps_ > 0.0
                ? std::abs(wbc_speed_cmd_mps_)
                : std::abs(kernel_nominal_velocity_x_mps_);
        if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
            high_speed_stop_brake_base_speed_mps_ =
                std::abs(params_.direction_sign * params_.step_length_m /
                         std::max(0.08, params_.period_s));
        high_speed_stop_brake_base_period_s_ =
            kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
        high_speed_stop_brake_base_duty_ =
            kernel_duty_factor_ > 0.25 ? kernel_duty_factor_ : params_.duty_factor;
        std::cout << "High-speed scheduled stop: braking in locomotion plant"
                  << " at gait_t=" << gait_time_s
                  << " v0=" << high_speed_stop_brake_base_speed_mps_
                  << "\n";
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
        !task_.sequence_finished_ && !emergency_stop_latched_ &&
        gait_time_s >= duration_s_ &&
        std::isfinite(duration_s_))
    {
        if (HighSpeedStopBrakeEnabled())
        {
            if (!high_speed_stop_brake_active_)
            {
                high_speed_stop_brake_active_ = true;
                high_speed_stop_brake_duration_s_ = std::clamp(
                    Full2EnvDouble("TROT_HS_GOV_BRAKE_DURATION_S", 2.00),
                    0.25, 2.00);
                high_speed_stop_brake_start_time_s_ = running_time_;
                high_speed_stop_brake_base_speed_mps_ =
                    wbc_speed_cmd_mps_ > 0.0
                        ? std::abs(wbc_speed_cmd_mps_)
                        : std::abs(kernel_nominal_velocity_x_mps_);
                if (!(high_speed_stop_brake_base_speed_mps_ > 0.05))
                    high_speed_stop_brake_base_speed_mps_ =
                        std::abs(params_.direction_sign * params_.step_length_m /
                                 std::max(0.08, params_.period_s));
                high_speed_stop_brake_base_period_s_ =
                    kernel_period_s_ > 0.05
                        ? kernel_period_s_ : params_.period_s;
                high_speed_stop_brake_base_duty_ =
                    kernel_duty_factor_ > 0.25
                        ? kernel_duty_factor_ : params_.duty_factor;
                std::cout << "High-speed timed stop: braking in"
                          << " locomotion plant\n";
            }
        }
        else if (!stop_brake_active_)
        {
            stop_brake_active_ = true;
            stop_brake_start_time_s_ = running_time_;
            stop_brake_base_step_m_ =
                std::abs(kernel_nominal_velocity_x_mps_) *
                std::max(kernel_period_s_, params_.period_s);
            if (!(stop_brake_base_step_m_ > 1.0e-6))
                stop_brake_base_step_m_ = std::abs(params_.step_length_m);
            std::cout << "Trot pre-stop brake: reducing gait reference\n";
        }
    }
    if (stop_brake_active_ && !HighSpeedStopBrakeEnabled() &&
        !task_.stop_requested_ &&
        !emergency_stop_latched_)
    {
        const double brake_u = Smoothstep(
            (running_time_ - stop_brake_start_time_s_) /
            kStopBrakeDurationS);
        // Keep the same gait plant while reducing momentum; do not jump
        // directly from a full-speed swing target into four-foot stance.
        const double step_scale = 1.0 - 0.55 * brake_u;
        const double base_step = stop_brake_base_step_m_ > 1.0e-6
            ? stop_brake_base_step_m_
            : std::abs(params_.step_length_m);
        locomotion_kernel_->SetGaitStepLength(
            base_step * step_scale);
        if (running_time_ - stop_brake_start_time_s_ >=
            kStopBrakeDurationS)
        {
            task_.stop_requested_ = true;
            if (task_.task_mode_)
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
    const bool high_speed_wbc = params_.wbc_full &&
        HighSpeedStopBrakeEnabled();
    const double high_speed_ramp_s = std::clamp(
        Full2EnvDouble("TROT_HS_PRIMARY_RAMP_S", 0.05),
        0.05, 1.50);
    const double primary_ramp = params_.wbc_full
        ? (high_speed_wbc
               ? Smoothstep(gait_elapsed_s / high_speed_ramp_s)
               : 1.0)
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
        const double blend_tau = std::clamp(
            Full2EnvDouble("FULL2_BLEND_TAU", kWbcPrimaryBlendTauS),
            0.010, 0.50);
        wbc_stance_blend_[leg] +=
            (target_blend - wbc_stance_blend_[leg]) *
            std::clamp(dt_ / blend_tau, 0.0, 1.0);
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
            const double full_stance_kp = std::max(
                0.0, Full2EnvDouble("FULL2_STANCE_KP", kWbcFullStanceKp));
            const double full_stance_kd = std::max(
                0.0, Full2EnvDouble("FULL2_STANCE_KD", kWbcFullStanceKd));
            const double stance_kp = params_.cartesian_world
                ? (joint == 0 ? hip_kp : sagittal_kp)
                : (params_.wbc_full
                    ? full_stance_kp
                    : (params_.impulse ? kImpulseStanceKp : kWbcPrimaryCommandKp));
            const double kd_cheetah =
                cartesian_kp_frozen_
                    ? Smoothstep((8.0 - cartesian_latched_kp_) / 5.0)
                    : 0.0;
            const double stance_kd = params_.cartesian_world
                ? (joint == 0 ? 2.6
                   : (2.5 * (1.0 - kd_cheetah) + 0.3 * kd_cheetah))
                : (params_.wbc_full
                    ? full_stance_kd
                    : kWbcPrimaryCommandKd);
            // 142128: CART-GOV printed kp=3 but FR_thigh motor kp
            // median was 33 because swing mixed params_.kp=63 through
            // wbc_stance_blend. Cheetah uses 3/0.3 on all sagittal
            // joints once latched. Do not mix 63 after freeze.
            const double swing_motor_kp = std::max(
                0.0, Full2EnvDouble("FULL2_SWING_MOTOR_KP", params_.kp));
            const double swing_motor_kd = std::max(
                0.0, Full2EnvDouble("FULL2_SWING_MOTOR_KD", params_.kd));
            double cmd_kp = stance_kp * wbc_stance_blend_[leg] +
                swing_motor_kp * (1.0 - wbc_stance_blend_[leg]);
            double cmd_kd = stance_kd * wbc_stance_blend_[leg] +
                swing_motor_kd * (1.0 - wbc_stance_blend_[leg]);
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
            double swing_tau_scale = 1.0;
            if (params_.wbc_full &&
                Full2EnvDouble("TROT_HS_SWING_TAU_SCALE", -1.0) >= 0.0)
            {
                const bool high_speed_curriculum =
                    Full2EnvDouble("TROT_HS_DISABLE", 0.0) <= 0.5 &&
                    (params_.gait_pattern !=
                         go2_control::GaitPattern::kDiagonalTrot ||
                     std::abs(wbc_speed_cmd_mps_) > 1.25 ||
                     std::abs(kernel_nominal_velocity_x_mps_) > 1.25);
                if (high_speed_curriculum && !leg_in_stance)
                    swing_tau_scale = std::clamp(
                        Full2EnvDouble("TROT_HS_SWING_TAU_SCALE", 1.0),
                        0.0, 1.0);
            }
            low_cmd_.motor_cmd()[i].tau() =
                primary_ramp * contact_scale *
                (params_.wbc_full ? 1.0 : wbc_stance_blend_[leg]) *
                swing_tau_scale * wbc_shadow_candidate_torques_[leg][joint];
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
