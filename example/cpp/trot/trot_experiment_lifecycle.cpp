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
#include "full2_campaign_env.h"
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "motion_frame_utils.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

// --- TrotExperiment::InitLowCmd ---
void TrotExperiment::InitLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;
    for (int i = 0; i < 20; ++i)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q() = kPosStopF;
        low_cmd_.motor_cmd()[i].kp() = 0.0;
        low_cmd_.motor_cmd()[i].dq() = kVelStopF;
        low_cmd_.motor_cmd()[i].kd() = 0.0;
        low_cmd_.motor_cmd()[i].tau() = 0.0;
    }
}

// --- TrotExperiment::LowStateMessageHandler ---
void TrotExperiment::LowStateMessageHandler(const void *message)
{
    const unitree_go::msg::dds_::LowState_ *msg =
        static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    const double arrival_wall_time_s = WallClockTelemetryTimeS();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const std::uint32_t tick = msg->tick();
        lowstate_arrival_tick_delta_ = 0;
        lowstate_arrival_repeated_ = false;
        lowstate_arrival_jumped_ = false;
        lowstate_arrival_reordered_ = false;
        if (have_lowstate_arrival_tick_)
        {
            const std::uint32_t delta =
                tick - previous_lowstate_arrival_tick_;
            lowstate_arrival_tick_delta_ = delta;
            lowstate_arrival_repeated_ = delta == 0;
            lowstate_arrival_jumped_ =
                delta > 1 && delta < 0x80000000u;
            lowstate_arrival_reordered_ = delta >= 0x80000000u;
        }
        previous_lowstate_arrival_tick_ = tick;
        have_lowstate_arrival_tick_ = true;
        lowstate_arrival_tick_ = tick;
        ++lowstate_arrival_count_;
        lowstate_arrival_wall_time_s_ = arrival_wall_time_s;
        low_state_ = *msg;
        have_low_state_ = true;
    }
    // Order-108 verification-only tick gate: strictly-new-tick detection
    // and (once engaged) stale/reorder/gap fail-closed. No-op for the
    // wall-clock runner (adapter off -> gate never engaged).
    if (lockstep_ack_enabled_)
        lockstep_writer_gate_.OnLowState(msg->tick());
}

// --- TrotExperiment::HighStateMessageHandler ---
void TrotExperiment::HighStateMessageHandler(const void *message)
{
    const auto *msg =
        static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    const double arrival_wall_time_s = WallClockTelemetryTimeS();
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_ = *msg;
    have_high_state_ = true;
    ++highstate_arrival_count_;
    highstate_arrival_wall_time_s_ = arrival_wall_time_s;
    highstate_stamp_s_ =
        static_cast<double>(msg->stamp().sec()) +
        static_cast<double>(msg->stamp().nanosec()) * 1.0e-9;
}

void TrotExperiment::EnvironmentHeightMapMessageHandler(const void *message)
{
    if (message == nullptr)
        return;
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool first_message = !have_environment_heightmap_;
    environment_heightmap_ =
        *static_cast<const unitree_go::msg::dds_::HeightMap_ *>(message);
    have_environment_heightmap_ = true;
    if (first_message)
    {
        std::cerr << "Environment map received: stamp="
                  << environment_heightmap_.stamp()
                  << " cells=" << environment_heightmap_.data().size()
                  << " frame=" << environment_heightmap_.frame_id() << "\n";
    }
}

void TrotExperiment::LidarHeightMapMessageHandler(const void *message)
{
    if (message == nullptr)
        return;
    const double arrival_wall_time_s = WallClockTelemetryTimeS();
    std::lock_guard<std::mutex> lock(terrain_map_mutex_);
    const bool first_message = !have_lidar_heightmap_;
    lidar_heightmap_ =
        *static_cast<const unitree_go::msg::dds_::HeightMap_ *>(message);
    have_lidar_heightmap_ = true;
    ++lidar_arrival_count_;
    lidar_arrival_wall_time_s_ = arrival_wall_time_s;
    lidar_stamp_s_ = lidar_heightmap_.stamp();
    if (first_message)
    {
        std::cerr << "Lidar map received: stamp="
                  << lidar_heightmap_.stamp()
                  << " cells=" << lidar_heightmap_.data().size()
                  << " frame=" << lidar_heightmap_.frame_id() << "\n";
    }
}

void TrotExperiment::LidarTerrainEnvelopeMessageHandler(const void *message)
{
    if (message == nullptr)
        return;
    const auto *msg =
        static_cast<const std_msgs::msg::dds_::String_ *>(message);
    const auto now_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    std::string wire;
    std::uint64_t sequence = 0;
    const auto transport = lidar_map_reassembler_.PushPacket(
        msg->data(), now_ms, wire, &sequence);
    if (transport == go2_terrain::TerrainMapChunkError::kNone ||
        transport == go2_terrain::TerrainMapChunkError::kDuplicate ||
        transport == go2_terrain::TerrainMapChunkError::kStaleSequence)
        return;
    if (transport != go2_terrain::TerrainMapChunkError::kComplete)
    {
        std::lock_guard<std::mutex> lock(terrain_map_mutex_);
        have_lidar_map_envelope_ = false;
        lidar_map_envelope_ = {};
        ++lidar_map_transport_errors_;
        if (lidar_map_transport_errors_ <= 3 ||
            lidar_map_transport_errors_ % 100 == 0)
            std::cerr << "Rejected terrain map transport error="
                      << go2_terrain::TerrainMapChunkErrorName(transport)
                      << " count=" << lidar_map_transport_errors_ << "\n";
        return;
    }
    const auto decoded = go2_terrain::DeserializeTerrainMapEnvelope(wire);
    std::lock_guard<std::mutex> lock(terrain_map_mutex_);
    if (!decoded.ok() || decoded.envelope.sequence != sequence)
    {
        // Keep the legacy HeightMap for diagnostics, but make the
        // actuation envelope unavailable after any malformed packet.
        have_lidar_map_envelope_ = false;
        lidar_map_envelope_ = {};
        std::cerr << "Rejected terrain capture envelope error="
                  << static_cast<int>(decoded.error) << "\n";
        return;
    }
    lidar_map_envelope_ = decoded.envelope;
    have_lidar_map_envelope_ = true;
    ++lidar_map_complete_count_;
    if (lidar_map_complete_count_ <= 3)
        std::cerr << "Terrain envelope received sequence=" << sequence
                  << " bytes=" << wire.size()
                  << " capture=" << lidar_map_envelope_.map_stamp_s << "\n";
    if (lidar_map_envelope_.frame_id != "base_link")
    {
        have_lidar_map_envelope_ = false;
        lidar_map_envelope_ = {};
        std::cerr << "Rejected terrain capture envelope frame" << "\n";
    }
}

// --- TrotExperiment::WaitForNaturalSettle ---
bool TrotExperiment::WaitForNaturalSettle(double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_s);
    auto stable_since = std::chrono::steady_clock::time_point{};

    while (std::chrono::steady_clock::now() < deadline)
    {
        bool stable = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_low_state_)
            {
                double max_joint_speed = 0.0;
                double max_body_angular_speed = 0.0;
                for (int i = 0; i < kMotorCount; ++i)
                {
                    max_joint_speed = std::max(
                        max_joint_speed,
                        std::abs(static_cast<double>(
                            low_state_.motor_state()[i].dq())));
                }
                for (int axis = 0; axis < 3; ++axis)
                {
                    max_body_angular_speed = std::max(
                        max_body_angular_speed,
                        std::abs(static_cast<double>(
                            low_state_.imu_state().gyroscope()[axis])));
                }
                stable =
                    max_joint_speed <= 0.05 &&
                    max_body_angular_speed <= 0.05;
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (stable)
        {
            if (stable_since == std::chrono::steady_clock::time_point{})
                stable_since = now;
            if (now - stable_since >= std::chrono::duration<double>(0.5))
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int i = 0; i < kMotorCount; ++i)
                    task_.start_joint_pos_[i] = low_state_.motor_state()[i].q();
                task_.have_start_joint_pos_ = true;
                std::cout << "Natural LowState settled\n";
                return true;
            }
        }
        else
        {
            stable_since = std::chrono::steady_clock::time_point{};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// --- TrotExperiment::CaptureWorldReference ---
bool TrotExperiment::CaptureWorldReference()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_low_state_ || !have_high_state_)
    {
        std::cerr << "World reference unavailable; continue without world pose feedback\n";
        return false;
    }
    const WorldPose pose = ComputeWorldPose(low_state_, high_state_);
    world_reference_x_m_ = pose.base.x;
    world_reference_y_m_ = pose.base.y;
    world_reference_yaw_rad_ = pose.yaw_rad;
    have_world_reference_ = true;
    std::cout << "World reference captured: x=" << pose.base.x
              << " y=" << pose.base.y << " yaw=" << pose.yaw_rad << "\n";
    if (task_.goal_enabled_)
    {
        std::cout << "World A→B remaining="
                  << task_.RemainingXy(pose.base.x, pose.base.y)
                  << " m toward (" << task_.goal_x_ << ", "
                  << task_.goal_y_ << ")\n";
    }
    return true;
}

// --- TrotExperiment::Init ---
bool TrotExperiment::Init()
{
    go2_trot::ContinuousVelocityGaitResearchConfig research_config;
    std::string research_config_error;
    if (!go2_trot::LoadContinuousVelocityGaitResearchConfigFromEnvironment(
            research_config, &research_config_error))
    {
        std::cerr << "Invalid runtime running-gait research config: "
                  << research_config_error << "\n";
        return false;
    }
    if (!velocity_gait_scheduler_.ConfigureResearch(research_config))
    {
        std::cerr << "Invalid runtime running-gait research config after parse\n";
        return false;
    }
    std::ostringstream research_config_record;
    research_config_record << std::fixed << std::setprecision(9)
                           << "Runtime running-gait research config: period_s="
                           << research_config.running_period_s
                           << " lift_floor_m="
                           << research_config.running_lift_floor_m;
    std::cout << research_config_record.str() << "\n";
    telemetry_start_time_ = std::chrono::steady_clock::now();
    csv_.open(csv_path_);
    if (!csv_)
    {
        std::cerr << "Failed to open CSV: " << csv_path_ << "\n";
        return false;
    }
    csv_ << std::fixed << std::setprecision(9);
    WriteCsvHeader();
    InitLowCmd();

    go2_terrain::TerrainPlannerConfig terrain_config;
    const bool terrain_consistency_shadow = Full2EnvDouble(
        "TROT_TERRAIN_EXECUTION_CONSISTENCY_SHADOW", 0.0) > 0.5;
    // Shadow is an opt-in diagnostic planner. It may publish an immutable
    // plan for validation, while terrain_actuation remains the sole switch
    // that permits gait/MPC/WBC consumers to use it.
    terrain_config.sensor_only =
        params_.terrain_sensor_only && !terrain_consistency_shadow;
    terrain_config.allow_actuation =
        params_.terrain_actuation || terrain_consistency_shadow;
    if (terrain_consistency_shadow)
    {
        // The shadow consumer validates an 8-knot MPC horizon after the
        // observed planner-consumption delay (up to about 300 ms). Generate
        // real knots through that delay plus the MPC horizon. The shadow
        // consumer separately requires complete coverage before checking.
        terrain_config.horizon_knots = 24;
        terrain_config.plan_validity_s = 0.46;
    }
    terrain_planner_ = go2_terrain::TerrainPlanner(terrain_config);

    if (params_.wbc_full)
    {
#ifdef GO2_MODEL_PATH
        rigid_body_ = std::make_unique<go2_control::Go2RigidBody>();
        if (!rigid_body_->Load(GO2_MODEL_PATH))
        {
            std::cerr << "Failed to load Go2 MJCF for --wbc-full: "
                      << GO2_MODEL_PATH << "\n";
            return false;
        }
        std::cout << "WBC-FULL: 18-DoF MJCF model loaded\n";
#else
        std::cerr << "--wbc-full requires a controller-side MuJoCo model\n";
        return false;
#endif
    }

    std::cout << "Locomotion kernel: " << locomotion_kernel_->Name() << "\n";

    lowcmd_publisher_.reset(
        new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(GO2_TROT_TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    // Order-105/106 verification-only causal handshake: when the sim runs
    // with the lockstep flag the harness also sets TROT_LOCKSTEP_ACK=1, so
    // this adapter acks {state_seq} after every LowCmd write. Off by
    // default; the flag never changes control math.
    if (Full2EnvDouble("TROT_LOCKSTEP_ACK", 0.0) > 0.5)
    {
        lockstep_ack_publisher_.reset(
            new ChannelPublisher<unitree_go::msg::dds_::Error_>(
                GO2_TROT_TOPIC_LOCKSTEP_ACK));
        lockstep_ack_publisher_->InitChannel();
        lockstep_ack_enabled_ = true;
        std::cout << "Lockstep ack adapter enabled on "
                  << GO2_TROT_TOPIC_LOCKSTEP_ACK << "\n";
    }
    // Order-108: fail closed if the writer waits longer than this for the
    // next strictly-new tick (default matches SIM_LOCKSTEP_EXCHANGE_TIMEOUT_S).
    lockstep_writer_gate_.SetTickWaitTimeoutS(
        Full2EnvDouble("TROT_LOCKSTEP_TICK_TIMEOUT_S", 5.0));

    lowstate_subscriber_.reset(
        new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(GO2_TROT_TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
        std::bind(
            &TrotExperiment::LowStateMessageHandler,
            this,
            std::placeholders::_1),
        1);

    highstate_subscriber_.reset(
        new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
            GO2_TROT_TOPIC_HIGHSTATE));
    highstate_subscriber_->InitChannel(
        std::bind(
            &TrotExperiment::HighStateMessageHandler,
            this,
            std::placeholders::_1),
        1);

    if (params_.auto_environment)
    {
        environment_heightmap_subscriber_.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>(
                GO2_TROT_TOPIC_ENVIRONMENT_MAP));
        environment_heightmap_subscriber_->InitChannel(
            std::bind(
                &TrotExperiment::EnvironmentHeightMapMessageHandler,
                this,
                std::placeholders::_1),
            1);
        std::cout << "Automatic environment map: "
                  << GO2_TROT_TOPIC_ENVIRONMENT_MAP << "\n";
    }

    if (params_.terrain_enabled)
    {
        lidar_heightmap_subscriber_.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>(
                GO2_TROT_TOPIC_LIDAR_MAP));
        lidar_heightmap_subscriber_->InitChannel(
            std::bind(
                &TrotExperiment::LidarHeightMapMessageHandler,
                this,
                std::placeholders::_1),
            1);
        std::cout << "Terrain lidar map: " << GO2_TROT_TOPIC_LIDAR_MAP
                  << " sensor_only="
                  << (params_.terrain_sensor_only ? "on" : "off") << "\n";

        lidar_map_envelope_subscriber_.reset(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(
                GO2_TROT_TOPIC_LIDAR_MAP_ENVELOPE));
        lidar_map_envelope_subscriber_->InitChannel(
            std::bind(
                &TrotExperiment::LidarTerrainEnvelopeMessageHandler,
                this,
                std::placeholders::_1),
            512);
        std::cout << "Terrain capture envelope: "
                  << GO2_TROT_TOPIC_LIDAR_MAP_ENVELOPE << "\n";
    }

    std::cout << "Waiting for natural settle...\n";
    if (!WaitForNaturalSettle(8.0))
    {
        std::cerr << "Natural settle timed out\n";
        return false;
    }
    CaptureWorldReference();

    auto start_terrain_worker = [this]() {
        terrain_worker_stop_.store(false);
        terrain_planner_thread_ = std::thread(
            &TrotExperiment::TerrainPlannerWorker, this);
    };
    // Diagnostic-only: keep the terrain subscription/lidar path but omit the
    // asynchronous planner worker. The production default remains enabled.
    const bool terrain_worker_disabled =
        Full2EnvDouble("TROT_TERRAIN_WORKER_DISABLE", 0.0) > 0.5;
    writer_stop_.store(false);
    low_cmd_write_thread_ = std::thread([this]() {
        PinCurrentThreadToEnv("TROT_WRITER_CPU");
        const auto interval = std::chrono::microseconds(
            static_cast<int64_t>(dt_ * 1000000.0));
        auto next = std::chrono::steady_clock::now();
        while (!writer_stop_.load() && !finished_.load())
        {
            // Order-108 verification-only tick gate: after the controller
            // handoff (TROT_LOCKSTEP_ACK on AND the first lockstep state
            // consumed post start-gait) the writer stops free-running on the
            // wall clock and consumes exactly ONE new physics tick per loop
            // iteration: one full LowCmdWrite/control update, one LowCmd
            // publish, one ack of the exact {state_seq, command_seq} pair,
            // then it waits for the next tick. Before the handoff -- and
            // whenever the adapter is off -- the original wall-clock
            // lifecycle below is unchanged.
            if (lockstep_ack_enabled_ && lockstep_epoch_valid_)
            {
                EngageLockstepWriterIfNeeded();
                std::uint32_t pending_tick = 0;
                const lockstep_writer::WaitResult wait =
                    lockstep_writer_gate_.WaitForTick([this]() {
                        return writer_stop_.load() || finished_.load();
                    }, &pending_tick);
                if (wait == lockstep_writer::WaitResult::kAborted)
                    break;
                if (wait == lockstep_writer::WaitResult::kTimeout)
                {
                    // The gate already printed the
                    // TROT_LOCKSTEP_WRITER_FAIL_CLOSED diagnostic.
                    finished_.store(true);
                    break;
                }
                if (!LowCmdWrite(pending_tick, true))
                {
                    finished_.store(true);
                    break;
                }
                lockstep_writer_gate_.RecordConsumed(pending_tick);
            }
            else
            {
                LowCmdWrite();
                next += interval;
                std::this_thread::sleep_until(next);
                if (std::chrono::steady_clock::now() > next + interval * 4)
                    next = std::chrono::steady_clock::now();
            }
        }
    });
    // Let the first wall-clock writer handoff establish itself before the
    // observer worker starts scheduling. The worker remains read-only in
    // sensor-only mode and is still started immediately after the writer.
    if (params_.terrain_enabled && !terrain_worker_disabled)
        start_terrain_worker();
    return true;
}

// The writer and the integration probe share this production handoff.
void TrotExperiment::EngageLockstepWriterIfNeeded()
{
    if (lockstep_writer_gate_.Engaged())
        return;
    lockstep_writer_gate_.Engage(last_consumed_state_tick_);
    // Rebase the motion clock at the exact handoff tick so the first gated
    // update advances time once, not twice (or zero times) across transition.
    lockstep_motion_clock_.Engage(last_consumed_state_tick_);
}

// --- TrotExperiment::Shutdown ---
void TrotExperiment::Shutdown()
{
    writer_stop_.store(true);
    if (low_cmd_write_thread_.joinable())
        low_cmd_write_thread_.join();
    terrain_worker_stop_.store(true);
    terrain_work_cv_.notify_all();
    if (terrain_planner_thread_.joinable())
        terrain_planner_thread_.join();
    csv_.close();
}

// --- TrotExperiment::RequestStop ---
void TrotExperiment::RequestStop()
{
    if (!external_stop_requested_.exchange(true))
    {
        task_.task_completion_requested_ = false;
        std::cout << "Trot external stop requested; returning to stand\n";
    }
}

// --- TrotExperiment::StopFileRequested ---
bool TrotExperiment::StopFileRequested() const
{
    if (stop_file_path_.empty())
        return false;
    std::ifstream stop_file(stop_file_path_);
    return stop_file.good();
}
