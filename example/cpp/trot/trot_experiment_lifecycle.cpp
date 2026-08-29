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
    std::lock_guard<std::mutex> lock(state_mutex_);
    low_state_ = *(unitree_go::msg::dds_::LowState_ *)message;
    have_low_state_ = true;
}

// --- TrotExperiment::HighStateMessageHandler ---
void TrotExperiment::HighStateMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_ = *(unitree_go::msg::dds_::SportModeState_ *)message;
    have_high_state_ = true;
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
    std::lock_guard<std::mutex> lock(terrain_map_mutex_);
    const bool first_message = !have_lidar_heightmap_;
    lidar_heightmap_ =
        *static_cast<const unitree_go::msg::dds_::HeightMap_ *>(message);
    have_lidar_heightmap_ = true;
    if (first_message)
    {
        std::cerr << "Lidar map received: stamp="
                  << lidar_heightmap_.stamp()
                  << " cells=" << lidar_heightmap_.data().size()
                  << " frame=" << lidar_heightmap_.frame_id() << "\n";
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
    terrain_config.sensor_only = params_.terrain_sensor_only;
    terrain_config.allow_actuation = params_.terrain_actuation;
    if (terrain_config.allow_actuation && !terrain_config.sensor_only)
    {
        // An actuating terrain plan is an atomic future-contact transaction,
        // not a one-tick observation.  Keep it valid through the configured
        // contact horizon so a foothold can be armed in stance and executed
        // at the next swing without falling back to a late riser handoff.
        terrain_config.plan_validity_s = 0.50;
        // The rear leg needs a longer preview than the default 24 knots:
        // with the base before the riser, its elevated foothold becomes
        // reachable only after the body has advanced onto the platform.
        terrain_config.horizon_knots = go2_terrain::kTerrainPlanMaxKnots;
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
                  << (params_.terrain_sensor_only ? "on" : "off")
                  << " actuation="
                  << (params_.terrain_actuation ? "on" : "off") << "\n";
    }

    std::cout << "Waiting for natural settle...\n";
    if (!WaitForNaturalSettle(8.0))
    {
        std::cerr << "Natural settle timed out\n";
        return false;
    }
    CaptureWorldReference();

    if (params_.terrain_enabled)
    {
        terrain_worker_stop_.store(false);
        terrain_planner_thread_ = std::thread(
            &TrotExperiment::TerrainPlannerWorker, this);
    }
    writer_stop_.store(false);
    low_cmd_write_thread_ = std::thread([this]() {
        PinCurrentThreadToEnv("TROT_WRITER_CPU");
        const auto interval = std::chrono::microseconds(
            static_cast<int64_t>(dt_ * 1000000.0));
        auto next = std::chrono::steady_clock::now();
        while (!writer_stop_.load() && !finished_.load())
        {
            LowCmdWrite();
            next += interval;
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() > next + interval * 4)
                next = std::chrono::steady_clock::now();
        }
    });
    return true;
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
