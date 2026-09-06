#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/go2/Error_.hpp>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include "../../example/cpp/terrain/terrain_map_envelope.h"
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cstdlib>
#if defined(__linux__)
#include <sched.h>
#endif
#include <thread>
#include <vector>
#include <limits>
#include <mutex>
#include <condition_variable>
#include <string>
#include <type_traits>

#include "param.h"
#include "physics_joystick.h"
#include "lockstep.h"

// Order-103 verification-only lockstep coordinator, owned by main.cc.
namespace lockstep
{
class Coordinator;
}
extern lockstep::Coordinator *g_lockstep;

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(
        mjModel *model,
        mjData *data,
        std::recursive_mutex *sim_mutex)
    : mj_model_(model), mj_data_(data), sim_mutex_(sim_mutex)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual ~UnitreeSDK2BridgeBase() = default;

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;
    std::recursive_mutex *sim_mutex_ = nullptr;

    std::unique_lock<std::recursive_mutex> LockSimulation()
    {
        if (sim_mutex_ == nullptr)
        {
            return {};
        }
        return std::unique_lock<std::recursive_mutex>(*sim_mutex_);
    }

    void PinCurrentThreadToEnv(const char *env_name)
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
            std::cerr << "Unable to pin " << env_name << " to CPU " << cpu << "\n";
#else
        (void)env_name;
#endif
    }

    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;
    std::array<int, 4> foot_force_adr_ = {-1, -1, -1, -1};

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        const std::array<const char *, 4> foot_force_sensor_names = {
            "FR_foot_force", "FL_foot_force", "RR_foot_force", "RL_foot_force"};
        for (std::size_t i = 0; i < foot_force_sensor_names.size(); ++i) {
            sensor_id = mj_name2id(
                mj_model_, mjOBJ_SENSOR, foot_force_sensor_names[i]);
            if (sensor_id >= 0) {
                foot_force_adr_[i] = mj_model_->sensor_adr[sensor_id];
            }
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

// LowCmd subscription that counts every DDS arrival so the lockstep
// exchange rule can wait for a full controller write period. Message state
// handling is identical to SubscriptionBase's default handler; the
// wall-clock path keeps the plain LowCmd_t and is byte-identical.
template <typename MsgType>
class CountingLowCmd : public unitree::robot::SubscriptionBase<MsgType>
{
public:
    CountingLowCmd(const std::string &topic, lockstep::Coordinator *coord)
        : unitree::robot::SubscriptionBase<MsgType>(
              topic, [this, coord](const void *msg) {
                  if (coord != nullptr) coord->OnCommandArrived();
                  std::lock_guard<std::mutex> lock(this->mutex_);
                  this->msg_ = *(const MsgType *)msg;
              })
    {
    }
};

// Order-107 verification-only ack subscription: the controller adapter
// publishes ack{state_seq, command_seq} (unitree Error_ type repurposed as
// a sequence-metadata carrier; Error_.source()/state() are uint32_t and
// carry the full-width frozen-state tick and the controller's exact
// command_seq) on rt/lockstep/ack after each LowCmd write. Only created
// when the lockstep flag is on.
class LockstepAckSubscriber
    : public unitree::robot::SubscriptionBase<unitree_go::msg::dds_::Error_>
{
public:
    explicit LockstepAckSubscriber(const std::string &topic,
                                   lockstep::Coordinator *coord)
        : unitree::robot::SubscriptionBase<unitree_go::msg::dds_::Error_>(
              topic, [coord](const void *msg) {
                  if (coord == nullptr) return;
                  const auto *m = static_cast<
                      const unitree_go::msg::dds_::Error_ *>(msg);
                  coord->OnAckReceived(m->source(), m->state());
              })
    {
    }
};

public:
    RobotBridge(
        mjModel *model,
        mjData *data,
        std::recursive_mutex *sim_mutex)
        : UnitreeSDK2BridgeBase(model, data, sim_mutex)
    {
        if (param::config.lockstep)
        {
            lowcmd = std::make_shared<CountingLowCmd<typename LowCmd_t::MsgType>>(
                "rt/lowcmd", g_lockstep);
            lockstep_ack_subscriber_ =
                std::make_shared<LockstepAckSubscriber>("rt/lockstep/ack",
                                                        g_lockstep);
        }
        else
        {
            lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        }
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        environment_heightmap = unitree::robot::ChannelFactory::Instance()
            ->CreateSendChannel<unitree_go::msg::dds_::HeightMap_>(
                "rt/go2/environment_heightmap");
        lidar_heightmap = unitree::robot::ChannelFactory::Instance()
            ->CreateSendChannel<unitree_go::msg::dds_::HeightMap_>(
                "rt/go2/lidar_heightmap");
        lidar_heightmap_envelope = unitree::robot::ChannelFactory::Instance()
            ->CreateSendChannel<std_msgs::msg::dds_::String_>(
                "rt/go2/lidar_heightmap_capture_v1");
        lidar_world_z_.assign(kLidarWorldCellCount,
                              std::numeric_limits<double>::quiet_NaN());
        lidar_world_t_.assign(kLidarWorldCellCount, -1.0e9);
        InitRuntimeTelemetry();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
        terrain_lidar_mode_ = ParseTerrainLidarRuntimeMode();
        if (param::config.terrain_lidar &&
            terrain_lidar_mode_ != TerrainLidarRuntimeMode::kNone)
        {
            terrain_lidar_stop_.store(false);
            terrain_lidar_thread_ = std::thread(
                [this]() { TerrainLidarLoop(); });
        }
    }

    ~RobotBridge() override
    {
        terrain_lidar_stop_.store(true);
        terrain_lidar_park_cv_.notify_all();
        if (terrain_lidar_thread_.joinable())
            terrain_lidar_thread_.join();
    }

private:
    enum class TerrainLidarRuntimeMode
    {
        kFull,
        kSnapshot,
        kPark,
        kNone,
    };

    static TerrainLidarRuntimeMode ParseTerrainLidarRuntimeMode()
    {
        const char *mode_env = std::getenv("TROT_SIM_LIDAR_MODE");
        if (mode_env != nullptr && mode_env[0] != 0)
        {
            const std::string mode(mode_env);
            if (mode == "none")
                return TerrainLidarRuntimeMode::kNone;
            if (mode == "park")
                return TerrainLidarRuntimeMode::kPark;
            if (mode == "snapshot" || mode == "noop")
                return TerrainLidarRuntimeMode::kSnapshot;
            if (mode == "full")
                return TerrainLidarRuntimeMode::kFull;
            std::cerr << "Unknown TROT_SIM_LIDAR_MODE='" << mode
                      << "'; using full\n";
            return TerrainLidarRuntimeMode::kFull;
        }
        const char *lidar_noop_env = std::getenv("TROT_SIM_LIDAR_NOOP");
        if (lidar_noop_env != nullptr && lidar_noop_env[0] == '1')
            return TerrainLidarRuntimeMode::kSnapshot;
        return TerrainLidarRuntimeMode::kFull;
    }

    void TerrainLidarLoop()
    {
#if defined(__linux__)
        sched_param scheduler_params{};
        (void)sched_setscheduler(0, SCHED_IDLE, &scheduler_params);
#endif
        PinCurrentThreadToEnv("TROT_SIM_LIDAR_CPU");
        if (terrain_lidar_mode_ == TerrainLidarRuntimeMode::kPark)
        {
            LogRuntimeTelemetry(
                "lidar_park", -1.0, 0, 0.0, 0.0, 0.0, -1.0, false);
            std::unique_lock<std::mutex> lock(terrain_lidar_park_mutex_);
            terrain_lidar_park_cv_.wait(lock, [this]() {
                return terrain_lidar_stop_.load();
            });
            return;
        }
        mjModel *sensor_model = mj_copyModel(nullptr, mj_model_);
        if (sensor_model == nullptr)
            return;
        mjData *sensor_data = mj_makeData(sensor_model);
        if (sensor_data == nullptr)
        {
            mj_deleteModel(sensor_model);
            return;
        }
        auto next = std::chrono::steady_clock::now();
        const bool lidar_snapshot =
            terrain_lidar_mode_ == TerrainLidarRuntimeMode::kSnapshot;
        while (!terrain_lidar_stop_.load())
        {
            const auto lock_wait_start = std::chrono::steady_clock::now();
            double lock_wait_s = 0.0;
            double lock_hold_s = 0.0;
            double sim_time = -1.0;
            {
                auto sim_lock = LockSimulation();
                const auto lock_acquired = std::chrono::steady_clock::now();
                lock_wait_s = std::chrono::duration<double>(
                    lock_acquired - lock_wait_start).count();
                if (mj_data_ != nullptr)
                {
                    mju_copy(sensor_data->qpos, mj_data_->qpos, sensor_model->nq);
                    mju_copy(sensor_data->qvel, mj_data_->qvel, sensor_model->nv);
                    sensor_data->time = mj_data_->time;
                    sim_time = sensor_data->time;
                }
                lock_hold_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - lock_acquired).count();
            }
            const auto lidar_operation_start = std::chrono::steady_clock::now();
            double publish_s = -1.0;
            bool published = false;
            if (sim_time >= 0.0 && !lidar_snapshot)
            {
                mj_fwdPosition(sensor_model, sensor_data);
                published = PublishLidarHeightMap(
                    sensor_model, sensor_data, &publish_s);
            }
            const double lidar_operation_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - lidar_operation_start).count();
            LogRuntimeTelemetry(
                lidar_snapshot ? "lidar_snapshot" : "lidar", sim_time,
                sim_time >= 0.0
                    ? static_cast<std::uint64_t>(std::llround(sim_time * 1000.0))
                    : 0,
                lock_wait_s, lock_hold_s, lidar_operation_s, publish_s,
                published);
            next += std::chrono::milliseconds(50);
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() > next +
                    std::chrono::milliseconds(50))
                next = std::chrono::steady_clock::now();
        }
        mj_deleteData(sensor_data);
        mj_deleteModel(sensor_model);
    }

public:

    void PublishEnvironmentHeightMap()
    {
        if (!param::config.terrain_lidar)
            return;
        constexpr float kResolution = 0.10f;
        constexpr uint32_t kWidth = 16;
        constexpr uint32_t kHeight = 16;
        constexpr float kOriginX = -0.20f;
        constexpr float kOriginY = -0.80f;
        const double sim_time = mj_data_->time;
        if (!environment_heightmap ||
            sim_time - last_environment_map_publish_s_ < 0.020)
            return;
        const int base_body_id = mj_name2id(
            mj_model_, mjOBJ_BODY, "base_link");
        if (base_body_id < 0)
            return;
        unitree_go::msg::dds_::HeightMap_ map;
        map.stamp(sim_time);
        map.frame_id("base_link");
        map.resolution(kResolution);
        map.width(kWidth);
        map.height(kHeight);
        map.origin() = {kOriginX, kOriginY};
        map.data().assign(
            static_cast<std::size_t>(kWidth) * kHeight, 0.0f);
        const mjtNum *base_pos = mj_data_->xpos + 3 * base_body_id;
        const mjtNum *base_mat = mj_data_->xmat + 9 * base_body_id;
        for (int geom_id = 0; geom_id < mj_model_->ngeom; ++geom_id)
        {
            if (mj_model_->geom_bodyid[geom_id] != 0 ||
                mj_model_->geom_type[geom_id] == mjGEOM_PLANE ||
                (mj_model_->geom_contype[geom_id] == 0 &&
                 mj_model_->geom_conaffinity[geom_id] == 0))
                continue;
            double footprint_radius = 0.0;
            double half_height = 0.0;
            const int type = mj_model_->geom_type[geom_id];
            const mjtNum *size = mj_model_->geom_size + 3 * geom_id;
            if (type == mjGEOM_BOX)
            {
                footprint_radius = std::hypot(size[0], size[1]);
                half_height = size[2];
            }
            else if (type == mjGEOM_CYLINDER)
            {
                footprint_radius = size[0];
                half_height = size[1];
            }
            else if (type == mjGEOM_CAPSULE)
            {
                footprint_radius = size[0];
                half_height = size[1] + size[0];
            }
            else if (type == mjGEOM_SPHERE)
            {
                footprint_radius = size[0];
                half_height = size[0];
            }
            else
            {
                footprint_radius = std::max(size[0], size[1]);
                half_height = std::max(size[0], size[2]);
            }
            if (!(footprint_radius > 0.0) || !(half_height > 0.0))
                continue;
            const mjtNum *geom_pos = mj_data_->geom_xpos + 3 * geom_id;
            const mjtNum *geom_delta = geom_pos;
            mjtNum world_delta[3] = {
                geom_delta[0] - base_pos[0],
                geom_delta[1] - base_pos[1],
                geom_delta[2] - base_pos[2]};
            mjtNum local[3] = {0.0, 0.0, 0.0};
            mju_mulMatTVec(local, base_mat, world_delta, 3, 3);
            const double top = geom_pos[2] + half_height;
            for (uint32_t iy = 0; iy < kHeight; ++iy)
            {
                const double y = kOriginY +
                    (static_cast<double>(iy) + 0.5) * kResolution;
                for (uint32_t ix = 0; ix < kWidth; ++ix)
                {
                    const double x = kOriginX +
                        (static_cast<double>(ix) + 0.5) * kResolution;
                    if (std::hypot(x - local[0], y - local[1]) >
                        footprint_radius + 0.5 * kResolution)
                        continue;
                    const std::size_t index =
                        static_cast<std::size_t>(iy) * kWidth + ix;
                    map.data()[index] = std::max(
                        map.data()[index], static_cast<float>(top));
                }
            }
        }
        (void)environment_heightmap->Write(map, 0);
        last_environment_map_publish_s_ = sim_time;
    }

    // Sensor-only local elevation map.  Rays are cast through the MuJoCo
    // scene so occlusion is real, but the controller receives only this
    // lidar-derived observation.  Heights are expressed relative to
    // base_link; unknown cells remain NaN and are never filled by the oracle.
    bool PublishLidarHeightMap(
        const mjModel *sensor_model,
        mjData *sensor_data,
        double *publish_duration_s)
    {
        if (!param::config.terrain_lidar || !lidar_heightmap ||
            sensor_model == nullptr)
            return false;
        if (sensor_data == nullptr)
            return false;
        const double sim_time = sensor_data->time;
        if (sim_time - last_lidar_map_publish_s_ < kLidarPublishPeriodS)
            return false;
        const int base_body_id = mj_name2id(
            sensor_model, mjOBJ_BODY, "base_link");
        if (base_body_id < 0)
            return false;
        last_lidar_map_publish_s_ = sim_time;
        const mjtNum *base_pos = sensor_data->xpos + 3 * base_body_id;
        const mjtNum *base_mat = sensor_data->xmat + 9 * base_body_id;
        struct Ring { double elevation_deg; int count; };
        static constexpr Ring rings[] = {
            {-15.0, 48}, {-25.0, 48}, {-35.0, 36}, {-45.0, 36},
            {-55.0, 24}, {-65.0, 24}, {-75.0, 16}, {-5.0, 16},
            {5.0, 16}};
        const double origin[3] = {
            base_pos[0] + 0.15 * base_mat[0],
            base_pos[1] + 0.15 * base_mat[3],
            base_pos[2] + 0.05};
        int geom_id_out[1] = {-1};
        for (const Ring &ring : rings)
        {
            const double elevation = ring.elevation_deg * M_PI / 180.0;
            const bool forward_only = std::abs(ring.elevation_deg) < 10.0;
            for (int i = 0; i < ring.count; ++i)
            {
                const double azimuth = forward_only
                    ? (-40.0 + 80.0 * i /
                       std::max(1, ring.count - 1)) * M_PI / 180.0
                    : 2.0 * M_PI * i / ring.count;
                const mjtNum direction_body[3] = {
                    std::cos(elevation) * std::cos(azimuth),
                    std::cos(elevation) * std::sin(azimuth),
                    std::sin(elevation)};
                mjtNum direction_world[3];
                mju_mulMatVec(direction_world, base_mat, direction_body, 3, 3);
                mjtNum ray_origin[3] = {origin[0], origin[1], origin[2]};
                bool accepted = false;
                mjtNum distance = -1.0;
                for (int bounce = 0; bounce < 3; ++bounce)
                {
                    distance = mj_ray(
                        sensor_model, sensor_data, ray_origin, direction_world,
                        nullptr, 1, base_body_id, geom_id_out);
                    if (distance < 0.0 || geom_id_out[0] < 0)
                        break;
                    const int geom_id = geom_id_out[0];
                    const bool skip =
                        sensor_model->geom_bodyid[geom_id] != 0 ||
                        (sensor_model->geom_contype[geom_id] == 0 &&
                         sensor_model->geom_conaffinity[geom_id] == 0);
                    if (!skip)
                    {
                        accepted = true;
                        break;
                    }
                    ray_origin[0] += direction_world[0] * (distance + 0.01);
                    ray_origin[1] += direction_world[1] * (distance + 0.01);
                    ray_origin[2] += direction_world[2] * (distance + 0.01);
                }
                if (!accepted)
                    continue;
                const double hit_x = ray_origin[0] +
                    distance * direction_world[0];
                const double hit_y = ray_origin[1] +
                    distance * direction_world[1];
                const double hit_z = ray_origin[2] +
                    distance * direction_world[2];
                const int ix = static_cast<int>(std::floor(
                    (hit_x - kLidarWorldOriginX) / kLidarWorldResolution));
                const int iy = static_cast<int>(std::floor(
                    (hit_y - kLidarWorldOriginY) / kLidarWorldResolution));
                if (ix < 0 || ix >= kLidarWorldWidth ||
                    iy < 0 || iy >= kLidarWorldHeight)
                    continue;
                const std::size_t index = static_cast<std::size_t>(iy) *
                    kLidarWorldWidth + static_cast<std::size_t>(ix);
                if (!std::isfinite(lidar_world_z_[index]) ||
                    hit_z < lidar_world_z_[index])
                    lidar_world_z_[index] = hit_z;
                lidar_world_t_[index] = sim_time;
            }
        }

        unitree_go::msg::dds_::HeightMap_ map;
        map.stamp(sim_time);
        map.frame_id("base_link");
        map.resolution(kLidarWindowResolution);
        map.width(kLidarWindowWidth);
        map.height(kLidarWindowHeight);
        map.origin() = {kLidarWindowOriginX, kLidarWindowOriginY};
        map.data().assign(kLidarWindowCellCount,
                          std::numeric_limits<float>::quiet_NaN());
        std::vector<double> direct_world_z(
            kLidarWindowCellCount,
            std::numeric_limits<double>::quiet_NaN());
        std::vector<double> direct_observation_stamps(
            kLidarWindowCellCount,
            std::numeric_limits<double>::quiet_NaN());
        const double yaw = std::atan2(base_mat[3], base_mat[0]);
        const double c = std::cos(yaw), s = std::sin(yaw);
        // Dense downward elevation sweep over the published local window.
        // It remains a sensor-derived ray observation through the scene.
        // Occluded or missed rays leave the corresponding cells unknown.
        for (uint32_t iy = 0; iy < kLidarWindowHeight; ++iy)
        {
            for (uint32_t ix = 0; ix < kLidarWindowWidth; ++ix)
            {
                const double local_x = kLidarWindowOriginX +
                    (static_cast<double>(ix) + 0.5) * kLidarWindowResolution;
                const double local_y = kLidarWindowOriginY +
                    (static_cast<double>(iy) + 0.5) * kLidarWindowResolution;
                const double world_x = base_pos[0] + c * local_x - s * local_y;
                const double world_y = base_pos[1] + s * local_x + c * local_y;
                mjtNum ray_origin[3] = {world_x, world_y,
                    base_pos[2] + 1.0};
                const mjtNum direction[3] = {0.0, 0.0, -1.0};
                bool accepted = false;
                mjtNum distance = -1.0;
                for (int bounce = 0; bounce < 16; ++bounce)
                {
                    distance = mj_ray(
                        sensor_model, sensor_data, ray_origin, direction,
                        nullptr, 1, base_body_id, geom_id_out);
                    if (distance < 0.0 || geom_id_out[0] < 0)
                        break;
                    const int geom_id = geom_id_out[0];
                    if (sensor_model->geom_bodyid[geom_id] == 0 &&
                        (sensor_model->geom_contype[geom_id] != 0 ||
                         sensor_model->geom_conaffinity[geom_id] != 0))
                    {
                        accepted = true;
                        break;
                    }
                    ray_origin[2] -= distance + 0.01;
                }
                if (!accepted)
                    continue;
                const double hit_z = ray_origin[2] - distance;
                const std::size_t direct_index =
                    static_cast<std::size_t>(iy) * kLidarWindowWidth + ix;
                direct_world_z[direct_index] = hit_z;
                direct_observation_stamps[direct_index] = sim_time;
                const int gx = static_cast<int>(std::floor(
                    (world_x - kLidarWorldOriginX) / kLidarWorldResolution));
                const int gy = static_cast<int>(std::floor(
                    (world_y - kLidarWorldOriginY) / kLidarWorldResolution));
                if (gx < 0 || gx >= kLidarWorldWidth ||
                    gy < 0 || gy >= kLidarWorldHeight)
                    continue;
                const std::size_t index = static_cast<std::size_t>(gy) *
                    kLidarWorldWidth + static_cast<std::size_t>(gx);
                if (!std::isfinite(lidar_world_z_[index]) ||
                    hit_z < lidar_world_z_[index])
                    lidar_world_z_[index] = hit_z;
                lidar_world_t_[index] = sim_time;
            }
        }
        for (uint32_t iy = 0; iy < kLidarWindowHeight; ++iy)
        {
            for (uint32_t ix = 0; ix < kLidarWindowWidth; ++ix)
            {
                const double local_x = kLidarWindowOriginX +
                    (static_cast<double>(ix) + 0.5) * kLidarWindowResolution;
                const double local_y = kLidarWindowOriginY +
                    (static_cast<double>(iy) + 0.5) * kLidarWindowResolution;
                const double world_x = base_pos[0] + c * local_x - s * local_y;
                const double world_y = base_pos[1] + s * local_x + c * local_y;
                const int gx = static_cast<int>(std::floor(
                    (world_x - kLidarWorldOriginX) / kLidarWorldResolution));
                const int gy = static_cast<int>(std::floor(
                    (world_y - kLidarWorldOriginY) / kLidarWorldResolution));
                if (gx < 0 || gx >= kLidarWorldWidth ||
                    gy < 0 || gy >= kLidarWorldHeight)
                    continue;
                const std::size_t index = static_cast<std::size_t>(gy) *
                    kLidarWorldWidth + static_cast<std::size_t>(gx);
                if (sim_time - lidar_world_t_[index] > kLidarMemoryS)
                    continue;
                const double world_z = lidar_world_z_[index];
                if (std::isfinite(world_z))
                {
                    map.data()[static_cast<std::size_t>(iy) *
                               kLidarWindowWidth + ix] = static_cast<float>(
                                   world_z - base_pos[2]);
                }
            }
        }
        const auto publish_start = std::chrono::steady_clock::now();
        if (lidar_heightmap_envelope)
        {
            unitree_go::msg::dds_::HeightMap_ snapshot_map = map;
            for (std::size_t i = 0; i < kLidarWindowCellCount; ++i)
            {
                if (std::isfinite(direct_world_z[i]))
                    snapshot_map.data()[i] = static_cast<float>(
                        direct_world_z[i] - base_pos[2]);
                else
                    snapshot_map.data()[i] =
                        std::numeric_limits<float>::quiet_NaN();
            }
            go2_terrain::TerrainMapEnvelope envelope;
            const std::array<double, 3> capture_position{
                base_pos[0], base_pos[1], base_pos[2]};
            if (go2_terrain::TerrainMapEnvelopeFromHeightMap(
                    snapshot_map, ++lidar_map_sequence_, capture_position, yaw,
                    direct_observation_stamps, envelope))
            {
                std::string wire;
                if (go2_terrain::SerializeTerrainMapEnvelope(envelope, wire))
                {
                    std_msgs::msg::dds_::String_ message;
                    message.data(wire);
                    const bool sent = lidar_heightmap_envelope->Write(message, 0);
                    if (lidar_map_sequence_ <= 3)
                        std::cout << "Terrain envelope publish seq=" << lidar_map_sequence_
                                  << " bytes=" << wire.size() << " sent=" << sent
                                  << " stamp=" << sim_time << "\n";
                }
            }
        }
        (void)lidar_heightmap->Write(map, 0);
        if (publish_duration_s != nullptr)
        {
            *publish_duration_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - publish_start).count();
        }
        return true;
    }

    void InitRuntimeTelemetry()
    {
        const char *path = std::getenv("TROT_WALLCLOCK_TELEMETRY_PATH");
        if (path == nullptr || path[0] == 0)
            return;
        runtime_telemetry_.open(path);
        if (!runtime_telemetry_)
        {
            std::cerr << "Failed to open wall-clock telemetry: " << path
                      << "\n";
            return;
        }
        runtime_telemetry_ << "event,wall_time_s,sim_time_s,state_tick"
                           << ",lidar_lock_wait_s,lidar_lock_hold_s"
                           << ",lidar_operation_s,lidar_publish_s"
                           << ",lidar_published\n";
        runtime_telemetry_ << std::fixed << std::setprecision(9);
        runtime_telemetry_.flush();
    }

    void LogRuntimeTelemetry(
        const char *event,
        double sim_time_s,
        std::uint64_t state_tick,
        double lock_wait_s,
        double lock_hold_s,
        double operation_s,
        double publish_s,
        bool published)
    {
        if (!runtime_telemetry_)
            return;
        std::lock_guard<std::mutex> lock(runtime_telemetry_mutex_);
        runtime_telemetry_
            << event << ","
            << std::chrono::duration<double>(
                   std::chrono::steady_clock::now() -
                   runtime_telemetry_start_)
                   .count()
            << "," << sim_time_s
            << "," << state_tick
            << "," << lock_wait_s
            << "," << lock_hold_s
            << "," << operation_s
            << "," << publish_s
            << "," << (published ? 1 : 0)
            << "\n";
    }

    virtual void run()
    {
        static thread_local bool affinity_initialized = false;
        if (!affinity_initialized)
        {
            PinCurrentThreadToEnv("TROT_SIM_BRIDGE_CPU");
            affinity_initialized = true;
        }
        if (param::config.lockstep && g_lockstep != nullptr)
        {
            RunLockstep();
            return;
        }
        RunWallClock();
    }

    // Wall-clock path: unchanged from the accepted Phase-1 bridge loop.
    void RunWallClock()
    {
        auto sim_lock = LockSimulation();
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        ApplyLatestCommand();
        PublishStateSnapshot(/*blocking_lowstate=*/false);
    }

    // Lockstep path (Order-103/105/106): the frozen physics state is
    // republished at the 1000 Hz bridge rate until the causal exchange for
    // it completes (new LowCmd after the first publish + matching
    // ack{state_seq}), then physics steps exactly once. Startup (before the
    // ready barrier) is identical to the wall-clock path; the frozen
    // discipline starts at the handoff tick. Each publish registers its
    // monotonic tick as the ack state_seq side-channel before the LowState
    // is sent; repeated publishes of the same frozen state keep the same
    // state_seq. The sim mutex is held across the publish so the tick read
    // and the consumed step-completed flag always refer to the same state
    // (NotifyStepCompleted runs inside the physics lock).
    void RunLockstep()
    {
        if (!g_lockstep->BarrierComplete())
        {
            RunWallClock();
            g_lockstep->OnStartupPublish(CurrentTickMs());
            return;
        }
        auto sim_lock = LockSimulation();
        if (!mj_data_) return;
        if (g_lockstep->FailedClosed()) return;
        const std::uint64_t sim_tick_ms = CurrentTickMs();
        const lockstep::PublishOutcome outcome =
            g_lockstep->OnPublish(sim_tick_ms);
        PublishStateSnapshot(/*blocking_lowstate=*/true);
        if (outcome == lockstep::PublishOutcome::kStepGranted)
        {
            ApplyLatestCommand();
            g_lockstep->NotifyCommandApplied();
        }
    }

    std::uint64_t CurrentTickMs() const
    {
        return static_cast<std::uint64_t>(
            std::llround(mj_data_->time * 1000.0));
    }

    void ApplyLatestCommand()
    {
        auto sim_lock = LockSimulation();
        if (!mj_data_) return;
        std::lock_guard<std::mutex> lock(lowcmd->mutex_);
        for(int i(0); i<num_motor_; i++) {
            auto & m = lowcmd->msg_.motor_cmd()[i];
            mj_data_->ctrl[i] = m.tau() +
                                m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }

    void PublishStateSnapshot(bool blocking_lowstate)
    {
        auto sim_lock = LockSimulation();
        if (!mj_data_) return;
        const bool lowstate_locked =
            blocking_lowstate ? (lowstate->lock(), true) : lowstate->trylock();
        // lowstate
        if(lowstate_locked) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }

            if constexpr (std::is_same_v<
                              std::decay_t<decltype(lowstate->msg_)>,
                              unitree_go::msg::dds_::LowState_>) {
                for(std::size_t i = 0; i < foot_force_adr_.size(); ++i) {
                    if(foot_force_adr_[i] >= 0) {
                        const double force = std::clamp(
                            mj_data_->sensordata[foot_force_adr_[i]],
                            0.0,
                            static_cast<double>(std::numeric_limits<int16_t>::max()));
                        const int16_t force_value =
                            static_cast<int16_t>(std::lround(force));
                        lowstate->msg_.foot_force()[i] = force_value;
                        lowstate->msg_.foot_force_est()[i] = force_value;
                    }
                }
            }
            
            // [动力学槽位] motor_state[12..17] 的 7 个 float 字段:
            // slot 0-35 = 基座质量矩阵 6x6(qM 展开), slot 36-41 = base qfrc_bias(6)
            if constexpr (std::is_same_v<
                              std::decay_t<decltype(lowstate->msg_)>,
                              unitree_go::msg::dds_::LowState_>) {
                static thread_local std::vector<mjtNum> full_mass(
                    static_cast<std::size_t>(mj_model_->nv) *
                    static_cast<std::size_t>(mj_model_->nv), 0.0);
                mj_fullM(mj_model_, full_mass.data(), mj_data_->qM);
                for (int slot = 0; slot < 42; ++slot) {
                    const int motor = 12 + slot / 7;
                    const int field = slot % 7;
                    double value = 0.0;
                    if (slot < 36) {
                        const int r = slot / 6;
                        const int c = slot % 6;
                        value = full_mass[static_cast<std::size_t>(
                            r * mj_model_->nv + c)];
                    } else {
                        value = mj_data_->qfrc_bias[slot - 36];
                    }
                    auto &ms = lowstate->msg_.motor_state()[motor];
                    switch (field) {
                        case 0: ms.q() = static_cast<float>(value); break;
                        case 1: ms.dq() = static_cast<float>(value); break;
                        case 2: ms.ddq() = static_cast<float>(value); break;
                        case 3: ms.tau_est() = static_cast<float>(value); break;
                        case 4: ms.q_raw() = static_cast<float>(value); break;
                        case 5: ms.dq_raw() = static_cast<float>(value); break;
                        case 6: ms.ddq_raw() = static_cast<float>(value); break;
                    }
                }
            }

            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    unitree::robot::ChannelPtr<unitree_go::msg::dds_::HeightMap_> environment_heightmap;
    unitree::robot::ChannelPtr<unitree_go::msg::dds_::HeightMap_> lidar_heightmap;
    unitree::robot::ChannelPtr<std_msgs::msg::dds_::String_>
        lidar_heightmap_envelope;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<unitree::robot::SubscriptionBase<typename LowCmd_t::MsgType>> lowcmd;
    std::shared_ptr<LockstepAckSubscriber> lockstep_ack_subscriber_;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    static constexpr float kLidarWorldResolution = 0.05f;
    static constexpr int kLidarWorldWidth = 440;
    static constexpr int kLidarWorldHeight = 80;
    static constexpr float kLidarWorldOriginX = -2.0f;
    static constexpr float kLidarWorldOriginY = -2.0f;
    static constexpr uint32_t kLidarWindowWidth = 32;
    static constexpr uint32_t kLidarWindowHeight = 10;
    static constexpr std::size_t kLidarWindowCellCount =
        static_cast<std::size_t>(kLidarWindowWidth) * kLidarWindowHeight;
    static constexpr float kLidarWindowResolution = 0.05f;
    static constexpr float kLidarWindowOriginX = -0.45f;
    static constexpr float kLidarWindowOriginY = -0.225f;
    static constexpr double kLidarMemoryS = 1.5;
    static constexpr double kLidarPublishPeriodS = 0.050;
    static constexpr std::size_t kLidarWorldCellCount =
        static_cast<std::size_t>(kLidarWorldWidth) * kLidarWorldHeight;
    std::vector<double> lidar_world_z_;
    std::vector<double> lidar_world_t_;
    double last_lidar_map_publish_s_ = -1.0e9;
    std::uint64_t lidar_map_sequence_ = 0;
    double last_environment_map_publish_s_ = -1.0e9;
    std::ofstream runtime_telemetry_;
    std::mutex runtime_telemetry_mutex_;
    std::chrono::steady_clock::time_point runtime_telemetry_start_ =
        std::chrono::steady_clock::now();
    unitree::common::RecurrentThreadPtr thread_;
    std::atomic<bool> terrain_lidar_stop_{false};
    std::thread terrain_lidar_thread_;
    TerrainLidarRuntimeMode terrain_lidar_mode_ =
        TerrainLidarRuntimeMode::kFull;
    std::mutex terrain_lidar_park_mutex_;
    std::condition_variable terrain_lidar_park_cv_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(
        mjModel *model,
        mjData *data,
        std::recursive_mutex *sim_mutex)
        : RobotBridge(model, data, sim_mutex)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();
        auto sim_lock = LockSimulation();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
