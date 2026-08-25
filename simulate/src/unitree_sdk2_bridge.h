#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <mutex>
#include <type_traits>

#include "param.h"
#include "physics_joystick.h"

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

    // Sensor data indices
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

public:
    RobotBridge(
        mjModel *model,
        mjData *data,
        std::recursive_mutex *sim_mutex)
        : UnitreeSDK2BridgeBase(model, data, sim_mutex)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        environment_heightmap = unitree::robot::ChannelFactory::Instance()
            ->CreateSendChannel<unitree_go::msg::dds_::HeightMap_>(
                "rt/go2/environment_heightmap");
        lidar_heightmap = unitree::robot::ChannelFactory::Instance()
            ->CreateSendChannel<unitree_go::msg::dds_::HeightMap_>(
                "rt/go2/lidar_heightmap");
        lidar_world_z_.assign(kLidarWorldCellCount,
                              std::numeric_limits<double>::quiet_NaN());
        lidar_world_t_.assign(kLidarWorldCellCount, -1.0e9);
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    void PublishEnvironmentHeightMap()
    {
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
    void PublishLidarHeightMap()
    {
        if (!lidar_heightmap)
            return;
        const double sim_time = mj_data_->time;
        if (sim_time - last_lidar_map_publish_s_ < 0.020)
            return;
        const int base_body_id = mj_name2id(
            mj_model_, mjOBJ_BODY, "base_link");
        if (base_body_id < 0)
            return;
        last_lidar_map_publish_s_ = sim_time;
        const mjtNum *base_pos = mj_data_->xpos + 3 * base_body_id;
        const mjtNum *base_mat = mj_data_->xmat + 9 * base_body_id;
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
                        mj_model_, mj_data_, ray_origin, direction_world,
                        nullptr, 1, base_body_id, geom_id_out);
                    if (distance < 0.0 || geom_id_out[0] < 0)
                        break;
                    const int geom_id = geom_id_out[0];
                    const bool skip =
                        mj_model_->geom_bodyid[geom_id] != 0 ||
                        (mj_model_->geom_contype[geom_id] == 0 &&
                         mj_model_->geom_conaffinity[geom_id] == 0);
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
        const double yaw = std::atan2(base_mat[3], base_mat[0]);
        const double c = std::cos(yaw), s = std::sin(yaw);
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
                    map.data()[static_cast<std::size_t>(iy) *
                               kLidarWindowWidth + ix] = static_cast<float>(
                                   world_z - base_pos[2]);
            }
        }
        (void)lidar_heightmap->Write(map, 0);
    }

    virtual void run()
    {
        auto sim_lock = LockSimulation();
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        PublishEnvironmentHeightMap();
        PublishLidarHeightMap();

        // lowstate
        if(lowstate->trylock()) {
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
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    static constexpr float kLidarWorldResolution = 0.05f;
    static constexpr int kLidarWorldWidth = 440;
    static constexpr int kLidarWorldHeight = 80;
    static constexpr float kLidarWorldOriginX = -2.0f;
    static constexpr float kLidarWorldOriginY = -2.0f;
    static constexpr uint32_t kLidarWindowWidth = 20;
    static constexpr uint32_t kLidarWindowHeight = 16;
    static constexpr std::size_t kLidarWindowCellCount =
        static_cast<std::size_t>(kLidarWindowWidth) * kLidarWindowHeight;
    static constexpr float kLidarWindowResolution = 0.05f;
    static constexpr float kLidarWindowOriginX = -0.10f;
    static constexpr float kLidarWindowOriginY = -0.40f;
    static constexpr double kLidarMemoryS = 1.5;
    static constexpr std::size_t kLidarWorldCellCount =
        static_cast<std::size_t>(kLidarWorldWidth) * kLidarWorldHeight;
    std::vector<double> lidar_world_z_;
    std::vector<double> lidar_world_t_;
    double last_lidar_map_publish_s_ = -1.0e9;
    double last_environment_map_publish_s_ = -1.0e9;
    unitree::common::RecurrentThreadPtr thread_;
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
