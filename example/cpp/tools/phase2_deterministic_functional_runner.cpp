// Deterministic, single-threaded Phase 2 functional verification runner.
//
// This executable deliberately does not use DDS, wall-clock sleeps, callbacks,
// or worker threads. One simulation tick performs one control update followed
// by exactly one mj_step. The terrain path is exercised as a sensor-only
// observer and its plan is never consumed by control.

#include <mujoco/mujoco.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include "go2_forward_kinematics.h"
#include "go2_inverse_kinematics.h"
#include "go2_rigid_body.h"
#include "inverse_dynamics_wbc.h"
#include "locomotion_kernel.h"
#include "raibert_trot_kernel.h"
#include "srbd_mpc.h"
#include "terrain_model.h"
#include "terrain_planner.h"
#include "velocity_command.h"
#include "velocity_filter.h"

namespace
{
constexpr int kMotorCount = 12;
constexpr int kGo2Nq = 19;
constexpr int kGo2Nv = 18;
constexpr double kDt = 0.002;
constexpr double kStandUpDurationS = 3.0;
constexpr double kStandSettleDurationS = 0.5;
constexpr double kStandDurationS =
    kStandUpDurationS + kStandSettleDurationS;
constexpr double kMpcDtS = 0.05;
constexpr int kMpcHorizon = 10;
constexpr int kMpcPeriodTicks = 5;
constexpr double kStandKp = 100.0;
constexpr double kStandKd = 3.5;
constexpr double kSwingKp = 80.0;
constexpr double kSwingKd = 4.5;
constexpr double kFullStanceKp = 25.0;
constexpr double kFullStanceKd = 2.0;
constexpr double kTauLimitNm = 35.0;

struct Options
{
    std::string profile_path =
        "example/cpp/configs/phase1_velocity_accel_1_to_3.csv";
    std::string scene_path =
        "unitree_robots/go2/scene_leg_lift_demo.xml";
    std::string model_path = "unitree_robots/go2/go2.xml";
    std::string output_dir = "phase2_determinism_run";
    std::string mode = "phase1";
    double duration_s = 40.0;
    double stand_duration_s = kStandDurationS;
    std::uint64_t seed = 1;
};

struct MotorAddress
{
    std::array<int, kMotorCount> qpos{};
    std::array<int, kMotorCount> dof{};
    std::array<int, kMotorCount> actuator{};
};

struct TerrainTelemetry
{
    std::uint64_t lidar_sequence = 0;
    std::uint64_t lidar_sim_tick = 0;
    std::uint64_t map_epoch = 0;
    std::uint64_t plan_epoch = 0;
    std::uint64_t control_generation = 0;
    double lidar_wall_age_s = 0.0;
    go2_terrain::TerrainPlanStatus plan_status =
        go2_terrain::TerrainPlanStatus::kEmpty;
    go2_terrain::TerrainPlanFailure plan_failure =
        go2_terrain::TerrainPlanFailure::kNone;
    double planner_elapsed_us = 0.0;
    bool planner_deadline_miss = false;
};

struct WbcTelemetry
{
    bool srbd_ok = false;
    double srbd_force_x_n = 0.0;
    double srbd_force_z_n = 0.0;
    double srbd_acc_x_mps2 = 0.0;
    double srbd_acc_z_mps2 = 0.0;
    double srbd_ang_acc_y_radps2 = 0.0;
    bool id_wbc_ok = false;
    double id_eq_residual = 0.0;
    double id_qdd_base_x_mps2 = 0.0;
    double id_force_x_n = 0.0;
    int torque_saturation_count = 0;
    double torque_max_abs_nm = 0.0;
    std::array<double, kMotorCount> tau{};
};

void Usage()
{
    std::cerr
        << "usage: phase2_deterministic_functional_runner"
        << " --profile PATH --mode phase1|terrain_sensor_only"
        << " --duration SEC --output DIR [--scene PATH] [--model PATH]"
        << " [--stand-duration SEC] [--seed N]\n";
}

bool ParseDouble(const std::string &value, double &out)
{
    try
    {
        std::size_t consumed = 0;
        out = std::stod(value, &consumed);
        return consumed == value.size() && std::isfinite(out);
    }
    catch (...)
    {
        return false;
    }
}

bool ParseUint64(const std::string &value, std::uint64_t &out)
{
    try
    {
        std::size_t consumed = 0;
        out = std::stoull(value, &consumed);
        return consumed == value.size();
    }
    catch (...)
    {
        return false;
    }
}

bool ParseOptions(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        auto require_value = [&](const char *name, std::string &value) {
            if (i + 1 >= argc)
            {
                std::cerr << name << " requires a value\n";
                return false;
            }
            value = argv[++i];
            return true;
        };
        if (arg == "--help" || arg == "-h")
        {
            Usage();
            return false;
        }
        if (arg == "--profile")
        {
            if (!require_value("--profile", options.profile_path))
                return false;
        }
        else if (arg == "--scene")
        {
            if (!require_value("--scene", options.scene_path))
                return false;
        }
        else if (arg == "--model")
        {
            if (!require_value("--model", options.model_path))
                return false;
        }
        else if (arg == "--output")
        {
            if (!require_value("--output", options.output_dir))
                return false;
        }
        else if (arg == "--mode")
        {
            if (!require_value("--mode", options.mode))
                return false;
            if (options.mode != "phase1" &&
                options.mode != "terrain_sensor_only")
            {
                std::cerr << "mode must be phase1 or terrain_sensor_only\n";
                return false;
            }
        }
        else if (arg == "--duration")
        {
            std::string value;
            if (!require_value("--duration", value) ||
                !ParseDouble(value, options.duration_s) ||
                !(options.duration_s > 0.0))
            {
                std::cerr << "duration must be a positive finite number\n";
                return false;
            }
        }
        else if (arg == "--stand-duration")
        {
            std::string value;
            if (!require_value("--stand-duration", value) ||
                !ParseDouble(value, options.stand_duration_s) ||
                !(options.stand_duration_s >= 0.0))
            {
                std::cerr << "stand-duration must be a nonnegative finite number\n";
                return false;
            }
        }
        else if (arg == "--seed")
        {
            std::string value;
            if (!require_value("--seed", value) ||
                !ParseUint64(value, options.seed))
            {
                std::cerr << "seed must be an unsigned integer\n";
                return false;
            }
        }
        else
        {
            std::cerr << "unknown argument: " << arg << "\n";
            Usage();
            return false;
        }
    }
    return true;
}

bool ResolveMotorAddress(const mjModel *model, MotorAddress &address)
{
    static constexpr std::array<const char *, kMotorCount> kMotorNames = {
        "FR_hip", "FR_thigh", "FR_calf",
        "FL_hip", "FL_thigh", "FL_calf",
        "RR_hip", "RR_thigh", "RR_calf",
        "RL_hip", "RL_thigh", "RL_calf"};
    if (model == nullptr || model->nq != kGo2Nq || model->nv != kGo2Nv ||
        model->nu < kMotorCount)
        return false;
    for (int motor = 0; motor < kMotorCount; ++motor)
    {
        const int joint = mj_name2id(
            model, mjOBJ_JOINT, go2_control::Go2MotorJointName(motor));
        const int actuator = mj_name2id(
            model, mjOBJ_ACTUATOR, kMotorNames[motor]);
        if (joint < 0 || actuator < 0)
            return false;
        address.qpos[motor] = model->jnt_qposadr[joint];
        address.dof[motor] = model->jnt_dofadr[joint];
        address.actuator[motor] = actuator;
    }
    return true;
}

std::array<double, kMotorCount> StandJointPositions()
{
    return {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};
}

std::array<double, kMotorCount> ReadJointPositions(
    const mjData *data, const MotorAddress &address)
{
    std::array<double, kMotorCount> values{};
    for (int motor = 0; motor < kMotorCount; ++motor)
        values[motor] = data->qpos[address.qpos[motor]];
    return values;
}

std::array<double, kMotorCount> ReadJointVelocities(
    const mjData *data, const MotorAddress &address)
{
    std::array<double, kMotorCount> values{};
    for (int motor = 0; motor < kMotorCount; ++motor)
        values[motor] = data->qvel[address.dof[motor]];
    return values;
}

go2_control::RigidBodyState MakeRigidBodyState(
    const mjData *data, const MotorAddress &address)
{
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(
        data->qpos[0], data->qpos[1], data->qpos[2]);
    state.quat_world_from_body = Eigen::Quaterniond(
        data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]);
    state.linear_vel_world = Eigen::Vector3d(
        data->qvel[0], data->qvel[1], data->qvel[2]);
    state.angular_vel_body = Eigen::Vector3d(
        data->qvel[3], data->qvel[4], data->qvel[5]);
    for (int motor = 0; motor < kMotorCount; ++motor)
    {
        state.q[motor] = data->qpos[address.qpos[motor]];
        state.dq[motor] = data->qvel[address.dof[motor]];
    }
    return state;
}

std::array<double, 3> EulerFromQuaternion(const mjtNum *q)
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    const double pitch_arg = std::clamp(
        2.0 * (w * y - z * x), -1.0, 1.0);
    return {
        std::atan2(2.0 * (w * x + y * z),
                   1.0 - 2.0 * (x * x + y * y)),
        std::asin(pitch_arg),
        std::atan2(2.0 * (w * z + x * y),
                   1.0 - 2.0 * (y * y + z * z))};
}

Eigen::Matrix3d RotationFromQuaternion(const mjtNum *q)
{
    Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
    return quat.normalized().toRotationMatrix();
}

std::uint32_t MeasuredContactMask(
    const mjData *data,
    const std::array<int, go2::kLegCount> &foot_geoms, int floor_geom)
{
    std::uint32_t mask = 0;
    for (int contact = 0; contact < data->ncon; ++contact)
    {
        const mjContact &candidate = data->contact[contact];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const bool foot_first =
                candidate.geom1 == foot_geoms[leg] &&
                (floor_geom < 0 || candidate.geom2 == floor_geom);
            const bool foot_second =
                candidate.geom2 == foot_geoms[leg] &&
                (floor_geom < 0 || candidate.geom1 == floor_geom);
            if (foot_first || foot_second)
                mask |= 1U << static_cast<unsigned>(leg);
        }
    }
    return mask;
}

std::uint32_t ContactScheduleMask(
    const std::array<std::array<bool, go2::kLegCount>,
                     go2_control::kSrbdMaxHorizon> &contact)
{
    std::uint32_t mask = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (contact[0][leg])
            mask |= 1U << static_cast<unsigned>(leg);
    return mask;
}

go2_control::Vector3 BodyVelocity(
    const std::array<double, 4> &quaternion,
    const Eigen::Vector3d &world_velocity)
{
    go2_control::Vector3 body{};
    if (!go2_control::WorldToBodyVelocity(
            quaternion,
            {world_velocity.x(), world_velocity.y(), world_velocity.z()},
            body))
    {
        return {};
    }
    return body;
}

Eigen::Vector3d ClampVector(const Eigen::Vector3d &value, double limit)
{
    Eigen::Vector3d clamped = value;
    for (int axis = 0; axis < 3; ++axis)
        clamped[axis] = std::clamp(clamped[axis], -limit, limit);
    return clamped;
}

void WriteCsvHeader(std::ofstream &csv)
{
    csv << "simulation_tick,simulation_time_s,physics_sequence,"
           "published_state_sequence,controller_tick,"
           "controller_input_sequence,controller_input_sim_tick,"
           "controller_output_sequence,low_cmd_sequence,"
           "state_age_s,command_age_s,deadline_jitter_s,"
           "requested_v_cmd_mps,shaped_v_cmd_mps,applied_v_cmd_mps,"
           "body_vx_mps,filtered_body_vx_mps,velocity_filter_alpha,"
           "gait_phase,gait_period_s,gait_duty_factor,gait_step_length_m,"
           "planned_contact_mask,measured_contact_mask,"
           "terrain_lidar_sequence,terrain_lidar_sim_tick,"
           "terrain_lidar_wall_age_s,terrain_map_epoch,"
           "terrain_plan_epoch,terrain_control_generation,"
           "terrain_plan_status,terrain_plan_failure,"
           "terrain_planner_elapsed_us,terrain_planner_deadline_miss,"
           "srbd_ok,srbd_first_force_x_n,srbd_first_force_z_n,"
           "srbd_linear_acc_x_mps2,srbd_linear_acc_z_mps2,"
           "srbd_angular_acc_y_radps2,id_wbc_ok,id_eq_residual,"
           "id_qdd_base_x_mps2,id_force_x_n,torque_saturation_count,"
           "torque_max_abs_nm,base_x_m,base_y_m,base_z_m,"
           "base_roll_rad,base_pitch_rad,base_yaw_rad,"
           "base_vx_mps,base_vy_mps,base_vz_mps";
    for (int i = 0; i < kGo2Nq; ++i)
        csv << ",qpos" << i;
    for (int i = 0; i < kGo2Nv; ++i)
        csv << ",qvel" << i;
    for (int i = 0; i < kMotorCount; ++i)
        csv << ",joint_target" << i;
    for (int i = 0; i < kMotorCount; ++i)
        csv << ",joint_tau" << i;
    csv << "\n";
}

void WriteCsvRow(
    std::ofstream &csv, std::uint64_t tick, double sim_time,
    const mjData *data, const std::array<double, kMotorCount> &joint_target,
    const std::array<double, kMotorCount> &joint_tau,
    const go2_trot::VelocityCommandState &command,
    double body_vx, double filtered_body_vx, double filter_alpha,
    double gait_phase, double gait_period, double gait_duty,
    double gait_step, std::uint32_t planned_contact,
    std::uint32_t measured_contact, const TerrainTelemetry &terrain,
    const WbcTelemetry &wbc, const MotorAddress &address)
{
    const auto euler = EulerFromQuaternion(data->qpos + 3);
    csv << tick << "," << std::setprecision(17) << sim_time
        << "," << tick << "," << tick << "," << tick
        << "," << tick << "," << tick << "," << tick << "," << tick
        << ",0,0,0,"
        << command.requested_mps << "," << command.shaped_mps << ","
        << command.applied_mps << "," << body_vx << "," << filtered_body_vx
        << "," << filter_alpha << "," << gait_phase << "," << gait_period
        << "," << gait_duty << "," << gait_step << "," << planned_contact
        << "," << measured_contact << "," << terrain.lidar_sequence << ","
        << terrain.lidar_sim_tick << "," << terrain.lidar_wall_age_s << ","
        << terrain.map_epoch << "," << terrain.plan_epoch << ","
        << terrain.control_generation << ","
        << static_cast<int>(terrain.plan_status) << ","
        << static_cast<int>(terrain.plan_failure) << ","
        << terrain.planner_elapsed_us << ","
        << (terrain.planner_deadline_miss ? 1 : 0) << ","
        << (wbc.srbd_ok ? 1 : 0) << "," << wbc.srbd_force_x_n << ","
        << wbc.srbd_force_z_n << "," << wbc.srbd_acc_x_mps2 << ","
        << wbc.srbd_acc_z_mps2 << "," << wbc.srbd_ang_acc_y_radps2 << ","
        << (wbc.id_wbc_ok ? 1 : 0) << "," << wbc.id_eq_residual << ","
        << wbc.id_qdd_base_x_mps2 << "," << wbc.id_force_x_n << ","
        << wbc.torque_saturation_count << "," << wbc.torque_max_abs_nm
        << "," << data->qpos[0] << "," << data->qpos[1] << ","
        << data->qpos[2] << ","
        << euler[0] << "," << euler[1] << "," << euler[2]
        << "," << data->qvel[0] << "," << data->qvel[1] << ","
        << data->qvel[2];
    for (int i = 0; i < kGo2Nq; ++i)
        csv << "," << data->qpos[i];
    for (int i = 0; i < kGo2Nv; ++i)
        csv << "," << data->qvel[i];
    for (double value : joint_target)
        csv << "," << value;
    for (double value : joint_tau)
        csv << "," << value;
    csv << "\n";
    (void)address;
}

void WriteMetadata(
    const Options &options, const std::filesystem::path &output,
    double total_duration_s)
{
    std::ofstream metadata(output / "run_metadata.txt");
    metadata << "format=phase2-deterministic-functional-v1\n"
             << "mode=" << options.mode << "\n"
             << "seed=" << options.seed << "\n"
             << "profile=" << options.profile_path << "\n"
             << "scene=" << options.scene_path << "\n"
             << "model=" << options.model_path << "\n"
             << "duration_s=" << std::setprecision(17)
             << options.duration_s << "\n"
             << "stand_duration_s=" << options.stand_duration_s << "\n"
             << "total_duration_s=" << total_duration_s << "\n"
             << "dt_s=" << kDt << "\n"
             << "control_schedule=one_control_update_then_one_mj_step_per_tick\n"
             << "threads=0\n"
             << "dds=disabled\n"
             << "wall_clock=disabled\n"
             << "terrain_plan_consumed=0\n"
             << "terrain_planner_measure_realtime_timing=0\n";
}

bool BuildTerrainTelemetry(
    const Options &options, std::uint64_t tick, double sim_time,
    double base_yaw, const Eigen::Vector3d &base_position,
    const Eigen::Vector3d &base_velocity, double base_roll,
    double base_pitch, double base_height, double gait_phase,
    double gait_period, double gait_duty, double commanded_vx,
    const std::array<go2::Vec3, go2::kLegCount> &current_feet,
    const std::array<go2::Vec3, go2::kLegCount> &nominal_feet,
    const std::array<bool, go2::kLegCount> &measured_contact,
    const std::array<std::array<bool, go2::kLegCount>,
                     go2_control::kSrbdMaxHorizon> &planned_contact,
    go2_terrain::TerrainPlanner &planner, std::uint64_t &next_map_epoch,
    std::uint64_t &next_plan_id, TerrainTelemetry &telemetry)
{
    if (options.mode != "terrain_sensor_only" || tick % 25U != 0U)
        return false;

    unitree_go::msg::dds_::HeightMap_ message;
    message.stamp(sim_time);
    message.frame_id("base_link");
    message.resolution(0.05f);
    message.width(20);
    message.height(16);
    message.origin() = {-0.10f, -0.40f};
    message.data().assign(20U * 16U, 0.0f);

    const std::uint64_t map_epoch = ++next_map_epoch;
    const auto model = go2_terrain::BuildTerrainModel(
        &message, sim_time, map_epoch, go2_terrain::TerrainSource::kLidar);
    telemetry.lidar_sequence = map_epoch;
    telemetry.lidar_sim_tick = tick;
    telemetry.map_epoch = model.model.epoch;
    telemetry.lidar_wall_age_s = 0.0;
    if (!model.ok())
    {
        telemetry.plan_status = go2_terrain::TerrainPlanStatus::kRejected;
        telemetry.plan_failure =
            go2_terrain::TerrainPlanFailure::kInvalidInput;
        return true;
    }

    go2_terrain::TerrainPlannerInput input;
    input.terrain = &model.model;
    input.state_stamp_s = sim_time;
    input.base_yaw_rad = base_yaw;
    input.base_position_world = {
        base_position.x(), base_position.y(), base_position.z()};
    input.base_velocity_world = {
        base_velocity.x(), base_velocity.y(), base_velocity.z()};
    input.base_roll_rad = base_roll;
    input.base_pitch_rad = base_pitch;
    input.base_height_m = base_height;
    input.gait_phase = gait_phase;
    input.gait_period_s = gait_period;
    input.duty_factor = gait_duty;
    input.commanded_vx_mps = commanded_vx;
    input.current_feet_base = current_feet;
    input.nominal_feet_base = nominal_feet;
    input.contact_schedule.measured_contact = measured_contact;
    input.contact_schedule.measured_valid = true;
    input.contact_schedule.planned_contact = planned_contact;
    input.contact_schedule.planned_valid = true;
    const auto result = planner.Build(input, ++next_plan_id);
    telemetry.plan_epoch = result.plan.plan_epoch;
    telemetry.control_generation = next_plan_id;
    telemetry.plan_status = result.plan.status;
    telemetry.plan_failure = result.plan.failure;
    telemetry.planner_elapsed_us = result.plan.solver.elapsed_us;
    telemetry.planner_deadline_miss = result.plan.solver.deadline_miss;
    return true;
}

WbcTelemetry ComputeWbc(
    const mjData *data, const MotorAddress &address,
    const go2_control::RigidBodyDynamics &dyn,
    const std::array<bool, go2::kLegCount> &contact,
    double applied_vx, double base_height_ref, const std::array<double, 3> &euler,
    int tick, go2_control::SrbdMpcOutput &last_mpc, bool &have_mpc)
{
    WbcTelemetry telemetry;
    go2_control::SrbdMpcParams mpc_params;
    mpc_params.horizon = kMpcHorizon;
    mpc_params.dt_s = kMpcDtS;
    mpc_params.mass_kg = dyn.mass_kg;
    mpc_params.inertia_com_world = dyn.inertia_com_world;

    if (!have_mpc || tick % kMpcPeriodTicks == 0)
    {
        go2_control::SrbdMpcInput mpc_input;
        mpc_input.state[0] = euler[0];
        mpc_input.state[1] = euler[1];
        mpc_input.state[2] = euler[2];
        mpc_input.state[3] = dyn.com_world.x();
        mpc_input.state[4] = dyn.com_world.y();
        mpc_input.state[5] = dyn.com_world.z();
        mpc_input.state[6] = data->qvel[3];
        mpc_input.state[7] = data->qvel[4];
        mpc_input.state[8] = data->qvel[5];
        mpc_input.state[9] = data->qvel[0];
        mpc_input.state[10] = data->qvel[1];
        mpc_input.state[11] = data->qvel[2];
        mpc_input.reference = mpc_input.state;
        mpc_input.reference[0] = 0.0;
        mpc_input.reference[1] = 0.0;
        mpc_input.reference[4] = 0.0;
        mpc_input.reference[5] = base_height_ref;
        mpc_input.reference[6] = 0.0;
        mpc_input.reference[7] = 0.0;
        mpc_input.reference[8] = 0.0;
        mpc_input.reference[9] = applied_vx;
        mpc_input.reference[10] = 0.0;
        mpc_input.reference[11] = 0.0;
        mpc_input.reference[3] =
            dyn.com_world.x() + applied_vx * kMpcDtS * kMpcHorizon * 0.5;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            mpc_input.foot_from_com_world[leg] =
                dyn.foot_pos_world[leg] - dyn.com_world;
        for (int k = 0; k < kMpcHorizon; ++k)
            mpc_input.contact[k] = contact;
        have_mpc =
            go2_control::SolveSrbdMpc(mpc_params, mpc_input, last_mpc) &&
            last_mpc.ok;
    }

    telemetry.srbd_ok = have_mpc && last_mpc.ok;
    if (telemetry.srbd_ok)
    {
        telemetry.srbd_force_x_n =
            last_mpc.first_force[0] + last_mpc.first_force[3] +
            last_mpc.first_force[6] + last_mpc.first_force[9];
        telemetry.srbd_force_z_n =
            last_mpc.first_force[2] + last_mpc.first_force[5] +
            last_mpc.first_force[8] + last_mpc.first_force[11];
        telemetry.srbd_acc_x_mps2 = last_mpc.first_linear_acc.x();
        telemetry.srbd_acc_z_mps2 = last_mpc.first_linear_acc.z();
        telemetry.srbd_ang_acc_y_radps2 = last_mpc.first_angular_acc.y();
    }

    go2_control::IdWbcInput wbc_input;
    wbc_input.dynamics = dyn;
    wbc_input.contact = contact;
    if (telemetry.srbd_ok)
    {
        wbc_input.desired_linear_acc_world = last_mpc.first_linear_acc;
        const Eigen::Quaterniond quat(
            data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]);
        wbc_input.desired_angular_acc_body =
            quat.normalized().toRotationMatrix().transpose() *
            last_mpc.first_angular_acc;
    }

    const Eigen::Matrix3d rotation =
        RotationFromQuaternion(data->qpos + 3);
    bool have_stance_acc = false;
    const Eigen::Vector3d base_position(
        data->qpos[0], data->qpos[1], data->qpos[2]);
    const std::array<go2::Vec3, go2::kLegCount> swing_neutral =
        go2::AllFootPositions(StandJointPositions());
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d foot_velocity =
            dyn.foot_jac_world[leg] * dyn.qvel;
        if (contact[leg])
        {
            wbc_input.stance_acc_world[leg] =
                ClampVector(-8.0 * foot_velocity, 4.0);
            have_stance_acc = true;
        }
        else
        {
            const Eigen::Vector3d desired_foot =
                base_position + rotation * Eigen::Vector3d(
                    swing_neutral[leg].x, swing_neutral[leg].y,
                    swing_neutral[leg].z);
            wbc_input.swing_acc_world[leg] = ClampVector(
                180.0 * (desired_foot - dyn.foot_pos_world[leg]) -
                    16.0 * foot_velocity,
                50.0);
        }
    }
    wbc_input.have_stance_acc = have_stance_acc;

    go2_control::IdWbcParams id_params;
    id_params.friction_mu = 0.8;
    id_params.min_normal_n = 1.0;
    id_params.max_normal_n = 180.0;
    id_params.tau_limit_nm = kTauLimitNm;
    id_params.w_base_lin = 80.0;
    id_params.w_base_ang = 40.0;
    id_params.w_swing = 80.0;
    id_params.w_stance_no_slip = 8.0;
    id_params.w_posture = 0.2;
    id_params.w_force = 1.0e-5;
    id_params.w_tau = 1.0e-4;
    go2_control::IdWbcOutput wbc_output;
    const bool solved =
        go2_control::SolveInverseDynamicsWbc(id_params, wbc_input, wbc_output) &&
        wbc_output.ok;
    telemetry.id_wbc_ok = solved;
    if (solved)
    {
        telemetry.id_eq_residual = wbc_output.eq_residual;
        telemetry.id_qdd_base_x_mps2 = wbc_output.qdd[0];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            telemetry.id_force_x_n +=
                wbc_output.force[3 * static_cast<int>(leg)];
        for (int motor = 0; motor < kMotorCount; ++motor)
        {
            const int joint_row = address.dof[motor] - 6;
            if (joint_row >= 0 && joint_row < kMotorCount)
                telemetry.tau[motor] = wbc_output.tau[joint_row];
        }
    }

    for (int motor = 0; motor < kMotorCount; ++motor)
    {
        telemetry.tau[motor] = std::clamp(
            std::isfinite(telemetry.tau[motor]) ? telemetry.tau[motor] : 0.0,
            -kTauLimitNm, kTauLimitNm);
        telemetry.torque_max_abs_nm = std::max(
            telemetry.torque_max_abs_nm, std::abs(telemetry.tau[motor]));
    }
    return telemetry;
}

bool Run(const Options &options)
{
    go2_trot::VelocityCommandProfile profile;
    std::string profile_error;
    if (!go2_trot::LoadVelocityCommandProfile(
            options.profile_path, profile, &profile_error))
    {
        std::cerr << profile_error << "\n";
        return false;
    }

    char error[1024] = {};
    mjModel *model = mj_loadXML(
        options.scene_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cerr << "Failed to load scene " << options.scene_path
                  << ": " << error << "\n";
        return false;
    }
    model->opt.timestep = kDt;
    mjData *data = mj_makeData(model);
    if (data == nullptr)
    {
        std::cerr << "Failed to allocate MuJoCo data\n";
        mj_deleteModel(model);
        return false;
    }
    const int home_key = mj_name2id(model, mjOBJ_KEY, "home");
    if (home_key < 0)
    {
        std::cerr << "home keyframe not found\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return false;
    }
    mj_resetDataKeyframe(model, data, home_key);
    mju_zero(data->qvel, model->nv);
    mj_forward(model, data);

    MotorAddress address;
    if (!ResolveMotorAddress(model, address))
    {
        std::cerr << "could not resolve Go2 motor addresses\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return false;
    }

    go2_control::Go2RigidBody rigid_body;
    if (!rigid_body.Load(options.model_path))
    {
        std::cerr << "could not load controller model " << options.model_path
                  << "\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return false;
    }

    std::array<int, go2::kLegCount> foot_geoms{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        foot_geoms[leg] = mj_name2id(
            model, mjOBJ_GEOM, go2_control::Go2FootGeomName(leg));
    const int floor_geom = mj_name2id(model, mjOBJ_GEOM, "floor");

    const auto stand_q = StandJointPositions();
    const auto home_q = ReadJointPositions(data, address);
    go2_control::FirstOrderVelocityFilter velocity_filter({4.0});
    go2_trot::VelocityCommandShaperParams shaper_params;
    shaper_params.max_accel_mps2 = 0.80;
    shaper_params.max_decel_mps2 = 1.20;
    shaper_params.max_jerk_mps3 = 4.0;
    shaper_params.max_speed_mps = 3.20;
    shaper_params.max_tracking_lead_mps = 0.20;
    go2_trot::VelocityCommandShaper velocity_shaper(shaper_params);
    velocity_shaper.Reset(0.0);

    go2_control::GaitKernelParams gait_params;
    gait_params.period_s = 0.14;
    gait_params.duty_factor = 0.44;
    gait_params.step_length_m = 0.50;
    gait_params.direction_sign = 1.0;
    gait_params.foot_lift_m = 0.20;
    gait_params.blend_duration_s = 0.8;
    gait_params.pattern = go2_control::GaitPattern::kRunningTrot;
    go2_control::RaibertTrotKernel gait_kernel({
        gait_params, 0.010, 0.06, 4, false});
    gait_kernel.Reset();
    gait_kernel.SetGaitEffectiveSpeedConvention(true);
#if defined(__linux__)
    (void)setenv("TROT_RUNNING_TROT_OFFSET", "0.46", 1);
#endif

    go2_terrain::TerrainPlannerConfig planner_config;
    planner_config.horizon_knots = 8;
    planner_config.knot_dt_s = 0.020;
    planner_config.sensor_only = true;
    planner_config.allow_actuation = false;
    planner_config.measure_realtime_timing = false;
    go2_terrain::TerrainPlanner planner(planner_config);
    std::uint64_t next_map_epoch = 0;
    std::uint64_t next_plan_id = 0;
    TerrainTelemetry terrain;
    go2_control::SrbdMpcOutput last_mpc;
    bool have_mpc = false;

    const double total_duration_s =
        options.stand_duration_s + options.duration_s;
    const std::uint64_t ticks = static_cast<std::uint64_t>(
        std::ceil(total_duration_s / kDt));
    std::filesystem::path output(options.output_dir);
    std::filesystem::create_directories(output);
    WriteMetadata(options, output, total_duration_s);
    std::ofstream csv(output / "data.csv");
    if (!csv)
    {
        std::cerr << "could not open " << (output / "data.csv") << "\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return false;
    }
    WriteCsvHeader(csv);

    for (std::uint64_t tick = 0; tick < ticks; ++tick)
    {
        const double sim_time = static_cast<double>(tick) * kDt;
        const auto joint_state = ReadJointPositions(data, address);
        const auto joint_velocity_state = ReadJointVelocities(data, address);
        const std::array<double, 4> quaternion = {
            data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]};
        const Eigen::Vector3d world_velocity(
            data->qvel[0], data->qvel[1], data->qvel[2]);
        const auto body_velocity =
            BodyVelocity(quaternion, world_velocity);
        const double body_vx = body_velocity[0];
        const go2_control::Vector3 body_velocity_vec = body_velocity;
        go2_control::Vector3 filtered_body_velocity{};
        if (!velocity_filter.Update(
                body_velocity_vec, kDt, filtered_body_velocity))
        {
            std::cerr << "velocity filter failed at tick " << tick << "\n";
            mj_deleteData(data);
            mj_deleteModel(model);
            return false;
        }
        const double filtered_body_vx = filtered_body_velocity[0];

        const double gait_time =
            std::max(0.0, sim_time - options.stand_duration_s);
        const bool gait_active = sim_time >= options.stand_duration_s;
        const double requested_vx =
            gait_active ? profile.Sample(gait_time) : 0.0;
        const auto command =
            velocity_shaper.Step(requested_vx, kDt);
        const auto schedule =
            go2_trot::ScheduleContinuousVelocityGait(command.shaped_mps);
        double applied_vx = command.shaped_mps;
        if (applied_vx > 0.90)
        {
            applied_vx = std::min(
                applied_vx, filtered_body_vx +
                    shaper_params.max_tracking_lead_mps);
            const double overspeed = filtered_body_vx - command.shaped_mps;
            if (overspeed > shaper_params.max_tracking_lead_mps)
                applied_vx = std::max(
                    0.0, command.shaped_mps -
                        (overspeed - shaper_params.max_tracking_lead_mps));
        }
        auto applied_command = command;
        applied_command.applied_mps = applied_vx;

        std::array<double, kMotorCount> joint_target = stand_q;
        double gait_phase = 0.0;
        double gait_period = schedule.period_s;
        double gait_duty = schedule.duty_factor;
        double gait_step = schedule.step_length_m;
        std::array<std::array<bool, go2::kLegCount>,
                   go2_control::kSrbdMaxHorizon> planned_contact{};
        if (gait_active)
        {
            gait_kernel.SetGaitPeriod(schedule.period_s);
            gait_kernel.SetGaitDuty(schedule.duty_factor);
            gait_kernel.SetGaitStepLength(schedule.step_length_m);
            gait_kernel.SetGaitFootLift(schedule.foot_lift_m);
            go2_control::GaitKernelRequest request;
            request.gait_time_s = gait_time;
            request.neutral_feet = go2::AllFootPositions(stand_q);
            request.body_velocity_x_mps = filtered_body_velocity[0];
            request.body_velocity_y_mps = filtered_body_velocity[1];
            request.body_velocity_z_mps = filtered_body_velocity[2];
            request.have_body_velocity = true;
            go2_control::GaitKernelResult result;
            if (!gait_kernel.Compute(request, result))
            {
                std::cerr << "gait kernel failed at tick " << tick << "\n";
                mj_deleteData(data);
                mj_deleteModel(model);
                return false;
            }
            auto feet = result.feet;
            if (!go2::AllLegInverseKinematicsClamped(
                    feet, joint_target))
            {
                std::cerr << "inverse kinematics failed at tick " << tick << "\n";
                mj_deleteData(data);
                mj_deleteModel(model);
                return false;
            }
            gait_phase = result.phase;
            gait_period = result.period_s;
            gait_duty = result.duty_factor;
            gait_step = result.step_length_m;
            go2_control::FillTrotContactSchedulePhase(
                gait_phase, gait_period, gait_duty, kMpcHorizon,
                kMpcDtS, planned_contact,
                go2_control::GaitPattern::kRunningTrot);
        }
        else
        {
            planned_contact.fill({});
            for (int k = 0; k < kMpcHorizon; ++k)
                planned_contact[k].fill(true);
            const double alpha = std::clamp(
                options.stand_duration_s > 0.0
                    ? sim_time / std::min(kStandUpDurationS,
                                          options.stand_duration_s)
                    : 1.0,
                0.0, 1.0);
            for (int motor = 0; motor < kMotorCount; ++motor)
                joint_target[motor] =
                    home_q[motor] + alpha * (stand_q[motor] - home_q[motor]);
            gait_phase = 0.0;
            gait_period = 0.14;
            gait_duty = 1.0;
            gait_step = 0.0;
        }

        const std::uint32_t measured_contact = MeasuredContactMask(
            data, foot_geoms, floor_geom);
        const std::uint32_t planned_mask =
            ContactScheduleMask(planned_contact);
        std::array<bool, go2::kLegCount> measured_contact_array{};
        std::array<bool, go2::kLegCount> planned_contact_array{};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            measured_contact_array[leg] =
                (measured_contact & (1U << leg)) != 0U;
            planned_contact_array[leg] =
                (planned_mask & (1U << leg)) != 0U;
        }

        go2_control::RigidBodyState rigid_state =
            MakeRigidBodyState(data, address);
        go2_control::RigidBodyDynamics dynamics;
        if (!rigid_body.Evaluate(rigid_state, dynamics))
        {
            std::cerr << "rigid body evaluation failed at tick " << tick << "\n";
            mj_deleteData(data);
            mj_deleteModel(model);
            return false;
        }
        const auto euler = EulerFromQuaternion(data->qpos + 3);
        const Eigen::Vector3d base_position(
            data->qpos[0], data->qpos[1], data->qpos[2]);
        const Eigen::Vector3d base_velocity(
            data->qvel[0], data->qvel[1], data->qvel[2]);
        const auto current_feet = go2::AllFootPositions(joint_state);
        const auto nominal_feet = go2::AllFootPositions(stand_q);
        BuildTerrainTelemetry(
            options, tick, sim_time, euler[2], base_position, base_velocity,
            euler[0], euler[1], base_position.z(), gait_phase, gait_period,
            gait_duty, applied_vx, current_feet, nominal_feet,
            measured_contact_array, planned_contact, planner, next_map_epoch,
            next_plan_id, terrain);

        const std::array<bool, go2::kLegCount> wbc_contact =
            gait_active
                ? planned_contact_array
                : std::array<bool, go2::kLegCount>{
                      true, true, true, true};
        WbcTelemetry wbc = ComputeWbc(
            data, address, dynamics, wbc_contact, applied_vx, 0.42, euler,
            static_cast<int>(tick), last_mpc, have_mpc);

        for (int motor = 0; motor < kMotorCount; ++motor)
        {
            const int actuator = address.actuator[motor];
            const double kp = gait_active
                ? (wbc_contact[motor / 3] ? kFullStanceKp : kSwingKp)
                : kStandKp;
            const double kd = gait_active
                ? (wbc_contact[motor / 3] ? kFullStanceKd : kSwingKd)
                : kStandKd;
            const double position_error =
                joint_target[motor] - joint_state[motor];
            const double raw_control = wbc.tau[motor] +
                kp * position_error - kd * joint_velocity_state[motor];
            const double limit = model->actuator_ctrllimited[actuator]
                ? model->actuator_ctrlrange[2 * actuator + 1]
                : 40.0;
            const double clamped_control =
                std::clamp(raw_control, -limit, limit);
            if (std::abs(raw_control) > limit)
                ++wbc.torque_saturation_count;
            wbc.torque_max_abs_nm = std::max(
                wbc.torque_max_abs_nm, std::abs(clamped_control));
            data->ctrl[actuator] = clamped_control;
        }

        WriteCsvRow(
            csv, tick, sim_time, data, joint_target, wbc.tau,
            applied_command, body_vx, filtered_body_vx,
            velocity_filter.last_alpha(), gait_phase, gait_period,
            gait_duty, gait_step, planned_mask, measured_contact,
            terrain, wbc, address);
        mj_step(model, data);
    }

    csv.flush();
    std::ofstream completion(output / "completion.txt");
    completion << "completed_ticks=" << ticks << "\n"
               << "completed_sim_time_s=" << std::setprecision(17)
               << static_cast<double>(ticks) * kDt << "\n";
    mj_deleteData(data);
    mj_deleteModel(model);
    return true;
}
}  // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!ParseOptions(argc, argv, options))
        return argc > 1 && (std::string(argv[1]) == "--help" ||
                            std::string(argv[1]) == "-h")
            ? 0
            : 2;
    return Run(options) ? 0 : 1;
}
