#pragma once
// Shared constants, parameters and small helpers for real_trot_go2.
// Kept header-only so the controller stays a single translation unit link.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include "go2_inverse_kinematics.h"
#include "locomotion_kernel.h"
#include "motion_event_response.h"
#include "raibert_trot_kernel.h"
#include "wbc_runtime_gate.h"
#include "velocity_command.h"

namespace go2_trot {


constexpr double kPi = 3.14159265358979323846;
constexpr double kPosStopF = 2.146E+9;
constexpr double kVelStopF = 16000.0;
constexpr int kMotorCount = 12;
constexpr double kDt = 0.002;

constexpr double kStandUpDuration = 3.0;
constexpr double kStandSettleDuration = 0.5;
constexpr double kStandDownDuration = 3.0;
constexpr double kGaitBlendDuration = 0.8;
constexpr double kStopTransitionDuration = 2.0;
constexpr double kSupportAnchorBlendDuration = 0.8;
constexpr double kFinalHoldDuration = 0.8;
constexpr double kLieDownHoldDuration = 0.8;

constexpr double kDefaultPeriodS = 0.8;
constexpr double kDefaultDutyFactor = 0.58;
constexpr double kDefaultStepLengthM = 0.12;
constexpr double kDefaultFootLiftM = 0.050;
constexpr double kDefaultKp = 80.0;
constexpr double kDefaultKd = 4.5;

constexpr double kStateClockMaxGapS = 0.008;
constexpr double kShadowWbcMassKg = 15.206408;
constexpr double kShadowWbcGravityMps2 = 9.81;
constexpr double kShadowWbcFrictionCoefficient = 0.8;
constexpr double kShadowWbcMaxNormalForce = 100.0;
constexpr double kShadowWbcBudgetUs = 1000.0;
constexpr double kShadowContactOnForceN = 5.0;
constexpr double kShadowContactOffForceN = 3.0;
constexpr double kWbcTorqueFeedforwardDefaultScale = 0.10;
constexpr double kWbcTorqueFeedforwardMaxScale = 0.25;
constexpr double kWbcTorqueFeedforwardMaxAbsNm = 3.0;
constexpr double kWbcVelocityGainSInv = 2.0;
constexpr double kWbcMaxForwardForceN = 20.0;
// WBC 主控模式(--wbc-primary):增强 wrench + 扭矩直接输出
constexpr double kWbcPrimaryBaseHeightM = 0.42;
constexpr double kWbcPrimaryHeightKp = 400.0;
constexpr double kWbcPrimaryHeightKd = 60.0;
constexpr double kWbcPrimaryRollKp = 60.0;
constexpr double kWbcPrimaryRollKd = 6.0;
constexpr double kWbcPrimaryPitchKp = 80.0;
constexpr double kWbcPrimaryPitchKd = 6.0;
constexpr double kWbcPrimaryYawKd = 2.0;
constexpr double kWbcPrimaryVelGain1S = 6.0;
constexpr double kWbcPrimaryHeightAccKp = 30.0;
constexpr double kWbcPrimaryHeightAccKd = 5.0;
constexpr double kWbcPrimaryRollAccKp = 600.0;
constexpr double kWbcPrimaryRollAccKd = 32.0;
constexpr double kWbcPrimaryPitchAccKp = 120.0;
constexpr double kWbcPrimaryPitchAccKd = 12.0;
constexpr double kWbcPrimaryYawAccKd = 8.0;
constexpr double kWbcPrimaryTurnYawAccKp = 12.0;
constexpr double kWbcPrimaryTurnYawKp = 3.0;
constexpr double kWbcPrimaryCommandKp = 63.0;
constexpr double kWbcPrimaryCommandKd = 2.8;
// Full centroidal path owns gravity in the wrench, so stance PD is softer.
constexpr double kWbcFullStanceKp = 25.0;
constexpr double kWbcFullStanceKd = 2.0;
constexpr double kWbcPrimaryEnterDelayS = 0.5;
constexpr double kWbcPrimaryRampS = 0.5;
constexpr double kWbcPrimaryBlendTauS = 0.12;
constexpr double kWbcPrimaryWrenchEnableS = 1.5;
constexpr double kWbcPrimaryMaxAbsTorqueNm = 25.0;
// 冲量主控 v1 (--impulse): 线动量任务(加速度域)增益
constexpr double kImpulseLinVelKpS = 2.0;    // 速度误差 -> 加速度(渐进, 留稳定余量)
constexpr double kImpulseLinVelKd = 2.0;     // 加速度阻尼(反速度差分)
constexpr double kImpulseLinAccMaxMps2 = 1.5; // 线加速度限幅(渐进加速,防瞬时过冲)
constexpr double kImpulseMassKg = 15.206408;  // 与 kShadowWbcMassKg 一致
constexpr double kImpulseStanceKp = 63.0;    // 冲量模式支撑腿 kp(位置环主导,力控叠加)
constexpr double kContactForceThreshold = 5.0;
constexpr int kMinimumSupportContacts = 2;

constexpr double kSafetyMaxRollRad = 8.0 * kPi / 180.0;
constexpr double kSafetyMaxPitchRad = 8.0 * kPi / 180.0;
constexpr double kSafetyMaxJointErrorRad = 0.18;
// This is a cycle-quality envelope, not the instantaneous emergency limit.
// tau_est can contain isolated samples from the simulated actuator sensor.
constexpr double kSafetyMaxTauEstDefault = 18.0;
constexpr double kSafetyMaxTauBurstEst = 40.0;  // 电机物理限(放宽后), 单样本尖峰正常
constexpr int kSafetyMaxTauOverLimitSamples = 5;
constexpr int kSafetyMaxConsecutiveTauOverLimitSamples = 3;
constexpr double kSafetyMinSupportFraction = 0.90;
constexpr int kSafetyMaxConsecutiveLowSupport = 20;

constexpr double kHardMaxRollRad = 15.0 * kPi / 180.0;
constexpr double kHardMaxPitchRad = 15.0 * kPi / 180.0;
constexpr double kHardMaxJointErrorRad = 0.30;
constexpr double kHardMaxTauEst = 30.0;

constexpr double kWorldFeedbackGain = 0.35;
constexpr double kWorldFeedbackMaxM = 0.020;
constexpr double kWorldFeedbackSlewM = 0.0015;
constexpr double kYawFeedbackGain = 0.20;
constexpr double kYawFeedbackMaxRad = 0.10;
constexpr double kAttitudeFeedbackGainMPerRad = 0.06;
constexpr double kAttitudeFeedbackMaxM = 0.018;
constexpr double kAttitudeFilterAlpha = 0.08;
// The Go2 scene's Unitree IMU axes map to base-body axes as (z, -y, x).
// Keep raw sensor fields and use this conversion for body-frame task features.

inline std::array<double, 3> ConvertImuGyroToBody(
    const std::array<double, 3> &imu_gyro)
{
    return {imu_gyro[2], -imu_gyro[1], imu_gyro[0]};
}

const std::array<const char *, kMotorCount> kMotorNames = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

const std::array<const char *, go2::kLegCount> kLegNames = {
    "FR", "FL", "RR", "RL"};

struct TrotParams
{
    double period_s = kDefaultPeriodS;
    double duty_factor = kDefaultDutyFactor;
    double step_length_m = kDefaultStepLengthM;
    double direction_sign = 1.0;
    bool support_anchor_feedback = false;
    double support_anchor_gain = 0.25;
    double foot_lift_m = kDefaultFootLiftM;
    double kp = kDefaultKp;
    double kd = kDefaultKd;
    bool world_feedback = true;
    double world_feedback_gain = kWorldFeedbackGain;
    double world_feedback_max_m = kWorldFeedbackMaxM;
    double world_feedback_slew_m = kWorldFeedbackSlewM;
    bool wbc_shadow = false;
    bool wbc_primary = false;
    bool wbc_full = false;
    bool cartesian_world = false;
    int preview_horizon_steps = 0;
    std::vector<std::pair<int, double>> step_plan;
    std::vector<std::pair<int, double>> period_plan;
    double turn_rate_radps = 0.0;
    double tau_limit_nm = kSafetyMaxTauEstDefault;
    double bounce_acc_amp = 0.0;
    bool wbc_torque_feedforward = false;
    bool wbc_task_torque_feedforward = false;
    double wbc_torque_scale = kWbcTorqueFeedforwardDefaultScale;
    bool wbc_velocity_wrench = false;
    bool impulse = false;          // 冲量主控 v1: 线动量任务参考生成
    double wbc_velocity_gain_s_inv = kWbcVelocityGainSInv;
    double wbc_max_forward_force_n = kWbcMaxForwardForceN;
    bool wbc_reduced_contact_task = false;
    bool attitude_feedback = true;
    std::string kernel_name = "hand-coded-trot";
    double raibert_velocity_gain_s = 0.20;
    double raibert_max_adjustment_m = 0.025;
    double velocity_filter_cutoff_hz = 4.0;
    bool velocity_feedforward = true;
    go2_control::GaitPattern gait_pattern =
        go2_control::GaitPattern::kDiagonalTrot;
    bool wall_clock_motion = false;
    bool reactive_events = false;
    bool auto_environment = false;
    bool terrain_enabled = false;
    bool terrain_sensor_only = false;
    bool terrain_actuation = false;
    bool runtime_velocity_command = false;
    double gait_phase_offset = 0.0;
    std::string velocity_command_script_path;
    VelocityCommandProfile velocity_command_profile;
    VelocityCommandShaperParams velocity_command_shaper{};
    // >= 0 enables a detector-driven emergency stop after a physical impact.
    // A negative value keeps the legacy detector-only behavior.
    double impact_to_emergency_stop_delay_s = -1.0;
    std::string event_script_path;
    std::vector<go2_control::MotionEvent> event_schedule;
};

inline std::unique_ptr<go2_control::LocomotionKernel> CreateLocomotionKernel(
    const TrotParams &params)
{
    const go2_control::GaitKernelParams gait_params{
        params.period_s, params.duty_factor,
        params.step_length_m, params.direction_sign,
        params.foot_lift_m, kGaitBlendDuration, -1.0,
        params.gait_pattern};
    if (params.kernel_name == "raibert-trot")
    {
        return std::make_unique<go2_control::RaibertTrotKernel>(
            go2_control::RaibertTrotKernelParams{
                gait_params, params.raibert_velocity_gain_s,
                params.raibert_max_adjustment_m,
                params.preview_horizon_steps,
                params.wbc_full && !params.step_plan.empty()});
    }
    return std::make_unique<go2_control::HandCodedTrotKernel>(
        gait_params);
}

struct WorldPose
{
    go2::Vec3 base{};
    std::array<double, 4> quaternion{};
    double yaw_rad = 0.0;
};

struct CycleDiagnostics
{
    double max_abs_roll_rad = 0.0;
    double max_abs_pitch_rad = 0.0;
    double max_abs_joint_error_rad = 0.0;
    double max_foot_error_m = 0.0;
    int touchdown_events = 0;
    double max_abs_touchdown_x_error_m = 0.0;
    double max_abs_touchdown_y_error_m = 0.0;
    double max_abs_tau_est = 0.0;
    double max_support_drift_m = 0.0;
    int min_support_contacts = go2::kLegCount;
    int support_contact_samples = 0;
    int support_contact_good_samples = 0;
    int consecutive_low_support_samples = 0;
    int max_consecutive_low_support_samples = 0;

    int tau_over_limit_samples = 0;
    int consecutive_tau_over_limit_samples = 0;
    int max_consecutive_tau_over_limit_samples = 0;
    int max_tau_motor_index = -1;
    std::array<go2::Vec3, go2::kLegCount> support_reference_world_feet{};
    std::array<bool, go2::kLegCount> support_reference_valid{};
};

struct WbcShadowDiagnostics
{
    bool enabled = false;
    bool solver_ok = false;
    bool mapping_ok = false;
    bool wrench_satisfied = false;
    bool constraint_feasible = false;
    int active_contacts = 0;
    int contact_mask = 0;
    int iterations = 0;
    double residual_norm = 0.0;
    double desired_force_x_n = 0.0;
    double max_axis_friction_ratio = 0.0;
    bool reduced_contact_task = false;
    bool task_satisfied = false;
    double task_residual_norm = 0.0;
    double max_radial_friction_ratio = 0.0;
    double min_contact_normal_force_n = 0.0;
    double elapsed_us = 0.0;
    bool within_budget = true;
    double max_abs_tau = 0.0;
    bool srbd_ok = false;
    double full_velocity_target_x_mps = 0.0;
    double full_requested_acc_x_mps2 = 0.0;
    double full_srbd_acc_x_mps2 = 0.0;
    double full_id_qdd_x_mps2 = 0.0;
    double full_id_contact_force_x_n = 0.0;
    bool id_wbc_ok = false;
    double id_eq_residual = 0.0;
    int feedforward_gate_code =
        static_cast<int>(go2_control::WbcFeedforwardGateCode::kDisabled);
    bool feedforward_ready = false;
    bool feedforward_applied = false;
    bool feedforward_reduced_task_gate = false;
    double feedforward_max_abs_tau = 0.0;
};

inline uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int crc = 0xFFFFFFFF;
    constexpr unsigned int polynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1u << 31;
        data = ptr[i];
        for (unsigned int bit = 0; bit < 32; ++bit)
        {
            if (crc & 0x80000000u)
            {
                crc <<= 1;
                crc ^= polynomial;
            }
            else
            {
                crc <<= 1;
            }
            if (data & xbit)
                crc ^= polynomial;
            xbit >>= 1;
        }
    }
    return crc;
}

inline double Smoothstep(double x)
{
    if (x <= 0.0)
        return 0.0;
    if (x >= 1.0)
        return 1.0;
    return x * x * (3.0 - 2.0 * x);
}

inline double WrapAngle(double angle)
{
    return std::remainder(angle, 2.0 * kPi);
}

inline double Clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

inline go2::Vec3 RotateByQuaternion(
    const std::array<double, 4> &quaternion,
    const go2::Vec3 &vector)
{
    const double w = quaternion[0];
    const double x = quaternion[1];
    const double y = quaternion[2];
    const double z = quaternion[3];
    return {
        (1.0 - 2.0 * (y * y + z * z)) * vector.x +
            2.0 * (x * y - z * w) * vector.y +
            2.0 * (x * z + y * w) * vector.z,
        2.0 * (x * y + z * w) * vector.x +
            (1.0 - 2.0 * (x * x + z * z)) * vector.y +
            2.0 * (y * z - x * w) * vector.z,
        2.0 * (x * z - y * w) * vector.x +
            2.0 * (y * z + x * w) * vector.y +
            (1.0 - 2.0 * (x * x + y * y)) * vector.z};
}

inline WorldPose ComputeWorldPose(
    const unitree_go::msg::dds_::LowState_ &low_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state)
{
    WorldPose pose;
    pose.quaternion = {
        low_state.imu_state().quaternion()[0],
        low_state.imu_state().quaternion()[1],
        low_state.imu_state().quaternion()[2],
        low_state.imu_state().quaternion()[3]};
    const go2::Vec3 imu_world = {
        high_state.position()[0],
        high_state.position()[1],
        high_state.position()[2]};
    const go2::Vec3 imu_offset_world =
        RotateByQuaternion(pose.quaternion, {-0.02557, 0.0, 0.04232});
    pose.base = {
        imu_world.x - imu_offset_world.x,
        imu_world.y - imu_offset_world.y,
        imu_world.z - imu_offset_world.z};
    pose.yaw_rad = low_state.imu_state().rpy()[2];
    return pose;
}

inline std::array<go2::Vec3, go2::kLegCount> ComputeWorldFeet(
    const unitree_go::msg::dds_::LowState_ &low_state,
    const WorldPose &pose)
{
    std::array<double, kMotorCount> joint_positions{};
    for (int i = 0; i < kMotorCount; ++i)
    {
        joint_positions[i] =
            static_cast<double>(low_state.motor_state()[i].q());
    }
    const auto body_feet = go2::AllFootPositions(joint_positions);
    std::array<go2::Vec3, go2::kLegCount> world_feet{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto rotated = RotateByQuaternion(
            pose.quaternion, body_feet[leg]);
        world_feet[leg] = {
            pose.base.x + rotated.x,
            pose.base.y + rotated.y,
            pose.base.z + rotated.z};
    }
    return world_feet;
}

}  // namespace go2_trot
