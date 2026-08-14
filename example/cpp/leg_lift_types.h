#pragma once
// Types and helpers for quasi-static leg-lift / multi-step sequences.

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include "go2_forward_kinematics.h"
#include "go2_inverse_kinematics.h"

namespace go2_leg {

constexpr double PosStopF = 2.146E+9;
constexpr double VelStopF = 16000.0;
constexpr int kMotorCount = 12;
constexpr double kStandUpDuration = 3.0;
constexpr double kStandSettleDuration = 0.5;
constexpr double kWeightShiftDuration = 2.0;
constexpr double kWeightShiftSettleDuration = 0.5;
constexpr double kFootLiftDuration = 1.0;
constexpr double kFootSwingDuration = 2.0;
constexpr double kFootLowerDuration = 2.0;
constexpr double kFootLowerSettleDuration = 0.5;
constexpr double kBodyReturnDuration = 3.0;
constexpr double kBetweenCycleSettleDuration = 1.0;
constexpr double kFinalHoldDuration = 0.5;
constexpr double kLandingForceThreshold = 5.0;
constexpr double kNaturalSettleHoldDuration = 0.5;
constexpr double kNaturalSettleMaxJointSpeed = 0.05;
constexpr double kNaturalSettleMaxBodyAngularSpeed = 0.05;
constexpr double kWorldFeedbackGain = 0.8;
constexpr double kWorldFeedbackMaxCorrectionM = 0.030;
constexpr double kWorldFeedbackMaxCorrectionStepM = 0.008;
// Small IMU-derived body-target trims keep the nominal gait geometry intact.
constexpr double kPitchFeedbackBodyShiftGainMPerRad = 0.060;
constexpr double kRollFeedbackBodyShiftGainMPerRad = 0.060;
constexpr double kAttitudeFeedbackMaxCorrectionM = 0.004;
constexpr double kAttitudeFeedbackFilterAlpha = 0.05;
constexpr double kWorldFeedbackReferenceTimeoutS = 2.0;
constexpr double kWorldYawFeedbackGainMPerRad = 0.20;
constexpr double kWorldYawFeedbackMaxCorrectionM = 0.020;
constexpr double kWorldYawBodyRotationGain = 0.35;
constexpr double kWorldYawBodyRotationMaxRad = 0.060;
constexpr double kWorldYawSwingFeedbackGainMPerRad = 0.08;
constexpr double kWorldYawSwingFeedbackMaxCorrectionM = 0.012;
constexpr double kWorldYawPlacedOffsetMaxM = 0.024;
constexpr double kFootCollisionRadiusM = 0.022;
constexpr double kPi = 3.14159265358979323846;
constexpr double kSafetyMaxAbsRollRad = 2.4 * kPi / 180.0;
constexpr double kSafetyMaxAbsPitchRad = 2.4 * kPi / 180.0;
constexpr double kSafetyMaxAbsYawErrorRad = 10.0 * kPi / 180.0;
constexpr double kSafetyMaxSupportDriftM = 0.015;
constexpr double kSafetyMaxJointErrorRad = 0.125;
constexpr double kSafetyMaxTauEst = 12.0;
constexpr double kTempoGovernorSoftJointErrorRad = 0.090;
constexpr double kTempoGovernorSoftTauEst = 9.0;
constexpr double kTempoGovernorSoftAttitudeRad = 1.6 * kPi / 180.0;
constexpr double kTempoGovernorMinScale = 0.15;
// Motion time follows the simulator tick. A long gap means the renderer or
// physics loop was temporarily stalled; resync without jumping the gait phase.
constexpr double kMaxStateClockGapS = 0.008;
constexpr double kSupportContactThreshold = 5.0;
constexpr int kSafetyMinSupportContacts = 2;
// Contact sensors may drop a single sample at a collision boundary.
constexpr double kSafetyMinSupportContactFraction = 0.98;
constexpr int kSafetyMaxConsecutiveLowSupportSamples = 5;
constexpr double kTerminalCorrectionDuration = 3.0;
constexpr double kTerminalCorrectionGain = 0.8;
constexpr double kTerminalCorrectionMaxM = 0.030;
constexpr double kTerminalPositionToleranceM = 0.010;
const go2::Vec3 kImuPositionInBase{-0.02557, 0.0, 0.04232};

const std::array<const char *, kMotorCount> kMotorNames = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

const std::array<const char *, go2::kLegCount> kLegNames = {
    "FR", "FL", "RR", "RL"};

struct StepConfig
{
    go2::Leg lift_leg;
    double body_shift_x_m;
    double body_shift_y_m;
    double foot_lift_height_m;
    double swing_x_m;
    double swing_y_m;
    double body_advance_x_m;
    double body_advance_y_m;
};

inline go2::Leg ParseLegName(const std::string &name)
{
    if (name == "FR")
        return go2::Leg::FR;
    if (name == "FL")
        return go2::Leg::FL;
    if (name == "RR")
        return go2::Leg::RR;
    if (name == "RL")
        return go2::Leg::RL;
    throw std::invalid_argument(
        "Unknown leg '" + name + "'. Use FR, FL, RR, or RL.");
}

inline bool LoadStepSequence(
    const std::string &path,
    std::vector<StepConfig> &steps)
{
    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "Failed to open step sequence: " << path << std::endl;
        return false;
    }
    std::string line;
    int line_number = 0;
    while (std::getline(file, line))
    {
        ++line_number;
        std::istringstream stream(line);
        std::string leg_name;
        if (!(stream >> leg_name))
            continue;
        if (leg_name[0] == '#')
            continue;
        StepConfig step{};
        if (!(stream >> step.body_shift_x_m >> step.body_shift_y_m
              >> step.foot_lift_height_m >> step.swing_x_m
              >> step.swing_y_m >> step.body_advance_x_m
              >> step.body_advance_y_m))
        {
            std::cerr << "Invalid step sequence at " << path << ":"
                      << line_number << std::endl;
            return false;
        }
        try
        {
            step.lift_leg = ParseLegName(leg_name);
        }
        catch (const std::exception &error)
        {
            std::cerr << error.what() << " at " << path << ":"
                      << line_number << std::endl;
            return false;
        }
        steps.push_back(step);
    }
    if (steps.empty())
    {
        std::cerr << "Step sequence is empty: " << path << std::endl;
        return false;
    }
    return true;
}

inline void PrintFootPositions(
    const std::string &label,
    const std::array<double, kMotorCount> &joint_positions)
{
    const auto feet = go2::AllFootPositions(joint_positions);
    std::cout << label << " foot positions in base_link frame (m):\n"
              << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < feet.size(); ++i)
    {
        std::cout << "  " << kLegNames[i]
                  << ": x=" << feet[i].x
                  << " y=" << feet[i].y
                  << " z=" << feet[i].z << "\n";
    }
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

struct WorldPose
{
    go2::Vec3 base{};
    std::array<double, 4> quaternion{};
    double yaw_rad = 0.0;
};
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
    const auto imu_offset_world =
        RotateByQuaternion(pose.quaternion, kImuPositionInBase);
    pose.base = {
        imu_world.x - imu_offset_world.x,
        imu_world.y - imu_offset_world.y,
        imu_world.z - imu_offset_world.z};
    pose.yaw_rad = low_state.imu_state().rpy()[2];
    return pose;
}
struct StepDiagnostics
{
    double max_abs_roll_rad = 0.0;
    double max_abs_pitch_rad = 0.0;
    double max_abs_yaw_error_rad = 0.0;
    double max_support_drift_m = 0.0;
    double max_abs_joint_error_rad = 0.0;
    double max_abs_tau_est = 0.0;
    int min_support_contacts = go2::kLegCount;
    int support_contact_samples = 0;
    int support_contact_good_samples = 0;
    int consecutive_low_support_samples = 0;
    int max_consecutive_low_support_samples = 0;
    double support_contact_fraction = 0.0;
    bool have_sample = false;
    bool support_reference_valid = false;
    std::array<go2::Vec3, go2::kLegCount> support_reference_world_feet{};
};


inline uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

inline double smoothstep(double x)
{
    if (x <= 0.0)
        return 0.0;
    if (x >= 1.0)
        return 1.0;
    return x * x * (3 - 2 * x);
}


}  // namespace go2_leg
