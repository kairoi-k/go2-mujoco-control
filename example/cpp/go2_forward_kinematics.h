// Go2 leg forward kinematics (joint angles -> foot positions).
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace go2
{

constexpr std::size_t kLegCount = 4;
constexpr std::size_t kJointsPerLeg = 3;
constexpr std::size_t kJointCount = kLegCount * kJointsPerLeg;

enum class Leg : std::size_t
{
    FR = 0,
    FL = 1,
    RR = 2,
    RL = 3,
};

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct LegGeometry
{
    double hip_x;
    double hip_y;
    double hip_link_y;
    double thigh_length;
    double calf_length;
};

inline LegGeometry Geometry(Leg leg)
{
    const bool front = leg == Leg::FR || leg == Leg::FL;
    const bool left = leg == Leg::FL || leg == Leg::RL;
    const double side_sign = left ? 1.0 : -1.0;

    return {
        front ? 0.1934 : -0.1934,
        side_sign * 0.0465,
        side_sign * 0.0955,
        0.213,
        0.213,
    };
}

// Returns foot position in the base_link coordinate frame:
// x forward, y left, z upward.
inline Vec3 FootPosition(Leg leg, double q_hip, double q_thigh, double q_calf)
{
    const LegGeometry geometry = Geometry(leg);

    const double sin_hip = std::sin(q_hip);
    const double cos_hip = std::cos(q_hip);
    const double sin_thigh = std::sin(q_thigh);
    const double cos_thigh = std::cos(q_thigh);
    const double sin_lower = std::sin(q_thigh + q_calf);
    const double cos_lower = std::cos(q_thigh + q_calf);

    // First calculate the sagittal-plane displacement from the thigh joint.
    const double leg_x =
        -geometry.thigh_length * sin_thigh -
        geometry.calf_length * sin_lower;
    const double leg_z =
        -geometry.thigh_length * cos_thigh -
        geometry.calf_length * cos_lower;

    // Hip abduction rotates both the lateral hip link and sagittal leg about x.
    const double lateral_y =
        cos_hip * geometry.hip_link_y - sin_hip * leg_z;
    const double lateral_z =
        sin_hip * geometry.hip_link_y + cos_hip * leg_z;

    return {
        geometry.hip_x + leg_x,
        geometry.hip_y + lateral_y,
        lateral_z,
    };
}

inline std::array<Vec3, kLegCount> AllFootPositions(
    const std::array<double, kJointCount> &joint_positions)
{
    std::array<Vec3, kLegCount> feet{};
    for (std::size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
    {
        const std::size_t joint_index = leg_index * kJointsPerLeg;
        feet[leg_index] = FootPosition(
            static_cast<Leg>(leg_index),
            joint_positions[joint_index],
            joint_positions[joint_index + 1],
            joint_positions[joint_index + 2]);
    }
    return feet;
}

} // namespace go2
