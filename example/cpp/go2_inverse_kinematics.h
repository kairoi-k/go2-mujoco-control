// Go2 leg inverse kinematics (foot target -> joint angles).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "go2_forward_kinematics.h"

namespace go2
{

struct LegJointPositions
{
    double hip = 0.0;
    double thigh = 0.0;
    double calf = 0.0;
};

inline bool LegInverseKinematics(
    Leg leg,
    const Vec3 &foot_position,
    LegJointPositions &joint_positions)
{
    const LegGeometry geometry = Geometry(leg);
    const double x = foot_position.x - geometry.hip_x;
    const double y = foot_position.y - geometry.hip_y;
    const double z = foot_position.z;

    const double leg_z_squared =
        y * y + z * z - geometry.hip_link_y * geometry.hip_link_y;
    if (leg_z_squared < -1e-12)
    {
        return false;
    }

    // The normal standing solution places the thigh joint below the hip link.
    const double leg_z = -std::sqrt(std::max(0.0, leg_z_squared));
    const double q_hip =
        std::atan2(z, y) - std::atan2(leg_z, geometry.hip_link_y);

    const double planar_distance_squared = x * x + leg_z * leg_z;
    const double cosine_calf =
        (planar_distance_squared -
         geometry.thigh_length * geometry.thigh_length -
         geometry.calf_length * geometry.calf_length) /
        (2.0 * geometry.thigh_length * geometry.calf_length);
    if (cosine_calf < -1.0 - 1e-12 || cosine_calf > 1.0 + 1e-12)
    {
        return false;
    }

    // Go2's normal knee configuration uses the negative knee-angle branch.
    const double q_calf = -std::acos(std::clamp(cosine_calf, -1.0, 1.0));
    const double q_thigh =
        std::atan2(-x, -leg_z) -
        std::atan2(
            geometry.calf_length * std::sin(q_calf),
            geometry.thigh_length +
                geometry.calf_length * std::cos(q_calf));

    joint_positions = {q_hip, q_thigh, q_calf};
    return true;
}

inline bool AllLegInverseKinematics(
    const std::array<Vec3, kLegCount> &foot_positions,
    std::array<double, kJointCount> &joint_positions)
{
    for (std::size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
    {
        LegJointPositions leg_joints;
        if (!LegInverseKinematics(
                static_cast<Leg>(leg_index),
                foot_positions[leg_index],
                leg_joints))
        {
            return false;
        }

        const std::size_t joint_index = leg_index * kJointsPerLeg;
        joint_positions[joint_index] = leg_joints.hip;
        joint_positions[joint_index + 1] = leg_joints.thigh;
        joint_positions[joint_index + 2] = leg_joints.calf;
    }
    return true;
}

} // namespace go2
