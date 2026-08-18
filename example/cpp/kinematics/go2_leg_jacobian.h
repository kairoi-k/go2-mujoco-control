#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "go2_forward_kinematics.h"

namespace go2_control
{

// Rows are foot x/y/z, columns are hip/thigh/calf joint velocities.
using LegFootJacobian = std::array<std::array<double, 3>, 3>;

inline LegFootJacobian FootJacobian(
    go2::Leg leg,
    double q_hip,
    double q_thigh,
    double q_calf)
{
    const go2::LegGeometry geometry = go2::Geometry(leg);
    const double lower_angle = q_thigh + q_calf;
    const double sin_hip = std::sin(q_hip);
    const double cos_hip = std::cos(q_hip);
    const double sin_thigh = std::sin(q_thigh);
    const double cos_thigh = std::cos(q_thigh);
    const double sin_lower = std::sin(lower_angle);
    const double cos_lower = std::cos(lower_angle);

    const double leg_x =
        -geometry.thigh_length * sin_thigh -
        geometry.calf_length * sin_lower;
    const double leg_z =
        -geometry.thigh_length * cos_thigh -
        geometry.calf_length * cos_lower;
    const double lower_x = -geometry.calf_length * sin_lower;
    const double lower_z = -geometry.calf_length * cos_lower;

    const double lateral_y =
        cos_hip * geometry.hip_link_y - sin_hip * leg_z;
    const double d_leg_z_d_thigh =
        geometry.thigh_length * sin_thigh +
        geometry.calf_length * sin_lower;
    const double d_leg_z_d_calf =
        geometry.calf_length * sin_lower;

    return {{
        {{0.0, leg_z, lower_z}},
        {{
            -sin_hip * geometry.hip_link_y - cos_hip * leg_z,
            -sin_hip * d_leg_z_d_thigh,
            -sin_hip * d_leg_z_d_calf}},
        {{
            lateral_y,
            cos_hip * d_leg_z_d_thigh,
            cos_hip * d_leg_z_d_calf}}}};
}

} // namespace go2_control

