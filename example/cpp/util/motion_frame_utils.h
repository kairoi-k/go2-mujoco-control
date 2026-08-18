#pragma once

#include <array>
#include <cmath>

namespace go2_control
{

using Quaternion = std::array<double, 4>;
using Vector3 = std::array<double, 3>;

// MuJoCo framelinvel is expressed in world coordinates. Convert it to the
// robot base frame before using it as a body-frame locomotion measurement.
inline bool WorldToBodyVelocity(
    const Quaternion &orientation_world_from_body,
    const Vector3 &world_velocity,
    Vector3 &body_velocity)
{
    double norm_squared = 0.0;
    for (double value : orientation_world_from_body)
    {
        if (!std::isfinite(value))
            return false;
        norm_squared += value * value;
    }
    if (!(norm_squared > 1e-12) || !std::isfinite(norm_squared))
        return false;
    for (double value : world_velocity)
    {
        if (!std::isfinite(value))
            return false;
    }

    // The conjugate rotates a world vector into the body frame.
    const double scale = 1.0 / std::sqrt(norm_squared);
    const double w = orientation_world_from_body[0] * scale;
    const double x = -orientation_world_from_body[1] * scale;
    const double y = -orientation_world_from_body[2] * scale;
    const double z = -orientation_world_from_body[3] * scale;
    const double vx = world_velocity[0];
    const double vy = world_velocity[1];
    const double vz = world_velocity[2];
    body_velocity = {
        (1.0 - 2.0 * (y * y + z * z)) * vx +
            2.0 * (x * y - z * w) * vy +
            2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx +
            (1.0 - 2.0 * (x * x + z * z)) * vy +
            2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx +
            2.0 * (y * z + x * w) * vy +
            (1.0 - 2.0 * (x * x + y * y)) * vz};
    return std::isfinite(body_velocity[0]) &&
           std::isfinite(body_velocity[1]) &&
           std::isfinite(body_velocity[2]);
}

} // namespace go2_control
