#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace go2_control
{

struct DynamicAccelerationTargetInput
{
    std::array<double, 4> quaternion{};
    std::array<double, 3> measured_acceleration_world{};
    std::array<double, 6> mass_qacc_qcoord{};
    std::array<double, 36> base_mass_matrix_qcoord{};
    std::array<double, 6> smooth_qcoord{};
    double desired_force_x_n = 0.0;
    double mass_kg = 1.0;
};

struct DynamicAccelerationTargetOptions
{
    double correction_limit_mps2 = 2.0;
    double correction_slew_limit_mps3 = 20.0;
};

struct DynamicAccelerationTargetResult
{
    bool valid = false;
    bool reference_held_for_duplicate_time = false;
    bool slew_limited = false;
    std::array<double, 6> desired_wrench_body{};
    std::array<double, 3> raw_correction_world{};
    std::array<double, 3> applied_correction_world{};
    std::array<double, 3> correction_slack_world{};
};

inline bool IsFinite(const std::array<double, 4> &value)
{
    for (double component : value)
    {
        if (!std::isfinite(component))
            return false;
    }
    return true;
}

template <std::size_t N>
inline bool IsFinite(const std::array<double, N> &value)
{
    for (double component : value)
    {
        if (!std::isfinite(component))
            return false;
    }
    return true;
}

inline std::array<double, 3> RotateBodyToWorld(
    const std::array<double, 4> &quaternion,
    const std::array<double, 3> &vector)
{
    const double norm = std::sqrt(
        quaternion[0] * quaternion[0] +
        quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] +
        quaternion[3] * quaternion[3]);
    if (!std::isfinite(norm) || norm <= 1e-12)
        return {0.0, 0.0, 0.0};

    const double w = quaternion[0] / norm;
    const double x = quaternion[1] / norm;
    const double y = quaternion[2] / norm;
    const double z = quaternion[3] / norm;
    const double vx = vector[0];
    const double vy = vector[1];
    const double vz = vector[2];
    return {
        (1.0 - 2.0 * (y * y + z * z)) * vx +
            2.0 * (x * y - z * w) * vy +
            2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx +
            (1.0 - 2.0 * (x * x + z * z)) * vy +
            2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx +
            2.0 * (y * z + x * w) * vy +
            (1.0 - 2.0 * (x * x + y * y)) * vz};
}

inline std::array<double, 3> RotateWorldToBody(
    const std::array<double, 4> &quaternion,
    const std::array<double, 3> &vector)
{
    const std::array<double, 4> conjugate = {
        quaternion[0],
        -quaternion[1],
        -quaternion[2],
        -quaternion[3]};
    return RotateBodyToWorld(conjugate, vector);
}

inline DynamicAccelerationTargetResult BuildDynamicAccelerationTarget(
    const DynamicAccelerationTargetInput &input,
    const DynamicAccelerationTargetOptions &options,
    const std::array<double, 3> *previous_applied_correction_world,
    double dt_s)
{
    DynamicAccelerationTargetResult result;
    if (!IsFinite(input.quaternion) ||
        !IsFinite(input.measured_acceleration_world) ||
        !IsFinite(input.mass_qacc_qcoord) ||
        !IsFinite(input.base_mass_matrix_qcoord) ||
        !IsFinite(input.smooth_qcoord) ||
        !std::isfinite(input.desired_force_x_n) ||
        !std::isfinite(input.mass_kg) ||
        input.mass_kg <= 0.0 ||
        !std::isfinite(options.correction_limit_mps2) ||
        options.correction_limit_mps2 < 0.0 ||
        !std::isfinite(options.correction_slew_limit_mps3) ||
        options.correction_slew_limit_mps3 < 0.0)
    {
        return result;
    }

    const std::array<double, 3> desired_acceleration_body = {
        input.desired_force_x_n / input.mass_kg, 0.0, 0.0};
    const std::array<double, 3> desired_acceleration_world =
        RotateBodyToWorld(input.quaternion, desired_acceleration_body);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result.raw_correction_world[axis] =
            desired_acceleration_world[axis] -
            input.measured_acceleration_world[axis];
        result.applied_correction_world[axis] =
            std::max(
                -options.correction_limit_mps2,
                std::min(
                    options.correction_limit_mps2,
                    result.raw_correction_world[axis]));
    }

    if (previous_applied_correction_world != nullptr &&
        IsFinite(*previous_applied_correction_world))
    {
        if (std::isfinite(dt_s) && dt_s <= 1e-12)
        {
            result.applied_correction_world =
                *previous_applied_correction_world;
            result.reference_held_for_duplicate_time = true;
        }
        else if (std::isfinite(dt_s) && dt_s > 0.0)
        {
            const double max_step =
                options.correction_slew_limit_mps3 * dt_s;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const double previous =
                    (*previous_applied_correction_world)[axis];
                const double lower =
                    previous - max_step;
                const double upper =
                    previous + max_step;
                const double limited =
                    std::max(
                        lower,
                        std::min(upper, result.applied_correction_world[axis]));
                if (limited != result.applied_correction_world[axis])
                    result.slew_limited = true;
                result.applied_correction_world[axis] = limited;
            }
        }
    }

    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result.correction_slack_world[axis] =
            result.raw_correction_world[axis] -
            result.applied_correction_world[axis];
    }

    std::array<double, 6> target_qforce{};
    for (std::size_t row = 0; row < 6; ++row)
    {
        target_qforce[row] =
            input.mass_qacc_qcoord[row] - input.smooth_qcoord[row];
        for (std::size_t column = 0; column < 3; ++column)
        {
            target_qforce[row] +=
                input.base_mass_matrix_qcoord[row * 6 + column] *
                result.applied_correction_world[column];
        }
    }
    const std::array<double, 3> target_force_world = {
        target_qforce[0], target_qforce[1], target_qforce[2]};
    const std::array<double, 3> target_moment_world = {
        target_qforce[3], target_qforce[4], target_qforce[5]};
    const std::array<double, 3> target_force_body =
        RotateWorldToBody(input.quaternion, target_force_world);
    const std::array<double, 3> target_moment_body =
        RotateWorldToBody(input.quaternion, target_moment_world);
    result.desired_wrench_body = {
        target_force_body[0],
        target_force_body[1],
        target_force_body[2],
        target_moment_body[0],
        target_moment_body[1],
        target_moment_body[2]};
    result.valid = IsFinite(result.desired_wrench_body);
    return result;
}

} // namespace go2_control
