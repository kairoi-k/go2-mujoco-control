#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "go2_leg_jacobian.h"

namespace go2_control
{

using JointAngles =
    std::array<std::array<double, 3>, go2::kLegCount>;
using ContactForces =
    std::array<go2::Vec3, go2::kLegCount>;
using JointTorques =
    std::array<std::array<double, 3>, go2::kLegCount>;

struct ContactTorqueMapRequest
{
    JointAngles joint_angles{};
    ContactForces contact_forces{};
    std::array<bool, go2::kLegCount> contact{};
    double zero_force_tolerance = 1e-9;
};

struct ContactTorqueMapSolution
{
    JointTorques torques{};
    double max_abs_torque = 0.0;
};

inline bool MapContactForcesToJointTorques(
    const ContactTorqueMapRequest &request,
    ContactTorqueMapSolution &solution)
{
    solution = ContactTorqueMapSolution{};
    if (!std::isfinite(request.zero_force_tolerance) ||
        request.zero_force_tolerance < 0.0)
    {
        return false;
    }

    for (std::size_t leg_index = 0;
         leg_index < go2::kLegCount;
         ++leg_index)
    {
        const auto &angles = request.joint_angles[leg_index];
        const go2::Vec3 &force = request.contact_forces[leg_index];
        if (!std::isfinite(angles[0]) ||
            !std::isfinite(angles[1]) ||
            !std::isfinite(angles[2]) ||
            !std::isfinite(force.x) ||
            !std::isfinite(force.y) ||
            !std::isfinite(force.z))
        {
            return false;
        }

        if (!request.contact[leg_index])
        {
            const double max_force_component = std::max(
                std::abs(force.x),
                std::max(std::abs(force.y), std::abs(force.z)));
            if (max_force_component > request.zero_force_tolerance)
                return false;
            continue;
        }

        const std::array<double, 3> force_components = {
            force.x, force.y, force.z};
        const LegFootJacobian jacobian = FootJacobian(
            static_cast<go2::Leg>(leg_index),
            angles[0],
            angles[1],
            angles[2]);
        for (std::size_t joint = 0; joint < 3; ++joint)
        {
            double torque = 0.0;
            for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
                torque += jacobian[coordinate][joint] * force_components[coordinate];
            if (!std::isfinite(torque))
                return false;
            solution.torques[leg_index][joint] = torque;
            solution.max_abs_torque = std::max(
                solution.max_abs_torque, std::abs(torque));
        }
    }

    return std::isfinite(solution.max_abs_torque);
}

struct JointTorqueLimitParams
{
    // Limits are per joint type [hip, thigh, calf], applied to each leg.
    std::array<double, 3> max_abs_torque{};
    double tolerance = 1e-9;
};

struct JointTorqueLimitReport
{
    bool valid = false;
    bool within_limits = false;
    int violation_count = 0;
    double max_abs_torque = 0.0;
    double max_violation = 0.0;
};

inline bool EvaluateJointTorqueLimits(
    const JointTorques &torques,
    const JointTorqueLimitParams &params,
    JointTorqueLimitReport &report)
{
    report = JointTorqueLimitReport{};
    if (!std::isfinite(params.tolerance) || params.tolerance < 0.0)
        return false;
    for (double limit : params.max_abs_torque)
    {
        if (!std::isfinite(limit) || limit <= 0.0)
            return false;
    }

    for (const auto &leg_torques : torques)
    {
        for (std::size_t joint = 0; joint < 3; ++joint)
        {
            const double torque = leg_torques[joint];
            if (!std::isfinite(torque))
                return false;
            const double absolute_torque = std::abs(torque);
            report.max_abs_torque = std::max(
                report.max_abs_torque, absolute_torque);
            const double violation =
                absolute_torque - params.max_abs_torque[joint];
            if (violation > params.tolerance)
            {
                ++report.violation_count;
                report.max_violation = std::max(
                    report.max_violation, violation);
            }
        }
    }

    report.valid = true;
    report.within_limits = report.violation_count == 0;
    return true;
}

} // namespace go2_control
