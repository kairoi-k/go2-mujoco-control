#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "contact_wrench_allocator.h"

namespace go2_control
{

struct ContactForceConstraintParams
{
    double friction_coefficient = 0.5;
    double min_normal_force = 0.0;
    double max_normal_force = std::numeric_limits<double>::infinity();
    double tolerance = 1e-9;
};

struct ContactForceConstraintReport
{
    bool feasible = false;
    int violation_count = 0;
    double max_violation = std::numeric_limits<double>::infinity();
};

inline bool ValidateContactForceConstraintParams(
    const ContactForceConstraintParams &params)
{
    return std::isfinite(params.friction_coefficient) &&
           params.friction_coefficient >= 0.0 &&
           std::isfinite(params.min_normal_force) &&
           params.min_normal_force >= 0.0 &&
           (std::isfinite(params.max_normal_force) ||
            std::isinf(params.max_normal_force)) &&
           params.max_normal_force >= params.min_normal_force &&
           std::isfinite(params.tolerance) &&
           params.tolerance >= 0.0;
}

inline bool EvaluateContactForceConstraints(
    const ContactWrenchRequest &request,
    const std::array<go2::Vec3, go2::kLegCount> &forces,
    const ContactForceConstraintParams &params,
    ContactForceConstraintReport &report)
{
    report = ContactForceConstraintReport{};
    if (!ValidateContactForceConstraintParams(params))
        return false;

    report.max_violation = 0.0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const go2::Vec3 &force = forces[leg];
        if (!std::isfinite(force.x) ||
            !std::isfinite(force.y) ||
            !std::isfinite(force.z))
        {
            return false;
        }

        double violation = 0.0;
        if (!request.contact[leg])
        {
            violation = std::max(
                std::abs(force.x),
                std::max(std::abs(force.y), std::abs(force.z)));
        }
        else
        {
            violation = std::max(
                params.min_normal_force - force.z,
                force.z - params.max_normal_force);
            violation = std::max(
                violation,
                std::hypot(force.x, force.y) -
                    params.friction_coefficient * force.z);
        }

        if (violation > params.tolerance)
            ++report.violation_count;
        report.max_violation = std::max(report.max_violation, violation);
    }

    report.feasible =
        report.violation_count == 0 &&
        report.max_violation <= params.tolerance;
    return true;
}

} // namespace go2_control
