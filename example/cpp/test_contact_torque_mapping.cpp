#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include "go2_contact_torque_mapping.h"

namespace
{

bool Near(double actual, double expected, double tolerance)
{
    return std::abs(actual - expected) <= tolerance;
}

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

} // namespace

int main()
{
    constexpr double kTolerance = 1e-9;
    go2_control::ContactTorqueMapRequest request;
    request.contact.fill(false);
    request.contact[0] = true;
    request.joint_angles[0] = {0.20, 0.62, -1.21};
    request.contact_forces[0] = {1.2, -2.0, 30.0};

    go2_control::ContactTorqueMapSolution solution;
    bool passed = true;
    bool ok = go2_control::MapContactForcesToJointTorques(
        request, solution);
    passed &= Check(ok, "Contact force to torque mapping failed.");

    const go2_control::LegFootJacobian jacobian =
        go2_control::FootJacobian(
            go2::Leg::FR,
            request.joint_angles[0][0],
            request.joint_angles[0][1],
            request.joint_angles[0][2]);
    const std::array<double, 3> force_components = {
        request.contact_forces[0].x,
        request.contact_forces[0].y,
        request.contact_forces[0].z};
    for (std::size_t joint = 0; joint < 3; ++joint)
    {
        double expected = 0.0;
        for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
        {
            expected += jacobian[coordinate][joint] *
                        force_components[coordinate];
        }
        passed &= Check(
            Near(solution.torques[0][joint], expected, kTolerance),
            "Mapped joint torque disagrees with J transpose times force.");
    }

    const std::array<double, 3> joint_velocity = {0.3, -0.4, 0.2};
    std::array<double, 3> foot_velocity{};
    for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
    {
        for (std::size_t joint = 0; joint < 3; ++joint)
            foot_velocity[coordinate] +=
                jacobian[coordinate][joint] * joint_velocity[joint];
    }
    double torque_power = 0.0;
    for (std::size_t joint = 0; joint < 3; ++joint)
        torque_power += solution.torques[0][joint] * joint_velocity[joint];
    const double force_power =
        request.contact_forces[0].x * foot_velocity[0] +
        request.contact_forces[0].y * foot_velocity[1] +
        request.contact_forces[0].z * foot_velocity[2];
    passed &= Check(
        Near(torque_power, force_power, 1e-12),
        "Torque and contact-force virtual work do not agree.");

    go2_control::JointTorqueLimitParams limits;
    limits.max_abs_torque = {100.0, 100.0, 100.0};
    go2_control::JointTorqueLimitReport limit_report;
    ok = go2_control::EvaluateJointTorqueLimits(
        solution.torques, limits, limit_report);
    passed &= Check(ok && limit_report.valid, "Torque limit report is invalid.");
    passed &= Check(
        limit_report.within_limits,
        "Conservative torque limits rejected a valid mapping.");

    double largest_torque = 0.0;
    std::size_t largest_joint = 0;
    for (const auto &leg_torques : solution.torques)
    {
        for (std::size_t joint = 0; joint < 3; ++joint)
        {
            if (std::abs(leg_torques[joint]) > largest_torque)
            {
                largest_torque = std::abs(leg_torques[joint]);
                largest_joint = joint;
            }
        }
    }
    passed &= Check(largest_torque > 1e-6, "Torque probe was unexpectedly zero.");
    limits.max_abs_torque[largest_joint] = 0.5 * largest_torque;
    ok = go2_control::EvaluateJointTorqueLimits(
        solution.torques, limits, limit_report);
    passed &= Check(ok, "Torque violation report rejected valid limits.");
    passed &= Check(
        !limit_report.within_limits &&
            limit_report.violation_count >= 1 &&
            limit_report.max_violation > 0.0,
        "Torque limit violation was not reported.");

    request.contact_forces[1] = {1.0, 0.0, 0.0};
    ok = go2_control::MapContactForcesToJointTorques(
        request, solution);
    passed &= Check(
        !ok,
        "Mapping accepted a nonzero swing-leg contact force.");

    request.contact_forces[1] = {};
    request.joint_angles[0][1] = std::numeric_limits<double>::quiet_NaN();
    ok = go2_control::MapContactForcesToJointTorques(
        request, solution);
    passed &= Check(!ok, "Mapping accepted non-finite joint angles.");

    limits.max_abs_torque = {100.0, 100.0, 100.0};
    limits.tolerance = -1.0;
    ok = go2_control::EvaluateJointTorqueLimits(
        solution.torques, limits, limit_report);
    passed &= Check(!ok, "Torque checker accepted negative tolerance.");

    if (!passed)
        return 1;
    std::cout << "Contact torque mapping checks passed.\n";
    return 0;
}
