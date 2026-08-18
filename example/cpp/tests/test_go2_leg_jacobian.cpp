#include <cmath>
#include <iostream>

#include "go2_forward_kinematics.h"
#include "go2_leg_jacobian.h"

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
    constexpr double kEpsilon = 1e-6;
    constexpr double kTolerance = 2e-7;
    bool passed = true;

    for (std::size_t leg_index = 0;
         leg_index < go2::kLegCount;
         ++leg_index)
    {
        const auto leg = static_cast<go2::Leg>(leg_index);
        const std::array<double, 3> q = {
            0.12 + 0.03 * static_cast<double>(leg_index),
            0.61 + 0.02 * static_cast<double>(leg_index),
            -1.24 - 0.01 * static_cast<double>(leg_index)};
        const go2_control::LegFootJacobian jacobian =
            go2_control::FootJacobian(leg, q[0], q[1], q[2]);

        for (std::size_t joint = 0; joint < 3; ++joint)
        {
            std::array<double, 3> q_plus = q;
            std::array<double, 3> q_minus = q;
            q_plus[joint] += kEpsilon;
            q_minus[joint] -= kEpsilon;
            const go2::Vec3 plus = go2::FootPosition(
                leg, q_plus[0], q_plus[1], q_plus[2]);
            const go2::Vec3 minus = go2::FootPosition(
                leg, q_minus[0], q_minus[1], q_minus[2]);
            const std::array<double, 3> numerical = {
                (plus.x - minus.x) / (2.0 * kEpsilon),
                (plus.y - minus.y) / (2.0 * kEpsilon),
                (plus.z - minus.z) / (2.0 * kEpsilon)};
            for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
            {
                passed &= Check(
                    Near(
                        jacobian[coordinate][joint],
                        numerical[coordinate],
                        kTolerance),
                    "Analytic Go2 foot Jacobian disagrees with finite difference.");
            }
        }
    }

    if (!passed)
        return 1;
    std::cout << "Go2 leg Jacobian checks passed.\n";
    return 0;
}

