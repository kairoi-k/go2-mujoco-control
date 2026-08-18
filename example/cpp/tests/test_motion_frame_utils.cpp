#include <cmath>
#include <iostream>
#include <limits>

#include "motion_frame_utils.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckIdentity()
{
    go2_control::Vector3 body{};
    return go2_control::WorldToBodyVelocity(
               {1.0, 0.0, 0.0, 0.0},
               {0.1, -0.2, 0.3}, body) &&
           Near(body[0], 0.1) && Near(body[1], -0.2) &&
           Near(body[2], 0.3);
}

bool CheckYawRotation()
{
    const double half_pi_over_two = 0.25 * 3.14159265358979323846;
    const double c = std::cos(half_pi_over_two);
    const double s = std::sin(half_pi_over_two);
    go2_control::Vector3 body{};
    return go2_control::WorldToBodyVelocity(
               {c, 0.0, 0.0, s}, {0.0, 1.0, 0.0}, body) &&
           Near(body[0], 1.0) && Near(body[1], 0.0) &&
           Near(body[2], 0.0);
}

bool CheckFullQuaternionAndInvalidInput()
{
    const double half_roll = 0.25 * 3.14159265358979323846;
    const double c = std::cos(half_roll);
    const double s = std::sin(half_roll);
    const go2_control::Vector3 expected_body{0.1, 0.2, -0.3};
    const go2_control::Vector3 world_velocity{0.1, 0.3, 0.2};
    go2_control::Vector3 body{};
    if (!go2_control::WorldToBodyVelocity(
            {c, s, 0.0, 0.0}, world_velocity, body) ||
        !Near(body[0], expected_body[0]) ||
        !Near(body[1], expected_body[1]) ||
        !Near(body[2], expected_body[2]))
    {
        return false;
    }
    const go2_control::Vector3 invalid_velocity{
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    return !go2_control::WorldToBodyVelocity(
               {0.0, 0.0, 0.0, 0.0}, invalid_velocity, body);
}

} // namespace

int main()
{
    if (!CheckIdentity() || !CheckYawRotation() ||
        !CheckFullQuaternionAndInvalidInput())
    {
        std::cerr << "Motion frame checks failed\n";
        return 1;
    }
    std::cout << "Motion frame checks passed.\n";
    return 0;
}
