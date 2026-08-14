#include <array>
#include <cmath>
#include <iostream>

#include "go2_inverse_kinematics.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckRoundTrip(const std::array<double, go2::kJointCount> &source_joints)
{
    const auto source_feet = go2::AllFootPositions(source_joints);
    std::array<double, go2::kJointCount> solved_joints{};
    if (!go2::AllLegInverseKinematics(source_feet, solved_joints))
    {
        std::cerr << "Inverse kinematics rejected a reachable pose\n";
        return false;
    }

    const auto solved_feet = go2::AllFootPositions(solved_joints);
    for (std::size_t i = 0; i < source_feet.size(); ++i)
    {
        if (!Near(source_feet[i].x, solved_feet[i].x) ||
            !Near(source_feet[i].y, solved_feet[i].y) ||
            !Near(source_feet[i].z, solved_feet[i].z))
        {
            std::cerr << "FK/IK round-trip failed for leg " << i << "\n";
            return false;
        }
    }
    return true;
}

bool CheckBodyShiftTargets()
{
    const std::array<double, go2::kJointCount> stand_pose = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};
    const auto stand_feet = go2::AllFootPositions(stand_pose);

    for (const go2::Vec3 body_shift : {
             go2::Vec3{-0.01, 0.01, 0.0},
             go2::Vec3{-0.02, 0.015, 0.0},
             go2::Vec3{-0.03, 0.02, 0.0}})
    {
        auto target_feet = stand_feet;
        for (auto &foot : target_feet)
        {
            foot.x -= body_shift.x;
            foot.y -= body_shift.y;
            foot.z -= body_shift.z;
        }

        std::array<double, go2::kJointCount> target_joints{};
        if (!go2::AllLegInverseKinematics(target_feet, target_joints))
        {
            std::cerr << "IK rejected a planned body-shift target\n";
            return false;
        }
        if (!CheckRoundTrip(target_joints))
        {
            return false;
        }
    }
    return true;
}

} // namespace

int main()
{
    const std::array<double, go2::kJointCount> stand_pose = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};

    if (!CheckRoundTrip(stand_pose) || !CheckBodyShiftTargets())
    {
        return 1;
    }

    std::cout << "Inverse kinematics checks passed for stand and body-shift targets.\n";
    return 0;
}
