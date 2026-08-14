#include <cmath>
#include <iostream>

#include "dynamic_acceleration_target.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

} // namespace

int main()
{
    go2_control::DynamicAccelerationTargetInput input;
    input.quaternion = {1.0, 0.0, 0.0, 0.0};
    input.measured_acceleration_world = {1.0, -2.0, 8.0};
    input.mass_qacc_qcoord.fill(0.0);
    input.base_mass_matrix_qcoord.fill(0.0);
    input.smooth_qcoord.fill(0.0);
    for (std::size_t axis = 0; axis < 3; ++axis)
        input.base_mass_matrix_qcoord[axis * 6 + axis] = 1.0;
    input.desired_force_x_n = 4.0;
    input.mass_kg = 1.0;

    go2_control::DynamicAccelerationTargetOptions options;
    options.correction_limit_mps2 = 1.0;
    options.correction_slew_limit_mps3 = 2.0;

    bool passed = true;
    auto result = go2_control::BuildDynamicAccelerationTarget(
        input, options, nullptr, 0.0);
    passed &= Check(result.valid, "Initial bounded acceleration target invalid.");
    passed &= Check(
        result.applied_correction_world == std::array<double, 3>{1.0, 1.0, -1.0},
        "Acceleration correction limit was not applied.");
    passed &= Check(
        result.correction_slack_world == std::array<double, 3>{2.0, 1.0, -7.0},
        "Acceleration correction slack was not recorded.");

    const std::array<double, 3> previous = {0.0, 0.0, 0.0};
    result = go2_control::BuildDynamicAccelerationTarget(
        input, options, &previous, 0.1);
    passed &= Check(
        result.slew_limited &&
            result.applied_correction_world ==
                std::array<double, 3>{0.2, 0.2, -0.2},
        "Acceleration slew limit was not applied.");

    result = go2_control::BuildDynamicAccelerationTarget(
        input, options, &result.applied_correction_world, 0.0);
    passed &= Check(
        result.reference_held_for_duplicate_time &&
            result.applied_correction_world ==
                std::array<double, 3>{0.2, 0.2, -0.2},
        "Duplicate timestamp did not hold the acceleration reference.");

    if (!passed)
        return 1;
    std::cout << "Dynamic acceleration target checks passed.\n";
    return 0;
}
