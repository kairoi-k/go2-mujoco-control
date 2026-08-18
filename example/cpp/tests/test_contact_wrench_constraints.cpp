#include <iostream>

#include "contact_wrench_constraints.h"

int main()
{
    go2_control::ContactWrenchRequest request;
    request.contact.fill(true);
    std::array<go2::Vec3, go2::kLegCount> feasible_forces{};
    for (go2::Vec3 &force : feasible_forces)
        force.z = 30.0;

    go2_control::ContactForceConstraintParams params;
    go2_control::ContactForceConstraintReport report;
    bool passed = true;
    passed &= go2_control::EvaluateContactForceConstraints(
        request, feasible_forces, params, report);
    passed &= report.feasible;
    passed &= report.violation_count == 0;

    auto friction_violation = feasible_forces;
    friction_violation[0] = {20.0, 0.0, 10.0};
    passed &= go2_control::EvaluateContactForceConstraints(
        request, friction_violation, params, report);
    passed &= !report.feasible;
    passed &= report.max_violation > 14.9;
    auto diagonal_friction_violation = feasible_forces;
    diagonal_friction_violation[0] = {12.0, 12.0, 30.0};
    passed &= go2_control::EvaluateContactForceConstraints(
        request, diagonal_friction_violation, params, report);
    passed &= !report.feasible;
    passed &= report.max_violation > 1.9;

    auto negative_normal = feasible_forces;
    negative_normal[1].z = -0.1;
    passed &= go2_control::EvaluateContactForceConstraints(
        request, negative_normal, params, report);
    passed &= !report.feasible;
    passed &= report.max_violation > 0.09;

    request.contact[2] = false;
    auto nonzero_swing_force = feasible_forces;
    passed &= go2_control::EvaluateContactForceConstraints(
        request, nonzero_swing_force, params, report);
    passed &= !report.feasible;

    params.friction_coefficient = -0.1;
    passed &= !go2_control::EvaluateContactForceConstraints(
        request, feasible_forces, params, report);

    if (!passed)
    {
        std::cerr << "Contact force constraint checks failed.\n";
        return 1;
    }
    std::cout << "Contact force constraint checks passed.\n";
    return 0;
}
