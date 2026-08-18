#include <cmath>
#include <iostream>

#include "contact_wrench_lexicographic_allocator.h"

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
    go2_control::LexicographicContactWrenchRequest request;
    request.wrench.contact_positions_body = {
        go2::Vec3{0.22, -0.12, -0.30},
        go2::Vec3{0.22, 0.12, -0.30},
        go2::Vec3{-0.22, -0.12, -0.30},
        go2::Vec3{-0.22, 0.12, -0.30}};
    request.wrench.contact.fill(true);
    request.wrench.desired_wrench = {0.0, 0.0, 120.0, 0.0, 0.0, 0.0};
    request.force_constraints.friction_coefficient = 0.5;
    request.force_constraints.max_normal_force = 100.0;
    request.force_tolerance = 0.01;
    request.moment_slack_limit = 0.05;
    request.residual_tolerance = 1e-5;

    go2_control::ContactWrenchLexicographicSlackAllocator allocator;
    go2_control::LexicographicContactWrenchSolution solution;
    bool passed = true;

    bool ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Four-contact slack solve failed.");
    passed &= Check(
        solution.policy_satisfied &&
            solution.constraint_report.feasible &&
            solution.moment_task_active,
        "Four-contact force-priority policy was not satisfied.");
    passed &= Check(
        solution.max_force_excess <= 1e-5 &&
            solution.max_moment_excess <= 1e-5,
        "Four-contact slack bounds were exceeded.");

    request.wrench.desired_wrench = {0.0, 0.0, 120.0, 0.04, 0.0, 0.0};
    ok = allocator.Solve(request, solution);
    passed &= Check(
        ok && solution.policy_satisfied &&
            std::abs(solution.moment_slack[0]) <= 0.05 + 1e-9,
        "Moment slack was not represented explicitly.");

    request.wrench.contact = {true, true, false, false};
    request.wrench.desired_wrench = {0.0, 0.0, 90.0, 0.0, 0.0, 0.0};
    request.moment_task_active = false;
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Two-contact force-priority solve failed.");
    passed &= Check(
        solution.policy_satisfied &&
            !solution.moment_task_active &&
            solution.max_moment_excess == 0.0,
        "Inactive two-contact moment task was not handled as a slack policy.");

    request.wrench.contact.fill(true);
    request.moment_task_active = true;
    request.wrench.desired_wrench = {0.0, 0.0, 120.0, 0.0, 0.0, 0.0};
    request.torque_rate_task_active = true;
    for (auto &angles : request.joint_angles)
        angles = {0.2, 0.62, -1.2};
    request.previous_torque.fill(0.0);
    request.dt_s = 0.002;
    request.torque_rate_limit_nm_s = 100000.0;
    ok = allocator.Solve(request, solution);
    passed &= Check(
        ok && solution.torque_rate_task_active &&
            solution.torque_rate_satisfied && solution.policy_satisfied,
        "Large torque-rate bound was not accepted.");

    request.torque_rate_limit_nm_s = 0.0;
    request.previous_torque.fill(1000.0);
    ok = allocator.Solve(request, solution);
    passed &= Check(
        ok && !solution.torque_rate_satisfied &&
            solution.max_torque_rate_excess > 1.0,
        "Torque-rate violation was not diagnosed.");
    request.wrench.desired_wrench = {200.0, 0.0, 40.0, 0.0, 0.0, 0.0};
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Infeasible force target did not return diagnostics.");
    passed &= Check(
        !solution.policy_satisfied &&
            solution.max_force_excess > 1.0,
        "Infeasible force target was incorrectly accepted.");
    passed &= Check(
        solution.constraint_report.feasible,
        "Infeasible target violated the force projection constraints.");

    if (!passed)
        return 1;
    std::cout << "Lexicographic slack allocator checks passed.\n";
    return 0;
}
