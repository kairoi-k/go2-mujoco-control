#include <cmath>
#include <iostream>

#include "contact_wrench_projected_allocator.h"

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
    go2_control::ProjectedContactWrenchRequest request;
    request.wrench.contact_positions_body = {
        go2::Vec3{0.22, -0.12, -0.30},
        go2::Vec3{0.22, 0.12, -0.30},
        go2::Vec3{-0.22, -0.12, -0.30},
        go2::Vec3{-0.22, 0.12, -0.30}};
    request.wrench.contact.fill(true);
    request.wrench.desired_wrench = {0.0, 0.0, 120.0, 0.0, 0.0, 0.0};
    request.force_constraints.friction_coefficient = 0.5;
    request.force_constraints.max_normal_force = 100.0;
    request.residual_tolerance = 1e-5;

    go2_control::ContactWrenchProjectedAllocator allocator;
    go2_control::ProjectedContactWrenchSolution solution;
    bool passed = true;
    bool ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Projected four-contact solve failed.");
    passed &= Check(
        solution.converged && solution.wrench_satisfied,
        "Projected vertical wrench did not converge.");
    passed &= Check(
        solution.constraint_report.feasible,
        "Projected vertical wrench violates force constraints.");
    passed &= Check(
        solution.residual_norm < 1e-5,
        "Projected vertical wrench residual is too large.");

    double total_force_z = 0.0;
    for (const go2::Vec3 &force : solution.forces)
    {
        passed &= Check(
            Near(force.x, 0.0, 1e-7) &&
                Near(force.y, 0.0, 1e-7),
            "Projected vertical wrench has horizontal force.");
        total_force_z += force.z;
    }
    passed &= Check(
        Near(total_force_z, 120.0, 1e-6),
        "Projected vertical wrench does not conserve total normal force.");
    passed &= Check(
        Near(solution.max_axis_friction_ratio, 0.0, 1e-9) &&
            Near(solution.max_radial_friction_ratio, 0.0, 1e-9),
        "Projected vertical wrench has unexpected friction demand.");
    passed &= Check(
        solution.min_contact_normal_force > 0.0,
        "Projected vertical wrench has no positive contact normal force.");

    request.wrench.desired_wrench = {200.0, 0.0, 40.0, 0.0, 0.0, 0.0};
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Infeasible target should return a diagnostic solution.");
    passed &= Check(
        solution.constraint_report.feasible,
        "Infeasible-target solution violates force constraints.");
    passed &= Check(
        !solution.wrench_satisfied && solution.residual_norm > 100.0,
        "Infeasible target was incorrectly reported as satisfied.");
    passed &= Check(
        solution.max_radial_friction_ratio <= 1.0 + 1e-9,
        "Projected infeasible target exceeds radial friction cone.");

    request.wrench.contact[3] = false;
    request.wrench.desired_wrench = {0.0, 0.0, 90.0, 0.0, 0.0, 0.0};
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Projected three-contact solve failed.");
    passed &= Check(
        solution.active_contacts == 3 &&
            solution.wrench_satisfied &&
            solution.constraint_report.feasible,
        "Projected three-contact result is not valid.");

    request.wrench.contact = {true, false, false, true};
    request.wrench.desired_wrench = {10.0, 0.0, 90.0, 0.0, 0.0, 0.0};
    request.wrench.task_weights.fill(1.0);
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Projected two-contact full-task solve failed.");
    passed &= Check(
        !solution.wrench_satisfied,
        "Projected two-contact full wrench was unexpectedly satisfied.");
    request.wrench.task_weights = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
    ok = allocator.Solve(request, solution);
    passed &= Check(
        ok && solution.task_satisfied && solution.constraint_report.feasible,
        "Reduced three-force task was not satisfied.");
    passed &= Check(
        solution.task_residual_norm < 1e-5,
        "Reduced contact task residual is too large.");
    request.force_constraints.friction_coefficient = -0.1;
    ok = allocator.Solve(request, solution);
    passed &= Check(!ok, "Projected allocator accepted invalid constraints.");
    request.force_constraints.friction_coefficient = 0.5;
    request.wrench.task_weights.fill(0.0);
    ok = allocator.Solve(request, solution);
    passed &= Check(!ok, "Projected allocator accepted empty task weights.");

    if (!passed)
        return 1;
    std::cout << "Projected contact wrench allocator checks passed.\n";
    return 0;
}
