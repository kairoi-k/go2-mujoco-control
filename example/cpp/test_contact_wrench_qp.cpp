#include <cmath>
#include <iostream>

#include "contact_wrench_qp.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

}  // namespace

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
    request.wrench.task_weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    request.force_constraints.friction_coefficient = 0.5;
    request.force_constraints.max_normal_force = 100.0;

    go2_control::ContactWrenchQpAllocator allocator;
    go2_control::ProjectedContactWrenchSolution solution;
    bool passed = allocator.Solve(request, solution);
    passed &= Check(passed, "Four-contact QP solve failed.");
    passed &= Check(
        solution.constraint_report.feasible,
        "Four-contact QP was not friction-feasible.");
    double total_z = 0.0;
    for (const auto &force : solution.forces)
        total_z += force.z;
    passed &= Check(
        std::abs(total_z - 120.0) < 1.0,
        "Four-contact QP did not carry gravity.");
    passed &= Check(
        solution.min_contact_normal_force > 0.0,
        "Four-contact QP produced a non-positive normal.");

    request.wrench.contact = {true, false, false, true};
    request.wrench.desired_wrench = {0.0, 0.0, 90.0, 0.0, 0.0, 0.0};
    passed &= Check(
        allocator.Solve(request, solution) &&
            solution.constraint_report.feasible &&
            solution.forces[1].z == 0.0 &&
            solution.forces[2].z == 0.0,
        "Diagonal two-contact QP failed.");

    if (!passed)
    {
        std::cerr << "contact wrench QP checks failed\n";
        return 1;
    }
    std::cout << "contact wrench QP checks passed.\n";
    return 0;
}
