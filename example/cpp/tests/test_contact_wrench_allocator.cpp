#include <cmath>
#include <iostream>
#include <limits>

#include "contact_wrench_allocator.h"

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
    go2_control::ContactWrenchRequest request;
    request.contact_positions_body = {
        go2::Vec3{0.22, -0.12, -0.30},
        go2::Vec3{0.22, 0.12, -0.30},
        go2::Vec3{-0.22, -0.12, -0.30},
        go2::Vec3{-0.22, 0.12, -0.30}};
    request.contact.fill(true);
    request.desired_wrench = {0.0, 0.0, 120.0, 0.0, 0.0, 0.0};

    go2_control::ContactWrenchLeastNormAllocator allocator;
    go2_control::ContactWrenchSolution solution;
    bool ok = allocator.Solve(request, solution);
    bool passed = true;
    passed &= Check(ok, "Four-contact vertical wrench solve failed.");
    passed &= Check(
        solution.active_contacts == 4,
        "Four-contact active contact count is wrong.");
    passed &= Check(
        solution.residual_norm < 1e-5,
        "Four-contact wrench residual is too large.");

    double total_force_z = 0.0;
    for (const go2::Vec3 &force : solution.forces)
    {
        passed &= Check(
            Near(force.x, 0.0, 1e-5) &&
                Near(force.y, 0.0, 1e-5) &&
                Near(force.z, 30.0, 1e-5),
            "Symmetric vertical wrench was not evenly distributed.");
        total_force_z += force.z;
    }
    passed &= Check(
        Near(total_force_z, 120.0, 1e-5),
        "Total vertical force is not conserved.");

    request.desired_wrench = {12.0, -4.0, 120.0, 1.8, -2.4, 0.6};
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "General four-contact wrench solve failed.");
    passed &= Check(
        solution.residual_norm < 1e-5,
        "General four-contact wrench residual is too large.");

    request.contact[3] = false;
    ok = allocator.Solve(request, solution);
    passed &= Check(ok, "Three-contact wrench solve failed.");
    passed &= Check(
        solution.active_contacts == 3,
        "Three-contact active contact count is wrong.");

    request.contact.fill(false);
    ok = allocator.Solve(request, solution);
    passed &= Check(
        !ok,
        "Allocator accepted a request with fewer than two contacts.");

    request.contact.fill(true);
    request.desired_wrench[0] = std::numeric_limits<double>::quiet_NaN();
    ok = allocator.Solve(request, solution);
    passed &= Check(
        !ok,
        "Allocator accepted a non-finite desired wrench.");

    if (!passed)
        return 1;
    std::cout << "Contact wrench allocator checks passed.\n";
    return 0;
}

