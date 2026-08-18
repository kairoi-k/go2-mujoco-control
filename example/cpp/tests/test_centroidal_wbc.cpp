#include <cmath>
#include <iostream>

#include "centroidal_wbc.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckIdentityMassNoBias()
{
    go2_control::CentroidalMass mass{};
    for (int i = 0; i < 6; ++i)
        mass.mass_matrix[static_cast<std::size_t>(i * 6 + i)] = 1.0;
    mass.include_bias = false;
    go2_control::CentroidalTask task{};
    task.desired_acc = {1.5, -0.25, 0.5, 0.1, -0.2, 0.05};
    go2_control::CentroidalWrench out{};
    if (!go2_control::BuildCentroidalWrench(mass, task, out) || !out.valid)
        return false;
    for (int i = 0; i < 6; ++i)
    {
        if (!Near(out.wrench[static_cast<std::size_t>(i)],
                  task.desired_acc[static_cast<std::size_t>(i)]))
            return false;
    }
    return true;
}

bool CheckBiasIsAddedWhenRequested()
{
    go2_control::CentroidalMass mass{};
    for (int i = 0; i < 6; ++i)
        mass.mass_matrix[static_cast<std::size_t>(i * 6 + i)] = 2.0;
    mass.bias = {0.0, 0.0, 149.1, 0.0, 0.0, 0.0};
    mass.include_bias = true;
    go2_control::CentroidalTask task{};
    task.desired_acc = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    go2_control::CentroidalWrench out{};
    return go2_control::BuildCentroidalWrench(mass, task, out) &&
           out.valid &&
           Near(out.wrench[2], 2.0 + 149.1);
}

bool CheckBiasCanBeDropped()
{
    go2_control::CentroidalMass mass{};
    for (int i = 0; i < 6; ++i)
        mass.mass_matrix[static_cast<std::size_t>(i * 6 + i)] = 2.0;
    mass.bias = {0.0, 0.0, 149.1, 0.0, 0.0, 0.0};
    mass.include_bias = false;
    go2_control::CentroidalTask task{};
    task.desired_acc = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    go2_control::CentroidalWrench out{};
    return go2_control::BuildCentroidalWrench(mass, task, out) &&
           Near(out.wrench[2], 2.0);
}

bool CheckNonFiniteRejected()
{
    go2_control::CentroidalMass mass{};
    go2_control::CentroidalTask task{};
    task.desired_acc[0] = std::nan("");
    go2_control::CentroidalWrench out{};
    return !go2_control::BuildCentroidalWrench(mass, task, out);
}

}  // namespace

int main()
{
    if (!CheckIdentityMassNoBias() ||
        !CheckBiasIsAddedWhenRequested() ||
        !CheckBiasCanBeDropped() ||
        !CheckNonFiniteRejected())
    {
        std::cerr << "centroidal WBC checks failed\n";
        return 1;
    }
    std::cout << "centroidal WBC checks passed.\n";
    return 0;
}
