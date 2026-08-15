#pragma once

// Centroidal wrench W = M a + h.
// The incremental path in this repo historically dropped h (gravity/bias) so
// the position servo could own it. The full path keeps h and expects stance
// kp to be lowered.

#include <array>
#include <cmath>

namespace go2_control
{

struct CentroidalMass
{
    std::array<double, 36> mass_matrix{};
    std::array<double, 6> bias{};
    bool include_bias = true;
};

struct CentroidalTask
{
    std::array<double, 6> desired_acc{};
};

struct CentroidalWrench
{
    std::array<double, 6> wrench{};
    bool valid = false;
};

inline bool BuildCentroidalWrench(
    const CentroidalMass &mass,
    const CentroidalTask &task,
    CentroidalWrench &out)
{
    out = CentroidalWrench{};
    for (double value : mass.mass_matrix)
    {
        if (!std::isfinite(value))
            return false;
    }
    for (double value : mass.bias)
    {
        if (!std::isfinite(value))
            return false;
    }
    for (double value : task.desired_acc)
    {
        if (!std::isfinite(value))
            return false;
    }

    for (int row = 0; row < 6; ++row)
    {
        double wrench = mass.include_bias ? mass.bias[static_cast<std::size_t>(row)] : 0.0;
        for (int col = 0; col < 6; ++col)
        {
            wrench += mass.mass_matrix[static_cast<std::size_t>(row * 6 + col)] *
                      task.desired_acc[static_cast<std::size_t>(col)];
        }
        if (!std::isfinite(wrench))
            return false;
        out.wrench[static_cast<std::size_t>(row)] = wrench;
    }
    out.valid = true;
    return true;
}

}  // namespace go2_control
