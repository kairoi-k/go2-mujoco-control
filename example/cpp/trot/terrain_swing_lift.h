#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace go2_trot
{

// The terrain executor adds only the lift above the gait kernel's current
// swing lift. In runtime-command mode, kernel_lift_m is the value after
// kernel-side slew/adaptation for this Compute tick; the CLI value is only the
// initial kernel parameter and must not dominate it. Keep the historical
// max(CLI, runtime) convention for non-runtime gait modes.
inline double TerrainKernelNominalLift(
    bool runtime_velocity_command,
    double cli_lift_m,
    double runtime_gait_lift_m,
    double kernel_lift_m) noexcept
{
    if (runtime_velocity_command)
        return std::isfinite(kernel_lift_m) && kernel_lift_m >= 0.0
            ? kernel_lift_m : std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(cli_lift_m) || cli_lift_m < 0.0 ||
        !std::isfinite(runtime_gait_lift_m) || runtime_gait_lift_m < 0.0)
        return std::numeric_limits<double>::quiet_NaN();
    return std::max(cli_lift_m, runtime_gait_lift_m);
}

inline double TerrainExtraLift(
    double target_lift_m, double nominal_lift_m) noexcept
{
    if (!std::isfinite(target_lift_m) || target_lift_m < 0.0 ||
        !std::isfinite(nominal_lift_m) || nominal_lift_m < 0.0)
        return std::numeric_limits<double>::quiet_NaN();
    return std::max(0.0, target_lift_m - nominal_lift_m);
}

}  // namespace go2_trot
