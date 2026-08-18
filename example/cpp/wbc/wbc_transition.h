#pragma once

// Small, deterministic helpers for the stand/settle/gait WBC handoff.
// They are intentionally independent of the controller so the transition
// contract can be unit-tested without DDS or MuJoCo.

#include <algorithm>
#include <cmath>

namespace go2_control
{

inline bool WbcPlantStageAllowed(bool wbc_full, int motion_stage)
{
    if (wbc_full)
        return motion_stage >= 1 && motion_stage <= 3;
    return motion_stage == 2;
}

inline bool WbcPlantStageRequiresFullSupport(int motion_stage)
{
    return motion_stage == 1 || motion_stage == 3;
}

inline double WbcSlewUnitBlend(
    double current, bool target, double dt_s,
    double rise_duration_s, double fall_duration_s)
{
    if (!std::isfinite(current))
        current = 0.0;
    current = std::clamp(current, 0.0, 1.0);
    if (!(dt_s > 0.0) || !std::isfinite(dt_s))
        return current;
    const double duration = target ? rise_duration_s : fall_duration_s;
    if (!(duration > 0.0) || !std::isfinite(duration))
        return target ? 1.0 : 0.0;
    const double step = std::clamp(dt_s / duration, 0.0, 1.0);
    return target
        ? std::min(1.0, current + step)
        : std::max(0.0, current - step);
}

inline double WbcGaitReferenceBlend(
    double gait_elapsed_s, double blend_duration_s)
{
    if (!(gait_elapsed_s > 0.0) ||
        !(blend_duration_s > 0.0) ||
        !std::isfinite(gait_elapsed_s) ||
        !std::isfinite(blend_duration_s))
        return 0.0;
    const double x = std::clamp(gait_elapsed_s / blend_duration_s, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

inline double WbcContactScheduleBlend(
    double gait_elapsed_s, double handoff_start_s, double handoff_duration_s)
{
    if (!(gait_elapsed_s > handoff_start_s) ||
        !(handoff_duration_s > 0.0) ||
        !std::isfinite(gait_elapsed_s) ||
        !std::isfinite(handoff_start_s) ||
        !std::isfinite(handoff_duration_s))
        return 0.0;
    const double x = std::clamp(
        (gait_elapsed_s - handoff_start_s) / handoff_duration_s,
        0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

}  // namespace go2_control
