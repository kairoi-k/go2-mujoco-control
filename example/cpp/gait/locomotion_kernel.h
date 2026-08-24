// Abstract locomotion gait kernel interface used by real_trot_go2.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <string>

#include "go2_forward_kinematics.h"

namespace go2_control
{

enum class GaitPattern
{
    kDiagonalTrot,
    // Running trot keeps the diagonal pairing but advances the opposite
    // pair slightly before the first pair leaves support.  It is the
    // high-speed, overlap-support variant; unlike the ordinary trot it does
    // not rely on an exact half-cycle hand-off at the WBC contact boundary.
    kRunningTrot,
    kBound,
    kPace,
    kCrawl,
    kGallop,
};

inline const char *GaitPatternName(GaitPattern pattern) noexcept
{
    switch (pattern)
    {
    case GaitPattern::kBound:
        return "bound";
    case GaitPattern::kGallop:
        return "gallop";
    case GaitPattern::kPace:
        return "pace";
    case GaitPattern::kCrawl:
        return "crawl";
    case GaitPattern::kDiagonalTrot:
        return "diagonal-trot";
    case GaitPattern::kRunningTrot:
        return "running-trot";
    default:
        return "diagonal-trot";
    }
}

inline bool ParseGaitPattern(
    const char *name,
    GaitPattern &pattern) noexcept
{
    if (name == nullptr)
        return false;
    const std::string value(name);
    if (value == "diagonal-trot" || value == "trot")
    {
        pattern = GaitPattern::kDiagonalTrot;
        return true;
    }
    if (value == "running-trot" || value == "running")
    {
        pattern = GaitPattern::kRunningTrot;
        return true;
    }
    if (value == "bound")
    {
        pattern = GaitPattern::kBound;
        return true;
    }
    if (value == "pace")
    {
        pattern = GaitPattern::kPace;
        return true;
    }
    if (value == "crawl")
    {
        pattern = GaitPattern::kCrawl;
        return true;
    }
    if (value == "gallop")
    {
        pattern = GaitPattern::kGallop;
        return true;
    }
    return false;
}

inline double WrapUnitPhase(double phase) noexcept
{
    double wrapped = phase - std::floor(phase);
    if (wrapped < 0.0)
        wrapped += 1.0;
    return wrapped;
}

inline double RunningTrotPhaseOffset() noexcept
{
    const char *value = std::getenv("TROT_RUNNING_TROT_OFFSET");
    if (value == nullptr || *value == '\0')
        return 0.46;
    char *end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || !std::isfinite(parsed))
        return 0.46;
    return std::clamp(parsed, 0.25, 0.50);
}

inline std::array<double, go2::kLegCount> GaitPatternPhaseOffsets(
    GaitPattern pattern) noexcept
{
    if (pattern == GaitPattern::kRunningTrot)
    {
        const double offset = RunningTrotPhaseOffset();
        return {0.0, offset, offset, 0.0};
    }
    if (pattern == GaitPattern::kCrawl)
        return {0.0, 0.25, 0.50, 0.75};
    if (pattern == GaitPattern::kBound)
        return {0.0, 0.0, 0.42, 0.42};
    if (pattern == GaitPattern::kPace)
        return {0.0, 0.5, 0.0, 0.5};
    if (pattern == GaitPattern::kGallop)
        return {0.50, 0.62, 0.0, 0.12};
    return {0.0, 0.5, 0.5, 0.0};
}

inline double GaitLegPhase(
    std::size_t leg,
    double phase,
    const std::array<double, go2::kLegCount> &offsets) noexcept
{
    return WrapUnitPhase(phase + offsets[leg % offsets.size()]);
}

inline double GaitLegPhase(
    std::size_t leg,
    double phase,
    GaitPattern pattern = GaitPattern::kDiagonalTrot) noexcept
{
    return GaitLegPhase(leg, phase, GaitPatternPhaseOffsets(pattern));
}

inline bool GaitLegScheduledStance(
    std::size_t leg,
    double phase,
    double duty,
    GaitPattern pattern = GaitPattern::kDiagonalTrot) noexcept
{
    return GaitLegPhase(leg, phase, pattern) < duty;
}

struct GaitKernelParams
{
    double period_s = 0.8;
    double duty_factor = 0.58;
    double step_length_m = 0.12;
    double direction_sign = 1.0;
    double foot_lift_m = 0.05;
    double blend_duration_s = 0.8;
    // Negative keeps the pattern default. Near one spreads swing motion over
    // the full flight window for low-duty running.
    double swing_reach_phase = -1.0;
    GaitPattern pattern = GaitPattern::kDiagonalTrot;
    std::array<double, go2::kLegCount> phase_offsets =
        GaitPatternPhaseOffsets(GaitPattern::kDiagonalTrot);
};

struct GaitKernelRequest
{
    double gait_time_s = 0.0;
    std::array<go2::Vec3, go2::kLegCount> neutral_feet{};
    // State inputs reserved for feedback-driven classical kernels.
    // Body-frame velocity: x forward, y left, z up.
    // The caller must convert sensor/world velocity before filling it.
    double body_velocity_x_mps = 0.0;
    double body_velocity_y_mps = 0.0;
    double body_velocity_z_mps = 0.0;
    bool have_body_velocity = false;
};

struct GaitKernelResult
{
    double phase = 0.0;
    int cycle_index = 0;
    std::array<go2::Vec3, go2::kLegCount> feet{};
    std::array<double, go2::kLegCount> touchdown_target_x_m{};
    double velocity_error_x_mps = 0.0;
    double nominal_velocity_x_mps = 0.0;  // [Fix 2026-08-13] kernel 当前生效目标速度 (换挡后 world 同步用)
    bool footstep_plan_valid = false;
    int preview_n_steps = 0;
    double preview_touchdown_x_m = 0.0;
    double preview_terminal_velocity_x_mps = 0.0;
    double preview_planned_acc_x_mps2 = 0.0;
    double period_s = 0.0;
    double duty_factor = 0.0;
    double step_length_m = 0.0;
};

class LocomotionKernel
{
public:
    virtual ~LocomotionKernel() = default;
    virtual const char *Name() const noexcept = 0;
    virtual bool Compute(
        const GaitKernelRequest &request,
        GaitKernelResult &result) = 0;
    virtual void SetGaitStepLength(double) {}
    virtual void SetGaitPeriod(double) {}
    virtual void SetGaitDuty(double) {}
    virtual void SetGaitFootLift(double) {}
    virtual void SetGaitPattern(GaitPattern) {}
    virtual void SetGaitPhaseOffsets(
        const std::array<double, go2::kLegCount> &) {}
    virtual void SetGaitSwingReachPhase(double) {}
    virtual void SetGaitEffectiveSpeedConvention(bool) {}
    virtual void SetGaitSlewLimits(double, double, double) {}
    virtual void SetStanceHold(bool, double) {}
};

class HandCodedTrotKernel final : public LocomotionKernel
{
public:
    explicit HandCodedTrotKernel(GaitKernelParams params)
        : params_(params)
    {
    }

    const char *Name() const noexcept override
    {
        return "hand-coded-trot";
    }

    void SetGaitStepLength(double step_m) override
    {
        if (step_m >= 0.0 && std::isfinite(step_m))
            params_.step_length_m = step_m;
    }
    void SetGaitPeriod(double period_s) override
    {
        if (period_s > 0.0 && std::isfinite(period_s))
            params_.period_s = period_s;
    }
    void SetGaitDuty(double duty) override
    {
        if (duty > 0.0 && duty < 1.0 && std::isfinite(duty))
            params_.duty_factor = duty;
    }
    void SetGaitFootLift(double lift_m) override
    {
        if (lift_m >= 0.0 && std::isfinite(lift_m))
            params_.foot_lift_m = lift_m;
    }

    void SetGaitPattern(GaitPattern pattern) override
    {
        params_.pattern = pattern;
        params_.phase_offsets = GaitPatternPhaseOffsets(pattern);
    }

    void SetGaitPhaseOffsets(
        const std::array<double, go2::kLegCount> &offsets) override
    {
        params_.phase_offsets = offsets;
    }

    void SetGaitSwingReachPhase(double phase) override
    {
        if (phase >= 0.5 && phase <= 1.0 && std::isfinite(phase))
            params_.swing_reach_phase = phase;
    }
    void SetStanceHold(bool hold, double gait_time_s) override
    {
        if (!hold && stance_hold_ && std::isfinite(gait_time_s))
            phase_origin_gait_time_s_ = gait_time_s;
        stance_hold_ = hold;
    }

    bool Compute(
        const GaitKernelRequest &request,
        GaitKernelResult &result) override
    {
        if (!std::isfinite(request.gait_time_s) ||
            request.gait_time_s < 0.0 ||
            !(params_.period_s > 0.0) ||
            !(params_.duty_factor > 0.0 &&
              params_.duty_factor < 1.0) ||
            !(params_.step_length_m >= 0.0) ||
            !(params_.foot_lift_m >= 0.0) ||
            !(params_.blend_duration_s > 0.0) ||
            !(std::abs(std::abs(params_.direction_sign) - 1.0) < 1e-9))
        {
            return false;
        }

        const double effective_gait_time = std::max(
            0.0, request.gait_time_s - phase_origin_gait_time_s_);
        const double cycle_position = effective_gait_time / params_.period_s;
        if (!std::isfinite(cycle_position) ||
            cycle_position >
                static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }

        result.phase = cycle_position - std::floor(cycle_position);
        result.cycle_index = static_cast<int>(std::floor(cycle_position));
        result.feet = request.neutral_feet;
        result.touchdown_target_x_m.fill(0.0);
        result.velocity_error_x_mps = 0.0;
        result.footstep_plan_valid = false;
        result.period_s = params_.period_s;
        result.duty_factor = params_.duty_factor;
        result.step_length_m = params_.step_length_m;

        const double blend =
            Smoothstep(effective_gait_time / params_.blend_duration_s);
        const double dt = last_gait_time_ >= 0.0
            ? std::max(0.0, effective_gait_time - last_gait_time_)
            : 0.0;
        const double hold_step = std::clamp(dt / 0.25, 0.0, 1.0);
        const double hold_target = stance_hold_ ? 1.0 : 0.0;
        stance_hold_blend_ +=
            (hold_target - stance_hold_blend_) * hold_step;
        const double gait_blend = blend * (1.0 - stance_hold_blend_);
        const double half_step = 0.5 * params_.step_length_m;
        const double stance_duration = params_.duty_factor;
        const double swing_duration = 1.0 - params_.duty_factor;

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const double leg_phase = GaitLegPhase(
                leg, result.phase, params_.phase_offsets);
            double x_offset = 0.0;
            double z_offset = 0.0;
            if (leg_phase < stance_duration)
            {
                const double stance_phase = leg_phase / stance_duration;
                x_offset =
                    half_step -
                    params_.step_length_m * Smoothstep(stance_phase);
            }
            else
            {
                const double swing_phase =
                    (leg_phase - stance_duration) / swing_duration;
                x_offset =
                    -half_step +
                    params_.step_length_m * Smoothstep(swing_phase);
                z_offset =
                    params_.foot_lift_m * std::sin(kPi * swing_phase);
            }
            x_offset *= params_.direction_sign;
            result.feet[leg].x += gait_blend * x_offset;
            result.feet[leg].z += gait_blend * z_offset;
        }
        last_gait_time_ = effective_gait_time;
        return true;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    static double Smoothstep(double x)
    {
        if (x <= 0.0)
            return 0.0;
        if (x >= 1.0)
            return 1.0;
        return x * x * (3.0 - 2.0 * x);
    }

    GaitKernelParams params_;
    bool stance_hold_ = false;
    double stance_hold_blend_ = 0.0;
    double last_gait_time_ = -1.0;
    double phase_origin_gait_time_s_ = 0.0;
};

} // namespace go2_control
