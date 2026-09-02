// Abstract locomotion gait kernel interface used by real_trot_go2.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
    // Quasi-static rotary crawl. With duty >= 0.75, at least three legs
    // remain scheduled in stance at every phase.
    kCrawl,
    kBound,
    kPace,
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
    case GaitPattern::kDiagonalTrot:
        return "diagonal-trot";
    case GaitPattern::kRunningTrot:
        return "running-trot";
    case GaitPattern::kCrawl:
        return "crawl";
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
    if (value == "crawl")
    {
        pattern = GaitPattern::kCrawl;
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

inline double GaitLegPhase(
    std::size_t leg,
    double phase,
    GaitPattern pattern = GaitPattern::kDiagonalTrot) noexcept
{
    static constexpr std::array<double, 4> kDiagonal = {0.0, 0.5, 0.5, 0.0};
    // A tunable phase offset lets experiments compare a near-flight running
    // trot with a small overlap-support variant without changing the normal
    // diagonal-trot convention.
    // Slight overlap between front and rear support pairs prevents a
    // low-duty bound from losing all vertical support at the hand-off.
    static constexpr std::array<double, 4> kBound = {0.0, 0.0, 0.42, 0.42};
    static constexpr std::array<double, 4> kPace = {0.0, 0.5, 0.0, 0.5};
    // True four-beat rotary gallop: right hind -> left hind -> right
    // fore -> left fore.  Leg order is FR, FL, RR, RL, so the rear pair
    // leads the fore pair by half a cycle while each side is staggered.
    static constexpr std::array<double, 4> kGallop = {0.50, 0.62, 0.0, 0.12};
    static constexpr std::array<double, 4> kCrawl = {0.0, 0.25, 0.50, 0.75};
    if (pattern == GaitPattern::kRunningTrot)
    {
        const double offset = RunningTrotPhaseOffset();
        return WrapUnitPhase(
            phase + ((leg == 0 || leg == 3) ? 0.0 : offset));
    }
    const auto &offsets =
        pattern == GaitPattern::kBound
            ? kBound
            : (pattern == GaitPattern::kPace
                   ? kPace
                   : (pattern == GaitPattern::kGallop
                   ? kGallop
                   : (pattern == GaitPattern::kCrawl ? kCrawl : kDiagonal)));
    return WrapUnitPhase(phase + offsets[leg % offsets.size()]);
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
};

// A complete, immutable timed terrain handoff. The adapter is the only
// producer; kernels consume this value and never select terrain policy.
struct GaitExecutionRequest
{
    bool valid = false;
    GaitPattern pattern = GaitPattern::kCrawl;
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    std::uint64_t map_epoch = 0;
    std::uint64_t input_hash = 0;
    double valid_from_s = 0.0;
    double valid_until_s = 0.0;
    double phase_origin_s = 0.0;
    double phase_origin = 0.0;
    double period_s = 0.8;
    double duty_factor = 0.75;
    double step_length_m = 0.0;
    double foot_lift_m = 0.05;
    std::array<double, go2::kLegCount> touchdown_time_s{};
    std::array<bool, go2::kLegCount> touchdown_time_valid{};
    std::array<double, go2::kLegCount> liftoff_time_s{};
    std::array<bool, go2::kLegCount> liftoff_time_valid{};
    std::array<double, go2::kLegCount> stance_start_time_s{};
    std::array<double, go2::kLegCount> stance_end_time_s{};
    std::array<bool, go2::kLegCount> stance_interval_valid{};
    std::array<go2::Vec3, go2::kLegCount> swing_start{};
    std::array<go2::Vec3, go2::kLegCount> swing_endpoint{};
    // Endpoint identity is deliberately separate from coordinates: once a
    // leg is in flight it must not be replaced by a newer partial snapshot.
    std::array<std::uint64_t, go2::kLegCount> endpoint_identity{};
    std::array<bool, go2::kLegCount> endpoint_valid{};
    // Exact planner-owned support row for this control instant. This remains
    // planned contact only; measured_support is the independent safety input.
    std::array<bool, go2::kLegCount> scheduled_support{};
    bool scheduled_support_valid = false;
    std::array<bool, go2::kLegCount> measured_support{};
    bool fallback = false;
    std::string fallback_reason;
};

struct GaitKernelRequest
{
    double gait_time_s = 0.0;
    std::array<go2::Vec3, go2::kLegCount> neutral_feet{};
    bool has_execution_request = false;
    GaitExecutionRequest execution{};

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
    // Exact nominal touchdown endpoints in the body frame.  The terrain
    // planner uses these as the Phase-1 geometric reference even when a leg
    // is already in swing; feet is the instantaneous commanded trajectory.
    std::array<go2::Vec3, go2::kLegCount> touchdown_target_feet_base{};
    bool touchdown_target_feet_valid = false;
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
    bool execution_request_valid = false;
    std::uint64_t execution_plan_id = 0;
    std::uint64_t execution_plan_epoch = 0;
    std::uint64_t execution_map_epoch = 0;
    std::uint64_t execution_input_hash = 0;
    bool execution_fallback = false;
    std::string execution_fallback_reason;
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
    virtual void SetGaitPattern(GaitPattern) {}
    virtual void SetGaitPeriod(double) {}
    virtual void SetGaitDuty(double) {}
    virtual void SetGaitFootLift(double) {}
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

    void SetGaitPattern(GaitPattern pattern) override
    {
        params_.pattern = pattern;
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

        const bool timed_execution = request.has_execution_request &&
            request.execution.valid &&
            std::isfinite(request.execution.valid_from_s) &&
            std::isfinite(request.execution.valid_until_s) &&
            request.gait_time_s >= request.execution.valid_from_s &&
            request.gait_time_s <= request.execution.valid_until_s &&
            request.execution.period_s > 0.0 &&
            request.execution.duty_factor > 0.0 &&
            request.execution.duty_factor < 1.0;
        const double effective_gait_time = timed_execution
            ? std::max(0.0, request.gait_time_s)
            : std::max(0.0, request.gait_time_s - phase_origin_gait_time_s_);
        const double cycle_position = timed_execution
            ? request.execution.phase_origin +
                (request.gait_time_s - request.execution.phase_origin_s) /
                    request.execution.period_s
            : effective_gait_time / params_.period_s;
        if (!std::isfinite(cycle_position) ||
            cycle_position >
                static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }

        result.phase = WrapUnitPhase(cycle_position);
        result.cycle_index = static_cast<int>(std::floor(cycle_position));
        result.execution_request_valid = timed_execution;
        result.execution_plan_id = timed_execution ? request.execution.plan_id : 0;
        result.execution_plan_epoch = timed_execution ? request.execution.plan_epoch : 0;
        result.execution_map_epoch = timed_execution ? request.execution.map_epoch : 0;
        result.execution_input_hash = timed_execution ? request.execution.input_hash : 0;
        result.execution_fallback = timed_execution && request.execution.fallback;
        result.execution_fallback_reason = timed_execution
            ? request.execution.fallback_reason : std::string{};
        result.feet = request.neutral_feet;
        result.touchdown_target_x_m.fill(0.0);
        result.touchdown_target_feet_base = request.neutral_feet;
        result.touchdown_target_feet_valid = true;
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
            const double leg_phase = timed_execution
                ? GaitLegPhase(leg, result.phase, request.execution.pattern)
                : GaitLegPhase(leg, result.phase, params_.pattern);
            if (timed_execution && request.execution.endpoint_valid[leg] &&
                request.execution.liftoff_time_valid[leg] &&
                request.execution.touchdown_time_valid[leg])
            {
                const double liftoff = request.execution.liftoff_time_s[leg];
                const double touchdown = request.execution.touchdown_time_s[leg];
                if (request.gait_time_s >= liftoff &&
                    request.gait_time_s < touchdown && touchdown > liftoff)
                {
                    const double u = std::clamp(
                        (request.gait_time_s - liftoff) /
                            (touchdown - liftoff), 0.0, 1.0);
                    const double smooth = Smoothstep(u);
                    result.feet[leg].x = request.execution.swing_start[leg].x +
                        smooth * (request.execution.swing_endpoint[leg].x -
                                  request.execution.swing_start[leg].x);
                    result.feet[leg].y = request.execution.swing_start[leg].y +
                        smooth * (request.execution.swing_endpoint[leg].y -
                                  request.execution.swing_start[leg].y);
                    result.feet[leg].z = request.execution.swing_start[leg].z +
                        smooth * (request.execution.swing_endpoint[leg].z -
                                  request.execution.swing_start[leg].z) +
                        request.execution.foot_lift_m * std::sin(kPi * u);
                    result.touchdown_target_feet_base[leg] =
                        request.execution.swing_endpoint[leg];
                    continue;
                }
                if (request.gait_time_s >= touchdown)
                {
                    result.feet[leg] = request.execution.swing_endpoint[leg];
                    result.touchdown_target_feet_base[leg] =
                        request.execution.swing_endpoint[leg];
                    continue;
                }
            }
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
            result.touchdown_target_feet_base[leg].x +=
                params_.direction_sign * half_step;
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
