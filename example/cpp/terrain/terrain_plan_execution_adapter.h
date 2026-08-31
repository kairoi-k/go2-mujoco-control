#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "locomotion_kernel.h"
#include "terrain_motion_plan.h"

namespace go2_terrain
{

// The sole plan-to-gait seam. It accepts complete immutable snapshots and
// never exposes a plan field independently to the gait kernel.
class TerrainPlanExecutionAdapter final
{
public:
    static constexpr double kDefaultGraceS = 0.10;
    struct Result
    {
        bool using_plan = false;
        bool adopted = false;
        bool rejected = false;
        std::uint64_t adopted_plan_id = 0;
        std::uint64_t rejected_plan_id = 0;
        std::string rejection_reason;
        std::string fallback_reason;
        go2_control::GaitExecutionRequest request{};
    };

    explicit TerrainPlanExecutionAdapter(bool enabled = false,
                                         double grace_s = kDefaultGraceS)
        : enabled_(enabled), grace_s_(std::max(0.0, grace_s)) {}

    void SetEnabled(bool enabled)
    {
        enabled_ = enabled;
        if (!enabled_)
            Reset();
    }
    bool enabled() const noexcept { return enabled_; }
    bool using_plan() const noexcept { return using_plan_; }
    void SetContactGuard(bool active, std::size_t age_ticks) noexcept
    {
        contact_guard_active_ = active;
        contact_guard_age_ticks_ = age_ticks;
    }
    std::uint64_t adopted_plan_id() const noexcept
    { return adopted_ ? adopted_->plan_id : 0; }
    // The WBC consumer must use the exact snapshot adopted by gait, not a
    // newer store value that arrived between event boundaries.
    std::shared_ptr<const TerrainMotionPlan> adopted_plan() const noexcept
    { return adopted_; }
    std::uint64_t last_rejected_plan_id() const noexcept
    { return last_rejected_plan_id_; }
    const std::string &last_rejection_reason() const noexcept
    { return last_rejection_reason_; }
    const std::string &last_fallback_reason() const noexcept
    { return last_fallback_reason_; }

    void Reset()
    {
        adopted_.reset();
        using_plan_ = false;
        fallback_age_steps_ = 0;
        last_rejected_plan_id_ = 0;
        last_rejection_reason_.clear();
        last_fallback_reason_.clear();
        last_request_ = {};
        in_flight_.fill(false);
        contact_guard_active_ = false;
        contact_guard_age_ticks_ = 0;
    }

    // A replacement is legal only at an explicit event boundary and when no
    // leg is in flight. The first snapshot is an initial boundary.
    bool IsLegalBoundary(double now_s) const noexcept
    {
        if (!adopted_)
            return true;
        if (!std::isfinite(now_s))
            return false;
        for (bool flight : in_flight_)
            if (flight)
                return false;
        return AtEventBoundary(*adopted_, now_s);
    }

    Result Update(const TerrainMotionPlan *candidate, double now_s,
                  bool event_boundary,
                  const std::array<bool, go2::kLegCount> &measured_support,
                  go2_control::GaitPattern fallback_pattern,
                  double fallback_period_s, double fallback_duty_factor,
                  double fallback_step_length_m,
                  double fallback_foot_lift_m)
    {
        Result result;
        if (!enabled_)
        {
            result.fallback_reason = "stage_c_execution_disabled";
            return result;
        }
        const bool candidate_valid = candidate != nullptr &&
            candidate->has_stage_c_timing && candidate->input_hash != 0 &&
            IsV2BPlan(*candidate) &&
            candidate->valid() && std::isfinite(now_s) &&
            candidate->identity.usable_at(now_s);
        if (candidate != nullptr && !candidate_valid && candidate->plan_id != 0)
        {
            result.rejected = true;
            result.rejected_plan_id = candidate->plan_id;
            last_rejected_plan_id_ = candidate->plan_id;
            last_rejection_reason_ = candidate->v3_c_shadow
                ? "v3_c_shadow_rejected"
                : (candidate->has_stage_c_timing
                    ? "invalid_or_provenance_mismatch"
                    : "stage_c_timing_missing");
            result.rejection_reason = last_rejection_reason_;
        }
        const bool same_plan = candidate_valid && adopted_ &&
            SameIdentity(*candidate, *adopted_);
        const bool boundary = event_boundary && IsLegalBoundary(now_s);
        const bool guard_keeps_snapshot = contact_guard_active_ && adopted_ &&
            !same_plan;
        if (candidate_valid && (!guard_keeps_snapshot) &&
            (!adopted_ || same_plan || boundary))
        {
            if (!same_plan)
            {
                adopted_ = std::make_shared<const TerrainMotionPlan>(*candidate);
                result.adopted = true;
                result.adopted_plan_id = candidate->plan_id;
            }
            using_plan_ = true;
            fallback_age_steps_ = 0;
            last_request_ = MakeRequest(*adopted_);
            result.using_plan = true;
            result.request = last_request_;
        }
        else if (candidate_valid && adopted_ && !boundary && !same_plan)
        {
            result.rejected = true;
            result.rejected_plan_id = candidate->plan_id;
            last_rejected_plan_id_ = candidate->plan_id;
            last_rejection_reason_ = "not_at_event_boundary_or_in_flight";
            result.rejection_reason = last_rejection_reason_;
        }
        const bool guard_safe_stop = contact_guard_active_ &&
            contact_guard_age_ticks_ >= 5;
        if (adopted_ && !guard_safe_stop && std::isfinite(now_s) &&
            now_s <= adopted_->valid_until_s + grace_s_)
        {
            using_plan_ = true;
            last_request_ = MakeRequest(*adopted_);
            result.using_plan = true;
            result.request = last_request_;
        }
        else
        {
            using_plan_ = false;
            if (contact_guard_active_)
                fallback_age_steps_ = contact_guard_age_ticks_;
            else
                ++fallback_age_steps_;
            result.fallback_reason = contact_guard_active_
                ? ContactGuardFallbackReason(fallback_age_steps_)
                : MeasuredSupportFallbackReason(fallback_age_steps_);
            last_fallback_reason_ = result.fallback_reason;
            last_request_ = MakeFallbackRequest(
                measured_support, fallback_pattern, fallback_period_s,
                fallback_duty_factor, fallback_step_length_m,
                fallback_foot_lift_m, result.fallback_reason);
            result.request = last_request_;
        }
        UpdateFlightState(last_request_, now_s);
        return result;
    }

    // Translate the already-adopted whole request into the kernel in one
    // operation. No caller should invoke terrain gait setters independently.
    void ApplyToKernel(go2_control::LocomotionKernel &kernel,
                       go2_control::GaitKernelRequest &request,
                       double gait_time_s,
                       const std::array<go2::Vec3, go2::kLegCount> &neutral_feet)
        const
    {
        request.gait_time_s = gait_time_s;
        request.neutral_feet = neutral_feet;
        request.has_execution_request = using_plan_ && last_request_.valid;
        request.execution = last_request_;
        if (!request.has_execution_request)
            request.execution = {};
        if (using_plan_ && last_request_.valid)
        {
            kernel.SetGaitPattern(last_request_.pattern);
            kernel.SetGaitPeriod(last_request_.period_s);
            kernel.SetGaitDuty(last_request_.duty_factor);
            kernel.SetGaitStepLength(last_request_.step_length_m);
            kernel.SetGaitFootLift(last_request_.foot_lift_m);
        }
    }

private:
    static bool SameIdentity(const TerrainMotionPlan &a,
                             const TerrainMotionPlan &b) noexcept
    {
        return a.plan_id == b.plan_id && a.plan_epoch == b.plan_epoch &&
            a.map_epoch == b.map_epoch && a.input_hash == b.input_hash;
    }
    static bool IsV2BPlan(const TerrainMotionPlan &plan) noexcept
    {
        if (plan.v3_c_shadow || plan.horizon_knots == 0 ||
            plan.horizon_knots > kTerrainPlanMaxKnots)
            return false;
        for (std::size_t k = 0; k < plan.horizon_knots; ++k)
        {
            std::size_t contacts = 0;
            for (bool contact : plan.contact_schedule.planned_contact[k])
                contacts += contact ? 1U : 0U;
            if (contacts < 3)
                return false;
        }
        return true;
    }
    static bool AtEventBoundary(const TerrainMotionPlan &plan,
                                double now_s) noexcept
    {
        constexpr double kBoundaryToleranceS = 1.0e-3;
        if (!std::isfinite(now_s))
            return false;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (plan.contact_timing.touchdown_time_valid[leg] &&
                std::abs(now_s - plan.contact_timing.touchdown_time_s[leg]) <=
                    kBoundaryToleranceS)
                return true;
            if (plan.contact_timing.liftoff_time_valid[leg] &&
                std::abs(now_s - plan.contact_timing.liftoff_time_s[leg]) <=
                    kBoundaryToleranceS)
                return true;
        }
        return false;
    }
    static std::uint64_t EndpointIdentity(const TerrainMotionPlan &plan,
                                           std::size_t leg,
                                           const go2::Vec3 &endpoint)
    {
        std::uint64_t hash = 1469598103934665603ULL;
        const auto add = [&hash](const void *data, std::size_t size) {
            const auto *bytes = static_cast<const unsigned char *>(data);
            for (std::size_t i = 0; i < size; ++i)
            { hash ^= bytes[i]; hash *= 1099511628211ULL; }
        };
        add(&plan.plan_id, sizeof(plan.plan_id));
        add(&plan.map_epoch, sizeof(plan.map_epoch));
        add(&leg, sizeof(leg));
        add(&endpoint, sizeof(endpoint));
        return hash == 0 ? 1 : hash;
    }
    static go2_control::GaitExecutionRequest MakeRequest(
        const TerrainMotionPlan &plan)
    {
        go2_control::GaitExecutionRequest request;
        request.valid = true;
        request.pattern = go2_control::GaitPattern::kCrawl;
        request.plan_id = plan.plan_id;
        request.plan_epoch = plan.plan_epoch;
        request.map_epoch = plan.map_epoch;
        request.input_hash = plan.input_hash;
        request.valid_from_s = plan.generated_at_s;
        request.valid_until_s = plan.valid_until_s;
        request.phase_origin_s = plan.state_stamp_s;
        request.phase_origin = plan.gait_phase;
        request.period_s = plan.contact_timing.period_s;
        request.duty_factor = plan.contact_timing.duty_factor;
        request.step_length_m = 0.0;
        request.foot_lift_m = 0.05;
        request.touchdown_time_s = plan.contact_timing.touchdown_time_s;
        request.touchdown_time_valid = plan.contact_timing.touchdown_time_valid;
        request.liftoff_time_s = plan.contact_timing.liftoff_time_s;
        request.liftoff_time_valid = plan.contact_timing.liftoff_time_valid;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            request.measured_support[leg] =
                plan.contact_schedule.measured_contact[leg];
            request.stance_start_time_s[leg] = plan.state_stamp_s;
            request.stance_end_time_s[leg] = plan.valid_until_s;
            request.stance_interval_valid[leg] =
                request.touchdown_time_valid[leg] ||
                request.liftoff_time_valid[leg];
            for (std::size_t k = 0; k < plan.horizon_knots; ++k)
            {
                const auto &foot = plan.predicted_foothold[k][leg];
                if (!foot.valid || !foot.touchdown)
                    continue;
                request.swing_start[leg] = foot.swing_start_position_world;
                request.swing_endpoint[leg] = foot.position_world;
                request.endpoint_valid[leg] = foot.swing_start_position_valid;
                request.endpoint_identity[leg] = EndpointIdentity(
                    plan, leg, foot.position_world);
                request.foot_lift_m = std::max(request.foot_lift_m,
                                               foot.swing_lift_m);
                break;
            }
        }
        return request;
    }
    static go2_control::GaitExecutionRequest MakeFallbackRequest(
        const std::array<bool, go2::kLegCount> &measured_support,
        go2_control::GaitPattern pattern, double period_s, double duty,
        double step, double lift, const std::string &reason)
    {
        go2_control::GaitExecutionRequest request;
        request.valid = true;
        request.pattern = pattern;
        request.period_s = period_s;
        request.duty_factor = duty;
        request.step_length_m = step;
        request.foot_lift_m = lift;
        request.measured_support = measured_support;
        request.fallback = true;
        request.fallback_reason = reason;
        return request;
    }
    static std::string MeasuredSupportFallbackReason(std::size_t age_steps)
    {
        if (age_steps <= 1) return "measured-support:N";
        if (age_steps <= 5) return "measured-support:N+1";
        if (age_steps <= 25) return "measured-support:N+5";
        return "measured-support:N+25";
    }
    static std::string ContactGuardFallbackReason(std::size_t age_ticks)
    {
        if (age_ticks == 0) return "measured-support:N";
        if (age_ticks < 5) return "measured-support:N+1";
        if (age_ticks < 25) return "measured-support:N+5";
        return "measured-support:N+25";
    }
    void UpdateFlightState(const go2_control::GaitExecutionRequest &request,
                           double now_s)
    {
        in_flight_.fill(false);
        if (!request.valid || !std::isfinite(now_s))
            return;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            in_flight_[leg] = request.endpoint_valid[leg] &&
                request.liftoff_time_valid[leg] &&
                request.touchdown_time_valid[leg] &&
                now_s >= request.liftoff_time_s[leg] &&
                now_s < request.touchdown_time_s[leg];
    }

    bool enabled_ = false;
    double grace_s_ = kDefaultGraceS;
    bool using_plan_ = false;
    std::size_t fallback_age_steps_ = 0;
    std::shared_ptr<const TerrainMotionPlan> adopted_;
    std::array<bool, go2::kLegCount> in_flight_{};
    bool contact_guard_active_ = false;
    std::size_t contact_guard_age_ticks_ = 0;
    go2_control::GaitExecutionRequest last_request_{};
    std::uint64_t last_rejected_plan_id_ = 0;
    std::string last_rejection_reason_;
    std::string last_fallback_reason_;
};

} // namespace go2_terrain
