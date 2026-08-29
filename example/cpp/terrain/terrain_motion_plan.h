#pragma once

#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "go2_forward_kinematics.h"
#include "terrain_control_interface.h"

namespace go2_terrain
{

constexpr std::size_t kTerrainPlanMaxKnots = kTerrainContactMaxKnots;

constexpr double kTerrainTransferTouchdownToleranceM = 0.045;

inline double TerrainTouchdownTolerance(
    bool transfer_window_active, double foot_patch_radius_m)
{
    const double geometric_tolerance = std::max(
        0.020, 1.5 * foot_patch_radius_m);
    return transfer_window_active
        ? std::max(geometric_tolerance,
                   kTerrainTransferTouchdownToleranceM)
        : geometric_tolerance;
}

// Return the horizontal path progress at a sensor-derived leading edge. This
// mirrors the bounded swing path's smoothstep without depending on the
// terrain feasibility sweep.
inline double TerrainSwingLeadingEdgePathProgress(double edge_phase)
{
    const double u = std::clamp(edge_phase, 0.10, 0.75);
    return u * u * (3.0 - 2.0 * u);
}

// A measured contact at or before the inferred edge is a corner catch, not a
// valid touchdown. Leave the transition requirement intact so the next plan
// can execute a fresh swing rather than committing or pinning the endpoint.
inline bool TerrainSwingLeadingEdgeReached(
    double elapsed_s, double swing_duration_s,
    bool leading_edge_phase_valid, double leading_edge_phase)
{
    if (!leading_edge_phase_valid || !std::isfinite(elapsed_s) ||
        !std::isfinite(swing_duration_s) || swing_duration_s <= 0.0)
        return true;
    const double phase = std::clamp(
        elapsed_s / swing_duration_s, 0.0, 1.0);
    return phase >= std::clamp(leading_edge_phase, 0.10, 0.75);
}

inline bool TerrainSwingContactBeforeLeadingEdge(
    const go2::Vec3 &start_world, const go2::Vec3 &target_world,
    const go2::Vec3 &actual_world, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    if (!leading_edge_phase_valid ||
        !std::isfinite(start_world.x) || !std::isfinite(target_world.x) ||
        !std::isfinite(actual_world.x) || target_world.x <= start_world.x)
        return false;
    const double edge_x = start_world.x +
        TerrainSwingLeadingEdgePathProgress(leading_edge_phase) *
            (target_world.x - start_world.x);
    // Allow the measured foot-site/contact filter's few-mm spatial jitter at
    // the inferred edge while still rejecting a corner catch in front of it.
    return actual_world.x <= edge_x + 0.005;
}

// The explicit crawl state owns the selected leg until its immutable
// touchdown time. Once that time is reached, let the normal endpoint-held
// path run so the measured-contact/endpoint commit gate can execute.
inline bool TerrainCrawlSwingStillInFlight(
    bool explicit_crawl_step, std::size_t active_leg, std::size_t leg,
    bool execution_valid, bool execution_in_flight,
    double now_s, double touchdown_time_s, double time_tolerance_s) noexcept
{
    if (!explicit_crawl_step || active_leg != leg)
        return false;
    if (!execution_valid)
        return true;
    return execution_in_flight && std::isfinite(now_s) &&
        std::isfinite(touchdown_time_s) &&
        now_s + std::max(0.0, time_tolerance_s) < touchdown_time_s;
}

// A target that cannot be handed off within its immutable touchdown window
// is a failed leg of the transaction. Keep required unchanged: planned
// requirements are part of the transfer-consistency record and a cancelled
// leg must never be silently rewritten as completed.
inline bool MarkTerrainTransitionLegCancelled(
    const std::array<bool, go2::kLegCount> &required,
    const std::array<bool, go2::kLegCount> &committed,
    std::array<bool, go2::kLegCount> &cancelled,
    std::array<bool, go2::kLegCount> &source_valid,
    std::size_t leg)
{
    if (leg >= go2::kLegCount || !required[leg] || committed[leg] ||
        cancelled[leg])
        return false;
    cancelled[leg] = true;
    source_valid[leg] = false;
    return true;
}

inline bool TerrainTransitionComplete(
    const std::array<bool, go2::kLegCount> &required,
    const std::array<bool, go2::kLegCount> &committed,
    const std::array<bool, go2::kLegCount> &cancelled)
{
    bool has_requirement = false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!required[leg])
            continue;
        has_requirement = true;
        if (cancelled[leg] || !committed[leg])
            return false;
    }
    return has_requirement;
}

// A transfer hold may be released only after every planned upper-surface
// endpoint has both measured contact and endpoint confirmation.  Cancellation
// is intentionally not a release path: it remains a failed transaction.
inline bool TerrainTransferHoldReleaseReady(
    const std::array<bool, go2::kLegCount> &required,
    const std::array<bool, go2::kLegCount> &committed,
    const std::array<bool, go2::kLegCount> &cancelled)
{
    return TerrainTransitionComplete(required, committed, cancelled);
}

// An endpoint-held target is still the support captured at the transfer
// boundary. Keep it in the WBC support set until measured touchdown; a
// transaction that has not committed may not release that set while a
// target waits at its endpoint. An in-flight target remains a swing task
// and is intentionally excluded from QP support.
// Once a transfer hold is active, the captured support set is authoritative.
// A later gait phase must not replace it with a nominal diagonal while the
// terrain target is still waiting for measured touchdown.
inline std::array<bool, go2::kLegCount> TerrainTransferHoldSupport(
    const std::array<bool, go2::kLegCount> &held_support,
    const std::array<bool, go2::kLegCount> &scheduled_support,
    bool hold_active)
{
    return hold_active ? held_support : scheduled_support;
}

// While active, retain the captured set and monotonically add a newly
// scheduled or measured stance foot. This permits a target leg to swing
// without reducing the remaining physical support below three feet.
inline std::array<bool, go2::kLegCount> TerrainTransferHoldSupport(
    const std::array<bool, go2::kLegCount> &held_support,
    const std::array<bool, go2::kLegCount> &scheduled_support,
    const std::array<bool, go2::kLegCount> &measured_support,
    bool hold_active)
{
    if (!hold_active)
        return scheduled_support;
    auto support = held_support;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        support[leg] = support[leg] || scheduled_support[leg] ||
            measured_support[leg];
    return support;
}

inline bool TerrainTransferSupportMustBeKept(
    const std::array<bool, go2::kLegCount> &required,
    const std::array<bool, go2::kLegCount> &committed,
    const std::array<bool, go2::kLegCount> &cancelled,
    bool endpoint_held,
    bool in_flight)
{
    bool has_uncommitted_requirement = false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (required[leg] && !committed[leg] && !cancelled[leg])
        {
            has_uncommitted_requirement = true;
            break;
        }
    }
    return endpoint_held || (has_uncommitted_requirement && !in_flight);
}

enum class TerrainPlanStatus : std::uint8_t
{
    kEmpty = 0,
    kValid,
    kDegraded,
    kStale,
    kRejected,
    kSafeStop,
};

inline const char *TerrainPlanStatusName(TerrainPlanStatus status)
{
    switch (status)
    {
    case TerrainPlanStatus::kValid: return "valid";
    case TerrainPlanStatus::kDegraded: return "degraded";
    case TerrainPlanStatus::kStale: return "stale";
    case TerrainPlanStatus::kRejected: return "rejected";
    case TerrainPlanStatus::kSafeStop: return "safe_stop";
    default: return "empty";
    }
}

enum class TerrainPlanFailure : std::uint8_t
{
    kNone = 0,
    kNoMap,
    kStaleMap,
    kUnknownTerrain,
    kNoSafeFoothold,
    kSupportInfeasible,
    kBodyInfeasible,
    kSolverFailure,
    kDeadlineMiss,
    kInvalidInput,
    kContactDisagreement,
};

struct TerrainBodyReference
{
    go2::Vec3 position{};
    go2::Vec3 linear_velocity{};
    go2::Vec3 linear_acceleration{};
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double yaw_rate_radps = 0.0;
    double height_m = 0.0;
    bool valid = false;
};

struct TerrainFootholdPrediction
{
    bool valid = false;
    bool touchdown = false;
    double touchdown_time_s = 0.0;
    double touchdown_phase = 0.0;
    go2::Vec3 position_world{};
    go2::Vec3 swing_start_position_world{};
    bool swing_start_position_valid = false;
    std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
    std::uint32_t region_id = 0;
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    double swing_clearance_m = 0.0;
    double swing_lift_m = 0.0;
    // Full terrain-conditioned swing duration, including the clearance
    // arch.  The consumer may rebase its start to a newer measured foot
    // anchor, but must preserve this feasibility-derived lower bound.
    double swing_duration_s = 0.0;
    double swing_peak_phase = 0.5;
    double swing_leading_edge_phase = 0.5;
    bool swing_leading_edge_phase_valid = false;
    double support_margin_m = 0.0;
    double collision_margin_m = 0.0;
    double uncertainty_m = 0.0;
    // Sensor-derived target elevation crosses the currently loaded support
    // surface. This identifies the contact event that must be confirmed;
    // it is never inferred from a fixed leg order or scene geometry.
    // Planner-owned target-surface intent.  Support validation consumes this
    // latched identity rather than comparing measured/blended foothold z.
    bool surface_transition_required = false;
    bool surface_transition_intent_valid = false;
};

struct TerrainVelocityRequest
{
    bool valid = false;
    bool is_cap = true;
    double target_vx_mps = 0.0;
    double max_vx_mps = 0.0;
    double max_accel_mps2 = 0.8;
    double max_decel_mps2 = 1.2;
    double max_jerk_mps3 = 4.0;
    int priority = 0;
    std::uint64_t plan_id = 0;
    double valid_until_s = 0.0;
    std::string reason;
};

struct TerrainSolverDiagnostics
{
    bool attempted = false;
    bool success = false;
    bool deadline_miss = false;
    std::uint32_t iterations = 0;
    double elapsed_us = 0.0;
    double deadline_us = 5000.0;
    TerrainPlanFailure failure = TerrainPlanFailure::kNone;
};

struct TerrainMotionPlan
{
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    std::uint64_t map_epoch = 0;
    double state_stamp_s = 0.0;
    double generated_at_s = 0.0;
    double valid_until_s = 0.0;
    std::string frame_id;
    TerrainPlanStatus status = TerrainPlanStatus::kEmpty;
    TerrainPlanFailure failure = TerrainPlanFailure::kNone;
    double map_age_s = 0.0;
    double uncertainty_m = 0.0;
    std::size_t horizon_knots = 0;
    // Preserve the continuous gait timing used when the atomic snapshot was
    // generated.  The target adapter can distinguish a plan made during an
    // active swing from one made while the leg was still supporting.
    double gait_phase = 0.0;
    double gait_period_s = 0.8;
    double duty_factor = 0.58;
    std::size_t current_support_count = 0;
    double min_edge_margin_m = 0.0;
    double min_uncertainty_inflated_edge_margin_m = 0.0;
    double min_slope_rad = 0.0;
    double max_roughness_m = 0.0;
    double min_reachability_margin_m = 0.0;
    double min_swing_clearance_m = 0.0;
    double min_support_margin_m = 0.0;
    double min_uncertainty_inflated_support_margin_m = 0.0;
    std::size_t committed_touchdowns = 0;
    std::array<TerrainBodyReference, kTerrainPlanMaxKnots> body_reference{};
    TerrainContactSchedule contact_schedule{};
    std::array<std::array<TerrainFootholdPrediction, go2::kLegCount>,
               kTerrainPlanMaxKnots>
        predicted_foothold{};
    std::array<TerrainFootholdPrediction, go2::kLegCount>
        current_support_anchor{};
    // Keep the sensor-derived terrain reference separate from the measured
    // kinematic foot anchors used for support and MPC lever arms.
    std::array<double, go2::kLegCount>
        current_support_surface_height_world{};
    std::array<bool, go2::kLegCount>
        current_support_surface_valid{};
    // Per-leg sensor-derived terrain at the current foot footprint.  This is
    // deliberately distinct from current_support_anchor: an unconfirmed
    // footprint may inform height adaptation, but never becomes measured
    // support merely because a map sample exists.
    std::array<double, go2::kLegCount>
        current_terrain_height_world{};
    std::array<bool, go2::kLegCount>
        current_terrain_height_valid{};
    // The planner checks the swept path from this measured/kinematic
    // per-leg start to the committed foothold.  Keep that start in the same
    // atomic snapshot so the target adapter executes the path it checked.
    std::array<go2::Vec3, go2::kLegCount>
        swing_start_position_world{};
    std::array<bool, go2::kLegCount>
        swing_start_position_valid{};
    TerrainVelocityRequest velocity_request{};
    TerrainSolverDiagnostics solver{};
    bool fallback_to_phase1 = true;
    bool safe_stop_requested = false;

    bool valid() const
    {
        if ((status != TerrainPlanStatus::kValid &&
             status != TerrainPlanStatus::kDegraded) ||
            plan_id == 0 || plan_epoch == 0 || map_epoch == 0 ||
            !contact_schedule.valid(horizon_knots) || frame_id.empty() ||
            !std::isfinite(state_stamp_s) || !std::isfinite(generated_at_s) ||
            !std::isfinite(valid_until_s) || valid_until_s < generated_at_s ||
            horizon_knots == 0 || horizon_knots > kTerrainPlanMaxKnots)
            return false;
        for (std::size_t k = 0; k < horizon_knots; ++k)
        {
            if (!body_reference[k].valid)
                return false;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!contact_schedule.planned_contact[k][leg])
                    continue;
                const auto &foot = predicted_foothold[k][leg];
                if (!foot.valid || !std::isfinite(foot.touchdown_time_s) ||
                    !std::isfinite(foot.position_world.x) ||
                    !std::isfinite(foot.position_world.y) ||
                    !std::isfinite(foot.position_world.z) ||
                    !std::isfinite(foot.swing_lift_m) ||
                    foot.swing_lift_m < 0.0 ||
                    (foot.touchdown &&
                     (!std::isfinite(foot.swing_duration_s) ||
                      foot.swing_duration_s <= 0.0)) ||
                    !std::isfinite(foot.swing_peak_phase) ||
                    foot.swing_peak_phase < 0.10 ||
                    foot.swing_peak_phase > 0.90 ||
                    !std::isfinite(foot.swing_leading_edge_phase) ||
                    (foot.swing_leading_edge_phase_valid &&
                     (foot.swing_leading_edge_phase < 0.10 ||
                      foot.swing_leading_edge_phase > 0.75)))
                    return false;
                if (foot.touchdown &&
                    (!foot.swing_start_position_valid ||
                     !std::isfinite(foot.swing_start_position_world.x) ||
                     !std::isfinite(foot.swing_start_position_world.y) ||
                     !std::isfinite(foot.swing_start_position_world.z)))
                    return false;
                if (foot.touchdown &&
                    foot.touchdown_time_s > valid_until_s + 1.0e-6)
                    return false;
            }
        }
        return true;
    }

    bool usable_at(double now_s) const
    {
        return valid() && std::isfinite(now_s) && now_s <= valid_until_s;
    }
};

// Map a published plan's absolute-time knots onto the consumer horizon.  The
// planner runs asynchronously, so knot zero is not necessarily "now" when
// SRBD-MPC consumes the snapshot.  Resampling by time keeps planned contact
// and planned foothold data on the same timeline and rejects a snapshot that
// no longer covers the complete consumer horizon.
inline bool BuildTerrainPlanHorizonIndices(
    const TerrainMotionPlan &plan,
    double now_s,
    double plan_knot_dt_s,
    double consumer_knot_dt_s,
    std::size_t consumer_horizon,
    std::array<std::size_t, kTerrainPlanMaxKnots> &indices)
{
    indices.fill(0);
    if (!plan.usable_at(now_s) ||
        !std::isfinite(now_s) ||
        !std::isfinite(plan.state_stamp_s) ||
        !std::isfinite(plan_knot_dt_s) ||
        !std::isfinite(consumer_knot_dt_s) ||
        plan_knot_dt_s <= 0.0 ||
        consumer_knot_dt_s <= 0.0 ||
        consumer_horizon == 0 ||
        consumer_horizon > kTerrainPlanMaxKnots)
        return false;

    const double age_s = now_s - plan.state_stamp_s;
    if (!std::isfinite(age_s) ||
        age_s < -0.5 * plan_knot_dt_s)
        return false;
    const double clamped_age_s = std::max(0.0, age_s);
    for (std::size_t k = 0; k < consumer_horizon; ++k)
    {
        const double relative_time_s = clamped_age_s +
            static_cast<double>(k) * consumer_knot_dt_s;
        const double plan_knot =
            relative_time_s / plan_knot_dt_s;
        if (!std::isfinite(plan_knot) || plan_knot < 0.0)
            return false;
        const auto index = static_cast<std::size_t>(
            std::floor(plan_knot + 1.0e-9));
        if (index >= plan.horizon_knots)
            return false;
        indices[k] = index;
    }
    return true;
}

class TerrainPlanStore
{
public:
    void Publish(const TerrainMotionPlan &plan)
    {
        if (!plan.valid())
            return;
        auto next = std::make_shared<const TerrainMotionPlan>(plan);
        std::atomic_store_explicit(&latest_, std::move(next),
                                   std::memory_order_release);
    }

    std::shared_ptr<const TerrainMotionPlan> Load() const
    {
        return std::atomic_load_explicit(&latest_, std::memory_order_acquire);
    }

    std::shared_ptr<const TerrainMotionPlan> LoadUsable(double now_s) const
    {
        auto plan = Load();
        if (plan && plan->usable_at(now_s))
            return plan;
        return nullptr;
    }

private:
    std::shared_ptr<const TerrainMotionPlan> latest_;
};

} // namespace go2_terrain
