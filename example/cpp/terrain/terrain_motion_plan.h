#pragma once

#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <iomanip>
#include <sstream>
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
    // A prepared target is still owned by the explicit crawl leg until its
    // immutable touchdown boundary. This includes the pre-launch handoff;
    // requiring execution_in_flight here deadlocks CRAWL_STEP before the
    // launch path can set that bit.
    (void)execution_in_flight;
    return std::isfinite(now_s) && std::isfinite(touchdown_time_s) &&
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
    // Every body knot is provenance-bound to the immutable plan snapshot.
    TerrainPlanIdentity provenance{};
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
    // A foothold is never independently publishable: this identity must
    // match TerrainMotionPlan::identity when the timed snapshot is enabled.
    TerrainPlanIdentity provenance{};
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

enum class TerrainTimingProvenance : std::uint8_t
{
    kNone = 0,
    kLegacyPhase1 = 1,
    kStageCPlanner = 2,
    kReplay = 3,
};

inline const char *TerrainTimingProvenanceName(TerrainTimingProvenance source)
{
    switch (source)
    {
    case TerrainTimingProvenance::kLegacyPhase1: return "legacy_phase1";
    case TerrainTimingProvenance::kStageCPlanner: return "stage_c_planner";
    case TerrainTimingProvenance::kReplay: return "replay";
    default: return "none";
    }
}

// Absolute touchdown times are the sole future execution time variable.
// Phase offsets, if later exposed, remain derived telemetry.
struct TerrainContactTiming
{
    // The plan identity binds timing to the same snapshot as body and feet.
    TerrainPlanIdentity identity{};
    double period_s = 0.8;
    double duty_factor = 0.58;
    std::array<double, go2::kLegCount> touchdown_time_s{};
    std::array<bool, go2::kLegCount> touchdown_time_valid{};
    std::array<double, go2::kLegCount> liftoff_time_s{};
    std::array<bool, go2::kLegCount> liftoff_time_valid{};
    std::size_t horizon_knots = 0;
    double knot_dt_s = 0.020;
    TerrainTimingProvenance provenance = TerrainTimingProvenance::kNone;

    bool valid(const TerrainTimingBounds &bounds) const
    {
        if (!identity.valid() || provenance == TerrainTimingProvenance::kNone ||
            !bounds.valid() || !std::isfinite(period_s) ||
            !std::isfinite(duty_factor) || !std::isfinite(knot_dt_s) ||
            knot_dt_s <= 0.0 || period_s < bounds.min_period_s ||
            period_s > bounds.max_period_s ||
            duty_factor < bounds.min_duty_factor ||
            duty_factor > bounds.max_duty_factor)
            return false;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (touchdown_time_valid[leg])
            {
                const double time = touchdown_time_s[leg];
                if (!std::isfinite(time) || time < bounds.window_start_s ||
                    time > bounds.window_end_s)
                    return false;
                if (bounds.next_touchdown_time_valid[leg] &&
                    std::abs(time - bounds.next_touchdown_time_s[leg]) >
                        bounds.knot_dt_s + 1.0e-9)
                    return false;
                if (bounds.touchdown_window_valid[leg] &&
                    (time < bounds.earliest_touchdown_time_s[leg] ||
                     time > bounds.latest_touchdown_time_s[leg]))
                    return false;
            }
            if (liftoff_time_valid[leg])
            {
                const double time = liftoff_time_s[leg];
                if (!std::isfinite(time) || time < bounds.window_start_s ||
                    time > bounds.window_end_s)
                    return false;
                if (touchdown_time_valid[leg] &&
                    time <= touchdown_time_s[leg])
                    return false;
            }
        }
        return true;
    }
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
    // Legacy scalar identity fields remain source-compatible; identity is the
    // C-000 immutable value snapshot used by later consumers.
    TerrainPlanIdentity identity{};
    TerrainContactTiming contact_timing{};
    TerrainTimingBounds timing_bounds{};
    bool has_stage_c_timing = false;
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
    // Deterministic crawl target measured directly from the lidar model.
    // The planner still validates the contact schedule and consumer horizon;
    // this field fixes only the script's where, not MPC/WBC consumption.
    std::array<TerrainFootholdPrediction, go2::kLegCount>
        scripted_target{};
    // World-frame body target for the canonical pre-crawl staging pose.
    // It is derived from the lidar edge in the planner snapshot.
    bool staging_target_valid = false;
    double staging_target_world_x_m = 0.0;
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

    // Rebind every nested value after a planner changes the snapshot
    // validity horizon.  This is the only supported way to keep provenance
    // coherent; consumers never repair fields independently.
    void BindIdentity()
    {
        identity.plan_id = plan_id;
        identity.plan_epoch = plan_epoch;
        identity.map_epoch = map_epoch;
        identity.generated_at_s = generated_at_s;
        identity.valid_until_s = valid_until_s;
        contact_timing.identity = identity;
        contact_schedule.provenance = identity;
        for (std::size_t k = 0; k < kTerrainPlanMaxKnots; ++k)
        {
            body_reference[k].provenance = identity;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                predicted_foothold[k][leg].provenance = identity;
        }
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            current_support_anchor[leg].provenance = identity;
            scripted_target[leg].provenance = identity;
        }
    }

    bool valid() const
    {
        const bool identity_matches =
            identity.plan_id == plan_id &&
            identity.plan_epoch == plan_epoch &&
            identity.map_epoch == map_epoch &&
            identity.generated_at_s == generated_at_s &&
            identity.valid_until_s == valid_until_s;
        if ((status != TerrainPlanStatus::kValid &&
             status != TerrainPlanStatus::kDegraded) ||
            plan_id == 0 || plan_epoch == 0 || map_epoch == 0 ||
            !identity.valid() || !identity_matches ||
            contact_schedule.provenance.plan_id != plan_id ||
            contact_schedule.provenance.plan_epoch != plan_epoch ||
            contact_schedule.provenance.map_epoch != map_epoch ||
            contact_schedule.provenance.generated_at_s != generated_at_s ||
            contact_schedule.provenance.valid_until_s != valid_until_s ||
            !contact_schedule.valid(horizon_knots) || frame_id.empty() ||
            !std::isfinite(state_stamp_s) || !std::isfinite(generated_at_s) ||
            !std::isfinite(valid_until_s) || valid_until_s < generated_at_s ||
            horizon_knots == 0 || horizon_knots > kTerrainPlanMaxKnots)
            return false;
        for (std::size_t k = 0; k < horizon_knots; ++k)
        {
            if (!body_reference[k].valid ||
                body_reference[k].provenance.plan_id != plan_id ||
                body_reference[k].provenance.plan_epoch != plan_epoch ||
                body_reference[k].provenance.map_epoch != map_epoch ||
                body_reference[k].provenance.generated_at_s != generated_at_s ||
                body_reference[k].provenance.valid_until_s != valid_until_s)
                return false;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto &foot = predicted_foothold[k][leg];
                if (foot.valid &&
                    (foot.provenance.plan_id != plan_id ||
                     foot.provenance.plan_epoch != plan_epoch ||
                     foot.provenance.map_epoch != map_epoch ||
                     foot.provenance.generated_at_s != generated_at_s ||
                     foot.provenance.valid_until_s != valid_until_s))
                    return false;
                if (!contact_schedule.planned_contact[k][leg])
                    continue;
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
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto provenance_matches = [this](
                const TerrainPlanIdentity &source) {
                return source.plan_id == plan_id &&
                    source.plan_epoch == plan_epoch &&
                    source.map_epoch == map_epoch &&
                    source.generated_at_s == generated_at_s &&
                    source.valid_until_s == valid_until_s;
            };
            if ((current_support_anchor[leg].valid &&
                 !provenance_matches(current_support_anchor[leg].provenance)) ||
                (scripted_target[leg].valid &&
                 !provenance_matches(scripted_target[leg].provenance)))
                return false;
        }
        if (has_stage_c_timing)
        {
            if (contact_timing.identity.plan_id != plan_id ||
                contact_timing.identity.plan_epoch != plan_epoch ||
                contact_timing.identity.map_epoch != map_epoch ||
                contact_timing.identity.generated_at_s != generated_at_s ||
                contact_timing.identity.valid_until_s != valid_until_s ||
                contact_timing.horizon_knots != horizon_knots ||
                !contact_timing.valid(timing_bounds))
                return false;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (contact_timing.touchdown_time_valid[leg] &&
                    contact_timing.touchdown_time_s[leg] > valid_until_s + 1.0e-9)
                    return false;
                if (contact_timing.liftoff_time_valid[leg] &&
                    contact_timing.liftoff_time_s[leg] > valid_until_s + 1.0e-9)
                    return false;
            }
            const double last_knot_time = state_stamp_s +
                static_cast<double>(horizon_knots - 1) *
                    contact_timing.knot_dt_s;
            if (state_stamp_s < timing_bounds.window_start_s ||
                !std::isfinite(last_knot_time) ||
                last_knot_time > timing_bounds.window_end_s + 1.0e-9)
                return false;
            bool have_previous_touchdown = false;
            double previous_touchdown_time = -std::numeric_limits<double>::infinity();
            std::array<bool, go2::kLegCount> previous_contact =
                contact_schedule.measured_contact;
            std::array<bool, go2::kLegCount> saw_touchdown{};
            std::array<bool, go2::kLegCount> saw_liftoff{};
            for (std::size_t k = 0; k < horizon_knots; ++k)
            {
                std::size_t planned_contacts = 0;
                const double knot_time = state_stamp_s +
                    static_cast<double>(k) * contact_timing.knot_dt_s;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    const bool planned = contact_schedule.planned_contact[k][leg];
                    planned_contacts += planned ? 1U : 0U;
                    const auto &foot = predicted_foothold[k][leg];
                    const bool touchdown_event = planned &&
                        !previous_contact[leg];
                    const bool liftoff_event = !planned &&
                        previous_contact[leg];
                    if (touchdown_event)
                    {
                        if (!contact_timing.touchdown_time_valid[leg] ||
                            !foot.touchdown || !foot.valid ||
                            std::abs(contact_timing.touchdown_time_s[leg] -
                                     knot_time) >
                                0.5 * contact_timing.knot_dt_s + 1.0e-9)
                            return false;
                        saw_touchdown[leg] = true;
                    }
                    if (liftoff_event)
                    {
                        if (!contact_timing.liftoff_time_valid[leg] ||
                            std::abs(contact_timing.liftoff_time_s[leg] -
                                     knot_time) >
                                0.5 * contact_timing.knot_dt_s + 1.0e-9)
                            return false;
                        saw_liftoff[leg] = true;
                    }
                    if (foot.touchdown)
                    {
                        if (!planned || !foot.valid ||
                            !std::isfinite(foot.touchdown_time_s) ||
                            foot.touchdown_time_s < timing_bounds.window_start_s ||
                            foot.touchdown_time_s > timing_bounds.window_end_s ||
                            (have_previous_touchdown &&
                             foot.touchdown_time_s + 1.0e-9 <
                                 previous_touchdown_time) ||
                            !contact_timing.touchdown_time_valid[leg] ||
                            std::abs(foot.touchdown_time_s -
                                     contact_timing.touchdown_time_s[leg]) >
                                0.5 * contact_timing.knot_dt_s + 1.0e-9)
                            return false;
                        previous_touchdown_time = foot.touchdown_time_s;
                        have_previous_touchdown = true;
                    }
                    previous_contact[leg] = planned;
                }
                if (planned_contacts < 3)
                    return false;
            }
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (contact_timing.touchdown_time_valid[leg] &&
                    !saw_touchdown[leg])
                    return false;
                if (contact_timing.liftoff_time_valid[leg] &&
                    !saw_liftoff[leg])
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

enum class TerrainShadowFamily : std::uint8_t { kV2B = 0, kV3CDraft = 1 };
inline const char *TerrainShadowFamilyName(TerrainShadowFamily family)
{ return family == TerrainShadowFamily::kV3CDraft ? "v3-c-draft" : "v2-b"; }

enum class TerrainShadowRejectReason : std::uint8_t
{
    kNone = 0, kInvalidInput, kStaleMap, kUnknownTerrain, kFrameMismatch,
    kNoFoothold, kAerial, kMinimumContacts, kTwoContactTimeout,
    kDynamicInfeasible, kFrictionOrUnilateral, kTorqueProxy, kReachability,
    kSweptClearance, kBodyPosture, kUncertainty, kTimingBounds
};
constexpr std::size_t kTerrainShadowFamilyCount = 2;
constexpr std::size_t kTerrainShadowRejectReasonCount =
    static_cast<std::size_t>(TerrainShadowRejectReason::kTimingBounds) + 1;
inline const char *TerrainShadowRejectReasonName(TerrainShadowRejectReason reason)
{
    static const char *const names[] = {"none", "invalid_input", "stale_map",
        "unknown_terrain", "frame_mismatch", "no_foothold", "aerial",
        "minimum_contacts", "two_contact_timeout", "dynamic_infeasible",
        "friction_unilateral", "torque_proxy", "reachability",
        "swept_clearance", "body_posture", "uncertainty", "timing_bounds"};
    const auto index = static_cast<std::size_t>(reason);
    return index < kTerrainShadowRejectReasonCount ? names[index] : "none";
}

// Immutable observer-only snapshot; no TerrainPlanStore or execution adapter.
struct TerrainShadowSnapshot
{
    TerrainPlanIdentity identity{};
    TerrainShadowFamily family = TerrainShadowFamily::kV2B;
    double period_s = 0.0;
    double duty_factor = 0.0;
    std::array<double, go2::kLegCount> touchdown_offset_s{};
    // Shadow-only event record retained with the schedule for replay.
    TerrainContactTiming contact_timing{};
    TerrainContactSchedule contact_schedule{};
    std::array<TerrainBodyReference, kTerrainPlanMaxKnots> body_reference{};
    std::array<std::array<TerrainFootholdPrediction, go2::kLegCount>, kTerrainPlanMaxKnots> predicted_foothold{};
    double min_support_margin_m = -std::numeric_limits<double>::infinity();
    double min_dynamic_margin = -std::numeric_limits<double>::infinity();
    double min_clearance_margin_m = -std::numeric_limits<double>::infinity();
    double min_reachability_margin_m = -std::numeric_limits<double>::infinity();
    double min_edge_margin_m = -std::numeric_limits<double>::infinity();
    double min_body_posture_margin = -std::numeric_limits<double>::infinity();
    double min_uncertainty_margin_m = -std::numeric_limits<double>::infinity();
    bool valid = false;
    bool no_aerial = false;
    std::uint64_t shadow_hash = 0;
};

struct TerrainShadowDiagnostics
{
    std::uint64_t input_hash = 0, chosen_shadow_hash = 0;
    // These fields are observer-only provenance for attributing startup
    // no_foothold results. They are populated and emitted only by the
    // existing env-gated shadow diagnostic stream.
    double input_state_stamp_s = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t input_map_epoch = 0;
    double input_map_age_s = std::numeric_limits<double>::infinity();
    std::size_t input_known_cells = 0, input_total_cells = 0;
    bool input_map_valid = false, input_map_fresh = false;
    bool input_frame_valid = false;
    std::array<std::uint64_t, go2::kLegCount>
        safe_region_candidate_count_by_leg{};
    bool safe_region_ready = false;
    TerrainShadowFamily chosen_family = TerrainShadowFamily::kV2B;
    std::array<std::uint64_t, 2> candidate_count{}, feasible_count{}, rejected_count{};
    std::array<std::array<std::uint64_t, kTerrainShadowRejectReasonCount>, 2> rejection_histogram{};
    std::array<double, 2> min_support_margin_m{}, min_dynamic_margin{}, min_clearance_margin_m{};
    std::array<double, 2> min_reachability_margin_m{}, min_edge_margin_m{};
    std::array<double, 2> min_body_posture_margin{}, min_uncertainty_margin_m{};
    double latency_us = 0.0, deadline_us = 0.0;
    bool deadline_miss = false, a_empty_b_feasible = false;
    bool shadow_output_consumed = false;
    static std::string Number(double value)
    {
        if (!std::isfinite(value)) return "null";
        std::ostringstream stream; stream << std::setprecision(17) << value; return stream.str();
    }
    std::string ToJson() const
    {
        std::ostringstream stream;
        stream << "{\"input_hash\":" << input_hash
               << ",\"input_state_stamp_s\":" << Number(input_state_stamp_s)
               << ",\"input_map_epoch\":" << input_map_epoch
               << ",\"input_map_age_s\":" << Number(input_map_age_s)
               << ",\"input_map_valid\":" << (input_map_valid ? "true" : "false")
               << ",\"input_map_fresh\":" << (input_map_fresh ? "true" : "false")
               << ",\"input_frame_valid\":" << (input_frame_valid ? "true" : "false")
               << ",\"input_known_cells\":" << input_known_cells
               << ",\"input_total_cells\":" << input_total_cells
               << ",\"safe_region_candidate_count_by_leg\":["
               << safe_region_candidate_count_by_leg[0] << ","
               << safe_region_candidate_count_by_leg[1] << ","
               << safe_region_candidate_count_by_leg[2] << ","
               << safe_region_candidate_count_by_leg[3] << "]"
               << ",\"safe_region_ready\":" << (safe_region_ready ? "true" : "false")
               << ",\"chosen_shadow_hash\":" << chosen_shadow_hash
               << ",\"chosen_family\":\"" << TerrainShadowFamilyName(chosen_family)
               << "\",\"families\":[";
        for (std::size_t f = 0; f < 2; ++f) {
            if (f) stream << ',';
            stream << "{\"name\":\"" << TerrainShadowFamilyName(static_cast<TerrainShadowFamily>(f))
                   << "\",\"candidates\":" << candidate_count[f]
                   << ",\"feasible\":" << feasible_count[f]
                   << ",\"rejected\":" << rejected_count[f]
                   << ",\"min_support_margin_m\":" << Number(min_support_margin_m[f])
                   << ",\"min_dynamic_margin\":" << Number(min_dynamic_margin[f])
                   << ",\"min_clearance_margin_m\":" << Number(min_clearance_margin_m[f])
                   << ",\"min_reachability_margin_m\":" << Number(min_reachability_margin_m[f])
                   << ",\"min_edge_margin_m\":" << Number(min_edge_margin_m[f])
                   << ",\"min_body_posture_margin\":" << Number(min_body_posture_margin[f])
                   << ",\"min_uncertainty_margin_m\":" << Number(min_uncertainty_margin_m[f])
                   << ",\"rejection_histogram\":{";
            for (std::size_t r = 0; r < kTerrainShadowRejectReasonCount; ++r) {
                if (r) stream << ',';
                stream << "\"" << TerrainShadowRejectReasonName(static_cast<TerrainShadowRejectReason>(r))
                       << "\":" << rejection_histogram[f][r];
            }
            stream << "}}";
        }
        stream << "],\"latency_us\":" << Number(latency_us)
               << ",\"deadline_us\":" << Number(deadline_us)
               << ",\"deadline_miss\":" << (deadline_miss ? "true" : "false")
               << ",\"a_empty_b_feasible\":" << (a_empty_b_feasible ? "true" : "false")
               << ",\"shadow_output_consumed\":false}";
        return stream.str();
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
    // Publish is the sole write operation.  It copies one complete validated
    // value and atomically replaces the prior snapshot; no field-level update
    // can be observed by a concurrent consumer.
    bool Publish(const TerrainMotionPlan &plan)
    {
        if (!plan.valid())
            return false;
        auto next = std::make_shared<const TerrainMotionPlan>(plan);
        std::atomic_store_explicit(&latest_, std::move(next),
                                   std::memory_order_release);
        return true;
    }

    // Named replacement keeps call sites explicit without introducing a
    // partial-update API.  It has exactly the same whole-snapshot semantics.
    bool Replace(const TerrainMotionPlan &plan)
    {
        return Publish(plan);
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

    // Grace is exclusively a bounded recovery lookup.  It never changes the
    // plan's valid_until or makes an expired snapshot usable for normal
    // execution.
    std::shared_ptr<const TerrainMotionPlan> LoadWithinGrace(
        double now_s, double grace_s) const
    {
        auto plan = Load();
        if (!plan || !std::isfinite(now_s) || !std::isfinite(grace_s) ||
            grace_s < 0.0 || !plan->valid())
            return nullptr;
        return now_s <= plan->valid_until_s + grace_s ? plan : nullptr;
    }

private:
    std::shared_ptr<const TerrainMotionPlan> latest_;
};

} // namespace go2_terrain
