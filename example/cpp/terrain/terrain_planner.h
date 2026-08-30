#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>

#include "terrain_feasibility.h"
#include "terrain_motion_plan.h"
#include "terrain_crawl_script.h"

namespace go2_terrain
{

struct TerrainPlannerConfig
{
    TerrainFeasibilityConfig feasibility{};
    // Bound hard-feasibility optimization independently of the atomic
    // execution tail stored for delayed SRBD-MPC consumption.
    std::size_t horizon_knots = 24;
    // Match the Phase 1 running-trot MPC sample time at the default period.
    // The planner remains configurable, but must not invent a second timing
    // base for the contact/foothold horizon.
    double knot_dt_s = 0.020;
    double plan_validity_s = 0.15;
    double deadline_us = 5000.0;
    double min_support_margin_m = 0.015;
    // Lateral half-width of the two-contact support capsule.  A trot
    // diagonal sits ~30 degrees off the direction of travel, so the COM
    // drifts off the line at ~0.13-0.18 m/s per 0.30 m/s of forward speed;
    // 0.040 left only ~25 mm of band and rejected every terrain-retimed
    // hold.  epoch11 showed the binding case is the mid-crossing straddle:
    // front foot on the plateau, rear foot still on flat ground forces the
    // support line ~45-46 mm off the COM path, and the 0.060 band (45 mm
    // budget) rejected it by ~1 mm, freezing the plan stream mid-crossing.
    // Sized as the observed straddle geometry (~46 mm) + the 15 mm margin
    // + placement-variation headroom; see
    // docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md section 7.
    double max_two_contact_line_error_m = 0.070;
    // A planner-owned transition intent selects this wider corridor for a
    // pair committed to different surfaces.  The intent is independent of
    // the blended support-cell z values; flat-ground support keeps the drift
    // band.
    double two_contact_straddle_corridor_m = 0.120;
    double candidate_x_span_m = 0.090;
    double candidate_y_span_m = 0.070;
    double candidate_spacing_m = 0.030;
    // Prefer a feasible surface that is measurably above the currently
    // loaded terrain. The gain is sensor-derived; it is not a step-height
    // threshold or a scene-coordinate hint.
    double upward_surface_preference_weight = 1.0;
    double swing_clearance_m = 0.030;
    bool sensor_only = true;
    bool allow_actuation = false;
};

struct TerrainPlannerInput
{
    const TerrainModel *terrain = nullptr;
    double state_stamp_s = 0.0;
    double base_yaw_rad = 0.0;
    go2::Vec3 base_position_world{};
    go2::Vec3 base_velocity_world{};
    go2::Vec3 base_acceleration_world{};
    double base_roll_rad = 0.0;
    double base_pitch_rad = 0.0;
    double base_height_m = 0.0;
    double gait_phase = 0.0;
    double gait_period_s = 0.8;
    double duty_factor = 0.58;
    double commanded_vx_mps = 0.0;
    std::array<go2::Vec3, go2::kLegCount> current_feet_base{};
    std::array<go2::Vec3, go2::kLegCount> nominal_feet_base{};
    // Instantaneous nominal_feet_base is the current gait trajectory.  This
    // separate endpoint is stable through an in-flight swing and is the
    // reference for terrain candidate ranking and support prediction.
    std::array<go2::Vec3, go2::kLegCount>
        nominal_touchdown_feet_base{};
    bool nominal_touchdown_feet_valid = false;
    // A terrain candidate may replace any future touchdown whose live foot
    // path is still executable.  In-flight legs are checked from their
    // measured current foot and the execution adapter rebases that path at
    // handoff; this keeps both diagonal pairs represented in one horizon.
    std::array<bool, go2::kLegCount> terrain_retarget_allowed{};
    bool terrain_retarget_allowed_valid = false;
    // Preserve each leg's measured pre-transition surface after another leg
    // reaches the new level, so candidate ranking keeps seeking the same
    // sensor-derived surface instead of moving its reference mid-transaction.
    bool terrain_surface_transition_active = false;
    std::array<bool, go2::kLegCount>
        terrain_surface_transition_required{};
    std::array<bool, go2::kLegCount>
        terrain_surface_transition_committed{};
    std::array<bool, go2::kLegCount>
        terrain_surface_transition_source_valid{};
    std::array<double, go2::kLegCount>
        terrain_surface_transition_source_height_m{};
    // The gait/WBC hold is an observed support snapshot. While an elevated
    // target is being confirmed, planner support validation must retain those
    // anchors instead of evaluating only the next nominal diagonal.
    bool terrain_transfer_hold_active = false;
    std::array<bool, go2::kLegCount>
        terrain_transfer_hold_contact{};
    TerrainContactSchedule contact_schedule{};
    // The discrete schedule feeds the MPC horizon, while this optional
    // absolute timestamp keeps swing execution aligned with the continuous
    // gait phase used by the low-level target generator.
    std::array<double, go2::kLegCount> next_touchdown_time_s{};
    std::array<bool, go2::kLegCount> next_touchdown_time_valid{};
};

struct TerrainPlannerResult
{
    TerrainMotionPlan plan{};
    std::array<std::vector<SafeFootholdRegion>, go2::kLegCount> regions{};
    std::array<FootholdCandidate, go2::kLegCount> selected{};
    std::array<std::array<FootholdCandidate, kTerrainPlanMaxKnots>,
               go2::kLegCount> selected_by_touchdown{};
    std::array<std::array<bool, kTerrainPlanMaxKnots>, go2::kLegCount>
        selected_by_touchdown_valid{};
    std::array<std::size_t, go2::kLegCount> candidate_counts{};
    std::array<std::size_t, go2::kLegCount> swing_candidate_counts{};
    std::array<double, go2::kLegCount> max_static_region_z_by_leg{};
    std::array<double, go2::kLegCount> max_swing_candidate_z_by_leg{};
    std::array<int, go2::kLegCount> touchdown_knot_by_leg{};
    std::array<bool, go2::kLegCount> candidate_required{};
    std::array<std::uint32_t, kFootholdRejectReasonCount>
        foothold_reject_counts{};
    std::array<std::array<std::uint32_t, kFootholdRejectReasonCount>,
               go2::kLegCount> foothold_reject_counts_by_leg{};
    FootholdRejectReason dominant_foothold_reject_reason =
        FootholdRejectReason::kNone;
    std::array<FootholdRejectReason, go2::kLegCount>
        dominant_foothold_reject_by_leg{};
    int failed_leg = -1;
    int support_failure_knot = -1;
    std::uint8_t support_failure_contact_mask = 0;
    double support_failure_margin_m =
        -std::numeric_limits<double>::infinity();
    bool map_usable = false;
    bool publishable = false;
    bool safe_stop_required = false;
};

inline go2::Vec3 RotateBaseToWorld(
    const go2::Vec3 &base_position, double yaw, const go2::Vec3 &local)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
        base_position.x + c * local.x - s * local.y,
        base_position.y + s * local.x + c * local.y,
        base_position.z + local.z};
}
inline go2::Vec3 RotateWorldVectorToBase(
    double yaw, const go2::Vec3 &world_vector)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
        c * world_vector.x + s * world_vector.y,
        -s * world_vector.x + c * world_vector.y,
        world_vector.z};
}
inline go2::Vec3 PredictBasePosition(
    const go2::Vec3 &base_position, const go2::Vec3 &base_velocity,
    double dt_s)
{
    if (!std::isfinite(dt_s) ||
        !std::isfinite(base_position.x) ||
        !std::isfinite(base_position.y) ||
        !std::isfinite(base_position.z) ||
        !std::isfinite(base_velocity.x) ||
        !std::isfinite(base_velocity.y) ||
        !std::isfinite(base_velocity.z))
        return base_position;
    return {
        base_position.x + base_velocity.x * dt_s,
        base_position.y + base_velocity.y * dt_s,
        base_position.z + base_velocity.z * dt_s};
}

inline bool PendingSurfaceTransition(
    const TerrainPlannerInput &input, std::size_t leg)
{
    return leg < go2::kLegCount &&
        input.terrain_surface_transition_active &&
        input.terrain_surface_transition_required[leg] &&
        !input.terrain_surface_transition_committed[leg];
}

inline void PopulateMeasuredSupportTransitionIntent(
    const TerrainPlannerInput &input,
    const std::array<bool, go2::kLegCount> &contacts,
    std::array<bool, go2::kLegCount> &surface_transition_required,
    std::array<bool, go2::kLegCount> &surface_transition_intent_valid)
{
    surface_transition_required.fill(false);
    surface_transition_intent_valid.fill(false);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!contacts[leg])
            continue;
        surface_transition_required[leg] =
            PendingSurfaceTransition(input, leg);
        surface_transition_intent_valid[leg] = true;
    }
}

inline double Cross2D(const go2::Vec3 &a, const go2::Vec3 &b,
                      const go2::Vec3 &p)
{
    return (b.x - a.x) * (p.y - a.y) -
        (b.y - a.y) * (p.x - a.x);
}

inline double DistancePointSegment2D(
    const go2::Vec3 &a, const go2::Vec3 &b, const go2::Vec3 &p)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double norm_sq = dx * dx + dy * dy;
    const double u = norm_sq > 1.0e-12
        ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / norm_sq,
                     0.0, 1.0)
        : 0.0;
    return std::hypot(p.x - (a.x + u * dx), p.y - (a.y + u * dy));
}

inline double SupportMargin2D(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    const std::array<bool, go2::kLegCount> &contact,
    const go2::Vec3 &com, double min_margin, double max_line_error)
{
    std::array<go2::Vec3, go2::kLegCount> points{};
    std::size_t count = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (contact[leg])
            points[count++] = feet[leg];
    }
    if (count < 2)
        return -std::numeric_limits<double>::infinity();
    if (count == 2)
    {
        const double line_error = DistancePointSegment2D(
            points[0], points[1], com);
        const double length = std::hypot(
            points[1].x - points[0].x, points[1].y - points[0].y);
        const double along = length > 1.0e-9
            ? ((com.x - points[0].x) * (points[1].x - points[0].x) +
               (com.y - points[0].y) * (points[1].y - points[0].y)) / length
            : 0.0;
        const double endpoint_margin = std::min(along, length - along);
        if (line_error > max_line_error)
            return -line_error;
        return std::min(endpoint_margin, max_line_error - line_error);
    }

    // Convex hull of at most four contacts, monotonic chain in XY.
    std::vector<go2::Vec3> sorted(points.begin(), points.begin() + count);
    std::sort(sorted.begin(), sorted.end(), [](const go2::Vec3 &a,
                                                const go2::Vec3 &b) {
        if (a.x != b.x)
            return a.x < b.x;
        return a.y < b.y;
    });
    std::vector<go2::Vec3> hull;
    for (const auto &point : sorted)
    {
        while (hull.size() >= 2 && Cross2D(
                   hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lower_size = hull.size();
    for (std::size_t i = sorted.size(); i-- > 0;)
    {
        const auto &point = sorted[i];
        while (hull.size() > lower_size && Cross2D(
                   hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    if (hull.size() > 1)
        hull.pop_back();
    if (hull.size() < 3)
        return -std::numeric_limits<double>::infinity();
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < hull.size(); ++i)
    {
        const auto &a = hull[i];
        const auto &b = hull[(i + 1) % hull.size()];
        const double edge_length = std::hypot(b.x - a.x, b.y - a.y);
        if (edge_length < 1.0e-9)
            continue;
        const double cross = Cross2D(a, b, com) / edge_length;
        if (cross < 0.0)
            return cross;
        minimum = std::min(minimum, cross);
    }
    return minimum >= min_margin ? minimum : minimum;
}

// Lateral line-error bound for a two-contact support set.  The support
// gate consumes planner-owned transition intent, not the z values of the
// support feet: riser-edge map cells may blend the measured height by an
// arbitrary amount as resolution changes.  Exactly one pending transition
// means the pair is straddling the old and new planned surfaces.  If both
// feet have pending intent for the same destination, or either identity is
// unavailable, use the ordinary drift band (fail closed for missing identity).
inline double TwoContactLineErrorBound(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    const std::array<bool, go2::kLegCount> &contact,
    const TerrainPlannerConfig &config,
    const std::array<bool, go2::kLegCount> &surface_transition_required,
    const std::array<bool, go2::kLegCount> &surface_transition_intent_valid)
{
    std::size_t count = 0;
    std::size_t pending_transition_count = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!contact[leg])
            continue;
        ++count;
        if (!surface_transition_intent_valid[leg])
            return config.max_two_contact_line_error_m;
        if (surface_transition_required[leg])
            ++pending_transition_count;
    }
    if (count == 2 && pending_transition_count == 1)
        return config.two_contact_straddle_corridor_m;
    // Epoch19/20 two-contact failures are diagonal, pre-commit support
    // pairs; the recorded pair geometry is an old/new straddle, never two
    // independently committed surfaces.  Conversely, when both contacts
    // carry pending transition intent they target the same upper plane in
    // this single-riser scene, so their segment has no old/new boundary.
    // Keep the ordinary drift band for zero or two pending legs rather than
    // widening it without a geometric surface split.
    return config.max_two_contact_line_error_m;
}

inline void LogTwoContactSupportBound(
    const char *evaluation, std::uint64_t plan_id, std::size_t knot,
    const std::array<bool, go2::kLegCount> &contact,
    const std::array<bool, go2::kLegCount> &surface_transition_required,
    const std::array<bool, go2::kLegCount> &surface_transition_intent_valid,
    double bound, const TerrainPlannerConfig &config)
{
    if (std::getenv("TROT_TERRAIN_DEBUG_SUPPORT_BOUND") == nullptr)
        return;
    std::size_t contact_count = 0;
    bool any_transition_required = false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (contact[leg])
            ++contact_count;
        any_transition_required = any_transition_required ||
            (contact[leg] && surface_transition_required[leg]);
    }
    if (contact_count != 2)
        return;
    // Match the existing swing diagnostics: keep a small ordinary sample,
    // but retain every transition-intent record needed by the canary proof.
    static std::size_t debug_support_bound_prints = 0;
    static std::size_t debug_support_bound_ordinary_prints = 0;
    if ((!any_transition_required && debug_support_bound_ordinary_prints >= 256) ||
        debug_support_bound_prints >= 4096)
        return;
    ++debug_support_bound_prints;
    if (!any_transition_required)
        ++debug_support_bound_ordinary_prints;
    std::fprintf(
        stderr,
        "Terrain support bound diagnostic eval=%s plan=%llu knot=%zu "
        "contact=%d%d%d%d bound=%s bound_m=%.6f required=%d%d%d%d "
        "intent_valid=%d%d%d%d\n",
        evaluation, static_cast<unsigned long long>(plan_id), knot,
        contact[0] ? 1 : 0, contact[1] ? 1 : 0, contact[2] ? 1 : 0,
        contact[3] ? 1 : 0,
        std::abs(bound - config.two_contact_straddle_corridor_m) < 1.0e-12
            ? "straddle" : "drift",
        bound,
        surface_transition_required[0] ? 1 : 0,
        surface_transition_required[1] ? 1 : 0,
        surface_transition_required[2] ? 1 : 0,
        surface_transition_required[3] ? 1 : 0,
        surface_transition_intent_valid[0] ? 1 : 0,
        surface_transition_intent_valid[1] ? 1 : 0,
        surface_transition_intent_valid[2] ? 1 : 0,
        surface_transition_intent_valid[3] ? 1 : 0);
}

class TerrainPlanner
{
public:
    explicit TerrainPlanner(TerrainPlannerConfig config = {})
        : config_(std::move(config))
    {
        config_.horizon_knots = std::clamp<std::size_t>(
            config_.horizon_knots, 1, kTerrainPlanMaxKnots);
        config_.knot_dt_s = std::clamp(config_.knot_dt_s, 0.01, 0.10);
        config_.plan_validity_s = std::clamp(
            config_.plan_validity_s, config_.knot_dt_s, 0.50);
    }

    const TerrainPlannerConfig &config() const { return config_; }

    TerrainPlannerResult Build(const TerrainPlannerInput &input,
                               std::uint64_t plan_id) const
    {
        TerrainPlannerResult result;
        result.touchdown_knot_by_leg.fill(-1);
        result.candidate_required.fill(false);
        for (auto &valid : result.selected_by_touchdown_valid)
            valid.fill(false);
        result.swing_candidate_counts.fill(0);
        result.max_static_region_z_by_leg.fill(
            -std::numeric_limits<double>::infinity());
        result.max_swing_candidate_z_by_leg.fill(
            -std::numeric_limits<double>::infinity());
        result.plan.plan_id = plan_id;
        result.plan.plan_epoch = plan_id;
        result.plan.map_epoch = input.terrain != nullptr
            ? input.terrain->epoch : 0;
        result.plan.state_stamp_s = input.state_stamp_s;
        result.plan.generated_at_s = input.state_stamp_s;
        result.plan.valid_until_s = input.state_stamp_s +
            config_.plan_validity_s;
        result.plan.gait_phase = input.gait_phase;
        result.plan.gait_period_s = input.gait_period_s;
        result.plan.duty_factor = input.duty_factor;
        result.plan.frame_id = input.terrain != nullptr
            ? input.terrain->frame_id : "";
        result.plan.horizon_knots = config_.horizon_knots;
        result.plan.solver.attempted = true;
        result.plan.solver.deadline_us = config_.deadline_us;
        const auto start = std::chrono::steady_clock::now();
        if (!input.contact_schedule.valid(config_.horizon_knots))
        {
            result.plan.failure = TerrainPlanFailure::kInvalidInput;
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.solver.failure = TerrainPlanFailure::kInvalidInput;
            return Finish(input, std::move(result), start);
        }
        if (input.terrain == nullptr || !input.terrain->valid())
        {
            result.plan.failure = TerrainPlanFailure::kNoMap;
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.solver.failure = TerrainPlanFailure::kNoMap;
            return Finish(input, std::move(result), start);
        }
        result.plan.map_age_s = input.terrain->age_s;
        // The script target is selected directly from the current lidar map,
        // with stable cell ordering and edge stand-off. It is not obtained
        // from the stochastic candidate ranking below.
        if (std::isfinite(input.base_position_world.x) &&
            std::isfinite(input.base_position_world.y) &&
            std::isfinite(input.base_position_world.z))
        {
            const auto nominal_feet = input.nominal_touchdown_feet_valid
                ? input.nominal_touchdown_feet_base : input.nominal_feet_base;
            const double nominal_front_x = 0.5 *
                (nominal_feet[0].x + nominal_feet[1].x);
            const auto staging = MeasureTerrainStagingReference(
                *input.terrain, input.base_position_world,
                input.base_yaw_rad, nominal_front_x,
                TerrainCrawlStateMachine::kCanonicalStandoffM);
            result.plan.staging_target_valid = staging.valid;
            result.plan.staging_target_world_x_m = staging.target_world_x_m;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto measured = MeasureTerrainScriptTarget(
                    *input.terrain, static_cast<go2::Leg>(leg),
                    input.current_feet_base[leg]);
                if (!measured.valid)
                    continue;
                auto &target = result.plan.scripted_target[leg];
                target.valid = true;
                target.touchdown = true;
                target.position_world = RotateBaseToWorld(
                    input.base_position_world, input.base_yaw_rad,
                    measured.position_base);
                target.edge_margin_m = measured.edge_margin_m;
                target.swing_clearance_m = config_.swing_clearance_m;
                target.swing_lift_m = config_.swing_clearance_m;
                target.swing_peak_phase = 0.40;
                target.swing_leading_edge_phase = 0.40;
                target.swing_leading_edge_phase_valid = true;
                target.surface_transition_required = true;
                target.surface_transition_intent_valid = true;
            }
        }

        result.map_usable = input.terrain->age_s <=
            config_.feasibility.max_map_age_s &&
            input.terrain->frame_id == config_.feasibility.required_frame;
        if (!result.map_usable)
        {
            result.plan.failure = input.terrain->age_s >
                config_.feasibility.max_map_age_s
                ? TerrainPlanFailure::kStaleMap
                : TerrainPlanFailure::kInvalidInput;
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.solver.failure = result.plan.failure;
            return Finish(input, std::move(result), start);
        }

        std::array<int, go2::kLegCount> touchdown_knots{};
        std::array<go2::Vec3, go2::kLegCount>
            future_base_displacement_base{};
        std::array<bool, go2::kLegCount>
            future_base_displacement_valid{};
        future_base_displacement_valid.fill(false);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            touchdown_knots[leg] = CandidateTouchdownKnot(input, leg);
            result.touchdown_knot_by_leg[leg] = touchdown_knots[leg];
            if (touchdown_knots[leg] < 0)
                continue;
            const bool exact_touchdown_time =
                input.next_touchdown_time_valid[leg] &&
                std::isfinite(input.next_touchdown_time_s[leg]);
            const double touchdown_dt_s = exact_touchdown_time
                ? input.next_touchdown_time_s[leg] - input.state_stamp_s
                : static_cast<double>(touchdown_knots[leg]) *
                      config_.knot_dt_s;
            if (!(touchdown_dt_s > 0.0) ||
                !std::isfinite(touchdown_dt_s) ||
                !std::isfinite(input.base_yaw_rad) ||
                !std::isfinite(input.base_velocity_world.x) ||
                !std::isfinite(input.base_velocity_world.y) ||
                !std::isfinite(input.base_velocity_world.z))
                continue;
            future_base_displacement_base[leg] =
                RotateWorldVectorToBase(
                    input.base_yaw_rad,
                    {input.base_velocity_world.x * touchdown_dt_s,
                     input.base_velocity_world.y * touchdown_dt_s,
                     input.base_velocity_world.z * touchdown_dt_s});
            future_base_displacement_valid[leg] = true;
        }
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            result.regions[leg] = BuildSafeFootholdRegions(
                *input.terrain, static_cast<go2::Leg>(leg),
                config_.feasibility,
                future_base_displacement_valid[leg]
                    ? &future_base_displacement_base[leg] : nullptr);
            result.candidate_counts[leg] = result.regions[leg].size();
            for (const auto &region : result.regions[leg])
            {
                if (region.valid && std::isfinite(region.center.z))
                    result.max_static_region_z_by_leg[leg] = std::max(
                        result.max_static_region_z_by_leg[leg],
                        region.center.z);
            }
        }

        if (config_.sensor_only || !config_.allow_actuation)
        {
            result.plan.status = TerrainPlanStatus::kDegraded;
            result.plan.fallback_to_phase1 = true;
            result.publishable = false;
            return Finish(input, std::move(result), start);
        }

        // Select a feasible touchdown for each leg that first enters stance
        // in the supplied Phase 1 schedule.  The schedule remains an input in
        // Stage B; the planner cannot switch topology or phase offsets.
        const double observed_support_surface_height_m =
            ObservedSupportSurfaceHeight(input);
        struct CandidateOption
        {
            double score = 0.0;
            int forward_elevated_surface = 0;
            FootholdCandidate candidate{};
        };
        std::array<std::vector<CandidateOption>, go2::kLegCount>
            candidate_options{};
        constexpr std::size_t kMaxJointOptionsPerLeg = 8;
        constexpr std::size_t kMaxSwingEvaluationsPerLeg = 32;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int touchdown_knot = touchdown_knots[leg];
            result.touchdown_knot_by_leg[leg] = touchdown_knot;
            const bool has_touchdown = touchdown_knot >= 0;
            if (!has_touchdown)
                continue;
            result.candidate_required[leg] =
                TerrainTouchdownRetargetAllowed(input, leg);
            if (!result.candidate_required[leg])
                continue;
            const double observed_leg_surface_height_m =
                ObservedTerrainReferenceHeight(input, leg);
            const double observed_surface_height_m =
                std::isfinite(observed_leg_surface_height_m)
                    ? observed_leg_surface_height_m
                    : observed_support_surface_height_m;
            const double command_direction =
                input.commanded_vx_mps > 1.0e-3 ? 1.0
                : (input.commanded_vx_mps < -1.0e-3 ? -1.0 : 0.0);
            const double forward_tolerance = 0.5 * std::max(
                1.0e-3, config_.candidate_spacing_m);
            const auto forward_elevated_surface =
                [&](const go2::Vec3 &position, double uncertainty) {
                    if (command_direction == 0.0 ||
                        !std::isfinite(observed_surface_height_m) ||
                        !std::isfinite(position.z) ||
                        !std::isfinite(input.current_feet_base[leg].x))
                        return 0;
                    const double surface_delta = position.z -
                        observed_surface_height_m;
                    const double deadband = std::max(
                        2.0 * std::max(0.0, uncertainty),
                        0.5 * config_.feasibility.foot_patch_radius_m);
                    const double directional_progress = command_direction *
                        (position.x - input.current_feet_base[leg].x);
                    return surface_delta > deadband &&
                        directional_progress >= -forward_tolerance ? 1 : 0;
                };
            const auto has_surface_standoff =
                [&](const go2::Vec3 &position) {
                    if (command_direction <= 0.0)
                        return true;
                    return go2_terrain::HasForwardElevatedSurfaceStandoff(
                        *input.terrain, position, observed_surface_height_m,
                        config_.feasibility.elevated_surface_standoff_m,
                        config_.feasibility.foot_patch_radius_m);
                };
            const auto bias_surface_position =
                [&](go2::Vec3 position) {
                    if (command_direction <= 0.0 ||
                        !std::isfinite(observed_surface_height_m) ||
                        position.z <= observed_surface_height_m + std::max(
                            2.0 * 0.0,
                            0.5 * config_.feasibility.foot_patch_radius_m))
                        return position;
                    const double edge_x =
                        go2_terrain::ForwardElevatedSurfaceEdgeX(
                            *input.terrain, position,
                            config_.feasibility.foot_patch_radius_m);
                    if (std::isfinite(edge_x))
                        position.x = std::max(
                            position.x,
                            edge_x +
                                config_.feasibility.elevated_surface_edge_bias_m +
                                config_.feasibility.elevated_surface_standoff_m);
                    return position;
                };
            const auto candidate_score = [&](const go2::Vec3 &position,
                                             double uncertainty,
                                             double edge_margin) {
                const auto nominal_touchdown =
                    NominalTouchdownFoot(input, leg);
                const double displacement = std::hypot(
                    position.x - nominal_touchdown.x,
                    position.y - nominal_touchdown.y);
                const double upward_surface_gain_m =
                    std::isfinite(observed_surface_height_m) &&
                            std::isfinite(position.z)
                        ? std::max(
                              0.0,
                              position.z - observed_surface_height_m)
                        : 0.0;
                return displacement + 0.5 * uncertainty -
                    0.1 * edge_margin -
                    config_.upward_surface_preference_weight *
                        upward_surface_gain_m;
            };

            // Region construction already performed the static model, surface,
            // IK and reachability gates.  Bound the expensive swept-volume
            // evaluation using the same sensor-derived ranking while retaining
            // every static-safe region in result.regions for diagnostics.
            std::vector<std::size_t> region_order;
            region_order.reserve(result.regions[leg].size());
            for (std::size_t region_index = 0;
                 region_index < result.regions[leg].size(); ++region_index)
            {
                if (result.regions[leg][region_index].valid)
                    region_order.push_back(region_index);
            }
            std::sort(region_order.begin(), region_order.end(),
                      [&](std::size_t lhs, std::size_t rhs) {
                          const auto &lhs_region = result.regions[leg][lhs];
                          const auto &rhs_region = result.regions[leg][rhs];
                          const int lhs_surface =
                              forward_elevated_surface(
                                  lhs_region.center, lhs_region.uncertainty_m);
                          const int rhs_surface =
                              forward_elevated_surface(
                                  rhs_region.center, rhs_region.uncertainty_m);
                          if (lhs_surface != rhs_surface)
                              return lhs_surface > rhs_surface;
                          const double lhs_score = candidate_score(
                              lhs_region.center, lhs_region.uncertainty_m,
                              lhs_region.edge_margin_m);
                          const double rhs_score = candidate_score(
                              rhs_region.center, rhs_region.uncertainty_m,
                              rhs_region.edge_margin_m);
                          if (lhs_score != rhs_score)
                              return lhs_score < rhs_score;
                          return lhs_region.region_id < rhs_region.region_id;
                      });
            if (region_order.size() > kMaxSwingEvaluationsPerLeg)
                region_order.resize(kMaxSwingEvaluationsPerLeg);

            for (const std::size_t region_index : region_order)
            {
                const auto &region = result.regions[leg][region_index];
                const auto nominal = NominalTouchdownFoot(input, leg);
                const auto &current = input.current_feet_base[leg];
                const go2::Vec3 desired_position =
                    std::isfinite(nominal.x) && std::isfinite(nominal.y)
                        ? nominal : current;
                const go2::Vec3 representative = bias_surface_position(
                    RegionRepresentative(
                        region, desired_position, input.terrain->resolution_m));
                FootholdCandidate candidate = EvaluateFoothold(
                    *input.terrain, static_cast<go2::Leg>(leg),
                    representative.x, representative.y,
                    config_.feasibility, nullptr,
                    std::numeric_limits<double>::infinity(),
                    future_base_displacement_valid[leg]
                        ? &future_base_displacement_base[leg] : nullptr);
                if (!candidate.hard_feasible ||
                    !has_surface_standoff(candidate.foot_position))
                    continue;
                candidate.region_id = region.region_id;
                candidate.support_margin_m = region.support_margin_m;
                FootholdRejectReason swing_reject_reason =
                    FootholdRejectReason::kSwingClearance;
                if (!CheckSwingClearance(
                        *input.terrain, input.current_feet_base[leg],
                        go2::ContactPatchToFootSite(candidate.foot_position),
                        config_.swing_clearance_m,
                        candidate.swing_clearance_m, &swing_reject_reason,
                        static_cast<go2::Leg>(leg), &candidate.swing_lift_m,
                        &candidate.swing_peak_phase,
                        &candidate.swing_leading_edge_phase,
                        &candidate.swing_leading_edge_phase_valid))
                {
                    const auto reason = static_cast<std::size_t>(
                        swing_reject_reason);
                    if (reason < result.foothold_reject_counts.size())
                    {
                        ++result.foothold_reject_counts[reason];
                        ++result.foothold_reject_counts_by_leg[leg][reason];
                    }
                    continue;
                }
                candidate.hard_feasible = true;
                candidate.collision_margin_m = candidate.swing_clearance_m;
                const double score = candidate_score(
                    candidate.foot_position, candidate.uncertainty_m,
                    candidate.edge_margin_m);
                const int surface_rank = forward_elevated_surface(
                    candidate.foot_position, candidate.uncertainty_m);
                // Latch the planner's target-surface intent once, before the
                // candidate enters the plan.  The support gate consumes this
                // identity and never re-derives it from blended foothold z.
                candidate.surface_transition_required =
                    surface_rank > 0 ||
                    (input.terrain_surface_transition_active &&
                     input.terrain_surface_transition_required[leg]);
                candidate.surface_transition_intent_valid = true;
                result.max_swing_candidate_z_by_leg[leg] = std::max(
                    result.max_swing_candidate_z_by_leg[leg],
                    candidate.foot_position.z);
                candidate_options[leg].push_back({
                    score, surface_rank, candidate});
            }
            auto &options = candidate_options[leg];
            std::sort(options.begin(), options.end(),
                      [](const CandidateOption &lhs,
                         const CandidateOption &rhs) {
                          if (lhs.forward_elevated_surface !=
                              rhs.forward_elevated_surface)
                              return lhs.forward_elevated_surface >
                                  rhs.forward_elevated_surface;
                          if (lhs.score != rhs.score)
                              return lhs.score < rhs.score;
                          return lhs.candidate.region_id <
                              rhs.candidate.region_id;
                      });
            if (options.size() > kMaxJointOptionsPerLeg)
                options.resize(kMaxJointOptionsPerLeg);
            result.swing_candidate_counts[leg] = options.size();
            if (options.empty())
            {
                result.failed_leg = static_cast<int>(leg);
                for (std::size_t reason = 1;
                     reason < result.foothold_reject_counts.size(); ++reason)
                {
                    if (result.foothold_reject_counts[reason] >
                        result.foothold_reject_counts[
                            static_cast<std::size_t>(
                                result.dominant_foothold_reject_reason)])
                        result.dominant_foothold_reject_reason =
                            static_cast<FootholdRejectReason>(reason);
                    if (result.foothold_reject_counts_by_leg[leg][reason] >
                        result.foothold_reject_counts_by_leg[leg][
                            static_cast<std::size_t>(
                                result.dominant_foothold_reject_by_leg[leg])])
                        result.dominant_foothold_reject_by_leg[leg] =
                            static_cast<FootholdRejectReason>(reason);
                }
                result.plan.failure = TerrainPlanFailure::kNoSafeFoothold;
                result.plan.status = TerrainPlanStatus::kRejected;
                result.plan.solver.failure = result.plan.failure;
                return Finish(input, std::move(result), start);
            }
        }

        // A per-leg optimum is not a whole-body plan: it can place the CoM
        // outside the support polygon at the next diagonal exchange.  Search a
        // bounded, deterministic Cartesian product of the best sensor-derived
        // candidates and retain only complete combinations that satisfy the
        // existing support margin.  This is a planner coupling step; it does
        // not alter the contact topology or relax any feasibility threshold.
        std::array<FootholdCandidate, go2::kLegCount> joint_selection{};
        std::array<FootholdCandidate, go2::kLegCount>
            best_feasible_selection{};
        std::array<FootholdCandidate, go2::kLegCount>
            best_infeasible_selection{};
        bool found_feasible_selection = false;
        bool found_infeasible_selection = false;
        double best_infeasible_margin =
            -std::numeric_limits<double>::infinity();
        int best_infeasible_knot = -1;
        std::uint8_t best_infeasible_contact_mask = 0;
        double best_infeasible_failure_margin =
            -std::numeric_limits<double>::infinity();

        // Group the bounded search by the sensor-derived touchdown event.
        // Running-trot support constraints are dominated by the two legs in
        // the same scheduled event; visiting those variables together lets a
        // bad support pair fail before unrelated later-event combinations are
        // expanded.  The order comes only from the supplied schedule.
        std::array<std::size_t, go2::kLegCount> search_order{};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            search_order[leg] = leg;
        std::stable_sort(
            search_order.begin(), search_order.end(),
            [&](std::size_t lhs, std::size_t rhs) {
                if (result.candidate_required[lhs] !=
                    result.candidate_required[rhs])
                    return result.candidate_required[lhs];
                if (touchdown_knots[lhs] != touchdown_knots[rhs])
                    return touchdown_knots[lhs] < touchdown_knots[rhs];
                return lhs < rhs;
            });

        int maximum_elevated_surface_count = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!result.candidate_required[leg])
                continue;
            if (std::any_of(
                    candidate_options[leg].begin(),
                    candidate_options[leg].end(),
                    [](const CandidateOption &option) {
                        return option.forward_elevated_surface > 0;
                    }))
                ++maximum_elevated_surface_count;
        }
        const auto visit_joint_candidates =
            [&](auto &&self, std::size_t depth,
                int elevated_surface_count,
                int target_elevated_surface_count) -> bool {
                if (elevated_surface_count > target_elevated_surface_count)
                    return false;
                int remaining_elevated = 0;
                for (std::size_t remaining = depth;
                     remaining < go2::kLegCount; ++remaining)
                {
                    const std::size_t leg = search_order[remaining];
                    if (!result.candidate_required[leg])
                        continue;
                    if (std::any_of(
                            candidate_options[leg].begin(),
                            candidate_options[leg].end(),
                            [](const CandidateOption &option) {
                                return option.forward_elevated_surface > 0;
                            }))
                        ++remaining_elevated;
                }
                if (elevated_surface_count + remaining_elevated <
                    target_elevated_surface_count)
                    return false;
                if (depth == go2::kLegCount)
                {
                    if (elevated_surface_count !=
                        target_elevated_surface_count)
                        return false;
                    if (!SupportFeasibleSelection(
                            input, joint_selection, result))
                    {
                        const double margin = result.support_failure_margin_m;
                        if (!found_infeasible_selection ||
                            margin > best_infeasible_margin)
                        {
                            found_infeasible_selection = true;
                            best_infeasible_margin = margin;
                            best_infeasible_selection = joint_selection;
                            best_infeasible_knot =
                                result.support_failure_knot;
                            best_infeasible_contact_mask =
                                result.support_failure_contact_mask;
                            best_infeasible_failure_margin = margin;
                        }
                        return false;
                    }
                    found_feasible_selection = true;
                    best_feasible_selection = joint_selection;
                    return true;
                }
                const std::size_t leg = search_order[depth];
                if (!result.candidate_required[leg])
                    return self(self, depth + 1,
                                elevated_surface_count,
                                target_elevated_surface_count);
                for (const auto &option : candidate_options[leg])
                {
                    joint_selection[leg] = option.candidate;
                    if (self(self, depth + 1,
                             elevated_surface_count +
                                 option.forward_elevated_surface,
                             target_elevated_surface_count))
                        return true;
                }
                return false;
            };
        for (int elevated = maximum_elevated_surface_count;
             elevated >= 0 && !found_feasible_selection; --elevated)
            visit_joint_candidates(
                visit_joint_candidates, 0, 0, elevated);

        if (!found_feasible_selection)
        {
            if (found_infeasible_selection)
            {
                result.selected = best_infeasible_selection;
                SeedTouchdownSelections(input, result);
                PopulatePlan(input, result);
                result.support_failure_knot = best_infeasible_knot;
                result.support_failure_contact_mask =
                    best_infeasible_contact_mask;
                result.support_failure_margin_m =
                    best_infeasible_failure_margin;
            }
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.failure = TerrainPlanFailure::kSupportInfeasible;
            result.plan.solver.failure = result.plan.failure;
            return Finish(input, std::move(result), start);
        }
        result.selected = best_feasible_selection;
        SeedTouchdownSelections(input, result);
        PopulateFutureTouchdownSelections(input, result);
        PopulatePlan(input, result);
        TerrainPlannerInput coherent_input = input;
        bool retimed = false;
        if (!BuildRetimedPlanInput(
                input, result, coherent_input, retimed))
        {
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.failure = TerrainPlanFailure::kNoSafeFoothold;
            result.plan.solver.failure = result.plan.failure;
            return Finish(input, std::move(result), start);
        }
        if (retimed)
            PopulatePlan(coherent_input, result);
        ExtendValidityThroughTouchdowns(result.plan);
        if (!SupportFeasible(coherent_input, result))
        {
            SeedTouchdownSelections(coherent_input, result);
            PopulatePlan(coherent_input, result);
            ExtendValidityThroughTouchdowns(result.plan);
        }
        if (!SupportFeasible(coherent_input, result))
        {
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.failure = TerrainPlanFailure::kSupportInfeasible;
            result.plan.solver.failure = result.plan.failure;
            return Finish(input, std::move(result), start);
        }
        if (!ExtendExecutionSupportTail(result.plan))
        {
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.failure = TerrainPlanFailure::kSupportInfeasible;
            result.plan.solver.failure = result.plan.failure;
            return Finish(input, std::move(result), start);
        }
        if (result.plan.velocity_request.valid)
            result.plan.velocity_request.valid_until_s =
                result.plan.valid_until_s;
        result.plan.status = TerrainPlanStatus::kValid;
        result.plan.fallback_to_phase1 = false;
        result.plan.solver.success = true;
        result.publishable = result.plan.valid();
        return Finish(input, std::move(result), start);
    }

private:
    static go2::Vec3 NominalTouchdownFoot(
        const TerrainPlannerInput &input, std::size_t leg)
    {
        if (input.nominal_touchdown_feet_valid &&
            leg < go2::kLegCount &&
            std::isfinite(input.nominal_touchdown_feet_base[leg].x) &&
            std::isfinite(input.nominal_touchdown_feet_base[leg].y) &&
            std::isfinite(input.nominal_touchdown_feet_base[leg].z))
            return input.nominal_touchdown_feet_base[leg];
        return input.nominal_feet_base[leg];
    }

    static bool TerrainTouchdownRetargetAllowed(
        const TerrainPlannerInput &input, std::size_t leg)
    {
        return !input.terrain_retarget_allowed_valid ||
            (leg < go2::kLegCount && input.terrain_retarget_allowed[leg]);
    }

    static go2::Vec3 RegionRepresentative(
        const SafeFootholdRegion &region, const go2::Vec3 &desired,
        double map_resolution_m)
    {
        if (!std::isfinite(desired.x) || !std::isfinite(desired.y) ||
            region.vertex_count == 0 ||
            region.vertex_count > SafeFootholdRegion::kMaxVertices)
            return region.center;
        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (std::size_t vertex = 0; vertex < region.vertex_count; ++vertex)
        {
            const auto &point = region.vertices[vertex];
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        // At the common 5 cm map / 2.5 cm foot-patch boundary the safe-region
        // construction can have zero numerical interior.  The candidate is
        // still re-evaluated against the full sensor patch below, so use the
        // containing map cell as the representative domain instead of forcing
        // a harmless flat-ground quantization offset into the gait.
        if (max_x - min_x <= 1.0e-6 &&
            std::isfinite(map_resolution_m) && map_resolution_m > 0.0)
        {
            min_x = region.center.x - 0.5 * map_resolution_m;
            max_x = region.center.x + 0.5 * map_resolution_m;
        }
        if (max_y - min_y <= 1.0e-6 &&
            std::isfinite(map_resolution_m) && map_resolution_m > 0.0)
        {
            min_y = region.center.y - 0.5 * map_resolution_m;
            max_y = region.center.y + 0.5 * map_resolution_m;
        }
        if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
            !std::isfinite(min_y) || !std::isfinite(max_y) ||
            desired.x < min_x || desired.x > max_x ||
            desired.y < min_y || desired.y > max_y)
            return region.center;
        return {desired.x, desired.y, region.center.z};
    }

    static double ObservedTerrainHeightAt(
        const TerrainPlannerInput &input, std::size_t leg)
    {
        if (input.terrain == nullptr || leg >= go2::kLegCount)
            return std::numeric_limits<double>::quiet_NaN();
        TerrainPatch patch;
        if (!input.terrain->SamplePatch(
                input.current_feet_base[leg].x,
                input.current_feet_base[leg].y,
                input.terrain->resolution_m * 0.5,
                patch) ||
            !patch.valid || !patch.all_known ||
            !std::isfinite(patch.center_height_m))
            return std::numeric_limits<double>::quiet_NaN();
        return patch.center_height_m;
    }

    static double ObservedSupportSurfaceHeight(
        const TerrainPlannerInput &input)
    {
        if (input.terrain == nullptr ||
            !input.contact_schedule.measured_valid)
            return std::numeric_limits<double>::quiet_NaN();
        std::vector<double> heights;
        heights.reserve(go2::kLegCount);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!input.contact_schedule.measured_contact[leg])
                continue;
            const double height = ObservedTerrainHeightAt(input, leg);
            if (std::isfinite(height))
                heights.push_back(height);
        }
        if (heights.empty())
            return std::numeric_limits<double>::quiet_NaN();
        const auto middle = heights.begin() +
            static_cast<std::ptrdiff_t>(heights.size() / 2);
        std::nth_element(heights.begin(), middle, heights.end());
        return *middle;
    }

    static double ObservedTerrainReferenceHeight(
        const TerrainPlannerInput &input, std::size_t leg)
    {
        if (input.terrain_surface_transition_active &&
            leg < go2::kLegCount &&
            input.terrain_surface_transition_required[leg] &&
            !input.terrain_surface_transition_committed[leg] &&
            input.terrain_surface_transition_source_valid[leg] &&
            std::isfinite(
                input.terrain_surface_transition_source_height_m[leg]))
            return input.terrain_surface_transition_source_height_m[leg];
        // A non-contact leg is in flight: its map footprint is not a loaded
        // support surface. Use the measured support median as the reference
        // so an upper surface ahead cannot be mistaken for current ground.
        if (input.contact_schedule.measured_valid &&
            leg < go2::kLegCount &&
            input.contact_schedule.measured_contact[leg])
        {
            const double measured_height =
                ObservedTerrainHeightAt(input, leg);
            if (std::isfinite(measured_height))
                return measured_height;
        }
        return ObservedSupportSurfaceHeight(input);
    }

    static double ObservedSupportSurfaceWorldHeight(
        const TerrainPlannerInput &input)
    {
        const double surface_height_m = ObservedSupportSurfaceHeight(input);
        if (!std::isfinite(surface_height_m) ||
            !std::isfinite(input.base_position_world.z))
            return std::numeric_limits<double>::quiet_NaN();
        return input.base_position_world.z + surface_height_m;
    }

    bool SupportFeasibleSelection(
        const TerrainPlannerInput &input,
        const std::array<FootholdCandidate, go2::kLegCount> &selection,
        TerrainPlannerResult &result) const
    {
        result.plan.min_support_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.min_uncertainty_inflated_support_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.uncertainty_m = 0.0;
        result.support_failure_knot = -1;
        result.support_failure_contact_mask = 0;
        result.support_failure_margin_m =
            -std::numeric_limits<double>::infinity();

        std::array<int, go2::kLegCount> touchdown_knots{};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            touchdown_knots[leg] = CandidateTouchdownKnot(input, leg);
            if (selection[leg].hard_feasible)
                result.plan.uncertainty_m = std::max(
                    result.plan.uncertainty_m, selection[leg].uncertainty_m);
        }

        std::size_t support_knots = 0;
        std::size_t support_horizon = config_.horizon_knots;
        if (!input.terrain_surface_transition_active)
        {
            // Before both front commits, do not reject an otherwise
            // usable target because a later preview knot enters the
            // pre-advance straddle geometry. The post-commit planner pass
            // validates the complete atomic horizon once transition intent
            // is latched.
            int latest_front_touchdown = -1;
            for (std::size_t leg = 0; leg < 2; ++leg)
                latest_front_touchdown = std::max(
                    latest_front_touchdown, touchdown_knots[leg]);
            if (latest_front_touchdown >= 0)
                support_horizon = std::min(
                    support_horizon,
                    static_cast<std::size_t>(latest_front_touchdown + 1));
        }
        for (std::size_t k = 0; k < support_horizon; ++k)
        {
            std::array<go2::Vec3, go2::kLegCount> feet{};
            std::array<bool, go2::kLegCount> contacts =
                input.contact_schedule.planned_contact[k];
            std::array<bool, go2::kLegCount> hold_anchor{};
            if (input.terrain_transfer_hold_active)
            {
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                    hold_anchor[leg] = input.terrain_transfer_hold_contact[leg];
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                    contacts[leg] = contacts[leg] || hold_anchor[leg];
            }
            std::array<bool, go2::kLegCount> surface_transition_required{};
            std::array<bool, go2::kLegCount> surface_transition_intent_valid{};
            std::size_t contact_count = 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!contacts[leg])
                    continue;
                surface_transition_intent_valid[leg] =
                    input.contact_schedule.measured_valid &&
                    input.contact_schedule.measured_contact[leg];
                surface_transition_required[leg] =
                    input.terrain_surface_transition_active &&
                    input.terrain_surface_transition_required[leg] &&
                    !input.terrain_surface_transition_committed[leg];
                const bool use_selected =
                    touchdown_knots[leg] >= 0 &&
                    static_cast<int>(k) >= touchdown_knots[leg];
                if (hold_anchor[leg])
                {
                    feet[leg] = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        input.current_feet_base[leg]);
                }
                else if (use_selected)
                {
                    surface_transition_intent_valid[leg] =
                        selection[leg].surface_transition_intent_valid;
                    surface_transition_required[leg] =
                        surface_transition_required[leg] ||
                        selection[leg].surface_transition_required;
                    if (selection[leg].hard_feasible)
                    {
                        feet[leg] = RotateBaseToWorld(
                            input.base_position_world, input.base_yaw_rad,
                            selection[leg].foot_position);
                    }
                    else if (!TerrainTouchdownRetargetAllowed(input, leg))
                    {
                        feet[leg] = RotateBaseToWorld(
                            input.base_position_world, input.base_yaw_rad,
                            NominalTouchdownFoot(input, leg));
                    }
                    else
                        return false;
                }
                else
                {
                    feet[leg] = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        input.current_feet_base[leg]);
                }
                ++contact_count;
            }
            // A running trot may intentionally have an aerial knot; static
            // support geometry does not apply there. A single loaded leg
            // remains rejected.
            if (contact_count == 0)
                continue;
            if (contact_count < 2)
                return false;
            ++support_knots;

            bool current_confirmed_support =
                input.contact_schedule.measured_valid;
            std::size_t measured_count = 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (input.contact_schedule.measured_contact[leg])
                    ++measured_count;
                if (!contacts[leg])
                    continue;
                if (!input.contact_schedule.measured_contact[leg])
                    current_confirmed_support = false;
                if (!hold_anchor[leg] && touchdown_knots[leg] >= 0 &&
                    static_cast<int>(k) >= touchdown_knots[leg])
                    current_confirmed_support = false;
            }
            if (current_confirmed_support && measured_count >= 2)
            {
                // Keep pending transition intent while replacing planned
                // geometry with measured support; only committed legs clear
                // their required bit.
                contacts = input.contact_schedule.measured_contact;
                PopulateMeasuredSupportTransitionIntent(
                    input, contacts, surface_transition_required,
                    surface_transition_intent_valid);
                contact_count = 0;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    if (!contacts[leg])
                        continue;
                    feet[leg] = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        input.current_feet_base[leg]);
                    ++contact_count;
                }
                if (contact_count < 2)
                    return false;
            }

            // Candidate combinations are scored against the same future
            // body reference later written into the atomic plan.  Using the
            // current base here could select a combination that only passes
            // before the body advances, then gets rejected by SupportFeasible
            // at the actual horizon knot.
            const go2::Vec3 support_body = PredictBasePosition(
                input.base_position_world, input.base_velocity_world,
                static_cast<double>(k) * config_.knot_dt_s);
            const double support_bound = TwoContactLineErrorBound(
                feet, contacts, config_, surface_transition_required,
                surface_transition_intent_valid);
            LogTwoContactSupportBound(
                "selection", result.plan.plan_id, k, contacts,
                surface_transition_required, surface_transition_intent_valid,
                support_bound, config_);
            const double margin = SupportMargin2D(
                feet, contacts, support_body, config_.min_support_margin_m,
                support_bound);
            result.plan.min_support_margin_m = std::min(
                result.plan.min_support_margin_m, margin);
            result.plan.min_uncertainty_inflated_support_margin_m = std::min(
                result.plan.min_uncertainty_inflated_support_margin_m,
                margin - result.plan.uncertainty_m);
            if (!std::isfinite(margin) ||
                margin < config_.min_support_margin_m)
            {
                result.support_failure_knot = static_cast<int>(k);
                result.support_failure_contact_mask = 0;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                    if (contacts[leg])
                        result.support_failure_contact_mask |=
                            static_cast<std::uint8_t>(1u << leg);
                result.support_failure_margin_m = margin;
                return false;
            }
        }
        return support_knots > 0;
    }

    bool SupportFeasible(const TerrainPlannerInput &input,
                         TerrainPlannerResult &result) const
    {
        result.plan.min_support_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.min_uncertainty_inflated_support_margin_m =
            std::numeric_limits<double>::infinity();
        std::size_t support_knots = 0;
        std::size_t support_horizon = result.plan.horizon_knots;
        if (!input.terrain_surface_transition_active)
        {
            // Match selection: before both front commits, late preview knots
            // belong to the future body-advance phase and must not veto the
            // front target that arms that phase.
            int latest_front_touchdown = -1;
            for (std::size_t leg = 0; leg < 2; ++leg)
                latest_front_touchdown = std::max(
                    latest_front_touchdown, CandidateTouchdownKnot(input, leg));
            if (latest_front_touchdown >= 0)
                support_horizon = std::min(
                    support_horizon,
                    static_cast<std::size_t>(latest_front_touchdown + 1));
        }
        for (std::size_t k = 0; k < support_horizon; ++k)
        {
            std::array<go2::Vec3, go2::kLegCount> feet{};
            std::size_t contact_count = 0;
            std::array<bool, go2::kLegCount> contacts =
                result.plan.contact_schedule.planned_contact[k];
            std::array<bool, go2::kLegCount> hold_anchor{};
            if (input.terrain_transfer_hold_active)
            {
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                    hold_anchor[leg] = input.terrain_transfer_hold_contact[leg];
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                    contacts[leg] = contacts[leg] || hold_anchor[leg];
            }
            std::array<bool, go2::kLegCount> surface_transition_required{};
            std::array<bool, go2::kLegCount> surface_transition_intent_valid{};
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const bool use_hold_anchor = hold_anchor[leg];
                if (!contacts[leg] ||
                    (!use_hold_anchor &&
                     !result.plan.predicted_foothold[k][leg].valid))
                    continue;
                ++contact_count;
                if (use_hold_anchor)
                {
                    feet[leg] = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        input.current_feet_base[leg]);
                    surface_transition_intent_valid[leg] =
                        input.contact_schedule.measured_valid;
                    continue;
                }
                const auto &predicted = result.plan.predicted_foothold[k][leg];
                feet[leg] = predicted.position_world;
                surface_transition_intent_valid[leg] =
                    predicted.surface_transition_intent_valid;
                surface_transition_required[leg] =
                    predicted.surface_transition_required ||
                    (input.terrain_surface_transition_active &&
                     input.terrain_surface_transition_required[leg] &&
                     !input.terrain_surface_transition_committed[leg]);
                surface_transition_intent_valid[leg] =
                    surface_transition_intent_valid[leg] ||
                    (input.terrain_surface_transition_active &&
                     input.terrain_surface_transition_required[leg]);
            }
            // A running trot may intentionally have an aerial knot; static support
            // geometry does not apply there. A single loaded leg remains rejected.
            if (contact_count == 0)
                continue;
            if (contact_count < 2)
                return false;
            ++support_knots;
            // A measured support set is an observation, not a replacement for
            // the planned contact schedule. Before the next planned
            // touchdown, however, it is the only trustworthy geometry for the
            // support that is already carrying load. Validate that observed
            // support set here instead of rejecting a healthy stance solely
            // because its kinematic foot line is offset from the body origin.
            bool current_confirmed_support =
                input.contact_schedule.measured_valid;
            std::size_t measured_count = 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (input.contact_schedule.measured_contact[leg])
                    ++measured_count;
                if (!contacts[leg])
                    continue;
                if (!input.contact_schedule.measured_contact[leg])
                    current_confirmed_support = false;
                const int touchdown = CandidateTouchdownKnot(input, leg);
                if (!hold_anchor[leg] && touchdown >= 0 &&
                    static_cast<int>(k) >= touchdown)
                    current_confirmed_support = false;
            }
            if (current_confirmed_support && measured_count >= 2)
            {
                // Preserve an uncommitted leg transition across this
                // measured-support replacement; committed measured legs
                // retain valid geometry but no longer require the corridor.
                contacts = input.contact_schedule.measured_contact;
                PopulateMeasuredSupportTransitionIntent(
                    input, contacts, surface_transition_required,
                    surface_transition_intent_valid);
                contact_count = 0;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    if (!contacts[leg] ||
                        !result.plan.current_support_anchor[leg].valid)
                        continue;
                    feet[leg] =
                        result.plan.current_support_anchor[leg].position_world;
                    ++contact_count;
                }
                if (contact_count < 2)
                    return false;
            }
            const auto &body = result.plan.body_reference[k];
            const double support_bound = TwoContactLineErrorBound(
                feet, contacts, config_, surface_transition_required,
                surface_transition_intent_valid);
            LogTwoContactSupportBound(
                "validation", result.plan.plan_id, k, contacts,
                surface_transition_required, surface_transition_intent_valid,
                support_bound, config_);
            const double margin = SupportMargin2D(
                feet, contacts, body.position, config_.min_support_margin_m,
                support_bound);
            result.plan.min_support_margin_m = std::min(
                result.plan.min_support_margin_m, margin);
            result.plan.min_uncertainty_inflated_support_margin_m = std::min(
                result.plan.min_uncertainty_inflated_support_margin_m,
                margin - result.plan.uncertainty_m);
            if (!std::isfinite(margin) || margin < config_.min_support_margin_m)
            {
                result.support_failure_knot = static_cast<int>(k);
                result.support_failure_contact_mask = 0;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                    if (contacts[leg])
                        result.support_failure_contact_mask |=
                            static_cast<std::uint8_t>(1u << leg);
                result.support_failure_margin_m = margin;
                return false;
            }
        }
        // A plan with no support knot is not a running plan; do not let the
        // aerial exception turn an invalid schedule into a valid snapshot.
        return support_knots > 0;
    }

    static int FirstTouchdownKnot(const TerrainPlannerInput &input,
                                  std::size_t leg)
    {
        bool previous = input.contact_schedule.measured_contact[leg];
        for (std::size_t k = 0; k < kTerrainPlanMaxKnots; ++k)
        {
            if (input.contact_schedule.planned_contact[k][leg] && !previous)
                return static_cast<int>(k);
            previous = input.contact_schedule.planned_contact[k][leg];
        }
        return -1;
    }
    int CandidateTouchdownKnot(const TerrainPlannerInput &input,
                               std::size_t leg) const
    {
        const int first = FirstTouchdownKnot(input, leg);
        if (first < 0)
            return -1;
        if (leg >= go2::kLegCount ||
            !input.next_touchdown_time_valid[leg] ||
            !std::isfinite(input.next_touchdown_time_s[leg]) ||
            !std::isfinite(input.gait_period_s) ||
            input.gait_period_s <= 0.0)
            return first;
        const double desired_touchdown_s =
            input.next_touchdown_time_s[leg];
        bool previous = input.contact_schedule.measured_contact[leg];
        for (std::size_t k = 0; k < kTerrainPlanMaxKnots; ++k)
        {
            const bool planned =
                input.contact_schedule.planned_contact[k][leg];
            if (planned && !previous)
            {
                const double knot_time = input.state_stamp_s +
                    static_cast<double>(k) * config_.knot_dt_s;
                if (knot_time + 0.5 * config_.knot_dt_s >=
                    desired_touchdown_s)
                    return static_cast<int>(k);
            }
            previous = planned;
        }
        return -1;
    }

    void SeedTouchdownSelections(const TerrainPlannerInput &input,
                                 TerrainPlannerResult &result) const
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            result.selected_by_touchdown_valid[leg].fill(false);
            const int touchdown = CandidateTouchdownKnot(input, leg);
            if (touchdown < 0 ||
                static_cast<std::size_t>(touchdown) >= config_.horizon_knots ||
                !result.selected[leg].hard_feasible)
                continue;
            result.selected_by_touchdown[leg][touchdown] =
                result.selected[leg];
            result.selected_by_touchdown_valid[leg][touchdown] = true;
        }
    }

    bool FutureBaseDisplacementAt(
        const TerrainPlannerInput &input, int first_touchdown,
        int touchdown, go2::Vec3 &displacement) const
    {
        if (touchdown <= 0 ||
            touchdown >= static_cast<int>(config_.horizon_knots) ||
            touchdown <= first_touchdown ||
            !std::isfinite(input.base_yaw_rad) ||
            !std::isfinite(input.base_velocity_world.x) ||
            !std::isfinite(input.base_velocity_world.y) ||
            !std::isfinite(input.base_velocity_world.z))
            return false;
        const double touchdown_dt_s =
            static_cast<double>(touchdown) * config_.knot_dt_s;
        if (!(touchdown_dt_s > 0.0) ||
            !std::isfinite(touchdown_dt_s))
            return false;
        displacement = RotateWorldVectorToBase(
            input.base_yaw_rad,
            {input.base_velocity_world.x * touchdown_dt_s,
             input.base_velocity_world.y * touchdown_dt_s,
             input.base_velocity_world.z * touchdown_dt_s});
        return std::isfinite(displacement.x) &&
            std::isfinite(displacement.y) &&
            std::isfinite(displacement.z);
    }
    void PopulateFutureTouchdownSelections(
        const TerrainPlannerInput &input,
        TerrainPlannerResult &result) const
    {
        constexpr std::size_t kMaxFutureSwingEvaluations = 32;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int first_touchdown = CandidateTouchdownKnot(input, leg);
            if (first_touchdown < 0 ||
                first_touchdown >= static_cast<int>(config_.horizon_knots) ||
                !result.selected_by_touchdown_valid[leg][
                    static_cast<std::size_t>(first_touchdown)])
                continue;
            FootholdCandidate previous_candidate =
                result.selected_by_touchdown[leg][
                    static_cast<std::size_t>(first_touchdown)];
            int previous_touchdown = first_touchdown;
            const auto nominal_touchdown =
                NominalTouchdownFoot(input, leg);
            const double observed_leg_surface_height_m =
                ObservedTerrainReferenceHeight(input, leg);
            const double observed_surface_height_m =
                std::isfinite(observed_leg_surface_height_m)
                    ? observed_leg_surface_height_m
                    : ObservedSupportSurfaceHeight(input);
            for (std::size_t k = static_cast<std::size_t>(first_touchdown + 1);
                 k < config_.horizon_knots; ++k)
            {
                if (!input.contact_schedule.planned_contact[k][leg] ||
                    input.contact_schedule.planned_contact[k - 1][leg])
                    continue;
                if (!previous_candidate.hard_feasible)
                    break;
                const go2::Vec3 swing_start =
                    previous_candidate.foot_position;
                const double elapsed_dt_s =
                    static_cast<double>(k - previous_touchdown) *
                    config_.knot_dt_s;
                const double command_direction =
                    input.commanded_vx_mps > 1.0e-3 ? 1.0
                    : (input.commanded_vx_mps < -1.0e-3 ? -1.0 : 0.0);
                const double expected_progress =
                    std::abs(input.commanded_vx_mps) * elapsed_dt_s;
                const auto bias_surface_position =
                    [&](go2::Vec3 position) {
                        if (command_direction <= 0.0 ||
                            !std::isfinite(observed_surface_height_m) ||
                            position.z <= observed_surface_height_m +
                                0.5 * config_.feasibility.foot_patch_radius_m)
                            return position;
                        const double edge_x =
                            go2_terrain::ForwardElevatedSurfaceEdgeX(
                                *input.terrain, position,
                                config_.feasibility.foot_patch_radius_m);
                        if (std::isfinite(edge_x))
                            position.x = std::max(
                                position.x,
                                edge_x + config_.feasibility.elevated_surface_edge_bias_m +
                                    config_.feasibility.elevated_surface_standoff_m);
                        return position;
                    };
                const auto region_score =
                    [&](const go2::Vec3 &position, double uncertainty,
                        double edge_margin) {
                        const double directional_progress = command_direction *
                            (position.x - swing_start.x);
                        const double progress_error = std::abs(
                            directional_progress - expected_progress);
                        const double lateral_error = std::abs(
                            position.y - nominal_touchdown.y);
                        const double elevation_gain = command_direction != 0.0
                            ? std::max(0.0, position.z - swing_start.z)
                            : 0.0;
                        const double uncertainty_term =
                            std::isfinite(uncertainty) ? uncertainty : 1.0;
                        return progress_error + 0.5 * lateral_error +
                            0.25 * uncertainty_term - 0.05 * edge_margin -
                            config_.upward_surface_preference_weight *
                                elevation_gain;
                    };
                std::vector<std::size_t> region_order;
                region_order.reserve(result.regions[leg].size());
                for (std::size_t region_index = 0;
                     region_index < result.regions[leg].size(); ++region_index)
                {
                    if (result.regions[leg][region_index].valid)
                        region_order.push_back(region_index);
                }
                std::sort(region_order.begin(), region_order.end(),
                          [&](std::size_t lhs, std::size_t rhs) {
                              const auto &lhs_region =
                                  result.regions[leg][lhs];
                              const auto &rhs_region =
                                  result.regions[leg][rhs];
                              const double lhs_score = region_score(
                                  lhs_region.center, lhs_region.uncertainty_m,
                                  lhs_region.edge_margin_m);
                              const double rhs_score = region_score(
                                  rhs_region.center, rhs_region.uncertainty_m,
                                  rhs_region.edge_margin_m);
                              if (lhs_score != rhs_score)
                                  return lhs_score < rhs_score;
                              return lhs_region.region_id < rhs_region.region_id;
                          });
                if (region_order.size() > kMaxFutureSwingEvaluations)
                    region_order.resize(kMaxFutureSwingEvaluations);
                go2::Vec3 future_displacement{};
                const bool future_displacement_valid =
                    FutureBaseDisplacementAt(
                        input, first_touchdown, static_cast<int>(k),
                        future_displacement);
                bool found = false;
                double best_score =
                    std::numeric_limits<double>::infinity();
                FootholdCandidate best_candidate{};
                for (const std::size_t region_index : region_order)
                {
                    const auto &region = result.regions[leg][region_index];
                    const go2::Vec3 desired_position{
                        swing_start.x + command_direction * expected_progress,
                        nominal_touchdown.y, region.center.z};
                    const go2::Vec3 representative = bias_surface_position(
                        RegionRepresentative(
                            region, desired_position, input.terrain->resolution_m));
                    FootholdCandidate candidate = EvaluateFoothold(
                        *input.terrain, static_cast<go2::Leg>(leg),
                        representative.x, representative.y,
                        config_.feasibility, &swing_start,
                        config_.swing_clearance_m,
                        future_displacement_valid
                            ? &future_displacement : nullptr);
                    if (!candidate.hard_feasible ||
                        (command_direction > 0.0 &&
                         !go2_terrain::HasForwardElevatedSurfaceStandoff(
                             *input.terrain, candidate.foot_position,
                             observed_surface_height_m,
                             config_.feasibility.elevated_surface_standoff_m,
                             config_.feasibility.foot_patch_radius_m)))
                        continue;
                    candidate.region_id = region.region_id;
                    candidate.support_margin_m = region.support_margin_m;
                    candidate.collision_margin_m =
                        candidate.swing_clearance_m;
                    // Future retimed touchdowns remain on the surface
                    // already committed by the preceding candidate.  If no
                    // intent exists, leave it false and fail closed to the
                    // ordinary drift band.
                    candidate.surface_transition_required =
                        previous_candidate.surface_transition_required;
                    candidate.surface_transition_intent_valid =
                        previous_candidate.surface_transition_intent_valid;
                    const double score = region_score(
                        candidate.foot_position, candidate.uncertainty_m,
                        candidate.edge_margin_m);
                    if (!found || score < best_score ||
                        (score == best_score &&
                         candidate.region_id < best_candidate.region_id))
                    {
                        found = true;
                        best_score = score;
                        best_candidate = candidate;
                    }
                }
                if (!found)
                    break;
                result.selected_by_touchdown[leg][k] = best_candidate;
                result.selected_by_touchdown_valid[leg][k] = true;
                previous_candidate = best_candidate;
                previous_touchdown = static_cast<int>(k);
            }
        }
    }

    // A terrain-conditioned swing may need more time than the nominal gait
    // flight interval.  Retiming only the execution trajectory would create
    // a new foothold with the old contact schedule, which is unsafe for both
    // SRBD-MPC and ID-WBC.  Delay the same per-leg contact event in the
    // atomic planner input, then repopulate the whole plan from that timeline.
    // The waveform prefix and the gait topology remain unchanged; only an
    // observed, feasibility-derived touchdown is delayed.
    bool BuildRetimedPlanInput(
        const TerrainPlannerInput &input,
        TerrainPlannerResult &result,
        TerrainPlannerInput &retimed_input,
        bool &changed) const
    {
        retimed_input = input;
        changed = false;
        const double nominal_swing_duration_s =
            std::isfinite(input.gait_period_s) &&
                    input.gait_period_s > 0.0 &&
                    std::isfinite(input.duty_factor) &&
                    input.duty_factor > 0.0 && input.duty_factor < 1.0
                ? (1.0 - input.duty_factor) * input.gait_period_s
                : config_.knot_dt_s;
        if (!std::isfinite(nominal_swing_duration_s) ||
            nominal_swing_duration_s <= 0.0 ||
            !std::isfinite(config_.knot_dt_s) ||
            config_.knot_dt_s <= 0.0)
            return true;

        std::array<int, go2::kLegCount> first_knot{};
        std::array<int, go2::kLegCount> delay_knots{};
        first_knot.fill(-1);
        delay_knots.fill(0);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int first = CandidateTouchdownKnot(input, leg);
            if (first < 0 ||
                first >= static_cast<int>(config_.horizon_knots) ||
                !result.selected_by_touchdown_valid[leg][
                    static_cast<std::size_t>(first)])
                continue;
            const auto &foot = result.plan.predicted_foothold[
                static_cast<std::size_t>(first)][leg];
            if (!foot.valid || !foot.touchdown ||
                !std::isfinite(foot.touchdown_time_s) ||
                !std::isfinite(foot.swing_duration_s) ||
                foot.swing_duration_s <= 0.0)
                continue;
            const double observed_leg_height_m =
                ObservedTerrainReferenceHeight(input, leg);
            const double observed_surface_world_z =
                std::isfinite(observed_leg_height_m) &&
                        std::isfinite(input.base_position_world.z)
                    ? input.base_position_world.z + observed_leg_height_m
                    : ObservedSupportSurfaceWorldHeight(input);
            const double terrain_deadband = std::max(
                2.0 * std::max(0.0, foot.uncertainty_m),
                0.5 * config_.feasibility.foot_patch_radius_m);
            // Only an observed height transition needs a terrain-retimed
            // touchdown. Flat-surface footholds retain the Phase-1 schedule,
            // even when their candidate path is longer than nominal.
            if (!std::isfinite(observed_surface_world_z) ||
                !std::isfinite(foot.position_world.z) ||
                foot.position_world.z - observed_surface_world_z <=
                    terrain_deadband)
                continue;
            const double nominal_swing_start_s = std::max(
                input.state_stamp_s,
                foot.touchdown_time_s - nominal_swing_duration_s);
            const double required_touchdown_s =
                nominal_swing_start_s + foot.swing_duration_s;
            if (!std::isfinite(nominal_swing_start_s) ||
                !std::isfinite(required_touchdown_s))
                return false;
            const double delay_s = std::max(
                0.0, required_touchdown_s - foot.touchdown_time_s);
            int delay = static_cast<int>(std::ceil(
                std::max(0.0, delay_s / config_.knot_dt_s - 1.0e-9)));
            if (delay <= 0)
                continue;
            // The terrain duration is known here, before publication and
            // execution rebase. Reserve one planner knot in the same atomic
            // stretch for handoff/rebase jitter; otherwise a later measured
            // swing start consumes the entire feasibility-derived delay.
            // This is timing structure, not a velocity reduction, and the
            // execution target remains frozen once the swing starts.
            ++delay;
            // A retimed touchdown that lands beyond the horizon is not a
            // plan-killing overflow: the insertion below truncates the
            // stretch at the horizon end and the selection shift drops the
            // event, so the near-term schedule still publishes and the
            // event is replanned once it slides into the window.
            first_knot[leg] = first;
            delay_knots[leg] = delay;
            changed = true;
        }
        // Retiming one member of a discrete touchdown event retimes every
        // member entering stance at that same event.  The group is derived
        // from the supplied schedule, never from leg identity: leaving one
        // co-event leg in the old knot can create a zero-margin triangular
        // support set while the other leg is still in swing.
        for (std::size_t trigger = 0; trigger < go2::kLegCount; ++trigger)
        {
            if (first_knot[trigger] < 0 || delay_knots[trigger] <= 0)
                continue;
            const int event = first_knot[trigger];
            const int delay = delay_knots[trigger];
            for (std::size_t member = 0; member < go2::kLegCount; ++member)
            {
                if (CandidateTouchdownKnot(input, member) != event)
                    continue;
                first_knot[member] = event;
                delay_knots[member] = std::max(delay_knots[member], delay);
            }
        }
        if (!changed)
            return true;

        // One terrain event stretches the whole timeline, not only the
        // triggered legs: the opposite diagonal pair must hold its stance
        // while the extended swing is in the air, and every later event
        // inherits the shift.  Insert `delay` copies of the pre-event
        // contact row at the event knot for every leg — swinging legs read
        // false (their swing continues), support legs read true (their
        // stance extends) — so no knot can drop below the pre-event
        // support count and gait, MPC, and WBC all read one identical
        // atomic schedule.
        std::vector<std::pair<int, int>> stretches;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (first_knot[leg] < 0 || delay_knots[leg] <= 0)
                continue;
            stretches.emplace_back(first_knot[leg], delay_knots[leg]);
        }
        std::sort(stretches.begin(), stretches.end());
        std::vector<std::pair<int, int>> merged;
        for (const auto &stretch : stretches)
        {
            if (!merged.empty() && merged.back().first == stretch.first)
                merged.back().second = std::max(
                    merged.back().second, stretch.second);
            else
                merged.push_back(stretch);
        }
        const auto shift_at = [&merged](int knot) {
            int shift = 0;
            for (const auto &stretch : merged)
                if (stretch.first <= knot)
                    shift += stretch.second;
            return shift;
        };

        const int horizon = static_cast<int>(config_.horizon_knots);
        const auto original_schedule = input.contact_schedule.planned_contact;
        auto &stretched = retimed_input.contact_schedule.planned_contact;
        int dst = 0;
        bool stretch_cut = false;
        std::size_t stretch_index = 0;
        for (int k = 0; k < horizon; ++k)
        {
            while (stretch_index < merged.size() &&
                   merged[stretch_index].first == k)
            {
                const int source = std::max(0, k - 1);
                for (int j = 0; j < merged[stretch_index].second; ++j)
                {
                    // Truncate at the horizon end instead of rejecting the
                    // plan: every written row is a copy of an original
                    // schedule row, so a partial stretch keeps the schedule
                    // internally consistent and only the far events that no
                    // longer fit are dropped (their shifted selections are
                    // dropped by the shift loop below as well).
                    if (dst >= horizon)
                    {
                        stretch_cut = true;
                        break;
                    }
                    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                        stretched[static_cast<std::size_t>(dst)][leg] =
                            original_schedule[
                                static_cast<std::size_t>(source)][leg];
                    ++dst;
                }
                ++stretch_index;
            }
            if (dst >= horizon)
                break;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                stretched[static_cast<std::size_t>(dst)][leg] =
                    original_schedule[static_cast<std::size_t>(k)][leg];
            ++dst;
        }

        // Graceful truncation is only safe while the measured state is
        // independently supported.  A plan whose recovery touchdowns fell
        // off the horizon must not be published over a one-contact or
        // airborne measured state; fail closed there, as the overflow
        // rejection did before.
        if (stretch_cut && input.contact_schedule.measured_valid)
        {
            std::size_t measured_contacts = 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                if (input.contact_schedule.measured_contact[leg])
                    ++measured_contacts;
            if (measured_contacts < 2)
                return false;
        }

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            auto old_selected = result.selected_by_touchdown[leg];
            auto old_selected_valid =
                result.selected_by_touchdown_valid[leg];
            result.selected_by_touchdown[leg].fill(FootholdCandidate{});
            result.selected_by_touchdown_valid[leg].fill(false);
            for (std::size_t event = 0; event < config_.horizon_knots;
                 ++event)
            {
                if (!old_selected_valid[event])
                    continue;
                const int shift = shift_at(static_cast<int>(event));
                const int shifted = static_cast<int>(event) + shift;
                if (shifted >= horizon)
                    continue;
                FootholdCandidate shifted_candidate = old_selected[event];
                // The foothold was selected for the pre-stretch event time:
                // its world anchor is fixed at the planning base pose while
                // the body keeps travelling.  At the shifted touchdown the
                // base has advanced by velocity*shift*dt past that anchor,
                // which would silently shrink the post-touchdown support
                // margin.  Carry the foothold forward with the stretch so
                // the base-relative landing geometry is the one that was
                // validated.
                if (shift > 0)
                {
                    const go2::Vec3 travel_base = RotateWorldVectorToBase(
                        input.base_yaw_rad, input.base_velocity_world);
                    const double travel_s =
                        static_cast<double>(shift) * config_.knot_dt_s;
                    shifted_candidate.foot_position.x +=
                        travel_base.x * travel_s;
                    shifted_candidate.foot_position.y +=
                        travel_base.y * travel_s;
                }
                result.selected_by_touchdown[leg][
                    static_cast<std::size_t>(shifted)] = shifted_candidate;
                result.selected_by_touchdown_valid[leg][
                    static_cast<std::size_t>(shifted)] = true;
            }
            if (result.touchdown_knot_by_leg[leg] >= 0)
                result.touchdown_knot_by_leg[leg] += shift_at(
                    result.touchdown_knot_by_leg[leg]);
            // Re-derive the next touchdown from the stretched schedule so
            // consumers never see an event time that disagrees with the
            // contact timeline they execute.
            bool previous =
                input.contact_schedule.measured_contact[leg];
            for (int k = 0; k < horizon; ++k)
            {
                const bool planned =
                    stretched[static_cast<std::size_t>(k)][leg];
                if (planned && !previous)
                {
                    retimed_input.next_touchdown_time_s[leg] =
                        input.state_stamp_s +
                        static_cast<double>(k) * config_.knot_dt_s;
                    retimed_input.next_touchdown_time_valid[leg] = true;
                    break;
                }
                previous = planned;
            }
        }
        return true;
    }

    void ExtendValidityThroughTouchdowns(TerrainMotionPlan &plan) const
    {
        // `valid_until_s` is the lifetime of the atomic execution contract,
        // not merely the freshness window of the planner snapshot.  A
        // terrain swing may be retimed beyond the nominal 150 ms replanning
        // period; expiring the plan before its committed touchdown makes the
        // gait consumer silently discard the foothold while MPC/WBC still see
        // the old schedule.  Keep the plan alive through every committed
        // touchdown, with one knot of handoff margin.
        double execution_until_s = plan.valid_until_s;
        for (std::size_t k = 0; k < plan.horizon_knots; ++k)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto &foot = plan.predicted_foothold[k][leg];
                if (!foot.valid || !foot.touchdown ||
                    !std::isfinite(foot.touchdown_time_s))
                    continue;
                execution_until_s = std::max(
                    execution_until_s,
                    foot.touchdown_time_s + config_.knot_dt_s);
            }
        }
        if (std::isfinite(execution_until_s))
            plan.valid_until_s = execution_until_s;
    }

    bool ExtendExecutionSupportTail(TerrainMotionPlan &plan) const
    {
        if (plan.horizon_knots == 0 ||
            plan.horizon_knots > kTerrainPlanMaxKnots)
            return false;
        if (plan.horizon_knots == kTerrainPlanMaxKnots)
            return true;

        // The optimizer proves the finite touchdown horizon. Consumers also
        // need a bounded support set while that transaction remains pinned.
        // Repeat the last already-proven support knot without enlarging the
        // combinatorial search or inventing a new contact sequence.
        std::size_t anchor = plan.horizon_knots;
        for (std::size_t k = plan.horizon_knots; k-- > 0;)
        {
            std::size_t contacts = 0;
            bool complete = true;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!plan.contact_schedule.planned_contact[k][leg])
                    continue;
                ++contacts;
                complete = complete &&
                    plan.predicted_foothold[k][leg].valid;
            }
            if (contacts >= 2 && complete)
            {
                anchor = k;
                break;
            }
        }
        if (anchor >= plan.horizon_knots)
            return false;

        for (std::size_t k = plan.horizon_knots;
             k < kTerrainPlanMaxKnots; ++k)
        {
            plan.contact_schedule.planned_contact[k] =
                plan.contact_schedule.planned_contact[anchor];
            plan.body_reference[k] = plan.body_reference[anchor];
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!plan.contact_schedule.planned_contact[k][leg])
                {
                    plan.predicted_foothold[k][leg] = {};
                    continue;
                }
                plan.predicted_foothold[k][leg] =
                    plan.predicted_foothold[anchor][leg];
                plan.predicted_foothold[k][leg].touchdown = false;
                plan.predicted_foothold[k][leg].touchdown_time_s =
                    plan.state_stamp_s +
                    static_cast<double>(k) * config_.knot_dt_s;
            }
        }
        plan.horizon_knots = kTerrainPlanMaxKnots;
        plan.valid_until_s = std::max(
            plan.valid_until_s,
            plan.state_stamp_s +
                static_cast<double>(kTerrainPlanMaxKnots - 1) *
                    config_.knot_dt_s);
        return true;
    }

    void PopulatePlan(const TerrainPlannerInput &input,
                      TerrainPlannerResult &result) const
    {
        result.plan.min_edge_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.min_uncertainty_inflated_edge_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.min_slope_rad = 0.0;
        result.plan.max_roughness_m = 0.0;
        result.plan.min_reachability_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.min_swing_clearance_m =
            std::numeric_limits<double>::infinity();
        result.plan.committed_touchdowns = 0;
        result.plan.current_support_count = 0;
        result.plan.current_support_surface_height_world.fill(
            std::numeric_limits<double>::quiet_NaN());
        result.plan.current_support_surface_valid.fill(false);
        result.plan.current_terrain_height_world.fill(
            std::numeric_limits<double>::quiet_NaN());
        result.plan.current_terrain_height_valid.fill(false);
        result.plan.swing_start_position_world.fill(go2::Vec3{});
        result.plan.swing_start_position_valid.fill(false);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &swing_start = input.current_feet_base[leg];
            if (std::isfinite(input.base_position_world.x) &&
                std::isfinite(input.base_position_world.y) &&
                std::isfinite(input.base_position_world.z) &&
                std::isfinite(swing_start.x) &&
                std::isfinite(swing_start.y) &&
                std::isfinite(swing_start.z))
            {
                result.plan.swing_start_position_world[leg] =
                    RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        swing_start);
                result.plan.swing_start_position_valid[leg] = true;
            }
            const double terrain_height_m =
                ObservedTerrainHeightAt(input, leg);
            if (!std::isfinite(terrain_height_m) ||
                !std::isfinite(input.base_position_world.z))
                continue;
            result.plan.current_terrain_height_world[leg] =
                input.base_position_world.z + terrain_height_m;
            result.plan.current_terrain_height_valid[leg] = true;
        }
        result.plan.contact_schedule.measured_contact =
            input.contact_schedule.measured_contact;
        result.plan.contact_schedule.measured_valid =
            input.contact_schedule.measured_valid;
        result.plan.contact_schedule.planned_valid =
            input.contact_schedule.planned_valid;
        // Current support anchors are populated only from measured contact.
        // A planned contact remains a prediction until the contact filter
        // confirms loading; it must not become a measured anchor merely
        // because it appears in the schedule.
        if (input.contact_schedule.measured_valid)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!input.contact_schedule.measured_contact[leg])
                    continue;
                TerrainFootholdPrediction anchor;
                anchor.valid = true;
                anchor.position_world = RotateBaseToWorld(
                    input.base_position_world, input.base_yaw_rad,
                    input.current_feet_base[leg]);
                anchor.touchdown_time_s = input.state_stamp_s;
                anchor.surface_normal = {0.0, 0.0, 1.0};
                result.plan.current_support_anchor[leg] = anchor;
                ++result.plan.current_support_count;
                if (result.plan.current_terrain_height_valid[leg])
                {
                    result.plan.current_support_surface_height_world[leg] =
                        result.plan.current_terrain_height_world[leg];
                    result.plan.current_support_surface_valid[leg] = true;
                }
            }
        }
        for (std::size_t k = 0; k < config_.horizon_knots; ++k)
        {
            result.plan.contact_schedule.planned_contact[k] =
                input.contact_schedule.planned_contact[k];
            result.plan.body_reference[k].position =
                PredictBasePosition(input.base_position_world,
                                    input.base_velocity_world,
                                    static_cast<double>(k) * config_.knot_dt_s);
            result.plan.body_reference[k].linear_velocity =
                input.base_velocity_world;
            result.plan.body_reference[k].linear_acceleration =
                input.base_acceleration_world;
            result.plan.body_reference[k].roll_rad = input.base_roll_rad;
            result.plan.body_reference[k].pitch_rad = input.base_pitch_rad;
            result.plan.body_reference[k].yaw_rad = input.base_yaw_rad;
            result.plan.body_reference[k].yaw_rate_radps = 0.0;
            result.plan.body_reference[k].height_m = input.base_height_m;
            result.plan.body_reference[k].valid = true;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const int first_touchdown =
                    CandidateTouchdownKnot(input, leg);
                int active_touchdown = -1;
                for (std::size_t event = 0; event <= k; ++event)
                {
                    if (result.selected_by_touchdown_valid[leg][event])
                        active_touchdown = static_cast<int>(event);
                }
                TerrainFootholdPrediction foot;
                if (active_touchdown >= 0)
                {
                    const auto &candidate =
                        result.selected_by_touchdown[leg][
                            static_cast<std::size_t>(active_touchdown)];
                    foot.valid = candidate.hard_feasible;
                    foot.touchdown =
                        static_cast<int>(k) == active_touchdown;
                    const bool exact_touchdown_time =
                        active_touchdown == first_touchdown &&
                        input.next_touchdown_time_valid[leg] &&
                        std::isfinite(input.next_touchdown_time_s[leg]);
                    foot.touchdown_time_s = exact_touchdown_time
                        ? input.next_touchdown_time_s[leg]
                        : input.state_stamp_s +
                              active_touchdown * config_.knot_dt_s;
                    foot.touchdown_phase = input.gait_phase;
                    if (std::isfinite(input.gait_phase) &&
                        std::isfinite(input.gait_period_s) &&
                        input.gait_period_s > 0.0)
                    {
                        foot.touchdown_phase = std::fmod(
                            input.gait_phase +
                                static_cast<double>(active_touchdown) *
                                    config_.knot_dt_s /
                                    input.gait_period_s,
                            1.0);
                        if (foot.touchdown_phase < 0.0)
                            foot.touchdown_phase += 1.0;
                    }
                    foot.position_world = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        candidate.foot_position);
                    foot.surface_normal = candidate.surface_normal;
                    foot.region_id = candidate.region_id;
                    foot.edge_margin_m = candidate.edge_margin_m;
                    foot.reachability_margin_m =
                        candidate.reachability_margin_m;
                    foot.swing_clearance_m = candidate.swing_clearance_m;
                    foot.swing_lift_m = candidate.swing_lift_m;
                    foot.swing_peak_phase = candidate.swing_peak_phase;
                    foot.swing_leading_edge_phase =
                        candidate.swing_leading_edge_phase;
                    foot.swing_leading_edge_phase_valid =
                        candidate.swing_leading_edge_phase_valid;
                    foot.support_margin_m = candidate.support_margin_m;
                    foot.collision_margin_m = candidate.collision_margin_m;
                    foot.uncertainty_m = candidate.uncertainty_m;
                    foot.surface_transition_required =
                        candidate.surface_transition_required;
                    foot.surface_transition_intent_valid =
                        candidate.surface_transition_intent_valid;
                    // This intent is deliberately not recomputed from
                    // candidate/observed z here: those values can be blended
                    // at a riser edge and are not a stable surface identity.
                    result.plan.uncertainty_m = std::max(
                        result.plan.uncertainty_m, candidate.uncertainty_m);
                    if (static_cast<int>(k) == active_touchdown)
                    {
                        ++result.plan.committed_touchdowns;
                        if (active_touchdown == first_touchdown)
                        {
                            foot.swing_start_position_world =
                                result.plan.swing_start_position_world[leg];
                            foot.swing_start_position_valid =
                                result.plan.swing_start_position_valid[leg];
                        }
                        else
                        {
                            for (int event = active_touchdown - 1;
                                 event >= 0; --event)
                            {
                                if (!result.selected_by_touchdown_valid[leg][
                                        static_cast<std::size_t>(event)])
                                    continue;
                                foot.swing_start_position_world =
                                    RotateBaseToWorld(
                                        input.base_position_world,
                                        input.base_yaw_rad,
                                        result.selected_by_touchdown[leg][
                                            static_cast<std::size_t>(event)]
                                                .foot_position);
                                foot.swing_start_position_valid =
                                    std::isfinite(
                                        foot.swing_start_position_world.x) &&
                                    std::isfinite(
                                        foot.swing_start_position_world.y) &&
                                    std::isfinite(
                                        foot.swing_start_position_world.z);
                                break;
                            }
                        }
                        const double nominal_swing_duration_s =
                            std::isfinite(input.gait_period_s) &&
                                    input.gait_period_s > 0.0 &&
                                    std::isfinite(input.duty_factor) &&
                                    input.duty_factor > 0.0 &&
                                    input.duty_factor < 1.0
                                ? (1.0 - input.duty_factor) *
                                      input.gait_period_s
                                : config_.knot_dt_s;
                        foot.swing_duration_s =
                            TerrainSwingDurationForPath(
                                nominal_swing_duration_s,
                                foot.swing_start_position_world,
                                go2::ContactPatchToFootSite(
                                    foot.position_world),
                                foot.swing_lift_m,
                                config_.feasibility.max_swing_speed_mps);
                        result.plan.min_edge_margin_m = std::min(
                            result.plan.min_edge_margin_m,
                            candidate.edge_margin_m);
                        result.plan.min_uncertainty_inflated_edge_margin_m =
                            std::min(result.plan.min_uncertainty_inflated_edge_margin_m,
                                     candidate.edge_margin_m - candidate.uncertainty_m);
                        result.plan.min_slope_rad = std::max(
                            result.plan.min_slope_rad, candidate.slope_rad);
                        result.plan.max_roughness_m = std::max(
                            result.plan.max_roughness_m, candidate.roughness_m);
                        result.plan.min_reachability_margin_m = std::min(
                            result.plan.min_reachability_margin_m,
                            candidate.reachability_margin_m);
                        result.plan.min_swing_clearance_m = std::min(
                            result.plan.min_swing_clearance_m,
                            candidate.swing_clearance_m);
                    }
                }
                else if (input.contact_schedule.planned_contact[k][leg])
                {
                    foot.valid = true;
                    const auto nominal_touchdown =
                        NominalTouchdownFoot(input, leg);
                    const bool after_first_touchdown =
                        first_touchdown >= 0 &&
                        static_cast<int>(k) >= first_touchdown &&
                        std::isfinite(nominal_touchdown.x) &&
                        std::isfinite(nominal_touchdown.y) &&
                        std::isfinite(nominal_touchdown.z);
                    foot.position_world = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        after_first_touchdown
                            ? nominal_touchdown
                            : input.current_feet_base[leg]);
                    foot.touchdown_time_s =
                        after_first_touchdown &&
                                input.next_touchdown_time_valid[leg] &&
                                std::isfinite(
                                    input.next_touchdown_time_s[leg])
                            ? input.next_touchdown_time_s[leg]
                            : input.state_stamp_s;
                    foot.surface_normal = {0.0, 0.0, 1.0};
                }
                result.plan.predicted_foothold[k][leg] = foot;
            }
        }

        // Raise the CoM only behind a support surface that is both planned
        // and confirmed. An unconfirmed elevated target must not lift the
        // old support feet before its measured touchdown; average only
        // across the surfaces that are already confirmed.
        const double current_surface_world_z =
            ObservedSupportSurfaceWorldHeight(input);
        if (std::isfinite(current_surface_world_z))
        {
            std::array<double, kTerrainPlanMaxKnots> future_surface_world_z{};
            double held_surface_world_z = current_surface_world_z;
            for (std::size_t k = 0; k < config_.horizon_knots; ++k)
            {
                double surface_sum = 0.0;
                std::size_t surface_count = 0;
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    if (!result.plan.contact_schedule.planned_contact[k][leg])
                        continue;
                    const auto &foot = result.plan.predicted_foothold[k][leg];
                    if (!foot.valid || !std::isfinite(foot.position_world.z))
                        continue;
                    const bool transition_confirmed =
                        input.terrain_surface_transition_active &&
                        input.terrain_surface_transition_required[leg] &&
                        input.terrain_surface_transition_committed[leg];
                    if (foot.surface_transition_required &&
                        !transition_confirmed)
                        continue;
                    surface_sum += foot.position_world.z;
                    ++surface_count;
                }
                if (surface_count > 0)
                    held_surface_world_z =
                        surface_sum / static_cast<double>(surface_count);
                future_surface_world_z[k] = held_surface_world_z;
            }

            constexpr double kBodySurfaceTransitionS = 0.20;
            std::array<double, kTerrainPlanMaxKnots> body_reference_z{};
            for (std::size_t k = 0; k < config_.horizon_knots; ++k)
            {
                const double t_s =
                    static_cast<double>(k) * config_.knot_dt_s;
                const double transition = TerrainSwingEase(
                    t_s / kBodySurfaceTransitionS);
                const double surface_delta =
                    future_surface_world_z[k] - current_surface_world_z;
                body_reference_z[k] =
                    input.base_position_world.z +
                    transition * surface_delta;
                result.plan.body_reference[k].position.z =
                    body_reference_z[k];
                result.plan.body_reference[k].height_m =
                    input.base_height_m + transition * surface_delta;
            }
            double previous_vertical_velocity =
                input.base_velocity_world.z;
            for (std::size_t k = 0; k < config_.horizon_knots; ++k)
            {
                double vertical_velocity = input.base_velocity_world.z;
                if (k > 0 && config_.knot_dt_s > 1.0e-6)
                {
                    vertical_velocity =
                        (body_reference_z[k] -
                         body_reference_z[k - 1]) /
                        config_.knot_dt_s;
                }
                result.plan.body_reference[k].linear_velocity.z =
                    vertical_velocity;
                result.plan.body_reference[k].linear_acceleration.z =
                    k > 0 && config_.knot_dt_s > 1.0e-6
                        ? (vertical_velocity - previous_vertical_velocity) /
                              config_.knot_dt_s
                        : input.base_acceleration_world.z;
                previous_vertical_velocity = vertical_velocity;
            }
        }

        result.plan.velocity_request.valid = false;
        if (!std::isfinite(result.plan.min_edge_margin_m))
            result.plan.min_edge_margin_m = 0.0;
        if (!std::isfinite(result.plan.min_uncertainty_inflated_edge_margin_m))
            result.plan.min_uncertainty_inflated_edge_margin_m = 0.0;
        if (!std::isfinite(result.plan.min_reachability_margin_m))
            result.plan.min_reachability_margin_m = 0.0;
        if (!std::isfinite(result.plan.min_swing_clearance_m))
            result.plan.min_swing_clearance_m = 0.0;
        // Keep the conservative uncertainty bound for support diagnostics.
    }

    TerrainPlannerResult Finish(const TerrainPlannerInput &input,
                                TerrainPlannerResult result,
                                std::chrono::steady_clock::time_point start) const
    {
        (void)input;
        result.plan.solver.elapsed_us = std::chrono::duration<double,
            std::micro>(std::chrono::steady_clock::now() - start).count();
        result.plan.solver.deadline_miss =
            result.plan.solver.elapsed_us > config_.deadline_us;
        if (result.plan.solver.deadline_miss &&
            result.plan.status != TerrainPlanStatus::kRejected)
        {
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.failure = TerrainPlanFailure::kDeadlineMiss;
            result.plan.solver.failure = TerrainPlanFailure::kDeadlineMiss;
            result.publishable = false;
        }
        return result;
    }

    TerrainPlannerConfig config_;
};

} // namespace go2_terrain
