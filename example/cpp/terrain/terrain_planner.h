#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "terrain_feasibility.h"
#include "terrain_motion_plan.h"

namespace go2_terrain
{

struct TerrainPlannerConfig
{
    TerrainFeasibilityConfig feasibility{};
    std::size_t horizon_knots = 8;
    // Match the Phase 1 running-trot MPC sample time at the default period.
    // The planner remains configurable, but must not invent a second timing
    // base for the contact/foothold horizon.
    double knot_dt_s = 0.020;
    double plan_validity_s = 0.15;
    double deadline_us = 5000.0;
    double min_support_margin_m = 0.015;
    double max_two_contact_line_error_m = 0.040;
    double candidate_x_span_m = 0.090;
    double candidate_y_span_m = 0.070;
    double candidate_spacing_m = 0.030;
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
    // Optional dynamics-model COM used by the consistency path.  The legacy
    // base position remains present so missing H5 fields are not fabricated.
    go2::Vec3 model_com_world{};
    bool model_com_valid = false;
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
    std::array<go2::Vec3, go2::kLegCount> measured_support_anchor_world{};
    std::array<bool, go2::kLegCount> measured_support_anchor_valid{};
    std::array<go2::Vec3, go2::kLegCount> nominal_feet_base{};
    std::array<go2::Vec3, go2::kLegCount> touchdown_target_feet_base{};
    bool touchdown_target_feet_valid = false;
    TerrainContactSchedule contact_schedule{};
};

struct TerrainPlannerResult
{
    TerrainMotionPlan plan{};
    std::array<std::vector<SafeFootholdRegion>, go2::kLegCount> regions{};
    std::array<FootholdCandidate, go2::kLegCount> selected{};
    std::array<std::size_t, go2::kLegCount> candidate_counts{};
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
        result.plan.plan_id = plan_id;
        result.plan.plan_epoch = plan_id;
        result.plan.map_epoch = input.terrain != nullptr
            ? input.terrain->epoch : 0;
        result.plan.state_stamp_s = input.state_stamp_s;
        result.plan.generated_at_s = input.state_stamp_s;
        result.plan.valid_until_s = input.state_stamp_s +
            config_.plan_validity_s;
        result.plan.frame_id = input.terrain != nullptr
            ? input.terrain->frame_id : "";
        result.plan.horizon_knots = config_.horizon_knots;
        result.plan.knot_dt_s = config_.knot_dt_s;
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

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            result.regions[leg] = BuildSafeFootholdRegions(
                *input.terrain, static_cast<go2::Leg>(leg),
                config_.feasibility);
            result.candidate_counts[leg] = result.regions[leg].size();
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
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const bool has_touchdown = FirstTouchdownKnot(input, leg) >= 0;
            if (!has_touchdown)
                continue;
            const bool debug_rejects =
                std::getenv("TROT_TERRAIN_DEBUG_PLANNER") != nullptr;
            std::array<std::size_t, kFootholdRejectReasonCount>
                reject_counts{};
            std::size_t valid_regions = 0;
            std::size_t hard_feasible = 0;
            FootholdCandidate best;
            double best_score = std::numeric_limits<double>::infinity();
            for (const auto &region : result.regions[leg])
            {
                if (!region.valid)
                    continue;
                ++valid_regions;
                FootholdCandidate candidate = EvaluateFoothold(
                    *input.terrain, static_cast<go2::Leg>(leg),
                    region.center.x, region.center.y, config_.feasibility,
                    &input.current_feet_base[leg],
                    config_.swing_clearance_m, nullptr,
                    input.contact_schedule.measured_contact[leg]);
                if (!candidate.hard_feasible)
                {
                    const auto reason = static_cast<std::size_t>(
                        candidate.reject_reason);
                    if (reason < reject_counts.size())
                        ++reject_counts[reason];
                    continue;
                }
                ++hard_feasible;
                candidate.region_id = region.region_id;
                const auto &phase1_target =
                    input.touchdown_target_feet_valid
                        ? input.touchdown_target_feet_base[leg]
                        : input.nominal_feet_base[leg];
                const double displacement = std::hypot(
                    candidate.foot_position.x -
                        phase1_target.x,
                    candidate.foot_position.y -
                        phase1_target.y);
                const double score = displacement +
                    0.5 * candidate.uncertainty_m -
                    0.1 * candidate.edge_margin_m;
                if (score < best_score)
                {
                    best = candidate;
                    best_score = score;
                }
            }
            if (debug_rejects)
            {
                std::fprintf(
                    stderr,
                    "Terrain planner leg=%zu regions=%zu valid=%zu evaluated=%zu feasible=%zu",
                    leg, result.regions[leg].size(), valid_regions,
                    valid_regions, hard_feasible);
                for (std::size_t reason = 1; reason < reject_counts.size();
                     ++reason)
                {
                    if (reject_counts[reason] == 0)
                        continue;
                    std::fprintf(
                        stderr, " %s=%zu",
                        FootholdRejectReasonName(
                            static_cast<FootholdRejectReason>(reason)),
                        reject_counts[reason]);
                }
                if (best.hard_feasible)
                {
                    std::fprintf(
                        stderr,
                        " selected=(%.6f,%.6f,%.6f) score=%.6f\n",
                        best.foot_position.x, best.foot_position.y,
                        best.foot_position.z, best_score);
                }
                else
                {
                    std::fprintf(stderr, " selected=none\n");
                }
            }
            if (!best.hard_feasible)
            {
                result.plan.failure = TerrainPlanFailure::kNoSafeFoothold;
                result.plan.status = TerrainPlanStatus::kRejected;
                result.plan.solver.failure = result.plan.failure;
                return Finish(input, std::move(result), start);
            }
            result.selected[leg] = best;
        }

        PopulatePlan(input, result);
        if (!SupportFeasible(result))
        {
            result.plan.status = TerrainPlanStatus::kRejected;
            result.plan.failure = TerrainPlanFailure::kSupportInfeasible;
            result.plan.solver.failure = result.plan.failure;
            return Finish(input, std::move(result), start);
        }
        result.plan.status = TerrainPlanStatus::kValid;
        result.plan.fallback_to_phase1 = false;
        result.plan.solver.success = true;
        result.publishable = result.plan.valid();
        return Finish(input, std::move(result), start);
    }

private:
    bool SupportFeasible(TerrainPlannerResult &result) const
    {
        result.plan.min_support_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.min_uncertainty_inflated_support_margin_m =
            std::numeric_limits<double>::infinity();
        result.plan.support_failure_knot = -1;
        result.plan.support_failure_contact_mask = 0;
        result.plan.support_failure_margin_m = 0.0;
        for (std::size_t k = 0; k < result.plan.horizon_knots; ++k)
        {
            std::array<go2::Vec3, go2::kLegCount> feet{};
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!result.plan.contact_schedule.planned_contact[k][leg] ||
                    !result.plan.predicted_foothold[k][leg].valid)
                    continue;
                feet[leg] = result.plan.predicted_foothold[k][leg].position_world;
            }
            const auto &body = result.plan.body_reference[k];
            std::array<bool, go2::kLegCount> contacts =
                result.plan.contact_schedule.planned_contact[k];
            const std::size_t contact_count = std::count(
                contacts.begin(), contacts.end(), true);
            int contact_mask = 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                if (contacts[leg])
                    contact_mask |= 1 << static_cast<int>(leg);
            // Running-trot legitimately contains an aerial knot at low duty.
            // It has no support polygon to evaluate; single-contact knots
            // remain infeasible and are still rejected below.
            if (contact_count == 0)
                continue;
            const double margin = SupportMargin2D(
                feet, contacts, body.model_com_valid
                    ? body.model_com_world : body.position,
                config_.min_support_margin_m,
                config_.max_two_contact_line_error_m);
            result.plan.min_support_margin_m = std::min(
                result.plan.min_support_margin_m, margin);
            result.plan.min_uncertainty_inflated_support_margin_m = std::min(
                result.plan.min_uncertainty_inflated_support_margin_m,
                margin - result.plan.uncertainty_m);
            if (!std::isfinite(margin) || margin < config_.min_support_margin_m)
            {
                result.plan.support_failure_knot = static_cast<int>(k);
                result.plan.support_failure_contact_mask = contact_mask;
                result.plan.support_failure_margin_m = margin;
                return false;
            }
        }
        return true;
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
        result.plan.contact_schedule.measured_contact =
            input.contact_schedule.measured_contact;
        result.plan.contact_schedule.measured_valid =
            input.contact_schedule.measured_valid;
        result.plan.contact_schedule.planned_valid =
            input.contact_schedule.planned_valid;
        for (std::size_t k = 0; k < config_.horizon_knots; ++k)
        {
            result.plan.contact_schedule.planned_contact[k] =
                input.contact_schedule.planned_contact[k];
            result.plan.body_reference[k].position = input.base_position_world;
            result.plan.body_reference[k].model_com_world = input.model_com_world;
            result.plan.body_reference[k].model_com_valid = input.model_com_valid;
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
                const int touchdown = FirstTouchdownKnot(input, leg);
                TerrainFootholdPrediction foot;
                if (touchdown >= 0 && static_cast<int>(k) >= touchdown)
                {
                    const auto &candidate = result.selected[leg];
                    foot.valid = candidate.hard_feasible;
                    foot.touchdown = static_cast<int>(k) == touchdown;
                    foot.touchdown_time_s = input.state_stamp_s +
                        touchdown * config_.knot_dt_s;
                    foot.touchdown_phase = input.gait_phase;
                    foot.position_world = RotateBaseToWorld(
                        input.base_position_world, input.base_yaw_rad,
                        candidate.foot_position);
                    foot.surface_normal = candidate.surface_normal;
                    foot.region_id = candidate.region_id;
                    foot.edge_margin_m = candidate.edge_margin_m;
                    foot.reachability_margin_m =
                        candidate.reachability_margin_m;
                    foot.swing_clearance_m = candidate.swing_clearance_m;
                    foot.support_margin_m = candidate.support_margin_m;
                    foot.collision_margin_m = candidate.collision_margin_m;
                    foot.uncertainty_m = candidate.uncertainty_m;
                    foot.swing_lift_m = candidate.swing_lift_m;
                    result.plan.uncertainty_m = std::max(
                        result.plan.uncertainty_m, candidate.uncertainty_m);
                    if (static_cast<int>(k) == touchdown)
                    {
                        ++result.plan.committed_touchdowns;
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
                    const bool use_measured_support_anchor =
                        input.contact_schedule.measured_valid &&
                        input.contact_schedule.measured_contact[leg] &&
                        input.measured_support_anchor_valid[leg];
                    foot.position_world = use_measured_support_anchor
                        ? input.measured_support_anchor_world[leg]
                        : RotateBaseToWorld(input.base_position_world,
                                            input.base_yaw_rad,
                                            input.current_feet_base[leg]);
                    foot.touchdown_time_s = input.state_stamp_s;
                    foot.surface_normal = {0.0, 0.0, 1.0};
                    if (!result.plan.current_support_anchor[leg].valid)
                    {
                        result.plan.current_support_anchor[leg] = foot;
                        ++result.plan.current_support_count;
                    }
                }
                result.plan.predicted_foothold[k][leg] = foot;
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
