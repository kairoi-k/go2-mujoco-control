#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "go2_inverse_kinematics.h"
#include "terrain_model.h"

namespace go2_terrain
{

enum class FootholdRejectReason : std::uint8_t
{
    kNone = 0,
    kInvalidModel,
    kFrameMismatch,
    kStale,
    kUnknown,
    kSlope,
    kRoughness,
    kUncertainty,
    kEdge,
    kSurfaceStep,
    kReachability,
    kSwingClearance,
    kCollision,
    kSupport,
};

constexpr std::size_t kFootholdRejectReasonCount =
    static_cast<std::size_t>(FootholdRejectReason::kSupport) + 1;

inline const char *FootholdRejectReasonName(FootholdRejectReason reason)
{
    switch (reason)
    {
    case FootholdRejectReason::kFrameMismatch: return "frame_mismatch";
    case FootholdRejectReason::kStale: return "stale";
    case FootholdRejectReason::kUnknown: return "unknown";
    case FootholdRejectReason::kSlope: return "slope";
    case FootholdRejectReason::kRoughness: return "roughness";
    case FootholdRejectReason::kUncertainty: return "uncertainty";
    case FootholdRejectReason::kEdge: return "edge";
    case FootholdRejectReason::kSurfaceStep: return "surface_step";
    case FootholdRejectReason::kReachability: return "reachability";
    case FootholdRejectReason::kSwingClearance: return "swing_clearance";
    case FootholdRejectReason::kCollision: return "collision";
    case FootholdRejectReason::kSupport: return "support";
    case FootholdRejectReason::kInvalidModel: return "invalid_model";
    default: return "none";
    }
}

struct TerrainFeasibilityConfig
{
    std::string required_frame = "base_link";
    double max_map_age_s = 0.20;
    double max_cell_age_s = 0.25;
    double min_known_fraction = 1.0;
    double foot_patch_radius_m = 0.025;
    double min_edge_margin_m = 0.040;
    double max_slope_rad = 20.0 * 3.14159265358979323846 / 180.0;
    double max_roughness_m = 0.025;
    double max_variance_m2 = 0.000625;
    double max_surface_step_m = 0.040;
    double min_reachability_margin_m = 0.010;
    double min_swing_clearance_m = 0.030;
    double region_half_extent_m = 0.035;
};

struct SafeFootholdRegion
{
    static constexpr std::size_t kMaxVertices = 4;
    go2::Leg leg = go2::Leg::FR;
    std::uint64_t map_epoch = 0;
    double support_margin_m = std::numeric_limits<double>::infinity();
    double collision_margin_m = std::numeric_limits<double>::infinity();
    std::uint32_t region_id = 0;
    std::array<go2::Vec3, kMaxVertices> vertices{};
    std::size_t vertex_count = 0;
    go2::Vec3 center{};
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    double height_min_m = std::numeric_limits<double>::quiet_NaN();
    double height_max_m = std::numeric_limits<double>::quiet_NaN();
    double slope_rad = std::numeric_limits<double>::infinity();
    double roughness_m = std::numeric_limits<double>::infinity();
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    double swing_clearance_m = std::numeric_limits<double>::infinity();
    double swing_lift_m = 0.0;
    double swing_peak_phase = 0.5;
    double swing_leading_edge_phase = 0.5;
    bool swing_leading_edge_phase_valid = false;
    double uncertainty_m = std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct FootholdCandidate
{
    go2::Leg leg = go2::Leg::FR;
    go2::Vec3 foot_position{};
    std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
    std::uint64_t map_epoch = 0;
    std::uint32_t region_id = 0;
    double height_min_m = std::numeric_limits<double>::quiet_NaN();
    double height_max_m = std::numeric_limits<double>::quiet_NaN();
    double slope_rad = std::numeric_limits<double>::infinity();
    double roughness_m = std::numeric_limits<double>::infinity();
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    double swing_clearance_m = std::numeric_limits<double>::infinity();
    double swing_lift_m = 0.0;
    double swing_peak_phase = 0.5;
    double swing_leading_edge_phase = 0.5;
    bool swing_leading_edge_phase_valid = false;
    double support_margin_m = std::numeric_limits<double>::infinity();
    double collision_margin_m = std::numeric_limits<double>::infinity();
    double uncertainty_m = std::numeric_limits<double>::infinity();
    FootholdRejectReason reject_reason = FootholdRejectReason::kNone;
    bool hard_feasible = false;
};

inline double LegReachabilityMargin(
    go2::Leg leg, const go2::Vec3 &foot_position)
{
    const go2::LegGeometry geometry = go2::Geometry(leg);
    const double x = foot_position.x - geometry.hip_x;
    const double y = foot_position.y - geometry.hip_y;
    const double leg_z_squared = y * y + foot_position.z * foot_position.z -
        geometry.hip_link_y * geometry.hip_link_y;
    if (leg_z_squared < 0.0)
        return -std::numeric_limits<double>::infinity();
    const double leg_z = -std::sqrt(std::max(0.0, leg_z_squared));
    const double radial = std::hypot(x, leg_z);
    const double min_radial = std::abs(
        geometry.thigh_length - geometry.calf_length);
    const double max_radial = geometry.thigh_length + geometry.calf_length;
    return std::min(radial - min_radial, max_radial - radial);
}

// A terrain-derived asymmetric swing profile. The peak phase is selected
// from the observed height profile, so an early riser does not force an
// unnecessarily large centered arch. The endpoint and peak velocities are
// zero, so a short swing does not inject a position-derivative impulse when
// the terrain trajectory takes over or reaches touchdown.
inline double TerrainSwingEase(double s)
{
    const double t = std::clamp(s, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

inline double TerrainSwingEaseDerivative(double s)
{
    const double t = std::clamp(s, 0.0, 1.0);
    return 6.0 * t * (1.0 - t);
}

inline double TerrainSwingProfile(double phase, double peak_phase)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    const double p = std::clamp(peak_phase, 0.10, 0.90);
    const double t = u <= p ? u / p : (1.0 - u) / (1.0 - p);
    return TerrainSwingEase(t);
}

inline double TerrainSwingProfileDerivative(
    double phase, double peak_phase)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    const double p = std::clamp(peak_phase, 0.10, 0.90);
    if (u <= p)
        return TerrainSwingEaseDerivative(u / p) / p;
    return -TerrainSwingEaseDerivative((1.0 - u) / (1.0 - p)) /
        (1.0 - p);
}

// A sensor-observed leading edge is cleared before horizontal progress is
// resumed.  This is deliberately expressed in swing phase, not scene/world
// coordinates: the same trajectory contract works for any observed riser.
inline double TerrainSwingHorizontalPhase(
    double phase, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    if (!leading_edge_phase_valid || !std::isfinite(leading_edge_phase))
        return u;
    const double clear_phase = std::clamp(leading_edge_phase, 0.10, 0.75);
    if (u <= clear_phase)
        return 0.0;
    return std::clamp(
        (u - clear_phase) / std::max(1.0e-6, 1.0 - clear_phase),
        0.0, 1.0);
}

inline double TerrainSwingHorizontalPhaseDerivative(
    double phase, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    if (!leading_edge_phase_valid || !std::isfinite(leading_edge_phase))
        return 1.0;
    const double clear_phase = std::clamp(leading_edge_phase, 0.10, 0.75);
    return u <= clear_phase
        ? 0.0
        : 1.0 / std::max(1.0e-6, 1.0 - clear_phase);
}

inline double TerrainSwingPathProgress(
    double phase, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    return TerrainSwingEase(TerrainSwingHorizontalPhase(
        phase, leading_edge_phase_valid, leading_edge_phase));
}

inline double TerrainSwingPathProgressDerivative(
    double phase, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    const double horizontal_phase = TerrainSwingHorizontalPhase(
        phase, leading_edge_phase_valid, leading_edge_phase);
    return TerrainSwingEaseDerivative(horizontal_phase) *
        TerrainSwingHorizontalPhaseDerivative(
            phase, leading_edge_phase_valid, leading_edge_phase);
}

// A swing starts and ends in contact with the observed terrain.  For an
// asymmetric, terrain-derived profile, use the same phase envelope for the
// clearance budget and for the lift itself.  Keeping their endpoint orders
// equal avoids an artificial lift spike as both terms approach touchdown.
inline double TerrainSwingClearanceRequirement(
    double phase, double clearance_m, double peak_phase = 0.5)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    const double p = std::clamp(peak_phase, 0.10, 0.90);
    return clearance_m * TerrainSwingProfile(u, p);
}

inline bool CheckSwingClearance(
    const TerrainModel &model, const go2::Vec3 &start, const go2::Vec3 &end,
    double clearance_m, double &minimum_clearance_m,
    FootholdRejectReason *failure_reason = nullptr,
    go2::Leg leg = go2::Leg::FR,
    double *required_lift_m = nullptr,
    double *required_peak_phase = nullptr,
    double *leading_edge_phase = nullptr,
    bool *leading_edge_phase_valid = nullptr)
{
    minimum_clearance_m = std::numeric_limits<double>::infinity();
    if (failure_reason != nullptr)
        *failure_reason = FootholdRejectReason::kNone;
    if (required_lift_m != nullptr)
        *required_lift_m = 0.0;
    if (required_peak_phase != nullptr)
        *required_peak_phase = 0.5;
    if (leading_edge_phase != nullptr)
        *leading_edge_phase = 0.5;
    if (leading_edge_phase_valid != nullptr)
        *leading_edge_phase_valid = false;
    const auto reject = [&](FootholdRejectReason reason) {
        if (failure_reason != nullptr)
            *failure_reason = reason;
        return false;
    };
    if (!model.valid() || !std::isfinite(clearance_m) || clearance_m < 0.0)
        return reject(FootholdRejectReason::kInvalidModel);

    const double distance = std::hypot(end.x - start.x, end.y - start.y);
    const int samples = std::max(2, static_cast<int>(std::ceil(distance /
        std::max(model.resolution_m * 0.25, 0.005))));
    const double sweep_radius_m = std::max(0.025, 0.5 * model.resolution_m);
    double weighted_phase = 0.0;
    double excess_weight = 0.0;
    double first_rise_phase = -1.0;
    std::vector<double> terrain_height(static_cast<std::size_t>(samples + 1));
    for (int i = 0; i <= samples; ++i)
    {
        const double u = static_cast<double>(i) / samples;
        const double path_progress = TerrainSwingEase(u);
        const double x = start.x + path_progress * (end.x - start.x);
        const double y = start.y + path_progress * (end.y - start.y);
        TerrainPatch patch;
        if (!model.SamplePatch(x, y, sweep_radius_m, patch) ||
            !patch.valid || !patch.all_known)
            return reject(FootholdRejectReason::kUnknown);
        terrain_height[static_cast<std::size_t>(i)] = patch.max_height_m;
        if (i == 0)
        {
            std::size_t start_ix = 0;
            std::size_t start_iy = 0;
            if (!model.CellIndex(start.x, start.y, start_ix, start_iy))
                return reject(FootholdRejectReason::kUnknown);
            const TerrainCell *start_cell = model.CellAt(start_ix, start_iy);
            if (start_cell == nullptr || !start_cell->known ||
                !std::isfinite(start_cell->height_m))
                return reject(FootholdRejectReason::kUnknown);
            terrain_height[0] = start_cell->height_m;
        }
        if (i > 0 && first_rise_phase < 0.0 &&
            terrain_height[static_cast<std::size_t>(i)] -
                    terrain_height[0] > clearance_m)
            first_rise_phase = u;
        if (i > 0 && i < samples)
        {
            const double required = patch.max_height_m + clearance_m -
                (start.z + path_progress * (end.z - start.z));
            const double excess = std::max(0.0, required - clearance_m);
            weighted_phase += u * excess;
            excess_weight += excess;
        }
        // A measured support anchor may be above the terrain, but it must not
        // already be penetrating the swept terrain volume.
        if (i == 0 && start.z < terrain_height[0] - clearance_m)
            return reject(FootholdRejectReason::kSwingClearance);
    }

    // A patch-radius observation can report the riser before the foot reaches
    // its leading edge.  For a sharply resolved leading edge, the peak must
    // move earlier or the foot remains below the upper surface in the swept
    // patch.  Broad relief keeps the demand centroid, so the profile remains
    // terrain-derived without penalizing a long, gradual transition.
    // The choice is based only on the observed height profile.
    const double weighted_peak_phase = excess_weight > 1.0e-9
        ? std::clamp(weighted_phase / excess_weight, 0.10, 0.90)
        : -1.0;
    const double first_rise_peak_phase = first_rise_phase >= 0.0
        ? std::clamp(first_rise_phase, 0.10, 0.90)
        : 0.90;
    const bool leading_edge_dominant =
        weighted_peak_phase >= 0.0 && first_rise_phase >= 0.0 &&
        first_rise_phase < 0.5 * weighted_peak_phase;
    const double terrain_peak_phase = leading_edge_dominant
        ? first_rise_peak_phase
        : weighted_peak_phase;
    const double best_peak_phase = weighted_peak_phase >= 0.0
        ? terrain_peak_phase
        : (first_rise_phase >= 0.0 ? first_rise_peak_phase : 0.5);
    const bool observed_leading_edge = first_rise_phase >= 0.0;
    const double execution_peak_phase = observed_leading_edge
        ? std::max(
              best_peak_phase,
              std::clamp(first_rise_phase, 0.10, 0.75))
        : best_peak_phase;
    double lift = std::max(clearance_m, 0.050);
    double lift_phase = 0.0;
    double lift_terrain_height = 0.0;
    double lift_linear_height = 0.0;
    double lift_clearance_requirement = 0.0;
    double lift_shape = 1.0;
    for (int i = 1; i < samples; ++i)
    {
        const double u = static_cast<double>(i) / samples;
        const double path_progress = TerrainSwingPathProgress(
            u, observed_leading_edge, first_rise_phase);
        const double clearance_requirement =
            TerrainSwingClearanceRequirement(
                u, clearance_m, execution_peak_phase);
        const double linear_height =
            start.z + path_progress * (end.z - start.z);
        const double target_height = terrain_height[
            static_cast<std::size_t>(i)] + clearance_requirement;
        const double required = target_height - linear_height;
        if (required <= 0.0)
            continue;
        const double shape = TerrainSwingProfile(
            u, execution_peak_phase);
        if (!(shape > 1.0e-9))
            return reject(FootholdRejectReason::kSwingClearance);
        double lift_for_sample = std::max(lift, required / shape);
        while (std::fma(shape, lift_for_sample, linear_height) <
               target_height)
        {
            const double next_lift = std::nextafter(
                lift_for_sample, std::numeric_limits<double>::infinity());
            if (!(next_lift > lift_for_sample))
                return reject(FootholdRejectReason::kSwingClearance);
            lift_for_sample = next_lift;
        }
        if (lift_for_sample > lift)
        {
            lift = lift_for_sample;
            lift_phase = u;
            lift_terrain_height = terrain_height[
                static_cast<std::size_t>(i)];
            lift_linear_height =
                start.z + path_progress * (end.z - start.z);
            lift_clearance_requirement = clearance_requirement;
            lift_shape = shape;
        }
    }
    if (!std::isfinite(lift))
        return reject(FootholdRejectReason::kSwingClearance);

    const auto knee_position = [leg](
        const go2::LegJointPositions &joints) {
        const go2::LegGeometry geometry = go2::Geometry(leg);
        const double sin_hip = std::sin(joints.hip);
        const double cos_hip = std::cos(joints.hip);
        const double leg_x =
            -geometry.thigh_length * std::sin(joints.thigh);
        const double leg_z =
            -geometry.thigh_length * std::cos(joints.thigh);
        return go2::Vec3{
            geometry.hip_x + leg_x,
            geometry.hip_y + cos_hip * geometry.hip_link_y -
                sin_hip * leg_z,
            sin_hip * geometry.hip_link_y + cos_hip * leg_z};
    };

    bool foot_violation = false;
    bool shin_violation = false;
    double minimum_foot_clearance_m =
        std::numeric_limits<double>::infinity();
    double minimum_shin_clearance_m =
        std::numeric_limits<double>::infinity();
    double worst_foot_deficit_m = std::numeric_limits<double>::infinity();
    double worst_foot_phase = 0.0;
    double worst_foot_height_m = 0.0;
    double worst_foot_patch_height_m = 0.0;
    double worst_foot_sample_height_m = 0.0;
    double worst_foot_requirement_m = 0.0;
    const auto evaluate_swept_geometry = [&](double test_lift) {
        foot_violation = false;
        shin_violation = false;
        minimum_clearance_m = std::numeric_limits<double>::infinity();
        minimum_foot_clearance_m =
            std::numeric_limits<double>::infinity();
        minimum_shin_clearance_m =
            std::numeric_limits<double>::infinity();
        worst_foot_deficit_m = std::numeric_limits<double>::infinity();
        worst_foot_phase = 0.0;
        worst_foot_height_m = 0.0;
        worst_foot_patch_height_m = 0.0;
        worst_foot_sample_height_m = 0.0;
        worst_foot_requirement_m = 0.0;
        for (int i = 0; i <= samples; ++i)
        {
            const double u = static_cast<double>(i) / samples;
            const double clearance_requirement =
                TerrainSwingClearanceRequirement(
                    u, clearance_m, execution_peak_phase);
            const double clearance_threshold = std::nextafter(
                clearance_requirement,
                -std::numeric_limits<double>::infinity());
            const double path_progress = TerrainSwingPathProgress(
                u, observed_leading_edge, first_rise_phase);
            const double linear_height =
                start.z + path_progress * (end.z - start.z);
            const double foot_height = std::fma(
                TerrainSwingProfile(u, execution_peak_phase), test_lift,
                linear_height);
            const go2::Vec3 foot{
                start.x + path_progress * (end.x - start.x),
                start.y + path_progress * (end.y - start.y), foot_height};
            go2::LegJointPositions joints;
            if (!go2::LegInverseKinematics(leg, foot, joints))
                return FootholdRejectReason::kReachability;
            if (i == 0 || i == samples)
                continue;

            TerrainPatch foot_patch;
            if (!model.SamplePatch(
                    foot.x, foot.y, sweep_radius_m, foot_patch) ||
                !foot_patch.valid || !foot_patch.all_known)
                return FootholdRejectReason::kUnknown;
            const double foot_clearance = foot.z - foot_patch.max_height_m;
            const double foot_target_height =
                foot_patch.max_height_m + clearance_requirement;
            minimum_clearance_m = std::min(
                minimum_clearance_m, foot_clearance);
            minimum_foot_clearance_m = std::min(
                minimum_foot_clearance_m, foot_clearance);
            const double foot_deficit = foot.z - foot_target_height;
            if (foot_deficit < worst_foot_deficit_m)
            {
                worst_foot_deficit_m = foot_deficit;
                worst_foot_phase = u;
                worst_foot_height_m = foot.z;
                worst_foot_patch_height_m = foot_patch.max_height_m;
                worst_foot_sample_height_m = terrain_height[
                    static_cast<std::size_t>(i)];
                worst_foot_requirement_m = clearance_requirement;
            }
            if (foot.z < foot_target_height)
                foot_violation = true;

            // Check the lower-leg/shin segment at interior swept samples. The
            // touchdown endpoint is allowed to meet the terrain; the shin is
            // not. This is part of the lift solve, not a post-hoc rejection.
            const go2::Vec3 knee = knee_position(joints);
            for (int segment = 1; segment < 3; ++segment)
            {
                const double alpha = 0.3333333333333333 * segment;
                const go2::Vec3 shin{
                    knee.x + alpha * (foot.x - knee.x),
                    knee.y + alpha * (foot.y - knee.y),
                    knee.z + alpha * (foot.z - knee.z)};
                TerrainPatch shin_patch;
                if (!model.SamplePatch(
                        shin.x, shin.y, sweep_radius_m, shin_patch) ||
                    !shin_patch.valid || !shin_patch.all_known)
                    return FootholdRejectReason::kUnknown;
                const double shin_clearance =
                    shin.z - shin_patch.max_height_m;
                minimum_clearance_m = std::min(
                    minimum_clearance_m, shin_clearance);
                minimum_shin_clearance_m = std::min(
                    minimum_shin_clearance_m, shin_clearance);
                if (shin_clearance < clearance_threshold)
                    shin_violation = true;
            }
        }
        return FootholdRejectReason::kNone;
    };

    constexpr int kMaxLiftRefinementIterations = 32;
    for (int attempt = 0; attempt < kMaxLiftRefinementIterations; ++attempt)
    {
        const FootholdRejectReason geometry_reason =
            evaluate_swept_geometry(lift);
        if (geometry_reason != FootholdRejectReason::kNone)
            return reject(geometry_reason);
        if (!foot_violation && !shin_violation)
            break;
        const double foot_deficit =
            foot_violation && std::isfinite(worst_foot_deficit_m)
                ? std::max(0.0, -worst_foot_deficit_m)
                : 0.0;
        const double shin_deficit =
            shin_violation && std::isfinite(minimum_shin_clearance_m)
                ? std::max(0.0, clearance_m - minimum_shin_clearance_m)
                : 0.0;
        const double increment = std::max({0.002, foot_deficit,
                                           shin_deficit});
        const double next_lift = std::nextafter(
            lift + increment, std::numeric_limits<double>::infinity());
        if (!std::isfinite(next_lift) || !(next_lift > lift))
            return reject(shin_violation
                              ? FootholdRejectReason::kCollision
                              : FootholdRejectReason::kSwingClearance);
        lift = next_lift;
    }
    if (shin_violation)
        return reject(FootholdRejectReason::kCollision);
    if (foot_violation)
        return reject(FootholdRejectReason::kSwingClearance);
    if (!std::isfinite(lift))
        return reject(FootholdRejectReason::kSwingClearance);
    if (required_lift_m != nullptr)
        *required_lift_m = lift;
    if (required_peak_phase != nullptr)
        *required_peak_phase = execution_peak_phase;
    if (leading_edge_phase != nullptr && observed_leading_edge)
        *leading_edge_phase = std::clamp(first_rise_phase, 0.10, 0.75);
    if (leading_edge_phase_valid != nullptr)
        *leading_edge_phase_valid = observed_leading_edge;
    if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr &&
        lift > 0.15 &&
        (first_rise_phase >= 0.0 || end.z - start.z > 0.04))
    {
        static int debug_prints = 0;
        if (debug_prints < 64)
        {
            std::fprintf(
                stderr,
                "Terrain swing diagnostic leg=%d start=(%.6f,%.6f,%.6f) "
                "end=(%.6f,%.6f,%.6f) samples=%d first_rise=%.6f "
                "peak=%.6f lift=%.6f lift_phase=%.6f terrain=%.6f "
                "linear=%.6f req=%.6f shape=%.6f\n",
                static_cast<int>(leg), start.x, start.y, start.z,
                end.x, end.y, end.z, samples, first_rise_phase,
                execution_peak_phase, lift, lift_phase, lift_terrain_height,
                lift_linear_height, lift_clearance_requirement, lift_shape);
            ++debug_prints;
        }
    }
    if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr &&
        end.z - start.z > 0.04 &&
        (foot_violation || shin_violation))
    {
        static int debug_reject_prints = 0;
        if (debug_reject_prints < 128)
        {
            std::fprintf(
                stderr,
                "Terrain swing reject leg=%d start=(%.6f,%.6f,%.6f) "
                "end=(%.6f,%.6f,%.6f) peak=%.6f lift=%.6f "
                "min_clearance=%.6f foot_violation=%d shin_violation=%d\n",
                static_cast<int>(leg), start.x, start.y, start.z,
                end.x, end.y, end.z, best_peak_phase, lift,
                minimum_clearance_m, foot_violation ? 1 : 0,
                shin_violation ? 1 : 0);
            ++debug_reject_prints;
            std::fprintf(
                stderr,
                "Terrain swing clearance detail foot_min=%.6f "
                "foot_deficit=%+.17e foot_phase=%.6f shin_min=%.6f\n"
                "Terrain swing clearance values foot_z=%+.17e "
                "final_patch=%+.17e first_patch=%+.17e req=%+.17e "
                "patch_delta=%+.17e\n",
                minimum_foot_clearance_m, worst_foot_deficit_m,
                worst_foot_phase, minimum_shin_clearance_m,
                worst_foot_height_m, worst_foot_patch_height_m,
                worst_foot_sample_height_m, worst_foot_requirement_m,
                worst_foot_patch_height_m - worst_foot_sample_height_m);
        }
    }
    return true;
}

inline FootholdCandidate EvaluateFoothold(
    const TerrainModel &model, go2::Leg leg, double x_m, double y_m,
    const TerrainFeasibilityConfig &config,
    const go2::Vec3 *swing_start = nullptr,
    double swing_clearance_m = std::numeric_limits<double>::infinity(),
    const go2::Vec3 *future_base_displacement_base = nullptr)
{
    FootholdCandidate candidate;
    candidate.leg = leg;
    candidate.map_epoch = model.epoch;
    if (!model.valid())
    {
        candidate.reject_reason = FootholdRejectReason::kInvalidModel;
        return candidate;
    }
    if (model.frame_id != config.required_frame)
    {
        candidate.reject_reason = FootholdRejectReason::kFrameMismatch;
        return candidate;
    }
    if (model.age_s > config.max_map_age_s)
    {
        candidate.reject_reason = FootholdRejectReason::kStale;
        return candidate;
    }
    TerrainPatch patch;
    if (!model.SamplePatch(x_m, y_m, config.foot_patch_radius_m, patch) ||
        !patch.valid ||
        static_cast<double>(patch.known_cells) /
            std::max<std::size_t>(1, patch.total_cells) <
            config.min_known_fraction)
    {
        candidate.reject_reason = FootholdRejectReason::kUnknown;
        return candidate;
    }
    if (model.age_s > config.max_cell_age_s)
    {
        candidate.reject_reason = FootholdRejectReason::kStale;
        return candidate;
    }
    candidate.foot_position = {x_m, y_m, patch.center_height_m};
    candidate.surface_normal = patch.normal;
    candidate.height_min_m = patch.min_height_m;
    candidate.height_max_m = patch.max_height_m;
    candidate.slope_rad = patch.slope_rad;
    candidate.roughness_m = patch.roughness_m;
    candidate.edge_margin_m = patch.map_edge_margin_m;
    candidate.uncertainty_m = std::sqrt(std::max(0.0, patch.variance_m2));
    const go2::Vec3 reachability_position =
        future_base_displacement_base != nullptr
            ? go2::Vec3{
                  candidate.foot_position.x -
                      future_base_displacement_base->x,
                  candidate.foot_position.y -
                      future_base_displacement_base->y,
                  candidate.foot_position.z -
                      future_base_displacement_base->z}
            : candidate.foot_position;
    candidate.reachability_margin_m = LegReachabilityMargin(
        leg, reachability_position);
    if (candidate.edge_margin_m < config.min_edge_margin_m)
    {
        candidate.reject_reason = FootholdRejectReason::kEdge;
        return candidate;
    }
    if (patch.max_height_m - patch.min_height_m > config.max_surface_step_m)
    {
        candidate.reject_reason = FootholdRejectReason::kSurfaceStep;
        return candidate;
    }
    if (patch.slope_rad > config.max_slope_rad)
    {
        candidate.reject_reason = FootholdRejectReason::kSlope;
        return candidate;
    }
    if (patch.roughness_m > config.max_roughness_m)
    {
        candidate.reject_reason = FootholdRejectReason::kRoughness;
        return candidate;
    }
    if (candidate.uncertainty_m > std::sqrt(config.max_variance_m2))
    {
        candidate.reject_reason = FootholdRejectReason::kUncertainty;
        return candidate;
    }
    go2::LegJointPositions joints;
    if (!go2::LegInverseKinematics(leg, reachability_position, joints) ||
        candidate.reachability_margin_m < config.min_reachability_margin_m)
    {
        candidate.reject_reason = FootholdRejectReason::kReachability;
        return candidate;
    }
    if (swing_start != nullptr && std::isfinite(swing_clearance_m))
    {
        FootholdRejectReason swing_reject_reason =
            FootholdRejectReason::kSwingClearance;
        if (!CheckSwingClearance(
                model, *swing_start, candidate.foot_position,
                swing_clearance_m, candidate.swing_clearance_m,
                &swing_reject_reason, leg, &candidate.swing_lift_m,
                &candidate.swing_peak_phase,
                &candidate.swing_leading_edge_phase,
                &candidate.swing_leading_edge_phase_valid))
        {
            candidate.reject_reason = swing_reject_reason;
            return candidate;
        }
    }
    candidate.hard_feasible = true;
    candidate.support_margin_m = candidate.edge_margin_m;
    candidate.collision_margin_m = candidate.swing_clearance_m;
    return candidate;
}

inline std::vector<SafeFootholdRegion> BuildSafeFootholdRegions(
    const TerrainModel &model, go2::Leg leg,
    const TerrainFeasibilityConfig &config,
    const go2::Vec3 *future_base_displacement_base = nullptr)
{
    std::vector<SafeFootholdRegion> regions;
    if (!model.valid() || model.frame_id != config.required_frame)
        return regions;
    for (std::size_t iy = 0; iy < model.height; ++iy)
    {
        for (std::size_t ix = 0; ix < model.width; ++ix)
        {
            const TerrainCell *cell = model.CellAt(ix, iy);
            if (cell == nullptr || !cell->known)
                continue;
            const double x = model.origin_m[0] +
                (static_cast<double>(ix) + 0.5) * model.resolution_m;
            const double y = model.origin_m[1] +
                (static_cast<double>(iy) + 0.5) * model.resolution_m;
            const FootholdCandidate candidate = EvaluateFoothold(
                model, leg, x, y, config, nullptr,
                std::numeric_limits<double>::infinity(),
                future_base_displacement_base);
            if (!candidate.hard_feasible)
                continue;
            const double half = std::min(
                config.region_half_extent_m,
                0.5 * model.resolution_m - config.foot_patch_radius_m);
            if (!(half > 0.0))
                continue;
            SafeFootholdRegion region;
            region.leg = leg;
            region.map_epoch = model.epoch;
            region.region_id = static_cast<std::uint32_t>(regions.size());
            region.center = candidate.foot_position;
            region.normal = candidate.surface_normal;
            region.height_min_m = candidate.height_min_m;
            region.height_max_m = candidate.height_max_m;
            region.slope_rad = cell->slope_rad;
            region.roughness_m = cell->roughness_m;
            region.edge_margin_m = candidate.edge_margin_m;
            region.reachability_margin_m = candidate.reachability_margin_m;
            region.uncertainty_m = candidate.uncertainty_m;
            region.swing_clearance_m = candidate.swing_clearance_m;
            region.swing_lift_m = candidate.swing_lift_m;
            region.support_margin_m = candidate.support_margin_m;
            region.swing_peak_phase = candidate.swing_peak_phase;
            region.swing_leading_edge_phase =
                candidate.swing_leading_edge_phase;
            region.swing_leading_edge_phase_valid =
                candidate.swing_leading_edge_phase_valid;
            region.collision_margin_m = candidate.collision_margin_m;
            region.vertex_count = 4;
            region.vertices = {
                go2::Vec3{x - half, y - half, candidate.foot_position.z},
                go2::Vec3{x + half, y - half, candidate.foot_position.z},
                go2::Vec3{x + half, y + half, candidate.foot_position.z},
                go2::Vec3{x - half, y + half, candidate.foot_position.z}};
            region.valid = true;
            regions.push_back(region);
        }
    }
    return regions;
}

} // namespace go2_terrain
