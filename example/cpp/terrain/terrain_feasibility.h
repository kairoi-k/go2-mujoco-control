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

enum class TerrainTelemetryGate : std::uint8_t
{
    kNone = 0,
    kFootholdUnknownSurface,
    kFootholdUnknownPatch,
    kFootholdUnknownCell,
    kFootholdRejectOther,
    kSwingUnknownPathSample,
    kSwingUnknownMapCell,
    kSwingCollision,
    kSwingHeight,
    kSwingClearance,
    kSwingRejectOther,
};

constexpr std::size_t kTerrainTelemetryGateCount =
    static_cast<std::size_t>(TerrainTelemetryGate::kSwingRejectOther) + 1;

inline const char *TerrainTelemetryGateName(TerrainTelemetryGate gate)
{
    switch (gate)
    {
    case TerrainTelemetryGate::kFootholdUnknownSurface: return "foothold_unknown_surface";
    case TerrainTelemetryGate::kFootholdUnknownPatch: return "foothold_unknown_patch";
    case TerrainTelemetryGate::kFootholdUnknownCell: return "foothold_unknown_cell";
    case TerrainTelemetryGate::kFootholdRejectOther: return "foothold_reject_other";
    case TerrainTelemetryGate::kSwingUnknownPathSample: return "swing_unknown_path_sample";
    case TerrainTelemetryGate::kSwingUnknownMapCell: return "swing_unknown_map_cell";
    case TerrainTelemetryGate::kSwingCollision: return "swing_collision";
    case TerrainTelemetryGate::kSwingHeight: return "swing_height";
    case TerrainTelemetryGate::kSwingClearance: return "swing_clearance";
    case TerrainTelemetryGate::kSwingRejectOther: return "swing_reject_other";
    default: return "none";
    }
}

struct TerrainModel;
struct TerrainPatch;

struct TerrainTelemetryWitness
{
    bool valid = false;
    go2::Leg leg = go2::Leg::FR;
    TerrainTelemetryGate gate = TerrainTelemetryGate::kNone;
    std::uint64_t map_epoch = 0;
    std::uint64_t input_hash = 0;
    std::uint64_t plan_hash = 0;
    std::uint32_t candidate_index = 0;
    std::uint32_t candidate_count = 0;
    int path_sample = -1;
    int cell_ix = -1;
    int cell_iy = -1;
    go2::Vec3 patch_world{};
    go2::Vec3 cell_world{};
    double known_fraction = std::numeric_limits<double>::quiet_NaN();
    double sampled_height_m = std::numeric_limits<double>::quiet_NaN();
};

// Fixed-size, observer-only candidate provenance.  Counts saturate and the
// first witness is retained in a bounded array; no per-candidate log is
// allocated or emitted.
struct TerrainCandidateTelemetry
{
    bool enabled = false;
    std::uint64_t input_hash = 0;
    std::uint64_t plan_hash = 0;
    std::array<std::array<std::uint32_t, kTerrainTelemetryGateCount>,
               go2::kLegCount> counts_by_leg{};
    std::array<std::uint32_t, go2::kLegCount> evaluated_candidates{};
    std::array<std::uint32_t, go2::kLegCount> accepted_candidates{};
    std::array<TerrainTelemetryWitness, kTerrainTelemetryGateCount>
        first_witness{};
    std::array<std::array<TerrainTelemetryWitness, kTerrainTelemetryGateCount>,
               go2::kLegCount> first_witness_by_leg{};

    void Configure(bool on, std::uint64_t input, std::uint64_t plan)
    {
        *this = {};
        enabled = on;
        input_hash = input;
        plan_hash = plan;
    }

    void ObserveCandidate(go2::Leg leg)
    {
        if (!enabled || static_cast<std::size_t>(leg) >= go2::kLegCount)
            return;
        auto &count = evaluated_candidates[static_cast<std::size_t>(leg)];
        if (count != std::numeric_limits<std::uint32_t>::max())
            ++count;
    }

    void ObserveAccepted(go2::Leg leg)
    {
        if (!enabled || static_cast<std::size_t>(leg) >= go2::kLegCount)
            return;
        auto &count = accepted_candidates[static_cast<std::size_t>(leg)];
        if (count != std::numeric_limits<std::uint32_t>::max())
            ++count;
    }

    void Record(go2::Leg leg, TerrainTelemetryGate gate,
                std::uint64_t map_epoch, const go2::Vec3 &position,
                const TerrainModel *model = nullptr,
                const TerrainPatch *patch = nullptr, int path_sample = -1,
                std::uint32_t candidate_index = 0);
};

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
    // A terrain retarget is allowed to use a longer, sensor-derived swing
    // window when the checked path would otherwise demand an unrealizable
    // foot velocity.  This is a feasibility limit, not a safety-threshold
    // relaxation and does not alter the Phase 1 gait gains.
    // Calibrated against the measured crux step-up swing: 162 ms realized
    // over an ~0.41 m L1 path is a ~4.6-4.7 m/s eased-profile peak, and
    // flat nominal 125 ms swings already peak at ~3.5 m/s.  The old 2.50
    // cap doubled the predicted duration (330 ms), over-stretching the
    // timeline until the two-contact drift band exhausted; 4.50 keeps the
    // prediction just above the best observed swing.
    // See docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md section 10.
    double max_swing_speed_mps = 4.50;
    double region_half_extent_m = 0.035;
    // Upper-surface candidates must clear the observed forward terrain edge
    // by this distance. This is evaluated from the height transition, not
    // from a scene/world coordinate, and only applies to elevated candidates
    // in the terrain-actuated planner path.
    double elevated_surface_standoff_m = 0.080;
    // Existing low-surface support feet must stay behind the measured riser
    // edge; this is separate from the elevated swing-target stand-off above.
    double support_edge_standoff_m = 0.080;
    // Harness calibration bounds the lidar edge under-estimate to two map
    // cells; retain that correction before applying the stand-off.
    double elevated_surface_edge_bias_m = 0.100;
};

inline void TerrainCandidateTelemetry::Record(
    go2::Leg leg, TerrainTelemetryGate gate, std::uint64_t map_epoch,
    const go2::Vec3 &position, const TerrainModel *model,
    const TerrainPatch *patch, int path_sample, std::uint32_t candidate_index)
{
    if (!enabled || gate == TerrainTelemetryGate::kNone ||
        static_cast<std::size_t>(leg) >= go2::kLegCount)
        return;
    const auto gate_index = static_cast<std::size_t>(gate);
    if (gate_index >= kTerrainTelemetryGateCount)
        return;
    auto &count = counts_by_leg[static_cast<std::size_t>(leg)][gate_index];
    if (count != std::numeric_limits<std::uint32_t>::max())
        ++count;
    TerrainTelemetryWitness witness;
    witness.valid = true;
    witness.leg = leg;
    witness.gate = gate;
    witness.map_epoch = map_epoch;
    witness.input_hash = input_hash;
    witness.plan_hash = plan_hash;
    witness.candidate_index = candidate_index;
    witness.candidate_count = evaluated_candidates[
        static_cast<std::size_t>(leg)];
    witness.path_sample = path_sample;
    witness.patch_world = position;
    witness.sampled_height_m = patch != nullptr ? patch->max_height_m
                                                 : kTerrainNaN;
    witness.known_fraction = patch != nullptr && patch->total_cells > 0
        ? static_cast<double>(patch->known_cells) /
            static_cast<double>(patch->total_cells)
        : 0.0;
    if (model != nullptr)
    {
        std::size_t ix = 0;
        std::size_t iy = 0;
        if (model->CellIndex(position.x, position.y, ix, iy))
        {
            witness.cell_ix = static_cast<int>(ix);
            witness.cell_iy = static_cast<int>(iy);
            witness.cell_world = {
                model->origin_m[0] + (static_cast<double>(ix) + 0.5) *
                    model->resolution_m,
                model->origin_m[1] + (static_cast<double>(iy) + 0.5) *
                    model->resolution_m,
                model->CellAt(ix, iy) != nullptr
                    ? model->CellAt(ix, iy)->height_m : kTerrainNaN};
        }
    }
    if (!first_witness[gate_index].valid)
        first_witness[gate_index] = witness;
    if (!first_witness_by_leg[static_cast<std::size_t>(leg)][gate_index].valid)
        first_witness_by_leg[static_cast<std::size_t>(leg)][gate_index] =
            witness;
}

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
    // Planner-owned surface intent.  This is latched when the candidate is
    // ranked against the current support surface; support validation must not
    // reclassify it from a blended cell height.
    bool surface_transition_required = false;
    bool surface_transition_intent_valid = false;
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

// The quintic/eased path has a peak normalized derivative of 1.875.  Use an
// L1 path-length bound so the terrain transaction can request enough time for
// horizontal travel, elevation change, and the clearance arch without
// changing the nominal command or gait topology.
inline double TerrainSwingDurationForPath(
    double nominal_duration_s,
    const go2::Vec3 &start,
    const go2::Vec3 &end,
    double swing_lift_m,
    double max_swing_speed_mps)
{
    if (!std::isfinite(nominal_duration_s) ||
        nominal_duration_s <= 0.0 ||
        !std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(start.z) || !std::isfinite(end.x) ||
        !std::isfinite(end.y) || !std::isfinite(end.z) ||
        !std::isfinite(swing_lift_m) || swing_lift_m < 0.0 ||
        !std::isfinite(max_swing_speed_mps) ||
        max_swing_speed_mps <= 0.0)
        return nominal_duration_s;
    const double path_length =
        std::abs(end.x - start.x) +
        std::abs(end.y - start.y) +
        std::abs(end.z - start.z) +
        swing_lift_m;
    if (!std::isfinite(path_length))
        return nominal_duration_s;
    const double required_duration =
        1.875 * path_length / max_swing_speed_mps;
    return std::max(nominal_duration_s, required_duration);
}

// A sensor-observed leading edge changes the vertical clearance timing, while
// horizontal progress remains continuous through the whole swing.  Delaying
// horizontal motion until a late edge phase would create an unreachable
// endpoint jump at the fixed gait boundary.  This is deliberately expressed
// in swing phase, not scene/world coordinates: the same trajectory contract
// works for any observed riser.
inline double TerrainSwingHorizontalPhase(
    double phase, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    (void)leading_edge_phase_valid;
    (void)leading_edge_phase;
    return u;
}

inline double TerrainSwingHorizontalPhaseDerivative(
    double phase, bool leading_edge_phase_valid,
    double leading_edge_phase)
{
    const double u = std::clamp(phase, 0.0, 1.0);
    (void)leading_edge_phase_valid;
    (void)leading_edge_phase;
    (void)u;
    return 1.0;
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
    bool *leading_edge_phase_valid = nullptr,
    TerrainCandidateTelemetry *telemetry = nullptr)
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
    const auto reject = [&](FootholdRejectReason reason,
                             TerrainTelemetryGate gate =
                                 TerrainTelemetryGate::kSwingRejectOther,
                             go2::Vec3 position = {},
                             const TerrainPatch *patch = nullptr,
                             int path_sample = -1) {
        if (failure_reason != nullptr)
            *failure_reason = reason;
        if (telemetry != nullptr)
            telemetry->Record(leg, gate, model.epoch, position, &model, patch,
                              path_sample);
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
        if (i > 0)
        {
            TerrainPatch patch;
            if (!model.SamplePatch(x, y, sweep_radius_m, patch) ||
                !patch.valid || patch.HasUnknownInside())
            {
                if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
                {
                    static int debug_unknown_path_prints = 0;
                    if (debug_unknown_path_prints < 512)
                    {
                        std::fprintf(
                            stderr,
                            "Terrain swing reject unknown[path] leg=%d i=%d "
                            "xy=(%.6f,%.6f) start=(%.6f,%.6f,%.6f) "
                            "end=(%.6f,%.6f,%.6f)\n",
                            static_cast<int>(leg), i, x, y,
                            start.x, start.y, start.z, end.x, end.y, end.z);
                        ++debug_unknown_path_prints;
                    }
                }
                return reject(
                    FootholdRejectReason::kUnknown,
                    patch.HasUnknownInside()
                        ? TerrainTelemetryGate::kSwingUnknownMapCell
                        : TerrainTelemetryGate::kSwingUnknownPathSample,
                    {x, y, patch.max_height_m}, &patch, i);
            }
            // Fringe samples straddle the sensor FOV edge; their height is
            // the max over the observed subset (the unobservable strip
            // carries no constraint).  Only in-grid unobserved cells
            // (occlusion holes) reject above.
            terrain_height[static_cast<std::size_t>(i)] = patch.max_height_m;
        }
        if (i == 0)
        {
            std::size_t start_ix = 0;
            std::size_t start_iy = 0;
            if (!model.CellIndex(start.x, start.y, start_ix, start_iy))
            {
                if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
                {
                    static int debug_unknown_anchor_oob_prints = 0;
                    if (debug_unknown_anchor_oob_prints < 256)
                    {
                        std::fprintf(
                            stderr,
                            "Terrain swing reject unknown[anchor_oob] "
                            "leg=%d start=(%.6f,%.6f,%.6f)\n",
                            static_cast<int>(leg),
                            start.x, start.y, start.z);
                        ++debug_unknown_anchor_oob_prints;
                    }
                }
                return reject(FootholdRejectReason::kUnknown,
                              TerrainTelemetryGate::kSwingUnknownMapCell,
                              start, nullptr, i);
            }
            // The swing start is the live encoder-FK support foot,
            // verified against ground truth.  Its single 5 cm cell can
            // still read a neighboring riser top: the sparse lidar sweep
            // fills any cell touching the riser with the riser height,
            // which faked anchor penetration in epoch15a/16 (all 64
            // rejects shared one stance anchor at the riser base, cell
            // ~29 mm above the measured foot).  A real support foot
            // stands on the LOWEST surface in its vicinity, so compare
            // against the neighborhood minimum; a start genuinely below
            // every nearby cell still rejects.
            const int reach = std::max(1, static_cast<int>(std::ceil(
                sweep_radius_m / model.resolution_m)));
            double anchor_min_m = std::numeric_limits<double>::infinity();
            for (int oy = -reach; oy <= reach; ++oy)
            {
                for (int ox = -reach; ox <= reach; ++ox)
                {
                    const int nx = static_cast<int>(start_ix) + ox;
                    const int ny = static_cast<int>(start_iy) + oy;
                    if (nx < 0 || ny < 0)
                        continue;
                    const TerrainCell *neighbor = model.CellAt(
                        static_cast<std::size_t>(nx),
                        static_cast<std::size_t>(ny));
                    if (neighbor == nullptr || !neighbor->known ||
                        !std::isfinite(neighbor->height_m))
                        continue;
                    anchor_min_m = std::min(anchor_min_m, neighbor->height_m);
                }
            }
            if (!std::isfinite(anchor_min_m))
            {
                if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
                {
                    static int debug_unknown_anchor_cells_prints = 0;
                    if (debug_unknown_anchor_cells_prints < 256)
                    {
                        std::fprintf(
                            stderr,
                            "Terrain swing reject unknown[anchor_cells] "
                            "leg=%d start=(%.6f,%.6f,%.6f)\n",
                            static_cast<int>(leg),
                            start.x, start.y, start.z);
                        ++debug_unknown_anchor_cells_prints;
                    }
                }
                return reject(FootholdRejectReason::kUnknown,
                              TerrainTelemetryGate::kSwingUnknownMapCell,
                              start, nullptr, i);
            }
            terrain_height[0] = anchor_min_m;
        }
        if (i > 0 && first_rise_phase < 0.0 &&
            terrain_height[static_cast<std::size_t>(i)] -
                    terrain_height[0] > clearance_m)
            first_rise_phase = u;
        if (i > 0 && i < samples)
        {
            const double required =
                terrain_height[static_cast<std::size_t>(i)] + clearance_m -
                (start.z + path_progress * (end.z - start.z));
            const double excess = std::max(0.0, required - clearance_m);
            weighted_phase += u * excess;
            excess_weight += excess;
        }
        // A measured support anchor may be above the terrain, but it must
        // not already be penetrating even the lowest nearby ground.
        if (i == 0 && start.z < terrain_height[0] - clearance_m)
        {
            if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
            {
                static int debug_anchor_prints = 0;
                if (debug_anchor_prints < 64)
                {
                    std::fprintf(
                        stderr,
                        "Terrain swing reject anchor leg=%d "
                        "start=(%.6f,%.6f,%.6f) end=(%.6f,%.6f,%.6f) "
                        "terrain0=%.6f clearance=%.6f\n",
                        static_cast<int>(leg), start.x, start.y, start.z,
                        end.x, end.y, end.z, terrain_height[0],
                        clearance_m);
                    ++debug_anchor_prints;
                }
            }
            return reject(FootholdRejectReason::kSwingClearance,
                          TerrainTelemetryGate::kSwingHeight, start, nullptr,
                          i);
        }
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
        ? std::min(
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
        {
            if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
            {
                static int debug_shape_prints = 0;
                if (debug_shape_prints < 64)
                {
                    std::fprintf(
                        stderr,
                        "Terrain swing reject shape leg=%d "
                        "start=(%.6f,%.6f,%.6f) end=(%.6f,%.6f,%.6f) "
                        "u=%.6f terrain=%.6f linear=%.6f required=%.6f "
                        "req_clearance=%.6f peak=%.6f shape=%.3e\n",
                        static_cast<int>(leg), start.x, start.y, start.z,
                        end.x, end.y, end.z, u,
                        terrain_height[static_cast<std::size_t>(i)],
                        linear_height, required, clearance_requirement,
                        execution_peak_phase, shape);
                    ++debug_shape_prints;
                }
            }
            return reject(FootholdRejectReason::kSwingClearance,
                          TerrainTelemetryGate::kSwingClearance,
                          {start.x + path_progress * (end.x - start.x),
                           start.y + path_progress * (end.y - start.y),
                           linear_height}, nullptr, i);
        }
        double lift_for_sample = std::max(lift, required / shape);
        while (std::fma(shape, lift_for_sample, linear_height) <
               target_height)
        {
            const double next_lift = std::nextafter(
                lift_for_sample, std::numeric_limits<double>::infinity());
            if (!(next_lift > lift_for_sample))
                return reject(FootholdRejectReason::kSwingClearance,
                              TerrainTelemetryGate::kSwingClearance, start,
                              nullptr, i);
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
        return reject(FootholdRejectReason::kSwingClearance,
                      TerrainTelemetryGate::kSwingClearance);

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
            {
                if (telemetry != nullptr)
                    telemetry->Record(leg,
                        TerrainTelemetryGate::kSwingRejectOther,
                        model.epoch, foot, &model, nullptr, i);
                return FootholdRejectReason::kReachability;
            }
            if (i == 0 || i == samples)
                continue;

            TerrainPatch foot_patch;
            if (!model.SamplePatch(
                    foot.x, foot.y, sweep_radius_m, foot_patch) ||
                !foot_patch.valid || foot_patch.HasUnknownInside())
            {
                if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
                {
                    static int debug_unknown_foot_prints = 0;
                    if (debug_unknown_foot_prints < 512)
                    {
                        std::fprintf(
                            stderr,
                            "Terrain swing reject unknown[foot] leg=%d i=%d "
                            "foot=(%.6f,%.6f,%.6f) start=(%.6f,%.6f,%.6f) "
                            "end=(%.6f,%.6f,%.6f)\n",
                            static_cast<int>(leg), i,
                            foot.x, foot.y, foot.z,
                            start.x, start.y, start.z, end.x, end.y, end.z);
                        ++debug_unknown_foot_prints;
                    }
                }
                if (telemetry != nullptr)
                    telemetry->Record(leg,
                        foot_patch.HasUnknownInside()
                            ? TerrainTelemetryGate::kSwingUnknownMapCell
                            : TerrainTelemetryGate::kSwingUnknownPathSample,
                        model.epoch, foot, &model, &foot_patch, i);
                return FootholdRejectReason::kUnknown;
            }
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
                    !shin_patch.valid || shin_patch.HasUnknownInside())
                {
                    // The local map window never observes its own lateral
                    // fringe (y beyond +/-0.225 m), and a rear leg's
                    // knee/shin grazes that fringe in every crux swing —
                    // rejecting there starved all 32 regions with kUnknown
                    // in epoch17 while the window was fully known.  An
                    // unobserved fringe carries no terrain constraint, so
                    // skip it; holes INSIDE the swept window still reject.
                    if (model.CoversPatch(shin.x, shin.y, sweep_radius_m))
                    {
                        if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
                        {
                            static int debug_unknown_shin_prints = 0;
                            if (debug_unknown_shin_prints < 512)
                            {
                                std::fprintf(
                                    stderr,
                                    "Terrain swing reject unknown[shin] leg=%d "
                                    "i=%d seg=%d shin=(%.6f,%.6f,%.6f) "
                                    "knee=(%.6f,%.6f,%.6f) foot=(%.6f,%.6f,%.6f)\n",
                                    static_cast<int>(leg), i, segment,
                                    shin.x, shin.y, shin.z,
                                    knee.x, knee.y, knee.z,
                                    foot.x, foot.y, foot.z);
                                ++debug_unknown_shin_prints;
                            }
                        }
                        if (telemetry != nullptr)
                            telemetry->Record(leg,
                                TerrainTelemetryGate::kSwingUnknownMapCell,
                                model.epoch, shin, &model, &shin_patch, i);
                        return FootholdRejectReason::kUnknown;
                    }
                    if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
                    {
                        static int debug_fringe_shin_prints = 0;
                        if (debug_fringe_shin_prints < 512)
                        {
                            std::fprintf(
                                stderr,
                                "Terrain swing skip fringe[shin] leg=%d "
                                "i=%d seg=%d shin=(%.6f,%.6f,%.6f)\n",
                                static_cast<int>(leg), i, segment,
                                shin.x, shin.y, shin.z);
                            ++debug_fringe_shin_prints;
                        }
                    }
                    continue;
                }
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
            return reject(geometry_reason,
                          geometry_reason == FootholdRejectReason::kCollision
                              ? TerrainTelemetryGate::kSwingCollision
                              : geometry_reason == FootholdRejectReason::kSwingClearance
                                    ? TerrainTelemetryGate::kSwingHeight
                                    : TerrainTelemetryGate::kSwingRejectOther);
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
                              : FootholdRejectReason::kSwingClearance,
                          shin_violation
                              ? TerrainTelemetryGate::kSwingCollision
                              : TerrainTelemetryGate::kSwingClearance);
        lift = next_lift;
    }
    if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr &&
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
    if (shin_violation)
        return reject(FootholdRejectReason::kCollision,
                      TerrainTelemetryGate::kSwingCollision);
    if (foot_violation)
        return reject(FootholdRejectReason::kSwingClearance,
                      TerrainTelemetryGate::kSwingHeight);
    if (!std::isfinite(lift))
        return reject(FootholdRejectReason::kSwingClearance,
                      TerrainTelemetryGate::kSwingClearance);
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
    return true;
}

inline FootholdCandidate EvaluateFoothold(
    const TerrainModel &model, go2::Leg leg, double x_m, double y_m,
    const TerrainFeasibilityConfig &config,
    const go2::Vec3 *swing_start = nullptr,
    double swing_clearance_m = std::numeric_limits<double>::infinity(),
    const go2::Vec3 *future_base_displacement_base = nullptr,
    TerrainCandidateTelemetry *telemetry = nullptr)
{
    FootholdCandidate candidate;
    candidate.leg = leg;
    candidate.map_epoch = model.epoch;
    const std::uint32_t candidate_index = telemetry != nullptr &&
        telemetry->enabled && static_cast<std::size_t>(leg) < go2::kLegCount
        ? telemetry->evaluated_candidates[static_cast<std::size_t>(leg)] : 0;
    if (telemetry != nullptr)
        telemetry->ObserveCandidate(leg);
    const auto record_reject = [&](TerrainTelemetryGate gate,
                                   const go2::Vec3 &position,
                                   const TerrainPatch *patch = nullptr) {
        if (telemetry != nullptr)
            telemetry->Record(leg, gate, model.epoch, position, &model, patch,
                              -1, candidate_index);
    };
    if (!model.valid())
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      {x_m, y_m, kTerrainNaN});
        candidate.reject_reason = FootholdRejectReason::kInvalidModel;
        return candidate;
    }
    if (model.frame_id != config.required_frame)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      {x_m, y_m, kTerrainNaN});
        candidate.reject_reason = FootholdRejectReason::kFrameMismatch;
        return candidate;
    }
    if (model.age_s > config.max_map_age_s)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      {x_m, y_m, kTerrainNaN});
        candidate.reject_reason = FootholdRejectReason::kStale;
        return candidate;
    }
    TerrainPatch patch;
    const bool patch_sampled = model.SamplePatch(
        x_m, y_m, config.foot_patch_radius_m, patch);
    const bool patch_known = static_cast<double>(patch.known_cells) /
        std::max<std::size_t>(1, patch.total_cells) >=
        config.min_known_fraction;
    if (!patch_sampled || !patch.valid || !patch_known)
    {
        const TerrainTelemetryGate gate = !patch_sampled
            ? (patch.known_cells == 0
                   ? TerrainTelemetryGate::kFootholdUnknownSurface
                   : TerrainTelemetryGate::kFootholdUnknownPatch)
            : (!patch.valid ? TerrainTelemetryGate::kFootholdUnknownPatch
                            : (patch.HasUnknownInside()
                                   ? TerrainTelemetryGate::kFootholdUnknownCell
                                   : TerrainTelemetryGate::kFootholdUnknownPatch));
        record_reject(gate, {x_m, y_m, patch.center_height_m}, &patch);
        if (std::getenv("TROT_TERRAIN_DEBUG_SWING") != nullptr)
        {
            static int debug_unknown_foothold_prints = 0;
            if (debug_unknown_foothold_prints < 256)
            {
                std::fprintf(
                    stderr,
                    "Terrain foothold reject unknown leg=%d "
                    "xy=(%.6f,%.6f) known=%zu/%zu\n",
                    static_cast<int>(leg), x_m, y_m,
                    patch.known_cells, patch.total_cells);
                ++debug_unknown_foothold_prints;
            }
        }
        candidate.reject_reason = FootholdRejectReason::kUnknown;
        return candidate;
    }
    if (model.age_s > config.max_cell_age_s)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
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
    // Map elevations describe the contact patch. Analytical FK and the
    // MuJoCo foot point describe the collision-sphere site, so reachability
    // must evaluate the calibrated site target rather than the patch plane.
    const go2::Vec3 reachability_position =
        future_base_displacement_base != nullptr
            ? go2::Vec3{
                  candidate.foot_position.x -
                      future_base_displacement_base->x,
                  candidate.foot_position.y -
                      future_base_displacement_base->y,
                  candidate.foot_position.z -
                      future_base_displacement_base->z +
                      go2::kFootSiteToContactPatchOffsetM}
            : go2::ContactPatchToFootSite(candidate.foot_position);
    candidate.reachability_margin_m = LegReachabilityMargin(
        leg, reachability_position);
    if (candidate.edge_margin_m < config.min_edge_margin_m)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
        candidate.reject_reason = FootholdRejectReason::kEdge;
        return candidate;
    }
    if (patch.max_height_m - patch.min_height_m > config.max_surface_step_m)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
        candidate.reject_reason = FootholdRejectReason::kSurfaceStep;
        return candidate;
    }
    if (patch.slope_rad > config.max_slope_rad)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
        candidate.reject_reason = FootholdRejectReason::kSlope;
        return candidate;
    }
    if (patch.roughness_m > config.max_roughness_m)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
        candidate.reject_reason = FootholdRejectReason::kRoughness;
        return candidate;
    }
    if (candidate.uncertainty_m > std::sqrt(config.max_variance_m2))
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
        candidate.reject_reason = FootholdRejectReason::kUncertainty;
        return candidate;
    }
    go2::LegJointPositions joints;
    if (!go2::LegInverseKinematics(leg, reachability_position, joints) ||
        candidate.reachability_margin_m < config.min_reachability_margin_m)
    {
        record_reject(TerrainTelemetryGate::kFootholdRejectOther,
                      candidate.foot_position, &patch);
        candidate.reject_reason = FootholdRejectReason::kReachability;
        return candidate;
    }
    if (swing_start != nullptr && std::isfinite(swing_clearance_m))
    {
        FootholdRejectReason swing_reject_reason =
            FootholdRejectReason::kSwingClearance;
        if (!CheckSwingClearance(
                model, *swing_start,
                go2::ContactPatchToFootSite(candidate.foot_position),
                swing_clearance_m, candidate.swing_clearance_m,
                &swing_reject_reason, leg, &candidate.swing_lift_m,
                &candidate.swing_peak_phase,
                &candidate.swing_leading_edge_phase,
                &candidate.swing_leading_edge_phase_valid, telemetry))
        {
            candidate.reject_reason = swing_reject_reason;
            return candidate;
        }
    }
    candidate.hard_feasible = true;
    if (telemetry != nullptr)
        telemetry->ObserveAccepted(leg);
    candidate.support_margin_m = candidate.edge_margin_m;
    candidate.collision_margin_m = candidate.swing_clearance_m;
    return candidate;
}

inline double ForwardElevatedSurfaceEdgeX(
    const TerrainModel &model, const go2::Vec3 &position,
    double patch_radius_m)
{
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
        return std::numeric_limits<double>::quiet_NaN();
    const double lower_surface_delta = std::max(0.030, patch_radius_m);
    std::size_t ix = 0;
    std::size_t center_iy = 0;
    if (!model.CellIndex(position.x, position.y, ix, center_iy))
        return std::numeric_limits<double>::quiet_NaN();
    (void)center_iy;
    const double lateral_radius = std::max(0.070, 2.0 * patch_radius_m);
    const int iy_min = std::max(
        0, static_cast<int>(std::floor(
            (position.y - lateral_radius - model.origin_m[1]) /
            model.resolution_m)));
    const int iy_max = std::min(
        static_cast<int>(model.height) - 1,
        static_cast<int>(std::floor(
            (position.y + lateral_radius - model.origin_m[1]) /
            model.resolution_m)));
    double nearest_edge_x = -std::numeric_limits<double>::infinity();
    for (int iy = iy_min; iy <= iy_max; ++iy)
    {
        const TerrainCell *candidate_cell = model.CellAt(
            ix, static_cast<std::size_t>(iy));
        if (candidate_cell == nullptr || !candidate_cell->known ||
            candidate_cell->height_m < position.z - lower_surface_delta)
            continue;
        for (std::size_t edge_ix = ix + 1; edge_ix > 0; --edge_ix)
        {
            const TerrainCell *cell = model.CellAt(
                edge_ix - 1, static_cast<std::size_t>(iy));
            if (cell == nullptr || !cell->known)
                continue;
            if (cell->height_m < position.z - lower_surface_delta)
            {
                nearest_edge_x = std::max(
                    nearest_edge_x, model.origin_m[0] +
                        static_cast<double>(edge_ix) * model.resolution_m);
                break;
            }
        }
    }
    return std::isfinite(nearest_edge_x)
        ? nearest_edge_x : std::numeric_limits<double>::quiet_NaN();
}

// Return the first persistent height transition in front of a low-surface
// foothold. Unlike ForwardElevatedSurfaceEdgeX, this observes the edge from
// the lower side, which is the geometry needed to keep existing support feet
// away from a riser lip.
inline double ForwardRiserEdgeX(
    const TerrainModel &model, const go2::Vec3 &position,
    double patch_radius_m)
{
    if (!model.valid() || !std::isfinite(position.x) ||
        !std::isfinite(position.y) || !std::isfinite(position.z))
        return std::numeric_limits<double>::quiet_NaN();
    std::size_t center_ix = 0;
    std::size_t center_iy = 0;
    if (!model.CellIndex(position.x, position.y, center_ix, center_iy))
        return std::numeric_limits<double>::quiet_NaN();
    (void)center_iy;
    const double lateral_radius = std::max(0.070, 2.0 * patch_radius_m);
    const int iy_min = std::max(
        0, static_cast<int>(std::floor(
            (position.y - lateral_radius - model.origin_m[1]) /
            model.resolution_m)));
    const int iy_max = std::min(
        static_cast<int>(model.height) - 1,
        static_cast<int>(std::floor(
            (position.y + lateral_radius - model.origin_m[1]) /
            model.resolution_m)));
    for (std::size_t edge_ix = center_ix + 1; edge_ix < model.width;
         ++edge_ix)
    {
        std::size_t transition_rows = 0;
        for (int iy = iy_min; iy <= iy_max; ++iy)
        {
            const auto *before = model.CellAt(
                edge_ix - 1, static_cast<std::size_t>(iy));
            const auto *after = model.CellAt(
                edge_ix, static_cast<std::size_t>(iy));
            if (before != nullptr && after != nullptr && before->known &&
                after->known && after->height_m > before->height_m + 0.02)
                ++transition_rows;
        }
        // Two lateral rows reject isolated lidar spikes and match the
        // consensus used by the direct crawl target.
        if (transition_rows >= 2)
            return model.origin_m[0] +
                static_cast<double>(edge_ix) * model.resolution_m;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

inline bool HasForwardSupportEdgeStandoff(
    const TerrainModel &model, const go2::Vec3 &position,
    double standoff_m, double patch_radius_m)
{
    if (!(standoff_m > 0.0) || !std::isfinite(position.x))
        return true;
    const double edge_x = ForwardRiserEdgeX(model, position, patch_radius_m);
    return !std::isfinite(edge_x) ||
        position.x <= edge_x - standoff_m + 1.0e-9;
}

inline bool HasForwardElevatedSurfaceStandoff(
    const TerrainModel &model, const go2::Vec3 &position,
    double reference_height_m, double standoff_m, double patch_radius_m)
{
    if (!(standoff_m > 0.0) || !std::isfinite(reference_height_m) ||
        !std::isfinite(position.z) ||
        position.z <= reference_height_m + std::max(
            2.0 * std::max(0.0, patch_radius_m), 0.5 * patch_radius_m))
        return true;
    const double edge_x = ForwardElevatedSurfaceEdgeX(
        model, position, patch_radius_m);
    return std::isfinite(edge_x) &&
        position.x - edge_x + 1.0e-9 >= standoff_m;
}

inline std::vector<SafeFootholdRegion> BuildSafeFootholdRegions(
    const TerrainModel &model, go2::Leg leg,
    const TerrainFeasibilityConfig &config,
    const go2::Vec3 *future_base_displacement_base = nullptr,
    TerrainCandidateTelemetry *telemetry = nullptr)
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
