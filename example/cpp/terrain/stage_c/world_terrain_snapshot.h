#pragma once
// Bounded, immutable-by-query history for capture-heading terrain views.
// The snapshot never fills holes or changes source models.  Historical use is
// explicitly conditional on a caller-declared stationary-terrain assumption;
// cell freshness is evaluated against snapshot.metadata.state_time_s, independently
// of any future prediction horizon.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include "capture_terrain_view.h"
#include "types.h"
#include "world_terrain_view.h"
namespace go2_terrain
{
namespace stage_c
{
constexpr std::size_t kWorldTerrainSnapshotMaxCaptures = 8;
constexpr double kWorldTerrainSnapshotDefaultHeightConflictToleranceM = 1.0e-3;
constexpr double kWorldTerrainSnapshotMaxHeightConflictToleranceM = 0.10;
constexpr double kWorldTerrainSnapshotDefaultMinimumNormalDot = 0.9998;
constexpr double kWorldTerrainSnapshotMinimumNormalZ = 1.0e-6;
// Capture sources are compared only where both views provide fresh, finite
// geometric evidence.  A missing or unsupported cell remains unavailable.
enum class WorldTerrainSnapshotError : std::uint8_t
{
    kNone = 0,
    kInvalidInput,
    kEmpty,
    kTooManyCaptures,
    kStationaryAssumptionRequired,
    kCaptureInvalid,
    kEpochMismatch,
    kFutureCapture,
    kDuplicateSource,
    kQueryInvalid,
    kOutside,
    kUnknownCells,
    kStaleCells,
    kNoFreshCompletePatch,
    kConflictingHistory,
    kNonfinitePatch,
};
inline const char *WorldTerrainSnapshotErrorName(
    WorldTerrainSnapshotError error)
{
    switch (error)
    {
    case WorldTerrainSnapshotError::kNone: return "none";
    case WorldTerrainSnapshotError::kInvalidInput: return "invalid_input";
    case WorldTerrainSnapshotError::kEmpty: return "empty";
    case WorldTerrainSnapshotError::kTooManyCaptures: return "too_many_captures";
    case WorldTerrainSnapshotError::kStationaryAssumptionRequired:
        return "stationary_assumption_required";
    case WorldTerrainSnapshotError::kCaptureInvalid: return "capture_invalid";
    case WorldTerrainSnapshotError::kEpochMismatch: return "epoch_mismatch";
    case WorldTerrainSnapshotError::kFutureCapture: return "future_capture";
    case WorldTerrainSnapshotError::kDuplicateSource: return "duplicate_source";
    case WorldTerrainSnapshotError::kQueryInvalid: return "query_invalid";
    case WorldTerrainSnapshotError::kOutside: return "outside";
    case WorldTerrainSnapshotError::kUnknownCells: return "unknown_cells";
    case WorldTerrainSnapshotError::kStaleCells: return "stale_cells";
    case WorldTerrainSnapshotError::kNoFreshCompletePatch:
        return "no_fresh_complete_patch";
    case WorldTerrainSnapshotError::kConflictingHistory:
        return "conflicting_history";
    case WorldTerrainSnapshotError::kNonfinitePatch: return "nonfinite_patch";
    default: return "unknown";
    }
}
struct WorldTerrainSnapshotOptions
{
    std::size_t max_captures = kWorldTerrainSnapshotMaxCaptures;
    // This must be explicitly enabled by the caller.  It authorizes bounded
    // historical reuse only; it does not suppress conflicts in new evidence.
    bool stationary_terrain_assumption = false;
    double height_conflict_tolerance_m =
        kWorldTerrainSnapshotDefaultHeightConflictToleranceM;
    double minimum_normal_dot = kWorldTerrainSnapshotDefaultMinimumNormalDot;
};
struct WorldTerrainSnapshotEntry
{
    TerrainModel model{};
    CaptureTerrainViewProvenance provenance{};
    CaptureTerrainViewScope scope = static_cast<CaptureTerrainViewScope>(0);
};
struct WorldTerrainSnapshotMetadata
{
    std::uint64_t aggregate_epoch = 0;
    double state_time_s = kTerrainMapUnknown;
    bool stationary_terrain_assumption = false;
    double height_conflict_tolerance_m =
        kWorldTerrainSnapshotDefaultHeightConflictToleranceM;
    double minimum_normal_dot = kWorldTerrainSnapshotDefaultMinimumNormalDot;
};
struct WorldTerrainSnapshot
{
    WorldTerrainSnapshotMetadata metadata{};
    bool valid = false;
    std::vector<WorldTerrainSnapshotEntry> captures;
    bool ok() const
    {
        return valid && metadata.aggregate_epoch != 0 &&
            std::isfinite(metadata.state_time_s) &&
            metadata.stationary_terrain_assumption &&
            std::isfinite(metadata.height_conflict_tolerance_m) &&
            metadata.height_conflict_tolerance_m >= 0.0 &&
            metadata.height_conflict_tolerance_m <=
                kWorldTerrainSnapshotMaxHeightConflictToleranceM &&
            std::isfinite(metadata.minimum_normal_dot) &&
            metadata.minimum_normal_dot >= 0.0 &&
            metadata.minimum_normal_dot <= 1.0 &&
            !captures.empty() &&
            captures.size() <= kWorldTerrainSnapshotMaxCaptures;
    }
    const WorldTerrainSnapshotEntry *latest_entry() const
    {
        return captures.empty() ? nullptr : &captures.front();
    }
    const TerrainModel *latest_model() const
    {
        const WorldTerrainSnapshotEntry *entry = latest_entry();
        return entry == nullptr ? nullptr : &entry->model;
    }
};
struct WorldTerrainSnapshotBuildResult
{
    WorldTerrainSnapshot snapshot{};
    WorldTerrainSnapshotError error = WorldTerrainSnapshotError::kInvalidInput;
    bool valid = false;
    bool ok() const
    {
        return valid && error == WorldTerrainSnapshotError::kNone &&
            snapshot.ok();
    }
};
struct WorldTerrainSnapshotQueryResult
{
    TerrainPatch patch{};
    WorldTerrainSnapshotError error = WorldTerrainSnapshotError::kInvalidInput;
    WorldTerrainQueryFailure latest_failure = WorldTerrainQueryFailure::kNone;
    WorldTerrainQueryDiagnostic diagnostic{};
    double selected_source_time_s = kTerrainMapUnknown;
    TimeNs selected_source_time{std::numeric_limits<std::int64_t>::min()};
    double conflict_height_gap_m = kTerrainMapUnknown;
    double conflict_normal_dot = kTerrainMapUnknown;
    std::uint64_t conflict_newer_sequence = 0;
    std::uint64_t conflict_older_sequence = 0;
    std::uint64_t selected_source_sequence = 0;
    bool history_used = false;
    bool stationary_terrain_assumption = false;
    bool valid = false;
    bool ok() const
    {
        return valid && error == WorldTerrainSnapshotError::kNone &&
            patch.valid && stationary_terrain_assumption &&
            std::isfinite(selected_source_time_s) &&
            selected_source_time.value !=
                std::numeric_limits<std::int64_t>::min() &&
            selected_source_sequence != 0;
    }
};
namespace world_terrain_snapshot_detail
{
inline bool KnownTerrainSource(TerrainSource source)
{
    switch (source)
    {
    case TerrainSource::kLidar:
    case TerrainSource::kStateEstimator:
    case TerrainSource::kTestFixture:
        return true;
    default:
        return false;
    }
}
inline bool Close(double left, double right, double tolerance)
{
    return std::isfinite(left) && std::isfinite(right) &&
        std::abs(left - right) <= tolerance;
}
inline bool SameArray3(const std::array<double, 3> &left,
                       const std::array<double, 3> &right,
                       double tolerance)
{
    return Close(left[0], right[0], tolerance) &&
        Close(left[1], right[1], tolerance) &&
        Close(left[2], right[2], tolerance);
}
inline bool CaptureHeadingEntryValid(
    const WorldTerrainSnapshotEntry &entry, std::uint64_t aggregate_epoch,
    double state_time_s)
{
    const TerrainModel &model = entry.model;
    const CaptureTerrainViewProvenance &provenance = entry.provenance;
    if (!model.valid() || !WorldTerrainMetadataValid(model) ||
        entry.scope != CaptureTerrainViewScope::kCaptureHeadingWorldQuery ||
        model.frame_id != "base_link" || !model.registered ||
        !KnownTerrainSource(model.source) || model.epoch != aggregate_epoch ||
        provenance.map_epoch != aggregate_epoch ||
        provenance.source != model.source ||
        provenance.source_sequence != model.map_sequence ||
        provenance.source_sequence == 0 ||
        !Close(provenance.source_map_stamp_s, model.map_stamp_s,
               kTerrainMapTimeToleranceS) ||
        !Close(provenance.state_stamp_s, model.state_stamp_s,
               kTerrainMapTimeToleranceS) ||
        !SameArray3(provenance.capture_position_world,
                    model.capture_position_world,
                    kTerrainMapTimeToleranceS) ||
        !Close(provenance.capture_yaw_rad, model.capture_yaw_rad,
               kTerrainMapTimeToleranceS) ||
        !SameArray3(model.capture_position_world,
                    model.registration_position_world,
                    kTerrainMapTimeToleranceS) ||
        !Close(model.capture_yaw_rad, model.registration_yaw_rad,
               kTerrainMapTimeToleranceS))
        return false;
    return std::isfinite(state_time_s) &&
        model.state_stamp_s <= state_time_s + kTerrainMapTimeToleranceS;
}
inline bool EntryBefore(const WorldTerrainSnapshotEntry &left,
                        const WorldTerrainSnapshotEntry &right)
{
    if (left.model.map_sequence != right.model.map_sequence)
        return left.model.map_sequence > right.model.map_sequence;
    if (left.model.map_stamp_s != right.model.map_stamp_s)
        return left.model.map_stamp_s > right.model.map_stamp_s;
    return left.model.state_stamp_s > right.model.state_stamp_s;
}
inline bool SnapshotEntriesValid(const WorldTerrainSnapshot &snapshot)
{
    if (snapshot.captures.empty() ||
        snapshot.captures.size() > kWorldTerrainSnapshotMaxCaptures ||
        snapshot.metadata.aggregate_epoch == 0 ||
        !std::isfinite(snapshot.metadata.state_time_s))
        return false;
    for (std::size_t i = 0; i < snapshot.captures.size(); ++i)
    {
        const WorldTerrainSnapshotEntry &entry = snapshot.captures[i];
        if (!CaptureHeadingEntryValid(
                entry, snapshot.metadata.aggregate_epoch,
                snapshot.metadata.state_time_s))
            return false;
        if (i != 0 &&
            (entry.model.map_sequence ==
                 snapshot.captures[i - 1].model.map_sequence ||
             !EntryBefore(snapshot.captures[i - 1], entry)))
            return false;
    }
    return true;
}

inline bool FiniteCellGeometry(const TerrainCell &cell)
{
    if (!cell.known || !std::isfinite(cell.height_m) ||
        !std::isfinite(cell.slope_rad) || !std::isfinite(cell.roughness_m) ||
        !std::isfinite(cell.variance_m2))
        return false;
    const double low = cell.has_height_bounds
        ? cell.height_min_m : cell.height_m;
    const double high = cell.has_height_bounds
        ? cell.height_max_m : cell.height_m;
    const double normal_norm = std::hypot(
        std::hypot(cell.normal[0], cell.normal[1]), cell.normal[2]);
    return std::isfinite(low) && std::isfinite(high) && low <= high &&
        std::isfinite(normal_norm) && normal_norm > 1.0e-9 &&
        std::abs(normal_norm - 1.0) <= 1.0e-5;
}
inline bool EffectiveMaxAge(const WorldTerrainSnapshot &snapshot,
                            const TerrainModel &model, double max_age_s,
                            double &effective_max_age_s)
{
    const double state_delta = snapshot.metadata.state_time_s - model.state_stamp_s;
    if (!std::isfinite(state_delta) ||
        state_delta < -kTerrainMapTimeToleranceS ||
        !std::isfinite(max_age_s) || max_age_s < 0.0)
        return false;
    effective_max_age_s = max_age_s - std::max(0.0, state_delta);
    return std::isfinite(effective_max_age_s) &&
        effective_max_age_s >= -kTerrainMapTimeToleranceS;
}
inline bool FreshAt(const TerrainCell &cell, double state_delta,
                    double max_age_s)
{
    if (!std::isfinite(cell.age_s) ||
        cell.age_s < -kTerrainMapTimeToleranceS)
        return false;
    const double age_now = cell.age_s + state_delta;
    return std::isfinite(age_now) &&
        age_now <= max_age_s + kTerrainMapTimeToleranceS;
}
struct SnapshotCellEvidence
{
    std::size_t ix = 0;
    std::size_t iy = 0;
    std::array<double, 3> center_world{
        kTerrainMapUnknown, kTerrainMapUnknown, kTerrainMapUnknown};
    std::array<double, 3> normal_world{0.0, 0.0, 1.0};
    double height_world = kTerrainMapUnknown;
    double min_height_world = kTerrainMapUnknown;
    double max_height_world = kTerrainMapUnknown;
};
struct SnapshotModelEvidence
{
    WorldTerrainQueryFailure failure = WorldTerrainQueryFailure::kMetadata;
    WorldTerrainQueryDiagnostic diagnostic{};
    TerrainPatch patch{};
    std::vector<SnapshotCellEvidence> cells;
    double cell_age_max_s = kTerrainMapUnknown;
    std::size_t total_cells = 0;
    std::size_t fresh_known_cells = 0;
    bool complete = false;
    bool covers_patch = false;
    bool any_fresh_known = false;
};
inline bool LocalToWorldCellEvidence(
    const TerrainModel &model, std::size_t ix, std::size_t iy,
    SnapshotCellEvidence &evidence)
{
    const TerrainCell *cell = model.CellAt(ix, iy);
    if (cell == nullptr || !FiniteCellGeometry(*cell) ||
        !std::isfinite(model.registration_position_world[2]))
        return false;
    const TerrainMapXY local_center{
        model.origin_m[0] + (static_cast<double>(ix) + 0.5) *
            model.resolution_m,
        model.origin_m[1] + (static_cast<double>(iy) + 0.5) *
            model.resolution_m};
    const TerrainMapXY world_center = TerrainMapToWorld(
        model.registration_position_world, model.registration_yaw_rad,
        local_center);
    const double c = std::cos(model.registration_yaw_rad);
    const double s = std::sin(model.registration_yaw_rad);
    const std::array<double, 3> normal{
        c * cell->normal[0] - s * cell->normal[1],
        s * cell->normal[0] + c * cell->normal[1], cell->normal[2]};
    const double normal_norm = std::hypot(
        std::hypot(normal[0], normal[1]), normal[2]);
    if (!std::isfinite(world_center.x) || !std::isfinite(world_center.y) ||
        !std::isfinite(normal_norm) || normal_norm <= 1.0e-9 ||
        std::abs(normal_norm - 1.0) > 1.0e-5 ||
        std::abs(normal[2]) < kWorldTerrainSnapshotMinimumNormalZ)
        return false;
    evidence.ix = ix;
    evidence.iy = iy;
    evidence.center_world = {
        world_center.x, world_center.y,
        cell->height_m + model.registration_position_world[2]};
    evidence.normal_world = {
        normal[0] / normal_norm, normal[1] / normal_norm,
        normal[2] / normal_norm};
    evidence.height_world = evidence.center_world[2];
    evidence.min_height_world = (cell->has_height_bounds
        ? cell->height_min_m : cell->height_m) +
        model.registration_position_world[2];
    evidence.max_height_world = (cell->has_height_bounds
        ? cell->height_max_m : cell->height_m) +
        model.registration_position_world[2];
    return std::isfinite(evidence.center_world[2]) &&
        std::isfinite(evidence.min_height_world) &&
        std::isfinite(evidence.max_height_world);
}
inline const SnapshotCellEvidence *FindEvidenceCell(
    const SnapshotModelEvidence &evidence, std::size_t ix, std::size_t iy)
{
    for (const SnapshotCellEvidence &cell : evidence.cells)
    {
        if (cell.ix == ix && cell.iy == iy)
            return &cell;
    }
    return nullptr;
}
inline SnapshotModelEvidence InspectModel(
    const WorldTerrainSnapshot &snapshot, const TerrainModel &model,
    double world_x, double world_y, double radius_m, double max_age_s)
{
    SnapshotModelEvidence evidence;
    evidence.diagnostic.world_x = world_x;
    evidence.diagnostic.world_y = world_y;
    evidence.diagnostic.radius_m = radius_m;
    evidence.diagnostic.max_cell_age_s = max_age_s;
    if (!WorldTerrainMetadataValid(model))
    {
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    double effective_max_age_s = 0.0;
    if (!EffectiveMaxAge(snapshot, model, max_age_s, effective_max_age_s))
    {
        evidence.failure = WorldTerrainQueryFailure::kStaleCells;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    double local_x = 0.0, local_y = 0.0, z_offset = 0.0;
    if (!world_terrain_view_detail::WorldToLocalXY(
            model, world_x, world_y, local_x, local_y, z_offset))
    {
        evidence.failure = WorldTerrainQueryFailure::kInvalidQuery;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    evidence.diagnostic.local_x = local_x;
    evidence.diagnostic.local_y = local_y;
    const double r = std::max(radius_m, 0.5 * model.resolution_m);
    if (!std::isfinite(r))
    {
        evidence.failure = WorldTerrainQueryFailure::kInvalidQuery;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    evidence.covers_patch = model.CoversPatch(local_x, local_y, r);
    evidence.diagnostic.patch_outside = evidence.covers_patch ? 0 : 1;
    const double query_min_x = local_x - r;
    const double query_max_x = local_x + r;
    const double query_min_y = local_y - r;
    const double query_max_y = local_y + r;
    const double map_min_x = model.origin_m[0];
    const double map_min_y = model.origin_m[1];
    const double map_max_x = map_min_x +
        static_cast<double>(model.width) * model.resolution_m;
    const double map_max_y = map_min_y +
        static_cast<double>(model.height) * model.resolution_m;
    if (!std::isfinite(query_min_x) || !std::isfinite(query_max_x) ||
        !std::isfinite(query_min_y) || !std::isfinite(query_max_y) ||
        !std::isfinite(map_max_x) || !std::isfinite(map_max_y))
    {
        evidence.failure = WorldTerrainQueryFailure::kInvalidQuery;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    if (query_max_x <= map_min_x || query_min_x >= map_max_x ||
        query_max_y <= map_min_y || query_min_y >= map_max_y)
    {
        evidence.failure = WorldTerrainQueryFailure::kOutside;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    const double overlap_max_x = std::min(
        query_max_x, std::nextafter(
            map_max_x, -std::numeric_limits<double>::infinity()));
    const double overlap_max_y = std::min(
        query_max_y, std::nextafter(
            map_max_y, -std::numeric_limits<double>::infinity()));
    const double overlap_min_x = std::max(query_min_x, map_min_x);
    const double overlap_min_y = std::max(query_min_y, map_min_y);
    if (!std::isfinite(overlap_min_x) || !std::isfinite(overlap_max_x) ||
        !std::isfinite(overlap_min_y) || !std::isfinite(overlap_max_y) ||
        overlap_min_x > overlap_max_x || overlap_min_y > overlap_max_y)
    {
        evidence.failure = WorldTerrainQueryFailure::kOutside;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    std::size_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    if (!model.CellIndex(overlap_min_x, overlap_min_y, x0, y0) ||
        !model.CellIndex(overlap_max_x, overlap_max_y, x1, y1) ||
        x1 < x0 || y1 < y0)
    {
        evidence.failure = WorldTerrainQueryFailure::kInvalidQuery;
        evidence.diagnostic.failure = evidence.failure;
        return evidence;
    }
    const double state_delta = snapshot.metadata.state_time_s - model.state_stamp_s;
    bool saw_unknown = false;
    bool saw_stale = false;
    bool saw_nonfinite = false;
    for (std::size_t iy = y0; iy <= y1; ++iy)
    {
        for (std::size_t ix = x0; ix <= x1; ++ix)
        {
            ++evidence.total_cells;
            const TerrainCell *cell = model.CellAt(ix, iy);
            if (cell == nullptr || !cell->known)
            {
                saw_unknown = true;
                continue;
            }
            const double age_now = std::isfinite(cell->age_s)
                ? cell->age_s + state_delta : kTerrainMapUnknown;
            if (std::isfinite(age_now))
                evidence.cell_age_max_s = std::isfinite(evidence.cell_age_max_s)
                    ? std::max(evidence.cell_age_max_s, age_now) : age_now;
            if (!FreshAt(*cell, state_delta, max_age_s))
            {
                saw_stale = true;
                continue;
            }
            if (!FiniteCellGeometry(*cell))
            {
                saw_nonfinite = true;
                continue;
            }
            SnapshotCellEvidence cell_evidence;
            if (!LocalToWorldCellEvidence(model, ix, iy, cell_evidence))
            {
                saw_nonfinite = true;
                continue;
            }
            evidence.cells.push_back(cell_evidence);
            ++evidence.fresh_known_cells;
        }
    }
    evidence.any_fresh_known = evidence.fresh_known_cells != 0;
    evidence.complete = evidence.covers_patch && !saw_unknown && !saw_stale && !saw_nonfinite &&
        evidence.total_cells != 0 &&
        evidence.fresh_known_cells == evidence.total_cells;
    evidence.diagnostic.patch_known = evidence.fresh_known_cells;
    evidence.diagnostic.patch_total = evidence.total_cells;
    evidence.diagnostic.cell_age_max_s = evidence.cell_age_max_s;
    if (!evidence.covers_patch)
        evidence.failure = WorldTerrainQueryFailure::kOutside;
    else if (saw_nonfinite)
        evidence.failure = WorldTerrainQueryFailure::kNonfinitePatch;
    else if (saw_unknown)
        evidence.failure = WorldTerrainQueryFailure::kUnknownCells;
    else if (saw_stale)
        evidence.failure = WorldTerrainQueryFailure::kStaleCells;
    else if (!evidence.complete)
        evidence.failure = WorldTerrainQueryFailure::kNonfinitePatch;
    if (!evidence.complete)
        evidence.diagnostic.failure = evidence.failure;
    else
    {
        WorldTerrainQueryDiagnostic diagnostic;
        if (!SampleWorldTerrainPatch(model, world_x, world_y, radius_m,
                                     effective_max_age_s, evidence.patch,
                                     &diagnostic))
        {
            evidence.failure = diagnostic.failure;
            evidence.diagnostic = diagnostic;
            evidence.diagnostic.cell_age_max_s = evidence.cell_age_max_s;
            evidence.complete = false;
        }
        else
        {
            evidence.failure = WorldTerrainQueryFailure::kNone;
            evidence.diagnostic = diagnostic;
            evidence.diagnostic.cell_age_max_s = evidence.cell_age_max_s;
        }
    }
    return evidence;
}
inline double PlaneHeightAt(const SnapshotCellEvidence &cell,
                            double world_x, double world_y)
{
    const double nz = cell.normal_world[2];
    if (!std::isfinite(nz) || std::abs(nz) <
            kWorldTerrainSnapshotMinimumNormalZ)
        return kTerrainMapUnknown;
    const double dzdx = -cell.normal_world[0] / nz;
    const double dzdy = -cell.normal_world[1] / nz;
    const double value = cell.height_world +
        dzdx * (world_x - cell.center_world[0]) +
        dzdy * (world_y - cell.center_world[1]);
    return std::isfinite(value) ? value : kTerrainMapUnknown;
}
inline bool HeightIntervalsConflict(const SnapshotCellEvidence &newer,
                                    const SnapshotCellEvidence &older,
                                    double common_x, double common_y,
                                    double tolerance_m, double &height_gap_m)
{
    const double newer_center = PlaneHeightAt(
        newer, common_x, common_y);
    const double older_center = PlaneHeightAt(
        older, common_x, common_y);
    height_gap_m = std::isfinite(newer_center) && std::isfinite(older_center)
        ? std::abs(newer_center - older_center) : kTerrainMapUnknown;
    if (!std::isfinite(newer_center) || !std::isfinite(older_center))
        return false;
    const double new_min = newer.min_height_world +
        (newer_center - newer.height_world);
    const double new_max = newer.max_height_world +
        (newer_center - newer.height_world);
    const double old_min = older.min_height_world +
        (older_center - older.height_world);
    const double old_max = older.max_height_world +
        (older_center - older.height_world);
    if (!std::isfinite(new_min) || !std::isfinite(new_max) ||
        !std::isfinite(old_min) || !std::isfinite(old_max) ||
        new_min > new_max || old_min > old_max)
        return false;
    const bool disjoint = new_max < old_min - tolerance_m ||
        old_max < new_min - tolerance_m;
    return disjoint && std::isfinite(height_gap_m) &&
        height_gap_m > tolerance_m;
}
struct WorldTerrainSnapshotConflictWitness
{
    double height_gap_m = kTerrainMapUnknown;
    double normal_dot = kTerrainMapUnknown;
};
inline bool ConflictBetween(const SnapshotModelEvidence &newer,
                            const SnapshotModelEvidence &older,
                            double height_tolerance_m, double minimum_normal_dot,
                            WorldTerrainSnapshotConflictWitness &witness)
{
    // Compare every fresh cell pair in this common query footprint.  A
    // capture-heading patch is treated as one locally planar support patch;
    // this avoids missing shifted-grid partial evidence at cell boundaries.
    for (const SnapshotCellEvidence &newer_cell : newer.cells)
    {
        for (const SnapshotCellEvidence &older_cell : older.cells)
        {
            const double common_x = 0.5 *
                (newer_cell.center_world[0] + older_cell.center_world[0]);
            const double common_y = 0.5 *
                (newer_cell.center_world[1] + older_cell.center_world[1]);
            const double normal_dot = newer_cell.normal_world[0] *
                    older_cell.normal_world[0] +
                newer_cell.normal_world[1] * older_cell.normal_world[1] +
                newer_cell.normal_world[2] * older_cell.normal_world[2];
            double height_gap_m = kTerrainMapUnknown;
            const bool height_conflict = HeightIntervalsConflict(
                newer_cell, older_cell, common_x, common_y,
                height_tolerance_m, height_gap_m);
            if (!std::isfinite(normal_dot) ||
                normal_dot < minimum_normal_dot || height_conflict)
            {
                witness.height_gap_m = height_gap_m;
                witness.normal_dot = normal_dot;
                return true;
            }
        }
    }
    return false;
}
inline WorldTerrainSnapshotError MapFailure(WorldTerrainQueryFailure failure)
{
    switch (failure)
    {
    case WorldTerrainQueryFailure::kOutside:
        return WorldTerrainSnapshotError::kOutside;
    case WorldTerrainQueryFailure::kUnknownCells:
        return WorldTerrainSnapshotError::kUnknownCells;
    case WorldTerrainQueryFailure::kStaleCells:
        return WorldTerrainSnapshotError::kStaleCells;
    case WorldTerrainQueryFailure::kNonfinitePatch:
        return WorldTerrainSnapshotError::kNonfinitePatch;
    case WorldTerrainQueryFailure::kInvalidQuery:
        return WorldTerrainSnapshotError::kQueryInvalid;
    case WorldTerrainQueryFailure::kMetadata:
        return WorldTerrainSnapshotError::kCaptureInvalid;
    default:
        return WorldTerrainSnapshotError::kNoFreshCompletePatch;
    }
}
} // namespace world_terrain_snapshot_detail
inline WorldTerrainSnapshotBuildResult BuildWorldTerrainSnapshot(
    std::uint64_t aggregate_epoch, double state_time_s,
    const std::vector<CaptureTerrainViewResult> &captures,
    WorldTerrainSnapshotOptions options = {})
{
    using namespace world_terrain_snapshot_detail;
    WorldTerrainSnapshotBuildResult result;
    if (aggregate_epoch == 0 || !std::isfinite(state_time_s) ||
        options.max_captures == 0 ||
        options.max_captures > kWorldTerrainSnapshotMaxCaptures ||
        !std::isfinite(options.height_conflict_tolerance_m) ||
        options.height_conflict_tolerance_m < 0.0 ||
        options.height_conflict_tolerance_m >
            kWorldTerrainSnapshotMaxHeightConflictToleranceM ||
        !std::isfinite(options.minimum_normal_dot) ||
        options.minimum_normal_dot < 0.0 || options.minimum_normal_dot > 1.0)
    {
        result.error = WorldTerrainSnapshotError::kInvalidInput;
        return result;
    }
    if (!options.stationary_terrain_assumption)
    {
        result.error = WorldTerrainSnapshotError::kStationaryAssumptionRequired;
        return result;
    }
    if (captures.empty())
    {
        result.error = WorldTerrainSnapshotError::kEmpty;
        return result;
    }
    if (captures.size() > options.max_captures)
    {
        result.error = WorldTerrainSnapshotError::kTooManyCaptures;
        return result;
    }
    WorldTerrainSnapshot snapshot;
    snapshot.metadata.aggregate_epoch = aggregate_epoch;
    snapshot.metadata.state_time_s = state_time_s;
    snapshot.metadata.stationary_terrain_assumption =
        options.stationary_terrain_assumption;
    snapshot.metadata.height_conflict_tolerance_m =
        options.height_conflict_tolerance_m;
    snapshot.metadata.minimum_normal_dot = options.minimum_normal_dot;
    snapshot.captures.reserve(captures.size());
    for (const CaptureTerrainViewResult &capture : captures)
    {
        if (!capture.ok() ||
            capture.scope != CaptureTerrainViewScope::kCaptureHeadingWorldQuery)
        {
            result.error = WorldTerrainSnapshotError::kCaptureInvalid;
            return result;
        }
        WorldTerrainSnapshotEntry entry;
        entry.model = capture.model;
        entry.provenance = capture.provenance;
        entry.scope = capture.scope;
        if (!CaptureHeadingEntryValid(entry, aggregate_epoch, state_time_s))
        {
            result.error = entry.model.state_stamp_s >
                    state_time_s + kTerrainMapTimeToleranceS
                ? WorldTerrainSnapshotError::kFutureCapture
                : WorldTerrainSnapshotError::kCaptureInvalid;
            return result;
        }
        snapshot.captures.push_back(std::move(entry));
    }
    std::stable_sort(snapshot.captures.begin(), snapshot.captures.end(),
                     EntryBefore);
    for (std::size_t i = 1; i < snapshot.captures.size(); ++i)
    {
        if (snapshot.captures[i - 1].model.map_sequence ==
            snapshot.captures[i].model.map_sequence)
        {
            result.error = WorldTerrainSnapshotError::kDuplicateSource;
            return result;
        }
    }
    snapshot.valid = true;
    result.snapshot = std::move(snapshot);
    result.error = WorldTerrainSnapshotError::kNone;
    result.valid = true;
    return result;
}
inline WorldTerrainSnapshotQueryResult SampleWorldTerrainSnapshot(
    const WorldTerrainSnapshot &snapshot, double world_x, double world_y,
    double radius_m, double max_cell_age_s)
{
    using namespace world_terrain_snapshot_detail;
    WorldTerrainSnapshotQueryResult result;
    result.stationary_terrain_assumption =
        snapshot.metadata.stationary_terrain_assumption;
    const bool snapshot_ok = snapshot.ok() && SnapshotEntriesValid(snapshot);
    if (!snapshot_ok || !std::isfinite(world_x) || !std::isfinite(world_y) ||
        !std::isfinite(radius_m) || radius_m < 0.0 ||
        !std::isfinite(max_cell_age_s) || max_cell_age_s < 0.0)
    {
        result.error = snapshot_ok ? WorldTerrainSnapshotError::kQueryInvalid
                                     : WorldTerrainSnapshotError::kInvalidInput;
        return result;
    }
    std::vector<SnapshotModelEvidence> evidence;
    evidence.reserve(snapshot.captures.size());
    for (const WorldTerrainSnapshotEntry &entry : snapshot.captures)
        evidence.push_back(InspectModel(snapshot, entry.model, world_x,
                                        world_y, radius_m, max_cell_age_s));
    result.latest_failure = evidence.front().failure;
    result.diagnostic = evidence.front().diagnostic;
    for (std::size_t newer_index = 0;
         newer_index < evidence.size(); ++newer_index)
    {
        if (!evidence[newer_index].any_fresh_known)
            continue;
        for (std::size_t older_index = newer_index + 1;
             older_index < evidence.size(); ++older_index)
        {
            if (!evidence[older_index].any_fresh_known)
                continue;
            WorldTerrainSnapshotConflictWitness witness;
            if (ConflictBetween(
                    evidence[newer_index],
                    evidence[older_index],
                    snapshot.metadata.height_conflict_tolerance_m,
                    snapshot.metadata.minimum_normal_dot, witness))
            {
                result.conflict_height_gap_m = witness.height_gap_m;
                result.conflict_normal_dot = witness.normal_dot;
                result.conflict_newer_sequence = snapshot.captures[
                    newer_index].model.map_sequence;
                result.conflict_older_sequence = snapshot.captures[
                    older_index].model.map_sequence;
                result.error = WorldTerrainSnapshotError::kConflictingHistory;
                result.diagnostic.failure =
                    WorldTerrainQueryFailure::kConflictingHistory;
                return result;
            }
        }
    }
    for (std::size_t index = 0; index < evidence.size(); ++index)
    {
        if (!evidence[index].complete)
            continue;
        result.patch = evidence[index].patch;
        result.diagnostic = evidence[index].diagnostic;
        result.selected_source_time_s =
            snapshot.captures[index].model.map_stamp_s;
        result.selected_source_time = TimeNs::FromSeconds(
            result.selected_source_time_s);
        result.selected_source_sequence =
            snapshot.captures[index].model.map_sequence;
        result.history_used = index != 0;
        result.error = WorldTerrainSnapshotError::kNone;
        result.valid = true;
        return result;
    }
    result.error = MapFailure(result.latest_failure);
    if (result.error == WorldTerrainSnapshotError::kNone)
        result.error = WorldTerrainSnapshotError::kNoFreshCompletePatch;
    return result;
}
} // namespace stage_c
} // namespace go2_terrain
