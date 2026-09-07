#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include "terrain_map_envelope.h"
#include "terrain_model.h"
namespace go2_terrain
{
namespace stage_c
{
enum class CaptureTerrainViewScope : std::uint8_t
{
    kCaptureHeadingWorldQuery = 1,
};
enum class CaptureTerrainViewError : std::uint8_t
{
    kNone = 0,
    kInvalidInput,
    kInvalidSource,
    kStaleSource,
    kRegistrationRejected,
    kModelBuildRejected,
    kCoverageChanged,
};
inline const char *CaptureTerrainViewErrorName(CaptureTerrainViewError error)
{
    switch (error)
    {
    case CaptureTerrainViewError::kNone: return "none";
    case CaptureTerrainViewError::kInvalidInput: return "invalid_input";
    case CaptureTerrainViewError::kInvalidSource: return "invalid_source";
    case CaptureTerrainViewError::kStaleSource: return "stale_source";
    case CaptureTerrainViewError::kRegistrationRejected:
        return "registration_rejected";
    case CaptureTerrainViewError::kModelBuildRejected:
        return "model_build_rejected";
    case CaptureTerrainViewError::kCoverageChanged: return "coverage_changed";
    default: return "unknown";
    }
}
struct CaptureTerrainViewProvenance
{
    std::uint64_t source_sequence = 0;
    std::uint64_t map_epoch = 0;
    double source_map_stamp_s = kTerrainMapUnknown;
    double state_stamp_s = kTerrainMapUnknown;
    std::array<double, 3> capture_position_world{
        kTerrainMapUnknown, kTerrainMapUnknown, kTerrainMapUnknown};
    double capture_yaw_rad = kTerrainMapUnknown;
    TerrainSource source = TerrainSource::kNone;
};
struct CaptureTerrainViewResult
{
    TerrainModel model{};
    CaptureTerrainViewScope scope =
        CaptureTerrainViewScope::kCaptureHeadingWorldQuery;
    CaptureTerrainViewProvenance provenance{};
    CaptureTerrainViewError error = CaptureTerrainViewError::kInvalidInput;
    TerrainMapRegistrationError registration_error =
        TerrainMapRegistrationError::kNone;
    TerrainModelError model_error = TerrainModelError::kNone;
    std::size_t source_known_cells = 0;
    std::size_t view_known_cells = 0;
    bool coverage_preserved = false;
    bool valid = false;

    bool ok() const
    {
        return valid && error == CaptureTerrainViewError::kNone &&
            coverage_preserved;
    }
};
namespace capture_terrain_view_detail
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
inline std::size_t CountKnownSourceCells(const TerrainMapEnvelope &source)
{
    return static_cast<std::size_t>(std::count_if(
        source.heights_m.begin(), source.heights_m.end(),
        [](double height) { return std::isfinite(height); }));
}
inline bool HeightPreserved(double source_height, double view_height)
{
    if (!std::isfinite(source_height) || !std::isfinite(view_height))
        return false;
    const double tolerance = 4.0 * std::numeric_limits<float>::epsilon() *
        std::max(1.0, std::abs(source_height));
    return std::abs(source_height - view_height) <= tolerance;
}
inline bool PreserveCaptureCells(
    const TerrainMapEnvelope &source, double state_stamp_s,
    const TerrainModel &view, std::size_t &view_known_cells)
{
    view_known_cells = 0;
    if (view.width != source.width || view.height != source.height ||
        view.cells.size() != source.heights_m.size())
        return false;
    bool preserved = true;
    for (std::size_t i = 0; i < source.heights_m.size(); ++i)
    {
        const bool source_known = std::isfinite(source.heights_m[i]);
        const TerrainCell &cell = view.cells[i];
        if (cell.known)
            ++view_known_cells;
        if (cell.known != source_known)
        {
            preserved = false;
            continue;
        }
        if (!source_known)
            continue;
        const double expected_age = state_stamp_s -
            source.observation_stamp_s[i];
        if (!HeightPreserved(source.heights_m[i], cell.height_m) ||
            !cell.has_height_bounds ||
            !std::isfinite(cell.height_min_m) ||
            !std::isfinite(cell.height_max_m) ||
            std::abs(cell.height_min_m - source.heights_m[i]) >
                kTerrainMapHeightToleranceM ||
            std::abs(cell.height_max_m - source.heights_m[i]) >
                kTerrainMapHeightToleranceM ||
            !std::isfinite(cell.age_s) ||
            std::abs(cell.age_s - expected_age) > kTerrainMapTimeToleranceS)
            preserved = false;
    }
    return preserved;
}
inline CaptureTerrainViewError MapRegistrationError(
    TerrainMapRegistrationError error)
{
    if (error == TerrainMapRegistrationError::kStaleMap ||
        error == TerrainMapRegistrationError::kInvalidCellAge)
        return CaptureTerrainViewError::kStaleSource;
    return CaptureTerrainViewError::kRegistrationRejected;
}
} // namespace capture_terrain_view_detail
// Build a world-query view whose registration pose is the source capture pose.
// There is deliberately no current-body pose argument: identity registration
// avoids re-rasterising a capture-heading grid into a moving body frame.
inline CaptureTerrainViewResult BuildCaptureHeadingTerrainView(
    const TerrainMapEnvelope &source, double state_stamp_s,
    std::uint64_t map_epoch, TerrainSource terrain_source,
    double max_age_s = kTerrainMapMaxAgeS)
{
    using namespace capture_terrain_view_detail;
    CaptureTerrainViewResult result;
    result.provenance.source_sequence = source.sequence;
    result.provenance.map_epoch = map_epoch;
    result.provenance.source_map_stamp_s = source.map_stamp_s;
    result.provenance.state_stamp_s = state_stamp_s;
    result.provenance.capture_position_world = source.capture_position_world;
    result.provenance.capture_yaw_rad = source.capture_yaw_rad;
    result.provenance.source = terrain_source;
    if (map_epoch == 0 || !KnownTerrainSource(terrain_source) ||
        !std::isfinite(state_stamp_s) || !std::isfinite(max_age_s) ||
        max_age_s < 0.0)
    {
        result.error = CaptureTerrainViewError::kInvalidInput;
        return result;
    }
    if (!TerrainMapEnvelopeShapeValid(source))
    {
        result.error = CaptureTerrainViewError::kInvalidSource;
        return result;
    }
    result.source_known_cells = CountKnownSourceCells(source);
    // Reject stale finite source cells before the legacy registration helper
    // can turn them into unknown output cells. This keeps the new view's
    // coverage contract explicit without changing that helper's behavior.
    for (std::size_t i = 0; i < source.heights_m.size(); ++i)
    {
        if (!std::isfinite(source.heights_m[i]))
            continue;
        const double age = state_stamp_s - source.observation_stamp_s[i];
        if (!std::isfinite(age) || age < -kTerrainMapTimeToleranceS ||
            age > max_age_s + kTerrainMapTimeToleranceS)
        {
            result.registration_error =
                TerrainMapRegistrationError::kInvalidCellAge;
            result.error = CaptureTerrainViewError::kStaleSource;
            return result;
        }
    }
    // Identity registration is the production provenance-preserving seam. It
    // still runs the existing bounded V2 checks, but never uses current body
    // pose and therefore does not move or resample the source grid.
    const auto registered = RegisterTerrainMap(
        source, state_stamp_s, source.capture_position_world,
        source.capture_yaw_rad, max_age_s,
        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    result.registration_error = registered.error;
    if (!registered.ok())
    {
        result.error = MapRegistrationError(registered.error);
        return result;
    }
    const auto built = BuildRegisteredTerrainModel(
        &registered.map, state_stamp_s, map_epoch, terrain_source);
    result.model_error = built.error;
    if (!built.ok())
    {
        result.error = CaptureTerrainViewError::kModelBuildRejected;
        return result;
    }
    result.view_known_cells = static_cast<std::size_t>(std::count_if(
        built.model.cells.begin(), built.model.cells.end(),
        [](const TerrainCell &cell) { return cell.known; }));
    result.coverage_preserved = PreserveCaptureCells(
        source, state_stamp_s, built.model, result.view_known_cells);
    if (!result.coverage_preserved ||
        result.source_known_cells != result.view_known_cells)
    {
        result.error = CaptureTerrainViewError::kCoverageChanged;
        return result;
    }
    result.model = built.model;
    result.error = CaptureTerrainViewError::kNone;
    result.valid = true;
    return result;
}
} // namespace stage_c
} // namespace go2_terrain
