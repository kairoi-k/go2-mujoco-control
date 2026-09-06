#include "terrain_feasibility.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
constexpr double kMapStamp = 1.0;
constexpr double kBaseZ = 0.5;
constexpr double kPatchHeight = -0.448;
constexpr double kSiteOffset = go2::kFootSiteToContactPatchOffsetM;

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Close(double a, double b, double tolerance = 1.0e-9)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a - b) <= tolerance;
}

go2_terrain::TerrainMapEnvelope MakeSource(
    std::uint32_t width = 20, std::uint32_t height = 20)
{
    go2_terrain::TerrainMapEnvelope source;
    source.sequence = 31;
    source.map_stamp_s = kMapStamp;
    source.frame_id = "base_link";
    source.resolution_m = 0.05;
    source.width = width;
    source.height = height;
    source.origin_m = {-0.5, -0.5};
    source.capture_position_world = {0.0, 0.0, kBaseZ};
    source.capture_yaw_rad = 0.0;
    const std::size_t count = static_cast<std::size_t>(source.width) *
        source.height;
    source.heights_m.assign(count, kPatchHeight);
    source.observation_stamp_s.assign(count, kMapStamp);
    return source;
}

bool MakeModel(go2_terrain::TerrainModel &model,
               std::uint32_t width = 20, std::uint32_t height = 20)
{
    const auto source = MakeSource(width, height);
    const auto registered = go2_terrain::RegisterTerrainMap(
        source, kMapStamp, {0.0, 0.0, kBaseZ}, 0.0);
    if (!Check(registered.ok(), "identity map registration"))
        return false;
    const auto built = go2_terrain::BuildRegisteredTerrainModel(
        &registered.map, kMapStamp, 7,
        go2_terrain::TerrainSource::kTestFixture);
    if (!Check(built.ok(), "registered model builds"))
        return false;
    model = built.model;
    return true;
}

go2_terrain::TerrainSwingFrameAdapter MakeFrame(
    const go2::Vec3 &base = {0.0, 0.0, kBaseZ},
    double yaw = 0.0)
{
    go2_terrain::TerrainSwingFrameAdapter frame;
    frame.Bind(base, {1.0, 0.0, 0.0, 0.0}, yaw, 7, kMapStamp);
    return frame;
}

go2_terrain::TerrainSwingContract GroundContract(
    const go2_terrain::TerrainSwingFrameAdapter &frame,
    double start_x = 0.1934, double target_x = 0.2334,
    double start_time = kMapStamp, double touchdown_time = 1.1)
{
    go2_terrain::TerrainSwingContract contract;
    contract.map_epoch = 7;
    contract.plan_epoch = 11;
    contract.model_state_stamp_s = kMapStamp;
    contract.start_time_s = start_time;
    contract.touchdown_time_s = touchdown_time;
    const go2::Vec3 start_body{start_x, -0.1300, -0.4260};
    const go2::Vec3 target_site_heading{target_x, -0.1300, -0.4260};
    if (!frame.BodyToWorld(start_body, contract.start_world) ||
        !frame.HeadingToWorld(target_site_heading, contract.target_world))
        contract.start_time_s = std::numeric_limits<double>::quiet_NaN();
    return contract;
}

bool CheckWorldRejected(
    const go2_terrain::TerrainModel &model,
    const go2_terrain::TerrainSwingFrameAdapter &frame,
    go2_terrain::TerrainSwingContract contract,
    bool measured_support_anchor = false,
    go2_terrain::FootholdRejectReason *observed_reason = nullptr)
{
    double minimum = std::numeric_limits<double>::infinity();
    double lift = std::numeric_limits<double>::quiet_NaN();
    double peak = std::numeric_limits<double>::quiet_NaN();
    go2_terrain::FootholdRejectReason reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool accepted = go2_terrain::CheckSwingClearanceWorld(
        model, frame, contract, 0.03, minimum, &reason, go2::Leg::FR,
        &lift, &peak, nullptr, nullptr, measured_support_anchor);
    if (observed_reason != nullptr)
        *observed_reason = reason;
    return !accepted && !go2_terrain::TerrainSwingResolved(contract);
}

bool CheckWorldAccepted(
    const go2_terrain::TerrainModel &model,
    const go2_terrain::TerrainSwingFrameAdapter &frame,
    go2_terrain::TerrainSwingContract contract,
    bool validate_absolute_horizon)
{
    double minimum = std::numeric_limits<double>::infinity();
    double lift = std::numeric_limits<double>::quiet_NaN();
    double peak = std::numeric_limits<double>::quiet_NaN();
    go2_terrain::FootholdRejectReason reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool accepted = go2_terrain::CheckSwingClearanceWorld(
        model, frame, contract, 0.03, minimum, &reason, go2::Leg::FR,
        &lift, &peak, nullptr, nullptr, false, validate_absolute_horizon);
    const bool resolved = go2_terrain::TerrainSwingResolved(contract);
    if (!(accepted && resolved && std::isfinite(lift) && std::isfinite(peak)))
        std::fprintf(stderr, "normalized result accepted=%d resolved=%d reason=%d min=%.9f lift=%.9f peak=%.9f\n",
                     accepted ? 1 : 0, resolved ? 1 : 0,
                     static_cast<int>(reason), minimum, lift, peak);
    return accepted && resolved && std::isfinite(lift) && std::isfinite(peak);
}

bool WorldHeightWitness()
{
    const auto frame = MakeFrame();
    const auto contract = GroundContract(frame);
    go2::Vec3 heading{};
    if (!Check(frame.WorldToHeading(contract.start_world, heading),
               "world to heading is finite"))
        return false;
    const double ground_world_z = kBaseZ + kPatchHeight;
    const double start_patch_world_z =
        kBaseZ + heading.z - kSiteOffset;
    // This is the corrected grounded endpoint.  The old fixture passed
    // target_heading.z=-.426 through ContactPatchToFootSite, yielding a
    // patch at world z=.074, 22 mm above the map ground z=.052.
    const double old_patch_world_z = kBaseZ - 0.426;
    std::printf("world_height ground=%.6f start_patch=%.6f old_patch=%.6f\n",
                ground_world_z, start_patch_world_z, old_patch_world_z);
    return Check(Close(ground_world_z, 0.052),
                 "map local height converts to world ground") &&
        Check(Close(start_patch_world_z, ground_world_z),
              "site endpoint converts to grounded patch") &&
        Check(Close(old_patch_world_z - ground_world_z, 0.022),
              "old fixture is a 22 mm floating patch");
}

bool RegisteredPoseMismatchWitness()
{
    go2_terrain::TerrainModel model;
    if (!MakeModel(model))
        return false;
    const auto good_frame = MakeFrame();
    const auto bad_frame = MakeFrame({0.01, 0.0, kBaseZ}, 0.0);
    return CheckWorldRejected(model, bad_frame, GroundContract(good_frame));
}

bool TouchdownHorizonWitness()
{
    go2_terrain::TerrainModel model;
    if (!MakeModel(model))
        return false;
    const auto frame = MakeFrame();
    // A plan/map stamped at 1.0 cannot certify a touchdown beyond the 200 ms
    // horizon even when every cell is known and the pose is unchanged.
    const auto expired = GroundContract(frame, 0.1934, 0.2334,
                                         1.10, 1.201);
    const bool absolute_rejected = CheckWorldRejected(model, frame, expired);
    const bool absolute_screen_passes = CheckWorldAccepted(
        model, frame, GroundContract(frame), true);
    const bool normalized_screen_passes = CheckWorldAccepted(
        model, frame, expired, false);
    return Check(absolute_screen_passes,
                 "normal absolute horizon passes") &&
        Check(absolute_rejected,
              "absolute touchdown horizon rejects expired map") &&
        Check(normalized_screen_passes,
              "normalized geometry screen is explicitly non-certificate");
}

bool UnknownPathAndAnchorWitness()
{
    go2_terrain::TerrainModel model;
    if (!MakeModel(model))
        return false;
    std::size_t ix = 0, iy = 0;
    if (!Check(model.CellIndex(0.2334, -0.1300, ix, iy),
               "path cell is in model"))
        return false;
    model.CellAt(ix, iy)->known = false;
    const auto frame = MakeFrame();
    go2_terrain::FootholdRejectReason path_reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool path_rejected = CheckWorldRejected(
        model, frame, GroundContract(frame), false, &path_reason);

    go2_terrain::TerrainModel anchor_model;
    if (!MakeModel(anchor_model))
        return false;
    if (!Check(anchor_model.CellIndex(0.1934, -0.1300, ix, iy),
               "anchor cell is in model"))
        return false;
    anchor_model.CellAt(ix, iy)->known = false;
    go2_terrain::FootholdRejectReason anchor_reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool anchor_rejected = CheckWorldRejected(
        anchor_model, frame, GroundContract(frame), true, &anchor_reason);
    return Check(path_rejected &&
                     path_reason == go2_terrain::FootholdRejectReason::kUnknown,
                 "unknown path cell rejects as unknown") &&
        Check(anchor_rejected &&
                  anchor_reason == go2_terrain::FootholdRejectReason::kUnknown,
              "measured support anchor cannot fill unknown world patch");
}

bool PerCellAgeWitness()
{
    go2_terrain::TerrainModel model;
    if (!MakeModel(model))
        return false;
    std::size_t ix = 0, iy = 0;
    if (!Check(model.CellIndex(0.2334, -0.1300, ix, iy),
               "aged path cell is in model"))
        return false;
    model.CellAt(ix, iy)->age_s = 0.19;
    const auto frame = MakeFrame();
    go2_terrain::FootholdRejectReason reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool rejected = CheckWorldRejected(
        model, frame, GroundContract(frame), false, &reason);
    return Check(rejected &&
                     reason == go2_terrain::FootholdRejectReason::kUnknown,
                 "cell age at touchdown rejects as unknown");
}

bool UnderMapEndpointWitness()
{
    go2_terrain::TerrainModel model;
    if (!MakeModel(model))
        return false;
    const auto frame = MakeFrame();
    auto penetrating = GroundContract(frame);
    // Use a small lateral offset so the 5 mm penetration remains IK-valid;
    // the checker must reject the known map geometry, not reachability.
    if (!frame.BodyToWorld({0.1934, -0.1000, -0.4310},
                           penetrating.start_world) ||
        !frame.HeadingToWorld({0.2334, -0.1000, -0.4310},
                              penetrating.target_world))
        return false;
    go2_terrain::FootholdRejectReason reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool rejected = CheckWorldRejected(
        model, frame, penetrating, false, &reason);
    return Check(rejected &&
                     reason == go2_terrain::FootholdRejectReason::kSwingClearance,
                 "known map 5 mm endpoint penetration rejects");
}

bool KnownFringeWitness()
{
    go2_terrain::TerrainModel model;
    // x=.2334 remains reachable, while a 25 mm patch crosses the right
    // grid boundary at origin+.75=.25.
    if (!MakeModel(model, 15, 20))
        return false;
    const auto frame = MakeFrame();
    const auto fringe = GroundContract(frame, 0.1934, 0.2334);
    go2_terrain::FootholdRejectReason reason =
        go2_terrain::FootholdRejectReason::kNone;
    const bool rejected = CheckWorldRejected(
        model, frame, fringe, false, &reason);
    return Check(rejected &&
                     reason == go2_terrain::FootholdRejectReason::kUnknown,
                 "known fringe rejects as unknown");
}
}  // namespace

int main()
{
    if (!WorldHeightWitness() || !RegisteredPoseMismatchWitness() ||
        !TouchdownHorizonWitness() || !UnknownPathAndAnchorWitness() ||
        !KnownFringeWitness())
        return 1;
    std::printf("terrain_swing_world_audit_passed\n");
    return 0;
}