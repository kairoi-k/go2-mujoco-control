#include "terrain_feasibility.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
namespace
{
bool Check(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}
bool Close(double a, double b, double tolerance = 1.0e-10)
{
    return std::abs(a - b) <= tolerance;
}
go2_terrain::TerrainModel FlatModel()
{
    go2_terrain::TerrainModel model;
    model.frame_id = "base_link";
    model.state_stamp_s = 1.0;
    model.map_stamp_s = 1.0;
    model.age_s = 0.0;
    model.epoch = 7;
    model.resolution_m = 0.05;
    model.origin_m = {-1.0, -1.0};
    model.width = 40;
    model.height = 40;
    model.source = go2_terrain::TerrainSource::kTestFixture;
    model.cells.resize(model.width * model.height);
    for (auto &cell : model.cells)
    {
        cell.height_m = -0.448;
        cell.age_s = 0.0;
        cell.slope_rad = 0.0;
        cell.roughness_m = 0.0;
        cell.variance_m2 = 0.0;
        cell.normal = {0.0, 0.0, 1.0};
        cell.known = true;
    }
    return model;
}
go2_terrain::TerrainSwingContract MakeContract(
    const go2_terrain::TerrainSwingFrameAdapter &frame)
{
    const go2::Vec3 start_body{0.1934, -0.1420, -0.4260};
    // This is already the foot-site target in heading coordinates.  Do not
    // apply ContactPatchToFootSite here: the checker contract is world site.
    const go2::Vec3 target_site_heading{0.1934, -0.1400, -0.4260};
    go2_terrain::TerrainSwingContract contract;
    contract.map_epoch = 7;
    contract.plan_epoch = 11;
    contract.model_state_stamp_s = 1.0;
    contract.start_time_s = 1.0;
    contract.touchdown_time_s = 1.1;
    contract.start_source = go2_terrain::TerrainSwingStartSource::kCommanded;
    if (!frame.BodyToWorld(start_body, contract.start_world) ||
        !frame.HeadingToWorld(target_site_heading, contract.target_world))
        contract.start_time_s = std::numeric_limits<double>::quiet_NaN();
    return contract;
}
bool TouchdownTimeWitness()
{
    using namespace go2_terrain;
    double resolved = 0.0;
    if (!Check(ResolveTerrainSwingTouchdownTime(
                   10.0, 0.44, 0.14, 10.05, resolved),
               "coarse touchdown matches next phase event"))
        return false;
    if (!Check(Close(resolved, 10.0784),
               "touchdown resolves from current period and phase"))
        return false;
    if (!Check(!ResolveTerrainSwingTouchdownTime(
                   10.0, 0.44, 0.14, 10.2184, resolved) &&
                   !std::isfinite(resolved),
               "coarse touchdown from next cycle rejects"))
        return false;
    return Check(!ResolveTerrainSwingTouchdownTime(
                      10.0, 1.0, 0.14, 10.05, resolved) &&
                      !std::isfinite(resolved),
                  "invalid swing phase rejects");
}

bool LifecycleWitness()
{
    using namespace go2_terrain;
    TerrainSwingContract contract;
    contract.start_world = {1.0, -0.2, 0.5};
    contract.target_world = {1.25, -0.1, 0.55};
    contract.start_time_s = 4.0;
    contract.touchdown_time_s = 4.10;
    if (!Check(ApplyTerrainSwingClearanceResult(contract, 0.08, 0.25),
               "valid clearance result applies"))
        return false;
    go2::Vec3 start{};
    go2::Vec3 touchdown{};
    if (!Check(EvaluateTerrainSwingAtTime(
                   contract, 4.0, start, TerrainSwingEase,
                   TerrainSwingProfile), "start endpoint evaluates") ||
        !Check(EvaluateTerrainSwingAtTime(
                   contract, 4.10, touchdown, TerrainSwingEase,
                   TerrainSwingProfile), "touchdown endpoint evaluates") ||
        !Check(Close(start.x, contract.start_world.x) &&
                   Close(start.y, contract.start_world.y) &&
                   Close(start.z, contract.start_world.z),
               "start endpoint is exact") ||
        !Check(Close(touchdown.x, contract.target_world.x) &&
                   Close(touchdown.y, contract.target_world.y) &&
                   Close(touchdown.z, contract.target_world.z),
               "touchdown endpoint is exact"))
        return false;
    go2::Vec3 outside{};
    if (!Check(!EvaluateTerrainSwingAtTime(
                       contract, 3.999, outside, TerrainSwingEase,
                       TerrainSwingProfile) &&
                   !EvaluateTerrainSwingAtTime(
                       contract, 4.101, outside, TerrainSwingEase,
                       TerrainSwingProfile) &&
                   !EvaluateTerrainSwingAtTime(
                       contract, std::numeric_limits<double>::quiet_NaN(),
                       outside, TerrainSwingEase, TerrainSwingProfile),
               "outside and nonfinite time reject"))
        return false;
    TerrainSwingContract late;
    if (!Check(RebaseTerrainSwing(
                   contract, 4.05, {1.11, -0.16, 0.51},
                   TerrainSwingStartSource::kMeasured, late),
               "late latch rebases"))
        return false;
    return Check(!TerrainSwingResolved(late) &&
                     !EvaluateTerrainSwingAtTime(
                         late, 4.05, outside, TerrainSwingEase,
                         TerrainSwingProfile),
                 "late latch invalidates old clearance");
}
bool FrameWitness()
{
    using namespace go2_terrain;
    const double roll = 0.20;
    const double pitch = -0.15;
    const double yaw = 0.35;
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    const std::array<double, 4> q{
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr};
    TerrainSwingFrameAdapter frame;
    if (!Check(frame.Bind({0.4, -0.2, 0.7}, q, yaw),
               "full quaternion frame binds"))
        return false;
    if (!Check(!frame.Bind({0.4, -0.2, 0.7},
                           {0.0, 0.0, 0.0, 0.0}, yaw) && !frame.valid,
               "failed rebind clears frame validity"))
        return false;
    if (!Check(frame.Bind({0.4, -0.2, 0.7}, q, yaw),
               "frame rebind restores validity"))
        return false;
    const go2::Vec3 body{0.22, -0.11, -0.31};
    go2::Vec3 world{};
    go2::Vec3 round_trip_body{};
    go2::Vec3 heading{};
    go2::Vec3 round_trip_heading{};
    if (!Check(frame.BodyToWorld(body, world) &&
                   frame.WorldToBody(world, round_trip_body) &&
                   frame.BodyToHeading(body, heading) &&
                   frame.HeadingToBody(heading, round_trip_heading),
               "full H/B transforms evaluate") ||
        !Check(Close(round_trip_body.x, body.x) &&
                   Close(round_trip_body.y, body.y) &&
                   Close(round_trip_body.z, body.z),
               "full body round trip") ||
        !Check(Close(round_trip_heading.x, body.x) &&
                   Close(round_trip_heading.y, body.y) &&
                   Close(round_trip_heading.z, body.z),
               "full heading round trip"))
        return false;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const go2::Vec3 yaw_only{
        c * body.x + s * body.y, -s * body.x + c * body.y, body.z};
    const double error = std::hypot(
        std::hypot(heading.x - yaw_only.x, heading.y - yaw_only.y),
        heading.z - yaw_only.z);
    return Check(error > 0.16, "yaw-only transform differs under tilt");
}
bool ProductionCheckerWitness()
{
    using namespace go2_terrain;
    TerrainSwingFrameAdapter frame;
    if (!Check(frame.Bind(
                   {0.0, 0.0, 0.5}, {1.0, 0.0, 0.0, 0.0}, 0.0, 7, 1.0),
               "checker pose binds"))
        return false;
    auto model = FlatModel();
    auto contract = MakeContract(frame);
    double minimum_clearance = std::numeric_limits<double>::infinity();
    FootholdRejectReason reason = FootholdRejectReason::kNone;
    double lift = std::numeric_limits<double>::quiet_NaN();
    double peak = std::numeric_limits<double>::quiet_NaN();
    const bool accepted = CheckSwingClearanceWorld(
        model, frame, contract, 0.03, minimum_clearance, &reason,
        go2::Leg::FR, &lift, &peak);
    if (!Check(accepted, "production world checker accepts flat swing") ||
        !Check(std::isfinite(lift) && lift >= 0.0,
               "checker resolves finite lift") ||
        !Check(TerrainSwingPeakValid(peak), "checker resolves valid peak") ||
        !Check(TerrainSwingResolved(contract),
               "checker result is executable"))
        return false;
    auto bad_epoch = contract;
    bad_epoch.map_epoch = 8;
    const bool epoch_rejected = CheckSwingClearanceWorld(
        model, frame, bad_epoch, 0.03, minimum_clearance, &reason,
        go2::Leg::FR, &lift, &peak);
    if (!Check(!epoch_rejected &&
                   !std::isfinite(bad_epoch.resolved_lift_m) &&
                   !std::isfinite(bad_epoch.peak_phase),
               "epoch mismatch fails closed"))
        return false;
    auto unknown_model = FlatModel();
    std::size_t ix = 0;
    std::size_t iy = 0;
    if (!Check(unknown_model.CellIndex(0.1934, -0.1400, ix, iy),
               "unknown cell is in model"))
        return false;
    unknown_model.CellAt(ix, iy)->known = false;
    auto unknown_contract = MakeContract(frame);
    const bool unknown_rejected = CheckSwingClearanceWorld(
        unknown_model, frame, unknown_contract, 0.03, minimum_clearance,
        &reason, go2::Leg::FR, &lift, &peak);
    return Check(!unknown_rejected &&
                     !std::isfinite(unknown_contract.resolved_lift_m) &&
                     !std::isfinite(unknown_contract.peak_phase),
                 "unknown terrain fails closed");
}
} // namespace
int main()
{
    if (!TouchdownTimeWitness() || !LifecycleWitness() ||
        !FrameWitness() || !ProductionCheckerWitness())
        return 1;
    std::printf("terrain_swing_contract_production_passed\n");
    return 0;
}
