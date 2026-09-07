#include "stage_c/prepare_joint_problem.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace go2_terrain;
using namespace go2_terrain::stage_c;
using go2_control::RigidBodyPlanningKinematics;
using go2_control::SrbdMpcParams;
namespace
{
void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
TimeNs T(double seconds)
{
    return TimeNs::FromSeconds(seconds);
}
TimedPoint SurfacePoint(double x, double y, double z, TimeNs source)
{
    return {{x, y, z}, Frame::kWorld, source, true,
            PointRole::kSurfaceContactPoint};
}
TerrainPlanningInput Input()
{
    TerrainPlanningInput input;
    const TimeNs t0 = T(1.0);
    const TimeNs t1 = T(1.02);
    input.identity = {17, t0, 7, 3, 0};
    input.body.valid = true;
    input.body.model_com_valid = true;
    input.body.base_position_world =
        {{0.0, 0.0, 0.4}, Frame::kWorld, t0, true, PointRole::kBodyOrigin};
    input.body.model_com_world =
        {{0.0, 0.0, 0.4}, Frame::kWorld, t0, true,
         PointRole::kCenterOfMass};
    input.body.com_velocity_world = {0.0, 0.0, 0.0};
    input.body.mass_kg = 10.0;
    input.measured_contact.mask.fill(true);
    input.measured_contact.provenance = ContactProvenance::kMeasured;
    input.measured_contact.source_time = t0;
    input.measured_contact.valid = true;
    for (auto &foot : input.feet)
    {
        foot.measured_support_anchor_world = SurfacePoint(0.0, 0.0, 0.0, t0);
        foot.measured_support_anchor_valid = true;
    }
    input.map.metadata_valid = true;
    input.map.epoch = 7;
    input.map.coverage = MapCoverageState::kKnown;
    input.map.width = input.map.height = 8;
    input.map.total_cells = input.map.known_cells = 64;
    input.command.valid = true;
    input.command.command_epoch = 4;
    input.command.period_s = 0.24;
    input.command.duty_factor = 0.50;
    input.budget.prediction_start = t0;
    input.budget.prediction_end = t1;
    return input;
}
ContactSurface Surface(TimeNs end)
{
    ContactSurface surface;
    surface.frame = Frame::kWorld;
    surface.coverage = MapCoverageState::kKnown;
    surface.map_epoch = 7;
    surface.valid_until = end;
    surface.friction_mu = 0.8;
    surface.min_normal_n = 0.0;
    surface.max_normal_n = 180.0;
    return surface;
}
FixedSchedulePreview Preview()
{
    const TimeNs t0 = T(1.0);
    const TimeNs t1 = T(1.005);
    const TimeNs td = T(1.01);
    const TimeNs t2 = T(1.02);
    FixedSchedulePreview preview;
    preview.grid = {t0, t1, td, t2};
    TouchdownEvent event;
    event.id = {3, go2::Leg::FR, 1};
    event.liftoff_time = T(0.90);
    event.liftoff_valid = true;
    event.touchdown_time = td;
    event.contact_interval_end = t2;
    // The schedule owner deliberately leaves the future target unbound.
    preview.events.events.push_back(event);
    preview.intervals.push_back(
        {t0, t1, {{true, true, true, true}}, {{-1, -1, -1, -1}}});
    preview.intervals.push_back(
        {t1, td, {{false, true, true, true}}, {{-1, -1, -1, -1}}});
    preview.intervals.push_back(
        {td, t2, {{true, true, true, true}}, {{0, -1, -1, -1}}});
    preview.complete = true;
    return preview;
}
TerrainCandidateGenerationResult Candidates()
{
    const TimeNs t0 = T(1.0);
    const TimeNs t2 = T(1.02);
    TerrainCandidateGenerationResult result;
    TerrainCandidateSet set;
    set.event_set.event_id = {3, go2::Leg::FR, 1};
    set.event_set.complete = true;
    set.complete = true;
    for (std::size_t i = 0; i < 2; ++i)
    {
        const double x = 0.01 * static_cast<double>(i);
        const TimedPoint surface = SurfacePoint(x, 0.0, 0.0, t0);
        StageCCandidate candidate;
        candidate.candidate_id = static_cast<std::uint32_t>(i + 1);
        candidate.target_world = surface;
        candidate.foothold_cost = x;
        candidate.edge_margin_m = 0.1;
        candidate.reachability_margin_m = 0.1;
        candidate.coverage = MapCoverageState::kKnown;
        candidate.geometry_hard_feasible = true;
        set.event_set.candidates.push_back(candidate);
        TerrainCandidateSurfaceMatch match;
        match.surface_world = surface;
        match.sphere_center_world =
            {{x, 0.0, 0.022}, Frame::kWorld, t0, true,
             PointRole::kFootCollisionCenter};
        match.contact_surface = Surface(t2);
        match.patch.valid = true;
        match.patch.all_known = true;
        match.patch.known_cells = 1;
        match.patch.total_cells = 1;
        match.patch.center_height_m = 0.0;
        match.patch.min_height_m = 0.0;
        match.patch.max_height_m = 0.0;
        match.patch.slope_rad = 0.0;
        match.patch.roughness_m = 0.0;
        match.patch.variance_m2 = 0.0;
        match.patch.normal = {0.0, 0.0, 1.0};
        match.patch.map_edge_margin_m = 1.0;
        match.collision_radius_m = 0.022;
        set.matched_surfaces.push_back(match);
    }
    result.sets.push_back(set);
    result.failure = JointPlannerFailure::kNone;
    result.valid = true;
    return result;
}
FixedSchedulePreview TailPreview()
{
    FixedSchedulePreviewRequest request;
    request.start = T(1.0);
    request.end = T(1.30);
    request.phase_zero_time = T(0.0);
    request.period = T(0.24);
    request.max_interval = T(0.04);
    request.schedule_epoch = 7;
    request.duty = 0.80;
    request.leg_offsets.fill(0.0);
    return BuildFixedSchedulePreview(request);
}
TerrainModel TailTerrain()
{
    TerrainModel terrain;
    terrain.frame_id = "world";
    terrain.state_stamp_s = 1.0;
    terrain.map_stamp_s = 0.95;
    terrain.age_s = 0.05;
    terrain.epoch = 7;
    terrain.resolution_m = 0.1;
    terrain.origin_m = {-1.0, -1.0};
    terrain.width = 40;
    terrain.height = 40;
    terrain.source = TerrainSource::kTestFixture;
    terrain.registered = true;
    terrain.cells.assign(terrain.width * terrain.height, TerrainCell{});
    for (auto &cell : terrain.cells)
    {
        cell.height_m = -0.25;
        cell.height_min_m = -0.25;
        cell.height_max_m = -0.25;
        cell.has_height_bounds = true;
        cell.age_s = 0.05;
        cell.slope_rad = 0.0;
        cell.roughness_m = 0.0;
        cell.variance_m2 = 0.0;
        cell.normal = {0.0, 0.0, 1.0};
        cell.known = true;
    }
    return terrain;
}
TerrainCandidateReference TailReference()
{
    TerrainCandidateReference reference;
    reference.com_world_valid = true;
    reference.com_world = {0.40, 0.0, 0.50};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        reference.nominal_foot_center_offset_world[leg] =
            {leg == 0 ? 0.10 : -0.10, 0.0, -0.30};
        reference.nominal_offset_valid[leg] = true;
        reference.foot_radius_m[leg] = 0.022;
        reference.foot_radius_valid[leg] = true;
    }
    return reference;
}
RigidBodyPlanningKinematics ActualModel()
{
    RigidBodyPlanningKinematics model;
    model.valid = true;
    model.dynamics.valid = true;
    model.dynamics.mass_kg = 10.0;
    model.dynamics.com_world = Eigen::Vector3d(0.0, 0.0, 0.4);
    model.dynamics.inertia_com_world = Eigen::Matrix3d::Identity();
    model.angular_momentum_world = Eigen::Vector3d::Zero();
    model.com_velocity_world = Eigen::Vector3d::Zero();
    model.dynamics.foot_geometry_valid.fill(true);
    return model;
}
SrbdMpcParams Model()
{
    SrbdMpcParams model;
    model.mass_kg = 10.0;
    model.inertia_com_world = Eigen::Matrix3d::Identity();
    model.horizon = 2;
    model.dt_s = 0.01;
    return model;
}
std::vector<StateBox> Bounds(std::size_t count)
{
    return std::vector<StateBox>(count);
}
bool SameSchedule(const std::vector<FixedScheduleInterval> &left,
                  const std::vector<FixedScheduleInterval> &right)
{
    if (left.size() != right.size())
        return false;
    for (std::size_t i = 0; i < left.size(); ++i)
        if (left[i].start != right[i].start || left[i].end != right[i].end ||
            left[i].contact != right[i].contact ||
            left[i].event_index != right[i].event_index)
            return false;
    return true;
}
void TestContactContinuationAssembly()
{
    const auto preview = TailPreview();
    Check(preview.complete && !preview.events.events.empty(),
          "tail stance schedule preview failed");
    const TimeNs horizon = T(1.30);
    bool has_tail_stance = false;
    for (const auto &event : preview.events.events)
        has_tail_stance |= event.contact_interval_end > horizon;
    Check(has_tail_stance, "tail stance was not represented with its real end");
    auto input = Input();
    input.identity.schedule_epoch = 7;
    input.budget.prediction_end = horizon;
    input.command.duty_factor = 0.80;
    auto continuation_config = TerrainCandidateConfig{};
    continuation_config.allow_contact_continuation_beyond_horizon = true;
    const auto generated = GenerateTerrainCandidates(
        TailTerrain(), preview.events, T(1.0), 7, TailReference(), horizon,
        continuation_config);
    Check(generated.valid && generated.sets.size() == preview.events.events.size(),
          "tail continuation candidate generation failed");
    std::array<ContactSurface, go2::kLegCount> initial{};
    for (auto &surface : initial)
        surface = Surface(horizon);
    const auto prepared = PrepareJointProblem(
        input, preview, generated, initial, ActualModel(), Model(),
        std::vector<StateBox>(preview.grid.size()));
    Check(prepared.ok && prepared.problem.required_end == horizon &&
              prepared.problem.request.events.events[0].contact_interval_end >
                  horizon,
          "tail continuation did not assemble through PrepareJointProblem");
}
} // namespace
int main()
{
    try
    {
        const auto input = Input();
        const auto preview = Preview();
        const auto generated = Candidates();
        std::array<ContactSurface, go2::kLegCount> initial{};
        for (auto &surface : initial)
            surface = Surface(T(1.02));
        const auto actual = ActualModel();
        const auto model = Model();
        const auto bounds = Bounds(preview.grid.size());
        const auto prepared = PrepareJointProblem(
            input, preview, generated, initial, actual, model, bounds);
        Check(prepared.ok && prepared.failure == JointPlannerFailure::kNone,
              "joint problem preparation failed");
        Check(prepared.problem.request.events.events.size() == 1 &&
                  !prepared.problem.request.events.events[0].target_world.valid,
              "future event target was selected or fabricated");
        Check(prepared.problem.request.candidate_sets.size() == 1 &&
                  prepared.problem.request.candidate_sets[0].candidates.size() == 2 &&
                  prepared.problem.candidate_surfaces.size() == 1 &&
                  prepared.problem.candidate_surfaces[0].size() == 2,
              "candidate-specific transport shape changed");
        Check(prepared.problem.combination.empty() &&
                  prepared.problem.grid == preview.grid &&
                  SameSchedule(prepared.problem.schedule, preview.intervals) &&
                  prepared.problem.required_start == input.budget.prediction_start &&
                  prepared.problem.required_end == input.budget.prediction_end &&
                  prepared.problem.initial_momentum_world ==
                      actual.angular_momentum_world,
              "absolute problem fields were not copied exactly");
        auto stale_com = actual;
        stale_com.dynamics.com_world.z() += 1.0e-3;
        const auto com_conflict = PrepareJointProblem(
            input, preview, generated, initial, stale_com, model, bounds);
        Check(!com_conflict.ok &&
                  com_conflict.failure == JointPlannerFailure::kInvalidInput,
              "stale actual-model COM was accepted");
        auto stale_velocity = actual;
        stale_velocity.com_velocity_world.x() = 1.0e-3;
        const auto velocity_conflict = PrepareJointProblem(
            input, preview, generated, initial, stale_velocity, model, bounds);
        Check(!velocity_conflict.ok &&
                  velocity_conflict.failure == JointPlannerFailure::kInvalidInput,
              "stale actual-model COM velocity was accepted");
        auto selected = prepared.problem;
        selected.combination = {0};
        const auto unbound_result = SolveCentroidalSubproblem(selected);
        auto bound = selected;
        bound.request.events.events[0].target_world =
            bound.request.candidate_sets[0].candidates[0].target_world;
        const auto bound_result = SolveCentroidalSubproblem(bound);
        Check(unbound_result.failure == JointPlannerFailure::kNone &&
                  bound_result.failure == JointPlannerFailure::kNone &&
                  std::abs(unbound_result.cost - bound_result.cost) < 1.0e-9,
              "unbound event and bound candidate solve differ");
        auto committed_unbound = preview;
        committed_unbound.events.events[0].committed = true;
        const auto committed = PrepareJointProblem(
            input, committed_unbound, generated, initial, actual, model, bounds);
        Check(!committed.ok && committed.failure == JointPlannerFailure::kCoverageIncomplete,
              "committed unbound event was accepted");
        auto bad_surface = generated;
        bad_surface.sets[0].matched_surfaces[1].contact_surface.coverage =
            MapCoverageState::kUnknownInside;
        bad_surface.valid = false;
        bad_surface.failure = JointPlannerFailure::kCoverageIncomplete;
        const auto unknown = PrepareJointProblem(
            input, preview, bad_surface, initial, actual, model, bounds);
        Check(!unknown.ok && unknown.failure == JointPlannerFailure::kCoverageIncomplete,
              "unknown candidate surface was accepted");
        auto bad_initial = initial;
        // Valid through the first intervals but expired exactly before the
        // terminal interval, proving the final interval is checked.
        bad_initial[1].valid_until = TimeNs{T(1.02).value - 1};
        const auto initial_failure = PrepareJointProblem(
            input, preview, generated, bad_initial, actual, model, bounds);
        Check(!initial_failure.ok &&
                  initial_failure.failure == JointPlannerFailure::kCoverageIncomplete,
              "unknown initial surface was accepted");
        auto zero_events = Preview();
        zero_events.events.events.clear();
        zero_events.intervals = {
            {T(1.0), T(1.005), {{true, true, true, true}}, {{-1, -1, -1, -1}}},
            {T(1.005), T(1.01), {{true, true, true, true}}, {{-1, -1, -1, -1}}},
            {T(1.01), T(1.02), {{true, true, true, true}}, {{-1, -1, -1, -1}}}};
        auto zero_candidates = generated;
        zero_candidates.sets.clear();
        const auto zero = PrepareJointProblem(
            input, zero_events, zero_candidates, initial, actual, model,
            Bounds(zero_events.grid.size()));
        Check(zero.ok && zero.problem.request.events.events.empty() &&
                  zero.problem.request.candidate_sets.empty(),
              "zero-event complete stance was rejected");
        auto bad_times = preview;
        bad_times.grid[1] = T(1.004);
        const auto timing_failure = PrepareJointProblem(
            input, bad_times, generated, initial, actual, model, bounds);
        Check(!timing_failure.ok &&
                  timing_failure.failure == JointPlannerFailure::kCoverageIncomplete,
              "inconsistent absolute preview timing was accepted");
        TestContactContinuationAssembly();
        std::cout << "Stage C joint problem preparation checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
