#include "stage_c/terminal_swing_binding.h"
#include "stage_c/foot_trajectory.h"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace go2_terrain::stage_c;
namespace
{
void Check(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}
TimeNs T(double seconds)
{
    return TimeNs::FromSeconds(seconds);
}
TimedPoint Point(
    double x, double y, double z, PointRole role,
    TimeNs source_time = T(1.0))
{
    return {{x, y, z}, Frame::kWorld, source_time, true, role};
}
ContactSurface Surface(const Eigen::Matrix3d &basis = Eigen::Matrix3d::Identity())
{
    ContactSurface surface;
    surface.basis_world = basis;
    surface.frame = Frame::kWorld;
    surface.coverage = MapCoverageState::kKnown;
    surface.map_epoch = 7;
    surface.valid_until = T(2.0);
    surface.friction_mu = 0.8;
    surface.max_normal_n = 180.0;
    return surface;
}
CentroidalProblem Fixture()
{
    CentroidalProblem problem;
    auto &input = problem.request.input;
    input.identity = {11, T(1.0), 7, 3, 0};
    input.body.valid = true;
    input.body.model_com_valid = true;
    input.body.base_position_world =
        Point(0.0, 0.0, 0.42, PointRole::kBodyOrigin);
    input.body.model_com_world =
        Point(0.0, 0.0, 0.40, PointRole::kCenterOfMass);
    input.body.mass_kg = 10.0;
    input.measured_contact.mask.fill(true);
    input.measured_contact.provenance = ContactProvenance::kMeasured;
    input.measured_contact.source_time = T(1.0);
    input.measured_contact.valid = true;
    input.map.metadata_valid = true;
    input.map.epoch = 7;
    input.map.width = input.map.height = input.map.total_cells =
        input.map.known_cells = 8;
    problem.schedule_epoch = 3;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        input.feet[leg].foot_collision_center_world =
            Point(-0.25 + 0.16 * static_cast<double>(leg),
                  -0.12 + 0.08 * static_cast<double>(leg), 0.22,
                  PointRole::kFootCollisionCenter);
        input.feet[leg].measured_support_anchor_world =
            Point(-0.25 + 0.16 * static_cast<double>(leg),
                  -0.12 + 0.08 * static_cast<double>(leg), 0.0,
                  PointRole::kSurfaceContactPoint);
        input.feet[leg].measured_support_anchor_valid = true;
    }
    return problem;
}
FootTrajectoryRequest Request(CentroidalProblem &problem)
{
    FootTrajectoryRequest request;
    request.problem = &problem;
    request.start = T(1.0);
    request.end = T(1.5);
    request.swing_clearance_m = 0.03;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        request.collision_radius_m[leg] = 0.022;
        request.collision_radius_valid[leg] = true;
        request.initial_velocity_world[leg] = {0.0, 0.0, 0.0};
        request.initial_velocity_valid[leg] = true;
    }
    return request;
}
void AddInterval(
    CentroidalProblem &problem, double start, double end,
    const std::array<bool, 4> &contact,
    const std::array<int, 4> &event_index)
{
    problem.schedule.push_back({T(start), T(end), contact, event_index});
}
void AddEvent(
    CentroidalProblem &problem, std::size_t leg, std::uint32_t sequence,
    double liftoff, double touchdown, double contact_end,
    const TimedPoint &target, const ContactSurface &surface)
{
    TouchdownEvent event;
    event.id = {3, static_cast<go2::Leg>(leg), sequence};
    event.liftoff_time = T(liftoff);
    event.liftoff_valid = true;
    event.touchdown_time = T(touchdown);
    event.contact_interval_end = T(contact_end);
    event.target_world = target;
    problem.request.events.events.push_back(event);
    StageCCandidate candidate;
    candidate.candidate_id = sequence;
    candidate.target_world = target;
    candidate.coverage = MapCoverageState::kKnown;
    candidate.geometry_hard_feasible = true;
    problem.request.candidate_sets.push_back(
        {event.id, {candidate}, true});
    problem.request.input.feet[leg].contact_patch_world = target;
    problem.combination.push_back(0);
    problem.candidate_surfaces.push_back({surface});
}
Eigen::Vector3d EigenValue(const TimedPoint &point)
{
    return {point.value.x, point.value.y, point.value.z};
}
double Norm(const go2::Vec3 &value)
{
    return std::sqrt(value.x * value.x + value.y * value.y +
                     value.z * value.z);
}
double Difference(const go2::Vec3 &a, const go2::Vec3 &b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) +
                     (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}
void CheckCenter(
    const FootTrajectorySample &sample, std::size_t leg,
    const Eigen::Vector3d &expected, double tolerance, const char *message)
{
    Check(sample.valid && sample.leg_valid[leg], message);
    Check((EigenValue(sample.center_world[leg]) - expected).norm() <= tolerance,
          message);
}
bool SameTimedPoint(const TimedPoint &a, const TimedPoint &b)
{
    return a.valid == b.valid && a.role == b.role && a.frame == b.frame &&
        a.source_time == b.source_time &&
        (!a.valid || (a.value.x == b.value.x && a.value.y == b.value.y &&
                      a.value.z == b.value.z));
}
} // namespace
int main()
{
    try
    {
        auto problem = Fixture();
        const Eigen::Matrix3d tilted =
            Eigen::AngleAxisd(0.22, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const TimedPoint first_target =
            Point(0.02, -0.12, 0.0, PointRole::kSurfaceContactPoint);
        const TimedPoint in_flight_target =
            Point(0.08, 0.20, 0.0, PointRole::kSurfaceContactPoint);
        const TimedPoint second_target =
            Point(0.24, -0.12, 0.0, PointRole::kSurfaceContactPoint);
        AddEvent(problem, 1, 1, 0.90, 1.08, 1.50, in_flight_target,
                 Surface());
        AddEvent(problem, 0, 1, 1.05, 1.15, 1.30, first_target, Surface());
        AddEvent(problem, 0, 2, 1.30, 1.35, 1.50, second_target,
                 Surface(tilted));
        AddInterval(problem, 1.00, 1.05, {{true, false, true, true}},
                    {{-1, -1, -1, -1}});
        AddInterval(problem, 1.05, 1.08, {{false, false, true, true}},
                    {{-1, -1, -1, -1}});
        AddInterval(problem, 1.08, 1.15, {{false, true, true, true}},
                    {{-1, 0, -1, -1}});
        AddInterval(problem, 1.15, 1.30, {{true, true, true, true}},
                    {{1, 0, -1, -1}});
        AddInterval(problem, 1.30, 1.35, {{false, true, true, true}},
                    {{-1, 0, -1, -1}});
        AddInterval(problem, 1.35, 1.50, {{true, true, true, true}},
                    {{2, 0, -1, -1}});
        // Independent full-curve versus re-seeded tail equivalence, including
        // early ascent, apex and descent on a non-axis-aligned normal.
        {
            foot_trajectory_detail::PreparedEvent full;
            full.liftoff=T(0);full.interpolation_start=T(0);full.touchdown=T(1);
            full.p0=Eigen::Vector3d(.1,-.2,.3);full.p1=Eigen::Vector3d(.4,.1,.2);
            full.v0=Eigen::Vector3d(.2,.1,0);full.normal=Eigen::Vector3d(.2,0,1).normalized();
            for(double phase : {.05,.5,.9}) {
                auto tail=full;Eigen::Vector3d a;
                foot_trajectory_detail::EvaluateSwing(full,T(phase),.03,tail.p0,tail.v0,a);
                tail.interpolation_start=T(phase);
                const double remaining=1-phase;
                for(int step=0;step<=20;++step) {
                    const auto time=T(phase+remaining*step/20.);
                    Eigen::Vector3d p1,v1,a1,p2,v2,a2;
                    foot_trajectory_detail::EvaluateSwing(full,time,.03,p1,v1,a1);
                    foot_trajectory_detail::EvaluateSwing(tail,time,.03*std::pow(remaining,4),p2,v2,a2);
                    Check((p1-p2).norm()<1e-10 && (v1-v2).norm()<1e-9 && (a1-a2).norm()<1e-8,
                          "phase-preserving tail exactly retains full position velocity acceleration");
                }
            }
        }
        auto request = Request(problem);
        request.initial_velocity_world[1] = {0.05, 0.0, 0.0};
        {
            auto phase_request=request;phase_request.preserve_inflight_clearance_phase=true;
            phase_request.add_clearance_to_inflight_continuation=false;
            foot_trajectory_detail::PreparedTrajectory prepared;
            Check(foot_trajectory_detail::Prepare(phase_request,prepared)==JointPlannerFailure::kNone,
                  "phase-preserving request prepares through production seam");
            bool checked=false;
            for(const auto &event:prepared.events)if(event.starts_in_flight) {
                Check(std::abs(event.inflight_clearance_m-.03*std::pow(.08/.18,4))<1e-12,
                      "original absolute event duration sets remaining amplitude");checked=true;
            }
            Check(checked,"in-flight amplitude fixture actually exercised");
        }

        { auto continued=request;continued.add_clearance_to_inflight_continuation=false;
        const auto original_start=SampleFootTrajectoryAt(request,T(1.0));
        const auto continued_start=SampleFootTrajectoryAt(continued,T(1.0));
        Check(original_start.valid && continued_start.valid,"inflight continuation inputs");
        Check((EigenValue(original_start.samples[0].center_world[1])-EigenValue(continued_start.samples[0].center_world[1])).norm()<1e-12,"continuation preserves initial position");
        Check(std::abs(original_start.samples[0].velocity_world[1].x-continued_start.samples[0].velocity_world[1].x)<1e-12,"continuation preserves initial velocity");
        Check(std::abs(original_start.samples[0].acceleration_world[1].z-continued_start.samples[0].acceleration_world[1].z-32*.03/(.08*.08))<1e-8,"no restarted clearance acceleration");
        const auto continued_td=SampleFootTrajectoryAt(continued,T(1.08));
        const auto original_td=SampleFootTrajectoryAt(request,T(1.08));
        Check(continued_td.valid && (EigenValue(continued_td.samples[0].center_world[1])-EigenValue(original_td.samples[0].center_world[1])).norm()<1e-12,"absolute touchdown unchanged");
        }
        auto prefix_problem=problem;
        prefix_problem.schedule.back().end=T(1.40);
        for(auto &surfaces:prefix_problem.candidate_surfaces)
            for(auto &surface:surfaces)surface.valid_until=T(1.40);
        auto prefix_request=request;prefix_request.problem=&prefix_problem;prefix_request.end=T(1.40);
        Check(!SampleFootTrajectoryAt(prefix_request,T(1.39)).valid,"legacy full contact lifetime remains strict");
        prefix_request.allow_surface_contact_tail_beyond_horizon=true;
        Check(SampleFootTrajectoryAt(prefix_request,T(1.39)).valid,"covered feedback prefix accepted");
        prefix_problem.candidate_surfaces[2][0].valid_until=T(1.34);
        Check(!SampleFootTrajectoryAt(prefix_request,T(1.39)).valid,"uncovered touchdown rejected");
        const std::vector<TimeNs> times{
            T(1.0), T(1.05), T(1.08), T(1.10), T(1.15), T(1.20),
            T(1.25), T(1.30), T(1.35), T(1.40), T(1.49)};
        const auto result = SampleFootTrajectory(request, times);
        Check(result.valid && result.failure == JointPlannerFailure::kNone,
              "valid multi-touchdown trajectory rejected");
        const Eigen::Vector3d normal = tilted.col(2);
        CheckCenter(result.samples.front(), 1,
                    EigenValue(problem.request.input.feet[1].foot_collision_center_world),
                    1.0e-12, "initial in-flight position changed");
        Check(std::abs(result.samples.front().velocity_world[1].x - 0.05) <
                  1.0e-12,
              "initial in-flight velocity was projected away");
        CheckCenter(result.samples[2], 1,
                    EigenValue(in_flight_target) + Eigen::Vector3d(0.0, 0.0, 0.022),
                    1.0e-12, "in-flight touchdown center omitted radius");
        Check(Norm(result.samples[2].velocity_world[1]) == 0.0 &&
                  EigenValue(TimedPoint{result.samples[2].acceleration_world[1],
                                        Frame::kWorld, T(1.0), true,
                                        PointRole::kFootCollisionCenter})
                      .allFinite(),
              "touchdown endpoint velocity or acceleration is invalid");
        CheckCenter(result.samples[4], 0,
                    EigenValue(first_target) + Eigen::Vector3d(0.0, 0.0, 0.022),
                    1.0e-12, "first touchdown center omitted radius");
        CheckCenter(result.samples[6], 0,
                    EigenValue(first_target) + Eigen::Vector3d(0.0, 0.0, 0.022),
                    1.0e-12, "stance was retimed between touchdowns");
        Check(Norm(result.samples[6].velocity_world[0]) == 0.0 &&
                  Norm(result.samples[6].acceleration_world[0]) == 0.0,
              "stance velocity was not zero");
        Check(Norm(result.samples[7].velocity_world[0]) == 0.0,
              "next liftoff did not preserve C1 velocity");
        CheckCenter(result.samples[8], 0,
                    EigenValue(second_target) + 0.022 * normal,
                    1.0e-12, "tilted surface normal was not applied");
        Check(result.samples[3].center_world[0].source_time == T(1.0),
              "generated point provenance was retimestamped to the future");
        // At u=0.5 the zero-slope Hermite position is the midpoint and the
        // clearance bump is exactly one clearance radius. Its analytic x
        // velocity is 1.5 * delta_x / dt and its x acceleration is zero.
        Check(std::abs(result.samples[3].center_world[0].value.x + 0.115) <
                  1.0e-12 &&
                  std::abs(result.samples[3].center_world[0].value.z - 0.151) <
                  1.0e-12,
              "Hermite midpoint or normal clearance is incorrect");
        Check(std::abs(result.samples[3].velocity_world[0].x - 4.05) <
                  1.0e-10 &&
                  std::abs(result.samples[3].acceleration_world[0].x) <
                  1.0e-10,
              "analytic swing derivative or acceleration is incorrect");
        Check(Norm(result.samples[3].velocity_world[0]) > 1.0e-6,
              "swing derivative was missing");
        Check(EigenValue(TimedPoint{result.samples[3].acceleration_world[0],
                                    Frame::kWorld, T(1.0), true,
                                    PointRole::kFootCollisionCenter})
                  .allFinite(),
              "swing acceleration was not finite");
        // The C1 contract fixes velocity at both endpoints; analytic
        // acceleration may jump when the swing begins or ends.
        Check(Norm(result.samples[1].velocity_world[0]) == 0.0,
              "liftoff endpoint velocity was not stance-compatible");
        Check(Norm(result.samples[4].velocity_world[0]) == 0.0,
              "touchdown endpoint velocity was not stance-compatible");
        const auto terminal = SampleFootTrajectoryEndState(request);
        Check(terminal.valid && terminal.samples.size() == 1 &&
                  terminal.samples.front().time == T(1.5),
              "closed terminal state was not exposed separately");
        CheckCenter(terminal.samples.front(), 0,
                    EigenValue(second_target) + 0.022 * normal, 1.0e-12,
                    "terminal state lost final stance target");
        Check(Norm(terminal.samples.front().velocity_world[0]) == 0.0 &&
                  Norm(terminal.samples.front().acceleration_world[0]) == 0.0,
              "terminal stance derivatives were not zero");
        auto uncovered_tail = problem;
        uncovered_tail.request.events.events[2].contact_interval_end = T(1.45);
        auto uncovered_request = request;
        uncovered_request.problem = &uncovered_tail;
        const auto uncovered = SampleFootTrajectory(uncovered_request, times);
        Check(!uncovered.valid &&
                  uncovered.failure == JointPlannerFailure::kCoverageIncomplete,
              "uncovered post-contact swing was fabricated");
        // An explicit terminal continuation supplies the next target without
        // extending the core event table or its dynamics/commitments.
        auto continued_problem = problem;
        continued_problem.request.events.events[2].contact_interval_end =
            T(1.45);
        continued_problem.schedule.clear();
        AddInterval(continued_problem, 1.00, 1.05, {{true, false, true, true}},
                    {{-1, -1, -1, -1}});
        AddInterval(continued_problem, 1.05, 1.08, {{false, false, true, true}},
                    {{-1, -1, -1, -1}});
        AddInterval(continued_problem, 1.08, 1.15, {{false, true, true, true}},
                    {{-1, 0, -1, -1}});
        AddInterval(continued_problem, 1.15, 1.30, {{true, true, true, true}},
                    {{1, 0, -1, -1}});
        AddInterval(continued_problem, 1.30, 1.35, {{false, true, true, true}},
                    {{-1, 0, -1, -1}});
        AddInterval(continued_problem, 1.35, 1.45, {{true, true, true, true}},
                    {{2, 0, -1, -1}});
        AddInterval(continued_problem, 1.45, 1.50, {{false, true, true, true}},
                    {{-1, 0, -1, -1}});
        FootSwingContinuation continuation;
        continuation.valid = true;
        continuation.event.id = {3, go2::Leg::FR, 3};
        continuation.event.liftoff_time = T(1.45);
        continuation.event.liftoff_valid = true;
        continuation.event.touchdown_time = T(1.50);
        continuation.event.contact_interval_end = T(1.65);
        continuation.event.target_world =
            Point(0.40, -0.12, 0.0, PointRole::kSurfaceContactPoint);
        continuation.candidate.candidate_id = 3;
        continuation.candidate.target_world =
            continuation.event.target_world;
        continuation.candidate.coverage = MapCoverageState::kKnown;
        continuation.candidate.geometry_hard_feasible = true;
        continuation.surface = Surface();
        auto continued_request = request;
        continued_request.problem = &continued_problem;
        continued_request.continuation[0] = continuation;
        const auto continued =
            SampleFootTrajectory(continued_request, times);
        Check(continued.valid, "explicit terminal continuation was rejected");
        Check(continued.samples.back().center_world[0].source_time == T(1.0),
              "terminal continuation retimestamped provenance");
        const auto continued_end =
            SampleFootTrajectoryEndState(continued_request);
        Check(continued_end.valid, "terminal continuation end state rejected");
        CheckCenter(continued_end.samples.front(), 0,
                    EigenValue(continuation.event.target_world) +
                        Eigen::Vector3d(0.0, 0.0, 0.022),
                    1.0e-12, "terminal continuation target was not sampled");
        Check(Norm(continued_end.samples.front().velocity_world[0]) == 0.0,
              "terminal continuation touchdown velocity was not zero");
        // Explicit observed candidate binding cannot change core events or
        // silently select a target. Failures leave the request untouched.
        TouchdownEventTable tails; tails.events.push_back(continuation.event);
        TerrainCandidateGenerationResult generated; generated.valid=true;
        TerrainCandidateSet tail_set; tail_set.event_set.event_id=continuation.event.id;
        tail_set.event_set.candidates.push_back(continuation.candidate);
        TerrainCandidateSurfaceMatch match;
        match.surface_world=continuation.candidate.target_world;
        match.sphere_center_world=match.surface_world;
        match.sphere_center_world.role=PointRole::kFootCollisionCenter;
        match.sphere_center_world.value.z+=0.022;
        match.collision_radius_m=0.022;match.contact_surface=continuation.surface;
        match.patch.valid=true;match.patch.all_known=true;
        tail_set.matched_surfaces.push_back(match);generated.sets.push_back(tail_set);
        auto to_bind=request;to_bind.problem=&continued_problem;
        const auto core_count=continued_problem.request.events.events.size();
        Check(BindTerminalSwingCandidates(to_bind,tails,generated,{0})==JointPlannerFailure::kNone,
              "observed terminal binding failed");
        Check(continued_problem.request.events.events.size()==core_count,
              "terminal binding mutated core events");
        auto invalid_bind=request;invalid_bind.problem=&continued_problem;
        Check(BindTerminalSwingCandidates(invalid_bind,tails,generated,{1})==JointPlannerFailure::kInvalidInput &&
              !invalid_bind.continuation[0].valid,"invalid selection changed request");
        auto unknown=generated;unknown.valid=false;
        unknown.failure=JointPlannerFailure::kCoverageIncomplete;
        Check(BindTerminalSwingCandidates(invalid_bind,tails,unknown,{0})==JointPlannerFailure::kCoverageIncomplete,
              "unknown terrain failure was relabeled");
        auto conflicting_target=tails;conflicting_target.events[0].target_world.value.x+=0.1;
        Check(BindTerminalSwingCandidates(invalid_bind,conflicting_target,generated,{0})==JointPlannerFailure::kInitialConditionConflict,
              "conflicting terminal target was overwritten");
        auto expired=generated;expired.sets[0].matched_surfaces[0].contact_surface.valid_until=T(1.49);
        Check(BindTerminalSwingCandidates(invalid_bind,tails,expired,{0})==JointPlannerFailure::kCoverageIncomplete &&
              !invalid_bind.continuation[0].valid,"expired terminal evidence accepted");
        auto conflict=generated;conflict.sets[0].matched_surfaces[0].contact_surface.map_epoch++;
        Check(BindTerminalSwingCandidates(invalid_bind,tails,conflict,{0})==JointPlannerFailure::kInvalidInput,
              "cross-map terminal evidence accepted");
        auto malformed_continuation = continued_request;
        malformed_continuation.continuation[0].event.liftoff_time = T(1.46);
        const auto malformed =
            SampleFootTrajectory(malformed_continuation, times);
        Check(!malformed.valid,
              "gapped terminal continuation was accepted");
        auto missing_liftoff = problem;
        missing_liftoff.request.events.events[1].liftoff_valid = false;
        Check(!SampleFootTrajectory(request, times).samples.empty(),
              "baseline trajectory unexpectedly empty");
        auto missing_request = request;
        missing_request.problem = &missing_liftoff;
        Check(!SampleFootTrajectory(missing_request, times).valid,
              "missing liftoff metadata was accepted");
        auto unknown_target = problem;
        unknown_target.request.candidate_sets[1].candidates[0].target_world.valid = false;
        auto unknown_request = request;
        unknown_request.problem = &unknown_target;
        Check(!SampleFootTrajectory(unknown_request, times).valid,
              "unknown selected target was accepted");
        auto unknown_surface = problem;
        unknown_surface.candidate_surfaces[1][0].coverage =
            MapCoverageState::kUnknownInside;
        auto unknown_surface_request = request;
        unknown_surface_request.problem = &unknown_surface;
        const auto unknown_surface_result =
            SampleFootTrajectory(unknown_surface_request, times);
        Check(!unknown_surface_result.valid &&
                  unknown_surface_result.failure ==
                      JointPlannerFailure::kCoverageIncomplete,
              "unknown surface coverage was accepted");
        auto future_target = problem;
        future_target.request.candidate_sets[1].candidates[0].target_world.source_time =
            T(1.01);
        auto future_target_request = request;
        future_target_request.problem = &future_target;
        Check(!SampleFootTrajectory(future_target_request, times).valid,
              "future target provenance was accepted");
        auto no_events = Fixture();
        AddInterval(no_events, 1.00, 1.50, {{true, true, true, true}},
                    {{-1, -1, -1, -1}});
        auto no_events_request = Request(no_events);
        const auto allstance = SampleFootTrajectory(no_events_request, times);
        Check(allstance.valid, "explicit all-stance schedule was rejected");
        no_events.schedule[0].contact[0] = false;
        no_events_request.problem = &no_events;
        const auto initial_aerial =
            SampleFootTrajectory(no_events_request, times);
        Check(!initial_aerial.valid &&
                  initial_aerial.failure ==
                      JointPlannerFailure::kCoverageIncomplete,
              "initial aerial interval without event was fabricated");
        auto bad_radius = request;
        bad_radius.collision_radius_valid[0] = false;
        Check(!SampleFootTrajectory(bad_radius, times).valid,
              "missing collision radius was accepted");
        auto slipping_stance = request;
        slipping_stance.initial_velocity_world[2] = {0.01, 0.0, 0.0};
        const auto slipping_result = SampleFootTrajectory(slipping_stance, times);
        Check(!slipping_result.valid &&
                  slipping_result.failure ==
                      JointPlannerFailure::kInitialConditionConflict,
              "initial measured stance slip was silently projected");
        auto bad_endpoint = SampleFootTrajectoryAt(request, T(1.5));
        Check(!bad_endpoint.valid &&
                  bad_endpoint.failure == JointPlannerFailure::kInvalidInput,
              "closed horizon endpoint was accepted");
        auto mixed_surfaces = problem;
        mixed_surfaces.event_surfaces.assign(mixed_surfaces.request.events.events.size(),
                                             Surface());
        auto mixed_request = request;
        mixed_request.problem = &mixed_surfaces;
        Check(!SampleFootTrajectory(mixed_request, times).valid,
              "legacy and candidate surfaces were combined");
        auto committed = problem;
        committed.request.events.events[1].committed = true;
        committed.request.accepted_commitments.events = {
            committed.request.events.events[1]};
        auto committed_request = request;
        committed_request.problem = &committed;
        const auto committed_result = SampleFootTrajectory(committed_request, times);
        Check(committed_result.valid, "unchanged committed absolute prefix rejected");
        auto retimed = committed;
        retimed.request.events.events[1].liftoff_time = T(1.06);
        auto retimed_request = committed_request;
        retimed_request.problem = &retimed;
        Check(!SampleFootTrajectory(retimed_request, times).valid,
              "committed liftoff was locally retimed");
        // A command-only handover may seed the initial collision-center p/v
        // without rewriting measured input or the planning identity.
        auto commanded_problem = problem;
        const auto identity_before = commanded_problem.request.input.identity;
        std::array<TimedPoint, go2::kLegCount> measured_before{};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            measured_before[leg] =
                commanded_problem.request.input.feet[leg].foot_collision_center_world;
        auto commanded_request = request;
        commanded_request.problem = &commanded_problem;
        // The legacy initial-velocity fields are intentionally unavailable and
        // differ from the command seed; the command branch must not consume them.
        commanded_request.initial_velocity_valid.fill(false);
        commanded_request.initial_velocity_world[1] = {0.20, 0.0, 0.0};
        commanded_request.commanded_initial.enabled = true;
        commanded_request.commanded_initial.command_epoch = 41;
        commanded_request.commanded_initial.source_time = T(1.0);
        commanded_request.commanded_initial.valid_until = T(1.20);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            commanded_request.commanded_initial.valid[leg] = true;
            commanded_request.commanded_initial.center_world[leg] =
                Point(0.50 + 0.03 * static_cast<double>(leg),
                      -0.20 + 0.02 * static_cast<double>(leg), 0.27,
                      PointRole::kFootCollisionCenter, T(1.0));
            commanded_request.commanded_initial.velocity_world[leg] =
                leg == 1 ? go2::Vec3{0.05, 0.0, 0.0}
                         : go2::Vec3{0.0, 0.0, 0.0};
        }
        const auto commanded_start =
            SampleFootTrajectoryAt(commanded_request, T(1.0));
        Check(commanded_start.valid,
              "valid command boundary was not accepted independently");
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            CheckCenter(commanded_start.samples.front(), leg,
                        EigenValue(commanded_request.commanded_initial.center_world[leg]),
                        1.0e-12, "command boundary center was not used");
            Check(commanded_start.samples.front().center_world[leg].source_time ==
                      T(1.0),
                  "command boundary provenance was retimestamped");
            Check(SameTimedPoint(
                      commanded_problem.request.input.feet[leg].foot_collision_center_world,
                      measured_before[leg]),
                  "measured foot center was modified by command handover");
        }
        Check(identity_before.source_state_tick ==
                  commanded_problem.request.input.identity.source_state_tick &&
                  identity_before.source_state_time ==
                      commanded_problem.request.input.identity.source_state_time &&
                  identity_before.map_epoch ==
                      commanded_problem.request.input.identity.map_epoch &&
                  identity_before.schedule_epoch ==
                      commanded_problem.request.input.identity.schedule_epoch &&
                  identity_before.source_plan_id ==
                      commanded_problem.request.input.identity.source_plan_id,
              "planning identity was modified by command handover");
        Check(std::abs(commanded_start.samples.front().velocity_world[1].x -
                           0.05) < 1.0e-10,
              "command boundary velocity was not used for in-flight swing");
        const auto commanded_td =
            SampleFootTrajectoryAt(commanded_request, T(1.08));
        Check(commanded_td.valid, "command handover touchdown sample failed");
        CheckCenter(commanded_td.samples.front(), 1,
                    EigenValue(in_flight_target) + Eigen::Vector3d(0.0, 0.0, 0.022),
                    1.0e-12, "command handover changed absolute touchdown");
        Check(Norm(commanded_td.samples.front().velocity_world[1]) == 0.0 &&
                  Norm(commanded_td.samples.front().acceleration_world[1]) == 0.0,
              "command handover touchdown was not C1");
        auto command_no_bump = commanded_request;
        command_no_bump.add_clearance_to_inflight_continuation = false;
        const auto command_no_bump_start =
            SampleFootTrajectoryAt(command_no_bump, T(1.0));
        Check(command_no_bump_start.valid,
              "command boundary no-bump variant failed");
        Check((EigenValue(command_no_bump_start.samples.front().center_world[1]) -
               EigenValue(commanded_start.samples.front().center_world[1])).norm() <
                  1.0e-12 &&
                  std::abs(command_no_bump_start.samples.front().velocity_world[1].x -
                           commanded_start.samples.front().velocity_world[1].x) < 1.0e-12,
              "command boundary restarted the in-flight swing");
        // A nonzero commanded stance velocity remains a hard conflict unless
        // the explicit soft-reference settling opt-in is enabled.
        auto settling_disabled = commanded_request;
        settling_disabled.commanded_initial.velocity_world[2] =
            {0.12, 0.0, 0.0};
        const auto disabled_stance =
            SampleFootTrajectoryAt(settling_disabled, T(1.0));
        Check(!disabled_stance.valid &&
                  disabled_stance.failure ==
                      JointPlannerFailure::kInitialConditionConflict,
              "commanded stance velocity was silently reset");
        auto settling_request = settling_disabled;
        settling_request.enable_commanded_stance_settling = true;
        settling_request.commanded_stance_settling_duration_s = 0.020;
        const auto settled = SampleFootTrajectory(
            settling_request,
            std::vector<TimeNs>{T(1.0), T(1.006666667), T(1.01), T(1.02), T(1.10)});
        Check(settled.valid, "commanded stance settling was rejected");
        const Eigen::Vector3d settle_p0 = EigenValue(
            settling_request.commanded_initial.center_world[2]);
        const Eigen::Vector3d settle_v0{0.12, 0.0, 0.0};
        Check((EigenValue(settled.samples[0].center_world[2]) - settle_p0).norm() <
                  1.0e-12 &&
                  Difference(settled.samples[0].velocity_world[2],
                             {0.12, 0.0, 0.0}) < 1.0e-12,
              "settling did not preserve commanded C1 start");
        const double settling_dt = T(1.02).seconds() - T(1.0).seconds();
        const double settling_elapsed =
            T(1.006666667).seconds() - T(1.0).seconds();
        const double settling_u = settling_elapsed / settling_dt;
        const double settling_h10 = settling_u * settling_u * settling_u -
            2.0 * settling_u * settling_u + settling_u;
        const Eigen::Vector3d expected_settle_mid =
            settle_p0 + settling_h10 * settling_dt * settle_v0;
        Check((EigenValue(settled.samples[1].center_world[2]) -
               expected_settle_mid).norm() < 1.0e-11,
              "settling Hermite position is incorrect");
        Check((EigenValue(settled.samples[1].center_world[2]) - settle_p0).norm() <=
                  (4.0 / 27.0) * settling_dt * settle_v0.norm() + 1.0e-11,
              "settling excursion exceeded 4/27 oracle");
        Check((EigenValue(settled.samples[3].center_world[2]) - settle_p0).norm() <
                  1.0e-12 &&
                  Norm(settled.samples[3].velocity_world[2]) < 1.0e-12,
              "settling did not reach the C1 zero-velocity endpoint");
        const auto original_future =
            SampleFootTrajectoryAt(commanded_request, T(1.10));
        const auto settled_future =
            SampleFootTrajectoryAt(settling_request, T(1.10));
        Check(original_future.valid && settled_future.valid &&
                  Difference(original_future.samples.front().center_world[0].value,
                             settled_future.samples.front().center_world[0].value) <
                      1.0e-12 &&
                  Difference(original_future.samples.front().velocity_world[0],
                             settled_future.samples.front().velocity_world[0]) <
                      1.0e-12 &&
                  Difference(original_future.samples.front().acceleration_world[0],
                             settled_future.samples.front().acceleration_world[0]) <
                      1.0e-12,
              "settling changed a future swing curve");
        auto zero_duration = settling_request;
        zero_duration.commanded_stance_settling_duration_s = 0.0;
        const auto zero_duration_result =
            SampleFootTrajectoryAt(zero_duration, T(1.0));
        Check(!zero_duration_result.valid &&
                  zero_duration_result.failure ==
                      JointPlannerFailure::kInitialConditionConflict,
              "zero settling duration was accepted");
        auto overlong_duration = settling_request;
        overlong_duration.commanded_stance_settling_duration_s = 0.021;
        const auto overlong_result =
            SampleFootTrajectoryAt(overlong_duration, T(1.0));
        Check(!overlong_result.valid &&
                  overlong_result.failure ==
                      JointPlannerFailure::kInitialConditionConflict,
              "overlong settling duration was accepted");
        // The actual bridge is clipped to the remaining stance interval. It
        // must reach zero velocity exactly at the next liftoff boundary.
        auto clipped_settling = settling_disabled;
        clipped_settling.start = T(1.04);
        clipped_settling.commanded_initial.source_time = T(1.04);
        clipped_settling.commanded_initial.valid_until = T(1.20);
        clipped_settling.commanded_initial.velocity_world[0] =
            {0.01, 0.0, 0.0};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            clipped_settling.commanded_initial.center_world[leg].source_time =
                T(1.04);
        clipped_settling.enable_commanded_stance_settling = true;
        clipped_settling.commanded_stance_settling_duration_s = 0.020;
        const auto clipped_start =
            SampleFootTrajectoryAt(clipped_settling, T(1.04));
        const auto clipped_liftoff =
            SampleFootTrajectoryAt(clipped_settling, T(1.05));
        Check(clipped_start.valid && clipped_liftoff.valid &&
                  Difference(clipped_start.samples.front().velocity_world[0],
                             {0.01, 0.0, 0.0}) < 1.0e-12 &&
                  Norm(clipped_liftoff.samples.front().velocity_world[0]) <
                      1.0e-12,
              "settling crossed the next liftoff boundary");
        auto late_problem = commanded_problem;
        auto late_request = commanded_request;
        late_request.problem = &late_problem;
        late_request.start = T(1.01);
        late_request.commanded_initial.source_time = T(1.01);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            late_request.commanded_initial.center_world[leg].source_time =
                T(1.01);
        const auto late = SampleFootTrajectoryAt(late_request, T(1.01));
        Check(late.valid,
              "command boundary at a later source time was rejected");
        Check(late.samples.front().center_world[1].source_time == T(1.01),
              "late command boundary lost source provenance");
        auto missing_boundary = commanded_request;
        missing_boundary.commanded_initial.valid[0] = false;
        const auto missing_boundary_result =
            SampleFootTrajectoryAt(missing_boundary, T(1.0));
        Check(!missing_boundary_result.valid &&
                  missing_boundary_result.failure ==
                      JointPlannerFailure::kObservationUnavailable,
              "missing command boundary was not fail-closed");
        auto expired_boundary = commanded_request;
        expired_boundary.commanded_initial.valid_until = T(0.99);
        const auto expired_boundary_result =
            SampleFootTrajectoryAt(expired_boundary, T(1.0));
        Check(!expired_boundary_result.valid &&
                  expired_boundary_result.failure ==
                      JointPlannerFailure::kObservationUnavailable,
              "expired command boundary was not fail-closed");
        auto future_boundary = commanded_request;
        future_boundary.commanded_initial.source_time = T(1.01);
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            future_boundary.commanded_initial.center_world[leg].source_time =
                T(1.01);
        const auto future_boundary_result =
            SampleFootTrajectoryAt(future_boundary, T(1.0));
        Check(!future_boundary_result.valid &&
                  future_boundary_result.failure ==
                      JointPlannerFailure::kObservationUnavailable,
              "future command boundary was not fail-closed");
        auto wrong_role_boundary = commanded_request;
        wrong_role_boundary.commanded_initial.center_world[0].role =
            PointRole::kFootSite;
        const auto wrong_role_result =
            SampleFootTrajectoryAt(wrong_role_boundary, T(1.0));
        Check(!wrong_role_result.valid &&
                  wrong_role_result.failure ==
                      JointPlannerFailure::kObservationUnavailable,
              "wrong command boundary role was not fail-closed");
        auto wrong_time_boundary = commanded_request;
        wrong_time_boundary.commanded_initial.center_world[0].source_time =
            T(1.01);
        const auto wrong_time_result =
            SampleFootTrajectoryAt(wrong_time_boundary, T(1.0));
        Check(!wrong_time_result.valid &&
                  wrong_time_result.failure ==
                      JointPlannerFailure::kObservationUnavailable,
              "inconsistent command boundary time was not fail-closed");
        auto invalid_measured_problem = commanded_problem;
        invalid_measured_problem.request.input.feet[0]
            .foot_collision_center_world.role = PointRole::kFootSite;
        auto invalid_measured_request = commanded_request;
        invalid_measured_request.problem = &invalid_measured_problem;
        const auto invalid_measured_result =
            SampleFootTrajectoryAt(invalid_measured_request, T(1.0));
        Check(!invalid_measured_result.valid &&
                  invalid_measured_result.failure ==
                      JointPlannerFailure::kObservationUnavailable,
              "invalid measured center was hidden by command boundary");
        std::cout << "Stage C foot trajectory role/time, radius-normal, C1 swing, "
                     "multi-touchdown and fail-closed checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
