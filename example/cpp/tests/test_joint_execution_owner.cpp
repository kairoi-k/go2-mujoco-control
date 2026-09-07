// Atomic proposal adoption and retained-curve contract fixtures.
#include "stage_c/joint_execution_owner.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace go2_terrain::stage_c;
using namespace go2_terrain::stage_c::joint_execution;
namespace
{
TimeNs T(double seconds)
{
    return TimeNs::FromSeconds(seconds);
}
TimedPoint Point(double x, double y, double z, PointRole role, TimeNs time)
{
    return {{x, y, z}, Frame::kWorld, time, true, role};
}
std::shared_ptr<JointExecutionProposal> ValidProposal(
    std::uint64_t id, double source_s, double end_s,
    bool with_inflight = false, double liftoff_s = -1.0,
    double touchdown_s = -1.0, double contact_end_s = -1.0,
    bool include_commitment = false)
{
    auto selected = std::make_shared<CentroidalJointProposal>();
    auto &problem = selected->selected_problem;
    auto &input = problem.request.input;
    const TimeNs source = T(source_s);
    const TimeNs end = T(end_s);
    input.identity = {100 + id, source, 7, 3, id};
    input.body.valid = true;
    input.body.base_position_world =
        Point(0.0, 0.0, 0.42, PointRole::kBodyOrigin, source);
    input.body.mass_kg = 12.0;
    input.measured_contact.mask.fill(true);
    input.measured_contact.provenance = ContactProvenance::kMeasured;
    input.measured_contact.source_time = source;
    input.measured_contact.valid = true;
    input.map.metadata_valid = true;
    input.map.epoch = 7;
    input.map.width = input.map.height = input.map.total_cells =
        input.map.known_cells = 8;
    problem.schedule_epoch = 3;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const double x = -0.25 + 0.16 * static_cast<double>(leg);
        const double y = -0.12 + 0.08 * static_cast<double>(leg);
        input.feet[leg].foot_collision_center_world =
            Point(x, y, 0.22, PointRole::kFootCollisionCenter, source);
        input.feet[leg].measured_support_anchor_world =
            Point(x, y, 0.0, PointRole::kSurfaceContactPoint, source);
        input.feet[leg].measured_support_anchor_valid = true;
    }
    if (!with_inflight)
    {
        problem.schedule.push_back({
            source, end, {true, true, true, true}, {-1, -1, -1, -1}});
    }
    else
    {
        const double liftoff_seconds = liftoff_s >= 0.0
            ? liftoff_s : source_s - 0.10;
        const double touchdown_seconds = touchdown_s >= 0.0
            ? touchdown_s : source_s + 0.10;
        const double contact_end_seconds = contact_end_s >= 0.0
            ? contact_end_s : source_s + 0.30;
        const TimeNs liftoff = T(liftoff_seconds);
        const TimeNs touchdown = T(touchdown_seconds);
        const TimeNs contact_end = T(contact_end_seconds);
        // Keep target provenance stable across replans; source_time may be
        // older than the new observation but must not be fabricated.
        const TimedPoint target = Point(
            0.30, -0.12, 0.0, PointRole::kSurfaceContactPoint, T(1.0));
        TouchdownEvent event;
        event.id = {3, go2::Leg::FR, 1};
        event.liftoff_time = liftoff;
        event.liftoff_valid = true;
        event.touchdown_time = touchdown;
        event.contact_interval_end = contact_end;
        event.target_world = target;
        event.committed = true;
        problem.request.events.events.push_back(event);
        if (include_commitment)
            problem.request.accepted_commitments.events.push_back(event);
        StageCCandidate candidate;
        candidate.candidate_id = 1;
        candidate.target_world = target;
        candidate.coverage = MapCoverageState::kKnown;
        candidate.geometry_hard_feasible = true;
        problem.request.candidate_sets.push_back({
            event.id, {candidate}, true});
        problem.combination.push_back(0);
        ContactSurface surface;
        surface.frame = Frame::kWorld;
        surface.coverage = MapCoverageState::kKnown;
        surface.map_epoch = 7;
        surface.valid_until = T(end_s + 0.20);
        surface.friction_mu = 0.8;
        surface.max_normal_n = 180.0;
        problem.candidate_surfaces.push_back({surface});
        if (source < liftoff)
            problem.schedule.push_back({
                source, liftoff, {true, true, true, true},
                {-1, -1, -1, -1}});
        const TimeNs swing_start = source < liftoff ? liftoff : source;
        problem.schedule.push_back({
            swing_start, touchdown, {false, true, true, true},
            {-1, -1, -1, -1}});
        if (touchdown < end)
            problem.schedule.push_back({
                touchdown, end, {true, true, true, true}, {0, -1, -1, -1}});
    }
    selected->selected_valid = true;
    selected->search.feasible = true;
    selected->selected_result.certificate.feasible = true;
    auto proposal = std::make_shared<JointExecutionProposal>();
    proposal->proposal_id = id;
    proposal->identity = input.identity;
    proposal->valid_until = end;
    proposal->selected = selected;
    proposal->foot_request.problem = &proposal->selected->selected_problem;
    proposal->foot_request.start = source;
    proposal->foot_request.end = end;
    proposal->foot_request.swing_clearance_m = 0.0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        proposal->foot_request.collision_radius_m[leg] = 0.022;
        proposal->foot_request.collision_radius_valid[leg] = true;
        proposal->foot_request.initial_velocity_world[leg] = {0.0, 0.0, 0.0};
        proposal->foot_request.initial_velocity_valid[leg] = true;
    }
    if (with_inflight && liftoff_s >= 0.0 && liftoff_s < source_s)
        proposal->foot_request.initial_velocity_world[0] = {0.05, 0.0, 0.0};
    return proposal;
}

void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
void TestNoPendingAndAtomicRetain()
{
    OwnerConfig config;
    config.expected_schedule_epoch = 3;
    config.expected_map_epoch = 7;
    AtomicJointExecutionOwner owner(config);
    auto empty = owner.Adopt(T(1.10), 101);
    Check(empty.status == OwnerStatus::kNoPending, "empty owner adopted");
    auto first = ValidProposal(1, 1.0, 1.5);
    owner.Publish(first);
    auto adopted = owner.Adopt(T(1.10), 101);
    Check(adopted.status == OwnerStatus::kAdopted, "valid proposal rejected");
    Check(adopted.accepted && adopted.accepted->execution_version == 1,
          "first version missing");
    Check(adopted.accepted->proposal->selected.get() == first->selected.get(),
          "selected problem/result were copied or replaced");
    owner.Publish(first);
    auto retained = owner.Adopt(T(1.20), 102);
    Check(retained.status == OwnerStatus::kRetained,
          "same proposal was adopted twice");
    Check(retained.accepted->execution_version == 1,
          "same proposal incremented version");
    const auto sample = owner.SampleAt(T(1.20));
    Check(sample.valid && sample.execution_version == 1,
          "accepted reference sample missing");
    Check(sample.center_reference.center_world[0].role ==
              PointRole::kFootCollisionCenter,
          "sample lost center role");
}
void TestStaleDoesNotDropAccepted()
{
    AtomicJointExecutionOwner owner;
    auto first = ValidProposal(1, 1.0, 1.5);
    owner.Publish(first);
    Check(owner.Adopt(T(1.10), 101).status == OwnerStatus::kAdopted,
          "setup proposal rejected");
    auto expired = ValidProposal(2, 1.0, 1.5);
    expired->valid_until = T(1.05);
    owner.Publish(expired);
    auto result = owner.Adopt(T(1.10), 101);
    Check(result.status == OwnerStatus::kStale,
          "expired proposal was not rejected");
    Check(result.accepted && result.accepted->execution_version == 1 &&
              result.accepted->proposal->proposal_id == 1,
          "stale proposal displaced accepted reference");
}
void TestCommandedHandoverNeverRewritesMeasuredInput()
{
    AtomicJointExecutionOwner owner;
    auto first = ValidProposal(1, 1.0, 1.5);
    first->first_handover_from_commanded = true;
    CommandedFootSeed seed;
    seed.command_epoch = 9;
    seed.source_time = T(1.10);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        seed.valid[leg] = true;
        seed.center_world[leg] =
            Point(0.1, 0.1, 0.2, PointRole::kFootCollisionCenter, T(1.10));
        seed.velocity_world[leg] = {0.2, 0.0, 0.0};
    }
    owner.Publish(first);
    auto result = owner.Adopt(T(1.10), 101, &seed);
    Check(result.status == OwnerStatus::kAdopted,
          "bounded commanded stance settling rejected nonzero boundary");
    const auto at_handover = owner.SampleAt(T(1.10));
    const auto after_handover = owner.SampleAt(T(1.11));
    Check(at_handover.valid && at_handover.bundle_valid &&
              after_handover.valid && at_handover.curve_valid[0] &&
              after_handover.curve_valid[0] &&
              std::abs(at_handover.center_reference.velocity_world[0].x -
                       seed.velocity_world[0].x) < 1.0e-12 &&
              std::abs(after_handover.center_reference.velocity_world[0].x) <
                  std::abs(seed.velocity_world[0].x),
          "commanded stance settling was not continuous or bounded");
    Check(result.accepted->proposal->selected.get() == first->selected.get(),
          "handover replaced physical optimization input");
    Check(first->selected->selected_problem.request.input.feet[0]
              .foot_collision_center_world.source_time == T(1.0),
          "handover rewrote measured source timestamp");
    Check(owner.Adopt(T(1.11), 102).status == OwnerStatus::kRetained,
          "bound copy caused repeated adoption or requested another seed");
    auto bad = ValidProposal(2, 1.0, 1.5);
    bad->first_handover_from_commanded = true;
    owner.Publish(bad);
    auto invalid_seed = seed;
    invalid_seed.source_time = T(1.09);
    Check(owner.Adopt(T(1.10), 102, &invalid_seed).status ==
              OwnerStatus::kMissingCommandedSeed,
          "handover accepted a seed with a conflicting source time");

}
void TestDelayedAdmissionPlanningPrefix()
{
    AtomicJointExecutionOwner owner;
    auto first=ValidProposal(1,1.0,1.5,true,1.15,1.25,1.6);
    owner.Publish(first);
    Check(owner.Adopt(T(1.10),101).status==OwnerStatus::kAdopted,"delay fixture setup");
    Check(owner.CommittedEvents(T(1.10)).events.empty(),"fixture already in flight");
    Check(owner.PlanningPrefix(T(1.10),T(1.14))->events.empty(),"prefix froze beyond deadline");
    const auto prefix=owner.PlanningPrefix(T(1.10),T(1.18));
    Check(prefix && prefix->events.size()==1 && prefix->events[0].committed,
          "deadline prefix missed imminent liftoff");
    auto changing=ValidProposal(2,1.0,1.5,true,1.15,1.25,1.6,false);
    auto changed=std::make_shared<CentroidalJointProposal>(*changing->selected);
    auto &problem=changed->selected_problem;
    problem.request.events.events[0].committed=false;
    problem.request.events.events[0].target_world.value.x+=.01;
    problem.request.candidate_sets[0].candidates[0].target_world.value.x+=.01;
    changing->selected=changed;changing->foot_request.problem=&changed->selected_problem;
    owner.Publish(changing);
    Check(owner.Adopt(T(1.16),102).status==OwnerStatus::kCommitmentConflict,
          "delayed changed target bypassed newly active lease");
    auto protected_plan=ValidProposal(3,1.0,1.5,true,1.15,1.25,1.6);
    auto bound=std::make_shared<CentroidalJointProposal>(*protected_plan->selected);
    bound->selected_problem.request.events.events[0]=prefix->events[0];
    bound->selected_problem.request.accepted_commitments=*prefix;
    bound->selected_problem.request.candidate_sets[0].candidates[0].target_world=prefix->events[0].target_world;
    protected_plan->selected=bound;
    protected_plan->foot_request.problem=&bound->selected_problem;
    protected_plan->latest_adoption_time=T(1.18);
    owner.Publish(protected_plan);
    Check(owner.Adopt(T(1.16),103).status==OwnerStatus::kAdopted,
          "protected target failed delayed admission");
    Check(owner.Adopt(T(1.20),103).status==OwnerStatus::kRetained &&
          owner.SampleAt(T(1.20)).valid && owner.Accepted()->proposal->valid_until==T(1.5),
          "admission deadline changed accepted trajectory validity");
    AtomicJointExecutionOwner late;late.Publish(protected_plan);
    Check(late.Adopt(T(1.19),104).status==OwnerStatus::kStale && !late.Accepted(),
          "late proposal was admitted on unprotected prefix");
    AtomicJointExecutionOwner endpoint;endpoint.Publish(protected_plan);
    Check(endpoint.Adopt(T(1.18),104).status==OwnerStatus::kAdopted,
          "inclusive admission endpoint rejected");
}
void TestProposalInitializationOnlyBeforeFirstAcceptance()
{
    AtomicJointExecutionOwner owner;
    auto first=ValidProposal(1,1.0,1.5);
    first->first_handover_from_commanded=true;
    const auto expected=SampleFootTrajectoryAt(first->foot_request,T(1.10));
    Check(expected.valid,"proposal reference fixture");
    owner.Publish(first);
    Check(owner.Adopt(T(1.10),101).status==OwnerStatus::kMissingCommandedSeed,
          "default silently bypassed commanded boundary");
    const auto adopted=owner.Adopt(T(1.10),101,nullptr,InitialReferenceMode::kProposalReference);
    Check(adopted.status==OwnerStatus::kAdopted && adopted.accepted->proposal.get()==first.get(),
          "initial proposal acquisition replaced source bundle");
    const auto actual=owner.SampleAt(T(1.10));
    Check(actual.valid && !adopted.accepted->proposal->foot_request.commanded_initial.enabled,
          "initial proposal acquisition fabricated commanded boundary");
    for(int leg=0;leg<4;++leg) {
        const auto &p=actual.center_reference.center_world[leg];
        const auto &q=expected.samples[0].center_world[leg];
        Check(p.source_time==q.source_time && p.value.x==q.value.x && p.value.y==q.value.y && p.value.z==q.value.z,
              "initial proposal reference was reseeded");
    }
    auto next=ValidProposal(2,1.0,1.5);next->first_handover_from_commanded=true;
    owner.Publish(next);
    Check(owner.Adopt(T(1.11),102,nullptr,InitialReferenceMode::kProposalReference).status==OwnerStatus::kMissingCommandedSeed,
          "proposal initialization bypassed later handover");
    Check(owner.Accepted()->proposal.get()==first.get(),"failed later handover lost accepted plan");
}
void TestInflightLeaseRefreshAndReplan()
{
    AtomicJointExecutionOwner owner;
    auto first = ValidProposal(1, 1.0, 1.30, true, 1.05, 1.15, 1.35);
    auto unbound = std::make_shared<CentroidalJointProposal>(*first->selected);
    unbound->selected_problem.request.events.events[0].target_world = {};
    unbound->selected_problem.request.events.events[0].committed = false;
    first->selected = unbound;
    first->foot_request.problem = &first->selected->selected_problem;
    owner.Publish(first);
    Check(owner.Adopt(T(1.00), 101).status == OwnerStatus::kAdopted,
          "inflight setup proposal rejected");
    const auto before_liftoff = owner.Accepted();
    // The event was not active at the first adoption. Refresh must commit it
    // before the same pending pointer is returned as retained.
    Check(owner.Adopt(T(1.06), 102).status == OwnerStatus::kRetained,
          "same pending was not retained after lease refresh");
    const auto commitments = owner.CommittedEvents(T(1.06));
    Check(commitments.events.size() == 1 &&
              commitments.events.front().touchdown_time == T(1.15),
          "future touchdown commitment was not exported");
    Check(!before_liftoff->in_flight[0].valid,
          "lease refresh mutated an already published accepted snapshot");
    Check(owner.CommittedEvents(T(1.08)).valid(),
          "unbound selected event did not resolve exact combination target");
    auto second = ValidProposal(
        2, 1.02, 1.30, true, 1.05, 1.15, 1.35, true);
    second->foot_request.swing_clearance_m = 0.02;
    const auto old_sample = SampleFootTrajectoryAt(
        first->foot_request, T(1.08));
    owner.Publish(second);
    auto adopted = owner.Adopt(T(1.08), 103);
    Check(adopted.status == OwnerStatus::kAdopted &&
              adopted.accepted->execution_version == 2,
          "compatible replan was rejected");
    const auto sample = owner.SampleAt(T(1.08));
    Check(sample.valid && sample.bundle_valid && sample.execution_version == 2 &&
              old_sample.valid &&
              std::abs(sample.center_reference.center_world[0].value.z -
                       old_sample.samples.front().center_world[0].value.z) <
                  1.0e-12,
          "midflight curve was restarted by replan");
    Check(owner.CommittedEvents(T(1.20)).events.empty(),
          "post-touchdown event remained a future commitment");
    const auto stance = owner.SampleAt(T(1.20));
    Check(stance.valid && stance.curve_valid[0],
          "old curve lease was dropped before contact end");
}
void TestLeaseSurvivesExpiredFootHorizon()
{
    AtomicJointExecutionOwner owner;
    auto first = ValidProposal(
        1, 1.0, 1.30, true, 1.05, 1.25, 1.35, true);
    // Restrict only the old reference/body lease; keep its original physical
    // schedule valid and immutable, including the future touchdown.
    first->foot_request.end = T(1.20);
    first->valid_until = T(1.20);
    owner.Publish(first);
    Check(owner.Adopt(T(1.06), 101).status == OwnerStatus::kAdopted,
          "short foot-horizon proposal rejected");

    auto expected_request = first->foot_request;
    expected_request.end = T(1.30);
    const auto expected = SampleFootTrajectoryAt(expected_request, T(1.22));
    Check(expected.valid && expected.samples.size() == 1,
          "independent old swing extension was not prepared");

    auto second = ValidProposal(
        2, 1.02, 1.30, true, 1.05, 1.25, 1.35, true);
    owner.Publish(second);
    const auto adopted = owner.Adopt(T(1.22), 103);
    Check(adopted.status == OwnerStatus::kAdopted &&
              adopted.accepted && adopted.accepted->execution_version == 2,
          "valid replacement body bundle rejected after old expiry");
    const auto sample = owner.SampleAt(T(1.22));
    Check(sample.valid && sample.bundle_valid && sample.curve_valid[0],
          "old in-flight curve did not coexist with new body bundle");
    Check(std::abs(sample.center_reference.center_world[0].value.x -
                   expected.samples.front().center_world[0].value.x) < 1.0e-12 &&
              std::abs(sample.center_reference.center_world[0].value.z -
                   expected.samples.front().center_world[0].value.z) < 1.0e-12 &&
              sample.center_reference.center_world[0].source_time == T(1.0),
          "old swing polynomial was restarted or provenance changed");
}

void TestLeaseDoesNotExtendWholeBundle()
{
    AtomicJointExecutionOwner owner;
    auto proposal = ValidProposal(3, 1.0, 1.30, true, 1.05, 1.15, 1.35);
    proposal->valid_until = T(1.10);
    owner.Publish(proposal);
    Check(owner.Adopt(T(1.06), 103).status == OwnerStatus::kAdopted,
          "short-lived proposal rejected");
    const auto retained = owner.Adopt(T(1.20), 104);
    Check(retained.status == OwnerStatus::kStale,
          "expired body bundle reported executable retention");
    const auto curve_only = owner.SampleAt(T(1.20));
    Check(!curve_only.valid && !curve_only.bundle_valid &&
              curve_only.curve_valid[0],
          "lease incorrectly extended centroidal/body/force validity");
}
void TestCommittedEventComparisonIsStrict()
{
    TouchdownEvent old_event;
    old_event.id = {3, go2::Leg::FR, 1};
    old_event.liftoff_valid = true;
    old_event.liftoff_time = T(1.0);
    old_event.touchdown_time = T(1.1);
    old_event.contact_interval_end = T(1.3);
    old_event.target_world =
        Point(0.2, -0.1, 0.0, PointRole::kSurfaceContactPoint, T(1.0));
    TouchdownEvent same = old_event;
    TouchdownEvent changed = same;
    changed.touchdown_time = T(1.11);
    Check(SameCommittedEventCore(old_event, same),
          "identical committed event rejected");
    Check(!SameCommittedEventCore(old_event, changed),
          "retimed committed event accepted");
}
} // namespace
int main()
{
    try
    {
        TestNoPendingAndAtomicRetain();
        TestStaleDoesNotDropAccepted();
        TestCommandedHandoverNeverRewritesMeasuredInput();
        TestProposalInitializationOnlyBeforeFirstAcceptance();
        TestDelayedAdmissionPlanningPrefix();
        TestInflightLeaseRefreshAndReplan();
        TestLeaseSurvivesExpiredFootHorizon();
        TestLeaseDoesNotExtendWholeBundle();
        TestCommittedEventComparisonIsStrict();
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << "\n";
        return 1;
    }
    std::cout << "atomic joint execution owner checks passed\n";
    return 0;
}
