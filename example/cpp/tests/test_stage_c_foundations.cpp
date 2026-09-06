#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "stage_c/input_adapter.h"
#include "stage_c/joint_planner.h"
#include "terrain_feasibility.h"
#include "terrain_planner.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1.0e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

go2_terrain::stage_c::RawPlanningObservation FixtureObservation()
{
    using namespace go2_terrain::stage_c;
    RawPlanningObservation raw;
    raw.identity = {11, TimeNs::FromSeconds(1.0), 7, 3, 0};
    raw.body.valid = true;
    raw.body.base_position_world = {
        {0.0, 0.0, 0.42}, Frame::kWorld, TimeNs::FromSeconds(1.0), true};
    raw.body.model_com_world = {
        {0.0, 0.0, 0.20}, Frame::kWorld, TimeNs::FromSeconds(1.0), true};
    raw.body.model_com_valid = true;
    raw.body.mass_kg = 12.0;
    raw.measured_contact.mask = {true, false, false, true};
    raw.measured_contact.provenance = ContactProvenance::kMeasured;
    raw.measured_contact.source_time = TimeNs::FromSeconds(1.0);
    raw.measured_contact.valid = true;
    raw.map.metadata_valid = true;
    raw.map.frame_id = "base_link";
    raw.map.source = go2_terrain::TerrainSource::kTestFixture;
    raw.map.epoch = 7;
    raw.map.acquisition_time = TimeNs::FromSeconds(0.99);
    raw.map.resolution_m = 0.05;
    raw.map.width = 8;
    raw.map.height = 8;
    raw.map.total_cells = 64;
    raw.map.known_cells = 64;
    raw.command.command_epoch = 5;
    raw.command.shaped_vx_mps = 1.0;
    raw.command.applied_vx_mps = 1.0;
    raw.command.period_s = 0.24;
    raw.command.duty_factor = 0.58;
    raw.command.valid = true;
    raw.budget.deadline_us = 5000.0;
    raw.budget.max_solver_iterations = 2;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        raw.feet[leg].foot_site_world = {
            {0.1 * static_cast<double>(leg), 0.0, 0.0}, Frame::kWorld,
            TimeNs::FromSeconds(1.0), true};
        raw.feet[leg].contact_patch_base = {
            {0.1 * static_cast<double>(leg), 0.0, -0.02}, Frame::kBase,
            TimeNs::FromSeconds(1.0), true};
    }
    raw.feet[0].measured_support_anchor_world = {
        {0.22, -0.10, -0.25}, Frame::kWorld, TimeNs::FromSeconds(1.0), true};
    raw.feet[3].measured_support_anchor_world = {
        {-0.22, 0.10, -0.25}, Frame::kWorld, TimeNs::FromSeconds(1.0), true};
    raw.feet[0].measured_support_anchor_valid = true;
    raw.feet[3].measured_support_anchor_valid = true;
    return raw;
}

go2_terrain::stage_c::TouchdownEvent Event(
    std::uint32_t sequence, go2::Leg leg, double time_s, double x,
    bool committed = false)
{
    using namespace go2_terrain::stage_c;
    const TimeNs time = TimeNs::FromSeconds(time_s);
    TouchdownEvent event;
    event.id = {3, leg, sequence};
    event.touchdown_time = time;
    event.contact_interval_end = TimeNs::FromSeconds(time_s + 0.12);
    event.target_world = {{x, 0.0, -0.22}, Frame::kWorld, time, true};
    event.source_plan_id = 4;
    event.committed = committed;
    return event;
}

go2_terrain::stage_c::EventCandidateSet CandidateSet(
    const go2_terrain::stage_c::TouchdownEvent &event)
{
    using namespace go2_terrain::stage_c;
    EventCandidateSet result;
    result.event_id = event.id;
    result.candidates = {
        {1, {{event.target_world.value.x, -0.04, event.target_world.value.z},
             Frame::kWorld, event.touchdown_time, true}, 1.0, 0.05, 0.02,
         MapCoverageState::kKnown, true},
        {2, {{event.target_world.value.x, 0.04, event.target_world.value.z},
             Frame::kWorld, event.touchdown_time, true}, 2.0, 0.05, 0.02,
         MapCoverageState::kKnown, true}};
    return result;
}

} // namespace

int main()
{
    using namespace go2_terrain::stage_c;
    bool passed = true;

    // T01: the mode bit must not gate a force-backed measured support anchor.
    auto shadow_raw = FixtureObservation();
    auto actuation_raw = shadow_raw;
    shadow_raw.capture_mode = CaptureMode::kShadow;
    actuation_raw.capture_mode = CaptureMode::kActuation;
    const auto shadow = NormalizePlanningInput(shadow_raw);
    const auto actuation = NormalizePlanningInput(actuation_raw);
    passed &= Check(shadow.ok && actuation.ok,
                    "T01 normalized inputs were not ready");
    passed &= Check(EquivalentPlannerInput(shadow.input, actuation.input),
                    "T01 shadow and actuation semantic inputs differ");
    passed &= Check(shadow.input.feet[0].contact_patch_world.value.x == 0.22 &&
                        shadow.input.feet[3].contact_patch_world.value.x == -0.22,
                    "T01 measured support anchors were not preserved");
    passed &= Check(!shadow.input.planned_contact.valid &&
                        !shadow.input.applied_contact.valid,
                    "T01 measured contacts were promoted to another provenance");
    auto missing_anchor_raw = shadow_raw;
    missing_anchor_raw.feet[0].measured_support_anchor_valid = false;
    const auto missing_anchor = NormalizePlanningInput(missing_anchor_raw);
    passed &= Check(!missing_anchor.ok &&
                        missing_anchor.failure ==
                            JointPlannerFailure::kObservationUnavailable,
                    "T01 missing measured anchor was not fail-closed");

    // T02: the existing region expression really degenerates at 5 cm/2.5 cm.
    const double exact_half = go2_terrain::SafeFootholdRegionHalfExtent(
        0.05, 0.025, 0.035);
    const double float_half = go2_terrain::SafeFootholdRegionHalfExtent(
        static_cast<double>(static_cast<float>(0.05)),
        static_cast<double>(static_cast<float>(0.025)), 0.035);
    passed &= Check(exact_half == 0.0,
                    "T02 exact 5 cm safe-region did not collapse to a point");
    passed &= Check(float_half >= 0.0 && float_half < 1.0e-8,
                    "T02 float32 safe-region round-off was mis-sized");

    // T03: metadata validity and observed coverage are independent facts.
    MapObservation map = shadow.input.map;
    map.known_cells = 0;
    map.outside_cells = 0;
    passed &= Check(ClassifyMapCoverage(map) == MapCoverageState::kUnknownInside,
                    "T03 all-unknown in-grid map was misclassified");
    map.outside_cells = map.total_cells;
    passed &= Check(ClassifyMapCoverage(map) == MapCoverageState::kOutsideGrid,
                    "T03 outside-grid map was misclassified");
    map.known_cells = map.total_cells;
    map.outside_cells = 0;
    passed &= Check(ClassifyMapCoverage(map) == MapCoverageState::kKnown,
                    "T03 fully known map was misclassified");
    passed &= Check(map.metadata_valid,
                    "T03 coverage fixture lost valid map metadata");

    // T04: heading-map XY and body FK use different transforms.
    const go2::Vec3 body_point{0.0, 0.0, -0.40};
    const auto yaw_only = RotateHeadingMapToWorld({}, 0.0, body_point);
    const auto full_body = RotateBodyToWorld(
        {}, 0.0, 5.0 * 3.14159265358979323846 / 180.0, 0.0, body_point);
    passed &= Check(std::abs(full_body.x - yaw_only.x) > 0.030,
                    "T04 full body pitch was lost by yaw-only transform");
    passed &= Check(Near(RotateHeadingMapToWorld({}, 0.5, {1.0, 0.0, 0.2}).z,
                         0.2),
                    "T04 heading-map height was not world-relative");

    // T05: every future touchdown has a distinct identity and target.
    TouchdownEventTable events;
    events.events = {Event(1, go2::Leg::FR, 0.20, 0.20),
                     Event(2, go2::Leg::FR, 0.44, 0.42),
                     Event(1, go2::Leg::FL, 0.32, 0.30)};
    std::sort(events.events.begin(), events.events.end(),
              [](const TouchdownEvent &a, const TouchdownEvent &b) {
                  return a.touchdown_time < b.touchdown_time;
              });
    passed &= Check(events.valid(),
                    "T05 repeated same-leg touchdown table was rejected");
    passed &= Check(events.events[0].id.leg == go2::Leg::FR &&
                        events.events[1].id.leg == go2::Leg::FL &&
                        events.events[2].id.sequence == 2 &&
                        events.events[0].target_world.value.x !=
                            events.events[2].target_world.value.x,
                    "T05 event identity or target was collapsed");
    auto duplicate_events = events;
    duplicate_events.events[1].id = duplicate_events.events[0].id;
    passed &= Check(!duplicate_events.valid(),
                    "T05 duplicate touchdown identity was accepted");

    // T06: coverage includes consumer interval ends, not just knot starts.
    std::vector<TimeNs> knots;
    for (int i = 0; i <= 23; ++i)
        knots.push_back(TimeNs{static_cast<std::int64_t>(i) * 20000000});
    const auto covered = CheckPredictionIntervalCoverage(
        knots, TimeNs::FromSeconds(0.10), TimeNs::FromSeconds(0.34));
    const auto uncovered = CheckPredictionIntervalCoverage(
        knots, TimeNs::FromSeconds(0.10), TimeNs::FromSeconds(0.51));
    passed &= Check(covered.covered && covered.available_end.value == 460000000,
                    "T06 complete interval coverage was not recognized");
    passed &= Check(!uncovered.covered && uncovered.required_end.value == 510000000,
                    "T06 horizon overrun was silently clamped");

    // T07: a frozen initial support conflict cannot be repaired by moving x0.
    const std::array<go2::Vec3, go2::kLegCount> feet{
        go2::Vec3{0.22, -0.10, 0.0}, go2::Vec3{}, go2::Vec3{},
        go2::Vec3{-0.22, 0.10, 0.0}};
    const std::array<bool, go2::kLegCount> contact{true, false, false, true};
    const double initial_margin = go2_terrain::SupportMargin2D(
        feet, contact, {0.0, 0.10, 0.0}, 0.015, 0.040);
    auto conflict_input = shadow.input;
    conflict_input.initial_support_margin_m = initial_margin;
    conflict_input.initial_support_margin_valid = true;
    passed &= Check(initial_margin < 0.015 &&
                        ValidateInitialCondition(conflict_input) ==
                            JointPlannerFailure::kInitialConditionConflict,
                    "T07 fixed initial support conflict was not classified");

    // T13: preserve the analyzer conflict as a contract witness.
    const auto transfer_conflict = ClassifyTransferContract(
        {true, 0, true});
    passed &= Check(transfer_conflict ==
                        FrozenContractConflict::
                            kTransferRequiresTwoContactsButAerialInterval,
                    "T13 aerial transfer/min-contact conflict disappeared");

    // T14: the frozen span field is solver-reference span, not supplied anchor
    // span or predicted COM span. Current legacy logging writes one reference
    // value to both endpoints; this fixture keeps those quantities separate.
    HorizontalReferenceTrace reference;
    reference.phase1_command_vx_mps = 1.0;
    reference.supplied_anchor_x_first_m = 0.0;
    reference.supplied_anchor_x_last_m = 0.40;
    reference.solver_reference_x_first_m = 0.10;
    reference.solver_reference_x_last_m = 0.10;
    reference.predicted_state_x_first_m = 0.10;
    reference.predicted_state_x_last_m = 0.16;
    passed &= Check(SolverHorizontalReferenceSpan(reference) == 0.0 &&
                        reference.supplied_anchor_x_last_m !=
                            reference.supplied_anchor_x_first_m &&
                        reference.predicted_state_x_last_m !=
                            reference.predicted_state_x_first_m,
                    "T14 reference span quantities were conflated");

    // T15: only the uncommitted suffix may be updated during replanning.
    TouchdownEventTable committed;
    committed.events = {Event(1, go2::Leg::FR, 0.20, 0.20, true),
                        Event(1, go2::Leg::FL, 0.32, 0.30, false)};
    TouchdownEventTable proposal = committed;
    proposal.events[1].touchdown_time = TimeNs::FromSeconds(0.36);
    proposal.events[1].target_world.value.x = 0.36;
    passed &= Check(committed.committed_prefix_compatible(proposal),
                    "T15 uncommitted schedule suffix could not be updated");
    proposal.events[0].target_world.value.x = 0.24;
    passed &= Check(!committed.committed_prefix_compatible(proposal),
                    "T15 committed touchdown was silently retimed");

    // C0-02: real deterministic multi-event exhaustive search and oracle.
    auto normalized = NormalizePlanningInput(shadow_raw);
    JointPlanningRequest request;
    request.input = normalized.input;
    request.events.events = {Event(1, go2::Leg::FR, 0.20, 0.20),
                             Event(1, go2::Leg::FL, 0.32, 0.30)};
    request.candidate_sets = {CandidateSet(request.events.events[0]),
                              CandidateSet(request.events.events[1])};
    const PlannerComparisonFixture comparison{
        request.input, request.events, request.candidate_sets};
    passed &= Check(comparison.events.events.size() == 2 &&
                        comparison.candidate_sets.size() == 2 &&
                        EquivalentPlannerInput(comparison.input, request.input),
                    "C0-02 planner comparison seam changed the shared input");
    const auto evaluator = [](const std::vector<std::size_t> &indices) {
        JointEvaluation evaluation;
        if (indices == std::vector<std::size_t>{0, 0})
        {
            evaluation.failure = JointPlannerFailure::kNoFeasibleCandidateInSet;
            return evaluation;
        }
        evaluation.feasible = true;
        evaluation.cost = indices == std::vector<std::size_t>{0, 1}
            ? 5.0 : (indices == std::vector<std::size_t>{1, 0} ? 3.0 : 4.0);
        return evaluation;
    };
    const auto oracle = ExhaustiveOracle(request, evaluator);
    const auto planner = DeterministicExhaustivePlanner{}.Plan(request, evaluator);
    passed &= Check(oracle.feasible && planner.feasible &&
                        oracle.plan.candidate_indices ==
                            std::vector<std::size_t>{1, 0} &&
                        planner.plan.candidate_indices == oracle.plan.candidate_indices &&
                        planner.diagnostics.combinations_considered == 4,
                    "C0-02 exhaustive joint search/oracle disagreed");

    const auto bounded = DeterministicExhaustivePlanner{
        {2}}.Plan(request, [](const std::vector<std::size_t> &indices) {
            JointEvaluation evaluation;
            evaluation.feasible = indices == std::vector<std::size_t>{1, 1};
            evaluation.cost = evaluation.feasible ? 1.0 :
                std::numeric_limits<double>::infinity();
            return evaluation;
        });
    passed &= Check(!bounded.feasible &&
                        bounded.failure == JointPlannerFailure::kBudgetExhausted,
                    "C0-02 budget exhaustion was mislabeled as no solution");
    auto no_candidate = request;
    no_candidate.candidate_sets[1].candidates[0].geometry_hard_feasible = false;
    no_candidate.candidate_sets[1].candidates[1].geometry_hard_feasible = false;
    const auto no_candidate_result =
        ExhaustiveOracle(no_candidate, evaluator);
    passed &= Check(!no_candidate_result.feasible &&
                        no_candidate_result.failure ==
                            JointPlannerFailure::kNoFeasibleCandidateInSet,
                    "C0-02 empty candidate set taxonomy was lost");

    auto incomplete = request;
    incomplete.candidate_sets[0].complete = false;
    const auto incomplete_result = ExhaustiveOracle(incomplete, evaluator);
    passed &= Check(!incomplete_result.feasible &&
                        incomplete_result.failure ==
                            JointPlannerFailure::kSearchIncomplete,
                    "C0-02 truncated candidate set was mislabeled");
    const auto numerical_result = ExhaustiveOracle(
        request, [](const std::vector<std::size_t> &) {
            JointEvaluation evaluation;
            evaluation.failure = JointPlannerFailure::kNumericalFailure;
            return evaluation;
        });
    passed &= Check(!numerical_result.feasible &&
                        numerical_result.failure ==
                            JointPlannerFailure::kNumericalFailure,
                    "C0-02 numerical failure taxonomy was lost");

    if (!passed)
        return 1;
    std::cout << "Stage C C0-01 fixtures and C0-02 discrete foundation checks passed.\n";
    return 0;
}
