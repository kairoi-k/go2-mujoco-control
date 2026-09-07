#include "stage_c/anytime_joint_search.h"
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
TimedPoint Point(double x, double y, double z = 0.0)
{
    return {{x, y, z}, Frame::kWorld, T(1.0), true,
            PointRole::kSurfaceContactPoint};
}
JointPlanningRequest Request(
    const std::vector<std::vector<double>> &costs)
{
    JointPlanningRequest request;
    request.input.identity = {11, T(1.0), 7, 3, 0};
    request.input.body.valid = true;
    request.input.body.model_com_valid = true;
    request.input.body.base_position_world =
        {{0.0, 0.0, 0.4}, Frame::kWorld, T(1.0), true,
         PointRole::kBodyOrigin};
    request.input.body.model_com_world =
        {{0.0, 0.0, 0.4}, Frame::kWorld, T(1.0), true,
         PointRole::kCenterOfMass};
    request.input.body.mass_kg = 10.0;
    request.input.measured_contact.valid = true;
    request.input.measured_contact.provenance = ContactProvenance::kMeasured;
    request.input.measured_contact.source_time = T(1.0);
    request.input.map.metadata_valid = true;
    request.input.map.epoch = 7;
    request.input.map.coverage = MapCoverageState::kKnown;
    request.input.map.width = request.input.map.height = 8;
    request.input.map.total_cells = request.input.map.known_cells = 64;
    for (std::size_t event = 0; event < costs.size(); ++event)
    {
        TouchdownEvent touchdown;
        touchdown.id = {3, go2::Leg::FR,
                        static_cast<std::uint32_t>(event + 1)};
        touchdown.touchdown_time = T(1.1 + 0.01 * event);
        touchdown.contact_interval_end = T(1.2 + 0.01 * event);
        touchdown.target_world = Point(0.1 * event, -0.1);
        request.events.events.push_back(touchdown);
        EventCandidateSet set;
        set.event_id = touchdown.id;
        set.complete = true;
        for (std::size_t candidate = 0; candidate < costs[event].size();
             ++candidate)
        {
            StageCCandidate foothold;
            foothold.candidate_id = static_cast<std::uint32_t>(candidate + 1);
            foothold.target_world =
                Point(0.1 * event, -0.1 + 0.01 * candidate);
            foothold.foothold_cost = costs[event][candidate];
            foothold.coverage = MapCoverageState::kKnown;
            foothold.geometry_hard_feasible = true;
            set.candidates.push_back(foothold);
        }
        request.candidate_sets.push_back(std::move(set));
    }
    return request;
}
JointEvaluation Feasible(double cost)
{
    JointEvaluation evaluation;
    evaluation.feasible = true;
    evaluation.cost = cost;
    return evaluation;
}
} // namespace
int main()
{
    try
    {
        auto small = Request({{2.0, 1.0, 1.0}, {0.0, 3.0}});
        const auto evaluator = [](const std::vector<std::size_t> &indices) {
            if (indices == std::vector<std::size_t>{2, 1})
                return Feasible(0.0);
            if (indices == std::vector<std::size_t>{1, 0})
                return Feasible(2.0);
            return Feasible(5.0 + indices[0] + indices[1]);
        };
        const auto exhaustive = ExhaustiveOracle(small, evaluator);
        const auto best = DeterministicBestFirstPlanner{}.Plan(small, evaluator);
        Check(exhaustive.feasible && best.feasible &&
                  best.diagnostics.search_complete &&
                  best.diagnostics.combinations_considered == 6 &&
                  best.plan.candidate_indices ==
                      exhaustive.plan.candidate_indices &&
                  best.plan.cost == exhaustive.plan.cost,
              "unbounded best-first differs from exhaustive oracle");
        auto ties = Request({{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}});
        std::vector<std::vector<std::size_t>> order;
        const auto tie_evaluator =
            [&](const std::vector<std::size_t> &indices) {
                order.push_back(indices);
                return Feasible(1.0);
            };
        const auto tie_result =
            DeterministicBestFirstPlanner{}.Plan(ties, tie_evaluator);
        Check(tie_result.feasible && tie_result.diagnostics.search_complete &&
                  tie_result.diagnostics.combinations_considered == 8 &&
                  tie_result.plan.candidate_indices ==
                      std::vector<std::size_t>{0, 0, 0} && order.size() == 8 &&
                  order[0] == std::vector<std::size_t>{0, 0, 0} &&
                  order[1] == std::vector<std::size_t>{1, 0, 0} &&
                  order[2] == std::vector<std::size_t>{0, 1, 0},
              "equal-cost queue or lexicographic tie order is unstable");
        std::vector<std::vector<std::size_t>> repeated_order;
        const auto repeated = DeterministicBestFirstPlanner{}.Plan(
            ties, [&](const std::vector<std::size_t> &indices) {
                repeated_order.push_back(indices);
                return Feasible(1.0);
            });
        Check(repeated.plan.candidate_indices == tie_result.plan.candidate_indices &&
                  repeated_order == order,
              "best-first ties were not deterministic");
        auto early = Request({{0.0, 1.0}, {0.0, 100.0}, {0.0, 100.0},
                              {0.0, 100.0}, {0.0, 100.0}, {0.0, 100.0},
                              {0.0, 100.0}, {0.0, 100.0}, {0.0, 100.0},
                              {0.0, 100.0}, {0.0, 100.0}, {0.0, 100.0}});
        std::vector<std::vector<std::size_t>> early_order;
        const auto early_result = DeterministicBestFirstPlanner{{2}}.Plan(
            early, [&](const std::vector<std::size_t> &indices) {
                early_order.push_back(indices);
                return indices[0] == 1 ? Feasible(1.0)
                                       : JointEvaluation{};
            });
        Check(early_result.feasible && !early_result.diagnostics.search_complete &&
                  early_result.diagnostics.combinations_considered == 2 &&
                  early_result.plan.candidate_indices[0] == 1 &&
                  early_order.size() == 2 &&
                  early_order[1][0] == 1,
              "budget did not reach an early event alternative");
        auto missing_input = early;
        missing_input.input.body.valid = false;
        Check(DeterministicBestFirstPlanner{}.Plan(missing_input, evaluator)
                      .failure == JointPlannerFailure::kObservationUnavailable,
              "missing input classification changed");
        auto malformed = small;
        malformed.candidate_sets[0].candidates[0].target_world.role =
            PointRole::kUnknown;
        Check(DeterministicBestFirstPlanner{}.Plan(malformed, evaluator)
                      .failure == JointPlannerFailure::kInvalidInput,
              "malformed hard candidate classification changed");
        auto unknown_coverage = small;
        unknown_coverage.candidate_sets[0].candidates[0].coverage =
            MapCoverageState::kUnknownInside;
        Check(DeterministicBestFirstPlanner{}.Plan(unknown_coverage, evaluator)
                      .failure == JointPlannerFailure::kCoverageIncomplete,
              "unknown hard candidate coverage classification changed");
        auto commitment = small;
        commitment.events.events[0].committed = true;
        commitment.accepted_commitments.events = {commitment.events.events[0]};
        commitment.events.events[0].target_world.value.x += 0.1;
        Check(DeterministicBestFirstPlanner{}.Plan(commitment, evaluator)
                      .failure == JointPlannerFailure::kCommitmentConflict,
              "commitment conflict classification changed");
        auto identity = small;
        identity.candidate_sets[0].event_id.sequence++;
        Check(DeterministicBestFirstPlanner{}.Plan(identity, evaluator).failure ==
                  JointPlannerFailure::kInvalidInput,
              "candidate identity classification changed");
        auto incomplete = small;
        incomplete.candidate_sets[0].complete = false;
        Check(DeterministicBestFirstPlanner{}.Plan(incomplete, evaluator).failure ==
                  JointPlannerFailure::kSearchIncomplete,
              "truncated candidate classification changed");
        Check(DeterministicBestFirstPlanner{}.Plan(small, {})
                      .failure == JointPlannerFailure::kInvalidInput,
              "missing evaluator classification changed");
        const auto dynamics = DeterministicBestFirstPlanner{}.Plan(
            small, [](const std::vector<std::size_t> &) {
                JointEvaluation evaluation;
                evaluation.failure = JointPlannerFailure::kDynamicsInfeasible;
                return evaluation;
            });
        const auto dynamics_oracle = ExhaustiveOracle(
            small, [](const std::vector<std::size_t> &) {
                JointEvaluation evaluation;
                evaluation.failure = JointPlannerFailure::kDynamicsInfeasible;
                return evaluation;
            });
        Check(!dynamics.feasible && dynamics.diagnostics.search_complete &&
                  dynamics.failure == dynamics_oracle.failure,
              "dynamics failure classification differs from exhaustive oracle");
        const auto numerical = DeterministicBestFirstPlanner{}.Plan(
            small, [](const std::vector<std::size_t> &) {
                JointEvaluation evaluation;
                evaluation.failure = JointPlannerFailure::kNumericalFailure;
                return evaluation;
            });
        Check(numerical.failure == JointPlannerFailure::kNumericalFailure,
              "numerical failure was not preserved");
        std::cout << "Stage C deterministic best-first joint search checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
