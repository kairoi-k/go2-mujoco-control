#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "joint_planner.h"
namespace go2_terrain
{
namespace stage_c
{
// A bounded best-first search over the complete joint candidate product. The
// foothold costs order the queue only; no cost bound is used to prune a node,
// so the evaluator remains the sole source of physical feasibility.
class DeterministicBestFirstPlanner final : public JointPlannerInterface
{
public:
    explicit DeterministicBestFirstPlanner(
        ExhaustivePlannerConfig config = {})
        : config_(config)
    {
    }
    JointPlanResult Plan(
        const JointPlanningRequest &request,
        const JointEvaluationFunction &evaluate) const override
    {
        JointPlanResult result;
        if (!request.input.basic_valid() ||
            (!request.events.events.empty() && !request.events.valid(false)) ||
            request.candidate_sets.size() != request.events.events.size())
        {
            result.failure = JointPlannerFailure::kObservationUnavailable;
            result.witness.failure = result.failure;
            result.witness.detail = "input_or_event_table_not_ready";
            return result;
        }
        if (!request.accepted_commitments.events.empty() &&
            !request.accepted_commitments.committed_prefix_compatible(
                request.events))
        {
            result.failure = JointPlannerFailure::kCommitmentConflict;
            result.witness.failure = result.failure;
            result.witness.detail = "accepted_touchdown_prefix_changed";
            return result;
        }
        std::vector<std::vector<std::size_t>> ranked_candidates;
        ranked_candidates.reserve(request.candidate_sets.size());
        for (std::size_t event = 0;
             event < request.candidate_sets.size(); ++event)
        {
            const auto &set = request.candidate_sets[event];
            if (!(set.event_id == request.events.events[event].id))
            {
                result.failure = JointPlannerFailure::kInvalidInput;
                result.witness = {result.failure, event, -1,
                                  "candidate_set_event_identity_mismatch"};
                return result;
            }
            if (!set.complete)
            {
                result.failure = JointPlannerFailure::kSearchIncomplete;
                result.witness = {result.failure, event, -1,
                                  "candidate_set_truncated"};
                return result;
            }
            std::vector<std::size_t> order;
            order.reserve(set.candidates.size());
            for (std::size_t candidate = 0;
                 candidate < set.candidates.size(); ++candidate)
            {
                const auto &foothold = set.candidates[candidate];
                if (!foothold.geometry_hard_feasible)
                {
                    ++result.diagnostics.combinations_pruned;
                    continue;
                }
                if (!foothold.valid())
                {
                    result.failure = JointPlannerFailure::kInvalidInput;
                    result.witness = {result.failure, event,
                                      static_cast<int>(candidate),
                                      "hard_candidate_malformed"};
                    return result;
                }
                if (foothold.coverage != MapCoverageState::kKnown)
                {
                    result.failure = JointPlannerFailure::kCoverageIncomplete;
                    result.witness = {result.failure, event,
                                      static_cast<int>(candidate),
                                      "hard_candidate_coverage_unknown"};
                    return result;
                }
                order.push_back(candidate);
            }
            if (order.empty())
            {
                result.failure = JointPlannerFailure::kNoFeasibleCandidateInSet;
                result.witness = {result.failure, event, -1,
                                  "event_has_no_hard_feasible_candidate"};
                return result;
            }
            std::stable_sort(
                order.begin(), order.end(), [&](std::size_t left,
                                                std::size_t right) {
                    const double left_cost =
                        set.candidates[left].foothold_cost;
                    const double right_cost =
                        set.candidates[right].foothold_cost;
                    if (left_cost != right_cost)
                        return left_cost < right_cost;
                    return left < right;
                });
            ranked_candidates.push_back(std::move(order));
        }
        if (!evaluate)
        {
            result.failure = JointPlannerFailure::kInvalidInput;
            result.witness = {result.failure, 0, -1, "missing_evaluator"};
            return result;
        }
        const std::size_t request_budget = request.input.budget
            .max_candidate_combinations;
        const std::size_t configured_budget = config_.max_combinations;
        const std::size_t budget = configured_budget != 0
            ? configured_budget : request_budget;
        struct SearchNode
        {
            std::vector<std::size_t> ranks;
            std::vector<std::size_t> candidate_indices;
            long double priority = 0.0L;
            std::size_t serial = 0;
        };
        struct SearchNodeGreater
        {
            bool operator()(const SearchNode &left,
                            const SearchNode &right) const
            {
                if (left.priority != right.priority)
                    return left.priority > right.priority;
                return left.serial > right.serial;
            }
        };
        std::priority_queue<SearchNode, std::vector<SearchNode>,
                            SearchNodeGreater>
            queue;
        std::set<std::vector<std::size_t>> queued;
        SearchNode first;
        first.ranks.assign(ranked_candidates.size(), 0);
        first.candidate_indices.reserve(ranked_candidates.size());
        for (std::size_t event = 0; event < ranked_candidates.size();
             ++event)
        {
            const std::size_t candidate = ranked_candidates[event][0];
            first.candidate_indices.push_back(candidate);
            first.priority += static_cast<long double>(
                request.candidate_sets[event].candidates[candidate]
                    .foothold_cost);
        }
        first.serial = 0;
        queued.insert(first.ranks);
        queue.push(first);
        bool budget_exhausted = false;
        bool saw_numerical_failure = false;
        JointPlannerFailure unresolved = JointPlannerFailure::kNone;
        std::size_t next_serial = 1;
        while (!queue.empty())
        {
            if (budget != 0 &&
                result.diagnostics.combinations_considered >= budget)
            {
                budget_exhausted = true;
                break;
            }
            SearchNode node = queue.top();
            queue.pop();
            ++result.diagnostics.combinations_considered;
            JointEvaluation evaluation = evaluate(node.candidate_indices);
            if (evaluation.feasible)
            {
                if (!std::isfinite(evaluation.cost))
                {
                    saw_numerical_failure = true;
                }
                else
                {
                    const bool better = !result.feasible ||
                        evaluation.cost < result.plan.cost ||
                        (evaluation.cost == result.plan.cost &&
                         node.candidate_indices <
                             result.plan.candidate_indices);
                    if (better)
                    {
                        result.feasible = true;
                        result.failure = JointPlannerFailure::kNone;
                        result.plan = evaluation.plan;
                        result.plan.candidate_indices =
                            node.candidate_indices;
                        result.plan.cost = evaluation.cost;
                    }
                }
            }
            else
            {
                saw_numerical_failure = saw_numerical_failure ||
                    evaluation.failure == JointPlannerFailure::kNumericalFailure;
                if (evaluation.failure != JointPlannerFailure::kDynamicsInfeasible &&
                    evaluation.failure !=
                        JointPlannerFailure::kNoFeasibleCandidateInSet &&
                    unresolved == JointPlannerFailure::kNone)
                    unresolved = evaluation.failure == JointPlannerFailure::kNone
                        ? JointPlannerFailure::kNumericalFailure
                        : evaluation.failure;
            }
            // Expand in the event table's chronological order. Stable serial
            // ordering makes equal summed foothold costs reproducible.
            for (std::size_t event = 0; event < node.ranks.size(); ++event)
            {
                const std::size_t next_rank = node.ranks[event] + 1;
                if (next_rank >= ranked_candidates[event].size())
                    continue;
                SearchNode neighbor;
                neighbor.ranks = node.ranks;
                neighbor.ranks[event] = next_rank;
                if (!queued.insert(neighbor.ranks).second)
                    continue;
                neighbor.candidate_indices = node.candidate_indices;
                neighbor.candidate_indices[event] =
                    ranked_candidates[event][next_rank];
                neighbor.priority = node.priority - static_cast<long double>(
                    request.candidate_sets[event]
                        .candidates[node.candidate_indices[event]]
                        .foothold_cost);
                neighbor.priority += static_cast<long double>(
                    request.candidate_sets[event]
                        .candidates[neighbor.candidate_indices[event]]
                        .foothold_cost);
                neighbor.serial = next_serial++;
                queue.push(std::move(neighbor));
            }
        }
        result.diagnostics.search_complete = !budget_exhausted && queue.empty();
        if (budget_exhausted)
        {
            result.failure = result.feasible
                ? JointPlannerFailure::kNone
                : JointPlannerFailure::kBudgetExhausted;
            if (!result.feasible)
                result.witness = {result.failure, 0, -1,
                                  "combination_budget_reached"};
            return result;
        }
        if (!result.feasible)
        {
            result.failure = saw_numerical_failure
                ? JointPlannerFailure::kNumericalFailure
                : (unresolved != JointPlannerFailure::kNone ? unresolved
                    : JointPlannerFailure::kNoFeasibleCandidateInSet);
            result.witness = {result.failure, 0, -1,
                              saw_numerical_failure
                                  ? "continuous_evaluation_failed"
                                  : "all_joint_combinations_rejected"};
        }
        return result;
    }
private:
    ExhaustivePlannerConfig config_{};
};
} // namespace stage_c
} // namespace go2_terrain
