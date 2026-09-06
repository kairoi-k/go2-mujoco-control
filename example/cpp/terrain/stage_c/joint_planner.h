#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <utility>

#include "types.h"

namespace go2_terrain
{
namespace stage_c
{

struct PredictionCoverageCheck
{
    bool covered = false;
    TimeNs required_start{};
    TimeNs required_end{};
    TimeNs available_end{};
};

inline PredictionCoverageCheck CheckPredictionIntervalCoverage(
    const std::vector<TimeNs> &state_knots, TimeNs required_start,
    TimeNs required_end)
{
    PredictionCoverageCheck result;
    result.required_start = required_start;
    result.required_end = required_end;
    if (state_knots.empty() || required_end < required_start)
        return result;
    for (std::size_t i = 1; i < state_knots.size(); ++i)
        if (state_knots[i] < state_knots[i - 1])
            return result;
    result.available_end = state_knots.back();
    result.covered = state_knots.front() <= required_start &&
        state_knots.back() >= required_end;
    return result;
}

inline JointPlannerFailure ValidateInitialCondition(
    const TerrainPlanningInput &input, double min_support_margin_m = 0.015)
{
    if (!input.basic_valid())
        return JointPlannerFailure::kObservationUnavailable;
    if (!std::isfinite(min_support_margin_m) || min_support_margin_m < 0.0)
        return JointPlannerFailure::kInvalidInput;
    if (input.initial_support_margin_valid &&
        (!std::isfinite(input.initial_support_margin_m) ||
         input.initial_support_margin_m < min_support_margin_m))
        return JointPlannerFailure::kInitialConditionConflict;
    return JointPlannerFailure::kNone;
}

enum class FrozenContractConflict : std::uint8_t
{
    kNone = 0,
    kTransferRequiresTwoContactsButAerialInterval,
    kTransferContactMismatch,
    kReferenceFieldNotSolverHorizon,
};

struct TransferContractObservation
{
    bool transfer_sample = false;
    int min_contacts = 0;
    bool contains_aerial_interval = false;
};

inline FrozenContractConflict ClassifyTransferContract(
    const TransferContractObservation &observation)
{
    if (!observation.transfer_sample)
        return FrozenContractConflict::kNone;
    if (observation.min_contacts < 2 && observation.contains_aerial_interval)
        return FrozenContractConflict::
            kTransferRequiresTwoContactsButAerialInterval;
    if (observation.min_contacts < 2)
        return FrozenContractConflict::kTransferContactMismatch;
    return FrozenContractConflict::kNone;
}

struct JointPlanningRequest
{
    TerrainPlanningInput input{};
    TouchdownEventTable events{};
    std::vector<EventCandidateSet> candidate_sets;
    TouchdownEventTable accepted_commitments{};
};

struct JointEvaluation
{
    bool feasible = false;
    double cost = std::numeric_limits<double>::infinity();
    JointPlannerFailure failure = JointPlannerFailure::kNone;
    PlanCandidate plan{};
};

struct JointPlannerDiagnostics
{
    std::size_t combinations_considered = 0;
    std::size_t combinations_pruned = 0;
    bool search_complete = false;
};

struct JointPlanResult
{
    bool feasible = false;
    JointPlannerFailure failure = JointPlannerFailure::kNone;
    FailureWitness witness{};
    PlanCandidate plan{};
    JointPlannerDiagnostics diagnostics{};
};

using JointEvaluationFunction = std::function<JointEvaluation(
    const std::vector<std::size_t> &candidate_indices)>;

class JointPlannerInterface
{
public:
    virtual ~JointPlannerInterface() = default;
    virtual JointPlanResult Plan(
        const JointPlanningRequest &request,
        const JointEvaluationFunction &evaluate) const = 0;
};

struct ExhaustivePlannerConfig
{
    // Zero means exhaustive. A nonzero value is a hard search budget and a
    // miss is reported as budget_exhausted, never as physical infeasibility.
    std::size_t max_combinations = 0;
};

class DeterministicExhaustivePlanner final : public JointPlannerInterface
{
public:
    explicit DeterministicExhaustivePlanner(ExhaustivePlannerConfig config = {})
        : config_(config)
    {
    }

    JointPlanResult Plan(
        const JointPlanningRequest &request,
        const JointEvaluationFunction &evaluate) const override
    {
        JointPlanResult result;
        if (!request.input.basic_valid() || !request.events.valid() ||
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
        for (std::size_t event = 0; event < request.candidate_sets.size(); ++event)
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
            if (!set.has_hard_candidate())
            {
                result.failure = JointPlannerFailure::kNoFeasibleCandidateInSet;
                result.witness = {result.failure, event, -1,
                                  "event_has_no_hard_feasible_candidate"};
                return result;
            }
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
        std::vector<std::size_t> current(request.candidate_sets.size(), 0);
        bool budget_exhausted = false;
        bool saw_numerical_failure = false;
        bool saw_other_failure = false;

        std::function<void(std::size_t)> visit = [&](std::size_t event) {
            if (budget_exhausted)
                return;
            if (event == request.candidate_sets.size())
            {
                if (budget != 0 &&
                    result.diagnostics.combinations_considered >= budget)
                {
                    budget_exhausted = true;
                    return;
                }
                ++result.diagnostics.combinations_considered;
                std::vector<std::size_t> selected = current;
                JointEvaluation evaluation = evaluate(selected);
                if (!evaluation.feasible)
                {
                    saw_numerical_failure = saw_numerical_failure ||
                        evaluation.failure == JointPlannerFailure::kNumericalFailure;
                    saw_other_failure = saw_other_failure ||
                        evaluation.failure != JointPlannerFailure::kNumericalFailure;
                    return;
                }
                if (!std::isfinite(evaluation.cost))
                {
                    saw_numerical_failure = true;
                    return;
                }
                const bool better = !result.feasible ||
                    evaluation.cost < result.plan.cost ||
                    (evaluation.cost == result.plan.cost &&
                     selected < result.plan.candidate_indices);
                if (better)
                {
                    result.feasible = true;
                    result.failure = JointPlannerFailure::kNone;
                    result.plan = evaluation.plan;
                    result.plan.candidate_indices = std::move(selected);
                    result.plan.cost = evaluation.cost;
                }
                return;
            }
            const auto &set = request.candidate_sets[event];
            for (std::size_t candidate = 0;
                 candidate < set.candidates.size(); ++candidate)
            {
                if (!set.candidates[candidate].geometry_hard_feasible)
                {
                    ++result.diagnostics.combinations_pruned;
                    continue;
                }
                current[event] = candidate;
                visit(event + 1);
                if (budget_exhausted)
                    return;
            }
        };
        visit(0);
        result.diagnostics.search_complete = !budget_exhausted;
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
            result.failure = saw_numerical_failure && !saw_other_failure
                ? JointPlannerFailure::kNumericalFailure
                : JointPlannerFailure::kNoFeasibleCandidateInSet;
            result.witness = {result.failure, 0, -1,
                              saw_numerical_failure && !saw_other_failure
                                  ? "continuous_evaluation_failed"
                                  : "all_joint_combinations_rejected"};
        }
        return result;
    }

private:
    ExhaustivePlannerConfig config_{};
};

inline JointPlanResult ExhaustiveOracle(
    const JointPlanningRequest &request, const JointEvaluationFunction &evaluate)
{
    return DeterministicExhaustivePlanner{}.Plan(request, evaluate);
}

} // namespace stage_c
} // namespace go2_terrain
