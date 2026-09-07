#pragma once
#include "anytime_joint_search.h"
#include "centroidal_subproblem.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>
#include <string>
namespace go2_terrain {
namespace stage_c {
// A diagnostic whole-combination result. The selected problem owns its request,
// candidate surfaces, schedule, grid and bounds; the selected result owns its
// states and forces. This bundle has no execution or command authority.
struct CentroidalJointProposal {
    JointPlanResult search{};
    CentroidalProblem selected_problem{};
    CentroidalResult selected_result{};
    bool selected_valid = false;
    std::size_t total_qp_iterations = 0;
    std::size_t total_scp_iterations = 0;
    // These fields follow the existing shadow telemetry: residual is the
    // maximum of position, velocity and momentum errors only.
    DynamicsResidual max_successful_residuals{};
    double max_successful_residual = 0.0;
    std::string last_solver_detail = "not_called";
};
// Solve each complete candidate combination once through the existing
// deterministic search. The callback retains the exact winner using the same
// cost then lexicographic candidate-index tie rule as the planner. Independent
// trajectory verification gates the retained bundle; it never reruns a solve.
inline CentroidalJointProposal SearchCentroidalJointProposal(
    const CentroidalProblem &problem,
    ExhaustivePlannerConfig planner_config = {}) {
    CentroidalJointProposal out;
    double selected_cost = std::numeric_limits<double>::infinity();
    std::vector<std::size_t> selected_indices;
    DeterministicBestFirstPlanner planner(planner_config);
    const auto record_success_residual = [&](const DynamicsResidual &r) {
        out.max_successful_residuals.position_m = std::max(
            out.max_successful_residuals.position_m, r.position_m);
        out.max_successful_residuals.velocity_mps = std::max(
            out.max_successful_residuals.velocity_mps, r.velocity_mps);
        out.max_successful_residuals.momentum_nms = std::max(
            out.max_successful_residuals.momentum_nms, r.momentum_nms);
        out.max_successful_residual = std::max({
            out.max_successful_residuals.position_m,
            out.max_successful_residuals.velocity_mps,
            out.max_successful_residuals.momentum_nms});
    };
    const auto evaluate =
        [&](const std::vector<std::size_t> &indices) -> JointEvaluation {
        CentroidalProblem candidate_problem = problem;
        candidate_problem.combination = indices;
        CentroidalResult candidate_result =
            SolveCentroidalSubproblem(candidate_problem);
        out.last_solver_detail = candidate_result.detail;
        if (candidate_result.qp_iterations > 0)
            out.total_qp_iterations +=
                static_cast<std::size_t>(candidate_result.qp_iterations);
        if (candidate_result.scp_iterations > 0)
            out.total_scp_iterations +=
                static_cast<std::size_t>(candidate_result.scp_iterations);

        // Preserve the existing reduced-problem transport and cost semantics.
        // AsJointEvaluation performs the existing independent certificate
        // check; a final explicit check gates only the bundle being exposed.
        JointEvaluation evaluation =
            AsJointEvaluation(candidate_problem, candidate_result);
        if (!evaluation.feasible)
            return evaluation;
        if (!std::isfinite(evaluation.cost)) {
            evaluation.feasible = false;
            evaluation.failure = JointPlannerFailure::kNumericalFailure;
            return evaluation;
        }
        const bool better = !out.selected_valid ||
            evaluation.cost < selected_cost ||
            (evaluation.cost == selected_cost && indices < selected_indices);
        if (!better) {
            record_success_residual(candidate_result.certificate.residual);
            return evaluation;
        }
        const DynamicsCertificate independent =
            VerifyCentroidalTrajectory(candidate_problem, candidate_result);
        if (!independent.feasible) {
            evaluation.feasible = false;
            evaluation.failure = JointPlannerFailure::kNumericalFailure;
            return evaluation;
        }
        record_success_residual(independent.residual);
        out.selected_problem = std::move(candidate_problem);
        out.selected_result = std::move(candidate_result);
        out.selected_valid = true;
        selected_cost = evaluation.cost;
        selected_indices = indices;
        return evaluation;
    };
    out.search = planner.Plan(problem.request, evaluate);
    if (!out.search.feasible) {
        // No valid proposal is exposed when the search preserves a failure.
        out.selected_valid = false;
        out.selected_problem = CentroidalProblem{};
        out.selected_result = CentroidalResult{};
        return out;
    }
    // This should agree by construction with DeterministicBestFirstPlanner.
    // Keep a fail-closed guard in case a future planner changes its transport.
    if (!out.selected_valid ||
        out.search.plan.candidate_indices != selected_indices ||
        out.search.plan.cost != selected_cost) {
        out.search.feasible = false;
        out.search.failure = JointPlannerFailure::kNumericalFailure;
        out.search.witness = {out.search.failure, 0, -1,
                              "selected_proposal_transport_mismatch"};
        out.selected_valid = false;
        out.selected_problem = CentroidalProblem{};
        out.selected_result = CentroidalResult{};
        out.max_successful_residuals = DynamicsResidual{};
        out.max_successful_residual = 0.0;
    }
    return out;
}

} // namespace stage_c
} // namespace go2_terrain
