#include "stage_c/centroidal_joint_proposal.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace go2_terrain::stage_c;
namespace {
void Check(bool value, const char *message)
{
    if (!value)
        throw std::runtime_error(message);
}
TimeNs T(double seconds)
{
    return TimeNs::FromSeconds(seconds);
}
TimedPoint Point(double x, double y, double z, PointRole role)
{
    return {{x, y, z}, Frame::kWorld, T(1.0), true, role};
}
ContactSurface Surface()
{
    ContactSurface surface;
    surface.frame = Frame::kWorld;
    surface.coverage = MapCoverageState::kKnown;
    surface.map_epoch = 7;
    surface.valid_until = T(2.0);
    surface.friction_mu = 0.8;
    surface.min_normal_n = 0.0;
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
    input.body.base_velocity_world = {0.0, 0.0, 0.0};
    input.body.com_velocity_world = {0.0, 0.0, 0.0};
    input.body.mass_kg = problem.model.mass_kg = 10.0;
    input.measured_contact.mask = {{false, false, false, true}};
    input.measured_contact.valid = true;
    input.measured_contact.provenance = ContactProvenance::kMeasured;
    input.measured_contact.source_time = T(1.0);
    input.feet[3].measured_support_anchor_world =
        Point(-0.2, 0.1, 0.0, PointRole::kSurfaceContactPoint);
    input.feet[3].measured_support_anchor_valid = true;
    input.map.metadata_valid = true;
    input.map.epoch = 7;
    input.map.coverage = MapCoverageState::kKnown;
    input.map.width = input.map.height = 8;
    input.map.total_cells = input.map.known_cells = 64;
    input.command.valid = true;
    input.command.command_epoch = 4;
    input.command.shaped_vx_mps = input.command.applied_vx_mps = 0.0;
    input.command.shaped_vy_mps = input.command.applied_vy_mps = 0.0;
    input.command.period_s = 0.24;
    input.command.duty_factor = 0.4;
    problem.initial_momentum_valid = true;
    problem.initial_momentum_world.setZero();
    problem.schedule_epoch = 3;
    problem.grid = {T(1.0), T(1.02)};
    problem.bounds.resize(problem.grid.size());
    for (auto &bound : problem.bounds)
    {
        bound.lower << -1.0, -1.0, 0.1, -20.0, -20.0, -20.0,
            -10.0, -10.0, -10.0;
        bound.upper << 1.0, 1.0, 1.0, 20.0, 20.0, 20.0,
            10.0, 10.0, 10.0;
    }
    problem.required_start = problem.grid.front();
    problem.required_end = problem.grid.back();
    input.budget.prediction_start = problem.grid.front();
    input.budget.prediction_end = problem.grid.back();
    for (auto &surface : problem.initial_surfaces)
        surface = Surface();
    // Pin terminal vertical velocity and angular momentum. With measured RL
    // support capped at 60 N, the low candidate's 1 N cap has total normal
    // capacity below mg and must fail by dynamics separation.
    problem.initial_surfaces[3].max_normal_n = 60.0;
    for (auto &bound : problem.bounds) {
        bound.lower.tail<3>().setZero();
        bound.upper.tail<3>().setZero();
    }
    problem.bounds[1].lower.segment<3>(3).setZero();
    problem.bounds[1].upper.segment<3>(3).setZero();
    TouchdownEvent event;
    event.id = {3, go2::Leg::FR, 1};
    event.touchdown_time = T(1.0);
    event.contact_interval_end = T(1.02);
    event.target_world =
        Point(0.2, -0.1, 0.0, PointRole::kSurfaceContactPoint);
    problem.request.events.events.push_back(event);
    EventCandidateSet candidates;
    candidates.event_id = event.id;
    candidates.complete = true;
    for (std::size_t index = 0; index < 2; ++index)
    {
        StageCCandidate candidate;
        candidate.candidate_id = static_cast<std::uint32_t>(index + 1);
        candidate.target_world = event.target_world;
        candidate.foothold_cost = 1.0;
        candidate.coverage = MapCoverageState::kKnown;
        candidate.geometry_hard_feasible = true;
        candidates.candidates.push_back(candidate);
    }
    problem.request.candidate_sets.push_back(candidates);
    problem.combination = {0};
    problem.candidate_surfaces = {{Surface(), Surface()}};
    problem.candidate_surfaces[0][0].max_normal_n = 1.0;
    problem.schedule.push_back({
        T(1.0), T(1.02), {{true, false, false, true}}, {{0, -1, -1, -1}}});
    return problem;
}
} // namespace
int main()
{
    try
    {
        auto problem = Fixture();
        const auto low = SolveCentroidalSubproblem(problem);
        Check(!low.certificate.feasible &&
                  low.failure == JointPlannerFailure::kDynamicsInfeasible,
              "low-capacity alternative must be dynamically infeasible");
        auto proposal = SearchCentroidalJointProposal(problem);
        Check(proposal.search.feasible && proposal.selected_valid,
              "feasible combination was not retained");
        Check(proposal.search.plan.candidate_indices ==
                  std::vector<std::size_t>{1},
              "winner candidate indices changed");
        Check(proposal.selected_problem.combination ==
                  proposal.search.plan.candidate_indices,
              "owned problem does not match planner winner");
        Check(proposal.selected_result.certificate.feasible,
              "owned result is not independently certified");
        Check(VerifyCentroidalTrajectory(
                  proposal.selected_problem, proposal.selected_result).feasible,
              "independent verification of owned result failed");
        Check(proposal.total_scp_iterations > 0 &&
                  std::isfinite(proposal.max_successful_residual) &&
                  std::isfinite(proposal.max_successful_residuals.position_m) &&
                  std::isfinite(proposal.max_successful_residuals.velocity_mps) &&
                  std::isfinite(proposal.max_successful_residuals.momentum_nms) &&
                  proposal.last_solver_detail ==
                      "original_continuous_centroidal_certificate",
              "search diagnostics were not reported");
        const auto retained_target =
            proposal.selected_problem.request.candidate_sets[0]
                .candidates[1].target_world.value;
        problem.request.candidate_sets[0].candidates[1].target_world.value.x =
            42.0;
        problem.candidate_surfaces[0][1].max_normal_n = 1.0;
        Check(proposal.selected_problem.request.candidate_sets[0]
                  .candidates[1].target_world.value.x == retained_target.x &&
                  proposal.selected_problem.combination[0] == 1,
              "selected proposal aliases mutable input");
        auto tied = Fixture();
        tied.candidate_surfaces[0][0] = tied.candidate_surfaces[0][1];
        const auto gated=SearchCentroidalJointProposal(tied,{},
            [](const CentroidalProblem &p,const CentroidalResult &){
                return CandidateReferenceVerdict{true,p.combination[0]==1,
                    p.combination[0]==1 ? JointPlannerFailure::kNone : JointPlannerFailure::kCandidateConstraintViolation};});
        Check(gated.selected_valid && gated.selected_reference_checked &&
              gated.selected_problem.combination[0]==1 && gated.reference_rejections==1,
              "reference validator must gate reduced-cost winner before selection");
        const auto unknown=SearchCentroidalJointProposal(tied,{},
            [](const CentroidalProblem &,const CentroidalResult &){return CandidateReferenceVerdict{};});
        Check(!unknown.selected_valid && !unknown.selected_reference_checked && unknown.reference_checks>0,
              "unchecked reference must never expose a selected bundle");
        const auto tie = SearchCentroidalJointProposal(tied);
        Check(tie.selected_valid && tie.selected_problem.combination == std::vector<std::size_t>{0},
              "equal-cost result must retain lexicographically first winner");
        auto no_feasible = Fixture();
        no_feasible.candidate_surfaces[0][1].max_normal_n = 1.0;
        const auto rejected = SearchCentroidalJointProposal(no_feasible);
        Check(!rejected.search.feasible && !rejected.selected_valid,
              "all rejected combinations produced a proposal");
        Check(rejected.search.failure ==
                  JointPlannerFailure::kNoFeasibleCandidateInSet,
              "no-feasible failure was not preserved");
        Check(rejected.total_qp_iterations == 0 &&
                  rejected.total_scp_iterations == 0,
              "rejected search reported invented solver work");
        std::cout << "Stage C centroidal joint proposal checks passed"
                  << " qp=" << proposal.total_qp_iterations
                  << " scp=" << proposal.total_scp_iterations
                  << " max_residual=" << proposal.max_successful_residual
                  << "\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
