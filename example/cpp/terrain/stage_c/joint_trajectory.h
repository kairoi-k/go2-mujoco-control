#pragma once
#include "foot_trajectory.h"
#include "articulated_certificate.h"
#include "anytime_joint_search.h"
namespace go2_terrain { namespace stage_c {
struct JointTrajectorySample {
    TimeNs time{};
    go2_control::RigidBodyState state{};
    Eigen::Matrix<double,18,1> qacc=Eigen::Matrix<double,18,1>::Zero();
    // Generalized joint DOF order 6..17, matching ID-WBC. Motor commands
    // require the actual model MotorDof mapping; this is not motor-name order.
    Eigen::Matrix<double,12,1> torque=Eigen::Matrix<double,12,1>::Zero();
    bool force_interval_valid=false;
    ContactForceInterval force{};
    ArticulatedSampleCertificate model_certificate{};
};
struct JointTrajectoryResult {
    JointPlannerFailure failure=JointPlannerFailure::kInvalidInput;
    CentroidalResult centroidal{};
    BodyTrajectoryReconstruction body{};
    std::vector<JointTrajectorySample> samples;
    bool model_samples_verified=false;
    // Geometry between samples and final actuator command remain separate
    // gates. This result alone cannot be published as an execution command.
    bool execution_ready=false;
};
inline JointTrajectoryResult SolveJointTrajectoryCandidate(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &initial,
    const CentroidalProblem &problem,FootTrajectoryRequest feet,
    double torque_limit_nm,double joint_velocity_limit_radps,
    TimeNs maximum_sample_interval=TimeNs{5000000}) {
    JointTrajectoryResult out;
    if(maximum_sample_interval.value<=0 || maximum_sample_interval.value>10000000) return out;
    out.centroidal=SolveCentroidalSubproblem(problem);
    if(!out.centroidal.certificate.feasible) {out.failure=out.centroidal.failure;return out;}
    const auto independent=VerifyCentroidalTrajectory(problem,out.centroidal);
    if(!independent.feasible) {out.failure=JointPlannerFailure::kNumericalFailure;return out;}
    feet.problem=&problem;feet.start=problem.grid.front();feet.end=problem.grid.back();
    std::vector<TimeNs> times{problem.grid.front()};
    for(std::size_t k=1;k<problem.grid.size();++k) {
        const auto span=problem.grid[k].value-problem.grid[k-1].value;
        const auto count=1+(span-1)/maximum_sample_interval.value;
        if(count>10000 || times.size()+count>10000) {out.failure=JointPlannerFailure::kBudgetExhausted;return out;}
        for(std::int64_t j=1;j<=count;++j)times.push_back({problem.grid[k-1].value+span*j/count});
    }
    const auto v=[](const go2::Vec3 &x){return Eigen::Vector3d(x.x,x.y,x.z);};
    const auto reference=[&](TimeNs time) {
        BodyReference ref;
        const auto c=SampleCentroidalTrajectory(problem,out.centroidal,time);
        const auto f=time==feet.end?SampleFootTrajectoryEndState(feet):SampleFootTrajectoryAt(feet,time);
        if(!c.state_valid || !f.valid || f.samples.size()!=1) return ref;
        ref.valid=true;ref.foot_reference_valid=f.samples[0].leg_valid;
        ref.com_world=c.state.head<3>();ref.com_velocity_world=c.state.segment<3>(3);
        ref.angular_momentum_world=c.state.tail<3>();
        for(int l=0;l<4;++l) {
            ref.foot_center_world[l]=v(f.samples[0].center_world[l].value);
            ref.foot_velocity_world[l]=v(f.samples[0].velocity_world[l]);
        }
        return ref;
    };
    std::vector<TimeNs> force_times(times.begin(),times.end()-1);
    auto foot_check=SampleFootTrajectory(feet,force_times);
    if(!foot_check.valid) {out.failure=foot_check.failure;return out;}
    const auto foot_end=SampleFootTrajectoryEndState(feet);
    if(!foot_end.valid || foot_end.samples.size()!=1) {out.failure=JointPlannerFailure::kCoverageIncomplete;return out;}
    foot_check.samples.push_back(foot_end.samples.front());
    out.body=ReconstructBodyTrajectory(robot,initial,times,reference);
    if(!out.body.kinematics_valid) {out.failure=out.body.failure;return out;}
    for(std::size_t k=0;k<times.size();++k) {
        JointTrajectorySample sample;sample.time=times[k];sample.state=out.body.knots[k].state;
        const auto c=SampleCentroidalTrajectory(problem,out.centroidal,times[k]);
        if(!c.state_valid) {out.failure=JointPlannerFailure::kCoverageIncomplete;return out;}
        if(!c.force_valid) {
            if(k+1!=times.size()) {out.failure=JointPlannerFailure::kCoverageIncomplete;return out;}
            out.samples.push_back(sample);continue;
        }
        auto lifted=ReconstructBodyReference(robot,sample.state,reference(times[k]));
        if(!lifted.kinematics_valid) {out.failure=lifted.failure;return out;}
        BodyAccelerationTarget target;target.com_acceleration_valid=true;
        target.angular_momentum_derivative_valid=true;target.foot_acceleration_valid=foot_check.samples[k].leg_valid;
        target.com_acceleration_world=Eigen::Vector3d(0,0,-problem.model.gravity_mps2);
        std::array<Eigen::Vector3d,4> points;
        std::array<ContactSurface,4> surfaces{};
        const auto interval=std::find_if(problem.schedule.begin(),problem.schedule.end(),[&](const FixedScheduleInterval &s){return s.start<=times[k] && times[k]<s.end;});
        if(interval==problem.schedule.end()) {out.failure=JointPlannerFailure::kCoverageIncomplete;return out;}
        for(int l=0;l<4;++l) {
            points[l]=lifted.model.dynamics.foot_pos_world[l];
            target.foot_acceleration_world[l]=v(foot_check.samples[k].acceleration_world[l]);
            if(c.force.contact[l]) {
                const int e=interval->event_index[l];
                if(e<0) {points[l]=v(problem.request.input.feet[l].measured_support_anchor_world.value);surfaces[l]=problem.initial_surfaces[l];}
                else {points[l]=v(problem.request.candidate_sets[e].candidates[problem.combination[e]].target_world.value);
                    surfaces[l]=problem.candidate_surfaces.empty()?problem.event_surfaces[e]:problem.candidate_surfaces[e][problem.combination[e]];}
            }
            const auto force=v(c.force.force_world[l]);
            target.com_acceleration_world+=force/problem.model.mass_kg;
            target.angular_momentum_derivative_world+=(points[l]-c.state.head<3>()).cross(force);
        }
        const auto acceleration=LiftBodyAcceleration(robot,lifted,target);
        if(!acceleration.valid) {out.failure=acceleration.failure;return out;}
        sample.qacc=acceleration.qacc;sample.force=c.force;sample.force_interval_valid=true;
        sample.model_certificate=VerifyArticulatedSample(robot,sample.state,sample.qacc,
            sample.force,points,surfaces,torque_limit_nm,joint_velocity_limit_radps);
        sample.torque=sample.model_certificate.torque;
        out.samples.push_back(sample);
        if(!sample.model_certificate.sample_feasible) {out.failure=sample.model_certificate.failure;return out;}
    }
    out.model_samples_verified=true;out.failure=JointPlannerFailure::kNone;return out;
}
struct JointTrajectorySearchResult {
    JointPlanResult search{};
    JointTrajectoryResult selected{};
};
inline JointTrajectorySearchResult SearchJointTrajectories(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &initial,
    const CentroidalProblem &problem,const FootTrajectoryRequest &feet,
    double torque_limit_nm,double joint_velocity_limit_radps) {
    JointTrajectorySearchResult out;
    double best_cost=std::numeric_limits<double>::infinity();
    std::vector<std::size_t> best_indices;
    DeterministicBestFirstPlanner planner;
    out.search=planner.Plan(problem.request,[&](const std::vector<std::size_t> &indices) {
        auto selected_problem=problem;selected_problem.combination=indices;
        auto candidate=SolveJointTrajectoryCandidate(robot,initial,selected_problem,feet,
            torque_limit_nm,joint_velocity_limit_radps);
        JointEvaluation evaluation;evaluation.failure=candidate.failure;
        evaluation.feasible=candidate.model_samples_verified;
        if(!evaluation.feasible) return evaluation;
        evaluation.cost=candidate.centroidal.cost;
        for(std::size_t e=0;e<indices.size();++e)
            evaluation.cost+=problem.request.candidate_sets[e].candidates[indices[e]].foothold_cost;
        // Preserve the explicitly reduced/sampled scope in the legacy search
        // transport. An outer feasible proposal is not execution readiness.
        evaluation.plan.cost=evaluation.cost;
        if(evaluation.cost<best_cost ||
           (evaluation.cost==best_cost && indices<best_indices)) {
            best_cost=evaluation.cost;best_indices=indices;out.selected=std::move(candidate);
        }
        return evaluation;
    });
    if(out.search.feasible && out.search.plan.candidate_indices!=best_indices) {
        out.search.feasible=false;out.search.failure=JointPlannerFailure::kNumericalFailure;
    }
    return out;
}
}} // namespace
