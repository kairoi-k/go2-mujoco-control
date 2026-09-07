#include "stage_c/joint_trajectory.h"
#include "stage_c/model_observation.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <stdexcept>
using namespace go2_terrain::stage_c;
using namespace go2_control;
void Check(bool v,const char *s){if(!v)throw std::runtime_error(s);}
int main(int argc,char **){try {
    Go2RigidBody robot;Check(robot.Load(GO2_MODEL_PATH),"load");
    RigidBodyState initial;initial.position_world.z()=.4;initial.q<<0,.8,-1.6,0,.8,-1.6,0,.8,-1.6,0,.8,-1.6;initial.dq.setZero();
    RigidBodyPlanningKinematics m;Check(robot.EvaluatePlanningKinematics(initial,m),"model");
    const TimeNs t0=TimeNs::FromSeconds(1),t1=TimeNs::FromSeconds(1.02),t2=TimeNs::FromSeconds(1.04);
    PlanningIdentity identity{1000,t0,7,1,1};
    ContactEvidence contact;contact.valid=true;contact.provenance=ContactProvenance::kMeasured;contact.source_time=t0;contact.mask.fill(true);
    std::array<TimedPoint,4> anchors;
    FootTrajectoryRequest feet;feet.collision_radius_valid.fill(true);feet.initial_velocity_valid.fill(true);
    for(int l=0;l<4;++l) {
        auto p=m.dynamics.foot_pos_world[l];feet.collision_radius_m[l]=m.dynamics.foot_geometry[l].collision_radius_m;
        p.z()-=feet.collision_radius_m[l];anchors[l]={{p.x(),p.y(),p.z()},Frame::kWorld,t0,true,PointRole::kSurfaceContactPoint};
    }
    MapObservation map;map.metadata_valid=true;map.epoch=7;map.coverage=MapCoverageState::kKnown;map.total_cells=map.known_cells=64;map.width=map.height=8;
    Phase1CommandAuthority command;command.valid=true;command.command_epoch=1;command.period_s=.24;command.duty_factor=.4;
    PlanningBudget budget;budget.prediction_start=t0;budget.prediction_end=t2;
    auto observed=CaptureModelPlanningObservation(robot,initial,identity,contact,anchors,map,command,budget,m);
    Check(observed.ok,"model observation");
    CentroidalProblem p;p.request.input=observed.input;p.grid={t0,t1,t2};p.bounds.resize(3);
    p.schedule_epoch=1;p.schedule.push_back({t0,t2,contact.mask,{{-1,-1,-1,-1}}});
    p.model.mass_kg=m.dynamics.mass_kg;p.initial_momentum_world=m.angular_momentum_world;p.initial_momentum_valid=true;
    p.required_start=t0;p.required_end=t2;
    for(auto &surface:p.initial_surfaces){surface.frame=Frame::kWorld;surface.coverage=MapCoverageState::kKnown;surface.map_epoch=7;surface.valid_until=t2;surface.friction_mu=.8;surface.max_normal_n=180;}
    auto result=SolveJointTrajectoryCandidate(robot,initial,p,feet,35,30);
    if(!result.model_samples_verified)std::cerr<<"failure "<<JointPlannerFailureName(result.failure)<<" samples="<<result.samples.size()<<" centroid="<<result.centroidal.detail<<"\n";
    if(!result.samples.empty() && !result.model_samples_verified){const auto &c=result.samples.back().model_certificate;std::cerr<<"certificate bits="<<c.dynamics.failure_bitmask<<" residual="<<c.dynamics.dynamics_residual.transpose()<<" surface="<<c.max_surface_force_violation_n<<" joint="<<c.max_joint_position_violation_rad<<" vel="<<c.max_joint_velocity_violation_radps<<"\n";}
    Check(result.model_samples_verified,"full candidate model samples");
    Check(!result.execution_ready,"sampled model cannot claim execution");
    Check(result.samples.front().time==t0 && result.samples.back().time==t2,"absolute complete horizon");
    Check(!result.samples.back().force_interval_valid,"terminal state has no invented force");
    double force_residual=0,moment_residual=0,max_tau=0;
    for(const auto &sample:result.samples)if(sample.force_interval_valid){
        Check(sample.model_certificate.dynamics.force_application_jacobian_used,"actual patch force model");
        force_residual=std::max(force_residual,sample.model_certificate.dynamics.max_dynamics_force_residual_N);
        moment_residual=std::max(moment_residual,sample.model_certificate.dynamics.max_dynamics_moment_residual_Nm);
        max_tau=std::max(max_tau,sample.torque.cwiseAbs().maxCoeff());
    }
    const auto searched=SearchJointTrajectories(robot,initial,p,feet,35,30);
    Check(searched.search.feasible && searched.selected.model_samples_verified && searched.search.diagnostics.search_complete,"joint search evaluator composition");
    auto conflict=initial;conflict.dq[0]=.2;
    Check(!SolveJointTrajectoryCandidate(robot,conflict,p,feet,35,30).model_samples_verified,"initial velocity conflict");
    if(argc>1) {
        std::vector<double> latency;
        for(int trial=-3;trial<31;++trial) {
            const auto start=std::chrono::steady_clock::now();
            const auto repeated=SolveJointTrajectoryCandidate(robot,initial,p,feet,35,30);
            const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            Check(repeated.model_samples_verified,"benchmark certificate");
            Check((repeated.samples.back().state.q-result.samples.back().state.q).norm()<1e-12,"deterministic trajectory repeated");
            if(trial>=0)latency.push_back(elapsed);
        }
        std::sort(latency.begin(),latency.end());
        std::cout<<"joint_latency_us samples="<<latency.size()<<" p50="<<latency[15]<<" p95="<<latency[29]<<" max="<<latency.back()<<"\n";
    }
    std::cout<<"joint model samples="<<result.samples.size()<<" force_N="<<force_residual<<" moment_Nm="<<moment_residual<<" tau_Nm="<<max_tau<<"\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}}
