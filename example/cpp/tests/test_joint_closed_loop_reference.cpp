#include "stage_c/joint_closed_loop_replay.h"
#include "stage_c/model_observation.h"
#include "stage_c/joint_feedback_controller.h"
#include <iostream>
using namespace go2_terrain::stage_c;
using namespace go2_terrain::stage_c::joint_closed_loop_detail;
using namespace go2_control;
void Check(bool b,const char*s){if(!b)throw std::runtime_error(s);}
int main(){try {
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

 CentroidalProblem prefix;std::string prefix_failure;
 Check(CopyReplaySchedule(p,t0,TimeNs::FromSeconds(1.031),prefix,prefix_failure),"off-grid reference horizon");
 Check(prefix.grid==p.grid && prefix.schedule.back().end==TimeNs::FromSeconds(1.031),"original dynamics grid preserved");
 auto result=SolveCentroidalSubproblem(p);
 Check(result.certificate.feasible,"nominal fixture");
 CentroidalJointProposal proposal; proposal.selected_valid=true; proposal.search.feasible=true;
 proposal.selected_problem=p; proposal.selected_result=result;
 feet.problem=&proposal.selected_problem; feet.start=t0; feet.end=t2;
 const auto foot_sample=SampleFootTrajectoryAt(feet,t0);
 Check(foot_sample.valid && foot_sample.samples.size()==1,"feedback feet fixture");
 const auto tick=joint_feedback_controller::FeedbackTick(proposal,robot,initial,
     foot_sample.samples.front(),contact.mask,t0,0.0);
 Check(tick.ok && tick.tau_valid && tick.id_certificate_valid && tick.motor_envelope_valid,
       tick.failure.c_str());
 const auto repeated=joint_feedback_controller::FeedbackTick(proposal,robot,initial,
     foot_sample.samples.front(),contact.mask,t0,0.0);
 Check((tick.tau-repeated.tau).norm()==0,"feedback tick deterministic");
 const auto expired=joint_feedback_controller::FeedbackTick(proposal,robot,initial,
     foot_sample.samples.front(),contact.mask,t2,0.0);
 Check(!expired.ok && !expired.tau_valid && !expired.tau.allFinite(),
       "expired force interval exposed executable torque");
 auto invalid=proposal;invalid.selected_result.certificate.original_dynamics_checked=false;
 const auto uncertified=joint_feedback_controller::FeedbackTick(invalid,robot,initial,
     foot_sample.samples.front(),contact.mask,t0,0.0);
 Check(!uncertified.tau_valid && !uncertified.tau.allFinite(),"unchecked proposal exposed torque");

 ClosedLoopResearchConfig config; Eigen::Matrix<double,6,1> desired,weights;
 ContactForceInterval force; CentroidalState reference; std::string failure;
 const auto interior=TimeNs::FromSeconds(1.002);
 Check(BuildCentroidalReference(p,result,interior,m,config,desired,weights,force,reference,failure),"interior force interval");
 Check(force.start==t0 && force.end==t1,"interval identity preserved");
 auto altered=m;altered.com_velocity_world+=Eigen::Vector3d(.1,.2,.3);
 altered.angular_momentum_world+=Eigen::Vector3d(.2,.3,.4);
 Eigen::Matrix<double,6,1> changed,unused;
 Check(BuildCentroidalReference(p,result,interior,altered,config,changed,unused,force,reference,failure),"perturbed actual");
 Eigen::Matrix<double,6,1> expected;expected<<-.7,-1.4,-2.4,-.6,-.9,-1.2;
 Check((changed-desired-expected).norm()<1e-10,"actual COM velocity and angular momentum feedback oracle");
 Check(!BuildCentroidalReference(p,result,t2,m,config,changed,unused,force,reference,failure),"endpoint has no force");
 ModelOwner plant;char error[1024]={};plant.model=mj_loadXML(GO2_MODEL_PATH,nullptr,error,sizeof(error));
 Check(plant.model!=nullptr,"plant model");plant.data=mj_makeData(plant.model);
 ModelOwner scene;const auto scene_path=std::filesystem::path(GO2_MODEL_PATH).parent_path()/"phase2_flat.xml";
 scene.model=mj_loadXML(scene_path.c_str(),nullptr,error,sizeof(error));Check(scene.model!=nullptr,"scene");
 Check(ValidateRobotModels(*scene.model,*plant.model,failure),"same robot with unnamed free joint");
 scene.model->opt.gravity[2]+=.1;
 Check(!ValidateRobotModels(*scene.model,*plant.model,failure),"gravity mismatch rejected");
 scene.model->opt.gravity[2]-=.1;
 scene.model->actuator_gear[0]=2;
 Check(!ValidateRobotModels(*scene.model,*plant.model,failure),"gear mismatch rejected");
 initial.linear_vel_world<<.3,-.2,.1;initial.angular_vel_body<<.1,.2,-.3;initial.dq.setConstant(.12);
 initial.quat_world_from_body.coeffs()*=1.0+2e-8;
 Check(WriteStateToPlant(*plant.model,*plant.data,initial,failure),"float-rounded quaternion state load");
 const auto recovered=StateFromPlant(*plant.model,*plant.data);
 Check((initial.dq-recovered.dq).norm()<1e-12 && (initial.linear_vel_world-recovered.linear_vel_world).norm()<1e-12 && (initial.angular_vel_body-recovered.angular_vel_body).norm()<1e-12,"state velocity unchanged");
 ReplayRow row;row.plant_time_s=.002;ContactObservation observation;std::array<bool,4> mask{};
 FillReplayRowState(row,1,1.002,initial,m,observation,mask,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr);
 Check(row.plant_time_s==.002,"plant clock retained");
 mjContact raw_contact{};raw_contact.frame[0]=0;raw_contact.frame[1]=1;raw_contact.frame[2]=0;
 raw_contact.frame[3]=1;raw_contact.frame[4]=0;raw_contact.frame[5]=0;raw_contact.frame[8]=-1;
 mjtNum f[3]={10,2,3};Check((ContactForceWorld(raw_contact,f,-1)-Eigen::Vector3d(-2,-10,3)).norm()<1e-12,"contact frame/sign oracle");
 std::cout<<"closed-loop reference interval, feedback, state, frame, clock oracles passed\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
