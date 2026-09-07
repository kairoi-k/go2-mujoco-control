#include "stage_c/feedback_step.h"
#include <iostream>
#include <stdexcept>
using namespace go2_terrain::stage_c;
using namespace go2_control;
void Check(bool b,const char*s){if(!b)throw std::runtime_error(s);}
int main(){try {
 Go2RigidBody robot;Check(robot.Load(GO2_MODEL_PATH),"model");RigidBodyState state;
 state.position_world.z()=.4;state.q<<0,.8,-1.6,0,.8,-1.6,0,.8,-1.6,0,.8,-1.6;
 state.linear_vel_world<<.7,0,-.2;state.dq.setConstant(.1);
 RigidBodyPlanningKinematics model;Check(robot.EvaluatePlanningKinematics(state,model),"evaluate");
 ArticulatedFeedbackReference r;r.start=TimeNs::FromSeconds(1);r.end=TimeNs::FromSeconds(1.002);
 r.nominal_force.start=r.start;r.nominal_force.end=r.end;
 r.centroidal_weights.setOnes();r.centroidal_derivative.setZero();r.centroidal_derivative[2]=-9.81;
 r.foot_acceleration.fill(Eigen::Vector3d(0,0,-9.81));r.foot_acceleration_valid.fill(true);
 IdWbcParams p;p.w_force_track=1e-2;
 const auto aerial=SolveArticulatedFeedbackStep(robot,state,r,p,30);
 if(!aerial.model_sample_verified)std::cerr<<JointPlannerFailureName(aerial.failure)<<"\n";
 Check(aerial.model_sample_verified,"aerial moving-state sample");
 Check(!aerial.contact_evolution_verified && !aerial.execution_ready,"scope separate");
 Check(aerial.solution.force.norm()==0,"aerial zero force");
 Check((aerial.next_state.linear_vel_world-state.linear_vel_world-.002*aerial.solution.qdd.head<3>()).norm()<1e-12,"velocity integration preserves input");
 Check((state.dq-Eigen::Matrix<double,12,1>::Constant(.1)).norm()==0,"input untouched");
 for(int j=0;j<12;++j)Check(std::abs(aerial.next_state.dq[j]-state.dq[j]-.002*aerial.solution.qdd[robot.MotorDof(j)])<1e-12,"motor mapping");
 r.nominal_force.contact.fill(true);r.centroidal_derivative.setZero();r.foot_acceleration.fill(Eigen::Vector3d::Zero());
 for(int l=0;l<4;++l){auto point=model.dynamics.foot_pos_world[l];point.z()-=model.dynamics.foot_geometry[l].collision_radius_m;
 r.surface_plane_points[l]={{point.x(),point.y(),point.z()},Frame::kWorld,r.start,true,PointRole::kSurfaceContactPoint};
 auto&s=r.surfaces[l];s.frame=Frame::kWorld;s.coverage=MapCoverageState::kKnown;s.map_epoch=1;s.valid_until=r.end;s.friction_mu=.8;s.max_normal_n=180;
 r.nominal_force.force_world[l]={0,0,model.dynamics.mass_kg*9.81/4};}
 const auto support=SolveArticulatedFeedbackStep(robot,state,r,p,30);
 if(!support.model_sample_verified)std::cerr<<JointPlannerFailureName(support.failure)<<" mask="<<support.certificate.dynamics.failure_bitmask<<"\n";
 Check(support.model_sample_verified,"moving support sample is not velocity projected");
 Check(support.surface_material_velocity_world[0].norm()>.1,"nonzero surface velocity retained");
 Check(!support.contact_evolution_verified,"soft motion task cannot certify slip");
 auto unknown=r;unknown.surfaces[0].coverage=MapCoverageState::kUnknownInside;
 Check(!SolveArticulatedFeedbackStep(robot,state,unknown,p,30).model_sample_verified,"unknown patch rejected");
 auto gap=r;gap.surface_plane_points[0].value.z-=.1;
 const auto separated=SolveArticulatedFeedbackStep(robot,state,gap,p,30);
 Check(std::abs(separated.signed_normal_gap_m[0]-.1)<1e-12 && !separated.contact_evolution_verified,"force at separated sphere not contact certificate");
 std::cout<<"feedback sample support force residual="<<support.certificate.dynamics.max_dynamics_force_residual_N<<" material speed="<<support.surface_material_velocity_world[0].norm()<<"\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
