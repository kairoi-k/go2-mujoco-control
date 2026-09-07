#include "stage_c/centroidal_wbc_task.h"
#include "id_wbc_certificate.h"
#include <iostream>
#include <stdexcept>
using namespace go2_control;
void Check(bool x,const char *m){if(!x)throw std::runtime_error(m);}
int main(){try {
    IdWbcInput input;input.dynamics.valid=true;input.dynamics.mass_kg=10;
    input.dynamics.mass_matrix.setIdentity();input.dynamics.bias.setZero();
    input.dynamics.qvel.setZero();
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    for(int l=0;l<4;++l){input.dynamics.foot_jac_world[l].setZero();input.dynamics.foot_jac_dot_world[l].setZero();}
    IdWbcParams params;params.w_posture=params.w_tau=0;
    IdWbcOutput old;Check(SolveInverseDynamicsWbc(params,input,old),"legacy solve");
    Check(old.qdd.norm()<1e-10 && !old.centroidal_motion_task_used,"legacy unchanged");
    input.have_centroidal_motion_task=true;input.centroidal_motion_map.setZero();
    input.centroidal_motion_map(0,6)=2;
    input.centroidal_motion_bias.setZero();input.centroidal_motion_bias[0]=1;
    input.desired_centroidal_derivative.setZero();input.desired_centroidal_derivative[0]=5;
    input.centroidal_motion_weights.setOnes();
    IdWbcOutput out;Check(SolveInverseDynamicsWbc(params,input,out),"mapped solve");
    Check(out.centroidal_motion_task_used && std::abs(out.qdd[6]-2)<1e-5,"independent scalar optimum (5-1)/2");
    Check(out.cost_terms.centroidal_motion<1e-9 && out.cost_terms.base_linear==0,"objective decomposition");
    auto priority_input=input;auto priority_params=params;
    priority_input.dynamics.foot_jac_world[0](0,6)=1;
    priority_input.swing_acc_world[0].x()=-100;
    priority_params.use_primal_active_set=true;priority_params.prioritize_body_and_stance=true;
    Check(SolveInverseDynamicsWbc(priority_params,priority_input,out) && out.ok,"priority solve");
    Check(out.body_stance_priority_used && std::abs(out.qdd[6]-2)<1e-5,"body task preserved despite conflicting swing");
    Check(out.priority_preservation_residual<1e-8,"priority preservation residual");
    auto bad=input;bad.centroidal_motion_map(1,2)=NAN;
    Check(!SolveInverseDynamicsWbc(params,bad,out),"unknown task rejected");
    bad=input;bad.centroidal_motion_weights[0]=-1;
    Check(!SolveInverseDynamicsWbc(params,bad,out),"negative weights rejected");
    // Analytic free-body/joint coupling: qdd_body=-0.5*qdd_joint.
    // Centroidal cost 4*(x-2)^2 plus attitude cost (x-4)^2 has x=2.4.
    auto coupled=input;auto coupled_params=params;coupled_params.w_base_ang=4;
    coupled.dynamics.mass_matrix(3,6)=coupled.dynamics.mass_matrix(6,3)=.5;
    coupled.have_centroidal_orientation_task=true;
    coupled.desired_angular_acc_body<<-2,0,0;
    Check(SolveInverseDynamicsWbc(coupled_params,coupled,out),"coupled attitude solve");
    Check(out.centroidal_orientation_task_used && std::abs(out.qdd[6]-2.4)<1e-5 &&
          std::abs(out.qdd[3]+1.2)<1e-5,"independent coupled optimum");
    Check(std::abs(out.cost_terms.base_angular-2.56)<1e-5 &&
          std::abs(out.cost_terms.centroidal_motion-.64)<1e-5,"attitude cost accounting");
    auto invalid_attitude=coupled;invalid_attitude.desired_angular_acc_body.x()=NAN;
    Check(!SolveInverseDynamicsWbc(coupled_params,invalid_attitude,out),"unknown attitude rejected");
    invalid_attitude=coupled;invalid_attitude.have_centroidal_motion_task=false;
    Check(!SolveInverseDynamicsWbc(coupled_params,invalid_attitude,out),"orphan attitude flag rejected");
    coupled_params.w_base_ang=0;
    Check(!SolveInverseDynamicsWbc(coupled_params,coupled,out),"absent attitude weight rejected");
    Go2RigidBody robot;Check(robot.Load(GO2_MODEL_PATH),"actual model");
    RigidBodyState state;state.position_world.z()=.4;
    state.q<<0,.8,-1.6,0,.8,-1.6,0,.8,-1.6,0,.8,-1.6;
    state.dq.setLinSpaced(-.4,.7);state.linear_vel_world<<.7,-.1,.2;state.angular_vel_body<<.2,-.3,.1;
    Check(go2_terrain::stage_c::SetCentroidalWbcTask(robot,state,Eigen::Matrix<double,6,1>::Zero(),Eigen::Matrix<double,6,1>::Ones(),input),"actual task");
    const auto qvel=input.dynamics.qvel;
    Eigen::Matrix<double,18,1> qacc=Eigen::Matrix<double,18,1>::LinSpaced(-.7,.9);
    const auto predicted=(input.centroidal_motion_map*qacc+input.centroidal_motion_bias).eval();
    constexpr double dt=1e-5;
    RigidBodyState plus,minus;
    Check(robot.IntegrateConfiguration(state,qvel+.5*dt*qacc,dt,plus),"plus configuration");
    Check(robot.IntegrateConfiguration(state,qvel-.5*dt*qacc,-dt,minus),"minus configuration");
    auto setv=[&](RigidBodyState &s,const Eigen::Matrix<double,18,1> &v){s.linear_vel_world=v.head<3>();s.angular_vel_body=v.segment<3>(3);for(int j=0;j<12;++j)s.dq[j]=v[robot.MotorDof(j)];};
    setv(plus,qvel+dt*qacc);setv(minus,qvel-dt*qacc);
    RigidBodyPlanningKinematics mp,mm;Check(robot.EvaluatePlanningKinematics(plus,mp)&&robot.EvaluatePlanningKinematics(minus,mm),"FD observations");
    Eigen::Matrix<double,6,1> fd;fd<< (mp.com_velocity_world-mm.com_velocity_world)/(2*dt),(mp.angular_momentum_world-mm.angular_momentum_world)/(2*dt);
    const double error=(predicted-fd).lpNorm<Eigen::Infinity>();
    Check(error<1e-7,"actual momentum derivative oracle");
    std::cout<<"centroidal WBC task passed FD_error="<<error<<"\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
