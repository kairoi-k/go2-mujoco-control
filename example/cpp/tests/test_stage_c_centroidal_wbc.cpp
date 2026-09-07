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
    auto bad=input;bad.centroidal_motion_map(1,2)=NAN;
    Check(!SolveInverseDynamicsWbc(params,bad,out),"unknown task rejected");
    bad=input;bad.centroidal_motion_weights[0]=-1;
    Check(!SolveInverseDynamicsWbc(params,bad,out),"negative weights rejected");
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
