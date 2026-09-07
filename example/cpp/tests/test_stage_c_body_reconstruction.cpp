#include "stage_c/body_reconstruction.h"
#include <iostream>
#include <stdexcept>
using namespace go2_terrain::stage_c;
using namespace go2_control;
namespace {
void Check(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
RigidBodyState Pose() {
    RigidBodyState s;s.position_world=Eigen::Vector3d(.4,-.1,.43);
    s.quat_world_from_body=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(.12,Eigen::Vector3d::UnitY());
    s.q<<.05,.7,-1.4,-.05,.65,-1.35,.03,.68,-1.37,-.04,.72,-1.42;
    s.dq<<.1,-.2,.3,-.1,.1,.2,.2,-.3,.1,-.2,.15,-.1;
    s.linear_vel_world=Eigen::Vector3d(.7,.1,-.02);
    s.angular_vel_body=Eigen::Vector3d(.1,-.2,.3);return s;
}
BodyReference Reference(const RigidBodyPlanningKinematics &m) {
    BodyReference r;r.valid=true;r.foot_reference_valid.fill(true);r.com_world=m.dynamics.com_world;
    r.com_velocity_world=m.com_velocity_world;r.angular_momentum_world=m.angular_momentum_world;
    for(int l=0;l<4;++l) {r.foot_center_world[l]=m.dynamics.foot_pos_world[l];
        r.foot_velocity_world[l]=m.dynamics.foot_jac_world[l]*m.dynamics.qvel;}
    return r;
}
}
int main() {
 try {
    Go2RigidBody robot;Check(robot.Load(GO2_MODEL_PATH),"model load");
    auto known=Pose();RigidBodyPlanningKinematics m;
    Check(robot.EvaluatePlanningKinematics(known,m),"planning kinematics");
    Check((m.angular_momentum_matrix_world*m.dynamics.qvel-m.angular_momentum_world).norm()<1e-9,"momentum matrix vs subtree oracle");
    Check((m.com_jacobian_world*m.dynamics.qvel-m.com_velocity_world).norm()<1e-9,"COM Jacobian vs subtree oracle");
    const double dt=1e-6;RigidBodyState plus,minus;
    Check(robot.IntegrateConfiguration(known,m.dynamics.qvel,dt,plus),"positive integrate");
    Check(robot.IntegrateConfiguration(known,m.dynamics.qvel,-dt,minus),"negative integrate");
    RigidBodyDynamics a,b;Check(robot.Evaluate(plus,a)&&robot.Evaluate(minus,b),"finite difference models");
    double fd=( (a.com_world-b.com_world)/(2*dt)-m.com_velocity_world).norm();
    for(int l=0;l<4;++l)fd=std::max(fd,((a.foot_pos_world[l]-b.foot_pos_world[l])/(2*dt)-m.dynamics.foot_jac_world[l]*m.dynamics.qvel).norm());
    Check(fd<1e-7,"independent configuration finite difference");
    const auto reference=Reference(m);
    auto seed=known;seed.position_world+=Eigen::Vector3d(.01,-.005,.008);seed.q.array()+=.012;
    auto lifted=ReconstructBodyReference(robot,seed,reference);
    Check(lifted.kinematics_valid,"known reachable reference reconstructs");
    Check((lifted.state.q-known.q).norm()<1e-6,"recover known joint pose");
    Check((lifted.model.dynamics.qvel-m.dynamics.qvel).norm()<1e-6,"recover full articulated velocity");
    Check((lifted.state.position_world-known.position_world).norm()<1e-7,"recover body from actual COM and feet");
    // A known constant-translation articulated pose is an independent exact
    // trajectory oracle; no fixed COM/base offset is supplied to the lift.
    auto translating=known;translating.angular_vel_body.setZero();translating.dq.setZero();
    RigidBodyPlanningKinematics translation_model;
    Check(robot.EvaluatePlanningKinematics(translating,translation_model),"translation model");
    const auto translation_reference=Reference(translation_model);
    const TimeNs origin=TimeNs::FromSeconds(2.0);
    std::vector<TimeNs> times;
    for(int k=0;k<=10;++k)times.push_back(TimeNs::FromSeconds(2+.01*k));
    const auto trajectory=ReconstructBodyTrajectory(robot,translating,times,[&](TimeNs time){
        auto r=translation_reference;const double elapsed=(time.value-origin.value)*1e-9;
        const auto shift=translating.linear_vel_world*elapsed;
        r.com_world+=shift;for(auto &foot:r.foot_center_world)foot+=shift;return r;
    });
    Check(trajectory.kinematics_valid && trajectory.knots.size()==times.size(),"body trajectory full horizon");
    for(const auto &k:trajectory.knots) {
        const double elapsed=(k.time.value-origin.value)*1e-9;
        Check((k.state.position_world-translating.position_world-translating.linear_vel_world*elapsed).norm()<1e-7,"translation position oracle");
        Check((k.state.q-translating.q).norm()<1e-6,"translation posture oracle");
        Check(std::abs(k.state.quat_world_from_body.norm()-1)<1e-12,"quaternion manifold");
    }
    // A rotating, articulating constant-generalized-velocity oracle exercises
    // the orientation integration rather than only translation.
    RigidBodyPlanningKinematics moving_model;
    Check(robot.EvaluatePlanningKinematics(known,moving_model),"rotating oracle model");
    const auto fixed_velocity=moving_model.dynamics.qvel;
    auto rotating=ReconstructBodyTrajectory(robot,known,times,[&](TimeNs time){
        RigidBodyState actual;RigidBodyPlanningKinematics actual_model;
        Check(robot.IntegrateConfiguration(known,fixed_velocity,(time.value-origin.value)*1e-9,actual),"rotating oracle integration");
        Check(robot.EvaluatePlanningKinematics(actual,actual_model),"rotating oracle FK");
        return Reference(actual_model);
    });
    Check(rotating.kinematics_valid,"rotating articulated full horizon");
    for(const auto &k:rotating.knots) {
        RigidBodyState expected;
        Check(robot.IntegrateConfiguration(known,fixed_velocity,(k.time.value-origin.value)*1e-9,expected),"rotating expected");
        Check((k.state.q-expected.q).norm()<1e-6,"rotating joint trajectory oracle");
        Check(k.state.quat_world_from_body.angularDistance(expected.quat_world_from_body)<1e-7,"body orientation manifold oracle");
    }
    auto conflicting_velocity=translating;conflicting_velocity.dq[0]=.1;
    Check(ReconstructBodyTrajectory(robot,conflicting_velocity,times,[&](TimeNs){return translation_reference;}).failure==JointPlannerFailure::kInitialConditionConflict,"initial joint velocity cannot be reset by trajectory lift");
    auto impossible=reference;impossible.foot_center_world[0].z()+=2;
    Check(!ReconstructBodyReference(robot,known,impossible).kinematics_valid,"unreconstructed geometry not certified");
    auto missing=reference;missing.com_world.x()=std::numeric_limits<double>::quiet_NaN();
    Check(ReconstructBodyReference(robot,known,missing).failure==JointPlannerFailure::kInvalidInput,"unknown reference rejected");
    auto absent=reference;absent.foot_reference_valid[1]=false;
    Check(!ReconstructBodyReference(robot,known,absent).kinematics_valid,"missing swing trajectory rejected");
    auto invalid=known;invalid.quat_world_from_body.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
    Check(!robot.EvaluatePlanningKinematics(invalid,m),"nonfinite quaternion rejected");
    std::cout<<"body reconstruction passed: fd_mps="<<fd<<" position_m="<<lifted.position_residual_m
             <<" velocity_mps="<<lifted.velocity_residual_mps<<" momentum_nms="<<lifted.momentum_residual_nms
             <<" iterations="<<lifted.projection_iterations<<"\n";return 0;
 }catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}
}
