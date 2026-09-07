#pragma once
#include "centroidal_wbc_task.h"
#include "articulated_certificate.h"
namespace go2_terrain { namespace stage_c {
struct ArticulatedFeedbackReference {
    TimeNs start{},end{};
    Eigen::Matrix<double,6,1> centroidal_derivative=
        Eigen::Matrix<double,6,1>::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double,6,1> centroidal_weights=
        Eigen::Matrix<double,6,1>::Constant(std::numeric_limits<double>::quiet_NaN());
    std::array<Eigen::Vector3d,4> foot_acceleration;
    std::array<bool,4> foot_acceleration_valid{};
    ContactForceInterval nominal_force{};
    std::array<TimedPoint,4> surface_plane_points{};
    std::array<ContactSurface,4> surfaces{};
};
struct ArticulatedFeedbackStep {
    JointPlannerFailure failure=JointPlannerFailure::kInvalidInput;
    go2_control::IdWbcOutput solution{};
    ArticulatedSampleCertificate certificate{};
    go2_control::RigidBodyState next_state{};
    ContactForceInterval force{};
    std::array<Eigen::Vector3d,4> force_application_points;
    std::array<double,4> signed_normal_gap_m{};
    std::array<Eigen::Vector3d,4> surface_material_velocity_world;
    bool model_sample_verified=false;
    // A force-bearing gap/sliding velocity must be judged by an explicit
    // contact/compliance model. Neither soft WBC tasks nor this one sample
    // establish that model, swept geometry or final actuator execution.
    bool contact_evolution_verified=false;
    bool execution_ready=false;
};
inline ArticulatedFeedbackStep SolveArticulatedFeedbackStep(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &state,
    const ArticulatedFeedbackReference &reference,
    go2_control::IdWbcParams params,double joint_velocity_limit_radps) {
    ArticulatedFeedbackStep out;
    const double dt=(reference.end.value-reference.start.value)*1e-9;
    if(reference.start.value<0 || reference.end<=reference.start || dt>.002000001 ||
       reference.nominal_force.start!=reference.start || reference.nominal_force.end!=reference.end ||
       !std::isfinite(joint_velocity_limit_radps) || joint_velocity_limit_radps<=0) return out;
    go2_control::IdWbcInput input;
    if(!SetCentroidalWbcTask(robot,state,reference.centroidal_derivative,reference.centroidal_weights,input)) return out;
    input.contact=reference.nominal_force.contact;
    input.contact_normal.fill(Eigen::Vector3d::Zero());
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    input.have_stance_acc=true;input.have_force_ref=true;
    params.friction_mu=std::numeric_limits<double>::infinity();
    params.max_normal_n=std::numeric_limits<double>::infinity();
    params.min_normal_n=0;
    // Uniform WBC caps form a conservative subset of candidate-specific caps.
    // Independent validation below always uses each original patch's limits.
    bool support=false;
    for(int l=0;l<4;++l) {
        if(!reference.foot_acceleration_valid[l] || !reference.foot_acceleration[l].allFinite() ||
           !input.dynamics.foot_geometry_valid[l]) return out;
        input.stance_acc_world[l]=input.swing_acc_world[l]=reference.foot_acceleration[l];
        out.surface_material_velocity_world[l].setZero();
        out.force_application_points[l]=input.dynamics.foot_pos_world[l];
        const auto &f=reference.nominal_force.force_world[l];
        input.force_ref.segment<3>(3*l)<<f.x,f.y,f.z;
        if(!input.force_ref.segment<3>(3*l).allFinite()) return out;
        if(!input.contact[l]) {
            if(input.force_ref.segment<3>(3*l).norm()>0) return out;
            continue;
        }
        support=true;
        const auto &s=reference.surfaces[l];const auto &p=reference.surface_plane_points[l];
        if(!TimedPointValidForRole(p,PointRole::kSurfaceContactPoint,Frame::kWorld) ||
           p.source_time>reference.start || s.frame!=Frame::kWorld ||
           s.coverage!=MapCoverageState::kKnown || !s.map_epoch || s.valid_until<reference.end ||
           !s.basis_world.allFinite() ||
           (s.basis_world.transpose()*s.basis_world-Eigen::Matrix3d::Identity()).norm()>1e-8 ||
           std::abs(s.basis_world.determinant()-1)>1e-8 ||
           !std::isfinite(s.friction_mu) || s.friction_mu<0 ||
           !std::isfinite(s.min_normal_n) || s.min_normal_n<0 ||
           !std::isfinite(s.max_normal_n) || s.max_normal_n<s.min_normal_n) return out;
        params.friction_mu=std::min(params.friction_mu,s.friction_mu);
        params.max_normal_n=std::min(params.max_normal_n,s.max_normal_n);
        params.min_normal_n_by_leg[l]=s.min_normal_n;
        input.contact_normal[l]=s.basis_world.col(2);input.contact_normal_valid[l]=true;
        const auto normal=s.basis_world.col(2);
        const double radius=input.dynamics.foot_geometry[l].collision_radius_m;
        out.force_application_points[l]-=radius*normal;
        out.signed_normal_gap_m[l]=normal.dot(out.force_application_points[l]-Eigen::Vector3d(p.value.x,p.value.y,p.value.z));
    }
    if(!support){params.friction_mu=0;params.max_normal_n=0;}
    input.have_force_application_jacobian=true;
    if(!robot.EvaluateContactJacobians(state,out.force_application_points,input.force_application_jac_world)) return out;
    for(int l=0;l<4;++l)if(input.contact[l])
        out.surface_material_velocity_world[l]=input.force_application_jac_world[l]*input.dynamics.qvel;
    if(!go2_control::SolveInverseDynamicsWbc(params,input,out.solution)) {
        out.failure=JointPlannerFailure::kNumericalFailure;return out;
    }
    out.force=reference.nominal_force;
    for(int l=0;l<4;++l){const auto f=out.solution.force.segment<3>(3*l);out.force.force_world[l]={f.x(),f.y(),f.z()};}
    out.certificate=VerifyArticulatedSample(robot,state,out.solution.qdd,out.force,
        out.force_application_points,reference.surfaces,params.tau_limit_nm,joint_velocity_limit_radps);
    if(!out.certificate.sample_feasible){out.failure=out.certificate.failure;return out;}
    if((out.certificate.torque-out.solution.tau).lpNorm<Eigen::Infinity>()>1e-8) {
        out.failure=JointPlannerFailure::kNumericalFailure;return out;
    }
    const auto v0=input.dynamics.qvel;
    // Constant-acceleration manifold step: no kinematic projection and no
    // implicit impact impulse. Future contact feasibility remains unverified.
    if(!robot.IntegrateConfiguration(state,v0+.5*dt*out.solution.qdd,dt,out.next_state)) {
        out.failure=JointPlannerFailure::kNumericalFailure;return out;
    }
    const Eigen::Matrix<double,18,1> v1=v0+dt*out.solution.qdd;
    out.next_state.linear_vel_world=v1.head<3>();out.next_state.angular_vel_body=v1.segment<3>(3);
    for(int j=0;j<12;++j)out.next_state.dq[j]=v1[robot.MotorDof(j)];
    out.model_sample_verified=true;out.failure=JointPlannerFailure::kNone;return out;
}
}} // namespace
