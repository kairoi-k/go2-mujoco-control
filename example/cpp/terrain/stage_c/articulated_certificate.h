#pragma once
#include "centroidal_subproblem.h"
#include "body_acceleration.h"
#include "id_wbc_certificate.h"
namespace go2_terrain { namespace stage_c {
struct ArticulatedSampleCertificate {
    go2_control::IdWbcPhysicalCertificate dynamics{};
    Eigen::Matrix<double,12,1> torque=Eigen::Matrix<double,12,1>::Zero();
    double max_surface_force_violation_n=std::numeric_limits<double>::infinity();
    double max_joint_position_violation_rad=std::numeric_limits<double>::infinity();
    double max_joint_velocity_violation_radps=std::numeric_limits<double>::infinity();
    JointPlannerFailure failure=JointPlannerFailure::kInvalidInput;
    // A finite sample-level declared-model check, never a swept-trajectory,
    // plant-contact, motor PD command or global infeasibility certificate.
    bool sample_feasible=false;
};
inline ArticulatedSampleCertificate VerifyArticulatedSample(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &state,
    const Eigen::Matrix<double,18,1> &qacc,const ContactForceInterval &force,
    const std::array<Eigen::Vector3d,4> &application_points_world,
    const std::array<ContactSurface,4> &surfaces,
    double torque_limit_nm,double joint_velocity_limit_radps) {
    ArticulatedSampleCertificate out;
    if(!qacc.allFinite() || !std::isfinite(torque_limit_nm) || torque_limit_nm<=0 ||
       !std::isfinite(joint_velocity_limit_radps) || joint_velocity_limit_radps<=0 ||
       force.start.value < 0 || force.end<=force.start) return out;
    go2_control::RigidBodyPlanningKinematics model;
    if(!robot.EvaluatePlanningKinematics(state,model)) {
        out.failure=JointPlannerFailure::kObservationUnavailable;return out;
    }
    go2_control::IdWbcInput input;input.dynamics=model.dynamics;input.contact=force.contact;
    // Eigen array defaults are not a physical zero contract; initialize every
    // optional field before the certificate inspects the active mask.
    input.contact_normal.fill(Eigen::Vector3d::Zero());
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    input.have_force_application_jacobian=true;
    if(!robot.EvaluateContactJacobians(state,application_points_world,input.force_application_jac_world)) {
        out.failure=JointPlannerFailure::kObservationUnavailable;return out;
    }
    go2_control::IdWbcParams parameters;parameters.min_normal_n=0;
    parameters.max_normal_n=0;parameters.friction_mu=0;parameters.tau_limit_nm=torque_limit_nm;
    Eigen::Matrix<double,12,1> stacked;
    out.max_surface_force_violation_n=0;
    for(int l=0;l<4;++l) {
        const auto &f=force.force_world[l];const Eigen::Vector3d value(f.x,f.y,f.z);
        if(!value.allFinite()) return out;
        stacked.segment<3>(3*l)=value;
        if(!force.contact[l]) {
            out.max_surface_force_violation_n=std::max(out.max_surface_force_violation_n,value.lpNorm<Eigen::Infinity>());
            continue;
        }
        const auto &surface=surfaces[l];
        if(surface.frame!=Frame::kWorld || surface.coverage!=MapCoverageState::kKnown ||
           surface.map_epoch == 0 || surface.valid_until<force.end ||
           !surface.basis_world.allFinite() ||
           (surface.basis_world.transpose()*surface.basis_world-Eigen::Matrix3d::Identity()).norm()>1e-8 ||
           std::abs(surface.basis_world.determinant()-1)>1e-8 ||
           !std::isfinite(surface.friction_mu) || surface.friction_mu<0 ||
           !std::isfinite(surface.min_normal_n) || surface.min_normal_n<0 ||
           !std::isfinite(surface.max_normal_n) || surface.max_normal_n<surface.min_normal_n) return out;
        input.contact_normal[l]=surface.basis_world.col(2);input.contact_normal_valid[l]=true;
        // Reuse WBC's full-model dynamics/torque validator. Its uniform cone
        // encloses all patches; the exact per-patch cones are checked below.
        parameters.friction_mu=std::max(parameters.friction_mu,surface.friction_mu);
        parameters.max_normal_n=std::max(parameters.max_normal_n,surface.max_normal_n);
        const Eigen::Vector3d local=surface.basis_world.transpose()*value;
        out.max_surface_force_violation_n=std::max({out.max_surface_force_violation_n,
            surface.min_normal_n-local.z(),local.z()-surface.max_normal_n,
            local.head<2>().norm()-surface.friction_mu*local.z()});
    }
    const auto &dyn=model.dynamics;
    Eigen::Matrix<double,18,1> generalized=dyn.mass_matrix*qacc+dyn.bias;
    for(int l=0;l<4;++l) generalized-=input.force_application_jac_world[l].transpose()*stacked.segment<3>(3*l);
    // IdWbcOutput::tau and the physical certificate use the generalized
    // joint-row order (qvel suffix 6..17), not the XML motor-name order.
    // Plant command code performs the explicit MotorDof mapping later.
    for(int j=0;j<12;++j)
        out.torque[j]=generalized[go2_control::kFloatingNv+j];
    out.dynamics=go2_control::VerifyIdWbcPhysicalCertificate(parameters,input,qacc,stacked,out.torque);
    out.max_joint_position_violation_rad=std::max({0.0,
        (model.joint_lower-state.q).maxCoeff(),(state.q-model.joint_upper).maxCoeff()});
    out.max_joint_velocity_violation_radps=std::max(0.0,state.dq.cwiseAbs().maxCoeff()-joint_velocity_limit_radps);
    out.sample_feasible=out.dynamics.feasible && out.max_surface_force_violation_n<=1e-6 &&
        out.max_joint_position_violation_rad<=1e-8 && out.max_joint_velocity_violation_radps<=1e-8;
    // A finite, internally valid tuple rejected by a declared bound is a
    // candidate constraint violation, not numerical failure or global
    // infeasibility. The new enum is added by the owning Stage-C types seam.
    if(out.sample_feasible)
        out.failure=JointPlannerFailure::kNone;
    else if(out.dynamics.checked && out.dynamics.input_valid && out.dynamics.valid)
        out.failure=JointPlannerFailure::kCandidateConstraintViolation;
    else
        out.failure=JointPlannerFailure::kInvalidInput;
    return out;
}
}} // namespace
