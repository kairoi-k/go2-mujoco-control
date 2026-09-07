#pragma once
#include "go2_rigid_body.h"
#include "types.h"
#include <functional>
namespace go2_terrain { namespace stage_c {
struct BodyReference {
    bool valid=false;
    std::array<bool,4> foot_reference_valid{};
    Eigen::Vector3d com_world=Eigen::Vector3d::Zero();
    Eigen::Vector3d com_velocity_world=Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_momentum_world=Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d,4> foot_center_world{
        Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
    std::array<Eigen::Vector3d,4> foot_velocity_world{
        Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
};
struct BodyReconstruction {
    go2_control::RigidBodyState state{};
    go2_control::RigidBodyPlanningKinematics model{};
    JointPlannerFailure failure=JointPlannerFailure::kNumericalFailure;
    double position_residual_m=std::numeric_limits<double>::infinity();
    double velocity_residual_mps=std::numeric_limits<double>::infinity();
    double momentum_residual_nms=std::numeric_limits<double>::infinity();
    int projection_iterations=0;
    bool kinematics_valid=false;
};
inline bool FiniteBodyReference(const BodyReference &r) {
    if(!r.valid || !r.com_world.allFinite() || !r.com_velocity_world.allFinite() ||
       !r.angular_momentum_world.allFinite()) return false;
    for(int l=0;l<4;++l)
        if(!r.foot_reference_valid[l] || !r.foot_center_world[l].allFinite() || !r.foot_velocity_world[l].allFinite()) return false;
    return true;
}
// Lift a centroidal/foot reference through the actual robot model. At a
// configuration, COM + all four foot positions constrain base translation and
// twelve joints (15 variables). Body orientation is the integration state,
// not inferred from COM minus a constant offset. The remaining velocity is
// fixed by COM velocity, angular momentum and four foot velocities (18 rows).
// This is a kinematic/momentum certificate, NOT an actuator or collision one.
inline BodyReconstruction ReconstructBodyReference(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &seed,
    const BodyReference &reference,int max_iterations=24) {
    BodyReconstruction out;out.state=seed;
    if(!FiniteBodyReference(reference) || max_iterations<=0 || max_iterations>128) {
        out.failure=JointPlannerFailure::kInvalidInput;return out;
    }
    out.state.linear_vel_world.setZero();out.state.angular_vel_body.setZero();out.state.dq.setZero();
    std::array<int,15> columns{{0,1,2}};
    for(int j=0;j<12;++j) columns[j+3]=robot.MotorDof(j);
    for(int c:columns) if(c<0 || c>=go2_control::kGo2Nv) {
        out.failure=JointPlannerFailure::kObservationUnavailable;return out;
    }
    for(int iteration=0;iteration<=max_iterations;++iteration) {
        if(!robot.EvaluatePlanningKinematics(out.state,out.model)) {
            out.failure=JointPlannerFailure::kObservationUnavailable;return out;
        }
        for(bool valid:out.model.dynamics.foot_geometry_valid) if(!valid) {
            out.failure=JointPlannerFailure::kObservationUnavailable;return out;
        }
        if((out.state.q.array()<out.model.joint_lower.array()-1e-9).any() ||
           (out.state.q.array()>out.model.joint_upper.array()+1e-9).any()) return out;
        Eigen::Matrix<double,15,1> error;
        error.head<3>()=reference.com_world-out.model.dynamics.com_world;
        Eigen::Matrix<double,15,go2_control::kGo2Nv> full;
        full.topRows<3>()=out.model.com_jacobian_world;
        for(int l=0;l<4;++l) {
            error.segment<3>(3+3*l)=reference.foot_center_world[l]-out.model.dynamics.foot_pos_world[l];
            full.block<3,go2_control::kGo2Nv>(3+3*l,0)=out.model.dynamics.foot_jac_world[l];
        }
        out.position_residual_m=error.lpNorm<Eigen::Infinity>();
        if(out.position_residual_m<=1e-7) break;
        if(iteration==max_iterations) return out;
        Eigen::Matrix<double,15,15> jac;
        for(int j=0;j<15;++j)jac.col(j)=full.col(columns[j]);
        const Eigen::FullPivLU<Eigen::Matrix<double,15,15>> factor(jac);
        if(factor.rank()!=15 || factor.rcond()<1e-10) return out;
        Eigen::Matrix<double,15,1> correction=factor.solve(error);
        if(!correction.allFinite()) return out;
        double scale=std::min(1.0,0.04/std::max(1e-12,correction.head<3>().norm()));
        scale=std::min(scale,0.2/std::max(1e-12,correction.tail<12>().cwiseAbs().maxCoeff()));
        out.state.position_world+=scale*correction.head<3>();
        out.state.q+=scale*correction.tail<12>();
        ++out.projection_iterations;
    }
    Eigen::Matrix<double,18,18> map;
    Eigen::Matrix<double,18,1> target;
    map.topRows<3>()=out.model.com_jacobian_world;
    map.middleRows<3>(3)=out.model.angular_momentum_matrix_world;
    target.head<3>()=reference.com_velocity_world;target.segment<3>(3)=reference.angular_momentum_world;
    for(int l=0;l<4;++l) {
        map.block<3,18>(6+3*l,0)=out.model.dynamics.foot_jac_world[l];
        target.segment<3>(6+3*l)=reference.foot_velocity_world[l];
    }
    const Eigen::FullPivLU<Eigen::Matrix<double,18,18>> factor(map);
    if(factor.rank()!=18 || factor.rcond()<1e-10) return out;
    const Eigen::Matrix<double,18,1> velocity=factor.solve(target);
    if(!velocity.allFinite()) return out;
    go2_control::RigidBodyState next;
    if(!robot.IntegrateConfiguration(out.state,velocity,0,next)) return out;
    out.state=next;
    if(!robot.EvaluatePlanningKinematics(out.state,out.model)) return out;
    out.velocity_residual_mps=(out.model.com_velocity_world-reference.com_velocity_world).lpNorm<Eigen::Infinity>();
    out.momentum_residual_nms=(out.model.angular_momentum_world-reference.angular_momentum_world).lpNorm<Eigen::Infinity>();
    for(int l=0;l<4;++l)
        out.velocity_residual_mps=std::max(out.velocity_residual_mps,
            (out.model.dynamics.foot_jac_world[l]*out.model.dynamics.qvel-
             reference.foot_velocity_world[l]).lpNorm<Eigen::Infinity>());
    out.kinematics_valid=std::isfinite(out.velocity_residual_mps) &&
        std::isfinite(out.momentum_residual_nms) && out.velocity_residual_mps<1e-7 &&
        out.momentum_residual_nms<1e-7;
    if(out.kinematics_valid) out.failure=JointPlannerFailure::kNone;
    return out;
}
struct BodyTrajectoryKnot {
    TimeNs time{};
    go2_control::RigidBodyState state{};
};
struct BodyTrajectoryReconstruction {
    std::vector<BodyTrajectoryKnot> knots;
    JointPlannerFailure failure=JointPlannerFailure::kNumericalFailure;
    double max_position_residual_m=0;
    double max_velocity_residual_mps=0;
    double max_momentum_residual_nms=0;
    bool kinematics_valid=false;
};
using BodyReferenceFunction=std::function<BodyReference(TimeNs)>;
inline BodyTrajectoryReconstruction ReconstructBodyTrajectory(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &initial,
    const std::vector<TimeNs> &times,const BodyReferenceFunction &reference) {
    BodyTrajectoryReconstruction out;
    if(times.empty() || !reference || times.front().value<0) {
        out.failure=JointPlannerFailure::kInvalidInput;return out;
    }
    for(std::size_t k=1;k<times.size();++k)
        if(times[k]<=times[k-1] || times[k].value-times[k-1].value>100000000) {
            out.failure=JointPlannerFailure::kCoverageIncomplete;return out;
        }
    auto current=ReconstructBodyReference(robot,initial,reference(times.front()));
    if(!current.kinematics_valid) {out.failure=current.failure;return out;}
    if((current.state.position_world-initial.position_world).norm()>1e-7 ||
       (current.state.q-initial.q).norm()>1e-6 ||
       (current.state.linear_vel_world-initial.linear_vel_world).norm()>1e-7 ||
       (current.state.angular_vel_body-initial.angular_vel_body).norm()>1e-7 ||
       (current.state.dq-initial.dq).norm()>1e-6) {
        out.failure=JointPlannerFailure::kInitialConditionConflict;return out;
    }
    const auto retain=[&](TimeNs time,const BodyReconstruction &sample) {
        out.knots.push_back({time,sample.state});
        out.max_position_residual_m=std::max(out.max_position_residual_m,sample.position_residual_m);
        out.max_velocity_residual_mps=std::max(out.max_velocity_residual_mps,sample.velocity_residual_mps);
        out.max_momentum_residual_nms=std::max(out.max_momentum_residual_nms,sample.momentum_residual_nms);
    };
    retain(times.front(),current);
    for(std::size_t k=1;k<times.size();++k) {
        const auto span=times[k].value-times[k-1].value;
        const double dt=span*1e-9;
        const TimeNs midpoint{times[k-1].value+span/2};
        go2_control::RigidBodyState mid_seed,next_seed;
        if(!robot.IntegrateConfiguration(current.state,current.model.dynamics.qvel,
                (midpoint.value-times[k-1].value)*1e-9,mid_seed)) return out;
        auto mid=ReconstructBodyReference(robot,mid_seed,reference(midpoint));
        if(!mid.kinematics_valid) {out.failure=mid.failure;return out;}
        if(!robot.IntegrateConfiguration(current.state,mid.model.dynamics.qvel,dt,next_seed)) return out;
        current=ReconstructBodyReference(robot,next_seed,reference(times[k]));
        if(!current.kinematics_valid) {out.failure=current.failure;return out;}
        retain(times[k],current);
    }
    out.failure=JointPlannerFailure::kNone;out.kinematics_valid=true;return out;
}
}} // namespace
