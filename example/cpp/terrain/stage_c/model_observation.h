#pragma once
#include "input_adapter.h"
#include "go2_rigid_body.h"
namespace go2_terrain { namespace stage_c {
// Assemble one immutable observation through the same articulated model used
// by WBC. Contact points are explicit producer inputs, never copied from sites.
// The caller supplies measured contact evidence and its observed terrain patch;
// planned or applied masks cannot become measured through this adapter.
inline InputAdapterResult CaptureModelPlanningObservation(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &state,
    const PlanningIdentity &identity,const ContactEvidence &measured,
    const std::array<TimedPoint,4> &measured_surface_points,
    const MapObservation &map,const Phase1CommandAuthority &command,
    const PlanningBudget &budget,go2_control::RigidBodyPlanningKinematics &model) {
    InputAdapterResult failure;failure.failure=JointPlannerFailure::kObservationUnavailable;
    if(!robot.EvaluatePlanningKinematics(state,model)) return failure;
    RawPlanningObservation raw;raw.identity=identity;raw.measured_contact=measured;
    raw.map=map;raw.command=command;raw.budget=budget;
    const auto point=[&](const Eigen::Vector3d &p,PointRole role) {
        return TimedPoint{{p.x(),p.y(),p.z()},Frame::kWorld,identity.source_state_time,true,role};
    };
    const auto vector=[](const Eigen::Vector3d &v){return go2::Vec3{v.x(),v.y(),v.z()};};
    raw.body.valid=true;raw.body.model_com_valid=true;
    raw.body.base_position_world=point(state.position_world,PointRole::kBodyOrigin);
    raw.body.model_com_world=point(model.dynamics.com_world,PointRole::kCenterOfMass);
    raw.body.base_velocity_world=vector(state.linear_vel_world);
    raw.body.com_velocity_world=vector(model.com_velocity_world);
    raw.body.angular_velocity_body=vector(state.angular_vel_body);
    raw.body.mass_kg=model.dynamics.mass_kg;
    const Eigen::Matrix3d rotation=state.quat_world_from_body.normalized().toRotationMatrix();
    raw.body.pitch_rad=std::asin(std::clamp(-rotation(2,0),-1.0,1.0));
    raw.body.roll_rad=std::atan2(rotation(2,1),rotation(2,2));
    raw.body.yaw_rad=std::atan2(rotation(1,0),rotation(0,0));
    for(int l=0;l<4;++l) {
        if(!model.dynamics.foot_geometry_valid[l]) return failure;
        raw.feet[l].foot_site_world=point(model.dynamics.foot_site_world[l],PointRole::kFootSite);
        raw.feet[l].foot_collision_center_world=point(model.dynamics.foot_pos_world[l],PointRole::kFootCollisionCenter);
        if(measured.mask[l]) {
            raw.feet[l].measured_support_anchor_world=measured_surface_points[l];
            raw.feet[l].measured_support_anchor_valid=TimedPointValidAt(
                measured_surface_points[l],PointRole::kSurfaceContactPoint,Frame::kWorld,identity.source_state_time);
        }
    }
    return NormalizePlanningInput(raw);
}
}} // namespace
