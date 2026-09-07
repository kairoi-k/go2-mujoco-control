#pragma once
#include "trot_types.h"
#include "go2_rigid_body.h"
namespace go2_trot {
// Single production state conversion shared by WBC and the planning worker.
inline go2_control::RigidBodyState MakeRigidBodyState(
    const unitree_go::msg::dds_::LowState_ &low,
    const unitree_go::msg::dds_::SportModeState_ &high,
    const Eigen::Vector3d &linear_vel_world)
{
    const WorldPose pose = ComputeWorldPose(low, high);
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(pose.base.x, pose.base.y, pose.base.z);
    state.quat_world_from_body = Eigen::Quaterniond(
        pose.quaternion[0], pose.quaternion[1],
        pose.quaternion[2], pose.quaternion[3]);
    state.linear_vel_world = linear_vel_world;
    state.angular_vel_body = Eigen::Vector3d(
        low.imu_state().gyroscope()[0],
        low.imu_state().gyroscope()[1],
        low.imu_state().gyroscope()[2]);
    for (int i = 0; i < kMotorCount; ++i)
    {
        state.q[i] = low.motor_state()[i].q();
        state.dq[i] = low.motor_state()[i].dq();
    }
    return state;
}

}
