#include <cmath>
#include <iostream>
#include <limits>

#include "motion_frame_utils.h"
#include "trot_types.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckIdentity()
{
    go2_control::Vector3 body{};
    return go2_control::WorldToBodyVelocity(
               {1.0, 0.0, 0.0, 0.0},
               {0.1, -0.2, 0.3}, body) &&
           Near(body[0], 0.1) && Near(body[1], -0.2) &&
           Near(body[2], 0.3);
}

bool CheckYawRotation()
{
    const double half_pi_over_two = 0.25 * 3.14159265358979323846;
    const double c = std::cos(half_pi_over_two);
    const double s = std::sin(half_pi_over_two);
    go2_control::Vector3 body{};
    return go2_control::WorldToBodyVelocity(
               {c, 0.0, 0.0, s}, {0.0, 1.0, 0.0}, body) &&
           Near(body[0], 1.0) && Near(body[1], 0.0) &&
           Near(body[2], 0.0);
}

bool CheckFullQuaternionAndInvalidInput()
{
    const double half_roll = 0.25 * 3.14159265358979323846;
    const double c = std::cos(half_roll);
    const double s = std::sin(half_roll);
    const go2_control::Vector3 expected_body{0.1, 0.2, -0.3};
    const go2_control::Vector3 world_velocity{0.1, 0.3, 0.2};
    go2_control::Vector3 body{};
    if (!go2_control::WorldToBodyVelocity(
            {c, s, 0.0, 0.0}, world_velocity, body) ||
        !Near(body[0], expected_body[0]) ||
        !Near(body[1], expected_body[1]) ||
        !Near(body[2], expected_body[2]))
    {
        return false;
    }
    const go2_control::Vector3 invalid_velocity{
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    return !go2_control::WorldToBodyVelocity(
               {0.0, 0.0, 0.0, 0.0}, invalid_velocity, body);
}

void SetPoseInputs(
    unitree_go::msg::dds_::LowState_ &low,
    unitree_go::msg::dds_::SportModeState_ &high,
    const std::array<float, 4> &quaternion,
    const std::array<float, 3> &position)
{
    low.imu_state().quaternion() = quaternion;
    low.imu_state().rpy()[2] = 0.25f;
    high.position() = position;
}

bool CheckValidatedNormalizedWorldPose()
{
    unitree_go::msg::dds_::LowState_ low;
    unitree_go::msg::dds_::SportModeState_ high;
    const double c = std::cos(0.25 * 3.14159265358979323846);
    const double s = std::sin(0.25 * 3.14159265358979323846);
    SetPoseInputs(low, high,
                  {static_cast<float>(2.0 * c), 0.0f, 0.0f,
                   static_cast<float>(2.0 * s)},
                  {1.0f, 2.0f, 3.0f});

    go2_trot::WorldPose pose;
    return go2_trot::TryComputeValidatedNormalizedWorldPose(low, high, pose) &&
        Near(pose.quaternion[0], c, 1.0e-6) &&
        Near(pose.quaternion[1], 0.0) && Near(pose.quaternion[2], 0.0) &&
        Near(pose.quaternion[3], s, 1.0e-6) &&
        Near(pose.base.x, 1.0, 1.0e-6) &&
        Near(pose.base.y, 2.02557, 1.0e-6) &&
        Near(pose.base.z, 2.95768, 1.0e-6) &&
        Near(pose.yaw_rad, 0.25, 1.0e-6);
}

bool CheckInvalidWorldPoseInputs()
{
    unitree_go::msg::dds_::LowState_ low;
    unitree_go::msg::dds_::SportModeState_ high;
    SetPoseInputs(low, high, {1.0f, 0.0f, 0.0f, 0.0f},
                  {1.0f, 2.0f, 3.0f});

    go2_trot::WorldPose pose;
    if (!go2_trot::TryComputeValidatedNormalizedWorldPose(low, high, pose))
        return false;

    high.position()[1] = std::numeric_limits<float>::quiet_NaN();
    if (go2_trot::TryComputeValidatedNormalizedWorldPose(low, high, pose) ||
        pose.base.x != 0.0 || pose.base.y != 0.0 || pose.base.z != 0.0 ||
        pose.quaternion[0] != 0.0 || pose.quaternion[1] != 0.0 ||
        pose.quaternion[2] != 0.0 || pose.quaternion[3] != 0.0)
        return false;

    high.position()[1] = 2.0f;
    low.imu_state().quaternion()[0] =
        std::numeric_limits<float>::quiet_NaN();
    if (go2_trot::TryComputeValidatedNormalizedWorldPose(low, high, pose))
        return false;

    low.imu_state().quaternion() = {1.0e-9f, 0.0f, 0.0f, 0.0f};
    if (go2_trot::TryComputeValidatedNormalizedWorldPose(low, high, pose))
        return false;

    low.imu_state().quaternion() = {1.0f, 0.0f, 0.0f, 0.0f};
    low.imu_state().rpy()[2] =
        std::numeric_limits<float>::quiet_NaN();
    return !go2_trot::TryComputeValidatedNormalizedWorldPose(
        low, high, pose);
}

bool CheckLegacyWorldPoseSemantics()
{
    unitree_go::msg::dds_::LowState_ low;
    unitree_go::msg::dds_::SportModeState_ high;
    const double c = std::cos(0.25 * 3.14159265358979323846);
    const double s = std::sin(0.25 * 3.14159265358979323846);
    SetPoseInputs(low, high,
                  {static_cast<float>(2.0 * c), 0.0f, 0.0f,
                   static_cast<float>(2.0 * s)},
                  {1.0f, 2.0f, 3.0f});

    const auto pose = go2_trot::ComputeWorldPose(low, high);
    return Near(pose.quaternion[0], 2.0 * c, 1.0e-6) &&
        Near(pose.quaternion[1], 0.0) && Near(pose.quaternion[2], 0.0) &&
        Near(pose.quaternion[3], 2.0 * s, 1.0e-6) &&
        Near(pose.base.x, 0.92329, 1.0e-6) &&
        Near(pose.base.y, 2.10228, 1.0e-6) &&
        Near(pose.base.z, 2.95768, 1.0e-6) &&
        Near(pose.yaw_rad, 0.25, 1.0e-6);
}

} // namespace

int main()
{
    if (!CheckIdentity() || !CheckYawRotation() ||
        !CheckFullQuaternionAndInvalidInput() ||
        !CheckValidatedNormalizedWorldPose() ||
        !CheckInvalidWorldPoseInputs() ||
        !CheckLegacyWorldPoseSemantics())
    {
        std::cerr << "Motion frame checks failed\n";
        return 1;
    }
    std::cout << "Motion frame checks passed.\n";
    return 0;
}
