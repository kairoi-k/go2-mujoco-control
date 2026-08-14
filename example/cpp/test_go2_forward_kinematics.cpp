#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#include "go2_forward_kinematics.h"

namespace
{

constexpr std::array<const char *, go2::kLegCount> kLegNames = {
    "FR", "FL", "RR", "RL"};

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckZeroPose()
{
    const std::array<double, go2::kJointCount> zero_pose{};
    const auto feet = go2::AllFootPositions(zero_pose);

    const std::array<go2::Vec3, go2::kLegCount> expected = {{
        {0.1934, -0.1420, -0.4260},
        {0.1934, 0.1420, -0.4260},
        {-0.1934, -0.1420, -0.4260},
        {-0.1934, 0.1420, -0.4260},
    }};

    for (std::size_t i = 0; i < feet.size(); ++i)
    {
        if (!Near(feet[i].x, expected[i].x) ||
            !Near(feet[i].y, expected[i].y) ||
            !Near(feet[i].z, expected[i].z))
        {
            std::cerr << "Zero-pose check failed for " << kLegNames[i] << "\n";
            return false;
        }
    }
    return true;
}

bool CheckStandPoseSymmetry()
{
    const std::array<double, go2::kJointCount> stand_pose = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};

    const auto feet = go2::AllFootPositions(stand_pose);
    const double tolerance = 1e-6;

    return Near(feet[0].x, feet[1].x, tolerance) &&
           Near(feet[2].x, feet[3].x, tolerance) &&
           Near(feet[0].y, -feet[1].y, tolerance) &&
           Near(feet[2].y, -feet[3].y, tolerance) &&
           Near(feet[0].z, feet[1].z, tolerance) &&
           Near(feet[2].z, feet[3].z, tolerance);
}

bool CheckAgainstMuJoCo(
    const std::array<double, go2::kJointCount> &joint_positions,
    const std::array<go2::Vec3, go2::kLegCount> &calculated_feet)
{
    char error[1024] = {};
    mjModel *model = mj_loadXML(GO2_MODEL_PATH, nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cerr << "Failed to load Go2 model: " << error << "\n";
        return false;
    }

    mjData *data = mj_makeData(model);
    if (data == nullptr)
    {
        std::cerr << "Failed to allocate MuJoCo data\n";
        mj_deleteModel(model);
        return false;
    }

    data->qpos[0] = 0.0;
    data->qpos[1] = 0.0;
    data->qpos[2] = 0.0;
    data->qpos[3] = 1.0;
    data->qpos[4] = 0.0;
    data->qpos[5] = 0.0;
    data->qpos[6] = 0.0;
    const std::array<const char *, go2::kJointCount> joint_names = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    for (std::size_t i = 0; i < joint_positions.size(); ++i)
    {
        const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_names[i]);
        if (joint_id < 0)
        {
            std::cerr << "Joint not found: " << joint_names[i] << "\n";
            mj_deleteData(data);
            mj_deleteModel(model);
            return false;
        }
        data->qpos[model->jnt_qposadr[joint_id]] = joint_positions[i];
    }
    mj_forward(model, data);

    const int base_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    const std::array<const char *, go2::kLegCount> foot_body_names = {
        "FR_foot", "FL_foot", "RR_foot", "RL_foot"};

    bool passed = base_id >= 0;
    for (std::size_t i = 0; i < foot_body_names.size(); ++i)
    {
        const int foot_id = mj_name2id(model, mjOBJ_BODY, foot_body_names[i]);
        if (foot_id < 0)
        {
            passed = false;
            break;
        }

        const go2::Vec3 mujoco_foot = {
            data->xpos[3 * foot_id] - data->xpos[3 * base_id],
            data->xpos[3 * foot_id + 1] - data->xpos[3 * base_id + 1],
            data->xpos[3 * foot_id + 2] - data->xpos[3 * base_id + 2],
        };
        const double tolerance = 1e-9;
        const bool leg_passed =
            Near(calculated_feet[i].x, mujoco_foot.x, tolerance) &&
            Near(calculated_feet[i].y, mujoco_foot.y, tolerance) &&
            Near(calculated_feet[i].z, mujoco_foot.z, tolerance);
        if (!leg_passed)
        {
            std::cerr << kLegNames[i]
                      << " calculated=("
                      << calculated_feet[i].x << ", "
                      << calculated_feet[i].y << ", "
                      << calculated_feet[i].z << ") mujoco=("
                      << mujoco_foot.x << ", "
                      << mujoco_foot.y << ", "
                      << mujoco_foot.z << ")\n";
        }
        passed = passed && leg_passed;
    }

    mj_deleteData(data);
    mj_deleteModel(model);

    if (!passed)
    {
        std::cerr << "Forward kinematics does not match MuJoCo body positions\n";
    }
    return passed;
}

} // namespace

int main()
{
    if (!CheckZeroPose())
    {
        return 1;
    }
    if (!CheckStandPoseSymmetry())
    {
        std::cerr << "Stand-pose symmetry check failed\n";
        return 1;
    }

    const std::array<double, go2::kJointCount> stand_pose = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};
    const auto feet = go2::AllFootPositions(stand_pose);
    if (!CheckAgainstMuJoCo(stand_pose, feet))
    {
        return 1;
    }

    std::cout << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < feet.size(); ++i)
    {
        std::cout << kLegNames[i]
                  << ": x=" << feet[i].x
                  << " y=" << feet[i].y
                  << " z=" << feet[i].z << " m\n";
    }
    std::cout << "Forward kinematics checks passed, including MuJoCo comparison.\n";
    return 0;
}
