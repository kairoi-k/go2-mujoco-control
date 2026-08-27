#include <cmath>
#include <iostream>

#include "go2_rigid_body.h"

#ifndef GO2_MODEL_PATH
#define GO2_MODEL_PATH "unitree_robots/go2/go2.xml"
#endif

namespace
{

bool Check(bool ok, const char *msg)
{
    if (!ok)
        std::cerr << msg << "\n";
    return ok;
}

go2_control::RigidBodyState StandState()
{
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(0.0, 0.0, 0.42);
    state.quat_world_from_body = Eigen::Quaterniond::Identity();
    state.q << 0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763;
    return state;
}

}  // namespace

int main()
{
    go2_control::Go2RigidBody model;
    if (!model.Load(GO2_MODEL_PATH))
    {
        std::cerr << "Failed to load " << GO2_MODEL_PATH << "\n";
        return 1;
    }
    go2_control::RigidBodyDynamics dyn;
    bool passed = model.Evaluate(StandState(), dyn);
    passed &= Check(dyn.valid, "dynamics invalid");
    passed &= Check(
        std::abs(dyn.mass_kg - 15.206) < 0.2, "mass mismatch");
    passed &= Check(dyn.com_world.z() > 0.2 && dyn.com_world.z() < 0.5,
                    "com height");
    passed &= Check(
        dyn.mass_matrix.llt().info() == Eigen::Success, "M not SPD");
    // Gravity generalized force on floating z is ~+mg.
    passed &= Check(
        std::abs(dyn.bias[2] - dyn.mass_kg * 9.81) < 25.0,
        "bias_z is not mg");
    Eigen::Matrix<double, go2_control::kGo2Nv, 1> qacc =
        Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
    qacc[2] = 0.3;
    qacc[7] = -0.2;
    const double rne = model.InverseDynamicsResidual(dyn, qacc);
    passed &= Check(rne < 0.5, "RNEA residual");
    std::array<double, go2::kJointCount> joint_positions{};
    for (std::size_t joint = 0; joint < joint_positions.size(); ++joint)
        joint_positions[joint] = StandState().q[
            static_cast<Eigen::Index>(joint)];
    const auto fk_feet = go2::AllFootPositions(joint_positions);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        passed &= Check(dyn.foot_pos_world[leg].z() > -0.01, "foot z");
        const Eigen::Vector3d expected(
            StandState().position_world.x() + fk_feet[leg].x,
            StandState().position_world.y() + fk_feet[leg].y,
            StandState().position_world.z() + fk_feet[leg].z);
        passed &= Check(
            (dyn.foot_pos_world[leg] - expected).norm() < 0.003,
            "MJCF foot point does not match analytical FK");
    }

    if (!passed)
    {
        std::cerr << "mass=" << dyn.mass_kg
                  << " comz=" << dyn.com_world.z()
                  << " biasz=" << dyn.bias[2]
                  << " rne=" << rne << "\n";
        return 1;
    }
    std::cout << "go2 rigid body checks passed. mass=" << dyn.mass_kg
              << " rne=" << rne << "\n";
    return 0;
}
