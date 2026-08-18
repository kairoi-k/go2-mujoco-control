#include <cmath>
#include <iostream>

#include "srbd_mpc.h"

namespace
{

bool Check(bool ok, const char *msg)
{
    if (!ok)
        std::cerr << msg << "\n";
    return ok;
}

}  // namespace

int main()
{
    go2_control::SrbdMpcParams params;
    params.horizon = 8;
    params.dt_s = 0.05;
    params.mass_kg = 15.206;
    params.inertia_com_world = Eigen::Matrix3d::Zero();
    params.inertia_com_world.diagonal() << 0.07, 0.12, 0.15;

    go2_control::SrbdMpcInput input;
    input.state[5] = 0.42;
    input.reference[5] = 0.42;
    input.reference[9] = 0.0;
    input.foot_from_com_world[0] = Eigen::Vector3d(0.20, -0.12, -0.28);
    input.foot_from_com_world[1] = Eigen::Vector3d(0.20, 0.12, -0.28);
    input.foot_from_com_world[2] = Eigen::Vector3d(-0.20, -0.12, -0.28);
    input.foot_from_com_world[3] = Eigen::Vector3d(-0.20, 0.12, -0.28);
    for (int k = 0; k < params.horizon; ++k)
        input.contact[k].fill(true);

    go2_control::SrbdMpcOutput four;
    bool passed = go2_control::SolveSrbdMpc(params, input, four);
    passed &= Check(four.ok, "4-contact MPC failed");
    double total_z = 0.0;
    for (int i = 0; i < 4; ++i)
        total_z += four.first_force[3 * i + 2];
    passed &= Check(
        std::abs(total_z - params.mass_kg * params.gravity_mps2) < 25.0,
        "4-contact did not carry gravity");
    passed &= Check(
        std::abs(four.first_linear_acc.z()) < 2.5,
        "4-contact az not near 0");

    go2_control::FillTrotContactSchedule(
        0.24, 0.60, 0.75, params.horizon, params.dt_s, input.contact);
    go2_control::SrbdMpcOutput two;
    passed &= Check(
        go2_control::SolveSrbdMpc(params, input, two) && two.ok,
        "2-contact MPC failed");
    total_z = 0.0;
    int stance = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (input.contact[0][static_cast<std::size_t>(i)])
        {
            ++stance;
            total_z += two.first_force[3 * i + 2];
        }
        else
        {
            passed &= Check(
                two.first_force.segment<3>(3 * i).norm() < 0.05,
                "swing force not zero");
        }
    }
    passed &= Check(stance == 2, "trot schedule is not 2-contact");
    passed &= Check(
        std::abs(total_z - params.mass_kg * params.gravity_mps2) < 40.0,
        "2-contact did not carry gravity");

    if (!passed)
    {
        std::cerr << "4z=" << four.first_force.transpose()
                  << " 2z=" << two.first_force.transpose() << "\n";
        return 1;
    }
    std::cout << "srbd mpc checks passed.\n";
    return 0;
}
