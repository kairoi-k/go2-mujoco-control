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

    go2_control::SrbdMpcInput indexed = input;
    indexed.has_time_indexed_footholds = true;
    for (int k = 0; k < params.horizon; ++k)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            indexed.foot_from_com_world_horizon[static_cast<std::size_t>(k)][leg] =
                input.foot_from_com_world[leg];
            indexed.foot_valid[static_cast<std::size_t>(k)][leg] = true;
        }
    }
    go2_control::SrbdMpcOutput indexed_output;
    passed &= Check(
        go2_control::SolveSrbdMpc(params, indexed, indexed_output) &&
            indexed_output.ok,
        "time-indexed foothold MPC failed");
    passed &= Check(
        (indexed_output.first_linear_acc - four.first_linear_acc).norm() < 1e-6,
        "time-indexed fallback changed the flat first acceleration");
    indexed.foot_from_com_world_horizon[4][0].x() += 0.25;
    indexed.foot_from_com_world_horizon[4][1].x() += 0.25;
    go2_control::SrbdMpcOutput shifted_output;
    passed &= Check(
        go2_control::SolveSrbdMpc(params, indexed, shifted_output) &&
            shifted_output.ok,
        "future foothold MPC failed");
    passed &= Check(
        (shifted_output.predicted_state - indexed_output.predicted_state).norm() >
            1.0e-7,
        "future foothold did not enter the MPC horizon");

    go2_control::SrbdMpcInput terrain_indexed = indexed;
    terrain_indexed.has_terrain_plan = true;
    terrain_indexed.plan_id = 17;
    terrain_indexed.plan_epoch = 23;
    terrain_indexed.terrain_plan.plan_id = 17;
    terrain_indexed.terrain_plan.plan_epoch = 23;
    terrain_indexed.terrain_plan.map_epoch = 4;
    terrain_indexed.terrain_plan.generated_at_s = 1.0;
    terrain_indexed.terrain_plan.valid_until_s = 2.0;
    terrain_indexed.measured_contact.fill(true);
    terrain_indexed.measured_contact_valid = true;
    go2_control::SrbdMpcOutput terrain_output;
    passed &= Check(
        go2_control::SolveSrbdMpc(params, terrain_indexed, terrain_output) &&
            terrain_output.ok && terrain_output.terrain_plan_consumed &&
            terrain_output.terrain_plan.plan_epoch == 23,
        "terrain SRBD identity/contact interface failed");

    // A contact transition must use the foothold at that same knot. This
    // catches the old single-anchor behavior, where touchdown lever arms were
    // silently reused across the whole horizon.
    auto transition = indexed;
    transition.contact[4][0] = false;
    transition.foot_from_com_world_horizon[4][0].x() += 0.40;
    go2_control::SrbdMpcOutput transition_output;
    passed &= Check(
        go2_control::SolveSrbdMpc(params, transition, transition_output) &&
            transition_output.ok,
        "contact-transition MPC failed");
    passed &= Check(
        (transition_output.predicted_state - indexed_output.predicted_state).norm() >
            1.0e-7,
        "touchdown-knot lever arm was not consumed");

    auto missing_lever_arm = indexed;
    missing_lever_arm.foot_valid[4][0] = false;
    go2_control::SrbdMpcOutput missing_output;
    passed &= Check(
        !go2_control::SolveSrbdMpc(params, missing_lever_arm, missing_output),
        "missing contact lever arm was not rejected");

    auto mismatched_plan = terrain_indexed;
    mismatched_plan.plan_id = 18;
    go2_control::SrbdMpcOutput mismatch_output;
    passed &= Check(
        !go2_control::SolveSrbdMpc(params, mismatched_plan, mismatch_output),
        "MPC accepted mismatched plan identity");

    auto timed_body = indexed;
    timed_body.has_time_indexed_reference = true;
    for (int k = 0; k < params.horizon; ++k)
        timed_body.reference_horizon[static_cast<std::size_t>(k)] =
            input.reference;
    timed_body.reference_horizon[4][3] = 0.30;
    go2_control::SrbdMpcOutput timed_body_output;
    passed &= Check(
        go2_control::SolveSrbdMpc(params, timed_body, timed_body_output) &&
            timed_body_output.ok,
        "time-indexed body reference MPC failed");
    passed &= Check(
        (timed_body_output.predicted_state - indexed_output.predicted_state).norm() >
            1.0e-7,
        "body reference was not aligned to its knot");
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
