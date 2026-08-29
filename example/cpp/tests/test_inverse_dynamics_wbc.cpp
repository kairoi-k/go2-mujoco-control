#include <cmath>
#include <iostream>

#include "go2_rigid_body.h"
#include "inverse_dynamics_wbc.h"
#include "srbd_mpc.h"

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

}  // namespace

int main()
{
    go2_control::Go2RigidBody model;
    if (!model.Load(GO2_MODEL_PATH))
        return 1;
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(0.0, 0.0, 0.42);
    state.q << 0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763;
    go2_control::RigidBodyDynamics dyn;
    if (!model.Evaluate(state, dyn))
        return 1;

    go2_control::IdWbcInput input;
    input.dynamics = dyn;
    input.desired_linear_acc_world = Eigen::Vector3d::Zero();
    input.desired_angular_acc_body = Eigen::Vector3d::Zero();
    input.contact.fill(true);

    go2_control::IdWbcOutput out;
    bool passed = go2_control::SolveInverseDynamicsWbc({}, input, out);
    passed &= Check(out.ok, "stand ID-WBC failed");
    passed &= Check(out.eq_residual < 1.0e-3, "floating-base residual");
    passed &= Check(out.rne_residual < 1.0e-3, "RNEA residual");
    double fz = 0.0;
    for (int i = 0; i < 4; ++i)
        fz += out.force[3 * i + 2];
    passed &= Check(
        std::abs(fz - dyn.mass_kg * 9.81) < 40.0, "ID-WBC gravity");
    passed &= Check(out.tau.cwiseAbs().maxCoeff() < 35.0, "tau limit");
    passed &= Check(std::isfinite(out.cost_terms.base_linear) &&
                        std::isfinite(out.cost_terms.stance_no_slip) &&
                        std::isfinite(out.cost_terms.torque),
                    "ID-WBC objective terms are not finite");
    passed &= Check(out.cost_terms.force_regularization >= 0.0,
                    "ID-WBC force cost is negative");

    // A terrain hold must keep every selected contact physically loadable,
    // rather than allowing the solver to satisfy the base equations with a
    // near-zero held-foot force.
    go2_control::IdWbcParams loaded_params = {};
    loaded_params.min_normal_n = 20.0;
    go2_control::IdWbcOutput loaded;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc(
            loaded_params, input, loaded) && loaded.ok,
        "loaded-contact ID-WBC failed");
    for (int i = 0; i < 4; ++i)
        passed &= Check(
            loaded.force[3 * i + 2] >= 19.9,
            "held contact fell below minimum normal force");

    input.has_terrain_plan = true;
    input.terrain_plan.plan_id = 5;
    input.terrain_plan.plan_epoch = 6;
    input.terrain_plan.map_epoch = 7;
    input.terrain_plan.generated_at_s = 1.0;
    input.terrain_plan.valid_until_s = 2.0;
    input.measured_contact.fill(true);
    input.measured_contact_valid = true;
    input.planned_contact = input.contact;
    input.planned_contact_valid = true;
    go2_control::IdWbcOutput terrain;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc({}, input, terrain) &&
            terrain.ok && terrain.terrain_plan_consumed &&
            terrain.terrain_plan.plan_epoch == 6,
        "terrain ID-WBC identity/contact interface failed");
    input.has_terrain_plan = false;
    input.contact = {true, false, false, true};
    go2_control::IdWbcOutput two;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc({}, input, two) && two.ok,
        "2-contact ID-WBC failed");
    passed &= Check(two.eq_residual < 5.0e-3, "2-contact eq residual");
    passed &= Check(two.force.segment<3>(3).norm() < 0.2, "FL force");
    passed &= Check(two.force.segment<3>(6).norm() < 0.2, "RR force");

    go2_control::IdWbcParams hard = {};
    hard.hard_stance_no_slip = true;
    input.desired_linear_acc_world = Eigen::Vector3d(0.4, 0.0, 0.0);
    input.have_stance_acc = true;
    go2_control::IdWbcOutput locked;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc(hard, input, locked) && locked.ok,
        "hard no-slip 2-contact failed");
    const Eigen::Vector3d acc_fr =
        dyn.foot_jac_world[0] * locked.qdd;
    const Eigen::Vector3d acc_rl =
        dyn.foot_jac_world[3] * locked.qdd;
    passed &= Check(acc_fr.norm() < 0.05, "FR foot acc");
    passed &= Check(acc_rl.norm() < 0.05, "RL foot acc");

    go2_control::IdWbcParams aniso = {};
    aniso.w_stance_no_slip = 250.0;
    aniso.w_stance_no_slip_x = 25.0;
    aniso.w_base_lin = 80.0;
    aniso.hard_stance_no_slip = false;
    input.desired_linear_acc_world = Eigen::Vector3d(2.0, 0.0, 0.0);
    input.have_stance_acc = true;
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    go2_control::IdWbcOutput push;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc(aniso, input, push) && push.ok,
        "aniso X no-slip failed");
    double fx = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        if (input.contact[static_cast<std::size_t>(i)])
            fx += push.force[3 * i];
    }
    passed &= Check(fx > 8.0, "aniso X should allow sagittal GRF");

    if (!passed)
    {
        std::cerr << "eq=" << out.eq_residual
                  << " rne=" << out.rne_residual
                  << " fz=" << fz
                  << " two_eq=" << two.eq_residual << "\n";
        return 1;
    }
    std::cout << "inverse dynamics wbc checks passed. eq=" << out.eq_residual
              << " rne=" << out.rne_residual << "\n";
    return 0;
}
