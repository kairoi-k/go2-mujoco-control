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

    input.contact = {true, false, false, true};
    go2_control::IdWbcOutput two;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc({}, input, two) && two.ok,
        "2-contact ID-WBC failed");
    passed &= Check(two.eq_residual < 5.0e-3, "2-contact eq residual");
    passed &= Check(two.force.segment<3>(3).norm() < 0.2, "FL force");
    passed &= Check(two.force.segment<3>(6).norm() < 0.2, "RR force");

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
