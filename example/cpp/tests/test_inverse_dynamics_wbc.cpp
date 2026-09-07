#include <cmath>
#include <iostream>

#include "go2_rigid_body.h"
#include "inverse_dynamics_wbc.h"
#include "id_wbc_certificate.h"
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
    passed &= Check(out.qp_converged, "stand ID-WBC QP did not converge");
    passed &= Check(!out.qp_recovery_used,
                    "nominal stand unexpectedly used QP recovery");
    passed &= Check(out.primary_iterations == out.iterations &&
                        out.recovery_iterations == 0,
                    "nominal QP iteration accounting");
    passed &= Check(out.solution_finite, "stand ID-WBC solution is not finite");
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

    // The independent certificate checks an actual MuJoCo-backed solution,
    // disregards convergence flags, and detects tampering of an accepted tau.
    auto certificate_proposal = out;
    certificate_proposal.qp_converged = false;
    const auto stand_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        {}, input, certificate_proposal);
    passed &= Check(stand_certificate.valid && stand_certificate.feasible,
                    "physical stand certificate must not depend on convergence flag");
    certificate_proposal.tau[0] += 1.0;
    certificate_proposal.ok = true;
    certificate_proposal.qp_converged = true;
    const auto tampered_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        {}, input, certificate_proposal);
    passed &= Check(!tampered_certificate.feasible &&
                    tampered_certificate.max_joint_dynamics_residual_Nm > 0.99,
                    "accepted output flags cannot certify a tampered joint torque");
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

    // A raised support plane must constrain force along its normal, not
    // world-Z. This catches the mixed-height cone regression while retaining
    // the flat default when normals are not supplied.
    const Eigen::Vector3d tilted_normal = Eigen::Vector3d(-0.10, 0.0, 0.995).normalized();
    input.contact_normal.fill(tilted_normal);
    input.contact_normal_valid.fill(true);
    go2_control::IdWbcOutput tilted;
    passed &= Check(
        go2_control::SolveInverseDynamicsWbc(loaded_params, input, tilted) &&
            tilted.ok,
        "tilted-contact ID-WBC failed");
    for (int i = 0; i < 4; ++i)
        passed &= Check(
            tilted.normal_force[i] >= 19.9 && tilted.normal_force[i] <= 180.1,
            "tilted contact normal limits");
    input.contact_normal_valid.fill(false);

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
    // A planned-only leg may be metadata for prediction, never a WBC
    // safety contact.
    input.measured_contact[3] = false;
    input.fused_contact = input.measured_contact;
    input.fused_contact_valid = true;
    input.contact[3] = true;
    go2_control::IdWbcOutput planned_only;
    passed &= Check(
        !go2_control::SolveInverseDynamicsWbc({}, input, planned_only),
        "planned contact entered ID-WBC safety mask");
    input.measured_contact[3] = true;
    input.contact[3] = false;
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


    // Audit F02: independent position finite differences, then an equivalent
    // acceleration-target formulation through the real production WBC QP.
    // Keeping M/h/J unchanged isolates Jdot*qvel from Coriolis forces.
    auto moving_state = state;
    moving_state.dq << 0.4, 3.0, -4.0, -0.3, -2.5, 3.5,
                       0.2, 2.0, -3.0, -0.4, -3.0, 4.0;
    go2_control::RigidBodyDynamics moving, plus, minus;
    passed &= Check(model.Evaluate(moving_state, moving), "moving dynamics");
    constexpr double dt = 1.0e-4;
    auto shifted = moving_state;
    shifted.q += dt * moving_state.dq;
    passed &= Check(model.Evaluate(shifted, plus), "plus dynamics");
    shifted.q = moving_state.q - dt * moving_state.dq;
    passed &= Check(model.Evaluate(shifted, minus), "minus dynamics");

    // Inactive contact forces are structurally absent from the intended
    // WBC formulation.  These MuJoCo-backed fixtures keep the plant
    // residual and active-contact checks while requiring exact zero on
    // aerial/mixed inactive legs, including a nonzero inactive force_ref.
    const auto check_active_force_mask =
        [&](const char *label,
            const go2_control::RigidBodyDynamics &fixture_dyn,
            const std::array<bool, 4> &contact_mask) {
            go2_control::IdWbcInput masked;
            masked.dynamics = fixture_dyn;
            masked.desired_linear_acc_world = Eigen::Vector3d::Zero();
            masked.desired_angular_acc_body = Eigen::Vector3d::Zero();
            masked.contact = contact_mask;
            masked.contact_normal.fill(Eigen::Vector3d::Zero());
            masked.contact_normal_valid.fill(false);
            masked.swing_acc_world.fill(Eigen::Vector3d::Zero());
            masked.stance_acc_world.fill(Eigen::Vector3d::Zero());
            masked.have_force_ref = true;
            masked.force_ref.setZero();
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                if (!contact_mask[leg])
                    masked.force_ref.segment<3>(3 * static_cast<int>(leg)) =
                        Eigen::Vector3d(4.0, -3.0, 2.0);
            go2_control::IdWbcParams masked_params = {};
            masked_params.w_force_track = 1.0;
            go2_control::IdWbcOutput masked_out;
            bool fixture_passed = go2_control::SolveInverseDynamicsWbc(
                masked_params, masked, masked_out) && masked_out.ok;
            fixture_passed &= Check(masked_out.solution_finite,
                                    "masked solution finite");
            fixture_passed &= Check(masked_out.rne_residual < 5.0e-2,
                                    "masked rigid-body residual");
            fixture_passed &= Check(masked_out.eq_residual < 5.0e-2,
                                    "masked floating-base residual");
            fixture_passed &= Check(
                masked_out.max_tau_violation_nm <= 5.1e-2,
                "masked torque limit");
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (contact_mask[leg])
                {
                    fixture_passed &= Check(
                        masked_out.normal_force[leg] >=
                            masked_params.min_normal_n - 1.0e-1 &&
                            masked_out.normal_force[leg] <=
                            masked_params.max_normal_n + 1.0e-1,
                        "masked active normal limit");
                }
                else
                {
                    fixture_passed &= Check(
                        masked_out.force
                                .segment<3>(3 * static_cast<int>(leg))
                                .cwiseAbs()
                                .maxCoeff() <= 1.0e-12,
                        "inactive force must be exact zero");
                }
            }
            if (!fixture_passed)
                std::cerr << "active-force fixture failed: " << label << "\n";
            return fixture_passed;
        };
    passed &= check_active_force_mask(
        "aerial qvel0", dyn, {false, false, false, false});
    passed &= check_active_force_mask(
        "mixed qvel0", dyn, {true, false, false, true});
    passed &= check_active_force_mask(
        "aerial moving", moving, {false, false, false, false});
    passed &= check_active_force_mask(
        "mixed moving", moving, {true, false, false, true});

    std::array<Eigen::Vector3d, 4> bias_acc;
    double fd_error = 0.0;
    for (int leg = 0; leg < 4; ++leg)
    {
        bias_acc[leg] = (plus.foot_pos_world[leg] -
            2.0 * moving.foot_pos_world[leg] + minus.foot_pos_world[leg]) / (dt * dt);
        fd_error = std::max(fd_error, (bias_acc[leg] -
            moving.foot_jac_dot_world[leg] * moving.qvel).norm());
    }
    passed &= Check(fd_error < 2.0e-5, "Jdot differs from position curvature");
    passed &= Check(bias_acc[1].norm() > 1.0, "moving fixture has no useful bias");
    for (bool aerial : {false, true})
    {
        go2_control::IdWbcInput physical;
        physical.dynamics = moving;
        physical.contact = aerial ? std::array<bool, 4>{false, false, false, false}
                                  : std::array<bool, 4>{true, false, false, true};
        for (int leg = 0; leg < 4; ++leg)
            physical.swing_acc_world[leg] = Eigen::Vector3d(0.7, -0.2, 1.3);
        auto equivalent = physical;
        for (int leg = 0; leg < 4; ++leg)
            if (!physical.contact[leg])
            {
                equivalent.dynamics.foot_jac_dot_world[leg].setZero();
                equivalent.swing_acc_world[leg] -= bias_acc[leg];
            }
        go2_control::IdWbcParams params;
        params.w_swing_x = 37.0;
        go2_control::IdWbcOutput actual, oracle;
        const bool actual_ok = go2_control::SolveInverseDynamicsWbc(params, physical, actual);
        const bool oracle_ok = go2_control::SolveInverseDynamicsWbc(params, equivalent, oracle);
        passed &= Check(actual_ok && oracle_ok, "moving QP rejected");
        const double qdd_delta = (actual.qdd - oracle.qdd).norm();
        passed &= Check(qdd_delta < 2.0e-3, "physical and compensated swing QPs disagree");
        double physical_cost = 0.0;
        for (int leg = 0; leg < 4; ++leg)
            if (!physical.contact[leg])
            {
                const Eigen::Vector3d error = moving.foot_jac_world[leg] * actual.qdd +
                    bias_acc[leg] - physical.swing_acc_world[leg];
                physical_cost += 37.0 * error.x() * error.x() +
                    params.w_swing * (error.y() * error.y() + error.z() * error.z());
                passed &= Check(actual.force.segment<3>(3 * leg).norm() < 0.2,
                                "swing force exceeds existing QP allowance");
            }
        const double cost_delta = std::abs(physical_cost - actual.cost_terms.swing);
        passed &= Check(cost_delta < 2.0e-3, "swing diagnostic omits physical bias");
        passed &= Check(actual.rne_residual < 1.0e-3, "moving RNE residual");
        std::cout << "swing_bias aerial=" << aerial << " fd_error=" << fd_error
                  << " qdd_delta=" << qdd_delta << " cost_delta=" << cost_delta
                  << " rne=" << actual.rne_residual << "\n";
    }
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
