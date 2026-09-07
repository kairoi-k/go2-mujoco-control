#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "id_wbc_certificate.h"

namespace
{

using go2_control::IdWbcInput;
using go2_control::IdWbcOutput;
using go2_control::IdWbcParams;
using go2_control::RigidBodyDynamics;

bool Check(bool ok, const char *message)
{
    if (!ok)
        std::cerr << message << "\n";
    return ok;
}

RigidBodyDynamics MakeDynamics()
{
    RigidBodyDynamics dyn;
    dyn.valid = true;
    dyn.mass_kg = 10.0;
    dyn.mass_matrix.setIdentity();
    dyn.mass_matrix.topLeftCorner<3, 3>() *= dyn.mass_kg;
    dyn.bias.setZero();
    dyn.com_world.setZero();
    dyn.inertia_com_world.setIdentity();
    dyn.qvel.setZero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        dyn.foot_pos_world[leg] = Eigen::Vector3d::Zero();
        dyn.foot_jac_world[leg].setZero();
        dyn.foot_jac_dot_world[leg].setZero();
        dyn.foot_jac_world[leg].block<3, 3>(0, 0).setIdentity();
    }
    return dyn;
}

IdWbcInput StaticInput()
{
    IdWbcInput input;
    input.dynamics = MakeDynamics();
    input.contact.fill(true);
    input.measured_contact.fill(true);
    input.planned_contact.fill(true);
    input.measured_contact_valid = true;
    input.planned_contact_valid = true;
    input.fused_contact = input.contact;
    input.fused_contact_valid = true;
    input.contact_normal.fill(Eigen::Vector3d::Zero());
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    return input;
}

Eigen::Matrix<double, go2_control::kGo2Nv, 1> ZeroQdd()
{
    return Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
}

Eigen::Matrix<double, go2_control::kGo2Nv, 1> AerialQdd()
{
    auto qdd = ZeroQdd();
    qdd(2) = -9.81;
    return qdd;
}

Eigen::Matrix<double, 12, 1> StaticForce()
{
    Eigen::Matrix<double, 12, 1> force =
        Eigen::Matrix<double, 12, 1>::Zero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        force.segment<3>(3 * static_cast<int>(leg)) =
            Eigen::Vector3d(0.0, 0.0, 24.525);
    return force;
}

Eigen::Matrix<double, go2::kJointCount, 1> ZeroTau()
{
    return Eigen::Matrix<double, go2::kJointCount, 1>::Zero();
}

bool HasBit(const go2_control::IdWbcPhysicalCertificate &certificate,
            go2_control::IdWbcPhysicalCertificateFailure failure)
{
    return (certificate.failure_bitmask &
            go2_control::IdWbcPhysicalCertificateFailureBit(failure)) != 0u;
}

bool RunFixtures()
{
    using Failure = go2_control::IdWbcPhysicalCertificateFailure;
    bool passed = true;
    const IdWbcParams params = {};
    const auto qdd = ZeroQdd();
    const auto tau = ZeroTau();

    auto static_input = StaticInput();
    auto static_force = StaticForce();
    static_input.dynamics.bias[2] = 4.0 * 24.525;
    auto static_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, static_input, qdd, static_force, tau);
    passed &= Check(static_certificate.checked &&
                        static_certificate.input_valid &&
                        static_certificate.valid &&
                        static_certificate.feasible,
                    "feasible static certificate");
    passed &= Check(static_certificate.max_dynamics_force_residual_N < 1.0e-12 &&
                        static_certificate.max_dynamics_moment_residual_Nm < 1.0e-12 &&
                        static_certificate.max_joint_dynamics_residual_Nm < 1.0e-12,
                    "static residual components");
    passed &= Check(static_certificate.flat_normal_default_used &&
                        static_certificate.normal_defaulted_by_leg[0] &&
                        static_certificate.normal_force_N[0] > 24.5,
                    "flat normal fallback metadata");

    IdWbcOutput proposed_output;
    proposed_output.qdd = qdd;
    proposed_output.force = static_force;
    proposed_output.tau = tau;
    const auto output_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, static_input, proposed_output);
    passed &= Check(output_certificate.feasible,
                    "IdWbcOutput overload");

    auto residual_input = static_input;
    residual_input.dynamics.bias[0] = 0.2;
    residual_input.dynamics.bias[3] = 0.3;
    const auto residual_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, residual_input, qdd, static_force, tau);
    passed &= Check(
        !residual_certificate.feasible &&
            HasBit(residual_certificate, Failure::kDynamicsResidual) &&
            residual_certificate.max_dynamics_force_residual_N >= 0.19 &&
            residual_certificate.max_dynamics_moment_residual_Nm >= 0.29,
        "force and moment residual units");

    auto aerial_input = static_input;
    aerial_input.contact.fill(false);
    aerial_input.measured_contact.fill(false);
    aerial_input.planned_contact.fill(false);
    aerial_input.fused_contact.fill(false);
    aerial_input.dynamics.bias.setZero();
    aerial_input.dynamics.bias[2] = 10.0 * 9.81;
    Eigen::Matrix<double, 12, 1> aerial_force =
        Eigen::Matrix<double, 12, 1>::Zero();
    auto aerial_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, aerial_input, AerialQdd(), aerial_force, tau);
    passed &= Check(aerial_certificate.feasible &&
                        aerial_certificate.max_swing_force_violation_N == 0.0 &&
                        !aerial_certificate.normal_defaulted_by_leg[0],
                    "feasible aerial certificate");

    auto friction_input = static_input;
    auto friction_force = static_force;
    friction_force.segment<3>(0) = Eigen::Vector3d(8.1, 0.0, 10.0);
    friction_input.dynamics.bias[0] = 8.1;
    friction_input.dynamics.bias[2] = 3.0 * 24.525 + 10.0;
    const auto friction_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, friction_input, qdd, friction_force, tau);
    passed &= Check(!friction_certificate.feasible &&
                        HasBit(friction_certificate, Failure::kFriction) &&
                        friction_certificate.max_friction_violation_N > 0.0,
                    "radial friction violation");

    auto normal_input = static_input;
    auto normal_force = static_force;
    normal_force(2) = 0.5;
    normal_input.dynamics.bias[2] = 3.0 * 24.525 + 0.5;
    const auto normal_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, normal_input, qdd, normal_force, tau);
    passed &= Check(!normal_certificate.feasible &&
                        HasBit(normal_certificate, Failure::kNormalForce) &&
                        normal_certificate.max_normal_violation_N >= 0.49,
                    "unilateral normal violation");

    auto lower_floor_input = static_input;
    auto lower_floor_force = static_force;
    lower_floor_force(2) = 0.75;
    lower_floor_input.dynamics.bias[2] = 3.0 * 24.525 + 0.75;
    IdWbcParams lower_floor_params = {};
    lower_floor_params.min_normal_n_by_leg[0] = 0.5;
    const auto lower_floor_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            lower_floor_params, lower_floor_input, qdd, lower_floor_force, tau);
    passed &= Check(
        !lower_floor_certificate.feasible &&
            HasBit(lower_floor_certificate, Failure::kNormalForce) &&
            lower_floor_certificate.normal_violation_N[0] >= 0.24,
        "lower per-leg floor must not relax global floor");

    auto per_leg_input = static_input;
    auto per_leg_force = static_force;
    per_leg_force(2) = 5.0;
    per_leg_input.dynamics.bias[2] = 3.0 * 24.525 + 5.0;
    IdWbcParams per_leg_params = {};
    per_leg_params.min_normal_n_by_leg[0] = 20.0;
    const auto per_leg_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            per_leg_params, per_leg_input, qdd, per_leg_force, tau);
    passed &= Check(
        !per_leg_certificate.feasible &&
            HasBit(per_leg_certificate, Failure::kNormalForce) &&
            per_leg_certificate.normal_violation_N[0] >= 14.9,
        "high per-leg floor must be enforced");

    auto swing_input = aerial_input;
    auto swing_force = aerial_force;
    swing_force(0) = 0.06;
    swing_input.dynamics.bias[0] = 0.06;
    const auto swing_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, swing_input, AerialQdd(), swing_force, tau);
    passed &= Check(!swing_certificate.feasible &&
                        HasBit(swing_certificate, Failure::kSwingForce) &&
                        swing_certificate.max_swing_force_violation_N >= 0.009,
                    "swing force allowance violation");

    auto torque_input = aerial_input;
    auto torque = tau;
    torque(0) = 35.01;
    torque_input.dynamics.bias[6] = 35.01;
    const auto torque_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, torque_input, AerialQdd(), aerial_force, torque);
    passed &= Check(!torque_certificate.feasible &&
                        HasBit(torque_certificate, Failure::kTorque) &&
                        torque_certificate.max_tau_violation_Nm > 0.0,
                    "torque limit violation");

    auto hard_input = static_input;
    hard_input.dynamics.bias[2] = 4.0 * 24.525;
    hard_input.have_stance_acc = true;
    hard_input.stance_acc_world[0] = Eigen::Vector3d(0.1, 0.0, 0.0);
    IdWbcParams hard_params = {};
    hard_params.hard_stance_no_slip = true;
    go2_control::IdWbcPhysicalCertificateThresholds hard_thresholds = {};
    hard_thresholds.max_hard_stance_acc_residual_mps2 = 0.01;
    const auto hard_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        hard_params, hard_input, qdd, static_force, tau, hard_thresholds);
    passed &= Check(!hard_certificate.feasible &&
                        HasBit(hard_certificate,
                               Failure::kHardStanceAcceleration) &&
                        hard_certificate.hard_stance_acc_residual_mps2 >= 0.099,
                    "hard stance acceleration violation");

    auto overflow_input = static_input;
    overflow_input.dynamics.foot_jac_dot_world[0](0, 0) =
        std::numeric_limits<double>::max();
    overflow_input.dynamics.qvel(0) = 2.0;
    IdWbcParams overflow_params = {};
    overflow_params.hard_stance_no_slip = true;
    const auto overflow_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            overflow_params, overflow_input, qdd, static_force, tau);
    passed &= Check(!overflow_certificate.valid &&
                        !overflow_certificate.feasible &&
                        HasBit(overflow_certificate, Failure::kInvalidDynamics),
                    "finite overflow in hard stance must fail closed");

    auto unused_acc_input = static_input;
    unused_acc_input.stance_acc_world[1] =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const auto unused_acc_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, unused_acc_input, qdd, static_force, tau);
    passed &= Check(unused_acc_certificate.feasible,
                    "unused stance acceleration must be ignored");

    auto huge_normal_input = static_input;
    huge_normal_input.contact_normal_valid.fill(true);
    huge_normal_input.contact_normal.fill(Eigen::Vector3d(
        0.0, 0.0, std::numeric_limits<double>::max()));
    const auto huge_normal_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, huge_normal_input, qdd, static_force, tau);
    passed &= Check(huge_normal_certificate.input_valid &&
                        huge_normal_certificate.valid &&
                        huge_normal_certificate.feasible,
                    "large finite normal must normalize stably");

    auto conflict_input = aerial_input;
    conflict_input.contact_normal_valid[0] = true;
    conflict_input.contact_normal[0] = Eigen::Vector3d::Zero();
    const auto conflict_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, conflict_input, AerialQdd(), aerial_force, tau);
    passed &= Check(conflict_certificate.checked &&
                        !conflict_certificate.input_valid &&
                        !conflict_certificate.valid &&
                        !conflict_certificate.feasible &&
                        HasBit(conflict_certificate, Failure::kInputConflict),
                    "declared zero normal is input conflict");

    auto nonfinite_qdd = qdd;
    nonfinite_qdd(0) = std::numeric_limits<double>::quiet_NaN();
    const auto nonfinite_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, aerial_input, nonfinite_qdd, aerial_force, tau);
    passed &= Check(!nonfinite_certificate.input_valid &&
                        !nonfinite_certificate.feasible &&
                        HasBit(nonfinite_certificate, Failure::kNonfiniteProposal),
                    "nonfinite proposal fail-closed");

    auto tilted_input = static_input;
    tilted_input.contact_normal_valid.fill(true);
    tilted_input.contact_normal.fill(Eigen::Vector3d(0.0, 0.0, -1.0));
    tilted_input.contact_normal[0] = Eigen::Vector3d(-0.1, 0.0, 0.995);
    const auto tilted_certificate = go2_control::VerifyIdWbcPhysicalCertificate(
        params, tilted_input, qdd, static_force, tau);
    passed &= Check(tilted_certificate.input_valid &&
                        !tilted_certificate.normal_defaulted_by_leg[0] &&
                        tilted_certificate.contact_normal_world[1].z() > 0.0,
                    "upward terrain normal convention");

    auto nan_normal_input = static_input;
    nan_normal_input.contact_normal_valid[0] = true;
    nan_normal_input.contact_normal[0] = Eigen::Vector3d(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0);
    const auto nan_normal_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, nan_normal_input, qdd, static_force, tau);
    passed &= Check(!nan_normal_certificate.input_valid &&
                        !nan_normal_certificate.valid &&
                        HasBit(nan_normal_certificate, Failure::kInputConflict),
                    "declared NaN normal is input conflict");

    auto invalid_params = params;
    invalid_params.min_normal_n = 10.0;
    invalid_params.max_normal_n = 5.0;
    const auto invalid_configuration_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            invalid_params, static_input, qdd, static_force, tau);
    passed &= Check(
        !invalid_configuration_certificate.input_valid &&
            !invalid_configuration_certificate.feasible &&
            HasBit(invalid_configuration_certificate,
                   Failure::kInvalidConfiguration),
        "invalid limits fail closed");
    return passed;
}

void RunBenchmark()
{
    auto input = StaticInput();
    input.dynamics.bias[2] = 4.0 * 24.525;
    auto qdd = ZeroQdd();
    const auto force = StaticForce();
    const auto tau = ZeroTau();
    constexpr int kSamples = 10000;
    std::vector<double> samples;
    samples.reserve(kSamples);
    volatile std::uint32_t sink = 0u;
    for (int i = 0; i < kSamples; ++i)
    {
        // Vary the proposal to prevent a loop-invariant constant-case timing.
        qdd[0] = 0.001 * static_cast<double>(i % 127);
        const auto start = std::chrono::steady_clock::now();
        const auto certificate = go2_control::VerifyIdWbcPhysicalCertificate(
            {}, input, qdd, force, tau);
        const auto stop = std::chrono::steady_clock::now();
        sink ^= certificate.failure_bitmask +
            static_cast<std::uint32_t>(certificate.checked);
        samples.push_back(std::chrono::duration<double, std::micro>(
                              stop - start).count());
    }
    std::sort(samples.begin(), samples.end());
    std::cout << "id_wbc_certificate_benchmark_us p50="
              << samples[kSamples / 2] << " p95="
              << samples[(kSamples * 95) / 100] << " max="
              << samples.back() << " sink=" << sink << "\n";
}

}  // namespace

int main(int argc, char **argv)
{
    if (!RunFixtures())
        return 1;
    if (argc > 1 && std::string(argv[1]) == "--benchmark")
        RunBenchmark();
    std::cout << "id-wbc physical certificate checks passed.\n";
    return 0;
}
