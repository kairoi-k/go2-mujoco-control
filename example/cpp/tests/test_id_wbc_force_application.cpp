#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <Eigen/Dense>
#include "id_wbc_certificate.h"
namespace
{
using go2_control::IdWbcFootJacobian;
using go2_control::IdWbcFootJacobianArray;
using go2_control::IdWbcInput;
using GeneralizedForce = Eigen::Matrix<double, go2_control::kGo2Nv, 1>;
bool Check(bool ok, const char *message)
{
    if (!ok)
        std::cerr << message << "\n";
    return ok;
}
Eigen::Matrix3d Skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d result;
    result << 0.0, -v.z(), v.y(),
        v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return result;
}
IdWbcFootJacobian CenterJacobian()
{
    return IdWbcFootJacobian::Zero();
}
IdWbcFootJacobian PointJacobian()
{
    const Eigen::Vector3d radius(0.0, 0.0, 0.022);
    IdWbcFootJacobian rotational = IdWbcFootJacobian::Zero();
    // A unit angular velocity column about world Y gives the standard
    // Jp = Jc - skew(P-C) Jr virtual-work shift.
    rotational(1, 6) = 1.0;
    return -Skew(radius) * rotational;
}
IdWbcInput MakeInput()
{
    IdWbcInput input;
    input.dynamics.valid = true;
    input.dynamics.mass_kg = 10.0;
    input.dynamics.mass_matrix.setIdentity();
    input.dynamics.bias.setZero();
    input.dynamics.com_world.setZero();
    input.dynamics.inertia_com_world.setIdentity();
    input.dynamics.qvel.setZero();
    input.contact.fill(false);
    input.contact[0] = true;
    input.contact_normal.fill(Eigen::Vector3d::Zero());
    input.contact_normal_valid.fill(false);
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        input.dynamics.foot_pos_world[leg] = Eigen::Vector3d::Zero();
        input.dynamics.foot_jac_world[leg] = CenterJacobian();
        input.dynamics.foot_jac_dot_world[leg].setZero();
        input.force_application_jac_world[leg] = CenterJacobian();
    }
    return input;
}
Eigen::Matrix<double, go2_control::kGo2Nv, 1> ZeroQdd()
{
    return Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
}
Eigen::Matrix<double, 12, 1> TestForce()
{
    Eigen::Matrix<double, 12, 1> force =
        Eigen::Matrix<double, 12, 1>::Zero();
    force.segment<3>(0) = Eigen::Vector3d(7.5, 0.0, 10.0);
    return force;
}
Eigen::Matrix<double, go2::kJointCount, 1> ZeroTau()
{
    return Eigen::Matrix<double, go2::kJointCount, 1>::Zero();
}
bool HasFailure(
    const go2_control::IdWbcPhysicalCertificate &certificate,
    go2_control::IdWbcPhysicalCertificateFailure failure)
{
    return (certificate.failure_bitmask &
            go2_control::IdWbcPhysicalCertificateFailureBit(failure)) != 0u;
}
bool RunTests()
{
    using Failure = go2_control::IdWbcPhysicalCertificateFailure;
    bool passed = true;
    const Eigen::Vector3d radius(0.0, 0.0, 0.022);
    const Eigen::Vector3d tangent_force(100.0, 0.0, 0.0);
    const IdWbcFootJacobian center = CenterJacobian();
    const IdWbcFootJacobian point = PointJacobian();
    IdWbcFootJacobian rotational = IdWbcFootJacobian::Zero();
    rotational(1, 6) = 1.0;
    const GeneralizedForce lhs =
        (point.transpose() - center.transpose()) * tangent_force;
    const GeneralizedForce rhs =
        rotational.transpose() * radius.cross(tangent_force);
    passed &= Check(
        (lhs - rhs).norm() < 1e-12,
        "point/center virtual-work shift identity");
    passed &= Check(
        std::abs(lhs[6] - 2.2) < 1e-12,
        "22 mm x 100 N generalized torque shift");
    passed &= Check(
        radius.cross(Eigen::Vector3d(0.0, 0.0, 100.0)).norm() < 1e-12,
        "radial normal force has zero moment shift");
    auto input = MakeInput();
    input.have_force_application_jacobian = true;
    auto legacy_input = input;
    legacy_input.have_force_application_jacobian = false;
    for (auto &jacobian : input.force_application_jac_world)
        jacobian = point;
    IdWbcFootJacobianArray selected;
    passed &= Check(
        go2_control::SelectIdWbcForceJacobians(input, selected),
        "explicit force Jacobian selection");
    passed &= Check(
        (selected[0] - point).norm() < 1e-12,
        "explicit force Jacobian was not selected");
    passed &= Check(
        go2_control::SelectIdWbcForceJacobians(legacy_input, selected),
        "legacy force Jacobian selection");
    passed &= Check(
        (selected[0] - center).norm() < 1e-12,
        "legacy path did not retain geom-center Jacobian");
    const auto force = TestForce();
    const auto qdd = ZeroQdd();
    const auto tau = ZeroTau();
    input.dynamics.bias = point.transpose() * force.segment<3>(0);
    legacy_input.dynamics.bias = input.dynamics.bias;
    const auto explicit_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            {}, input, qdd, force, tau);
    passed &= Check(
        explicit_certificate.checked && explicit_certificate.input_valid &&
            explicit_certificate.valid && explicit_certificate.feasible &&
            explicit_certificate.force_application_jacobian_used,
        "explicit force Jacobian certificate");
    passed &= Check(
        explicit_certificate.max_joint_dynamics_residual_Nm < 1e-12,
        "explicit force Jacobian dynamics residual");
    const auto legacy_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            {}, legacy_input, qdd, force, tau);
    passed &= Check(
        !legacy_certificate.feasible &&
            HasFailure(legacy_certificate, Failure::kDynamicsResidual) &&
            legacy_certificate.max_joint_dynamics_residual_Nm > 0.16,
        "legacy center Jacobian must expose point-force residual");
    auto invalid_input = input;
    invalid_input.force_application_jac_world[2](0, 0) =
        std::numeric_limits<double>::quiet_NaN();
    const auto invalid_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            {}, invalid_input, qdd, force, tau);
    passed &= Check(
        !invalid_certificate.input_valid && !invalid_certificate.valid &&
            !invalid_certificate.feasible &&
            HasFailure(invalid_certificate, Failure::kInputConflict),
        "invalid explicit force Jacobian fail-closed");
    go2_control::IdWbcOutput output;
    passed &= Check(
        !go2_control::SolveInverseDynamicsWbc(
            {}, invalid_input, output),
        "solver accepted invalid explicit force Jacobian");
    return passed;
}
}  // namespace
int main()
{
    if (!RunTests())
        return 1;
    std::cout << "id-wbc force application Jacobian tests passed\n";
    return 0;
}
