#pragma once

// Independent physical checks for one proposed ID-WBC (qdd, force, tau)
// tuple.  This is a shadow diagnostic: it does not inspect QP rows or solver
// status and it does not certify a final PD torque command.

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

#include "inverse_dynamics_wbc.h"

namespace go2_control
{

// Keep this equal to the current swing-force allowance in the ID-WBC QP.
inline constexpr double kIdWbcSwingForceAllowanceN = 0.05;

enum class IdWbcPhysicalCertificateFailure : std::uint32_t
{
    kNone = 0u,
    kInvalidConfiguration = 1u << 0,
    kInvalidDynamics = 1u << 1,
    kInputConflict = 1u << 2,
    kNonfiniteProposal = 1u << 3,
    kDynamicsResidual = 1u << 4,
    kNormalForce = 1u << 5,
    kFriction = 1u << 6,
    kSwingForce = 1u << 7,
    kTorque = 1u << 8,
    kHardStanceAcceleration = 1u << 9,
};

inline constexpr std::uint32_t IdWbcPhysicalCertificateFailureBit(
    IdWbcPhysicalCertificateFailure failure)
{
    return static_cast<std::uint32_t>(failure);
}

// These are diagnostic acceptance tolerances for this certificate only.
// They are deliberately separate from DenseQp tolerances and do not alter
// the QP or any existing SolveInverseDynamicsWbc acceptance decision.
struct IdWbcPhysicalCertificateThresholds
{
    double max_dynamics_force_residual_N = 1.0e-3;
    double max_dynamics_moment_residual_Nm = 1.0e-3;
    double max_joint_dynamics_residual_Nm = 1.0e-3;
    double max_friction_violation_N = 1.0e-6;
    double max_normal_violation_N = 1.0e-6;
    double max_swing_force_violation_N = 1.0e-6;
    double max_tau_violation_Nm = 1.0e-6;
    double max_hard_stance_acc_residual_mps2 = 1.0e-6;
};

struct IdWbcPhysicalCertificate
{
    // checked means this function ran; input_valid means every required
    // input was finite and internally valid; valid means all computed values
    // are finite; feasible additionally requires every configured threshold.
    bool checked = false;
    bool input_valid = false;
    bool valid = false;
    bool feasible = false;
    std::uint32_t failure_bitmask = 0u;

    bool dynamics_checked = false;
    bool normal_checked = false;
    bool friction_checked = false;
    bool swing_force_checked = false;
    bool torque_checked = false;
    bool hard_stance_acc_checked = false;

    Eigen::Matrix<double, kGo2Nv, 1> dynamics_residual =
        Eigen::Matrix<double, kGo2Nv, 1>::Zero();
    double dynamics_residual_norm = 0.0;
    double max_dynamics_force_residual_N = 0.0;
    double max_dynamics_moment_residual_Nm = 0.0;
    double max_joint_dynamics_residual_Nm = 0.0;

    std::array<Eigen::Vector3d, go2::kLegCount> contact_normal_world{};
    std::array<bool, go2::kLegCount> normal_defaulted_by_leg{};
    bool flat_normal_default_used = false;
    std::array<double, go2::kLegCount> normal_force_N{};
    std::array<double, go2::kLegCount> normal_violation_N{};
    double max_normal_violation_N = 0.0;

    std::array<double, go2::kLegCount> tangential_force_N{};
    std::array<double, go2::kLegCount> friction_limit_N{};
    std::array<double, go2::kLegCount> friction_violation_N{};
    double max_friction_violation_N = 0.0;

    std::array<double, go2::kLegCount> swing_force_max_abs_axis_N{};
    std::array<double, go2::kLegCount> swing_force_violation_N{};
    double max_swing_force_violation_N = 0.0;

    std::array<double, go2::kJointCount> tau_violation_Nm{};
    double max_tau_violation_Nm = 0.0;

    std::array<double, go2::kLegCount> hard_stance_acc_residual_mps2_by_leg{};
    double hard_stance_acc_residual_mps2 = 0.0;
};

namespace id_wbc_certificate_detail
{

inline bool FiniteNonnegative(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

inline bool ValidThresholds(const IdWbcPhysicalCertificateThresholds &thresholds)
{
    return FiniteNonnegative(thresholds.max_dynamics_force_residual_N) &&
        FiniteNonnegative(thresholds.max_dynamics_moment_residual_Nm) &&
        FiniteNonnegative(thresholds.max_joint_dynamics_residual_Nm) &&
        FiniteNonnegative(thresholds.max_friction_violation_N) &&
        FiniteNonnegative(thresholds.max_normal_violation_N) &&
        FiniteNonnegative(thresholds.max_swing_force_violation_N) &&
        FiniteNonnegative(thresholds.max_tau_violation_Nm) &&
        FiniteNonnegative(thresholds.max_hard_stance_acc_residual_mps2);
}

inline bool ValidPhysicalParams(const IdWbcParams &params)
{
    if (!FiniteNonnegative(params.friction_mu) ||
        !FiniteNonnegative(params.min_normal_n) ||
        !FiniteNonnegative(params.max_normal_n) ||
        !FiniteNonnegative(params.tau_limit_nm) ||
        params.min_normal_n > params.max_normal_n)
        return false;
    for (const double floor : params.min_normal_n_by_leg)
        if (!std::isfinite(floor) || floor < 0.0)
            return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const double floor = std::max(
            params.min_normal_n,
            params.min_normal_n_by_leg[leg] > 0.0
                ? params.min_normal_n_by_leg[leg] : 0.0);
        if (floor > params.max_normal_n)
            return false;
    }
    return true;
}

inline bool FiniteDynamics(const RigidBodyDynamics &dyn)
{
    if (!dyn.valid || !std::isfinite(dyn.mass_kg) || dyn.mass_kg <= 0.0 ||
        !dyn.com_world.allFinite() || !dyn.inertia_com_world.allFinite() ||
        !dyn.mass_matrix.allFinite() || !dyn.bias.allFinite() ||
        !dyn.qvel.allFinite())
        return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (!dyn.foot_pos_world[leg].allFinite() ||
            !dyn.foot_jac_world[leg].allFinite() ||
            !dyn.foot_jac_dot_world[leg].allFinite())
            return false;
    return true;
}

inline Eigen::Vector3d UpwardNormal(const Eigen::Vector3d &raw)
{
    const double magnitude = raw.stableNorm();
    Eigen::Vector3d normal = raw / magnitude;
    // Preserve the existing terrain convention: orient a supplied plane
    // normal toward world +Z before constructing the physical cone.
    if (normal.z() < 0.0)
        normal = -normal;
    return normal;
}

inline bool Exceeds(double value, double threshold)
{
    return !std::isfinite(value) || value > threshold;
}

}  // namespace id_wbc_certificate_detail

inline IdWbcPhysicalCertificate VerifyIdWbcPhysicalCertificate(
    const IdWbcParams &params,
    const IdWbcInput &input,
    const Eigen::Matrix<double, kGo2Nv, 1> &proposed_qdd,
    const Eigen::Matrix<double, 12, 1> &proposed_force,
    const Eigen::Matrix<double, go2::kJointCount, 1> &proposed_tau,
    const IdWbcPhysicalCertificateThresholds &thresholds = {})
{
    using Failure = IdWbcPhysicalCertificateFailure;
    const double inf = std::numeric_limits<double>::infinity();
    IdWbcPhysicalCertificate result;
    result.checked = true;
    result.normal_defaulted_by_leg.fill(false);
    result.dynamics_residual.setConstant(inf);
    result.dynamics_residual_norm = inf;
    result.max_dynamics_force_residual_N = inf;
    result.max_dynamics_moment_residual_Nm = inf;
    result.max_joint_dynamics_residual_Nm = inf;
    result.normal_force_N.fill(inf);
    result.normal_violation_N.fill(inf);
    result.max_normal_violation_N = inf;
    result.tangential_force_N.fill(inf);
    result.friction_limit_N.fill(inf);
    result.friction_violation_N.fill(inf);
    result.max_friction_violation_N = inf;
    result.swing_force_max_abs_axis_N.fill(inf);
    result.swing_force_violation_N.fill(inf);
    result.max_swing_force_violation_N = inf;
    result.tau_violation_Nm.fill(inf);
    result.max_tau_violation_Nm = inf;
    result.hard_stance_acc_residual_mps2_by_leg.fill(inf);
    result.hard_stance_acc_residual_mps2 = inf;

    std::uint32_t failures = 0u;
    bool input_valid = true;
    if (!id_wbc_certificate_detail::ValidPhysicalParams(params) ||
        !id_wbc_certificate_detail::ValidThresholds(thresholds))
    {
        failures |= IdWbcPhysicalCertificateFailureBit(
            Failure::kInvalidConfiguration);
        input_valid = false;
    }
    if (!id_wbc_certificate_detail::FiniteDynamics(input.dynamics))
    {
        failures |= IdWbcPhysicalCertificateFailureBit(Failure::kInvalidDynamics);
        input_valid = false;
    }
    if (!ValidateIdWbcTerrainReference(input))
    {
        failures |= IdWbcPhysicalCertificateFailureBit(Failure::kInputConflict);
        input_valid = false;
    }
    if (!proposed_qdd.allFinite() || !proposed_force.allFinite() ||
        !proposed_tau.allFinite())
    {
        failures |= IdWbcPhysicalCertificateFailureBit(
            Failure::kNonfiniteProposal);
        input_valid = false;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const bool acceleration_finite =
            input.contact[leg]
                ? (!input.have_stance_acc ||
                   input.stance_acc_world[leg].allFinite())
                : input.swing_acc_world[leg].allFinite();
        if (!acceleration_finite)
        {
            failures |= IdWbcPhysicalCertificateFailureBit(
                Failure::kInputConflict);
            input_valid = false;
        }
        if (input.contact_normal_valid[leg])
        {
            const double magnitude = input.contact_normal[leg].stableNorm();
            const Eigen::Vector3d normal =
                id_wbc_certificate_detail::UpwardNormal(
                    input.contact_normal[leg]);
            if (!input.contact_normal[leg].allFinite() ||
                !std::isfinite(magnitude) || magnitude <= 1.0e-9 ||
                !normal.allFinite() || !std::isfinite(normal.stableNorm()))
            {
                // A declared but malformed normal must not silently become a
                // flat default, since that changes the physical constraint.
                failures |= IdWbcPhysicalCertificateFailureBit(
                    Failure::kInputConflict);
                input_valid = false;
            }
            else
            {
                result.normal_defaulted_by_leg[leg] = false;
                result.contact_normal_world[leg] =
                    id_wbc_certificate_detail::UpwardNormal(
                        input.contact_normal[leg]);
            }
        }
    }
    if (!input_valid)
    {
        result.failure_bitmask = failures;
        return result;
    }
    result.input_valid = true;
    result.max_normal_violation_N = 0.0;
    result.max_friction_violation_N = 0.0;
    result.max_swing_force_violation_N = 0.0;
    result.max_tau_violation_Nm = 0.0;
    result.hard_stance_acc_residual_mps2 = 0.0;
    result.normal_force_N.fill(0.0);
    result.normal_violation_N.fill(0.0);
    result.tangential_force_N.fill(0.0);
    result.friction_limit_N.fill(0.0);
    result.friction_violation_N.fill(0.0);
    result.swing_force_max_abs_axis_N.fill(0.0);
    result.swing_force_violation_N.fill(0.0);
    result.tau_violation_Nm.fill(0.0);
    result.hard_stance_acc_residual_mps2_by_leg.fill(0.0);

    const auto &dyn = input.dynamics;
    result.dynamics_checked = true;
    result.dynamics_residual = dyn.mass_matrix * proposed_qdd + dyn.bias;
    result.dynamics_residual.tail<go2::kJointCount>() -= proposed_tau;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        result.dynamics_residual -= dyn.foot_jac_world[leg].transpose() *
            proposed_force.segment<3>(3 * static_cast<int>(leg));
    result.dynamics_residual_norm = result.dynamics_residual.norm();
    result.max_dynamics_force_residual_N =
        result.dynamics_residual.head<3>().cwiseAbs().maxCoeff();
    result.max_dynamics_moment_residual_Nm =
        result.dynamics_residual.segment<3>(3).cwiseAbs().maxCoeff();
    result.max_joint_dynamics_residual_Nm =
        result.dynamics_residual.tail<go2::kJointCount>().cwiseAbs().maxCoeff();

    bool any_contact = false;
    bool any_swing = false;
    bool computations_finite = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d force =
            proposed_force.segment<3>(3 * static_cast<int>(leg));
        if (input.contact[leg])
        {
            any_contact = true;
            Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
            if (input.contact_normal_valid[leg])
                normal = result.contact_normal_world[leg];
            else
            {
                result.flat_normal_default_used = true;
                result.normal_defaulted_by_leg[leg] = true;
                result.contact_normal_world[leg] = normal;
            }
            const double fn = normal.dot(force);
            const Eigen::Vector3d tangent = force - fn * normal;
            const double floor = std::max(
                params.min_normal_n,
                params.min_normal_n_by_leg[leg] > 0.0
                    ? params.min_normal_n_by_leg[leg] : 0.0);
            const double normal_violation = std::max(
                0.0, std::max(floor - fn, fn - params.max_normal_n));
            const double friction_limit = params.friction_mu * fn;
            const double tangent_norm = tangent.stableNorm();
            const double friction_violation = std::max(
                0.0, tangent_norm - friction_limit);
            if (!normal.allFinite() || !std::isfinite(normal.stableNorm()) ||
                !std::isfinite(fn) || !tangent.allFinite() ||
                !std::isfinite(tangent_norm) ||
                !std::isfinite(friction_limit) ||
                !std::isfinite(normal_violation) ||
                !std::isfinite(friction_violation))
                computations_finite = false;
            result.normal_force_N[leg] = fn;
            result.normal_violation_N[leg] = normal_violation;
            result.tangential_force_N[leg] = tangent_norm;
            result.friction_limit_N[leg] = friction_limit;
            result.friction_violation_N[leg] = friction_violation;
            result.max_normal_violation_N = std::max(
                result.max_normal_violation_N, normal_violation);
            result.max_friction_violation_N = std::max(
                result.max_friction_violation_N, friction_violation);
        }
        else
        {
            any_swing = true;
            const double max_axis = force.cwiseAbs().maxCoeff();
            result.swing_force_max_abs_axis_N[leg] = max_axis;
            result.swing_force_violation_N[leg] = std::max(
                0.0, max_axis - kIdWbcSwingForceAllowanceN);
            if (!std::isfinite(max_axis) ||
                !std::isfinite(result.swing_force_violation_N[leg]))
                computations_finite = false;
            result.max_swing_force_violation_N = std::max(
                result.max_swing_force_violation_N,
                result.swing_force_violation_N[leg]);
        }
    }
    if (!any_contact)
    {
        result.max_normal_violation_N = 0.0;
        result.max_friction_violation_N = 0.0;
    }
    if (!any_swing)
        result.max_swing_force_violation_N = 0.0;
    result.normal_checked = any_contact;
    result.friction_checked = any_contact;
    result.swing_force_checked = any_swing;

    result.torque_checked = true;
    for (std::size_t joint = 0; joint < go2::kJointCount; ++joint)
    {
        result.tau_violation_Nm[joint] = std::max(
            0.0, std::abs(proposed_tau[static_cast<int>(joint)]) -
                params.tau_limit_nm);
        if (!std::isfinite(result.tau_violation_Nm[joint]))
            computations_finite = false;
        result.max_tau_violation_Nm = std::max(
            result.max_tau_violation_Nm, result.tau_violation_Nm[joint]);
    }

    if (params.hard_stance_no_slip)
    {
        result.hard_stance_acc_checked = true;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!input.contact[leg])
            {
                result.hard_stance_acc_residual_mps2_by_leg[leg] = 0.0;
                continue;
            }
            const Eigen::Vector3d target = input.have_stance_acc
                ? input.stance_acc_world[leg] : Eigen::Vector3d::Zero();
            result.hard_stance_acc_residual_mps2_by_leg[leg] =
                (dyn.foot_jac_world[leg] * proposed_qdd +
                 dyn.foot_jac_dot_world[leg] * dyn.qvel - target).norm();
            if (!std::isfinite(
                    result.hard_stance_acc_residual_mps2_by_leg[leg]))
                computations_finite = false;
            result.hard_stance_acc_residual_mps2 = std::max(
                result.hard_stance_acc_residual_mps2,
                result.hard_stance_acc_residual_mps2_by_leg[leg]);
        }
    }
    else
    {
        result.hard_stance_acc_residual_mps2 = 0.0;
    }

    result.valid = computations_finite &&
        result.dynamics_residual.allFinite() &&
        std::isfinite(result.dynamics_residual_norm) &&
        std::isfinite(result.max_dynamics_force_residual_N) &&
        std::isfinite(result.max_dynamics_moment_residual_Nm) &&
        std::isfinite(result.max_joint_dynamics_residual_Nm) &&
        std::isfinite(result.max_normal_violation_N) &&
        std::isfinite(result.max_friction_violation_N) &&
        std::isfinite(result.max_swing_force_violation_N) &&
        std::isfinite(result.max_tau_violation_Nm) &&
        std::isfinite(result.hard_stance_acc_residual_mps2);
    if (!result.valid)
    {
        result.failure_bitmask = failures | IdWbcPhysicalCertificateFailureBit(
            Failure::kInvalidDynamics);
        return result;
    }

    if (id_wbc_certificate_detail::Exceeds(
            result.max_dynamics_force_residual_N,
            thresholds.max_dynamics_force_residual_N) ||
        id_wbc_certificate_detail::Exceeds(
            result.max_dynamics_moment_residual_Nm,
            thresholds.max_dynamics_moment_residual_Nm) ||
        id_wbc_certificate_detail::Exceeds(
            result.max_joint_dynamics_residual_Nm,
            thresholds.max_joint_dynamics_residual_Nm))
        failures |= IdWbcPhysicalCertificateFailureBit(
            Failure::kDynamicsResidual);
    if (id_wbc_certificate_detail::Exceeds(
            result.max_normal_violation_N, thresholds.max_normal_violation_N))
        failures |= IdWbcPhysicalCertificateFailureBit(Failure::kNormalForce);
    if (id_wbc_certificate_detail::Exceeds(
            result.max_friction_violation_N, thresholds.max_friction_violation_N))
        failures |= IdWbcPhysicalCertificateFailureBit(Failure::kFriction);
    if (id_wbc_certificate_detail::Exceeds(
            result.max_swing_force_violation_N,
            thresholds.max_swing_force_violation_N))
        failures |= IdWbcPhysicalCertificateFailureBit(Failure::kSwingForce);
    if (id_wbc_certificate_detail::Exceeds(
            result.max_tau_violation_Nm, thresholds.max_tau_violation_Nm))
        failures |= IdWbcPhysicalCertificateFailureBit(Failure::kTorque);
    if (result.hard_stance_acc_checked &&
        id_wbc_certificate_detail::Exceeds(
            result.hard_stance_acc_residual_mps2,
            thresholds.max_hard_stance_acc_residual_mps2))
        failures |= IdWbcPhysicalCertificateFailureBit(
            Failure::kHardStanceAcceleration);

    result.failure_bitmask = failures;
    result.feasible = result.valid && result.failure_bitmask == 0u;
    return result;
}

inline IdWbcPhysicalCertificate VerifyIdWbcPhysicalCertificate(
    const IdWbcParams &params,
    const IdWbcInput &input,
    const IdWbcOutput &proposed,
    const IdWbcPhysicalCertificateThresholds &thresholds = {})
{
    return VerifyIdWbcPhysicalCertificate(
        params, input, proposed.qdd, proposed.force, proposed.tau, thresholds);
}

}  // namespace go2_control
