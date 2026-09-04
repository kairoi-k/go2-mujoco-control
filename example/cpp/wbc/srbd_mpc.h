#pragma once

// Receding-horizon single-rigid-body MPC (Di Carlo / Mini Cheetah class).
// Decision variables are contact forces at each knot. State is condensed.
// Friction pyramid and swing-foot zeros are hard. First-knot force and
// implied CoM acceleration are the WBC reference.

#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

#include "dense_qp.h"
#include "go2_forward_kinematics.h"
#include "terrain_control_interface.h"
#include "locomotion_kernel.h"

namespace go2_control
{

constexpr int kSrbdStateSize = 12;
constexpr int kSrbdMaxHorizon = 24;
constexpr int kSrbdForceSize = 12;

struct SrbdMpcParams
{
    int horizon = 10;
    double dt_s = 0.05;
    double gravity_mps2 = 9.81;
    double friction_mu = 0.8;
    double min_normal_n = 2.0;
    double max_normal_n = 180.0;
    double mass_kg = 15.206;
    Eigen::Matrix3d inertia_com_world = Eigen::Matrix3d::Identity();
    double w_pos_xy = 40.0;
    double w_pos_z = 80.0;
    double w_ori = 80.0;
    double w_vel_xy = 40.0;
    double w_vel_z = 8.0;
    double w_omega = 4.0;
    double w_force = 1.0e-4;
    double w_force_trot_xy = 4.0e-4;
};

struct SrbdMpcInput
{
    // x = [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]
    Eigen::Matrix<double, kSrbdStateSize, 1> state =
        Eigen::Matrix<double, kSrbdStateSize, 1>::Zero();
    Eigen::Matrix<double, kSrbdStateSize, 1> reference =
        Eigen::Matrix<double, kSrbdStateSize, 1>::Zero();
    bool has_time_indexed_reference = false;
    std::array<Eigen::Matrix<double, kSrbdStateSize, 1>,
               kSrbdMaxHorizon>
        reference_horizon{};
    std::array<Eigen::Vector3d, go2::kLegCount> foot_from_com_world{};
    // Optional Stage-B contract.  When enabled, every scheduled contact knot
    // must have a valid foot position; the solver never mixes this sequence
    // with the legacy single-anchor field.
    bool has_time_indexed_footholds = false;
    std::array<std::array<Eigen::Vector3d, go2::kLegCount>,
               kSrbdMaxHorizon>
        foot_from_com_world_horizon{};
    std::array<std::array<bool, go2::kLegCount>, kSrbdMaxHorizon>
        foot_valid{};
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    // Planner input identity is diagnostic provenance; it is not a solver
    // decision variable and remains optional for legacy/flat inputs.
    std::uint64_t terrain_input_hash = 0;
    bool has_terrain_plan = false;
    go2_terrain::TerrainPlanIdentity terrain_plan{};
    std::array<bool, go2::kLegCount> measured_contact{};
    bool measured_contact_valid = false;
    std::array<std::array<bool, go2::kLegCount>, kSrbdMaxHorizon> contact{};
};

struct SrbdMpcOutput
{
    bool ok = false;
    int iterations = 0;
    double cost = 0.0;
    Eigen::Matrix<double, kSrbdForceSize, 1> first_force =
        Eigen::Matrix<double, kSrbdForceSize, 1>::Zero();
    bool terrain_plan_consumed = false;
    go2_terrain::TerrainPlanIdentity terrain_plan{};
    Eigen::Vector3d first_linear_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d first_angular_acc = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, kSrbdStateSize, 1> predicted_state =
        Eigen::Matrix<double, kSrbdStateSize, 1>::Zero();
};

inline Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize> SrbdAd(
    double dt_s)
{
    Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize> A =
        Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize>::Identity();
    A.block<3, 3>(0, 6) = dt_s * Eigen::Matrix3d::Identity();
    A.block<3, 3>(3, 9) = dt_s * Eigen::Matrix3d::Identity();
    return A;
}

inline Eigen::Matrix<double, kSrbdStateSize, kSrbdForceSize> SrbdBd(
    const SrbdMpcParams &params,
    const std::array<Eigen::Vector3d, go2::kLegCount> &r_com,
    const std::array<bool, go2::kLegCount> &contact)
{
    Eigen::Matrix<double, kSrbdStateSize, kSrbdForceSize> B =
        Eigen::Matrix<double, kSrbdStateSize, kSrbdForceSize>::Zero();
    const Eigen::Matrix3d Iinv = params.inertia_com_world.inverse();
    const double dt = params.dt_s;
    const double inv_m = 1.0 / params.mass_kg;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!contact[leg])
            continue;
        const int c = static_cast<int>(3 * leg);
        Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
        skew << 0.0, -r_com[leg].z(), r_com[leg].y(),
            r_com[leg].z(), 0.0, -r_com[leg].x(),
            -r_com[leg].y(), r_com[leg].x(), 0.0;
        B.block<3, 3>(6, c) = dt * Iinv * skew;
        B.block<3, 3>(9, c) = dt * inv_m * Eigen::Matrix3d::Identity();
    }
    return B;
}

inline const Eigen::Vector3d &SrbdFootAt(
    const SrbdMpcInput &input, int knot, std::size_t leg)
{
    // Only valid timed entries are consumed. This keeps unused swing entries
    // out of the dynamics without silently replacing a required contact
    // lever arm (the validator rejects that case).
    return input.has_time_indexed_footholds &&
            input.foot_valid[static_cast<std::size_t>(knot)][leg]
        ? input.foot_from_com_world_horizon[static_cast<std::size_t>(knot)][leg]
        : input.foot_from_com_world[leg];
}

inline bool ValidateSrbdFootHorizon(const SrbdMpcParams &params,
                                    const SrbdMpcInput &input)
{
    if (!input.has_time_indexed_footholds)
        return true;
    for (int k = 0; k < params.horizon; ++k)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            // A lever arm is required exactly where a contact wrench may be
            // applied. Unscheduled legs have no prediction and must not make
            // a complete snapshot fail due to an unused Eigen value.
            if (!input.contact[k][leg])
            {
                if (input.foot_valid[k][leg] &&
                    !SrbdFootAt(input, k, leg).allFinite())
                    return false;
                continue;
            }
            if (!input.foot_valid[k][leg] ||
                !SrbdFootAt(input, k, leg).allFinite())
                return false;
        }
    }
    return true;
}

inline bool ValidateSrbdReferenceHorizon(const SrbdMpcParams &params,
                                         const SrbdMpcInput &input)
{
    if (!input.has_time_indexed_reference)
        return true;
    for (int k = 0; k < params.horizon; ++k)
        if (!input.reference_horizon[static_cast<std::size_t>(k)].allFinite())
            return false;
    return true;
}

inline bool ValidateSrbdTerrainReference(const SrbdMpcInput &input)
{
    if (!input.has_terrain_plan)
        return true;
    if (!input.terrain_plan.valid() || !input.measured_contact_valid ||
        input.plan_id == 0 || input.plan_epoch == 0)
        return false;
    // Plan, map and validity identity are one atomic provenance token. Do
    // not pair a fresh horizon with another plan.
    return input.terrain_plan.plan_id == input.plan_id &&
        input.terrain_plan.plan_epoch == input.plan_epoch;
}

inline Eigen::Matrix<double, kSrbdStateSize, 1> SrbdGravity(
    const SrbdMpcParams &params)
{
    Eigen::Matrix<double, kSrbdStateSize, 1> g =
        Eigen::Matrix<double, kSrbdStateSize, 1>::Zero();
    g[11] = -params.gravity_mps2 * params.dt_s;
    return g;
}

inline Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize> SrbdQ(
    const SrbdMpcParams &params)
{
    Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize> Q =
        Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize>::Zero();
    Q(0, 0) = params.w_ori;
    Q(1, 1) = params.w_ori;
    Q(2, 2) = params.w_ori * 0.25;
    Q(3, 3) = params.w_pos_xy;
    Q(4, 4) = params.w_pos_xy;
    Q(5, 5) = params.w_pos_z;
    Q(6, 6) = params.w_omega;
    Q(7, 7) = params.w_omega;
    Q(8, 8) = params.w_omega;
    Q(9, 9) = params.w_vel_xy;
    Q(10, 10) = params.w_vel_xy;
    Q(11, 11) = params.w_vel_z;
    return Q;
}

inline bool SolveSrbdMpc(
    const SrbdMpcParams &params,
    const SrbdMpcInput &input,
    SrbdMpcOutput &output)
{
    output = SrbdMpcOutput{};
    const int N = params.horizon;
    if (N <= 0 || N > kSrbdMaxHorizon)
        return false;
    if (!(params.dt_s > 0.0) ||
        !(params.mass_kg > 1.0) ||
        !(params.friction_mu >= 0.0) ||
        !input.state.allFinite() ||
        !input.reference.allFinite() ||
        !params.inertia_com_world.allFinite() ||
        !ValidateSrbdFootHorizon(params, input) ||
        !ValidateSrbdReferenceHorizon(params, input) ||
        !ValidateSrbdTerrainReference(input))
    {
        return false;
    }
    const Eigen::LDLT<Eigen::Matrix3d> inertia_ldlt(params.inertia_com_world);
    if (inertia_ldlt.info() != Eigen::Success)
        return false;

    const int nu = kSrbdForceSize * N;
    const int nx = kSrbdStateSize * N;
    Eigen::MatrixXd H_xu = Eigen::MatrixXd::Zero(nx, nu);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(nx);
    const auto A = SrbdAd(params.dt_s);
    const auto g = SrbdGravity(params);
    Eigen::Matrix<double, kSrbdStateSize, 1> x_pred = input.state;
    for (int k = 0; k < N; ++k)
    {
        std::array<Eigen::Vector3d, go2::kLegCount> feet{};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            feet[leg] = SrbdFootAt(input, k, leg);
        const auto B = SrbdBd(params, feet, input.contact[k]);
        x_pred = A * x_pred + g;
        d.segment<kSrbdStateSize>(kSrbdStateSize * k) = x_pred;
        Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize> Ak =
            Eigen::Matrix<double, kSrbdStateSize, kSrbdStateSize>::Identity();
        for (int j = k; j >= 0; --j)
        {
            std::array<Eigen::Vector3d, go2::kLegCount> feet_j{};
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                feet_j[leg] = SrbdFootAt(input, j, leg);
            const auto Bj = SrbdBd(params, feet_j, input.contact[j]);
            H_xu.block(
                kSrbdStateSize * k, kSrbdForceSize * j,
                kSrbdStateSize, kSrbdForceSize) = Ak * Bj;
            if (j > 0)
                Ak = Ak * A;
        }
        x_pred = d.segment<kSrbdStateSize>(kSrbdStateSize * k);
    }

    const auto Q = SrbdQ(params);
    Eigen::MatrixXd Qblk = Eigen::MatrixXd::Zero(nx, nx);
    Eigen::VectorXd xref = Eigen::VectorXd::Zero(nx);
    for (int k = 0; k < N; ++k)
    {
        Qblk.block<kSrbdStateSize, kSrbdStateSize>(
            kSrbdStateSize * k, kSrbdStateSize * k) = Q;
        Eigen::Matrix<double, kSrbdStateSize, 1> knot_ref =
            input.has_time_indexed_reference
                ? input.reference_horizon[static_cast<std::size_t>(k)]
                : input.reference;
        if (!input.has_time_indexed_reference)
        {
            knot_ref[3] = input.reference[3] +
                static_cast<double>(k + 1) * params.dt_s * input.reference[9];
            knot_ref[4] = input.reference[4] +
                static_cast<double>(k + 1) * params.dt_s * input.reference[10];
        }
        xref.segment<kSrbdStateSize>(kSrbdStateSize * k) = knot_ref;
    }
    Eigen::MatrixXd H = H_xu.transpose() * Qblk * H_xu;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu, nu);
    for (int k = 0; k < N; ++k)
    {
        for (int i = 0; i < kSrbdForceSize; ++i)
        {
            const int idx = k * kSrbdForceSize + i;
            const bool lateral = (i % 3) != 2;
            R(idx, idx) = lateral ? params.w_force_trot_xy : params.w_force;
        }
    }
    H += R;
    H.diagonal().array() += 1.0e-8;
    const Eigen::VectorXd gvec =
        H_xu.transpose() * Qblk * (d - xref);

    const int m = 6 * 4 * N;
    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(m, nu);
    Eigen::VectorXd bineq = Eigen::VectorXd::Zero(m);
    int row = 0;
    const double mu = params.friction_mu / std::sqrt(2.0);
    for (int k = 0; k < N; ++k)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int c = k * kSrbdForceSize + static_cast<int>(3 * leg);
            if (!input.contact[k][leg])
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    Aineq(row, c + axis) = 1.0;
                    bineq[row] = 1.0e-6;
                    ++row;
                    Aineq(row, c + axis) = -1.0;
                    bineq[row] = 1.0e-6;
                    ++row;
                }
                continue;
            }
            Aineq(row, c + 2) = -1.0;
            bineq[row] = -params.min_normal_n;
            ++row;
            Aineq(row, c + 2) = 1.0;
            bineq[row] = params.max_normal_n;
            ++row;
            Aineq(row, c + 0) = 1.0;
            Aineq(row, c + 2) = -mu;
            ++row;
            Aineq(row, c + 0) = -1.0;
            Aineq(row, c + 2) = -mu;
            ++row;
            Aineq(row, c + 1) = 1.0;
            Aineq(row, c + 2) = -mu;
            ++row;
            Aineq(row, c + 1) = -1.0;
            Aineq(row, c + 2) = -mu;
            ++row;
        }
    }

    Eigen::VectorXd u;
    int iters = 0;
    DenseQpSettings settings;
    settings.max_iterations = 120;
    settings.rho = 8.0;
    settings.abs_tol = 1e-5;
    settings.rel_tol = 1e-4;
    settings.feasibility_tol = 1e-4;
    if (!SolveDenseQp(H, gvec, Aineq, bineq, u, iters, settings) ||
        u.size() != nu)
    {
        return false;
    }

    output.ok = true;
    output.iterations = iters;
    output.first_force = u.head<kSrbdForceSize>();
    const Eigen::VectorXd X = H_xu * u + d;
    output.predicted_state = X.head<kSrbdStateSize>();
    output.cost = 0.5 * u.dot(H * u) + gvec.dot(u);
    Eigen::Vector3d fsum = Eigen::Vector3d::Zero();
    Eigen::Vector3d tau = Eigen::Vector3d::Zero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!input.contact[0][leg])
            continue;
        const Eigen::Vector3d f = output.first_force.segment<3>(3 * leg);
        fsum += f;
        tau += SrbdFootAt(input, 0, leg).cross(f);
    }
    output.first_linear_acc = fsum / params.mass_kg +
        Eigen::Vector3d(0.0, 0.0, -params.gravity_mps2);
    if (input.has_terrain_plan)
    {
        output.terrain_plan_consumed = true;
        output.terrain_plan = input.terrain_plan;
    }
    output.first_angular_acc = inertia_ldlt.solve(tau);
    return true;
}

// Diagonal trot contact from the kernel phase (0-1), then dt/period ahead.
template <std::size_t Horizon>
inline void FillTrotContactSchedulePhase(
    double phase,
    double period_s,
    double duty,
    int horizon,
    double dt_s,
    std::array<std::array<bool, go2::kLegCount>, Horizon> &contact,
    GaitPattern pattern = GaitPattern::kDiagonalTrot)
{
    contact = {};
    if (!(period_s > 0.0))
        return;
    for (int k = 0;
         k < horizon && k < static_cast<int>(Horizon); ++k)
    {
        double a = phase + static_cast<double>(k) * dt_s / period_s;
        a -= std::floor(a);
        if (a < 0.0)
            a += 1.0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            contact[k][leg] = GaitLegScheduledStance(
                leg, a, duty, pattern);
    }
}

// Diagonal trot contact at time t: FR+RL vs FL+RR, offset by half period.
inline void FillTrotContactSchedule(
    double gait_time_s,
    double period_s,
    double duty,
    int horizon,
    double dt_s,
    std::array<std::array<bool, go2::kLegCount>, kSrbdMaxHorizon> &contact,
    GaitPattern pattern = GaitPattern::kDiagonalTrot)
{
    const double cycle = (period_s > 0.0) ? (gait_time_s / period_s) : 0.0;
    FillTrotContactSchedulePhase(
        cycle - std::floor(cycle), period_s, duty, horizon, dt_s, contact,
        pattern);
}

}  // namespace go2_control
