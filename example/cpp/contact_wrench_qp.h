#pragma once

// Inequality-constrained contact-force QP:
//   min  1/2 ||W (G f - w*)||^2 + (ε/2) ||f||^2
//   s.t. friction pyramid, unilaterality, inactive feet at 0.
// The pyramid uses μ/√2 so the inscribed square stays inside the cone.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Dense>

#include "contact_wrench_projected_allocator.h"
#include "dense_qp.h"

namespace go2_control
{

class ContactWrenchQpAllocator
{
public:
    bool Solve(
        const ProjectedContactWrenchRequest &request,
        ProjectedContactWrenchSolution &solution) const
    {
        solution = ProjectedContactWrenchSolution{};
        ContactWrenchProjectedAllocator projected;
        if (!projected.Solve(request, solution))
            return false;
        const ProjectedContactWrenchSolution projected_seed = solution;

        const int active = solution.active_contacts;
        if (active < 2)
            return true;

        Eigen::Matrix<double, 6, 12> wrench_map;
        wrench_map.setZero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.wrench.contact[leg])
                continue;
            wrench_map.block<3, 3>(0, 3 * static_cast<int>(leg)) =
                Eigen::Matrix3d::Identity();
            wrench_map.block<3, 3>(3, 3 * static_cast<int>(leg)) <<
                0.0, -request.wrench.contact_positions_body[leg].z,
                    request.wrench.contact_positions_body[leg].y,
                request.wrench.contact_positions_body[leg].z, 0.0,
                    -request.wrench.contact_positions_body[leg].x,
                -request.wrench.contact_positions_body[leg].y,
                    request.wrench.contact_positions_body[leg].x, 0.0;
        }

        Eigen::Matrix<double, 6, 6> weight = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> desired;
        for (int i = 0; i < 6; ++i)
        {
            weight(i, i) = request.wrench.task_weights[i];
            desired[i] = request.wrench.desired_wrench[i];
        }

        std::array<int, go2::kLegCount> packed{-1, -1, -1, -1};
        int n = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.wrench.contact[leg])
                continue;
            packed[leg] = n;
            n += 3;
        }

        Eigen::Matrix<double, 6, Eigen::Dynamic> G(6, n);
        G.setZero();
        Eigen::VectorXd seed(n);
        seed.setZero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (packed[leg] < 0)
                continue;
            G.block(0, packed[leg], 6, 3) =
                wrench_map.block<6, 3>(0, 3 * static_cast<int>(leg));
            seed[packed[leg] + 0] = solution.forces[leg].x;
            seed[packed[leg] + 1] = solution.forces[leg].y;
            seed[packed[leg] + 2] = solution.forces[leg].z;
        }

        const Eigen::Matrix<double, 6, Eigen::Dynamic> WG = weight * G;
        Eigen::MatrixXd H = WG.transpose() * WG;
        H.diagonal().array() += request.wrench.regularization;
        const Eigen::VectorXd gvec = -WG.transpose() * (weight * desired);

        const double mu =
            request.force_constraints.friction_coefficient / std::sqrt(2.0);
        const double fmin = request.force_constraints.min_normal_force;
        const double fmax = request.force_constraints.max_normal_force;
        const int constraints_per_contact = 6;
        const int m = active * constraints_per_contact;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, n);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(m);
        int row = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (packed[leg] < 0)
                continue;
            const int c = packed[leg];
            // -fz <= -fmin
            A(row, c + 2) = -1.0;
            b[row] = -fmin;
            ++row;
            // fz <= fmax
            A(row, c + 2) = 1.0;
            b[row] = std::isfinite(fmax) ? fmax : 1.0e6;
            ++row;
            // fx - mu fz <= 0
            A(row, c + 0) = 1.0;
            A(row, c + 2) = -mu;
            ++row;
            // -fx - mu fz <= 0
            A(row, c + 0) = -1.0;
            A(row, c + 2) = -mu;
            ++row;
            // fy - mu fz <= 0
            A(row, c + 1) = 1.0;
            A(row, c + 2) = -mu;
            ++row;
            // -fy - mu fz <= 0
            A(row, c + 1) = -1.0;
            A(row, c + 2) = -mu;
            ++row;
        }

        Eigen::VectorXd qp_x = seed;
        int qp_iters = 0;
        DenseQpSettings settings;
        settings.max_iterations = std::min(request.max_iterations, 400);
        if (!SolveDenseQp(H, gvec, A, b, qp_x, qp_iters, settings) ||
            qp_x.size() != n)
        {
            return true;
        }

        Eigen::Matrix<double, 12, 1> force_vector;
        force_vector.setZero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (packed[leg] < 0)
            {
                solution.forces[leg] = {};
                continue;
            }
            solution.forces[leg] = {
                qp_x[packed[leg] + 0],
                qp_x[packed[leg] + 1],
                qp_x[packed[leg] + 2]};
            force_vector[3 * static_cast<int>(leg) + 0] = solution.forces[leg].x;
            force_vector[3 * static_cast<int>(leg) + 1] = solution.forces[leg].y;
            force_vector[3 * static_cast<int>(leg) + 2] = solution.forces[leg].z;
        }

        const Eigen::Matrix<double, 6, 1> residual =
            wrench_map * force_vector - desired;
        solution.residual_norm = residual.norm();
        const Eigen::Matrix<double, 6, 1> task_residual =
            weight * residual;
        solution.task_residual_norm = task_residual.norm();
        solution.iterations += qp_iters;
        solution.task_satisfied =
            std::isfinite(solution.task_residual_norm) &&
            solution.task_residual_norm <= request.residual_tolerance;
        solution.wrench_satisfied =
            std::isfinite(solution.residual_norm) &&
            solution.residual_norm <= request.residual_tolerance;
        solution.converged =
            solution.wrench_satisfied || solution.task_satisfied;

        if (!EvaluateContactForceConstraints(
                request.wrench,
                solution.forces,
                request.force_constraints,
                solution.constraint_report) ||
            !solution.constraint_report.feasible)
        {
            solution = projected_seed;
            return true;
        }
        solution.max_axis_friction_ratio = 0.0;
        solution.max_radial_friction_ratio = 0.0;
        solution.min_contact_normal_force =
            std::numeric_limits<double>::infinity();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.wrench.contact[leg])
                continue;
            const go2::Vec3 &force = solution.forces[leg];
            solution.min_contact_normal_force = std::min(
                solution.min_contact_normal_force, force.z);
            const double tangent_axis = std::max(
                std::abs(force.x), std::abs(force.y));
            const double tangent_radial = std::hypot(force.x, force.y);
            const double denominator =
                request.force_constraints.friction_coefficient * force.z;
            if (denominator > request.force_constraints.tolerance)
            {
                solution.max_axis_friction_ratio = std::max(
                    solution.max_axis_friction_ratio,
                    tangent_axis / denominator);
                solution.max_radial_friction_ratio = std::max(
                    solution.max_radial_friction_ratio,
                    tangent_radial / denominator);
            }
        }
        if (!std::isfinite(solution.min_contact_normal_force))
            solution.min_contact_normal_force = 0.0;
        return true;
    }
};

}  // namespace go2_control
