#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Dense>

#include "contact_wrench_constraints.h"

namespace go2_control
{

struct ProjectedContactWrenchRequest
{
    ContactWrenchRequest wrench{};
    ContactForceConstraintParams force_constraints{};
    int max_iterations = 2000;
    // Euclidean norm mixes force (N) and moment (N m); keep it explicit.
    double residual_tolerance = 1e-5;
    double step_tolerance = 1e-10;
};

struct ProjectedContactWrenchSolution
{
    std::array<go2::Vec3, go2::kLegCount> forces{};
    int active_contacts = 0;
    int iterations = 0;
    bool converged = false;
    bool wrench_satisfied = false;
    double residual_norm = std::numeric_limits<double>::infinity();
    double task_residual_norm = std::numeric_limits<double>::infinity();
    bool task_satisfied = false;
    double max_axis_friction_ratio = 0.0;
    double max_radial_friction_ratio = 0.0;
    double min_contact_normal_force = std::numeric_limits<double>::infinity();
    ContactForceConstraintReport constraint_report{};
};

class ContactWrenchProjectedAllocator
{
public:
    bool Solve(
        const ProjectedContactWrenchRequest &request,
        ProjectedContactWrenchSolution &solution) const
    {
        solution = ProjectedContactWrenchSolution{};
        if (!ValidateRequest(request))
            return false;

        const Eigen::Matrix<double, 6, 12> wrench_map =
            BuildWrenchMap(request.wrench);
        Eigen::Matrix<double, 12, 1> force_vector;
        force_vector.setZero();
        ContactWrenchLeastNormAllocator unconstrained_allocator;
        ContactWrenchSolution seed;
        if (unconstrained_allocator.Solve(request.wrench, seed))
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                force_vector[3 * leg + 0] = seed.forces[leg].x;
                force_vector[3 * leg + 1] = seed.forces[leg].y;
                force_vector[3 * leg + 2] = seed.forces[leg].z;
            }
        }
        ProjectAllForces(request, force_vector);

        Eigen::Matrix<double, 6, 1> desired_wrench;
        for (int i = 0; i < 6; ++i)
            desired_wrench[i] = request.wrench.desired_wrench[i];

        Eigen::Matrix<double, 6, 12> weighted_wrench_map =
            wrench_map;
        Eigen::Matrix<double, 6, 1> weighted_desired_wrench;
        for (int i = 0; i < 6; ++i)
        {
            weighted_wrench_map.row(i) *= request.wrench.task_weights[i];
            weighted_desired_wrench[i] =
                request.wrench.task_weights[i] * desired_wrench[i];
        }

        const double lipschitz_bound =
            weighted_wrench_map.squaredNorm() + request.wrench.regularization;
        if (!std::isfinite(lipschitz_bound) || lipschitz_bound <= 0.0)
            return false;
        const double step = 0.9 / lipschitz_bound;

        for (int iteration = 0;
             iteration < request.max_iterations;
             ++iteration)
        {
            const Eigen::Matrix<double, 6, 1> residual =
                wrench_map * force_vector - desired_wrench;
            solution.residual_norm = residual.norm();
            if (!std::isfinite(solution.residual_norm))
                return false;
            const Eigen::Matrix<double, 6, 1> weighted_residual =
                weighted_wrench_map * force_vector -
                weighted_desired_wrench;
            solution.task_residual_norm = weighted_residual.norm();
            if (!std::isfinite(solution.task_residual_norm))
                return false;
            if (solution.task_residual_norm <= request.residual_tolerance)
            {
                solution.converged = true;
                solution.task_satisfied = true;
                solution.iterations = iteration;
                break;
            }

            const Eigen::Matrix<double, 12, 1> gradient =
                weighted_wrench_map.transpose() * weighted_residual +
                request.wrench.regularization * force_vector;
            Eigen::Matrix<double, 12, 1> next_force =
                force_vector - step * gradient;
            ProjectAllForces(request, next_force);
            const double step_delta = (next_force - force_vector).norm();
            if (!std::isfinite(step_delta))
                return false;
            force_vector = next_force;
            solution.iterations = iteration + 1;
            if (step_delta <= request.step_tolerance)
            {
                solution.converged = true;
                break;
            }
        }

        const Eigen::Matrix<double, 6, 1> final_residual =
            wrench_map * force_vector - desired_wrench;
        solution.residual_norm = final_residual.norm();
        const Eigen::Matrix<double, 6, 1> final_weighted_residual =
            weighted_wrench_map * force_vector - weighted_desired_wrench;
        solution.task_residual_norm = final_weighted_residual.norm();
        solution.task_satisfied =
            std::isfinite(solution.task_residual_norm) &&
            solution.task_residual_norm <= request.residual_tolerance;
        solution.wrench_satisfied =
            std::isfinite(solution.residual_norm) &&
            solution.residual_norm <= request.residual_tolerance;
        if (solution.wrench_satisfied || solution.task_satisfied)
            solution.converged = true;

        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            solution.forces[leg] = {
                force_vector[3 * leg + 0],
                force_vector[3 * leg + 1],
                force_vector[3 * leg + 2]};
        }
        solution.active_contacts = CountActiveContacts(request.wrench);
        UpdateForceDiagnostics(request, solution);
        if (!EvaluateContactForceConstraints(
                request.wrench,
                solution.forces,
                request.force_constraints,
                solution.constraint_report))
        {
            return false;
        }
        return true;
    }

private:
    static bool ValidateRequest(
        const ProjectedContactWrenchRequest &request)
    {
        if (!ValidateContactForceConstraintParams(
                request.force_constraints) ||
            !std::isfinite(request.wrench.regularization) ||
            request.wrench.regularization <= 0.0 ||
            request.wrench.regularization > 1.0 ||
            request.max_iterations <= 0 ||
            request.max_iterations > 100000 ||
            !std::isfinite(request.residual_tolerance) ||
            request.residual_tolerance <= 0.0 ||
            !std::isfinite(request.step_tolerance) ||
            request.step_tolerance <= 0.0)
        {
            return false;
        }

        if (CountActiveContacts(request.wrench) < 2)
            return false;
        for (const go2::Vec3 &position :
             request.wrench.contact_positions_body)
        {
            if (!std::isfinite(position.x) ||
                !std::isfinite(position.y) ||
                !std::isfinite(position.z))
            {
                return false;
            }
        }
        for (double value : request.wrench.desired_wrench)
        {
            if (!std::isfinite(value))
                return false;
        }
        bool has_task_weight = false;
        for (double weight : request.wrench.task_weights)
        {
            if (!std::isfinite(weight) || weight < 0.0)
                return false;
            has_task_weight = has_task_weight || weight > 0.0;
        }
        if (!has_task_weight)
            return false;
        return true;
    }

    static int CountActiveContacts(
        const ContactWrenchRequest &request)
    {
        int count = 0;
        for (bool active : request.contact)
            count += active ? 1 : 0;
        return count;
    }

    static Eigen::Matrix3d Skew(const go2::Vec3 &vector)
    {
        Eigen::Matrix3d matrix;
        matrix << 0.0, -vector.z, vector.y,
            vector.z, 0.0, -vector.x,
            -vector.y, vector.x, 0.0;
        return matrix;
    }

    static Eigen::Matrix<double, 6, 12> BuildWrenchMap(
        const ContactWrenchRequest &request)
    {
        Eigen::Matrix<double, 6, 12> wrench_map;
        wrench_map.setZero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.contact[leg])
                continue;
            wrench_map.block<3, 3>(0, 3 * leg) =
                Eigen::Matrix3d::Identity();
            wrench_map.block<3, 3>(3, 3 * leg) =
                Skew(request.contact_positions_body[leg]);
        }
        return wrench_map;
    }

    static void UpdateForceDiagnostics(
        const ProjectedContactWrenchRequest &request,
        ProjectedContactWrenchSolution &solution)
    {
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
            if (denominator <= request.force_constraints.tolerance)
            {
                if (tangent_axis > request.force_constraints.tolerance)
                {
                    solution.max_axis_friction_ratio =
                        std::numeric_limits<double>::infinity();
                    solution.max_radial_friction_ratio =
                        std::numeric_limits<double>::infinity();
                }
                continue;
            }
            solution.max_axis_friction_ratio = std::max(
                solution.max_axis_friction_ratio,
                tangent_axis / denominator);
            solution.max_radial_friction_ratio = std::max(
                solution.max_radial_friction_ratio,
                tangent_radial / denominator);
        }
        if (!std::isfinite(solution.min_contact_normal_force))
            solution.min_contact_normal_force = 0.0;
    }

    static void ProjectContactForce(
        const ContactForceConstraintParams &params,
        go2::Vec3 &force)
    {
        const double normal = std::max(
            params.min_normal_force,
            std::min(params.max_normal_force, force.z));
        const double tangent_norm = std::hypot(force.x, force.y);
        if (params.friction_coefficient <= params.tolerance)
        {
            force.x = 0.0;
            force.y = 0.0;
            force.z = normal;
            return;
        }

        const double tangent_limit =
            params.friction_coefficient * normal;
        if (tangent_norm <= tangent_limit)
        {
            force.z = normal;
            return;
        }

        const double projected_normal = std::max(
            params.min_normal_force,
            std::min(
                params.max_normal_force,
                (params.friction_coefficient * tangent_norm + force.z) /
                    (1.0 + params.friction_coefficient *
                               params.friction_coefficient)));
        const double projected_tangent =
            params.friction_coefficient * projected_normal;
        const double scale =
            projected_tangent / std::max(tangent_norm, 1e-12);
        force.x *= scale;
        force.y *= scale;
        force.z = projected_normal;
    }

    static void ProjectAllForces(
        const ProjectedContactWrenchRequest &request,
        Eigen::Matrix<double, 12, 1> &force_vector)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int index = static_cast<int>(3 * leg);
            if (!request.wrench.contact[leg])
            {
                force_vector.segment<3>(index).setZero();
                continue;
            }

            go2::Vec3 force{
                force_vector[index + 0],
                force_vector[index + 1],
                force_vector[index + 2]};
            ProjectContactForce(request.force_constraints, force);
            force_vector[index + 0] = force.x;
            force_vector[index + 1] = force.y;
            force_vector[index + 2] = force.z;
        }
    }
};

} // namespace go2_control
