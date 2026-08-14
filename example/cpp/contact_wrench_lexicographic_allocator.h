#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Dense>

#include "contact_wrench_projected_allocator.h"
#include "go2_leg_jacobian.h"

namespace go2_control
{

struct LexicographicContactWrenchRequest
{
    ContactWrenchRequest wrench{};
    ContactForceConstraintParams force_constraints{};
    // Force is the primary task. Moment residual inside this bound is
    // represented by an explicit slack variable and is not penalized.
    bool moment_task_active = true;
    double force_tolerance = 0.01;
    double moment_slack_limit = 0.05;
    double force_weight = 1000.0;
    double moment_weight = 1.0;
    int max_iterations = 4000;
    double step_tolerance = 1e-10;
    bool torque_rate_task_active = false;
    std::array<std::array<double, 3>, go2::kLegCount> joint_angles{};
    std::array<double, go2::kJointCount> previous_torque{};
    double dt_s = 0.0;
    double torque_rate_limit_nm_s = 0.0;
    double torque_rate_weight = 1.0;
    double residual_tolerance = 1e-5;
};

struct LexicographicContactWrenchSolution
{
    std::array<go2::Vec3, go2::kLegCount> forces{};
    std::array<double, 3> force_residual{};
    std::array<double, 3> moment_residual{};
    std::array<double, 3> force_slack{};
    std::array<double, 3> moment_slack{};
    int active_contacts = 0;
    int iterations = 0;
    bool converged = false;
    bool wrench_satisfied = false;
    bool policy_satisfied = false;
    bool moment_task_active = false;
    bool torque_rate_task_active = false;
    bool fallback_to_force_solution = false;
    double residual_norm = std::numeric_limits<double>::infinity();
    double max_force_excess = std::numeric_limits<double>::infinity();
    double max_torque_rate_excess = std::numeric_limits<double>::infinity();
    bool torque_rate_satisfied = false;
    double max_moment_excess = std::numeric_limits<double>::infinity();
    double max_axis_friction_ratio = 0.0;
    double max_radial_friction_ratio = 0.0;
    double min_contact_normal_force =
        std::numeric_limits<double>::infinity();
    ContactForceConstraintReport constraint_report{};
};

class ContactWrenchLexicographicSlackAllocator
{
public:
    bool Solve(
        const LexicographicContactWrenchRequest &request,
        LexicographicContactWrenchSolution &solution) const
    {
        solution = LexicographicContactWrenchSolution{};
        if (!ValidateRequest(request))
            return false;

        ProjectedContactWrenchRequest force_request;
        force_request.wrench = request.wrench;
        force_request.wrench.task_weights =
            {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
        force_request.force_constraints = request.force_constraints;
        force_request.max_iterations = request.max_iterations;
        force_request.residual_tolerance = request.residual_tolerance;
        force_request.step_tolerance = request.step_tolerance;

        ContactWrenchProjectedAllocator force_allocator;
        ProjectedContactWrenchSolution force_solution;
        if (!force_allocator.Solve(force_request, force_solution))
            return false;

        Eigen::Matrix<double, 12, 1> force_vector;
        force_vector.setZero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            force_vector[3 * leg + 0] = force_solution.forces[leg].x;
            force_vector[3 * leg + 1] = force_solution.forces[leg].y;
            force_vector[3 * leg + 2] = force_solution.forces[leg].z;
        }

        Eigen::Matrix<double, 12, 1> selected_force = force_vector;
        bool optimizer_converged = true;
        int optimizer_iterations = 0;
        const Eigen::Matrix<double, 12, 12> torque_map =
            BuildTorqueMap(request);
        if (request.moment_task_active ||
            request.torque_rate_task_active)
        {
            const Eigen::Matrix<double, 6, 12> wrench_map =
                BuildWrenchMap(request.wrench);
            const double lipschitz_bound =
                request.force_weight *
                    wrench_map.topRows<3>().squaredNorm() +
                request.moment_weight *
                    wrench_map.bottomRows<3>().squaredNorm() +
                request.wrench.regularization +
                (request.torque_rate_task_active
                     ? request.torque_rate_weight * torque_map.squaredNorm()
                     : 0.0);
            if (!std::isfinite(lipschitz_bound) ||
                lipschitz_bound <= 0.0)
                return false;
            const double step = 0.9 / lipschitz_bound;

            optimizer_converged = false;
            for (int iteration = 0;
                 iteration < request.max_iterations;
                 ++iteration)
            {
                const Eigen::Matrix<double, 6, 1> residual =
                    wrench_map * selected_force -
                    ToEigen(request.wrench.desired_wrench);
                Eigen::Matrix<double, 6, 1> weighted_residual;
                weighted_residual.setZero();
                for (int axis = 0; axis < 3; ++axis)
                {
                    weighted_residual[axis] =
                        request.force_weight *
                        (request.torque_rate_task_active
                             ? residual[axis]
                             : HingeResidual(
                                   residual[axis],
                                   request.force_tolerance));
                }
                for (int axis = 3; axis < 6; ++axis)
                {
                    weighted_residual[axis] =
                        request.moment_weight *
                        HingeResidual(
                            residual[axis],
                            request.moment_slack_limit);
                }
                Eigen::Matrix<double, 12, 1> weighted_torque_residual;
                weighted_torque_residual.setZero();
                if (request.torque_rate_task_active)
                {
                    const Eigen::Matrix<double, 12, 1> torque_residual =
                        torque_map * selected_force -
                        ToEigen(request.previous_torque);
                    const double torque_tolerance =
                        request.torque_rate_limit_nm_s * request.dt_s;
                    for (int axis = 0; axis < 12; ++axis)
                    {
                        weighted_torque_residual[axis] =
                            request.torque_rate_weight *
                            HingeResidual(
                                torque_residual[axis],
                                torque_tolerance);
                    }

                }
                Eigen::Matrix<double, 12, 1> gradient =
                    wrench_map.transpose() * weighted_residual +
                    request.wrench.regularization * selected_force;
                if (request.torque_rate_task_active)
                    gradient +=
                        torque_map.transpose() * weighted_torque_residual;
                Eigen::Matrix<double, 12, 1> next_force =
                    selected_force - step * gradient;
                ProjectAllForces(request, next_force);
                const double step_delta =
                    (next_force - selected_force).norm();
                if (!std::isfinite(step_delta))
                    return false;
                selected_force = next_force;
                optimizer_iterations = iteration + 1;
                if (step_delta <= request.step_tolerance)
                {
                    optimizer_converged = true;
                    break;
                }
            }
        }

        LexicographicContactWrenchSolution candidate;
        Evaluate(
            request,
            selected_force,
            candidate);
        const bool force_guard_pass =
            candidate.max_force_excess <=
            request.residual_tolerance;
        if (force_guard_pass)
        {
            solution = candidate;
            solution.fallback_to_force_solution = false;
            solution.iterations = optimizer_iterations;
            solution.converged =
                force_solution.converged && optimizer_converged;
            return true;
        }

        Evaluate(request, force_vector, solution);
        solution.fallback_to_force_solution = true;
        solution.iterations = force_solution.iterations;
        solution.converged = force_solution.converged;
        return true;
    }

private:
    static bool ValidateRequest(
        const LexicographicContactWrenchRequest &request)
    {
        if (!ValidateContactForceConstraintParams(
                request.force_constraints) ||
            CountActiveContacts(request.wrench) < 2 ||
            !std::isfinite(request.wrench.regularization) ||
            request.wrench.regularization <= 0.0 ||
            request.wrench.regularization > 1.0 ||
            !std::isfinite(request.force_tolerance) ||
            request.force_tolerance < 0.0 ||
            !std::isfinite(request.moment_slack_limit) ||
            request.moment_slack_limit < 0.0 ||
            !std::isfinite(request.force_weight) ||
            request.force_weight <= 0.0 ||
            !std::isfinite(request.moment_weight) ||
            request.moment_weight < 0.0 ||
            request.max_iterations <= 0 ||
            request.max_iterations > 100000 ||
            !std::isfinite(request.step_tolerance) ||
            request.step_tolerance <= 0.0 ||
            !std::isfinite(request.residual_tolerance) ||
            request.residual_tolerance <= 0.0)
        {
            return false;
        }

        for (const go2::Vec3 &position :
             request.wrench.contact_positions_body)
        {
            if (!std::isfinite(position.x) ||
                !std::isfinite(position.y) ||
                !std::isfinite(position.z))
                return false;
        }
        for (double value : request.wrench.desired_wrench)
        {
            if (!std::isfinite(value))
                return false;
        }
        if (request.torque_rate_task_active)
        {
            if (!std::isfinite(request.dt_s) ||
                request.dt_s <= 0.0 ||
                !std::isfinite(request.torque_rate_limit_nm_s) ||
                request.torque_rate_limit_nm_s < 0.0 ||
                !std::isfinite(request.torque_rate_weight) ||
                request.torque_rate_weight <= 0.0)
                return false;
            for (const auto &angles : request.joint_angles)
            {
                for (double angle : angles)
                {
                    if (!std::isfinite(angle))
                        return false;
                }
            }
            for (double torque : request.previous_torque)
            {
                if (!std::isfinite(torque))
                    return false;
            }
        }
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
    static Eigen::Matrix<double, 12, 12> BuildTorqueMap(
        const LexicographicContactWrenchRequest &request)
    {
        Eigen::Matrix<double, 12, 12> torque_map;
        torque_map.setZero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &angles = request.joint_angles[leg];
            const LegFootJacobian jacobian = FootJacobian(
                static_cast<go2::Leg>(leg),
                angles[0],
                angles[1],
                angles[2]);
            for (std::size_t joint = 0; joint < go2::kJointsPerLeg; ++joint)
            {
                for (std::size_t coordinate = 0;
                     coordinate < 3;
                     ++coordinate)
                {
                    torque_map(3 * leg + joint, 3 * leg + coordinate) =
                        jacobian[coordinate][joint];
                }
            }
        }
        return torque_map;
    }

    static Eigen::Matrix<double, 6, 1> ToEigen(
        const std::array<double, 6> &values)
    {
        Eigen::Matrix<double, 6, 1> result;
        for (int index = 0; index < 6; ++index)
            result[index] = values[index];
        return result;
    }

    static Eigen::Matrix<double, 12, 1> ToEigen(
        const std::array<double, go2::kJointCount> &values)
    {
        Eigen::Matrix<double, 12, 1> result;
        for (int index = 0; index < 12; ++index)
            result[index] = values[index];
        return result;
    }

    static double HingeResidual(
        double residual,
        double tolerance)
    {
        const double magnitude = std::abs(residual);
        if (magnitude <= tolerance)
            return 0.0;
        return std::copysign(magnitude - tolerance, residual);
    }

    static double ClampedSlack(
        double residual,
        double limit)
    {
        return std::max(-limit, std::min(limit, residual));
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
        const LexicographicContactWrenchRequest &request,
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

    static void UpdateForceDiagnostics(
        const LexicographicContactWrenchRequest &request,
        const Eigen::Matrix<double, 12, 1> &force_vector,
        LexicographicContactWrenchSolution &solution)
    {
        solution.max_axis_friction_ratio = 0.0;
        solution.max_radial_friction_ratio = 0.0;
        solution.min_contact_normal_force =
            std::numeric_limits<double>::infinity();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.wrench.contact[leg])
                continue;
            const go2::Vec3 force{
                force_vector[3 * leg + 0],
                force_vector[3 * leg + 1],
                force_vector[3 * leg + 2]};
            solution.min_contact_normal_force = std::min(
                solution.min_contact_normal_force,
                force.z);
            const double tangent_axis =
                std::max(std::abs(force.x), std::abs(force.y));
            const double tangent_radial =
                std::hypot(force.x, force.y);
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

    static void Evaluate(
        const LexicographicContactWrenchRequest &request,
        const Eigen::Matrix<double, 12, 1> &force_vector,
        LexicographicContactWrenchSolution &solution)
    {
        const Eigen::Matrix<double, 6, 1> residual =
            BuildWrenchMap(request.wrench) * force_vector -
            ToEigen(request.wrench.desired_wrench);
        const Eigen::Matrix<double, 12, 12> torque_map =
            BuildTorqueMap(request);
        solution.torque_rate_task_active =
            request.torque_rate_task_active;
        solution.active_contacts = CountActiveContacts(request.wrench);
        solution.moment_task_active = request.moment_task_active;
        solution.residual_norm = residual.norm();
        solution.wrench_satisfied =
            std::isfinite(solution.residual_norm) &&
            solution.residual_norm <= request.residual_tolerance;
        solution.max_force_excess = 0.0;
        solution.max_moment_excess = 0.0;
        solution.max_torque_rate_excess = 0.0;
        if (request.torque_rate_task_active)
        {
            const Eigen::Matrix<double, 12, 1> torque =
                torque_map * force_vector;
            const double torque_tolerance =
                request.torque_rate_limit_nm_s * request.dt_s;
            for (int axis = 0; axis < 12; ++axis)
            {
                solution.max_torque_rate_excess = std::max(
                    solution.max_torque_rate_excess,
                    std::max(
                        0.0,
                        std::abs(
                            torque[axis] -
                            request.previous_torque[axis]) -
                            torque_tolerance));
            }
            solution.torque_rate_satisfied =
                solution.max_torque_rate_excess <=
                request.residual_tolerance;
        }
        else
        {
            solution.torque_rate_satisfied = true;
        }
        for (int axis = 0; axis < 3; ++axis)
        {
            solution.force_residual[axis] = residual[axis];
            solution.force_slack[axis] =
                ClampedSlack(residual[axis], request.force_tolerance);
            solution.max_force_excess = std::max(
                solution.max_force_excess,
                std::max(
                    0.0,
                    std::abs(residual[axis]) -
                        request.force_tolerance));
            solution.moment_residual[axis] = residual[axis + 3];
            solution.moment_slack[axis] =
                request.moment_task_active
                    ? ClampedSlack(
                          residual[axis + 3],
                          request.moment_slack_limit)
                    : 0.0;
            if (request.moment_task_active)
            {
                solution.max_moment_excess = std::max(
                    solution.max_moment_excess,
                    std::max(
                        0.0,
                        std::abs(residual[axis + 3]) -
                            request.moment_slack_limit));
            }
        }
        solution.policy_satisfied =
            solution.max_force_excess <= request.residual_tolerance &&
            (!request.moment_task_active ||
             solution.max_moment_excess <= request.residual_tolerance) &&
            (!request.torque_rate_task_active ||
             solution.torque_rate_satisfied);

        solution.forces = {};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            solution.forces[leg] = {
                force_vector[3 * leg + 0],
                force_vector[3 * leg + 1],
                force_vector[3 * leg + 2]};
        }
        EvaluateContactForceConstraints(
            request.wrench,
            solution.forces,
            request.force_constraints,
            solution.constraint_report);
        UpdateForceDiagnostics(request, force_vector, solution);
    }
};

} // namespace go2_control
