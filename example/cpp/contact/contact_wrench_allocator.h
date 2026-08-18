#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Dense>

#include "go2_forward_kinematics.h"

namespace go2_control
{

struct ContactWrenchRequest
{
    std::array<go2::Vec3, go2::kLegCount> contact_positions_body{};
    std::array<bool, go2::kLegCount> contact{};
    // [Fx, Fy, Fz, Mx, My, Mz], expressed in the body frame.
    std::array<double, 6> desired_wrench{};
    // Per-component residual scale; zero ignores a lower-priority task.
    std::array<double, 6> task_weights{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // Small positive Tikhonov term for rank-deficient two-contact phases.
    double regularization = 1e-8;
};

struct ContactWrenchSolution
{
    std::array<go2::Vec3, go2::kLegCount> forces{};
    int active_contacts = 0;
    double residual_norm = std::numeric_limits<double>::infinity();
};

class ContactWrenchLeastNormAllocator
{
public:
    bool Solve(
        const ContactWrenchRequest &request,
        ContactWrenchSolution &solution) const
    {
        solution = ContactWrenchSolution{};
        if (!ValidateRequest(request))
            return false;

        const int active_contacts = CountActiveContacts(request);
        if (active_contacts < 2)
            return false;

        Eigen::Matrix<double, 6, Eigen::Dynamic> wrench_map(
            6, 3 * active_contacts);
        wrench_map.setZero();

        int active_index = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.contact[leg])
                continue;

            const go2::Vec3 &position = request.contact_positions_body[leg];
            wrench_map.block<3, 3>(0, 3 * active_index) =
                Eigen::Matrix3d::Identity();
            wrench_map.block<3, 3>(3, 3 * active_index) =
                Skew(position);
            ++active_index;
        }

        Eigen::Matrix<double, 6, 1> desired_wrench;
        for (int i = 0; i < 6; ++i)
            desired_wrench[i] = request.desired_wrench[i];

        Eigen::Matrix<double, 6, 6> gram =
            wrench_map * wrench_map.transpose();
        gram.diagonal().array() += request.regularization;
        const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> factor(gram);
        if (factor.info() != Eigen::Success)
            return false;

        const Eigen::Matrix<double, 6, 1> dual =
            factor.solve(desired_wrench);
        if (factor.info() != Eigen::Success || !dual.allFinite())
            return false;

        const Eigen::VectorXd force_vector =
            wrench_map.transpose() * dual;
        if (!force_vector.allFinite())
            return false;

        const Eigen::Matrix<double, 6, 1> residual =
            wrench_map * force_vector - desired_wrench;
        if (!residual.allFinite())
            return false;

        active_index = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!request.contact[leg])
                continue;
            solution.forces[leg] = {
                force_vector[3 * active_index + 0],
                force_vector[3 * active_index + 1],
                force_vector[3 * active_index + 2]};
            ++active_index;
        }

        solution.active_contacts = active_contacts;
        solution.residual_norm = residual.norm();
        return std::isfinite(solution.residual_norm);
    }

private:
    static Eigen::Matrix3d Skew(const go2::Vec3 &vector)
    {
        Eigen::Matrix3d matrix;
        matrix << 0.0, -vector.z, vector.y,
            vector.z, 0.0, -vector.x,
            -vector.y, vector.x, 0.0;
        return matrix;
    }

    static int CountActiveContacts(const ContactWrenchRequest &request)
    {
        int count = 0;
        for (bool active : request.contact)
            count += active ? 1 : 0;
        return count;
    }

    static bool ValidateRequest(const ContactWrenchRequest &request)
    {
        if (!std::isfinite(request.regularization) ||
            request.regularization <= 0.0 ||
            request.regularization > 1.0)
        {
            return false;
        }
        for (const go2::Vec3 &position : request.contact_positions_body)
        {
            if (!std::isfinite(position.x) ||
                !std::isfinite(position.y) ||
                !std::isfinite(position.z))
            {
                return false;
            }
        }
        for (double value : request.desired_wrench)
        {
            if (!std::isfinite(value))
                return false;
        }
        bool has_task_weight = false;
        for (double weight : request.task_weights)
        {
            if (!std::isfinite(weight) || weight < 0.0)
                return false;
            has_task_weight = has_task_weight || weight > 0.0;
        }
        if (!has_task_weight)
            return false;
        return true;
    }
};

} // namespace go2_control

