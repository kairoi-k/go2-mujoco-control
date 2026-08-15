#pragma once

// Dense convex QP: min 1/2 x^T H x + g^T x  s.t. A x <= b.
// ADMM, no third-party solver. Sized for contact forces and short MPC.

#include <Eigen/Dense>
#include <cmath>

namespace go2_control
{

struct DenseQpSettings
{
    int max_iterations = 200;
    double rho = 10.0;
    double abs_tol = 1e-8;
    double rel_tol = 1e-6;
    double feasibility_tol = 1e-6;
};

inline bool SolveDenseQp(
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &g,
    const Eigen::MatrixXd &A,
    const Eigen::VectorXd &b,
    Eigen::VectorXd &x,
    int &iterations,
    const DenseQpSettings &settings = {})
{
    iterations = 0;
    x.resize(0);
    const int n = static_cast<int>(H.rows());
    if (n <= 0 ||
        H.cols() != n ||
        g.size() != n ||
        settings.max_iterations <= 0 ||
        !(settings.rho > 0.0) ||
        !H.allFinite() ||
        !g.allFinite())
    {
        return false;
    }

    const int m = (A.size() == 0) ? 0 : static_cast<int>(A.rows());
    if (m < 0 ||
        (m > 0 && (A.cols() != n || b.size() != m ||
                   !A.allFinite() || !b.allFinite())))
    {
        return false;
    }

    const Eigen::LDLT<Eigen::MatrixXd> unconstrained(H);
    if (unconstrained.info() != Eigen::Success)
        return false;
    Eigen::VectorXd unconstrained_x = unconstrained.solve(-g);
    if (!unconstrained_x.allFinite())
        return false;

    if (m == 0)
    {
        x = unconstrained_x;
        return true;
    }

    const Eigen::VectorXd unconstrained_violation =
        (A * unconstrained_x - b).cwiseMax(0.0);
    if (unconstrained_violation.maxCoeff() <= settings.feasibility_tol)
    {
        x = unconstrained_x;
        return true;
    }

    Eigen::MatrixXd hessian = H;
    hessian.noalias() += settings.rho * A.transpose() * A;
    const Eigen::LDLT<Eigen::MatrixXd> factor(hessian);
    if (factor.info() != Eigen::Success)
        return false;

    x = unconstrained_x;
    Eigen::VectorXd z = (A * x).cwiseMin(b);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(m);
    for (int k = 0; k < settings.max_iterations; ++k)
    {
        const Eigen::VectorXd rhs =
            -g + settings.rho * A.transpose() * (z - u);
        x = factor.solve(rhs);
        if (!x.allFinite())
            return false;
        const Eigen::VectorXd Ax = A * x;
        const Eigen::VectorXd z_prev = z;
        z = (Ax + u).cwiseMin(b);
        u += Ax - z;
        ++iterations;
        const double primal = (Ax - z).norm();
        const double dual =
            settings.rho * (A.transpose() * (z - z_prev)).norm();
        const double eps_primal =
            settings.abs_tol * std::sqrt(static_cast<double>(m)) +
            settings.rel_tol * std::max(Ax.norm(), z.norm());
        const double eps_dual =
            settings.abs_tol * std::sqrt(static_cast<double>(n)) +
            settings.rel_tol * (A.transpose() * (settings.rho * u)).norm();
        if (primal <= eps_primal && dual <= eps_dual)
            return true;
    }

    const Eigen::VectorXd violation = (A * x - b).cwiseMax(0.0);
    return x.allFinite() &&
           violation.maxCoeff() <= settings.feasibility_tol * 10.0;
}

}  // namespace go2_control
