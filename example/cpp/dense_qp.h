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

// Equalities stay in a regularized KKT system. ADMM only handles inequalities.
// Two-sided inequality encoding of Cx=d is too tight for 2-contact ID-WBC.
inline bool SolveDenseQpEq(
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &g,
    const Eigen::MatrixXd &Aineq,
    const Eigen::VectorXd &bineq,
    const Eigen::MatrixXd &Aeq,
    const Eigen::VectorXd &beq,
    Eigen::VectorXd &x,
    int &iterations,
    const DenseQpSettings &settings = {})
{
    iterations = 0;
    x.resize(0);
    const int n = static_cast<int>(H.rows());
    const int meq = (Aeq.size() == 0) ? 0 : static_cast<int>(Aeq.rows());
    const int mineq = (Aineq.size() == 0) ? 0 : static_cast<int>(Aineq.rows());
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
    if (meq > 0 && (Aeq.cols() != n || beq.size() != meq ||
                    !Aeq.allFinite() || !beq.allFinite()))
    {
        return false;
    }
    if (mineq > 0 && (Aineq.cols() != n || bineq.size() != mineq ||
                      !Aineq.allFinite() || !bineq.allFinite()))
    {
        return false;
    }
    if (meq == 0)
        return SolveDenseQp(H, g, Aineq, bineq, x, iterations, settings);

    const double rho = settings.rho;
    const double eq_reg = 1.0e-8;
    Eigen::MatrixXd h_rho = H;
    h_rho.diagonal().array() += 1.0e-10;
    if (mineq > 0)
        h_rho.noalias() += rho * Aineq.transpose() * Aineq;

    Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(n + meq, n + meq);
    kkt.topLeftCorner(n, n) = h_rho;
    kkt.topRightCorner(n, meq) = Aeq.transpose();
    kkt.bottomLeftCorner(meq, n) = Aeq;
    kkt.bottomRightCorner(meq, meq) =
        -eq_reg * Eigen::MatrixXd::Identity(meq, meq);
    const Eigen::LDLT<Eigen::MatrixXd> factor(kkt);
    if (factor.info() != Eigen::Success)
        return false;

    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + meq);
    rhs.head(n) = -g;
    rhs.tail(meq) = beq;
    Eigen::VectorXd xy = factor.solve(rhs);
    if (!xy.allFinite() || xy.size() != n + meq)
        return false;
    x = xy.head(n);

    if (mineq == 0)
    {
        iterations = 1;
        return (Aeq * x - beq).lpNorm<Eigen::Infinity>() <=
               settings.feasibility_tol;
    }

    Eigen::VectorXd z = (Aineq * x).cwiseMin(bineq);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(mineq);
    for (int k = 0; k < settings.max_iterations; ++k)
    {
        rhs.head(n) = -g + rho * Aineq.transpose() * (z - u);
        rhs.tail(meq) = beq;
        xy = factor.solve(rhs);
        if (!xy.allFinite())
            return false;
        x = xy.head(n);
        const Eigen::VectorXd ax = Aineq * x;
        const Eigen::VectorXd z_prev = z;
        z = (ax + u).cwiseMin(bineq);
        u += ax - z;
        ++iterations;
        const double primal = (ax - z).norm();
        const double dual =
            rho * (Aineq.transpose() * (z - z_prev)).norm();
        const double eps_primal =
            settings.abs_tol * std::sqrt(static_cast<double>(mineq)) +
            settings.rel_tol * std::max(ax.norm(), z.norm());
        const double eps_dual =
            settings.abs_tol * std::sqrt(static_cast<double>(n)) +
            settings.rel_tol * (Aineq.transpose() * (rho * u)).norm();
        const double eq_res = (Aeq * x - beq).lpNorm<Eigen::Infinity>();
        if (primal <= eps_primal &&
            dual <= eps_dual &&
            eq_res <= settings.feasibility_tol)
        {
            return true;
        }
    }

    const Eigen::VectorXd violation = (Aineq * x - bineq).cwiseMax(0.0);
    const double eq_res = (Aeq * x - beq).lpNorm<Eigen::Infinity>();
    return x.allFinite() &&
           violation.maxCoeff() <= std::max(0.05, settings.feasibility_tol * 50.0) &&
           eq_res <= settings.feasibility_tol * 10.0;
}

}  // namespace go2_control
