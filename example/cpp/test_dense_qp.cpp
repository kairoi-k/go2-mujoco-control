#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "dense_qp.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-6)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckUnconstrained()
{
    Eigen::MatrixXd H(1, 1);
    H(0, 0) = 1.0;
    Eigen::VectorXd g(1);
    g[0] = -2.0;
    Eigen::MatrixXd A(0, 1);
    Eigen::VectorXd b(0);
    Eigen::VectorXd x;
    int iterations = 0;
    return go2_control::SolveDenseQp(H, g, A, b, x, iterations) &&
           x.size() == 1 && Near(x[0], 2.0);
}

bool CheckBound()
{
    Eigen::MatrixXd H(1, 1);
    H(0, 0) = 1.0;
    Eigen::VectorXd g(1);
    g[0] = -2.0;
    Eigen::MatrixXd A(1, 1);
    A(0, 0) = 1.0;
    Eigen::VectorXd b(1);
    b[0] = 0.5;
    Eigen::VectorXd x;
    int iterations = 0;
    return go2_control::SolveDenseQp(H, g, A, b, x, iterations) &&
           x.size() == 1 && Near(x[0], 0.5, 1e-5);
}

bool CheckTwoSided()
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd A(1, 2);
    A << -1.0, 0.0;
    Eigen::VectorXd b(1);
    b[0] = -1.0;
    Eigen::VectorXd x;
    int iterations = 0;
    return go2_control::SolveDenseQp(H, g, A, b, x, iterations) &&
           Near(x[0], 1.0, 1e-5) && Near(x[1], 0.0, 1e-5);
}

}  // namespace

int main()
{
    if (!CheckUnconstrained() || !CheckBound() || !CheckTwoSided())
    {
        std::cerr << "dense QP checks failed\n";
        return 1;
    }
    std::cout << "dense QP checks passed.\n";
    return 0;
}
