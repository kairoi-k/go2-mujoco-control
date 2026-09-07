#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>
#include "go2_rigid_body.h"
#ifndef GO2_MODEL_PATH
#define GO2_MODEL_PATH "unitree_robots/go2/go2.xml"
#endif
namespace
{
using Jacobian = Eigen::Matrix<double, 3, go2_control::kGo2Nv>;
using GeneralizedForce = Eigen::Matrix<double, go2_control::kGo2Nv, 1>;
using JacobianArray =
    std::array<Jacobian, go2::kLegCount>;
bool Check(bool ok, const char *message)
{
    if (!ok)
        std::cerr << message << "\n";
    return ok;
}
go2_control::RigidBodyState InclinedState()
{
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(0.17, -0.11, 0.49);
    state.quat_world_from_body =
        Eigen::AngleAxisd(0.31, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-0.21, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitX());
    state.q <<
        0.12, 0.55, -1.10,
        -0.08, 0.70, -1.25,
        0.15, 0.62, -1.16,
        -0.11, 0.66, -1.30;
    state.dq.setZero();
    return state;
}
bool SetOracleState(
    const mjModel *model, mjData *data,
    const go2_control::RigidBodyState &state)
{
    if (model == nullptr || data == nullptr ||
        model->nq != go2_control::kGo2Nq || model->nv != go2_control::kGo2Nv)
        return false;
    if (!state.position_world.allFinite() ||
        !state.quat_world_from_body.coeffs().allFinite() ||
        !state.q.allFinite() || !state.dq.allFinite())
        return false;
    Eigen::Quaterniond quat = state.quat_world_from_body;
    const double norm = quat.coeffs().stableNorm();
    if (!std::isfinite(norm) || !(norm > 1e-12))
        return false;
    quat.coeffs() /= norm;
    mju_zero(data->qpos, model->nq);
    mju_zero(data->qvel, model->nv);
    data->qpos[0] = state.position_world.x();
    data->qpos[1] = state.position_world.y();
    data->qpos[2] = state.position_world.z();
    data->qpos[3] = quat.w();
    data->qpos[4] = quat.x();
    data->qpos[5] = quat.y();
    data->qpos[6] = quat.z();
    data->qvel[0] = state.linear_vel_world.x();
    data->qvel[1] = state.linear_vel_world.y();
    data->qvel[2] = state.linear_vel_world.z();
    data->qvel[3] = state.angular_vel_body.x();
    data->qvel[4] = state.angular_vel_body.y();
    data->qvel[5] = state.angular_vel_body.z();
    for (int motor = 0;
         motor < static_cast<int>(go2::kJointCount); ++motor)
    {
        const int joint = mj_name2id(
            model, mjOBJ_JOINT, go2_control::Go2MotorJointName(motor));
        if (joint < 0)
            return false;
        data->qpos[model->jnt_qposadr[joint]] = state.q[motor];
        data->qvel[model->jnt_dofadr[joint]] = state.dq[motor];
    }
    mj_forward(model, data);
    return true;
}
Eigen::Vector3d MjPoint(const mjtNum *values, int index)
{
    return Eigen::Vector3d(
        values[3 * index + 0], values[3 * index + 1],
        values[3 * index + 2]);
}
Jacobian MjJacobian(const mjtNum *values)
{
    Jacobian result = Jacobian::Zero();
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < go2_control::kGo2Nv; ++col)
            result(row, col) = values[row * go2_control::kGo2Nv + col];
    return result;
}
}  // namespace
int main()
{
    go2_control::Go2RigidBody robot;
    if (!robot.Load(GO2_MODEL_PATH))
    {
        std::cerr << "Failed to load " << GO2_MODEL_PATH << "\n";
        return 1;
    }
    char error[1024] = {};
    mjModel *oracle_model = mj_loadXML(
        GO2_MODEL_PATH, nullptr, error, sizeof(error));
    if (oracle_model == nullptr)
    {
        std::cerr << "Independent MuJoCo load failed: " << error << "\n";
        return 1;
    }
    mjData *oracle_data = mj_makeData(oracle_model);
    if (oracle_data == nullptr)
    {
        mj_deleteModel(oracle_model);
        std::cerr << "Independent MuJoCo data allocation failed\n";
        return 1;
    }
    const go2_control::RigidBodyState state = InclinedState();
    bool passed = SetOracleState(oracle_model, oracle_data, state);
    passed &= Check(passed, "independent MuJoCo state setup");
    if (!passed)
    {
        mj_deleteData(oracle_data);
        mj_deleteModel(oracle_model);
        return 1;
    }
    JacobianArray centers;
    JacobianArray application_jacobians;
    std::array<Eigen::Vector3d, go2::kLegCount> center_points;
    std::array<Eigen::Vector3d, go2::kLegCount> application_points;
    for (auto &jacobian : centers)
        jacobian.setZero();
    for (auto &jacobian : application_jacobians)
        jacobian.setZero();
    for (auto &point : center_points)
        point.setZero();
    for (auto &point : application_points)
        point.setZero();
    const Eigen::Vector3d offset = 0.022 * Eigen::Vector3d::UnitZ();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const int geom = mj_name2id(
            oracle_model, mjOBJ_GEOM, go2_control::Go2FootGeomName(leg));
        passed &= Check(geom >= 0, "named foot geom missing in oracle");
        if (geom < 0)
            continue;
        center_points[leg] = MjPoint(oracle_data->geom_xpos, geom);
        application_points[leg] = center_points[leg] + offset;
    }
    passed &= Check(
        robot.EvaluateContactJacobians(state, center_points, centers),
        "center Jacobian evaluation");
    passed &= Check(
        robot.EvaluateContactJacobians(
            state, application_points, application_jacobians),
        "application-point Jacobian evaluation");
    const Eigen::Vector3d tangent_force(100.0, 0.0, 0.0);
    const Eigen::Vector3d normal_force(0.0, 0.0, 100.0);
    passed &= Check(
        std::abs(offset.cross(tangent_force).norm() - 2.2) < 1e-12,
        "22 mm tangential force shift is not 2.2 Nm");
    passed &= Check(
        offset.cross(normal_force).norm() < 1e-12,
        "normal force should have zero moment for radial offset");
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const int geom = mj_name2id(
            oracle_model, mjOBJ_GEOM, go2_control::Go2FootGeomName(leg));
        const int body = oracle_model->geom_bodyid[geom];
        const mjtNum point_c[3] = {
            center_points[leg].x(), center_points[leg].y(), center_points[leg].z()};
        const mjtNum point_p[3] = {
            application_points[leg].x(), application_points[leg].y(),
            application_points[leg].z()};
        mjtNum jacp_c[3 * go2_control::kGo2Nv] = {};
        mjtNum jacr_c[3 * go2_control::kGo2Nv] = {};
        mjtNum jacp_p[3 * go2_control::kGo2Nv] = {};
        mj_jacGeom(
            oracle_model, oracle_data, jacp_c, jacr_c, geom);
        mj_jac(
            oracle_model, oracle_data, jacp_p, nullptr, point_p, body);
        const Jacobian jacobian_c = MjJacobian(jacp_c);
        const Jacobian rotational_c = MjJacobian(jacr_c);
        const Jacobian jacobian_p = MjJacobian(jacp_p);
        passed &= Check(
            (centers[leg] - jacobian_c).norm() < 1e-12,
            "center Jacobian differs from independent mj_jacGeom");
        passed &= Check(
            (application_jacobians[leg] - jacobian_p).norm() < 1e-12,
            "application Jacobian differs from independent mj_jac");
        const Eigen::Vector3d r = application_points[leg] - center_points[leg];
        const GeneralizedForce tangent_lhs =
            (application_jacobians[leg].transpose() -
             centers[leg].transpose()) * tangent_force;
        const GeneralizedForce tangent_rhs =
            rotational_c.transpose() * r.cross(tangent_force);
        passed &= Check(
            (tangent_lhs - tangent_rhs).norm() < 1e-9,
            "tangential generalized force shift identity failed");
        passed &= Check(
            std::abs(tangent_rhs.segment<3>(3).norm() - 2.2) < 1e-9,
            "22 mm shift is not 2.2 Nm in base generalized torque");
        const GeneralizedForce normal_lhs =
            (application_jacobians[leg].transpose() -
             centers[leg].transpose()) * normal_force;
        const GeneralizedForce normal_rhs =
            rotational_c.transpose() * r.cross(normal_force);
        passed &= Check(
            (normal_lhs - normal_rhs).norm() < 1e-9 && normal_rhs.norm() < 1e-9,
            "radial normal generalized force shift is nonzero");
    }
    auto invalid_points = application_points;
    invalid_points[1].x() = std::numeric_limits<double>::quiet_NaN();
    for (auto &jacobian : application_jacobians)
        jacobian.setConstant(7.0);
    passed &= Check(
        !robot.EvaluateContactJacobians(state, invalid_points, application_jacobians),
        "nonfinite application point was accepted");
    for (const auto &jacobian : application_jacobians)
        passed &= Check(jacobian.isZero(0.0), "failed evaluation leaked Jacobian output");
    mj_deleteData(oracle_data);
    mj_deleteModel(oracle_model);
    if (!passed)
        return 1;
    std::cout << "go2 contact point Jacobian tests passed\n";
    return 0;
}
