#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "go2_rigid_body.h"

#ifndef GO2_MODEL_PATH
#define GO2_MODEL_PATH "unitree_robots/go2/go2.xml"
#endif

namespace
{

bool Check(bool ok, const char *msg)
{
    if (!ok)
        std::cerr << msg << "\n";
    return ok;
}

Eigen::Vector3d MjVec3(const mjtNum *values, int index)
{
    return Eigen::Vector3d(
        values[3 * index + 0], values[3 * index + 1],
        values[3 * index + 2]);
}

Eigen::Matrix3d MjRotation(const mjtNum *values, int index)
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            rotation(row, col) = values[9 * index + 3 * row + col];
    return rotation;
}

bool SetMujocoState(
    const mjModel *model, mjData *data,
    const go2_control::RigidBodyState &state)
{
    if (model == nullptr || data == nullptr || model->nq != go2_control::kGo2Nq)
        return false;
    const Eigen::Quaterniond quaternion =
        state.quat_world_from_body.normalized();
    if (!quaternion.coeffs().allFinite())
        return false;
    mju_zero(data->qpos, model->nq);
    data->qpos[0] = state.position_world.x();
    data->qpos[1] = state.position_world.y();
    data->qpos[2] = state.position_world.z();
    data->qpos[3] = quaternion.w();
    data->qpos[4] = quaternion.x();
    data->qpos[5] = quaternion.y();
    data->qpos[6] = quaternion.z();
    for (int motor = 0; motor < static_cast<int>(go2::kJointCount); ++motor)
    {
        const int joint = mj_name2id(
            model, mjOBJ_JOINT, go2_control::Go2MotorJointName(motor));
        if (joint < 0)
            return false;
        data->qpos[model->jnt_qposadr[joint]] = state.q[motor];
    }
    mj_forward(model, data);
    return true;
}

std::string MetadataFixtureXml(bool include_sites, bool sphere_geom)
{
    static constexpr const char *kLegNames[go2::kLegCount] = {
        "FR", "FL", "RR", "RL"};
    std::string xml =
        "<mujoco model=\"go2_metadata_fixture\"><worldbody>"
        "<body name=\"base_link\" pos=\"0 0 0.4\"><freejoint/>"
        "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"1 1 1\"/>";
    for (const char *leg : kLegNames)
    {
        const std::string prefix = leg;
        xml += "<body name=\"" + prefix + "_hip\"><inertial pos=\"0 0 0\" mass=\"0.1\" diaginertia=\"0.001 0.001 0.001\"/><joint name=\"" +
            prefix + "_hip_joint\" type=\"hinge\" axis=\"1 0 0\"/>";
        xml += "<body name=\"" + prefix + "_thigh\" pos=\"0 0 -0.2\"><inertial pos=\"0 0 0\" mass=\"0.1\" diaginertia=\"0.001 0.001 0.001\"/><joint name=\"" +
            prefix + "_thigh_joint\" type=\"hinge\" axis=\"0 1 0\"/>";
        xml += "<body name=\"" + prefix + "_calf\" pos=\"0 0 -0.2\"><inertial pos=\"0 0 0\" mass=\"0.1\" diaginertia=\"0.001 0.001 0.001\"/><joint name=\"" +
            prefix + "_calf_joint\" type=\"hinge\" axis=\"0 1 0\"/>";
        if (sphere_geom)
            xml += "<geom name=\"" + prefix + "\" type=\"sphere\" size=\"0.022\"/>";
        else
            xml += "<geom name=\"" + prefix + "\" type=\"box\" size=\"0.02 0.02 0.02\"/>";
        if (include_sites)
            xml += "<site name=\"" + prefix + "_foot_contact\" pos=\"0 0 0\"/>";
        xml += "</body></body></body>";
    }
    xml += "</body></worldbody></mujoco>";
    return xml;
}

bool WriteMetadataFixture(
    const std::string &path, bool include_sites, bool sphere_geom)
{
    std::ofstream output(path);
    if (!output)
        return false;
    output << MetadataFixtureXml(include_sites, sphere_geom);
    return output.good();
}

bool CheckMetadataFixtureLoadable(const std::string &path)
{
    char error[1024] = {};
    mjModel *fixture_model = mj_loadXML(
        path.c_str(), nullptr, error, sizeof(error));
    if (fixture_model == nullptr)
    {
        std::cerr << "metadata fixture mj_loadXML failed: " << error << "\n";
        return false;
    }
    mj_deleteModel(fixture_model);
    return true;
}

std::string MakeMetadataFixturePath()
{
    char path[] = "/tmp/go2_rigid_body_metadata_XXXXXX";
    const int descriptor = mkstemp(path);
    if (descriptor < 0)
        return {};
    close(descriptor);
    return path;
}

bool CheckInvalidMetadataFixture(
    const std::string &path, bool expect_site_valid, bool expect_sphere_valid)
{
    go2_control::Go2RigidBody fixture;
    if (!fixture.Load(path))
        return false;
    bool passed = true;
    go2_control::RigidBodyState zero_state;
    zero_state.position_world = Eigen::Vector3d(0.0, 0.0, 0.4);
    zero_state.quat_world_from_body = Eigen::Quaterniond::Identity();
    zero_state.q.setZero();
    zero_state.dq.setZero();
    go2_control::RigidBodyDynamics dynamics;
    passed &= Check(fixture.Evaluate(zero_state, dynamics),
                    "metadata fixture dynamics evaluation");
    passed &= Check(dynamics.valid,
                    "metadata fixture dynamics became invalid");
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &metadata = fixture.FootGeometry()[leg];
        passed &= Check(metadata.site_id_valid == expect_site_valid,
                        "fixture site validity mismatch");
        passed &= Check(metadata.sphere_valid == expect_sphere_valid,
                        "fixture sphere validity mismatch");
        passed &= Check(!metadata.metadata_valid,
                        "invalid fixture metadata marked valid");
        passed &= Check(dynamics.foot_geom_center_valid[leg],
                        "metadata fixture geom center invalid");
        passed &= Check(dynamics.foot_site_valid[leg] == expect_site_valid,
                        "metadata fixture site observation mismatch");
        passed &= Check(!dynamics.foot_geometry_valid[leg],
                        "invalid fixture geometry observation marked valid");
    }
    return passed;
}

go2_control::RigidBodyState StandState()
{
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(0.0, 0.0, 0.42);
    state.quat_world_from_body = Eigen::Quaterniond::Identity();
    state.q << 0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763;
    state.dq.setZero();
    return state;
}

go2_control::RigidBodyState TiltedState()
{
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(0.31, -0.22, 0.63);
    state.quat_world_from_body =
        Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.16, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-0.09, Eigen::Vector3d::UnitX());
    state.q <<
        0.12, 0.55, -1.10,
        -0.08, 0.70, -1.25,
        0.15, 0.62, -1.16,
        -0.11, 0.66, -1.30;
    state.dq.setZero();
    return state;
}

}  // namespace

int main()
{
    go2_control::Go2RigidBody model;
    if (!model.Load(GO2_MODEL_PATH))
    {
        std::cerr << "Failed to load " << GO2_MODEL_PATH << "\n";
        return 1;
    }
    const auto &geometry = model.FootGeometry();
    bool passed = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const auto &metadata = geometry[leg];
        passed &= Check(metadata.geom_id_valid, "foot geom metadata missing");
        passed &= Check(metadata.site_id_valid, "foot site metadata missing");
        passed &= Check(metadata.sphere_valid, "foot sphere metadata missing");
        passed &= Check(metadata.metadata_valid, "foot geometry metadata invalid");
        passed &= Check(metadata.geom_body_id >= 0 && metadata.site_body_id >= 0,
                        "foot geometry body metadata missing");
        passed &= Check(metadata.geom_type == mjGEOM_SPHERE,
                        "foot geom is not a sphere");
        passed &= Check(
            std::abs(metadata.collision_radius_m - 0.022) < 1e-12,
            "foot sphere radius metadata");
        passed &= Check(
            (metadata.geom_pos_local - metadata.site_pos_local -
             Eigen::Vector3d(-0.002, 0.0, 0.0)).norm() < 1e-12,
            "foot geom/site local offset metadata");
    }
    go2_control::RigidBodyDynamics dyn;
    passed &= model.Evaluate(StandState(), dyn);
    passed &= Check(dyn.valid, "dynamics invalid");
    passed &= Check(
        std::abs(dyn.mass_kg - 15.206) < 0.2, "mass mismatch");
    passed &= Check(dyn.com_world.z() > 0.2 && dyn.com_world.z() < 0.5,
                    "com height");
    passed &= Check(
        dyn.mass_matrix.llt().info() == Eigen::Success, "M not SPD");
    // Gravity generalized force on floating z is ~+mg.
    passed &= Check(
        std::abs(dyn.bias[2] - dyn.mass_kg * 9.81) < 25.0,
        "bias_z is not mg");
    Eigen::Matrix<double, go2_control::kGo2Nv, 1> qacc =
        Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
    qacc[2] = 0.3;
    qacc[7] = -0.2;
    const double rne = model.InverseDynamicsResidual(dyn, qacc);
    passed &= Check(rne < 0.5, "RNEA residual");
    std::array<double, go2::kJointCount> joint_positions{};
    for (std::size_t joint = 0; joint < joint_positions.size(); ++joint)
        joint_positions[joint] = StandState().q[
            static_cast<Eigen::Index>(joint)];
    const auto fk_feet = go2::AllFootPositions(joint_positions);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        passed &= Check(dyn.foot_geom_center_valid[leg],
                        "foot geom center invalid");
        passed &= Check(dyn.foot_site_valid[leg], "foot site invalid");
        passed &= Check(dyn.foot_geometry_valid[leg],
                        "foot geometry observation invalid");
        passed &= Check(dyn.foot_pos_world[leg].z() > -0.01, "foot z");
        const Eigen::Vector3d expected(
            StandState().position_world.x() + fk_feet[leg].x,
            StandState().position_world.y() + fk_feet[leg].y,
            StandState().position_world.z() + fk_feet[leg].z);
        passed &= Check(
            (dyn.foot_site_world[leg] - expected).norm() < 1.0e-7,
            "MJCF contact site does not match analytical FK");
        passed &= Check(
            (dyn.foot_pos_world[leg] - expected).norm() < 0.003,
            "MJCF foot geom center is unexpectedly far from analytical FK");
    }

    const auto tilted_state = TiltedState();
    go2_control::RigidBodyDynamics tilted_dyn;
    passed &= Check(model.Evaluate(tilted_state, tilted_dyn),
                    "tilted dynamics evaluation");
    char error[1024] = {};
    mjModel *oracle_model = mj_loadXML(
        GO2_MODEL_PATH, nullptr, error, sizeof(error));
    mjData *oracle_data = oracle_model == nullptr
        ? nullptr : mj_makeData(oracle_model);
    passed &= Check(oracle_model != nullptr && oracle_data != nullptr,
                    "independent MuJoCo oracle load");
    const bool oracle_state_valid = oracle_model != nullptr &&
        oracle_data != nullptr &&
        SetMujocoState(oracle_model, oracle_data, tilted_state);
    passed &= Check(oracle_state_valid,
                    "independent MuJoCo oracle state");
    if (oracle_state_valid)
    {
        const int base_body = mj_name2id(
            oracle_model, mjOBJ_BODY, "base_link");
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &metadata = geometry[leg];
            const int geom = metadata.geom_id;
            const int site = metadata.site_id;
            passed &= Check(
                tilted_dyn.foot_geom_center_valid[leg] &&
                    tilted_dyn.foot_site_valid[leg] &&
                    tilted_dyn.foot_geometry_valid[leg],
                "tilted foot observation invalid");
            const Eigen::Vector3d oracle_geom = MjVec3(
                oracle_data->geom_xpos, geom);
            const Eigen::Vector3d oracle_site = MjVec3(
                oracle_data->site_xpos, site);
            passed &= Check(
                (tilted_dyn.foot_pos_world[leg] - oracle_geom).norm() < 1.0e-10,
                "geom center disagrees with independent mj_forward");
            passed &= Check(
                (tilted_dyn.foot_site_world[leg] - oracle_site).norm() < 1.0e-10,
                "contact site disagrees with independent mj_forward");
            passed &= Check(
                std::abs(metadata.collision_radius_m -
                         oracle_model->geom_size[3 * geom]) < 1.0e-12,
                "sphere radius disagrees with MJCF");
            passed &= Check(
                metadata.geom_body_id == oracle_model->geom_bodyid[geom] &&
                    metadata.site_body_id == oracle_model->site_bodyid[site],
                "geom/site body association disagrees with MJCF");
            passed &= Check(
                (metadata.geom_pos_local -
                 Eigen::Vector3d(
                     oracle_model->geom_pos[3 * geom + 0],
                     oracle_model->geom_pos[3 * geom + 1],
                     oracle_model->geom_pos[3 * geom + 2])).norm() < 1.0e-12 &&
                    (metadata.site_pos_local -
                     Eigen::Vector3d(
                         oracle_model->site_pos[3 * site + 0],
                         oracle_model->site_pos[3 * site + 1],
                         oracle_model->site_pos[3 * site + 2])).norm() < 1.0e-12,
                "local geom/site position disagrees with MJCF");

            const int geom_body = oracle_model->geom_bodyid[geom];
            const int site_body = oracle_model->site_bodyid[site];
            const Eigen::Vector3d world_delta = oracle_geom - oracle_site;
            if (geom_body == site_body)
            {
                passed &= Check(geom_body == site_body,
                                "local offset requires a shared parent body");
                const Eigen::Vector3d local_delta(
                    oracle_model->geom_pos[3 * geom + 0] -
                        oracle_model->site_pos[3 * site + 0],
                    oracle_model->geom_pos[3 * geom + 1] -
                        oracle_model->site_pos[3 * site + 1],
                    oracle_model->geom_pos[3 * geom + 2] -
                        oracle_model->site_pos[3 * site + 2]);
                const Eigen::Matrix3d calf_rotation = MjRotation(
                    oracle_data->xmat, geom_body);
                const Eigen::Matrix3d base_rotation = MjRotation(
                    oracle_data->xmat, base_body);
                passed &= Check(
                    (world_delta - calf_rotation * local_delta).norm() < 1.0e-10,
                    "geom/site offset did not use parent calf rotation");
                passed &= Check(
                    (calf_rotation * local_delta -
                     base_rotation * local_delta).norm() > 1.0e-7,
                    "test pose did not distinguish calf from base rotation");
            }
            else
            {
                // Different parent bodies have no shared local frame. The
                // world observations remain authoritative; no local
                // subtraction is attempted in that case.
                passed &= Check(
                    geom_body >= 0 && site_body >= 0,
                    "invalid differing geom/site body association");
            }
        }

        std::array<double, go2::kJointCount> tilted_joints{};
        for (std::size_t joint = 0; joint < tilted_joints.size(); ++joint)
            tilted_joints[joint] = tilted_state.q[
                static_cast<Eigen::Index>(joint)];
        const auto tilted_fk = go2::AllFootPositions(tilted_joints);
        const Eigen::Quaterniond body_quaternion =
            tilted_state.quat_world_from_body.normalized();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const Eigen::Vector3d fk_site =
                tilted_state.position_world + body_quaternion * Eigen::Vector3d(
                    tilted_fk[leg].x, tilted_fk[leg].y, tilted_fk[leg].z);
            passed &= Check(
                (tilted_dyn.foot_site_world[leg] - fk_site).norm() < 1.0e-7,
                "tilted contact site broke analytical FK compatibility");
        }
    }
    if (oracle_data != nullptr)
        mj_deleteData(oracle_data);
    if (oracle_model != nullptr)
        mj_deleteModel(oracle_model);

    const std::string missing_site_path = MakeMetadataFixturePath();
    const std::string nonsphere_path = MakeMetadataFixturePath();
    if (!missing_site_path.empty())
    {
        passed &= Check(
            WriteMetadataFixture(missing_site_path, false, true),
            "write missing-site metadata fixture");
        passed &= Check(
            CheckMetadataFixtureLoadable(missing_site_path),
            "load missing-site metadata fixture with MuJoCo");
        passed &= Check(
            CheckInvalidMetadataFixture(missing_site_path, false, true),
            "missing-site metadata fixture");
        std::remove(missing_site_path.c_str());
    }
    else
        passed &= Check(false, "create missing-site metadata fixture path");
    if (!nonsphere_path.empty())
    {
        passed &= Check(
            WriteMetadataFixture(nonsphere_path, true, false),
            "write non-sphere metadata fixture");
        passed &= Check(
            CheckMetadataFixtureLoadable(nonsphere_path),
            "load non-sphere metadata fixture with MuJoCo");
        passed &= Check(
            CheckInvalidMetadataFixture(nonsphere_path, true, false),
            "non-sphere metadata fixture");
        std::remove(nonsphere_path.c_str());
    }
    else
        passed &= Check(false, "create non-sphere metadata fixture path");

    if (!passed)
    {
        std::cerr << "mass=" << dyn.mass_kg
                  << " comz=" << dyn.com_world.z()
                  << " biasz=" << dyn.bias[2]
                  << " rne=" << rne << "\n";
        return 1;
    }
    std::cout << "go2 rigid body checks passed. mass=" << dyn.mass_kg
              << " rne=" << rne << "\n";
    return 0;
}
