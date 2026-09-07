#pragma once

// Controller-side 18-DoF Go2 model. Loads the same MJCF the simulator uses
// and evaluates M(q), h(q,qd), contact Jacobians, and CoM inertia from
// LowState / SportModeState. Independent of the packed LowState spare slots.

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "go2_forward_kinematics.h"

namespace go2_control
{

constexpr int kFloatingNv = 6;
constexpr int kGo2Nv = 18;
constexpr int kGo2Nq = 19;

inline const char *Go2MotorJointName(int motor)
{
    static constexpr const char *kNames[go2::kJointCount] = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    return (motor >= 0 && motor < static_cast<int>(go2::kJointCount))
        ? kNames[motor]
        : "";
}

inline const char *Go2FootGeomName(std::size_t leg)
{
    static constexpr const char *kNames[go2::kLegCount] = {
        "FR", "FL", "RR", "RL"};
    return kNames[leg];
}

inline const char *Go2FootSiteName(std::size_t leg)
{
    static constexpr const char *kNames[go2::kLegCount] = {
        "FR_foot_contact", "FL_foot_contact", "RR_foot_contact",
        "RL_foot_contact"};
    return kNames[leg];
}

struct RigidBodyState
{
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat_world_from_body = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear_vel_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel_body = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, go2::kJointCount, 1> q = {};
    Eigen::Matrix<double, go2::kJointCount, 1> dq = {};
};

// Immutable geometry metadata read from the loaded MJCF. The local positions
// retain their owning body IDs; they must not be subtracted when those IDs
// differ. Dynamic world positions are supplied by Evaluate() after mj_forward.
struct RigidBodyFootGeometry
{
    bool geom_id_valid = false;
    bool site_id_valid = false;
    bool sphere_valid = false;
    bool metadata_valid = false;
    int geom_id = -1;
    int site_id = -1;
    int geom_body_id = -1;
    int site_body_id = -1;
    int geom_type = -1;
    double collision_radius_m = 0.0;
    Eigen::Vector3d geom_pos_local = Eigen::Vector3d::Zero();
    Eigen::Vector3d site_pos_local = Eigen::Vector3d::Zero();
};

struct RigidBodyDynamics
{
    bool valid = false;
    double mass_kg = 0.0;
    Eigen::Vector3d com_world = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia_com_world = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, kGo2Nv, kGo2Nv> mass_matrix =
        Eigen::Matrix<double, kGo2Nv, kGo2Nv>::Zero();
    Eigen::Matrix<double, kGo2Nv, 1> bias =
        Eigen::Matrix<double, kGo2Nv, 1>::Zero();
    // Existing interface: the named MJCF foot geom center in world frame.
    std::array<Eigen::Vector3d, go2::kLegCount> foot_pos_world{};
    // The MJCF contact site position in the same evaluated world state.
    std::array<Eigen::Vector3d, go2::kLegCount> foot_site_world{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    std::array<bool, go2::kLegCount> foot_geom_center_valid{};
    std::array<bool, go2::kLegCount> foot_site_valid{};
    // True only when the named geom is a valid sphere and the named site is
    // available. Missing sites and non-sphere geoms remain explicitly false.
    std::array<bool, go2::kLegCount> foot_geometry_valid{};
    std::array<RigidBodyFootGeometry, go2::kLegCount> foot_geometry{
        RigidBodyFootGeometry{}, RigidBodyFootGeometry{},
        RigidBodyFootGeometry{}, RigidBodyFootGeometry{}};
    std::array<Eigen::Matrix<double, 3, kGo2Nv>, go2::kLegCount> foot_jac_world{};
    std::array<Eigen::Matrix<double, 3, kGo2Nv>, go2::kLegCount> foot_jac_dot_world{};
    Eigen::Matrix<double, kGo2Nv, 1> qvel =
        Eigen::Matrix<double, kGo2Nv, 1>::Zero();
};

class Go2RigidBody
{
public:
    Go2RigidBody() = default;
    Go2RigidBody(const Go2RigidBody &) = delete;
    Go2RigidBody &operator=(const Go2RigidBody &) = delete;

    ~Go2RigidBody()
    {
        Reset();
    }

    bool Load(const std::string &model_path)
    {
        Reset();
        char error[1024] = {};
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
        if (model_ == nullptr)
            return false;
        data_ = mj_makeData(model_);
        if (data_ == nullptr)
        {
            Reset();
            return false;
        }
        if (model_->nv != kGo2Nv || model_->nq != kGo2Nq)
        {
            Reset();
            return false;
        }
        base_body_ = mj_name2id(model_, mjOBJ_BODY, "base_link");
        if (base_body_ < 0)
        {
            Reset();
            return false;
        }
        for (int motor = 0; motor < static_cast<int>(go2::kJointCount); ++motor)
        {
            joint_id_[motor] = mj_name2id(
                model_, mjOBJ_JOINT, Go2MotorJointName(motor));
            if (joint_id_[motor] < 0)
            {
                Reset();
                return false;
            }
        }
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            foot_geom_[leg] = mj_name2id(
                model_, mjOBJ_GEOM, Go2FootGeomName(leg));
            if (foot_geom_[leg] < 0)
            {
                Reset();
                return false;
            }

            // Geometry metadata is an independent observation seam. The
            // existing dynamics path still requires the named geom above,
            // while an absent site or non-sphere geom only invalidates this
            // metadata, leaving Load() usable for existing dynamics callers.
            RigidBodyFootGeometry metadata;
            metadata.geom_id = foot_geom_[leg];
            metadata.geom_id_valid = true;
            metadata.geom_body_id = model_->geom_bodyid[foot_geom_[leg]];
            metadata.geom_type = static_cast<int>(
                model_->geom_type[foot_geom_[leg]]);
            metadata.geom_pos_local = Eigen::Vector3d(
                model_->geom_pos[3 * foot_geom_[leg] + 0],
                model_->geom_pos[3 * foot_geom_[leg] + 1],
                model_->geom_pos[3 * foot_geom_[leg] + 2]);
            if (metadata.geom_type == mjGEOM_SPHERE)
            {
                metadata.collision_radius_m =
                    model_->geom_size[3 * foot_geom_[leg]];
                metadata.sphere_valid = std::isfinite(
                    metadata.collision_radius_m) &&
                    metadata.collision_radius_m > 0.0;
            }
            metadata.site_id = mj_name2id(
                model_, mjOBJ_SITE, Go2FootSiteName(leg));
            if (metadata.site_id >= 0)
            {
                metadata.site_id_valid = true;
                metadata.site_body_id = model_->site_bodyid[metadata.site_id];
                metadata.site_pos_local = Eigen::Vector3d(
                    model_->site_pos[3 * metadata.site_id + 0],
                    model_->site_pos[3 * metadata.site_id + 1],
                    model_->site_pos[3 * metadata.site_id + 2]);
            }
            metadata.metadata_valid = metadata.geom_id_valid &&
                metadata.site_id_valid && metadata.sphere_valid &&
                metadata.geom_body_id >= 0 && metadata.site_body_id >= 0 &&
                metadata.geom_pos_local.allFinite() &&
                metadata.site_pos_local.allFinite();
            foot_geometry_[leg] = metadata;
        }
        loaded_ = true;
        return true;
    }

    bool loaded() const { return loaded_; }

    // No MuJoCo ownership or raw model pointer escapes this interface.
    const std::array<RigidBodyFootGeometry, go2::kLegCount> &FootGeometry() const
    {
        return foot_geometry_;
    }

    int MotorDof(int motor) const
    {
        if (!loaded_ || motor < 0 || motor >= static_cast<int>(go2::kJointCount))
            return -1;
        return model_->jnt_dofadr[joint_id_[motor]];
    }

    bool Evaluate(const RigidBodyState &state, RigidBodyDynamics &out)
    {
        out = RigidBodyDynamics{};
        if (!loaded_ || !SetState(state))
            return false;
        mj_forward(model_, data_);

        out.mass_kg = mj_getTotalmass(model_);
        out.com_world = Eigen::Vector3d(
            data_->subtree_com[3 * base_body_ + 0],
            data_->subtree_com[3 * base_body_ + 1],
            data_->subtree_com[3 * base_body_ + 2]);

        double dense_m[kGo2Nv * kGo2Nv];
        mj_fullM(model_, dense_m, data_->qM);
        out.mass_matrix = Eigen::Map<
            Eigen::Matrix<double, kGo2Nv, kGo2Nv, Eigen::RowMajor>>(dense_m);

        mju_zero(data_->qacc, model_->nv);
        mj_inverse(model_, data_);
        for (int i = 0; i < kGo2Nv; ++i)
            out.bias[i] = data_->qfrc_inverse[i];

        out.inertia_com_world = CompositeInertiaAboutCom(out.com_world);

        for (int i = 0; i < kGo2Nv; ++i)
            out.qvel[i] = data_->qvel[i];
        mjtNum jacp[3 * kGo2Nv];
        mjtNum jacr[3 * kGo2Nv];
        mjtNum jacp_dot[3 * kGo2Nv];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const int geom = foot_geom_[leg];
            const int body = model_->geom_bodyid[geom];
            out.foot_geometry[leg] = foot_geometry_[leg];
            // The named foot geom is offset from the calf body origin.  Its
            // position must match mj_jacGeom's point; using body xpos here
            // silently paired a calf-origin lever arm with a foot Jacobian.
            out.foot_pos_world[leg] = Eigen::Vector3d(
                data_->geom_xpos[3 * geom + 0],
                data_->geom_xpos[3 * geom + 1],
                data_->geom_xpos[3 * geom + 2]);
            out.foot_geom_center_valid[leg] =
                out.foot_pos_world[leg].allFinite();
            const int site = foot_geometry_[leg].site_id;
            if (site >= 0)
            {
                out.foot_site_world[leg] = Eigen::Vector3d(
                    data_->site_xpos[3 * site + 0],
                    data_->site_xpos[3 * site + 1],
                    data_->site_xpos[3 * site + 2]);
                out.foot_site_valid[leg] = out.foot_site_world[leg].allFinite();
            }
            out.foot_geometry_valid[leg] =
                foot_geometry_[leg].metadata_valid &&
                out.foot_geom_center_valid[leg] && out.foot_site_valid[leg];
            mj_jacGeom(model_, data_, jacp, jacr, geom);
            const mjtNum point[3] = {
                out.foot_pos_world[leg].x(),
                out.foot_pos_world[leg].y(),
                out.foot_pos_world[leg].z()};
            mj_jacDot(model_, data_, jacp_dot, nullptr, point, body);
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < kGo2Nv; ++col)
                {
                    out.foot_jac_world[leg](row, col) = jacp[row * kGo2Nv + col];
                    out.foot_jac_dot_world[leg](row, col) =
                        jacp_dot[row * kGo2Nv + col];
                }
            }
        }
        out.valid = out.mass_kg > 1.0 && out.mass_matrix.allFinite() &&
                    out.bias.allFinite();
        return out.valid;
    }

    // M qacc + h should match mj_inverse(qacc).
    double InverseDynamicsResidual(
        const RigidBodyDynamics &dyn,
        const Eigen::Matrix<double, kGo2Nv, 1> &qacc) const
    {
        if (!loaded_ || !dyn.valid)
            return std::numeric_limits<double>::infinity();
        for (int i = 0; i < kGo2Nv; ++i)
            data_->qacc[i] = qacc[i];
        mj_inverse(model_, data_);
        Eigen::Matrix<double, kGo2Nv, 1> predicted = dyn.mass_matrix * qacc + dyn.bias;
        double residual = 0.0;
        for (int i = 0; i < kGo2Nv; ++i)
        {
            const double err = predicted[i] - data_->qfrc_inverse[i];
            residual += err * err;
        }
        return std::sqrt(residual);
    }

private:
    bool SetState(const RigidBodyState &state)
    {
        if (!loaded_)
            return false;
        if (!state.position_world.allFinite() ||
            !state.linear_vel_world.allFinite() ||
            !state.angular_vel_body.allFinite() ||
            !state.q.allFinite() ||
            !state.dq.allFinite())
        {
            return false;
        }
        Eigen::Quaterniond quat = state.quat_world_from_body.normalized();
        if (quat.norm() < 0.5)
            return false;
        data_->qpos[0] = state.position_world.x();
        data_->qpos[1] = state.position_world.y();
        data_->qpos[2] = state.position_world.z();
        data_->qpos[3] = quat.w();
        data_->qpos[4] = quat.x();
        data_->qpos[5] = quat.y();
        data_->qpos[6] = quat.z();
        data_->qvel[0] = state.linear_vel_world.x();
        data_->qvel[1] = state.linear_vel_world.y();
        data_->qvel[2] = state.linear_vel_world.z();
        data_->qvel[3] = state.angular_vel_body.x();
        data_->qvel[4] = state.angular_vel_body.y();
        data_->qvel[5] = state.angular_vel_body.z();
        for (int motor = 0; motor < static_cast<int>(go2::kJointCount); ++motor)
        {
            const int jnt = joint_id_[motor];
            data_->qpos[model_->jnt_qposadr[jnt]] = state.q[motor];
            data_->qvel[model_->jnt_dofadr[jnt]] = state.dq[motor];
        }
        return true;
    }

    Eigen::Matrix3d CompositeInertiaAboutCom(const Eigen::Vector3d &com) const
    {
        Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
        for (int body = 1; body < model_->nbody; ++body)
        {
            const double mass = model_->body_mass[body];
            if (!(mass > 0.0))
                continue;
            const Eigen::Vector3d body_com(
                data_->xipos[3 * body + 0],
                data_->xipos[3 * body + 1],
                data_->xipos[3 * body + 2]);
            const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
                rot(data_->ximat + 9 * body);
            Eigen::Matrix3d I_body = Eigen::Matrix3d::Zero();
            I_body(0, 0) = model_->body_inertia[3 * body + 0];
            I_body(1, 1) = model_->body_inertia[3 * body + 1];
            I_body(2, 2) = model_->body_inertia[3 * body + 2];
            const Eigen::Matrix3d I_world = rot * I_body * rot.transpose();
            const Eigen::Vector3d r = body_com - com;
            inertia += I_world + mass * (r.squaredNorm() * Eigen::Matrix3d::Identity() -
                                         r * r.transpose());
        }
        return inertia;
    }

    void Reset()
    {
        if (data_ != nullptr)
            mj_deleteData(data_);
        if (model_ != nullptr)
            mj_deleteModel(model_);
        data_ = nullptr;
        model_ = nullptr;
        loaded_ = false;
        base_body_ = -1;
        joint_id_.fill(-1);
        foot_geom_.fill(-1);
        foot_geometry_.fill(RigidBodyFootGeometry{});
    }

    mjModel *model_ = nullptr;
    mjData *data_ = nullptr;
    bool loaded_ = false;
    int base_body_ = -1;
    std::array<int, go2::kJointCount> joint_id_{};
    std::array<int, go2::kLegCount> foot_geom_{};
    std::array<RigidBodyFootGeometry, go2::kLegCount> foot_geometry_{
        RigidBodyFootGeometry{}, RigidBodyFootGeometry{},
        RigidBodyFootGeometry{}, RigidBodyFootGeometry{}};
};

}  // namespace go2_control
