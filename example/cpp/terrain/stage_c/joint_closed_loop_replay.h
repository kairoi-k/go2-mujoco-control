#pragma once
// Research-only same-MJCF closed-loop diagnostic. This owner deliberately has no
// execution authority: it initializes a private MuJoCo plant from one
// recorded RigidBodyState, solves the existing model WBC, applies one direct
// torque-only control per step, and writes diagnostics. It is not B1 evidence.
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>
#include "contact_state_filter.h"
#include "centroidal_subproblem.h"
#include "centroidal_wbc_task.h"
#include "foot_trajectory.h"
#include "go2_rigid_body.h"
#include "id_wbc_certificate.h"
#include "inverse_dynamics_wbc.h"
#include "joint_planning_shadow.h"
#include "motor_command_certificate.h"
auto constexpr kClosedLoopReplayDtS = 0.002;
namespace go2_terrain {
namespace stage_c {
struct JointClosedLoopReplayResult
{
    bool completed = false;
    bool model_match = false;
    std::size_t rows = 0;
    std::string failure = "not_run";
};
namespace joint_closed_loop_detail {
struct ModelOwner
{
    mjModel *model = nullptr;
    mjData *data = nullptr;
    ~ModelOwner()
    {
        if (data != nullptr)
            mj_deleteData(data);
        if (model != nullptr)
            mj_deleteModel(model);
    }
    ModelOwner() = default;
    ModelOwner(const ModelOwner &) = delete;
    ModelOwner &operator=(const ModelOwner &) = delete;
};
inline Eigen::Vector3d Vec(const go2::Vec3 &v)
{
    return {v.x, v.y, v.z};
}
inline go2::Vec3 Vec(const Eigen::Vector3d &v)
{
    return {v.x(), v.y(), v.z()};
}
inline Eigen::Vector3d NanVec()
{
    return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
}
inline bool Near(double a, double b, double absolute = 1.0e-10)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a - b) <= absolute * std::max({1.0, std::abs(a), std::abs(b)});
}
inline bool FiniteState(const go2_control::RigidBodyState &state)
{
    const double qnorm = state.quat_world_from_body.coeffs().norm();
    return state.position_world.allFinite() &&
        state.quat_world_from_body.coeffs().allFinite() &&
        std::isfinite(qnorm) && qnorm > 1.0e-12 &&
        state.linear_vel_world.allFinite() &&
        state.angular_vel_body.allFinite() && state.q.allFinite() &&
        state.dq.allFinite();
}
inline std::string ActuatorName(int motor)
{
    const char *joint = go2_control::Go2MotorJointName(motor);
    std::string name = joint == nullptr ? std::string() : std::string(joint);
    constexpr const char *suffix = "_joint";
    if (name.size() < 6 || name.compare(name.size() - 6, 6, suffix) != 0)
        return {};
    name.resize(name.size() - 6);
    return name;
}
inline bool IsDescendantOf(const mjModel &model, int body, int ancestor)
{
    if (body < 0 || body >= model.nbody || ancestor < 0 || ancestor >= model.nbody)
        return false;
    for (int current = body; current >= 0; current = model.body_parentid[current])
    {
        if (current == ancestor)
            return true;
        if (current == 0)
            break;
    }
    return false;
}
inline bool SameArray(const mjtNum *a, const mjtNum *b, int n, double tol)
{
    for (int i = 0; i < n; ++i)
        if (!Near(a[i], b[i], tol))
            return false;
    return true;
}
inline bool SameIntArray(const int *a, const int *b, int n)
{
    for (int i = 0; i < n; ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
inline bool ValidateRobotModels(
    const mjModel &scene, const mjModel &controller, std::string &failure)
{
    if (scene.nq != controller.nq || scene.nv != controller.nv ||
        scene.nu != controller.nu || scene.nq != 19 || scene.nv != 18 ||
        scene.nu != 12)
    {
        failure = "robot_shape_mismatch";
        return false;
    }
    if (!Near(mj_getTotalmass(&scene), mj_getTotalmass(&controller), 1.0e-9) ||
        !SameArray(scene.opt.gravity, controller.opt.gravity, 3, 1.0e-9))
    {
        failure = "robot_mass_or_gravity_mismatch";
        return false;
    }
    const int scene_base = mj_name2id(&scene, mjOBJ_BODY, "base_link");
    const int controller_base = mj_name2id(&controller, mjOBJ_BODY, "base_link");
    if (scene_base < 0 || controller_base < 0)
    {
        failure = "base_link_missing";
        return false;
    }
    // Compare every named controller robot descendant. This avoids a partial
    // hard-coded body list being reported as a full-model match.
    std::vector<bool> matched_scene_body(static_cast<std::size_t>(scene.nbody), false);
    for (int c = 0; c < controller.nbody; ++c)
    {
        if (!IsDescendantOf(controller, c, controller_base))
            continue;
        const char *name = mj_id2name(&controller, mjOBJ_BODY, c);
        const int s = name == nullptr ? -1 : mj_name2id(&scene, mjOBJ_BODY, name);
        if (s < 0 || !IsDescendantOf(scene, s, scene_base) ||
            !Near(scene.body_mass[s], controller.body_mass[c], 1.0e-9) ||
            !SameArray(scene.body_inertia + 3 * s,
                       controller.body_inertia + 3 * c, 3, 1.0e-9) ||
            !SameArray(scene.body_pos + 3 * s, controller.body_pos + 3 * c, 3, 1.0e-9) ||
            !SameArray(scene.body_quat + 4 * s, controller.body_quat + 4 * c, 4, 1.0e-9) ||
            !SameArray(scene.body_ipos + 3 * s, controller.body_ipos + 3 * c, 3, 1.0e-9) ||
            !SameArray(scene.body_iquat + 4 * s, controller.body_iquat + 4 * c, 4, 1.0e-9))
        {
            failure = "robot_body_layout_mismatch";
            return false;
        }
        const int sp = scene.body_parentid[s];
        const int cp = controller.body_parentid[c];
        const char *spn = sp >= 0 ? mj_id2name(&scene, mjOBJ_BODY, sp) : nullptr;
        const char *cpn = cp >= 0 ? mj_id2name(&controller, mjOBJ_BODY, cp) : nullptr;
        if ((spn == nullptr) != (cpn == nullptr) ||
            (spn != nullptr && std::string(spn) != std::string(cpn)))
        {
            failure = "robot_body_parent_mismatch";
            return false;
        }
        matched_scene_body[static_cast<std::size_t>(s)] = true;
    }
    for (int s = 0; s < scene.nbody; ++s)
    {
        if (s == 0 || !IsDescendantOf(scene, s, scene_base))
            continue;
        const char *name = mj_id2name(&scene, mjOBJ_BODY, s);
        if (!matched_scene_body[static_cast<std::size_t>(s)] || name == nullptr)
        {
            failure = "scene_extra_robot_body";
            return false;
        }
    }
    // Compare every named robot joint, including axes and inertial properties.
    for (int c = 0; c < controller.njnt; ++c)
    {
        if (!IsDescendantOf(controller, controller.jnt_bodyid[c], controller_base))
            continue;
        const char *name = mj_id2name(&controller, mjOBJ_JOINT, c);
        int s = name == nullptr ? -1 : mj_name2id(&scene, mjOBJ_JOINT, name);
        // The canonical MJCF free joint is intentionally unnamed. Match it
        // by its unique base body and type, retaining exact qpos/dof checks.
        if (name == nullptr && controller.jnt_type[c] == mjJNT_FREE &&
            controller.jnt_bodyid[c] == controller_base)
            for (int j=0;j<scene.njnt;++j)
                if (scene.jnt_type[j]==mjJNT_FREE && scene.jnt_bodyid[j]==scene_base) {
                    if (s>=0) { failure="ambiguous_base_free_joint"; return false; }
                    s=j;
                }
        if (s < 0 || scene.jnt_bodyid[s] < 0 ||
            mj_id2name(&scene, mjOBJ_BODY, scene.jnt_bodyid[s]) == nullptr ||
            std::string(mj_id2name(&scene, mjOBJ_BODY, scene.jnt_bodyid[s])) !=
                std::string(mj_id2name(&controller, mjOBJ_BODY, controller.jnt_bodyid[c])) ||
            scene.jnt_type[s] != controller.jnt_type[c] ||
            scene.jnt_qposadr[s] != controller.jnt_qposadr[c] ||
            scene.jnt_dofadr[s] != controller.jnt_dofadr[c] ||
            !SameArray(scene.jnt_pos + 3 * s, controller.jnt_pos + 3 * c, 3, 1.0e-9) ||
            !SameArray(scene.jnt_axis + 3 * s, controller.jnt_axis + 3 * c, 3, 1.0e-9) ||
            !Near(scene.dof_armature[scene.jnt_dofadr[s]], controller.dof_armature[controller.jnt_dofadr[c]], 1.0e-9) ||
            !Near(scene.dof_damping[scene.jnt_dofadr[s]], controller.dof_damping[controller.jnt_dofadr[c]], 1.0e-9) ||
            !Near(scene.dof_frictionloss[scene.jnt_dofadr[s]], controller.dof_frictionloss[controller.jnt_dofadr[c]], 1.0e-9) ||
            !SameArray(scene.jnt_range + 2 * s, controller.jnt_range + 2 * c, 2, 1.0e-9))
        {
            failure = "robot_joint_layout_mismatch";
            return false;
        }
    }
    for (int c = 0; c < controller.nu; ++c)
    {
        const char *name = mj_id2name(&controller, mjOBJ_ACTUATOR, c);
        const int s = name == nullptr ? -1 : mj_name2id(&scene, mjOBJ_ACTUATOR, name);
        if (name == nullptr || std::string(name) != ActuatorName(c))
        { failure = "controller_actuator_order_mismatch"; return false; }
        if (s != c || controller.actuator_trntype[c] != scene.actuator_trntype[s] ||
            controller.actuator_dyntype[c] != scene.actuator_dyntype[s] ||
            controller.actuator_gaintype[c] != scene.actuator_gaintype[s] ||
            controller.actuator_biastype[c] != scene.actuator_biastype[s] ||
            !SameIntArray(scene.actuator_trnid + 2 * s, controller.actuator_trnid + 2 * c, 2) ||
            !SameArray(scene.actuator_gear + 6 * s, controller.actuator_gear + 6 * c, 6, 1.0e-9) ||
            !SameArray(scene.actuator_gainprm + mjNGAIN * s,
                       controller.actuator_gainprm + mjNGAIN * c, mjNGAIN, 1.0e-9) ||
            !SameArray(scene.actuator_biasprm + mjNBIAS * s,
                       controller.actuator_biasprm + mjNBIAS * c, mjNBIAS, 1.0e-9) ||
            !SameArray(scene.actuator_ctrlrange + 2 * s, controller.actuator_ctrlrange + 2 * c, 2, 1.0e-9))
        {
            failure = "motor_layout_or_envelope_mismatch";
            return false;
        }
    }
    return true;
}

inline int FindBaseFreeJoint(const mjModel &model)
{
    const int base = mj_name2id(&model, mjOBJ_BODY, "base_link");
    if (base < 0)
        return -1;
    for (int joint = 0; joint < model.njnt; ++joint)
        if (model.jnt_bodyid[joint] == base &&
            model.jnt_type[joint] == mjJNT_FREE)
            return joint;
    return -1;
}
inline bool WriteStateToPlant(
    const mjModel &model, mjData &data,
    const go2_control::RigidBodyState &state, std::string &failure)
{
    if (!FiniteState(state))
    {
        failure = "initial_state_nonfinite";
        return false;
    }
    const int free_joint = FindBaseFreeJoint(model);
    if (free_joint < 0)
    {
        failure = "free_base_missing";
        return false;
    }
    const int qa = model.jnt_qposadr[free_joint];
    const int va = model.jnt_dofadr[free_joint];
    if (qa < 0 || qa + 6 >= model.nq || va < 0 || va + 5 >= model.nv)
    {
        failure = "free_base_address_invalid";
        return false;
    }
    // Match Go2RigidBody::SetState: normalize the quaternion representation,
    // preserving its rotation and every physical q/dq. DDS float rounding
    // produces nonunit norms even for the recorded valid robot orientation.
    Eigen::Quaterniond quat=state.quat_world_from_body;
    quat.coeffs()/=quat.coeffs().stableNorm();
    mj_resetData(&model, &data);
    data.qpos[qa + 0] = state.position_world.x();
    data.qpos[qa + 1] = state.position_world.y();
    data.qpos[qa + 2] = state.position_world.z();
    data.qpos[qa + 3] = quat.w();
    data.qpos[qa + 4] = quat.x();
    data.qpos[qa + 5] = quat.y();
    data.qpos[qa + 6] = quat.z();
    data.qvel[va + 0] = state.linear_vel_world.x();
    data.qvel[va + 1] = state.linear_vel_world.y();
    data.qvel[va + 2] = state.linear_vel_world.z();
    data.qvel[va + 3] = state.angular_vel_body.x();
    data.qvel[va + 4] = state.angular_vel_body.y();
    data.qvel[va + 5] = state.angular_vel_body.z();
    for (int motor = 0; motor < static_cast<int>(go2::kJointCount); ++motor)
    {
        const int joint = mj_name2id(
            &model, mjOBJ_JOINT, go2_control::Go2MotorJointName(motor));
        if (joint < 0)
        {
            failure = "initial_motor_joint_missing";
            return false;
        }
        data.qpos[model.jnt_qposadr[joint]] = state.q[motor];
        data.qvel[model.jnt_dofadr[joint]] = state.dq[motor];
    }
    mj_forward(&model, &data);
    if (!std::all_of(data.qpos, data.qpos + model.nq,
                     [](mjtNum x) { return std::isfinite(x); }) ||
        !std::all_of(data.qvel, data.qvel + model.nv,
                     [](mjtNum x) { return std::isfinite(x); }))
    {
        failure = "initial_model_state_nonfinite";
        return false;
    }
    return true;
}
inline go2_control::RigidBodyState StateFromPlant(
    const mjModel &model, const mjData &data)
{
    go2_control::RigidBodyState state;
    const int free_joint = FindBaseFreeJoint(model);
    if (free_joint < 0)
        return state;
    const int qa = model.jnt_qposadr[free_joint];
    const int va = model.jnt_dofadr[free_joint];
    state.position_world = Eigen::Vector3d(
        data.qpos[qa + 0], data.qpos[qa + 1], data.qpos[qa + 2]);
    state.quat_world_from_body = Eigen::Quaterniond(
        data.qpos[qa + 3], data.qpos[qa + 4], data.qpos[qa + 5],
        data.qpos[qa + 6]);
    state.linear_vel_world = Eigen::Vector3d(
        data.qvel[va + 0], data.qvel[va + 1], data.qvel[va + 2]);
    state.angular_vel_body = Eigen::Vector3d(
        data.qvel[va + 3], data.qvel[va + 4], data.qvel[va + 5]);
    for (int motor = 0; motor < static_cast<int>(go2::kJointCount); ++motor)
    {
        const int joint = mj_name2id(
            &model, mjOBJ_JOINT, go2_control::Go2MotorJointName(motor));
        if (joint < 0)
            continue;
        state.q[motor] = data.qpos[model.jnt_qposadr[joint]];
        state.dq[motor] = data.qvel[model.jnt_dofadr[joint]];
    }
    return state;
}
inline bool ValidateSceneSensors(const mjModel &model, std::string &failure)
{
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const std::string name = std::string(
            go2_control::Go2FootGeomName(leg)) + "_foot_force_3d";
        const int sensor = mj_name2id(&model, mjOBJ_SENSOR, name.c_str());
        if (sensor < 0 || model.sensor_dim[sensor] != 3)
        {
            failure = "foot_force_sensor_missing";
            return false;
        }
        const int geom = mj_name2id(
            &model, mjOBJ_GEOM, go2_control::Go2FootGeomName(leg));
        const int site = mj_name2id(
            &model, mjOBJ_SITE, go2_control::Go2FootSiteName(leg));
        if (geom < 0 || site < 0)
        {
            failure = "foot_geom_or_site_missing";
            return false;
        }
    }
    return true;
}
struct ContactObservation
{
    std::array<bool, go2::kLegCount> geom_contact{};
    std::array<bool, go2::kLegCount> sensor_contact{};
    std::array<Eigen::Vector3d, go2::kLegCount> force_world{};
    Eigen::Vector3d total_force_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_moment_about_com_world = Eigen::Vector3d::Zero();
    // MuJoCo contact torque is retained separately. total_moment_world is
    // only the lever-arm moment of contact forces; it must not be compared
    // with angular momentum change while omitting this local couple.
    Eigen::Vector3d total_couple_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_contact_wrench_moment_world = Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d, go2::kLegCount> couple_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> local_force{};
    std::array<Eigen::Vector3d, go2::kLegCount> local_couple{};
    int nonfoot_contact_count = 0;
    double nonfoot_contact_force_n = 0.0;
    int ncon = 0;
    ContactObservation()
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            force_world[leg].setZero();
            couple_world[leg].setZero();
            local_force[leg].setZero();
            local_couple[leg].setZero();
        }
    }
};
inline Eigen::Vector3d ContactForceWorld(
    const mjContact &contact, const mjtNum *force, double sign)
{
    const double x = contact.frame[0] * force[0] +
        contact.frame[3] * force[1] + contact.frame[6] * force[2];
    const double y = contact.frame[1] * force[0] +
        contact.frame[4] * force[1] + contact.frame[7] * force[2];
    const double z = contact.frame[2] * force[0] +
        contact.frame[5] * force[1] + contact.frame[8] * force[2];
    return sign * Eigen::Vector3d(x, y, z);
}
inline Eigen::Vector3d ContactCoupleWorld(
    const mjContact &contact, const mjtNum *torque, double sign)
{
    return ContactForceWorld(contact, torque, sign);
}
inline bool ReadContactObservation(
    const mjModel &model, const mjData &data, const std::array<bool, 4> &previous,
    std::array<bool, 4> &next, ContactObservation &out, std::string &failure)
{
    out = ContactObservation{};
    out.ncon = data.ncon;
    const int base = mj_name2id(&model, mjOBJ_BODY, "base_link");
    if (base < 0)
    {
        failure = "contact_base_missing";
        return false;
    }
    std::array<int, go2::kLegCount> foot_geom{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        foot_geom[leg] = mj_name2id(
            &model, mjOBJ_GEOM, go2_control::Go2FootGeomName(leg));
    for (int contact_id = 0; contact_id < data.ncon; ++contact_id)
    {
        const mjContact &contact = data.contact[contact_id];
        if (contact.exclude != 0 || contact.efc_address < 0 ||
            contact.geom[0] < 0 || contact.geom[1] < 0 ||
            contact.geom[0] >= model.ngeom || contact.geom[1] >= model.ngeom)
            continue;
        const int body0 = model.geom_bodyid[contact.geom[0]];
        const int body1 = model.geom_bodyid[contact.geom[1]];
        const bool robot0 = IsDescendantOf(model, body0, base);
        const bool robot1 = IsDescendantOf(model, body1, base);
        if (robot0 == robot1)
            continue;
        const bool robot_is_geom0 = robot0;
        const int robot_geom = robot_is_geom0 ? contact.geom[0] : contact.geom[1];
        const double sign = robot_is_geom0 ? -1.0 : 1.0;
        mjtNum force[6] = {};
        mj_contactForce(&model, &data, contact_id, force);
        const Eigen::Vector3d local_force(force[0], force[1], force[2]);
        const Eigen::Vector3d local_couple(force[3], force[4], force[5]);
        const Eigen::Vector3d world_force = ContactForceWorld(contact, force, sign);
        const Eigen::Vector3d world_couple = ContactCoupleWorld(contact, force + 3, sign);
        if (!local_force.allFinite() || !local_couple.allFinite() ||
            !world_force.allFinite() || !world_couple.allFinite())
        {
            failure = "contact_wrench_nonfinite";
            return false;
        }
        out.total_force_world += world_force;
        out.total_couple_world += world_couple;
        const Eigen::Vector3d contact_position(
            contact.pos[0], contact.pos[1], contact.pos[2]);
        const Eigen::Vector3d com_origin(
            data.subtree_com[3 * base + 0],
            data.subtree_com[3 * base + 1],
            data.subtree_com[3 * base + 2]);
        if (!com_origin.allFinite())
        {
            failure = "plant_com_nonfinite";
            return false;
        }
        out.total_moment_about_com_world +=
            (contact_position - com_origin).cross(world_force);
        out.total_contact_wrench_moment_world +=
            (contact_position - com_origin).cross(world_force) + world_couple;
        bool foot = false;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (robot_geom != foot_geom[leg])
                continue;
            out.geom_contact[leg] = true;
            out.force_world[leg] += world_force;
            out.couple_world[leg] += world_couple;
            out.local_force[leg] += local_force;
            out.local_couple[leg] += local_couple;
            foot = true;
            break;
        }
        if (!foot)
        {
            ++out.nonfoot_contact_count;
            out.nonfoot_contact_force_n += world_force.norm();
        }
    }
    if (!out.total_force_world.allFinite() ||
        !out.total_moment_about_com_world.allFinite() ||
        !out.total_couple_world.allFinite() ||
        !out.total_contact_wrench_moment_world.allFinite() ||
        !std::isfinite(out.nonfoot_contact_force_n))
    {
        failure = "contact_observation_nonfinite";
        return false;
    }
    next = previous;
    const go2_control::HystereticContactParams contact_params{5.0, 3.0};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const std::string name = std::string(
            go2_control::Go2FootGeomName(leg)) + "_foot_force_3d";
        const int sensor = mj_name2id(&model, mjOBJ_SENSOR, name.c_str());
        const int site = mj_name2id(
            &model, mjOBJ_SITE, go2_control::Go2FootSiteName(leg));
        if (sensor < 0 || site < 0 || model.sensor_dim[sensor] != 3)
        {
            failure = "contact_sensor_metadata_invalid";
            return false;
        }
        const mjtNum *raw = data.sensordata + model.sensor_adr[sensor];
        const mjtNum *site_mat = data.site_xmat + 9 * site;
        const Eigen::Vector3d sensor_force(
            site_mat[0] * raw[0] + site_mat[1] * raw[1] + site_mat[2] * raw[2],
            site_mat[3] * raw[0] + site_mat[4] * raw[1] + site_mat[5] * raw[2],
            site_mat[6] * raw[0] + site_mat[7] * raw[1] + site_mat[8] * raw[2]);
        if (!sensor_force.allFinite())
        {
            failure = "sensor_force_nonfinite";
            return false;
        }
        bool current = false;
        if (!go2_control::UpdateHystereticContact(
                previous[leg], sensor_force.norm(), contact_params, current))
        {
            failure = "sensor_contact_filter_invalid";
            return false;
        }
        next[leg] = current;
        out.sensor_contact[leg] = current;
    }
    return true;
}
inline const FixedScheduleInterval *FindSchedule(
    const CentroidalProblem &problem, TimeNs time)
{
    for (const auto &interval : problem.schedule)
        if (interval.start <= time && time < interval.end)
            return &interval;
    return nullptr;
}
inline bool ValidReplaySurface(
    const ContactSurface &surface, std::uint64_t epoch, TimeNs end)
{
    return surface.frame == Frame::kWorld &&
        surface.coverage == MapCoverageState::kKnown &&
        surface.map_epoch == epoch && surface.valid_until >= end &&
        surface.basis_world.allFinite() &&
        (surface.basis_world.transpose() * surface.basis_world -
            Eigen::Matrix3d::Identity()).norm() <= 1.0e-8 &&
        std::abs(surface.basis_world.determinant() - 1.0) <= 1.0e-8 &&
        std::isfinite(surface.friction_mu) && surface.friction_mu >= 0.0 &&
        std::isfinite(surface.min_normal_n) && surface.min_normal_n >= 0.0 &&
        std::isfinite(surface.max_normal_n) &&
        surface.max_normal_n >= surface.min_normal_n;
}
inline bool ResolveScheduleSurface(
    const CentroidalProblem &problem, const FixedScheduleInterval &interval,
    std::size_t leg, TimeNs end, Eigen::Vector3d &point,
    ContactSurface &surface)
{
    if (!interval.contact[leg])
    {
        point = Eigen::Vector3d::Zero();
        surface = ContactSurface{};
        return true;
    }
    const int event = interval.event_index[leg];
    if (event < 0)
    {
        if (event != -1 || !problem.request.input.feet[leg].measured_support_anchor_valid)
            return false;
        point = Vec(problem.request.input.feet[leg].measured_support_anchor_world.value);
        surface = problem.initial_surfaces[leg];
    }
    else
    {
        const std::size_t e = static_cast<std::size_t>(event);
        if (e >= problem.combination.size() ||
            e >= problem.request.candidate_sets.size())
            return false;
        const std::size_t candidate_index = problem.combination[e];
        const auto &set = problem.request.candidate_sets[e];
        if (candidate_index >= set.candidates.size())
            return false;
        point = Vec(set.candidates[candidate_index].target_world.value);
        if (!problem.candidate_surfaces.empty())
        {
            if (e >= problem.candidate_surfaces.size() ||
                candidate_index >= problem.candidate_surfaces[e].size())
                return false;
            surface = problem.candidate_surfaces[e][candidate_index];
        }
        else
        {
            if (e >= problem.event_surfaces.size())
                return false;
            surface = problem.event_surfaces[e];
        }
    }
    return point.allFinite() && ValidReplaySurface(
        surface, problem.request.input.identity.map_epoch, end);
}
inline double WrapAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
inline Eigen::Vector3d Rpy(const Eigen::Quaterniond &q)
{
    const Eigen::Matrix3d r = q.toRotationMatrix();
    return Eigen::Vector3d(
        std::atan2(r(2, 1), r(2, 2)),
        std::asin(std::clamp(-r(2, 0), -1.0, 1.0)),
        std::atan2(r(1, 0), r(0, 0)));
}
struct ReplayRow
{
    std::size_t step = 0;
    bool terminal = false;
    std::string status = "not_run";
    double plant_time_s = std::numeric_limits<double>::quiet_NaN();
    double source_time_s = std::numeric_limits<double>::quiet_NaN();
    double reference_time_s = std::numeric_limits<double>::quiet_NaN();
    int nominal_contact_mask = 0;
    int recorded_source_measured_mask = 0;
    int model_sensor_contact_mask = 0;
    int mujoco_geom_contact_mask = 0;
    int wbc_contact_mask = 0;
    int ncon = -1;
    int nonfoot_contact_count = -1;
    double nonfoot_contact_force_n = std::numeric_limits<double>::quiet_NaN();
    double com_error_m = std::numeric_limits<double>::quiet_NaN();
    double com_velocity_error_mps = std::numeric_limits<double>::quiet_NaN();
    double momentum_error_nms = std::numeric_limits<double>::quiet_NaN();
    double pitch_error_rad = std::numeric_limits<double>::quiet_NaN();
    double pitch_rate_radps = std::numeric_limits<double>::quiet_NaN();
    double eq_residual = std::numeric_limits<double>::quiet_NaN();
    double rne_residual = std::numeric_limits<double>::quiet_NaN();
    double certificate_force_residual_n = std::numeric_limits<double>::quiet_NaN();
    double certificate_moment_residual_nm = std::numeric_limits<double>::quiet_NaN();
    double certificate_joint_residual_nm = std::numeric_limits<double>::quiet_NaN();
    double max_tau_violation_nm = std::numeric_limits<double>::quiet_NaN();
    double max_tau_nm = std::numeric_limits<double>::quiet_NaN();
    double max_motor_saturation_nm = std::numeric_limits<double>::quiet_NaN();
    double gt_force_norm_n = std::numeric_limits<double>::quiet_NaN();
    double gt_force_moment_about_com_norm_nm = std::numeric_limits<double>::quiet_NaN();
    double gt_contact_couple_norm_nm = std::numeric_limits<double>::quiet_NaN();
    double gt_wrench_moment_norm_nm = std::numeric_limits<double>::quiet_NaN();
    double reference_force_norm_n = std::numeric_limits<double>::quiet_NaN();
    double wbc_force_norm_n = std::numeric_limits<double>::quiet_NaN();
    double desired_orientation_acc_norm_radps2 = std::numeric_limits<double>::quiet_NaN();
    bool actual_state_finite = false;
    bool reference_state_valid = false;
    bool foot_reference_valid = false;
    double solve_us = std::numeric_limits<double>::quiet_NaN();
    int qp_iterations = 0;
    bool solver_returned = false;
    bool solver_converged = false;
    bool independent_certificate_valid = false;
    bool motor_envelope_valid = false;
    std::size_t foot_feedback_clipped_count = 0;
    bool orientation_feedback_clipped = false;
    bool cold_start = true;
    bool original_warmstart_replayed = false;
    Eigen::Vector3d base_position = NanVec();
    Eigen::Quaterniond base_quaternion =
        Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d com_world = NanVec();
    Eigen::Vector3d com_reference_world = NanVec();
    Eigen::Vector3d com_velocity_world = NanVec();
    Eigen::Vector3d com_velocity_reference_world = NanVec();
    Eigen::Vector3d momentum_world = NanVec();
    Eigen::Vector3d momentum_reference_world = NanVec();
    Eigen::Vector3d rpy = NanVec();
    Eigen::Vector3d desired_orientation_acc_body = NanVec();
    std::array<Eigen::Vector3d, go2::kLegCount> foot_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> foot_reference_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> foot_error_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> foot_velocity_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> foot_velocity_reference_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> gt_force_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> gt_local_force{};
    std::array<Eigen::Vector3d, go2::kLegCount> gt_couple_world{};
    std::array<Eigen::Vector3d, go2::kLegCount> gt_local_couple{};
    std::array<double, go2::kJointCount> q{};
    std::array<double, go2::kJointCount> dq{};
    std::array<double, go2::kJointCount> wbc_tau{};
    std::array<double, go2::kJointCount> requested_tau{};
    std::array<double, go2::kJointCount> applied_tau{};
    ReplayRow()
    {
        for (auto *v : {&foot_world, &foot_reference_world,
                        &foot_error_world, &foot_velocity_world,
                        &foot_velocity_reference_world, &gt_force_world,
                        &gt_local_force, &gt_couple_world, &gt_local_couple})
            for (auto &x : *v)
                x = NanVec();
        q.fill(std::numeric_limits<double>::quiet_NaN());
        dq.fill(std::numeric_limits<double>::quiet_NaN());
        wbc_tau.fill(std::numeric_limits<double>::quiet_NaN());
        requested_tau.fill(std::numeric_limits<double>::quiet_NaN());
        applied_tau.fill(std::numeric_limits<double>::quiet_NaN());
    }
};
inline int ContactMask(const std::array<bool, go2::kLegCount> &mask)
{
    int value = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (mask[leg])
            value |= 1 << static_cast<int>(leg);
    return value;
}
inline std::string CsvDouble(double value)
{
    if (!std::isfinite(value))
        return "nan";
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}
inline void CsvVec(std::ostream &out, const Eigen::Vector3d &value)
{
    out << CsvDouble(value.x()) << ',' << CsvDouble(value.y()) << ','
        << CsvDouble(value.z());
}
inline void CsvBool(std::ostream &out, bool value) { out << (value ? 1 : 0); }
inline void WriteReplayHeader(std::ostream &out)
{
    out << "step,terminal,status,plant_time_s,source_time_s,reference_time_s,"
           "nominal_contact_mask,recorded_source_measured_mask,"
           "model_sensor_contact_mask,mujoco_geom_contact_mask,wbc_contact_mask,"
           "ncon,nonfoot_contact_count,nonfoot_contact_force_n,"
           "actual_state_finite,reference_state_valid,foot_reference_valid,"
           "solve_us,qp_iterations,solver_returned,solver_converged,independent_certificate_valid,"
           "motor_envelope_valid,foot_feedback_clipped_count,orientation_feedback_clipped,"
           "cold_start,original_warmstart_replayed,"
           "base_px,base_py,base_pz,base_qw,base_qx,base_qy,base_qz,"
           "com_x,com_y,com_z,ref_com_x,ref_com_y,ref_com_z,"
           "com_vx,com_vy,com_vz,ref_com_vx,ref_com_vy,ref_com_vz,"
           "momentum_x,momentum_y,momentum_z,ref_momentum_x,ref_momentum_y,"
           "ref_momentum_z,roll_rad,pitch_rad,yaw_rad,pitch_error_rad,"
           "pitch_rate_radps,desired_ori_acc_x,desired_ori_acc_y,"
           "desired_ori_acc_z,desired_orientation_acc_norm_radps2,"
           "com_error_m,com_velocity_error_mps,momentum_error_nms,"
           "eq_residual,rne_residual,certificate_force_residual_n,"
           "certificate_moment_residual_nm,certificate_joint_residual_nm,"
           "max_tau_violation_nm,max_tau_nm,"
           "max_motor_saturation_nm,gt_force_norm_n,gt_force_moment_about_com_norm_nm,"
           "gt_contact_couple_norm_nm,gt_wrench_moment_norm_nm,"
           "reference_force_norm_n,wbc_force_norm_n";
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        out << ",foot" << leg << "_x,foot" << leg << "_y,foot" << leg
            << "_z,ref_foot" << leg << "_x,ref_foot" << leg << "_y,ref_foot"
            << leg << "_z,foot_error" << leg << "_x,foot_error" << leg
            << "_y,foot_error" << leg << "_z,foot_v" << leg << "_x,foot_v"
            << leg << "_y,foot_v" << leg << "_z,ref_foot_v" << leg
            << "_x,ref_foot_v" << leg << "_y,ref_foot_v" << leg << "_z,"
               "gt_force" << leg << "_x,gt_force" << leg << "_y,gt_force"
            << leg << "_z,gt_local_force" << leg << "_x,gt_local_force" << leg
            << "_y,gt_local_force" << leg << "_z,gt_couple" << leg << "_x,"
               "gt_couple" << leg << "_y,gt_couple" << leg << "_z,"
               "gt_local_couple" << leg << "_x,gt_local_couple" << leg
            << "_y,gt_local_couple" << leg << "_z";
    for (std::size_t j = 0; j < go2::kJointCount; ++j)
        out << ",q" << j << ",dq" << j << ",wbc_tau" << j
            << ",requested_tau" << j << ",applied_tau" << j;
    out << '\n';
}
inline void WriteReplayRow(std::ostream &out, const ReplayRow &r)
{
    out << r.step << ',' << (r.terminal ? 1 : 0) << ',' << r.status << ','
        << CsvDouble(r.plant_time_s) << ',' << CsvDouble(r.source_time_s) << ','
        << CsvDouble(r.reference_time_s) << ',' << r.nominal_contact_mask << ','
        << r.recorded_source_measured_mask << ',' << r.model_sensor_contact_mask
        << ',' << r.mujoco_geom_contact_mask << ',' << r.wbc_contact_mask << ','
        << r.ncon << ',' << r.nonfoot_contact_count << ','
        << CsvDouble(r.nonfoot_contact_force_n) << ',';
    CsvBool(out, r.actual_state_finite); out << ',';
    CsvBool(out, r.reference_state_valid); out << ',';
    CsvBool(out, r.foot_reference_valid); out << ',';
    out<<CsvDouble(r.solve_us)<<','<<r.qp_iterations<<',';
    CsvBool(out, r.solver_returned); out << ',';
    CsvBool(out, r.solver_converged); out << ',';
    CsvBool(out, r.independent_certificate_valid); out << ',';
    CsvBool(out, r.motor_envelope_valid); out << ',';
    out << r.foot_feedback_clipped_count << ',';
    CsvBool(out, r.orientation_feedback_clipped); out << ',';
    CsvBool(out, r.cold_start); out << ',';
    CsvBool(out, r.original_warmstart_replayed); out << ',';
    CsvVec(out, r.base_position); out << ',';
    out << CsvDouble(r.base_quaternion.w()) << ',' << CsvDouble(r.base_quaternion.x())
        << ',' << CsvDouble(r.base_quaternion.y()) << ','
        << CsvDouble(r.base_quaternion.z()) << ',';
    CsvVec(out, r.com_world); out << ','; CsvVec(out, r.com_reference_world); out << ',';
    CsvVec(out, r.com_velocity_world); out << ','; CsvVec(out, r.com_velocity_reference_world); out << ',';
    CsvVec(out, r.momentum_world); out << ','; CsvVec(out, r.momentum_reference_world); out << ',';
    CsvVec(out, r.rpy); out << ',' << CsvDouble(r.pitch_error_rad) << ','
        << CsvDouble(r.pitch_rate_radps) << ',';
    CsvVec(out, r.desired_orientation_acc_body); out << ','
        << CsvDouble(r.desired_orientation_acc_norm_radps2) << ','
        << CsvDouble(r.com_error_m) << ',' << CsvDouble(r.com_velocity_error_mps)
        << ',' << CsvDouble(r.momentum_error_nms) << ','
        << CsvDouble(r.eq_residual) << ',' << CsvDouble(r.rne_residual) << ','
        << CsvDouble(r.certificate_force_residual_n) << ','
        << CsvDouble(r.certificate_moment_residual_nm) << ','
        << CsvDouble(r.certificate_joint_residual_nm) << ','
        << CsvDouble(r.max_tau_violation_nm) << ',' << CsvDouble(r.max_tau_nm) << ','
        << CsvDouble(r.max_motor_saturation_nm) << ',' << CsvDouble(r.gt_force_norm_n)
        << ',' << CsvDouble(r.gt_force_moment_about_com_norm_nm) << ','
        << CsvDouble(r.gt_contact_couple_norm_nm) << ','
        << CsvDouble(r.gt_wrench_moment_norm_nm) << ','
        << CsvDouble(r.reference_force_norm_n) << ',' << CsvDouble(r.wbc_force_norm_n);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        out << ','; CsvVec(out, r.foot_world[leg]); out << ',';
        CsvVec(out, r.foot_reference_world[leg]); out << ',';
        CsvVec(out, r.foot_error_world[leg]); out << ',';
        CsvVec(out, r.foot_velocity_world[leg]); out << ',';
        CsvVec(out, r.foot_velocity_reference_world[leg]); out << ',';
        CsvVec(out, r.gt_force_world[leg]); out << ',';
        CsvVec(out, r.gt_local_force[leg]); out << ',';
        CsvVec(out, r.gt_couple_world[leg]); out << ',';
        CsvVec(out, r.gt_local_couple[leg]);
    }
    for (std::size_t j = 0; j < go2::kJointCount; ++j)
        out << ',' << CsvDouble(r.q[j]) << ',' << CsvDouble(r.dq[j]) << ','
            << CsvDouble(r.wbc_tau[j]) << ',' << CsvDouble(r.requested_tau[j])
            << ',' << CsvDouble(r.applied_tau[j]);
    out << '\n';
}
struct ClosedLoopResearchConfig
{
    // These are research replay settings, recorded in the sidecar metadata by
    // the caller. They are not B1 thresholds and do not change the planner.
    double swing_clearance_m = 0.03;
    double com_kp_xy = 18.0;
    double com_kp_z = 24.0;
    double com_kd_xy = 7.0;
    double com_kd_z = 8.0;
    double momentum_kp = 3.0;
    double foot_kp = 80.0;
    double foot_kd = 8.0;
    double foot_feedback_component_limit_mps2 = 25.0;
    double orientation_kp_roll = 20.0;
    double orientation_kp_pitch = 16.0;
    double orientation_kp_yaw = 8.0;
    double orientation_kd_roll = 3.0;
    double orientation_kd_pitch = 2.5;
    double orientation_kd_yaw = 1.5;
    double orientation_acc_limit_radps2 = 8.0;
    double torque_limit_nm = 35.0;
    double force_track_weight = 1.0e-4;
    double centroidal_weight = 1.0;
    double momentum_weight = 4.0;
};
inline bool ValidClosedLoopConfig(const ClosedLoopResearchConfig &c)
{
    const std::array<double, 19> values = {
        c.swing_clearance_m, c.com_kp_xy, c.com_kp_z, c.com_kd_xy,
        c.com_kd_z, c.momentum_kp, c.foot_kp, c.foot_kd,
        c.foot_feedback_component_limit_mps2, c.orientation_kp_roll,
        c.orientation_kp_pitch, c.orientation_kp_yaw, c.orientation_kd_roll,
        c.orientation_kd_pitch, c.orientation_kd_yaw,
        c.orientation_acc_limit_radps2, c.torque_limit_nm,
        c.force_track_weight,
        c.centroidal_weight};
    for (double value : values)
        if (!std::isfinite(value) || value < 0.0)
            return false;
    return std::isfinite(c.momentum_weight) && c.momentum_weight >= 0.0;
}
inline bool ValidateProposalForReplay(
    const CentroidalJointProposal &proposal,
    const go2_control::RigidBodyState &state, double duration_s,
    std::string &failure)
{
    if (!proposal.selected_valid || !proposal.search.feasible)
    {
        failure = "proposal_not_valid";
        return false;
    }
    if (!FiniteState(state))
    {
        failure = "source_state_nonfinite";
        return false;
    }
    if (!std::isfinite(duration_s) || duration_s < 0.15 || duration_s > 0.25)
    {
        failure = "duration_out_of_diagnostic_range";
        return false;
    }
    const auto &p = proposal.selected_problem;
    const auto &r = proposal.selected_result;
    const auto &input = p.request.input;
    if (!input.basic_valid() || !input.identity.valid() ||
        p.schedule.empty() || p.grid.size() < 2 ||
        r.states.size() != p.grid.size() ||
        r.forces.size() + 1 != p.grid.size() ||
        !r.certificate.feasible || !r.certificate.input_checked ||
        !r.certificate.original_dynamics_checked)
    {
        failure = "proposal_shape_or_certificate_invalid";
        return false;
    }
    if (p.grid.front() != input.identity.source_state_time ||
        p.required_start != p.grid.front() ||
        p.grid.back().value < p.grid.front().value +
            TimeNs::FromSeconds(duration_s).value)
    {
        failure = "proposal_grid_does_not_cover_replay";
        return false;
    }
    const auto horizon = TimeNs{p.grid.front().value +
        TimeNs::FromSeconds(duration_s).value};
    if (horizon <= p.grid.front() || horizon > p.grid.back())
    {
        failure = "replay_end_outside_source_coverage";
        return false;
    }
    for (std::size_t k = 1; k < p.grid.size(); ++k)
        if (p.grid[k] <= p.grid[k - 1])
        {
            failure = "proposal_grid_nonmonotonic";
            return false;
        }
    return true;
}
inline bool CopyReplaySchedule(
    const CentroidalProblem &source, TimeNs start, TimeNs end,
    CentroidalProblem &copy, std::string &failure)
{
    copy = source;
    copy.schedule.clear();
    TimeNs cursor = start;
    for (const auto &interval : source.schedule)
    {
        if (interval.end <= start)
            continue;
        if (interval.start > cursor || interval.start < start ||
            interval.end <= interval.start || interval.start != cursor)
        {
            failure = "source_schedule_gap_or_overlap";
            return false;
        }
        if (interval.start >= end)
            break;
        if (interval.end > end)
        {
            FixedScheduleInterval clipped = interval;
            clipped.end = end;
            copy.schedule.push_back(clipped);
            cursor = end;
            break;
        }
        copy.schedule.push_back(interval);
        cursor = interval.end;
        if (cursor == end)
            break;
    }
    if (cursor != end || copy.schedule.empty())
    {
        failure = "replay_schedule_not_covered";
        return false;
    }
    copy.required_end = end;
    // Only the foot-reference schedule is restricted. The solved dynamics
    // grid and states remain immutable; the sampler supports interior times.
    return true;
}
inline bool BuildFootReplayRequest(
    const CentroidalProblem &problem, go2_control::Go2RigidBody &robot,
    const go2_control::RigidBodyState &state, TimeNs end,
    const ClosedLoopResearchConfig &config, FootTrajectoryRequest &request,
    std::array<Eigen::Vector3d, go2::kLegCount> &actual_velocity,
    std::string &failure)
{
    go2_control::RigidBodyDynamics dyn;
    if (!robot.Evaluate(state, dyn) || !dyn.valid)
    {
        failure = "controller_dynamics_unavailable";
        return false;
    }
    request = FootTrajectoryRequest{};
    request.problem = &problem;
    request.start = problem.request.input.identity.source_state_time;
    request.end = end;
    request.swing_clearance_m = config.swing_clearance_m;
    request.allow_surface_contact_tail_beyond_horizon = true;
    request.add_clearance_to_inflight_continuation = false;
    const auto *source_interval = FindSchedule(problem, request.start);
    if (source_interval == nullptr)
    {
        failure = "source_schedule_interval_missing";
        return false;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!dyn.foot_geometry_valid[leg] ||
            !dyn.foot_pos_world[leg].allFinite())
        {
            failure = "controller_foot_geometry_invalid";
            return false;
        }
        const auto &point = problem.request.input.feet[leg].foot_collision_center_world;
        if (!TimedPointValidAt(point, PointRole::kFootCollisionCenter,
                               Frame::kWorld, request.start))
        {
            failure = "source_foot_observation_invalid";
            return false;
        }
        const Eigen::Vector3d source_center = Vec(point.value);
        if ((source_center - dyn.foot_pos_world[leg]).norm() > 1.0e-6)
        {
            failure = "source_foot_observation_model_mismatch";
            return false;
        }
        const double radius = dyn.foot_geometry[leg].collision_radius_m;
        request.collision_radius_m[leg] = radius;
        request.collision_radius_valid[leg] =
            std::isfinite(radius) && radius > 0.0;
        actual_velocity[leg] = dyn.foot_jac_world[leg] * dyn.qvel;
        if (!actual_velocity[leg].allFinite())
        {
            failure = "source_foot_velocity_nonfinite";
            return false;
        }
        request.initial_velocity_world[leg] = Vec(actual_velocity[leg]);
        request.initial_velocity_valid[leg] = true;
        bool in_flight = false;
        // Swing schedule intervals intentionally carry event_index=-1. Scan
        // the authoritative event table by leg to preserve a nonzero actual
        // initial velocity when the source timestamp lies inside a swing.
        for (const auto &event : problem.request.events.events)
            if (static_cast<std::size_t>(event.id.leg) == leg &&
                event.liftoff_valid && event.liftoff_time < request.start &&
                request.start < event.touchdown_time)
                in_flight = true;
        // A moving stance foot is measured plant state and stays untouched in
        // mjData. The zero below is only a nominal stationary stance reference;
        // nonzero measured velocity is retained in actual_velocity telemetry.
        if (!in_flight)
            request.initial_velocity_world[leg] = go2::Vec3{0.0, 0.0, 0.0};
    }
    return true;
}
inline bool BuildCentroidalReference(
    const CentroidalProblem &problem, const CentroidalResult &result,
    TimeNs time, const go2_control::RigidBodyPlanningKinematics &actual,
    const ClosedLoopResearchConfig &config, Eigen::Matrix<double, 6, 1> &desired,
    Eigen::Matrix<double, 6, 1> &weights, ContactForceInterval &planned_force,
    CentroidalState &planned_state, std::string &failure)
{
    const auto sample = SampleCentroidalTrajectory(problem, result, time);
    if (!sample.state_valid || !sample.force_valid ||
        !sample.state.allFinite() || !actual.valid || !actual.dynamics.valid)
    {
        failure = "centroidal_reference_unavailable";
        return false;
    }
    planned_state = sample.state;
    planned_force = sample.force;
    if (planned_force.start > time || planned_force.end <= time)
    {
        failure = "centroidal_force_interval_mismatch";
        return false;
    }
    const auto *interval = FindSchedule(problem, time);
    if (interval == nullptr)
    {
        failure = "centroidal_schedule_missing";
        return false;
    }
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_moment = Eigen::Vector3d::Zero();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d force = Vec(planned_force.force_world[leg]);
        if (!force.allFinite() || planned_force.contact[leg] != interval->contact[leg] ||
            (!planned_force.contact[leg] && force.norm() > 1.0e-9))
        {
            failure = "centroidal_force_mask_mismatch";
            return false;
        }
        if (!planned_force.contact[leg])
            continue;
        Eigen::Vector3d point;
        ContactSurface surface;
        if (!ResolveScheduleSurface(problem, *interval, leg, planned_force.end,
                                    point, surface))
        {
            failure = "centroidal_surface_unavailable";
            return false;
        }
        total_force += force;
        total_moment += (point - planned_state.head<3>()).cross(force);
    }
    if (!std::isfinite(problem.model.mass_kg) || problem.model.mass_kg <= 0.0 ||
        !std::isfinite(problem.model.gravity_mps2) ||
        !actual.dynamics.com_world.allFinite() ||
        !actual.com_velocity_world.allFinite() ||
        !actual.angular_momentum_world.allFinite())
    {
        failure = "centroidal_physical_reference_nonfinite";
        return false;
    }
    const Eigen::Vector3d com_error = planned_state.head<3>() - actual.dynamics.com_world;
    const Eigen::Vector3d velocity_error =
        planned_state.segment<3>(3) - actual.com_velocity_world;
    const Eigen::Vector3d momentum_error =
        planned_state.tail<3>() - actual.angular_momentum_world;
    Eigen::Vector3d com_acc = Eigen::Vector3d(0.0, 0.0,
        -problem.model.gravity_mps2) + total_force / problem.model.mass_kg;
    com_acc.x() += config.com_kp_xy * com_error.x() + config.com_kd_xy * velocity_error.x();
    com_acc.y() += config.com_kp_xy * com_error.y() + config.com_kd_xy * velocity_error.y();
    com_acc.z() += config.com_kp_z * com_error.z() + config.com_kd_z * velocity_error.z();
    desired.head<3>() = com_acc;
    desired.tail<3>() = total_moment + config.momentum_kp * momentum_error;
    weights << config.centroidal_weight, config.centroidal_weight,
        config.centroidal_weight, config.momentum_weight,
        config.momentum_weight, config.momentum_weight;
    return desired.allFinite() && weights.allFinite();
}

inline Eigen::Vector3d ClampComponentwise(
    const Eigen::Vector3d &value, double limit, bool &clipped)
{
    clipped = false;
    if (!value.allFinite() || !std::isfinite(limit) || limit < 0.0)
        return NanVec();
    Eigen::Vector3d out = value;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double old = out[axis];
        out[axis] = std::clamp(old, -limit, limit);
        clipped = clipped || old != out[axis];
    }
    return out;
}
inline bool BuildWbcReplayInput(
    const CentroidalProblem &problem,
    go2_control::Go2RigidBody &robot,
    const go2_control::RigidBodyState &state,
    const go2_control::RigidBodyPlanningKinematics &actual,
    const FootTrajectorySample &feet,
    const ContactForceInterval &planned_force,
    const Eigen::Matrix<double, 6, 1> &desired,
    const Eigen::Matrix<double, 6, 1> &weights,
    const FixedScheduleInterval &nominal,
    const ContactObservation &observation,
    double initial_yaw, const ClosedLoopResearchConfig &config,
    go2_control::IdWbcParams &params, go2_control::IdWbcInput &input,
    std::array<Eigen::Vector3d, go2::kLegCount> &application_points,
    std::size_t &feedback_clipped_count, Eigen::Vector3d &orientation_acc,
    bool &orientation_clipped, std::string &failure)
{
    if (!actual.valid || !actual.dynamics.valid || !feet.valid ||
        !desired.allFinite() || !weights.allFinite() ||
        planned_force.start > feet.time || feet.time >= planned_force.end)
    {
        failure = "actual_or_reference_sample_invalid";
        return false;
    }
    if (!SetCentroidalWbcTask(robot, state, desired, weights, input))
    {
        failure = "centroidal_task_setup_failed";
        return false;
    }
    const auto *interval = FindSchedule(problem, feet.time);
    if (interval == nullptr || interval->start != nominal.start ||
        interval->end != nominal.end || interval->contact != nominal.contact ||
        interval->event_index != nominal.event_index)
    {
        failure = "wbc_schedule_interval_mismatch";
        return false;
    }
    input.contact = go2_control::MergeRunningTrotContact(
        nominal.contact, observation.sensor_contact, 0);
    input.measured_contact = observation.sensor_contact;
    input.measured_contact_valid = true;
    input.fused_contact = input.contact;
    input.fused_contact_valid = true;
    input.planned_contact = nominal.contact;
    input.planned_contact_valid = true;
    input.have_stance_acc = true;
    input.have_force_ref = true;
    input.contact_normal.fill(Eigen::Vector3d::Zero());
    input.contact_normal_valid.fill(false);
    input.swing_acc_world.fill(Eigen::Vector3d::Zero());
    input.stance_acc_world.fill(Eigen::Vector3d::Zero());
    input.force_ref.setZero();
    feedback_clipped_count = 0;
    params = go2_control::IdWbcParams{};
    params.tau_limit_nm = config.torque_limit_nm;
    params.use_primal_active_set = true;
    params.prioritize_body_and_stance = true;
    params.w_force_track = config.force_track_weight;
    params.min_normal_n = 0.0;
    params.max_normal_n = std::numeric_limits<double>::infinity();
    params.friction_mu = std::numeric_limits<double>::infinity();
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d ref_position = Vec(feet.center_world[leg].value);
        const Eigen::Vector3d ref_velocity = Vec(feet.velocity_world[leg]);
        const Eigen::Vector3d actual_position = actual.dynamics.foot_pos_world[leg];
        const Eigen::Vector3d actual_velocity =
            actual.dynamics.foot_jac_world[leg] * actual.dynamics.qvel;
        if (!feet.leg_valid[leg] ||
            !TimedPointValidForRole(feet.center_world[leg],
                PointRole::kFootCollisionCenter, Frame::kWorld) ||
            !ref_position.allFinite() ||
            !ref_velocity.allFinite() || !Vec(feet.acceleration_world[leg]).allFinite() ||
            !actual_position.allFinite() || !actual_velocity.allFinite())
        {
            failure = "foot_reference_or_actual_invalid";
            return false;
        }
        bool clipped = false;
        const Eigen::Vector3d feedback = ClampComponentwise(
            config.foot_kp * (ref_position - actual_position) +
            config.foot_kd * (ref_velocity - actual_velocity),
            config.foot_feedback_component_limit_mps2, clipped);
        if (!feedback.allFinite())
        {
            failure = "foot_feedback_nonfinite";
            return false;
        }
        feedback_clipped_count += clipped ? 1u : 0u;
        input.swing_acc_world[leg] = Vec(feet.acceleration_world[leg]) + feedback;
        input.stance_acc_world[leg] = input.swing_acc_world[leg];
        const Eigen::Vector3d force = Vec(planned_force.force_world[leg]);
        if (!force.allFinite() || planned_force.contact[leg] != nominal.contact[leg] ||
            (!planned_force.contact[leg] && force.norm() > 1.0e-9))
        {
            failure = "force_reference_mask_invalid";
            return false;
        }
        input.force_ref.segment<3>(3 * static_cast<int>(leg)) = force;
        if (!input.contact[leg])
        {
            application_points[leg] = actual_position;
            continue;
        }
        Eigen::Vector3d point;
        ContactSurface surface;
        if (!ResolveScheduleSurface(problem, nominal, leg, planned_force.end,
                                    point, surface))
        {
            failure = "wbc_surface_unavailable";
            return false;
        }
        const Eigen::Vector3d normal = surface.basis_world.col(2);
        const double radius = actual.dynamics.foot_geometry[leg].collision_radius_m;
        if (!normal.allFinite() || std::abs(normal.norm() - 1.0) > 1.0e-8 ||
            !std::isfinite(radius) || radius <= 0.0)
        {
            failure = "actual_contact_geometry_invalid";
            return false;
        }
        input.contact_normal[leg] = normal;
        input.contact_normal_valid[leg] = true;
        params.min_normal_n_by_leg[leg] = surface.min_normal_n;
        params.friction_mu = std::min(params.friction_mu, surface.friction_mu);
        params.max_normal_n = std::min(params.max_normal_n, surface.max_normal_n);
        // The map point is a reference. WBC force application follows the
        // actual sphere center and current surface normal, preserving measured
        // plant geometry when the actual foot has drifted from that reference.
        application_points[leg] = actual_position - radius * normal;
    }
    if (params.max_normal_n == std::numeric_limits<double>::infinity())
    {
        params.max_normal_n = 0.0;
        params.friction_mu = 0.0;
    }
    if (!robot.EvaluateContactJacobians(
            state, application_points, input.force_application_jac_world))
    {
        failure = "force_application_jacobian_failed";
        return false;
    }
    input.have_force_application_jacobian = true;
    const Eigen::Vector3d rpy = Rpy(state.quat_world_from_body);
    orientation_acc = Eigen::Vector3d(
        -config.orientation_kp_roll * rpy.x() -
            config.orientation_kd_roll * state.angular_vel_body.x(),
        -config.orientation_kp_pitch * rpy.y() -
            config.orientation_kd_pitch * state.angular_vel_body.y(),
        -config.orientation_kp_yaw * WrapAngle(rpy.z() - initial_yaw) -
            config.orientation_kd_yaw * state.angular_vel_body.z());
    orientation_acc = ClampComponentwise(
        orientation_acc, config.orientation_acc_limit_radps2,
        orientation_clipped);
    if (!orientation_acc.allFinite())
    {
        failure = "orientation_reference_nonfinite";
        return false;
    }
    input.desired_angular_acc_body = orientation_acc;
    // This flag is consumed only by the proposed centroidal orientation seam;
    // without that seam the underlying WBC must reject or visibly ignore it.
    input.have_centroidal_orientation_task = true;
    return true;
}
inline void FillReplayRowState(
    ReplayRow &row, std::size_t step, double source_time_s,
    const go2_control::RigidBodyState &state,
    const go2_control::RigidBodyPlanningKinematics &actual,
    const ContactObservation &observation,
    const std::array<bool, go2::kLegCount> &source_measured,
    const FixedScheduleInterval *nominal,
    const FootTrajectorySample *feet,
    const CentroidalState *planned_state,
    const ContactForceInterval *planned_force,
    const go2_control::IdWbcOutput *wbc,
    const go2_control::IdWbcPhysicalCertificate *certificate,
    const go2_trot::MotorCommandCertificate *motor)
{
    row.step = step;
    row.source_time_s = source_time_s;
    row.reference_time_s = source_time_s;
    row.actual_state_finite = FiniteState(state) && actual.valid && actual.dynamics.valid;
    row.recorded_source_measured_mask = ContactMask(source_measured);
    row.model_sensor_contact_mask = ContactMask(observation.sensor_contact);
    row.mujoco_geom_contact_mask = ContactMask(observation.geom_contact);
    row.ncon = observation.ncon;
    row.nonfoot_contact_count = observation.nonfoot_contact_count;
    row.nonfoot_contact_force_n = observation.nonfoot_contact_force_n;
    row.nominal_contact_mask = nominal == nullptr ? 0 : ContactMask(nominal->contact);
    row.com_world = actual.dynamics.com_world;
    row.com_velocity_world = actual.com_velocity_world;
    row.momentum_world = actual.angular_momentum_world;
    row.base_position = state.position_world;
    row.base_quaternion = state.quat_world_from_body;
    row.rpy = Rpy(state.quat_world_from_body);
    row.pitch_rate_radps = state.angular_vel_body.y();
    for (std::size_t j = 0; j < go2::kJointCount; ++j)
    {
        row.q[j] = state.q[j];
        row.dq[j] = state.dq[j];
    }
    if (nominal != nullptr)
        row.wbc_contact_mask = ContactMask(nominal->contact);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        row.foot_world[leg] = actual.dynamics.foot_pos_world[leg];
        row.foot_velocity_world[leg] =
            actual.dynamics.foot_jac_world[leg] * actual.dynamics.qvel;
        row.gt_force_world[leg] = observation.force_world[leg];
        row.gt_local_force[leg] = observation.local_force[leg];
        row.gt_couple_world[leg] = observation.couple_world[leg];
        row.gt_local_couple[leg] = observation.local_couple[leg];
    }
    row.gt_force_norm_n = observation.total_force_world.norm();
    row.gt_force_moment_about_com_norm_nm =
        observation.total_moment_about_com_world.norm();
    row.gt_contact_couple_norm_nm = observation.total_couple_world.norm();
    row.gt_wrench_moment_norm_nm =
        observation.total_contact_wrench_moment_world.norm();
    if (feet != nullptr && feet->valid)
    {
        row.foot_reference_valid = true;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            row.foot_reference_world[leg] = Vec(feet->center_world[leg].value);
            row.foot_velocity_reference_world[leg] = Vec(feet->velocity_world[leg]);
            row.foot_error_world[leg] =
                row.foot_reference_world[leg] - row.foot_world[leg];
        }
    }
    if (planned_state != nullptr && planned_state->allFinite())
    {
        row.reference_state_valid = true;
        row.com_reference_world = planned_state->head<3>();
        row.com_velocity_reference_world = planned_state->segment<3>(3);
        row.momentum_reference_world = planned_state->tail<3>();
        row.com_error_m = (row.com_reference_world - row.com_world).norm();
        row.com_velocity_error_mps =
            (row.com_velocity_reference_world - row.com_velocity_world).norm();
        row.momentum_error_nms =
            (row.momentum_reference_world - row.momentum_world).norm();
    }
    if (planned_force != nullptr)
    {
        row.reference_force_norm_n = [&]() {
            Eigen::Vector3d total = Eigen::Vector3d::Zero();
            for (const auto &f : planned_force->force_world)
                total += Vec(f);
            return total.norm();
        }();
    }
    if (wbc != nullptr)
    {
        row.solver_returned = wbc->solution_finite;
        row.solver_converged = wbc->qp_converged;
        row.eq_residual = wbc->eq_residual;
        row.rne_residual = wbc->rne_residual;
        row.max_tau_violation_nm = wbc->max_tau_violation_nm;
        row.max_tau_nm = wbc->tau.cwiseAbs().maxCoeff();
        row.wbc_force_norm_n = wbc->force.norm();
        for (std::size_t j = 0; j < go2::kJointCount; ++j)
            row.wbc_tau[j] = wbc->tau[j];
        if (nominal != nullptr)
            row.wbc_contact_mask = ContactMask(nominal->contact);
    }
    if (certificate != nullptr)
    {
        row.independent_certificate_valid = certificate->feasible;
        row.certificate_force_residual_n = certificate->max_dynamics_force_residual_N;
        row.certificate_moment_residual_nm = certificate->max_dynamics_moment_residual_Nm;
        row.certificate_joint_residual_nm = certificate->max_joint_dynamics_residual_Nm;
        row.max_tau_violation_nm = certificate->max_tau_violation_Nm;
    }
    if (motor != nullptr)
    {
        row.motor_envelope_valid = motor->input_valid && motor->within_model_envelope;
        row.max_motor_saturation_nm = motor->maximum_saturation_nm;
        for (std::size_t j = 0; j < go2::kJointCount; ++j)
        {
            row.requested_tau[j] = motor->requested_torque_nm[j];
            row.applied_tau[j] = motor->predicted_applied_torque_nm[j];
        }
    }
}
inline bool WriteReplayMetadata(
    std::ostream &out, const std::filesystem::path &scene_path,
    const std::filesystem::path &controller_path, double duration_s,
    double source_time_s, const ClosedLoopResearchConfig &config)
{
    out << "# replay_kind=same_mjcf_closed_loop_diagnostic\n"
        << "# execution_authority=0\n"
        << "# b1_claim=0\n"
        << "# contact_evolution_verified=0\n"
        << "# quaternion_representation=normalized_as_Go2RigidBody_SetState\n"
        << "# cold_start=1\n"
        << "# original_warmstart_replayed=0\n"
        << "# torque_mode=direct_torque_only_kp0_kd0\n"
        << "# qp_solver=two_level_primal_active_set_body_stance_then_swing_momentum\n"
        << "# inflight_continuation=measured_p_v_no_additional_bump\n"
        << "# contact_merge_mode=0_scheduled_only\n"
        << "# map_source=proposal_only_scene_geometry_not_used_to_fill_map\n"
        << "# gt_contact_scope=external_contacts_only_robot_self_contacts_omitted\n"
        << "# gt_moment_origin=controller_com_world_subtree_com\n"
        << "# gt_wrench_note=lever_arm_moment_and_local_contact_couple_are_separate\n"
        << "# gt_force_application_scope=point_force_does_not_model_rolling_or_torsional_couples\n"
        << "# scene_path=" << scene_path.string() << '\n'
        << "# controller_model_path=" << controller_path.string() << '\n'
        << "# source_time_s=" << CsvDouble(source_time_s) << '\n'
        << "# duration_s=" << CsvDouble(duration_s) << '\n'
        << "# timestep_s=" << CsvDouble(kClosedLoopReplayDtS) << '\n'
        << "# swing_clearance_m=" << CsvDouble(config.swing_clearance_m) << '\n'
        << "# foot_feedback_limit_mps2=" << CsvDouble(config.foot_feedback_component_limit_mps2) << '\n'
        << "# model_sample_verified_is_independent_certificate_and_motor_envelope\n";
    return out.good();
}
#ifndef GO2_MODEL_PATH
#define GO2_MODEL_PATH "unitree_robots/go2/go2.xml"
#endif
inline JointClosedLoopReplayResult RunJointClosedLoopReplay(
    const go2_control::RigidBodyState &source_state,
    const CentroidalJointProposal &proposal,
    const std::filesystem::path &scene_path,
    const std::filesystem::path &output_path,
    double duration_s = 0.20,
    ClosedLoopResearchConfig config = {})
{
    JointClosedLoopReplayResult result;
    std::string failure;
    if (!ValidClosedLoopConfig(config) ||
        !ValidateProposalForReplay(proposal, source_state, duration_s, failure))
    {
        result.failure = failure.empty() ? "invalid_replay_input" : failure;
        return result;
    }
    if (scene_path.empty() || output_path.empty() ||
        !std::filesystem::exists(scene_path) ||
        !std::filesystem::is_regular_file(scene_path) ||
        std::filesystem::exists(output_path))
    {
        result.failure = std::filesystem::exists(output_path)
            ? "output_exists_refused" : "scene_or_output_path_invalid";
        return result;
    }
    const auto parent = output_path.parent_path();
    if (!parent.empty() && !std::filesystem::exists(parent))
    {
        result.failure = "output_parent_missing";
        return result;
    }
    char error[1024] = {};
    ModelOwner scene;
    scene.model = mj_loadXML(scene_path.string().c_str(), nullptr, error, sizeof(error));
    if (scene.model == nullptr)
    {
        result.failure = std::string("scene_load_failed:") + error;
        return result;
    }
    scene.data = mj_makeData(scene.model);
    if (scene.data == nullptr)
    {
        result.failure = "scene_data_alloc_failed";
        return result;
    }
    if (!Near(scene.model->opt.timestep, kClosedLoopReplayDtS, 1.0e-12))
    {
        result.failure = "scene_timestep_must_be_002";
        return result;
    }
    char controller_error[1024] = {};
    ModelOwner controller_raw;
    controller_raw.model = mj_loadXML(GO2_MODEL_PATH, nullptr,
                                       controller_error, sizeof(controller_error));
    if (controller_raw.model == nullptr)
    {
        result.failure = std::string("controller_model_load_failed:") + controller_error;
        return result;
    }
    if (!ValidateRobotModels(*scene.model, *controller_raw.model, failure))
    {
        result.failure = failure;
        return result;
    }
    if (!ValidateSceneSensors(*scene.model, failure))
    {
        result.failure = failure;
        return result;
    }
    result.model_match = true;
    go2_control::Go2RigidBody controller;
    if (!controller.Load(GO2_MODEL_PATH) ||
        !WriteStateToPlant(*scene.model, *scene.data, source_state, failure))
    {
        result.failure = failure.empty() ? "controller_or_initial_state_failed" : failure;
        return result;
    }
    const TimeNs start = proposal.selected_problem.request.input.identity.source_state_time;
    const TimeNs duration_ns = TimeNs::FromSeconds(duration_s);
    const TimeNs end{start.value + duration_ns.value};
    CentroidalProblem foot_problem;
    if (!CopyReplaySchedule(proposal.selected_problem, start, end,
                            foot_problem, failure))
    {
        result.failure = failure;
        return result;
    }
    std::array<Eigen::Vector3d, go2::kLegCount> initial_velocity{};
    FootTrajectoryRequest foot_request;
    if (!BuildFootReplayRequest(foot_problem, controller, source_state, end,
                                config, foot_request, initial_velocity, failure))
    {
        result.failure = failure;
        return result;
    }
    const std::size_t steps = static_cast<std::size_t>(duration_ns.value /
        static_cast<std::int64_t>(kClosedLoopReplayDtS * 1.0e9));
    if (steps == 0 || duration_ns.value %
        static_cast<std::int64_t>(kClosedLoopReplayDtS * 1.0e9) != 0)
    {
        result.failure = "duration_not_integral_scene_steps";
        return result;
    }
    std::ofstream output(output_path, std::ios::out | std::ios::trunc);
    if (!output)
    {
        result.failure = "output_open_failed";
        return result;
    }
    if (!WriteReplayMetadata(output, scene_path, GO2_MODEL_PATH, duration_s,
                             start.seconds(), config))
    {
        result.failure = "output_metadata_failed";
        return result;
    }
    WriteReplayHeader(output);
    const double initial_yaw = Rpy(source_state.quat_world_from_body).z();
    std::array<bool, go2::kLegCount> measured_state =
        proposal.selected_problem.request.input.measured_contact.mask;
    const auto recorded_source_measured_mask = measured_state;
    std::size_t rows = 0;
    for (std::size_t step = 0; step < steps; ++step)
    {
        const TimeNs time{start.value + static_cast<std::int64_t>(step) *
            static_cast<std::int64_t>(kClosedLoopReplayDtS * 1.0e9)};
        const double source_time_s = time.seconds();
        const auto state = StateFromPlant(*scene.model, *scene.data);
        go2_control::RigidBodyPlanningKinematics actual;
        ContactObservation observation;
        std::array<bool, go2::kLegCount> next_measured{};
        ReplayRow row;
        row.plant_time_s = scene.data->time;
        row.reference_time_s = source_time_s;
        if (!controller.EvaluatePlanningKinematics(state, actual) ||
            !ReadContactObservation(*scene.model, *scene.data, measured_state,
                                    next_measured, observation, failure))
        {
            row.status = failure.empty() ? "actual_observation_failed" : failure;
            FillReplayRowState(row, step, source_time_s, state, actual,
                               observation, recorded_source_measured_mask, nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr, nullptr);
            WriteReplayRow(output, row); ++rows;
            result.failure = row.status; result.rows = rows;
            return result;
        }
        measured_state = next_measured;
        const auto *nominal = FindSchedule(foot_problem, time);
        auto feet_result = SampleFootTrajectoryAt(foot_request, time);
        FootTrajectorySample feet;
        if (feet_result.valid && feet_result.samples.size() == 1)
            feet = feet_result.samples.front();
        ContactForceInterval planned_force{};
        CentroidalState planned_state = CentroidalState::Constant(
            std::numeric_limits<double>::quiet_NaN());
        Eigen::Matrix<double, 6, 1> desired;
        Eigen::Matrix<double, 6, 1> weights;
        if (nominal == nullptr || !feet_result.valid || feet_result.samples.size() != 1 ||
            !BuildCentroidalReference(proposal.selected_problem,
                proposal.selected_result, time, actual, config, desired, weights,
                planned_force, planned_state, failure))
        {
            row.status = !feet_result.valid
                ? std::string("foot_reference_")+JointPlannerFailureName(feet_result.failure)
                : (failure.empty() ? "reference_unavailable" : failure);
            FillReplayRowState(row, step, source_time_s, state, actual,
                               observation, recorded_source_measured_mask, nominal, &feet,
                               nullptr, nullptr, nullptr, nullptr, nullptr);
            WriteReplayRow(output, row); ++rows;
            result.failure = row.status; result.rows = rows;
            return result;
        }
        go2_control::IdWbcParams params;
        go2_control::IdWbcInput input;
        std::array<Eigen::Vector3d, go2::kLegCount> application_points{};
        std::size_t clipped_feedback = 0;
        Eigen::Vector3d orientation_acc;
        bool orientation_clipped = false;
        if (!BuildWbcReplayInput(foot_problem, controller, state, actual, feet,
                planned_force, desired, weights, *nominal, observation,
                initial_yaw, config, params, input, application_points,
                clipped_feedback, orientation_acc, orientation_clipped, failure))
        {
            row.status = failure.empty() ? "wbc_input_failed" : failure;
            FillReplayRowState(row, step, source_time_s, state, actual,
                               observation, recorded_source_measured_mask, nominal, &feet,
                               &planned_state, &planned_force, nullptr, nullptr, nullptr);
            WriteReplayRow(output, row); ++rows;
            result.failure = row.status; result.rows = rows;
            return result;
        }
        go2_control::IdWbcOutput wbc;
        go2_control::IdWbcQpSnapshot qp_snapshot;
        const auto solve_start=std::chrono::steady_clock::now();
        const bool solver_returned =
            go2_control::SolveInverseDynamicsWbc(params, input, wbc, &qp_snapshot) && wbc.ok;
        row.solve_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-solve_start).count();
        row.qp_iterations=wbc.iterations;
        go2_control::IdWbcPhysicalCertificate certificate;
        if (wbc.solution_finite)
            certificate = go2_control::VerifyIdWbcPhysicalCertificate(params, input, wbc);
        std::array<go2_trot::MotorCommandSample, go2::kJointCount> commands{};
        for (std::size_t motor = 0; motor < go2::kJointCount; ++motor)
        {
            const int dof = controller.MotorDof(static_cast<int>(motor));
            const int joint_row = dof - go2_control::kFloatingNv;
            if (joint_row < 0 || joint_row >= static_cast<int>(go2::kJointCount))
            {
                failure = "motor_dof_mapping_invalid";
                break;
            }
            commands[motor].q = state.q[motor];
            commands[motor].dq = state.dq[motor];
            commands[motor].kp = 0.0;
            commands[motor].kd = 0.0;
            commands[motor].tau = wbc.tau[joint_row];
        }
        const auto motor = failure.empty()
            ? go2_trot::VerifyMotorCommandComposition(controller, commands, state)
            : go2_trot::MotorCommandCertificate{};
        row.status = solver_returned && certificate.feasible && motor.input_valid &&
            motor.within_model_envelope ? "applied" :
            (!solver_returned ? "wbc_solver_failed" :
             (!certificate.feasible ? "independent_certificate_failed" :
              (!motor.input_valid ? "motor_command_invalid" :
               "motor_envelope_violation")));
        row.plant_time_s = scene.data->time;
        row.reference_time_s = source_time_s;
        row.wbc_contact_mask = ContactMask(input.contact);
        row.desired_orientation_acc_body = orientation_acc;
        row.desired_orientation_acc_norm_radps2 = orientation_acc.norm();
        FillReplayRowState(row, step, source_time_s, state, actual,
                           observation, recorded_source_measured_mask, nominal, &feet,
                           &planned_state, &planned_force, &wbc,
                           wbc.solution_finite ? &certificate : nullptr, &motor);
        row.solver_returned = solver_returned;
        row.solver_converged = wbc.qp_converged;
        row.independent_certificate_valid = certificate.feasible;
        row.motor_envelope_valid = motor.input_valid && motor.within_model_envelope;
        row.foot_feedback_clipped_count = clipped_feedback;
        row.orientation_feedback_clipped = orientation_clipped;
        row.cold_start = step == 0;
        row.original_warmstart_replayed = false;
        WriteReplayRow(output, row); ++rows;
        if (!solver_returned || !certificate.feasible || !motor.input_valid ||
            !motor.within_model_envelope || !failure.empty())
        {
            const auto path=output_path.string()+".qp.txt";
            if(!std::filesystem::exists(path)) {
                std::ofstream qp(path);qp<<std::setprecision(17);
                const auto matrix=[&](const char*name,const Eigen::MatrixXd &m){
                    qp<<name<<" "<<m.rows()<<" "<<m.cols()<<"\n"<<m<<"\n";};
                matrix("H",qp_snapshot.H);matrix("g",qp_snapshot.g);
                matrix("Aineq",qp_snapshot.Aineq);matrix("bineq",qp_snapshot.bineq);
                matrix("Aeq",qp_snapshot.Aeq);matrix("beq",qp_snapshot.beq);
                matrix("iterate",qp_snapshot.iterate);
            }
            result.failure = row.status; result.rows = rows;
            return result;
        }
        for (std::size_t motor_id = 0; motor_id < go2::kJointCount; ++motor_id)
            scene.data->ctrl[motor_id] = motor.predicted_applied_torque_nm[motor_id];
        mj_step(scene.model, scene.data);
        // mj_step advances q/dq and usually forwards internally; explicitly
        // recompute derived geom/site/contact arrays before the next row so
        // state, contacts and sensor masks share one plant timestamp.
        mj_forward(scene.model, scene.data);
        const auto next_state = StateFromPlant(*scene.model, *scene.data);
        if (!FiniteState(next_state))
        {
            ReplayRow plant_failure;
            plant_failure.step = step + 1;
            plant_failure.plant_time_s = scene.data->time;
            const TimeNs failed_time{start.value +
                static_cast<std::int64_t>(step + 1) *
                static_cast<std::int64_t>(kClosedLoopReplayDtS * 1.0e9)};
            plant_failure.reference_time_s = failed_time.seconds();
            plant_failure.status = "plant_state_nonfinite";
            plant_failure.cold_start = false;
            FillReplayRowState(plant_failure, step + 1, failed_time.seconds(), next_state,
                               actual, observation, recorded_source_measured_mask, nullptr,
                               nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
            WriteReplayRow(output, plant_failure); ++rows;
            result.failure = plant_failure.status; result.rows = rows;
            return result;
        }
    }
    ReplayRow terminal;
    const auto terminal_state = StateFromPlant(*scene.model, *scene.data);
    go2_control::RigidBodyPlanningKinematics terminal_actual;
    ContactObservation terminal_observation;
    std::array<bool, go2::kLegCount> terminal_measured{};
    std::string terminal_failure;
    if (!controller.EvaluatePlanningKinematics(terminal_state, terminal_actual) ||
        !ReadContactObservation(*scene.model, *scene.data, measured_state,
                                terminal_measured, terminal_observation, terminal_failure))
    {
        terminal.status = terminal_failure.empty() ? "terminal_observation_failed" : terminal_failure;
        FillReplayRowState(terminal, steps, end.seconds(), terminal_state,
                           terminal_actual, terminal_observation, recorded_source_measured_mask,
                           nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
        WriteReplayRow(output, terminal); ++rows;
        result.failure = terminal.status; result.rows = rows;
        return result;
    }
    terminal.terminal = true;
    terminal.status = "terminal";
    terminal.plant_time_s = scene.data->time;
    terminal.reference_time_s = end.seconds();
    FillReplayRowState(terminal, steps, end.seconds(), terminal_state,
                       terminal_actual, terminal_observation, recorded_source_measured_mask,
                       nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    WriteReplayRow(output, terminal); ++rows;
    output.flush();
    if (!output.good()) { result.failure = "output_write_failed"; result.rows = rows; return result; }
    result.completed = true;
    result.model_match = true;
    result.rows = rows;
    result.failure = "none";
    return result;
}
} // namespace joint_closed_loop_detail
} // namespace stage_c
} // namespace go2_terrain
