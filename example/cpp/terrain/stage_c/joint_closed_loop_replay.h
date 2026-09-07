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
#include "joint_feedback_reference.h"
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
using namespace joint_feedback_reference;
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
inline bool Near(double a, double b, double absolute = 1.0e-10)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a - b) <= absolute * std::max({1.0, std::abs(a), std::abs(b)});
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
                planned_force, desired, weights, *nominal, observation.sensor_contact,
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
