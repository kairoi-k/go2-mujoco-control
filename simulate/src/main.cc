// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// !!! hack code: make glfw_adapter.window_ public
#define private public
#include "glfw_adapter.h"
#undef private

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#if defined(__linux__)
#include <sched.h>
#endif
#include <cstring>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <vector>
#include "simulate.h"
#include "array_safety.h"
#include "unitree_sdk2_bridge.h"
#include "param.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"
#define NUM_MOTOR_IDL_GO 20

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }


  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;


namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;
  volatile sig_atomic_t shutdown_requested = 0;
  void RequestShutdown(int)
  {
    shutdown_requested = 1;
    const char msg[] = "SIGNAL: shutdown requested\n";
    ::write(2, msg, sizeof(msg) - 1);
  }

  using Seconds = std::chrono::duration<double>;

  class GroundTruthContactLogger
  {
  public:
    void Configure(mjModel *model)
    {
      if (param::config.ground_truth_log.empty())
      {
        return;
      }
      if (model == model_ && ready_)
      {
        return;
      }

      model_ = model;
      full_mass_matrix_.assign(model->nv * model->nv, 0.0);
      ready_ = false;
      if (model->nq < 7 || model->nv < 6)
      {
        std::cerr << "Ground-truth contact logger requires a free base"
                  << std::endl;
        return;
      }

      total_mass_kg_ = 0.0;
      base_body_id_ = mj_name2id(model, mjOBJ_BODY, "base_link");
      for (int body_id = 1; body_id < model->nbody; ++body_id)
      {
        total_mass_kg_ += model->body_mass[body_id];
      }
      if (base_body_id_ < 0)
      {
        std::cerr << "Ground-truth contact logger missing base_link body"
                  << std::endl;
        return;
      }
      dof_labels_.clear();
      static constexpr std::array<const char *, 6> kFreeDofLabels = {
          "base_qcoord_trans_x",
          "base_qcoord_trans_y",
          "base_qcoord_trans_z",
          "base_qcoord_rot_x",
          "base_qcoord_rot_y",
          "base_qcoord_rot_z"};
      for (int dof = 0; dof < model->nv; ++dof)
      {
        if (dof < static_cast<int>(kFreeDofLabels.size()))
        {
          dof_labels_.emplace_back(kFreeDofLabels[dof]);
          continue;
        }
        const int joint_id = model->dof_jntid[dof];
        const char *joint_name =
            mj_id2name(model, mjOBJ_JOINT, joint_id);
        dof_labels_.emplace_back(
            joint_name != nullptr
                ? joint_name
                : "dof_" + std::to_string(dof));
      }
      sensor_ids_.fill(-1);
      touch_ids_.fill(-1);
      site_ids_.fill(-1);
      foot_body_ids_.fill(-1);
      foot_geom_ids_.fill(-1);
      geom_leg_ids_.assign(model->ngeom, -1);
      terrain_geom_ids_.assign(model->ngeom, false);
      leg_root_body_ids_.fill(-1);
      obstacle_geom_id_ = mj_name2id(model, mjOBJ_GEOM, "reactive_obstacle");

      static constexpr std::array<const char *, 4> kLegs = {
          "FR", "FL", "RR", "RL"};
      for (std::size_t i = 0; i < kLegs.size(); ++i)
      {
        const std::string force_name =
            std::string(kLegs[i]) + "_foot_force_3d";
        const std::string touch_name =
            std::string(kLegs[i]) + "_foot_force";
        const std::string site_name =
            std::string(kLegs[i]) + "_foot_contact";

        sensor_ids_[i] = mj_name2id(model, mjOBJ_SENSOR, force_name.c_str());
        touch_ids_[i] = mj_name2id(model, mjOBJ_SENSOR, touch_name.c_str());
        site_ids_[i] = mj_name2id(model, mjOBJ_SITE, site_name.c_str());
        leg_root_body_ids_[i] = mj_name2id(model, mjOBJ_BODY, (std::string(kLegs[i]) + "_hip").c_str());
        foot_body_ids_[i] = mj_name2id(model, mjOBJ_BODY, (std::string(kLegs[i]) + "_foot").c_str());
        foot_geom_ids_[i] = mj_name2id(model, mjOBJ_GEOM, kLegs[i]);
        if (sensor_ids_[i] < 0 || touch_ids_[i] < 0 || site_ids_[i] < 0 ||
            foot_body_ids_[i] < 0 || leg_root_body_ids_[i] < 0 || foot_geom_ids_[i] < 0)
        {
          std::cerr << "Ground-truth contact logger missing sensor/site for "
                    << kLegs[i] << std::endl;
          return;
        }
        if (model->sensor_dim[sensor_ids_[i]] != 3 ||
            model->sensor_dim[touch_ids_[i]] != 1)
        {
          std::cerr << "Ground-truth contact logger found an unexpected sensor "
                    << "dimension for " << kLegs[i] << std::endl;
          return;
        }
      }

      for (int geom_id = 0; geom_id < model->ngeom; ++geom_id)
      {
        const char *geom_name = mj_id2name(model, mjOBJ_GEOM, geom_id);
        terrain_geom_ids_[geom_id] = geom_name != nullptr &&
            std::strncmp(geom_name, "phase2_step", 11) == 0;
        const int geom_body_id = model->geom_bodyid[geom_id];
        for (std::size_t leg = 0; leg < leg_root_body_ids_.size(); ++leg)
        {
          int candidate = geom_body_id;
          while (candidate > 0 && candidate != leg_root_body_ids_[leg])
          {
            candidate = model->body_parentid[candidate];
          }
          if (candidate == leg_root_body_ids_[leg])
          {
            geom_leg_ids_[geom_id] = static_cast<int>(leg);
          }
        }
      }

      if (!stream_.is_open())
      {
        const auto parent = param::config.ground_truth_log.parent_path();
        std::error_code error;
        if (!parent.empty())
        {
          std::filesystem::create_directories(parent, error);
          if (error)
          {
            std::cerr << "Ground-truth contact logger cannot create directory: "
                      << error.message() << std::endl;
            return;
          }
        }

        stream_.open(param::config.ground_truth_log,
                     std::ios::out | std::ios::trunc);
        if (!stream_)
        {
          std::cerr << "Ground-truth contact logger cannot open: "
                    << param::config.ground_truth_log << std::endl;
          return;
        }

        stream_ << "time_s,step_index,total_mass_kg"
                 << ",base_pos_world_x_m,base_pos_world_y_m,base_pos_world_z_m"
                 << ",base_quat_w,base_quat_x,base_quat_y,base_quat_z"
                 << ",base_qvel_world_x_mps,base_qvel_world_y_mps,base_qvel_world_z_mps"
                 << ",base_angvel_body_x_radps,base_angvel_body_y_radps,base_angvel_body_z_radps"
                 << ",base_qacc_world_x_mps2,base_qacc_world_y_mps2,base_qacc_world_z_mps2"
                 << ",base_angacc_body_x_radps2,base_angacc_body_y_radps2,base_angacc_body_z_radps2"
                 << ",base_qfrc_constraint_trans_x_N,base_qfrc_constraint_trans_y_N,base_qfrc_constraint_trans_z_N"
                 << ",base_qfrc_constraint_rot_x_Nm,base_qfrc_constraint_rot_y_Nm,base_qfrc_constraint_rot_z_Nm"
                 << ",gravity_world_x_mps2,gravity_world_y_mps2,gravity_world_z_mps2"
                 << ",subtree_com_world_x_m,subtree_com_world_y_m,subtree_com_world_z_m,subtree_linvel_world_x_mps,subtree_linvel_world_y_mps,subtree_linvel_world_z_mps"
                 << ",total_contact_grf_world_x_N,total_contact_grf_world_y_N,total_contact_grf_world_z_N"
                 << ",total_contact_moment_world_x_Nm,total_contact_moment_world_y_Nm,total_contact_moment_world_z_Nm";
        stream_ << ",reactive_obstacle_contact_count,reactive_obstacle_contact_force_N"
                 << ",reactive_obstacle_contact_normal_force_N"
                 << ",reactive_obstacle_contact_other_geom_id"
                 << ",phase2_terrain_foot_contact_mask"
                 << ",phase2_terrain_nonfoot_contact_count"
                 << ",phase2_terrain_nonfoot_contact_force_N";
        for (const char *leg : kLegs)
        {
          stream_ << "," << leg << "_sensor_force_site_x_N"
                  << "," << leg << "_sensor_force_site_y_N"
                  << "," << leg << "_sensor_force_site_z_N"
                  << "," << leg << "_sensor_force_world_x_N"
                  << "," << leg << "_sensor_force_world_y_N"
                  << "," << leg << "_sensor_force_world_z_N"
                  << "," << leg << "_contact_grf_world_x_N"
                  << "," << leg << "_contact_grf_world_y_N"
                  << "," << leg << "_contact_grf_world_z_N"
                  << "," << leg << "_foot_contact_grf_world_x_N"
                  << "," << leg << "_foot_contact_grf_world_y_N"
                  << "," << leg << "_foot_contact_grf_world_z_N"
                  << "," << leg << "_pos_world_x_m"
                  << "," << leg << "_pos_world_y_m"
                  << "," << leg << "_pos_world_z_m"
                  << "," << leg << "_touch_N";
        }
        for (int row = 0; row < 6; ++row)
          for (int col = 0; col < 6; ++col)
            stream_ << ",base_mass_matrix_qcoord_r" << row << "c" << col;
        for (const char *prefix : {
                 "base_mass_qacc_qfrc_qcoord_",
                 "base_qfrc_smooth_qcoord_",
                 "base_qfrc_bias_qcoord_",
                 "base_qfrc_passive_qcoord_",
                 "base_qfrc_actuator_qcoord_",
                 "base_qfrc_applied_qcoord_",
                 "base_dynamics_residual_qcoord_"})
        {
          AppendQcoordVectorHeader(stream_, prefix);
        }
        for (const char *prefix : {
                 "full_mass_qacc_qfrc_qcoord_",
                 "full_qfrc_smooth_qcoord_",
                 "full_qfrc_constraint_qcoord_",
                 "full_qfrc_actuator_qcoord_"})
        {
          AppendDofVectorHeader(stream_, prefix);
        }
        stream_ << "\n";
      }

      ready_ = true;
    }
    std::array<std::array<double, 3>, 4> ComputeContactGrf(
        const mjModel *model, const mjData *data,
        std::array<double, 3> *total_contact_grf_world,
        std::array<double, 3> *total_contact_moment_world,
        std::array<std::array<double, 3>, 4> *foot_contact_grf_world) const
    {
      std::array<std::array<double, 3>, 4> contact_grf_world{};
      total_contact_grf_world->fill(0.0);
      total_contact_moment_world->fill(0.0);
      for (auto &force : *foot_contact_grf_world)
      {
        force.fill(0.0);
      }
      mjtNum contact_force[6];
      const mjtNum *base_pos = data->xpos + 3 * base_body_id_;
      for (int contact_id = 0; contact_id < data->ncon; ++contact_id)
      {
        const mjContact &contact = data->contact[contact_id];
        if (contact.exclude != 0 || contact.efc_address < 0 ||
            contact.geom[0] < 0 || contact.geom[1] < 0 ||
            contact.geom[0] >= model->ngeom ||
            contact.geom[1] >= model->ngeom)
        {
          continue;
        }

        const int geom0_body = model->geom_bodyid[contact.geom[0]];
        const int geom1_body = model->geom_bodyid[contact.geom[1]];
        if ((geom0_body == 0) == (geom1_body == 0))
        {
          continue;
        }

        const bool robot_is_geom0 = geom0_body != 0;
        const int robot_geom = robot_is_geom0 ? contact.geom[0] : contact.geom[1];
        const int leg = geom_leg_ids_[robot_geom];

        // mj_contactForce points from geom[0] to geom[1]. Negate it when the
        // robot is geom[0] so the accumulated value is force on the robot.
        const double sign = robot_is_geom0 ? -1.0 : 1.0;
        mj_contactForce(model, data, contact_id, contact_force);

        // Contact-frame axes are stored as rows, unlike ordinary MuJoCo matrices.
        const double force_x =
            contact.frame[0] * contact_force[0] +
            contact.frame[3] * contact_force[1] +
            contact.frame[6] * contact_force[2];
        const double force_y =
            contact.frame[1] * contact_force[0] +
            contact.frame[4] * contact_force[1] +
            contact.frame[7] * contact_force[2];
        const double force_z =
            contact.frame[2] * contact_force[0] +
            contact.frame[5] * contact_force[1] +
            contact.frame[8] * contact_force[2];
        const double torque_x =
            contact.frame[0] * contact_force[3] +
            contact.frame[3] * contact_force[4] +
            contact.frame[6] * contact_force[5];
        const double torque_y =
            contact.frame[1] * contact_force[3] +
            contact.frame[4] * contact_force[4] +
            contact.frame[7] * contact_force[5];
        const double torque_z =
            contact.frame[2] * contact_force[3] +
            contact.frame[5] * contact_force[4] +
            contact.frame[8] * contact_force[5];
        const double signed_torque_x = sign * torque_x;
        const double signed_torque_y = sign * torque_y;
        const double signed_torque_z = sign * torque_z;
        const double signed_force_x = sign * force_x;
        (*total_contact_moment_world)[0] += signed_torque_x;
        (*total_contact_moment_world)[1] += signed_torque_y;
        (*total_contact_moment_world)[2] += signed_torque_z;
        const double signed_force_y = sign * force_y;
        const double signed_force_z = sign * force_z;

        (*total_contact_grf_world)[0] += sign * force_x;
        (*total_contact_grf_world)[1] += sign * force_y;
        (*total_contact_grf_world)[2] += sign * force_z;
        (*total_contact_moment_world)[0] +=
            (contact.pos[1] - base_pos[1]) * signed_force_z
            - (contact.pos[2] - base_pos[2]) * signed_force_y;
        (*total_contact_moment_world)[1] +=
            (contact.pos[2] - base_pos[2]) * signed_force_x
            - (contact.pos[0] - base_pos[0]) * signed_force_z;
        (*total_contact_moment_world)[2] +=
            (contact.pos[0] - base_pos[0]) * signed_force_y
            - (contact.pos[1] - base_pos[1]) * signed_force_x;
        if (leg >= 0)
        {
          contact_grf_world[leg][0] += sign * force_x;
          if (robot_geom == foot_geom_ids_[leg])
          {
            (*foot_contact_grf_world)[leg][0] += sign * force_x;
            (*foot_contact_grf_world)[leg][1] += sign * force_y;
            (*foot_contact_grf_world)[leg][2] += sign * force_z;
          }
          contact_grf_world[leg][1] += sign * force_y;
          contact_grf_world[leg][2] += sign * force_z;
        }
      }
      return contact_grf_world;
    }
    void ComputeObstacleContact(
        const mjModel *model, const mjData *data,
        int *contact_count, double *force_N, double *normal_force_N,
        int *other_geom_id) const
    {
      *contact_count = 0;
      *force_N = 0.0;
      *normal_force_N = 0.0;
      *other_geom_id = -1;
      if (obstacle_geom_id_ < 0)
        return;
      mjtNum contact_force[6];
      for (int contact_id = 0; contact_id < data->ncon; ++contact_id)
      {
        const mjContact &contact = data->contact[contact_id];
        if (contact.exclude != 0 || contact.efc_address < 0 ||
            (contact.geom[0] != obstacle_geom_id_ &&
             contact.geom[1] != obstacle_geom_id_))
        {
          continue;
        }
        mj_contactForce(model, data, contact_id, contact_force);
        const int other_geom = contact.geom[0] == obstacle_geom_id_
            ? contact.geom[1]
            : contact.geom[0];
        if (model->geom_bodyid[other_geom] == 0)
        {
          continue;
        }
        if (*other_geom_id < 0)
          *other_geom_id = other_geom;
        ++(*contact_count);
        *force_N += std::hypot(
            contact_force[0],
            std::hypot(contact_force[1], contact_force[2]));
        *normal_force_N += std::abs(contact_force[0]);
      }
    }

    void ComputePhase2TerrainContact(
        const mjModel *model, const mjData *data, int *foot_contact_mask,
        int *nonfoot_contact_count, double *nonfoot_contact_force_N) const
    {
      *foot_contact_mask = 0;
      *nonfoot_contact_count = 0;
      *nonfoot_contact_force_N = 0.0;
      mjtNum contact_force[6];
      for (int contact_id = 0; contact_id < data->ncon; ++contact_id)
      {
        const mjContact &contact = data->contact[contact_id];
        if (contact.exclude != 0 || contact.efc_address < 0 ||
            contact.geom[0] < 0 || contact.geom[1] < 0)
          continue;
        const int terrain_side = terrain_geom_ids_[contact.geom[0]] ? 0
            : (terrain_geom_ids_[contact.geom[1]] ? 1 : -1);
        if (terrain_side < 0)
          continue;
        const int robot_geom = contact.geom[1 - terrain_side];
        if (model->geom_bodyid[robot_geom] == 0)
          continue;
        bool foot_contact = false;
        for (std::size_t leg = 0; leg < foot_geom_ids_.size(); ++leg)
        {
          if (robot_geom == foot_geom_ids_[leg])
          {
            *foot_contact_mask |= 1 << static_cast<int>(leg);
            foot_contact = true;
            break;
          }
        }
        if (foot_contact)
          continue;
        mj_contactForce(model, data, contact_id, contact_force);
        ++(*nonfoot_contact_count);
        *nonfoot_contact_force_N += std::hypot(
            contact_force[0],
            std::hypot(contact_force[1], contact_force[2]));
      }
    }

    void Log(const mjModel *model, mjData *data)
    {
      if (!ready_ || model != model_ || !stream_)
      {
        return;
      }

      static constexpr std::array<const char *, 4> kLegs = {
          "FR", "FL", "RR", "RL"};
      const mjtNum *base_pos = data->qpos;
      // MuJoCo free-joint qvel/qacc store linear components before angular ones.
      const mjtNum *base_qvel = data->qvel;
      const mjtNum *base_qacc = data->qacc;
      mjtNum base_velocity[6] = {};
      mjtNum base_acceleration[6] = {};
      mj_objectVelocity(model, data, mjOBJ_BODY, base_body_id_,
                        base_velocity, 1);
      mj_objectAcceleration(model, data, mjOBJ_BODY, base_body_id_,
                            base_acceleration, 1);
      mj_fullM(model, full_mass_matrix_.data(), data->qM);
      std::vector<double> full_mass_qacc_qfrc(
          model->nv, 0.0);
      for (int row = 0; row < model->nv; ++row)
      {
        for (int col = 0; col < model->nv; ++col)
        {
          full_mass_qacc_qfrc[row] +=
              full_mass_matrix_[row * model->nv + col] * data->qacc[col];
        }
      }
      std::array<double, 6> base_mass_qacc_qfrc{};
      std::array<double, 6> base_qfrc_smooth{};
      std::array<double, 6> base_qfrc_bias{};
      std::array<double, 6> base_qfrc_passive{};
      std::array<double, 6> base_qfrc_actuator{};
      std::array<double, 6> base_qfrc_applied{};
      std::array<double, 6> base_dynamics_residual{};
      for (int row = 0; row < 6; ++row)
      {
        base_mass_qacc_qfrc[row] = full_mass_qacc_qfrc[row];
        base_qfrc_smooth[row] = data->qfrc_smooth[row];
        base_qfrc_bias[row] = data->qfrc_bias[row];
        base_qfrc_passive[row] = data->qfrc_passive[row];
        base_qfrc_actuator[row] = data->qfrc_actuator[row];
        base_qfrc_applied[row] = data->qfrc_applied[row];
        base_dynamics_residual[row] =
            base_mass_qacc_qfrc[row]
            - base_qfrc_smooth[row]
            - data->qfrc_constraint[row];
      }
      mj_subtreeVel(model, data);
      const mjtNum *subtree_com = data->subtree_com + 3 * base_body_id_;
      const mjtNum *subtree_linvel = data->subtree_linvel + 3 * base_body_id_;
      std::array<double, 3> total_contact_grf_world{};
      std::array<double, 3> total_contact_moment_world{};
      std::array<std::array<double, 3>, 4> foot_contact_grf_world{};
      const auto contact_grf_world =
          ComputeContactGrf(model, data, &total_contact_grf_world,
                            &total_contact_moment_world,
                            &foot_contact_grf_world);
      int obstacle_contact_count = 0;
      double obstacle_contact_force_N = 0.0;
      double obstacle_contact_normal_force_N = 0.0;
      int obstacle_contact_other_geom_id = -1;
      ComputeObstacleContact(model, data, &obstacle_contact_count,
                             &obstacle_contact_force_N,
                             &obstacle_contact_normal_force_N,
                             &obstacle_contact_other_geom_id);
      int terrain_foot_contact_mask = 0;
      int terrain_nonfoot_contact_count = 0;
      double terrain_nonfoot_contact_force_N = 0.0;
      ComputePhase2TerrainContact(
          model, data, &terrain_foot_contact_mask,
          &terrain_nonfoot_contact_count, &terrain_nonfoot_contact_force_N);
      stream_ << std::setprecision(12) << data->time
              << "," << step_index_
              << "," << total_mass_kg_
              << "," << base_pos[0]
              << "," << base_pos[1]
              << "," << base_pos[2]
              << "," << base_pos[3]
              << "," << base_pos[4]
              << "," << base_pos[5]
              << "," << base_pos[6]
              << "," << base_qvel[0]
              << "," << base_qvel[1]
              << "," << base_qvel[2]
              << "," << base_velocity[0]
              << "," << base_velocity[1]
              << "," << base_velocity[2]
              << "," << base_qacc[0]
              << "," << base_qacc[1]
              << "," << base_qacc[2]
              << "," << base_acceleration[0]
              << "," << base_acceleration[1]
              << "," << base_acceleration[2]
              << "," << data->qfrc_constraint[0]
              << "," << data->qfrc_constraint[1]
              << "," << data->qfrc_constraint[2]
              << "," << data->qfrc_constraint[3]
              << "," << data->qfrc_constraint[4]
              << "," << data->qfrc_constraint[5];
      stream_ << "," << model->opt.gravity[0]
              << "," << model->opt.gravity[1]
              << "," << model->opt.gravity[2]
              << "," << subtree_com[0] << "," << subtree_com[1] << "," << subtree_com[2]
              << "," << subtree_linvel[0] << "," << subtree_linvel[1] << "," << subtree_linvel[2]
              << "," << total_contact_grf_world[0]
              << "," << total_contact_grf_world[1]
              << "," << total_contact_grf_world[2]
              << "," << total_contact_moment_world[0]
              << "," << total_contact_moment_world[1]
              << "," << total_contact_moment_world[2]
              << "," << obstacle_contact_count
              << "," << obstacle_contact_force_N
              << "," << obstacle_contact_normal_force_N
              << "," << obstacle_contact_other_geom_id
              << "," << terrain_foot_contact_mask
              << "," << terrain_nonfoot_contact_count
              << "," << terrain_nonfoot_contact_force_N;

      for (std::size_t i = 0; i < kLegs.size(); ++i)
      {
        const mjtNum *force =
            data->sensordata + model->sensor_adr[sensor_ids_[i]];
        const int site_id = site_ids_[i];
        const mjtNum *site_xmat = data->site_xmat + 9 * site_id;
        const mjtNum *site_pos = data->site_xpos + 3 * site_id;

        // Keep the raw site sensor for comparison; contact_grf_world is computed from contacts below.
        const double world_x =
            site_xmat[0] * force[0] + site_xmat[1] * force[1] +
            site_xmat[2] * force[2];
        const double world_y =
            site_xmat[3] * force[0] + site_xmat[4] * force[1] +
            site_xmat[5] * force[2];
        const double world_z =
            site_xmat[6] * force[0] + site_xmat[7] * force[1] +
            site_xmat[8] * force[2];
        const double touch =
            data->sensordata[model->sensor_adr[touch_ids_[i]]];

        stream_ << "," << force[0]
                << "," << force[1]
                << "," << force[2]
                << "," << world_x
                << "," << world_y
                << "," << world_z
                << "," << contact_grf_world[i][0]
                << "," << contact_grf_world[i][1]
                << "," << contact_grf_world[i][2]
                << "," << foot_contact_grf_world[i][0]
                << "," << foot_contact_grf_world[i][1]
                << "," << foot_contact_grf_world[i][2]
                << "," << site_pos[0]
                << "," << site_pos[1]
                << "," << site_pos[2]
                << "," << touch;
      }
      for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
          stream_ << "," << full_mass_matrix_[row * model->nv + col];
      AppendQcoordVector(stream_, base_mass_qacc_qfrc);
      AppendQcoordVector(stream_, base_qfrc_smooth);
      AppendQcoordVector(stream_, base_qfrc_bias);
      AppendQcoordVector(stream_, base_qfrc_passive);
      AppendQcoordVector(stream_, base_qfrc_actuator);
      AppendQcoordVector(stream_, base_qfrc_applied);
      AppendQcoordVector(stream_, base_dynamics_residual);
      AppendDofVector(stream_, full_mass_qacc_qfrc);
      AppendDofVector(stream_, data->qfrc_smooth, model->nv);
      AppendDofVector(stream_, data->qfrc_constraint, model->nv);
      AppendDofVector(stream_, data->qfrc_actuator, model->nv);
      stream_ << "\n";
      ++step_index_;
      if ((step_index_ % 256) == 0)
      {
        stream_.flush();
      }
    }

    void Close()
    {
      stream_.flush();
      stream_.close();
      ready_ = false;
    }

  private:
    static void AppendQcoordVectorHeader(
        std::ofstream &stream, const char *prefix)
    {
      static constexpr std::array<const char *, 6> kLabels = {
          "trans_x_N", "trans_y_N", "trans_z_N",
          "rot_x_Nm", "rot_y_Nm", "rot_z_Nm"};
      for (const char *label : kLabels)
      {
        stream << "," << prefix << label;
      }
    }

    static void AppendQcoordVector(
        std::ofstream &stream, const std::array<double, 6> &values)
    {
      for (double value : values)
      {
        stream << "," << value;
      }
    }
    void AppendDofVectorHeader(
        std::ofstream &stream, const char *prefix) const
    {
      for (const std::string &label : dof_labels_)
      {
        stream << "," << prefix << label;
      }
    }

    static void AppendDofVector(
        std::ofstream &stream, const std::vector<double> &values)
    {
      for (double value : values)
      {
        stream << "," << value;
      }
    }

    static void AppendDofVector(
        std::ofstream &stream, const mjtNum *values, int count)
    {
      for (int index = 0; index < count; ++index)
      {
        stream << "," << values[index];
      }
    }
    mjModel *model_ = nullptr;
    int base_body_id_ = -1;
    std::array<int, 4> sensor_ids_ = {-1, -1, -1, -1};
    int obstacle_geom_id_ = -1;
    std::vector<mjtNum> full_mass_matrix_;
    std::vector<std::string> dof_labels_;
    std::array<int, 4> touch_ids_ = {-1, -1, -1, -1};
    std::array<int, 4> site_ids_ = {-1, -1, -1, -1};
    std::array<int, 4> foot_body_ids_ = {-1, -1, -1, -1};
    std::array<int, 4> foot_geom_ids_ = {-1, -1, -1, -1};
    std::array<int, 4> leg_root_body_ids_ = {-1, -1, -1, -1};
    std::vector<int> geom_leg_ids_;
    std::vector<bool> terrain_geom_ids_;
    std::ofstream stream_;
    double total_mass_kg_ = 0.0;
    std::uint64_t step_index_ = 0;
    bool ready_ = false;
  };

  GroundTruthContactLogger ground_truth_logger;

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  void ConfigureCamera(mj::Simulate *sim)
  {
    if (!param::config.camera_follow || m == nullptr)
      return;
    const int base_body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
    if (base_body_id > 0)
    {
      sim->cam.type = mjCAMERA_TRACKING;
      sim->cam.trackbodyid = base_body_id;
      sim->cam.fixedcamid = -1;
    }
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    static bool push_logged_ = false;
    static bool push_active_logged_ = false;
    static bool push_vel_applied_ = false;
    static bool payload_applied_ = false;
    static int friction_geom_id = -1;
    static mjtNum friction_nominal_mu = 0.0;
    static bool friction_saved = false;
    static bool friction_changed = false;
    static bool friction_config_logged = false;
    static bool friction_event_restored = false;
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;
    auto next_render_snapshot = mj::Simulate::Clock::now();

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (!friction_config_logged)
      {
        std::cout << "FRICTION config time=" << param::config.friction_time_s
                  << " mu=" << param::config.friction_mu
                  << " duration=" << param::config.friction_duration_s
                  << "\n";
        friction_config_logged = true;
      }
      // disturbance push: set xfrc every step (covers all stepping paths)
      static int push_body_id = -1;
      if (push_body_id < 0)
      {
        push_body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
        if (push_body_id < 0)
          push_body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
        std::cout << "PUSH body_id=" << push_body_id << "\n";
      }
      if (!friction_saved && m != nullptr)
      {
        friction_geom_id = mj_name2id(m, mjOBJ_GEOM, "floor");
        if (friction_geom_id >= 0)
        {
          friction_nominal_mu = m->geom_friction[3 * friction_geom_id];
          friction_saved = true;
          std::cout << "FRICTION floor_geom_id=" << friction_geom_id
                    << " nominal_mu=" << friction_nominal_mu << "\n";
        }
      }
      if (param::config.payload_kg > 0.0 && !payload_applied_)
      {
        int payload_body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
        if (payload_body_id < 0)
          payload_body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
        if (payload_body_id < 0)
          std::cerr << "PAYLOAD: torso_link/base_link not found\n";
        else
        {
          m->body_mass[payload_body_id] += param::config.payload_kg;
          payload_applied_ = true;
          std::cout << "PAYLOAD applied " << param::config.payload_kg
                    << " kg to body_id=" << payload_body_id
                    << " (mass=" << m->body_mass[payload_body_id] << ")\n";
        }
      }
      const bool push_window =
          param::config.push_time_s >= 0.0 && d != nullptr &&
          d->time >= param::config.push_time_s &&
          d->time <= param::config.push_time_s +
                         param::config.push_duration_s;
      if (friction_saved && d != nullptr &&
          param::config.friction_time_s >= 0.0 &&
          !friction_changed && !friction_event_restored &&
          d->time >= param::config.friction_time_s)
      {
        const double mu = std::max(
            0.01, std::min(5.0, param::config.friction_mu));
        m->geom_friction[3 * friction_geom_id] = mu;
        friction_changed = true;
        std::cerr << "FRICTION event active t=" << d->time
                  << " mu=" << mu << std::endl;
      }
      if (friction_changed && friction_saved && d != nullptr &&
          d->time > param::config.friction_time_s +
                         param::config.friction_duration_s)
      {
        m->geom_friction[3 * friction_geom_id] = friction_nominal_mu;
        friction_changed = false;
        friction_event_restored = true;
        std::cerr << "FRICTION restored t=" << d->time
                  << " mu=" << friction_nominal_mu << std::endl;
      }
      if (param::config.push_time_s >= 0.0 && d != nullptr &&
          !push_window)
      {
        for (int qi = 0; qi < m->nv; ++qi)
          d->qfrc_applied[qi] = 0.0;
      }
      if (push_window && param::config.push_vel_x_mps != 0.0 &&
          !push_vel_applied_)
      {
        d->qvel[0] += param::config.push_vel_x_mps;
        push_vel_applied_ = true;
        std::cerr << "PUSH vel applied t=" << d->time
                  << " qvel0=" << d->qvel[0] << std::endl;
      }
      if (push_window && push_body_id >= 0)
      {
        mjtNum force[3] = {param::config.push_force_x_n, 0.0, 0.0};
        mjtNum torque[3] = {0.0, param::config.push_torque_pitch_nm, 0.0};
        mjtNum point[3] = {
            d->xpos[3 * push_body_id + 0],
            d->xpos[3 * push_body_id + 1],
            d->xpos[3 * push_body_id + 2]};
        std::vector<mjtNum> qfrc_target(
            static_cast<std::size_t>(m->nv), 0.0);
        mj_applyFT(m, d, force, torque, point, push_body_id,
                   qfrc_target.data());
        for (int qi = 0; qi < m->nv; ++qi)
            d->qfrc_applied[qi] = qfrc_target[static_cast<std::size_t>(qi)];
        if (!push_active_logged_)
        {
          push_active_logged_ = true;
          std::cerr << "PUSH active t=" << d->time
                    << " force=" << param::config.push_force_x_n
                    << " qfrc0=" << d->qfrc_applied[0]
                    << " qfrc3=" << d->qfrc_applied[3]
                    << " qvel0=" << d->qvel[0] << std::endl;
        }
      }
      if (shutdown_requested)
      {
        sim.exitrequest.store(1);
        continue;
      }
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);
          ConfigureCamera(&sim);
          ground_truth_logger.Configure(m);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);
          ConfigureCamera(&sim);
          ground_truth_logger.Configure(m);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              mj_step(m, d);
              ground_truth_logger.Log(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (param::config.enable_elastic_band == 1)
                {
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    d->xfrc_applied[param::config.band_attached_link] = elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = elastic_band.f_[2];
                  }
                }

                // call mj_step
                mj_step(m, d);
                ground_truth_logger.Log(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
      if (sim.is_passive_ && m && d &&
          mj::Simulate::Clock::now() >= next_render_snapshot)
      {
        sim.PublishRenderSnapshot(m, d);
        next_render_snapshot = mj::Simulate::Clock::now() +
                               std::chrono::milliseconds(16);
      }
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

namespace
{
void PinSimulatorThreadToEnv(const char *env_name)
{
#if defined(__linux__)
  const char *value = std::getenv(env_name);
  if (value == nullptr || value[0] == 0)
    return;
  char *end = nullptr;
  const long cpu = std::strtol(value, &end, 10);
  if (end == value || *end != 0 || cpu < 0 || cpu >= CPU_SETSIZE)
    return;
  cpu_set_t cpu_set{};
  CPU_ZERO(&cpu_set);
  CPU_SET(static_cast<int>(cpu), &cpu_set);
  if (sched_setaffinity(0, sizeof(cpu_set), &cpu_set) != 0)
    std::cerr << "Unable to pin " << env_name << " to CPU " << cpu << "\n";
#else
  (void)env_name;
#endif
}
}
void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  PinSimulatorThreadToEnv("TROT_SIM_PHYSICS_CPU");
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      if (param::config.headless)
      {
        // Headless: install the model directly. Simulate::Load() waits on
        // the render thread (cond_loadrequest) that does not exist in
        // headless mode; PhysicsLoop and the DDS bridge use the globals
        // m/d, and PublishRenderSnapshot no-ops without a passive copy.
        sim->m_ = m;
        sim->d_ = d;
      }
      else
      {
        sim->Load(m, d, filename);
      }
      ConfigureCamera(sim);
      if (std::isfinite(param::config.initial_x_m) && m->nq >= 1)
        d->qpos[0] = param::config.initial_x_m;
      if (std::isfinite(param::config.initial_y_m) && m->nq >= 2)
        d->qpos[1] = param::config.initial_y_m;
      mj_forward(m, d);
      ground_truth_logger.Configure(m);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);
  ground_truth_logger.Close();

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  auto *sim_mutex = static_cast<std::recursive_mutex *>(arg);
  // Wait for mujoco data
  while (true)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, param::config.interface);
  std::cout << "Unitree DDS bridge ready" << std::endl;


  int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (body_id < 0) {
    body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  }
  param::config.band_attached_link = 6 * body_id;
  
  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;
  if (m->nu > NUM_MOTOR_IDL_GO) {
    interface = std::make_unique<G1Bridge>(m, d, sim_mutex);
  } else {
    interface = std::make_unique<Go2Bridge>(m, d, sim_mutex);
  }
  interface->start();
  
  while (true)
  {
    sleep(1);
  }
}
//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// user keyboard callback
void user_key_cb(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act==GLFW_PRESS)
  {
    if(param::config.enable_elastic_band == 1) {
      if (key==GLFW_KEY_9) {
        elastic_band.enable_ = !elastic_band.enable_;
      } else if (key==GLFW_KEY_7 || key==GLFW_KEY_UP) {
        elastic_band.length_ -= 0.1;
      } else if (key==GLFW_KEY_8 || key==GLFW_KEY_DOWN) {
        elastic_band.length_ += 0.1;
      }
    }
    if(key==GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d);
      mj_forward(m, d);
    }
  }
}

// Minimal no-window UI adapter for --headless runs. Physics and the DDS
// bridge run exactly as in GUI mode; only window creation and the render
// loop are skipped, removing WSLg/window-scheduler interference.
class HeadlessAdapter : public mj::PlatformUIAdapter {
 public:
  std::pair<double, double> GetCursorPosition() const override { return {0.0, 0.0}; }
  double GetDisplayPixelsPerInch() const override { return 72.0; }
  std::pair<int, int> GetFramebufferSize() const override { return {1280, 720}; }
  std::pair<int, int> GetWindowSize() const override { return {1280, 720}; }
  bool IsGPUAccelerated() const override { return false; }
  void PollEvents() override {}
  void SetClipboardString(const char* /*text*/) override {}
  void SetVSync(bool /*enabled*/) override {}
  void SetWindowTitle(const char* /*title*/) override {}
  bool ShouldCloseWindow() const override { return false; }
  void SwapBuffers() override {}
  void ToggleFullscreen() override {}
  bool IsLeftMouseButtonPressed() const override { return false; }
  bool IsMiddleMouseButtonPressed() const override { return false; }
  bool IsRightMouseButtonPressed() const override { return false; }
  bool IsAltKeyPressed() const override { return false; }
  bool IsCtrlKeyPressed() const override { return false; }
  bool IsShiftKeyPressed() const override { return false; }
  bool IsMouseButtonDownEvent(int /*act*/) const override { return false; }
  bool IsKeyDownEvent(int /*act*/) const override { return false; }
  int TranslateKeyCode(int /*key*/) const override { return 0; }
  mjtButton TranslateMouseButton(int /*button*/) const override { return mjBUTTON_NONE; }
};

// run event loop
int main(int argc, char **argv)
{

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  ::signal(SIGTERM, RequestShutdown);
  ::signal(SIGINT, RequestShutdown);
  mjv_defaultPerturb(&pert);

  // Load simulation configuration
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path();
  param::config.load_from_yaml(proj_dir / "config.yaml");
  param::helper(argc, argv);
  if (const char *value = std::getenv("TROT_INITIAL_X_M"))
    param::config.initial_x_m = std::strtod(value, nullptr);
  if (const char *value = std::getenv("TROT_INITIAL_Y_M"))
    param::config.initial_y_m = std::strtod(value, nullptr);
  if(param::config.robot_scene.is_relative()) {
    param::config.robot_scene = proj_dir.parent_path() / "unitree_robots" / param::config.robot / param::config.robot_scene;
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
    param::config.headless
      ? std::unique_ptr<mj::PlatformUIAdapter>(new HeadlessAdapter())
      : std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ true);

  // WSLg rendering can consume most of a 60 Hz refresh slice, starving the
  // physics thread.  Keep the normal viewer unchanged, but allow recording
  // runs to reserve a larger physics budget without changing simulation time.
  if (const char *refresh_env = std::getenv("UNITREE_MUJOCO_REFRESH_RATE"))
  {
    char *end = nullptr;
    const long refresh = std::strtol(refresh_env, &end, 10);
    if (end != refresh_env && *end == '\0' && refresh >= 10 && refresh <= 240)
      sim->refresh_rate = static_cast<int>(refresh);
  }

  if (!param::config.headless) {
    auto* glfw_adapter = static_cast<mj::GlfwAdapter*>(sim->platform_ui.get());
    glfwSetWindowPos(glfw_adapter->window_, 80, 80);
    glfwShowWindow(glfw_adapter->window_);
    glfwFocusWindow(glfw_adapter->window_);
  }

  std::thread unitree_thread(UnitreeSdk2BridgeThread, &sim->mtx);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());
  if (param::config.headless) {
    // headless: no window, no render loop. The main thread (a) consumes
    // load requests that RenderLoop would otherwise handle (Simulate::Load
    // waits for loadrequest to return to 0), (b) watches for shutdown
    // (SIGTERM/SIGINT -> shutdown_requested) or a physics-loop exit.
    // PhysicsLoop may be blocked on the sim mutex held by the DDS bridge
    // while a controller is attached, so do not join it; flush the
    // ground-truth log and exit directly.
    std::printf("headless mode: waiting for physics loop (SIGTERM to stop)\n");
    while (!sim->exitrequest.load()) {
      if (shutdown_requested) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("headless mode: exit requested, flushing ground-truth log and exiting\n");
    std::fflush(stdout);
    ground_truth_logger.Close();
    _exit(0);
  } else {
    // start simulation UI loop (blocking call)
    glfwSetKeyCallback(static_cast<mj::GlfwAdapter*>(sim->platform_ui.get())->window_,user_key_cb);
    sim->RenderLoop();
  }
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
