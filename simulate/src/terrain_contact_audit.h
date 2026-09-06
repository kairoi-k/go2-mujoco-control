#pragma once
#include <mujoco/mujoco.h>
#include <cmath>

// Offline evidence only; never a controller input. Force is on the robot.
struct TerrainContactAudit {
  bool top;
  double top_force_world_z;
  double nontop_force_norm;
};
inline TerrainContactAudit AuditTerrainContact(
    const mjModel* model, const mjData* data, const mjContact& contact,
    int terrain_side, const mjtNum* force) {
  const int geom = contact.geom[terrain_side];
  const double sign = terrain_side == 0 ? 1.0 : -1.0;
  const mjtNum* rotation = data->geom_xmat + 9 * geom;
  const double normal_dot_top = sign * (
      contact.frame[0] * rotation[2] + contact.frame[1] * rotation[5] +
      contact.frame[2] * rotation[8]);
  const bool top = model->geom_type[geom] == mjGEOM_BOX && normal_dot_top >= 0.99;
  const double world_z = sign * (contact.frame[2] * force[0] +
      contact.frame[5] * force[1] + contact.frame[8] * force[2]);
  return {top, top ? world_z : 0.0,
      top ? 0.0 : std::hypot(force[0], std::hypot(force[1], force[2]))};
}
