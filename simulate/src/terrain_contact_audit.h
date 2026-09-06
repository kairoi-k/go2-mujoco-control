#pragma once
#include <mujoco/mujoco.h>
#include <cmath>
#include <limits>
#include <algorithm>

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

// Conservative world-X rear bound of all collision geoms in the robot subtree.
// MuJoCo geom_rbound encloses the geom for every orientation. Unlike a base
// origin proxy, this also covers knees/calves trailing behind the feet.
inline double RobotCollisionRearBound(
    const mjModel* model, const mjData* data, int root_body) {
  double bound = std::numeric_limits<double>::infinity();
  for (int geom = 0; geom < model->ngeom; ++geom) {
    if (!(model->geom_contype[geom] || model->geom_conaffinity[geom])) continue;
    int body = model->geom_bodyid[geom];
    while (body > 0 && body != root_body) body = model->body_parentid[body];
    if (body != root_body || root_body <= 0) continue;
    const double radius = model->geom_rbound[geom];
    const double x = data->geom_xpos[3 * geom];
    if (!std::isfinite(radius) || radius < 0 || !std::isfinite(x))
      return std::numeric_limits<double>::quiet_NaN();
    bound = std::min(bound, x - radius);
  }
  return std::isfinite(bound) ? bound : std::numeric_limits<double>::quiet_NaN();
}
