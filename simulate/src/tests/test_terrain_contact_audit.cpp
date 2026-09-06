#include "terrain_contact_audit.h"
#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>

int main() {
  int checks = 0;
  for (const bool top : {true, false}) {
    const std::string position = top ? "0 0 .069" : ".119 0 .025";
    const std::string xml = "<mujoco><worldbody><geom name='step' type='box' size='.1 .1 .025' pos='0 0 .025'/><body pos='" + position + "'><freejoint/><geom name='foot' type='sphere' size='.02' mass='1'/></body></worldbody></mujoco>";
    mjVFS vfs; mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "test.xml", xml.data(), xml.size());
    char error[1024]{};
    mjModel* m = mj_loadXML("test.xml", &vfs, error, sizeof(error));
    mj_deleteVFS(&vfs);
    if (!m) { std::cerr << error; return 1; }
    mjData* d = mj_makeData(m); mj_forward(m, d);
    if (d->ncon != 1) return 2;
    const mjContact& c = d->contact[0];
    const int terrain = mj_name2id(m, mjOBJ_GEOM, "step");
    const int side = c.geom[0] == terrain ? 0 : 1;
    mjtNum force[6]{}; mj_contactForce(m, d, 0, force);
    auto a = AuditTerrainContact(m, d, c, side, force);
    if (a.top != top || (top && a.top_force_world_z <= 1) ||
        (!top && a.nontop_force_norm <= 1)) return 3;
    // Independent force sign check: constraint generalized vertical force
    // is the full world contact force for the isolated free sphere.
    if (top && std::abs(a.top_force_world_z - d->qfrc_constraint[2]) > 1e-9) return 4;
    ++checks;
    mjContact reversed = c;
    std::swap(reversed.geom[0], reversed.geom[1]);
    for (int i=0; i<9; ++i) reversed.frame[i] = -reversed.frame[i];
    auto b = AuditTerrainContact(m, d, reversed, 1-side, force);
    if (a.top != b.top || std::abs(a.top_force_world_z-b.top_force_world_z)>1e-9 ||
        std::abs(a.nontop_force_norm-b.nontop_force_norm)>1e-9) return 5;
    ++checks;
    mj_deleteData(d); mj_deleteModel(m);
  }
  std::cout << checks << " real-contact and geom-order checks PASS\n";
}
