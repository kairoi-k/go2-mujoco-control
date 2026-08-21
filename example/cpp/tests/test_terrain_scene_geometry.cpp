#include <cmath>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#ifndef TERRAIN_BARRIER_SCENE
#error "TERRAIN_BARRIER_SCENE must be defined"
#endif
#ifndef TERRAIN_STAIR_SCENE
#error "TERRAIN_STAIR_SCENE must be defined"
#endif

namespace
{

bool CheckBox(
    const mjModel *model,
    const char *name,
    double expected_x,
    double expected_z)
{
    const int geom_id = mj_name2id(model, mjOBJ_GEOM, name);
    if (geom_id < 0 || model->geom_type[geom_id] != mjGEOM_BOX ||
        model->geom_contype[geom_id] == 0 ||
        model->geom_conaffinity[geom_id] == 0 ||
        model->geom_bodyid[geom_id] != 0)
    {
        return false;
    }
    return std::abs(model->geom_pos[3 * geom_id] - expected_x) < 1e-9 &&
           std::abs(model->geom_pos[3 * geom_id + 2] - expected_z) < 1e-9;
}

bool LoadAndCheckBarrier()
{
    char error[1024] = {};
    mjModel *model = mj_loadXML(TERRAIN_BARRIER_SCENE, nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cerr << "barrier scene load failed: " << error << "\n";
        return false;
    }
    const bool ok = CheckBox(model, "terrain_barrier", 0.58, 0.075);
    mj_deleteModel(model);
    return ok;
}

bool LoadAndCheckStairs()
{
    char error[1024] = {};
    mjModel *model = mj_loadXML(TERRAIN_STAIR_SCENE, nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cerr << "stair scene load failed: " << error << "\n";
        return false;
    }
    bool ok = true;
    for (int i = 1; i <= 4; ++i)
    {
        const std::string name = "terrain_step_" + std::to_string(i);
        ok = ok && CheckBox(
            model,
            name.c_str(),
            0.42 + 0.24 * static_cast<double>(i - 1),
            0.05 + 0.10 * static_cast<double>(i - 1));
    }
    mj_deleteModel(model);
    return ok;
}

}  // namespace

int main()
{
    if (!LoadAndCheckBarrier() || !LoadAndCheckStairs())
    {
        std::cerr << "terrain scene geometry checks failed\n";
        return 1;
    }
    std::cout << "terrain scene geometry checks passed: physical barrier and four stairs.\n";
    return 0;
}
