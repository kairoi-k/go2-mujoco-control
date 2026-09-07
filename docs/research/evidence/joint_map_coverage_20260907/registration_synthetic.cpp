#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>
#include "terrain_map_envelope.h"
#include "terrain_model.h"
using namespace go2_terrain;
TerrainMapEnvelope MakeSource() {
    TerrainMapEnvelope e;
    e.sequence = 9001;
    e.map_stamp_s = 10.0;
    e.frame_id = "base_link";
    e.resolution_m = 0.05;
    e.width = 32;
    e.height = 10;
    e.origin_m = {-0.45, -0.225};
    e.capture_position_world = {0.0, 0.0, 0.5};
    e.capture_yaw_rad = 0.0;
    const std::size_t n = static_cast<std::size_t>(e.width) * e.height;
    e.heights_m.assign(n, -0.5);
    e.observation_stamp_s.assign(n, 10.0);
    return e;
}
void Run(const char* label, double x, double y, double yaw) {
    const auto source = MakeSource();
    const auto reg = RegisterTerrainMap(source, 10.04, {x, y, 0.5}, yaw,
        kTerrainMapMaxAgeS, TerrainMapRegistrationPolicy::kRegisteredIntervalsV2);
    std::size_t reg_known = 0;
    if (reg.ok()) for (float z : reg.map.map.data()) if (std::isfinite(z)) ++reg_known;
    const auto built = reg.ok() ? BuildRegisteredTerrainModel(
        &reg.map, 10.04, 77, TerrainSource::kLidar) : TerrainModelBuildResult{};
    std::size_t model_known = 0;
    if (built.ok()) for (const auto& c : built.model.cells) if (c.known) ++model_known;
    std::printf("%s dx=%.9g dy=%.9g yaw=%.9g reg_ok=%d model_ok=%d reg_known=%zu model_known=%zu\n",
        label, x, y, yaw, reg.ok() ? 1 : 0, built.ok() ? 1 : 0, reg_known, model_known);
    if (reg.ok() && built.ok()) {
        std::printf("  unknown_indices=");
        for (std::size_t i = 0; i < built.model.cells.size(); ++i)
            if (!built.model.cells[i].known) std::printf(" %zu", i);
        std::printf("\n");
    }
}
int main() {
    Run("identity", 0, 0, 0);
    Run("dx_pos_1e-6", 1e-6, 0, 0);
    Run("dx_neg_1e-6", -1e-6, 0, 0);
    Run("dy_pos_1e-6", 0, 1e-6, 0);
    Run("dy_neg_1e-6", 0, -1e-6, 0);
    Run("dx_pos_1e-3", 1e-3, 0, 0);
    Run("dx_neg_1e-3", -1e-3, 0, 0);
    Run("dy_pos_1e-3", 0, 1e-3, 0);
    Run("dy_neg_1e-3", 0, -1e-3, 0);
    Run("dx_pos_.01", .01, 0, 0);
    Run("dx_neg_.01", -.01, 0, 0);
    Run("dy_pos_.01", 0, .01, 0);
    Run("dy_neg_.01", 0, -.01, 0);
    Run("yaw_pos_1e-6", 0, 0, 1e-6);
    Run("yaw_neg_1e-6", 0, 0, -1e-6);
    Run("yaw_pos_.001", 0, 0, .001);
    Run("yaw_neg_.001", 0, 0, -.001);
    Run("yaw_pos_.01", 0, 0, .01);
    Run("yaw_neg_.01", 0, 0, -.01);
    Run("actual_wall", -1.239e-6, -4e-9, -4e-8);
    Run("actual_state", -2.366e-6, -15e-9, -146e-9);
    Run("tiny_both_pos", 1e-6, 1e-6, 0);
    Run("tiny_both_neg", -1e-6, -1e-6, 0);
    Run("tiny_dx_neg_dy_pos", -1e-6, 1e-6, 0);
    Run("tiny_dx_pos_dy_neg", 1e-6, -1e-6, 0);
    Run("combined_pos", 1e-3, 1e-3, .001);
    Run("combined_neg", -1e-3, -1e-3, -.001);
}
