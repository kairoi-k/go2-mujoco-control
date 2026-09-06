#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
namespace go2_trot {
struct NominalComHeight {
    bool valid = false;
    double base_world_z = std::numeric_limits<double>::quiet_NaN();
    double com_world_z = std::numeric_limits<double>::quiet_NaN();
};
// A posture reference, NOT an IK/dynamics feasibility certificate. Foot sites
// and COM offset are world-Z quantities; neutral sites use desired level body
// orientation. The lowest eligible support/upcoming site preserves nominal
// extension on split levels instead of stretching the lower leg to the upper.
inline NominalComHeight MakeNominalComHeight(
    const std::array<double, 4>& site_world_z,
    const std::array<double, 4>& neutral_body_z,
    const std::array<bool, 4>& eligible,
    double com_minus_base_world_z) {
    NominalComHeight out;
    if (!std::isfinite(com_minus_base_world_z)) return out;
    double base = std::numeric_limits<double>::infinity();
    for (unsigned leg = 0; leg < 4; ++leg) {
        if (!eligible[leg]) continue;
        if (!std::isfinite(site_world_z[leg]) ||
            !std::isfinite(neutral_body_z[leg]) || neutral_body_z[leg] >= 0)
            return out;
        base = std::min(base, site_world_z[leg] - neutral_body_z[leg]);
    }
    if (!std::isfinite(base)) return out;
    out.base_world_z = base;
    out.com_world_z = base + com_minus_base_world_z;
    out.valid = std::isfinite(out.com_world_z);
    return out;
}
} // namespace go2_trot
