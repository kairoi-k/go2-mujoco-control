// Pure synthetic 5 cm riser foot-height algebra witness.
// It uses production interpolation helpers and mirrors gait execution algebra.
// It does NOT call full CheckSwingClearance/IK/model validation.
// A full checker pass must not be inferred from this witness.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "terrain_feasibility.h"

struct P { double x, y, z; };
static double E(double s) { return go2_terrain::TerrainSwingEase(s); }
static double Q(double u, double peak) {
    return go2_terrain::TerrainSwingProfile(u, peak);
}
// Mirrors the foot-height algebra in CheckSwingClearance (terrain_feasibility.h:
// 661-673), with path progress equal to TerrainSwingPathProgress (288-294).
static P CheckerAlgebra(double u, P a, P b, double lift, double peak) {
    const double s = E(u);
    return {a.x + s * (b.x - a.x), a.y + s * (b.y - a.y),
            a.z + s * (b.z - a.z) + lift * Q(u, peak)};
}
// Mirrors the current execution in trot_experiment_gait.cpp:1586-1608.
static P GaitAlgebra(double u, P a, P b, double target, double nominal,
                     double start_u) {
    const double s = E(u);
    P n{a.x + s * (b.x - a.x), a.y + s * (b.y - a.y),
        a.z + s * (b.z - a.z) + nominal * Q(u, 0.5)};
    const double h = E((u - start_u) / std::max(1.0e-3, 1.0 - start_u));
    P out{n.x + (b.x - n.x) * h, n.y + (b.y - n.y) * h,
          n.z + (b.z - n.z) * h};
    out.z += std::max(0.0, target - nominal) * Q(u, 0.5);
    return out;
}

int main() {
    const double cli_lift = .200, effective_kernel_lift = .035, required = .080;
    const double current_extra = std::max(0., required - std::max(cli_lift, effective_kernel_lift));
    const double effective_extra = std::max(0., required - effective_kernel_lift);
    assert(current_extra == 0.);
    assert(std::abs(effective_extra - .045) < 1.e-12);
    std::cout << "lift_accounting current_extra_m=" << current_extra
              << " effective_extra_m=" << effective_extra << "\n";
    // Controlled optimistic nominal path, not a replay of the full gait kernel.

    const P a{0.70, -0.12, 0.00}, b{1.05, -0.12, 0.05};
    constexpr double target = 0.080, nominal = 0.035, peak = 0.25;
    constexpr double clearance = 0.030, riser_x = 0.755, riser_z = 0.050;
    constexpr double start_u = 0.0;  // every sampled phase is reachable
    double checker_min = 1e9, gait_min = 1e9, worst_u = 0.0;
    for (int i = 0; i <= 1000; ++i) {
        const double u = static_cast<double>(i) / 1000.0;
        const P c = CheckerAlgebra(u, a, b, target, peak);
        const P g = GaitAlgebra(u, a, b, target, nominal, start_u);
        const double hc = c.x >= riser_x ? riser_z : 0.0;
        const double hg = g.x >= riser_x ? riser_z : 0.0;
        const double req = clearance * Q(u, peak);
        checker_min = std::min(checker_min, c.z - hc - req);
        const double gm = g.z - hg - req;
        if (gm < gait_min) { gait_min = gm; worst_u = u; }
    }
    // These are only foot-height algebra predicates, not full checker claims.
    assert(checker_min >= -1e-12);
    assert(gait_min < -0.020);
    std::cout << std::setprecision(12)
              << "checker_foot_height_algebra_clears="
              << (checker_min >= -1e-12 ? "true" : "false")
              << " gait_foot_height_algebra_fails="
              << (gait_min < -0.020 ? "true" : "false")
              << " full_check_swing_clearance_executed=false"
              << " checker_min_margin_m=" << checker_min
              << " gait_min_margin_m=" << gait_min
              << " worst_u=" << worst_u << std::endl;
}
