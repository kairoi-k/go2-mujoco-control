// Independent arithmetic probe of audited expressions, not a production test.
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
int main() {
    constexpr float resolution = 0.05f;
    const double half = 0.5 * static_cast<double>(resolution) - 0.025;
    const double exact_half = 0.5 * 0.05 - 0.025;
    assert(half > 0.0 && half < 1e-8);
    assert(exact_half == 0.0);
    const bool terrain_actuation = false, high_state = true, shadow = true;
    const bool actual_anchor_initialized = terrain_actuation && high_state;
    assert(shadow && !actual_anchor_initialized);
    std::cout << std::setprecision(18)
              << "promoted_resolution_m=" << static_cast<double>(resolution) << '\n'
              << "region_half_width_m=" << half << '\n'
              << "shadow_anchor_can_initialize=" << actual_anchor_initialized << '\n'
              << "PASS: arithmetic/guard witnesses only; no Atlas controller build.\n";
}
