#include <array>
#include <cstddef>
#include <iostream>

#include "contact_state_filter.h"

namespace {
using Mask = std::array<bool, go2::kLegCount>;
int Bits(const Mask &mask)
{
    int bits = 0;
    for (std::size_t leg = 0; leg < mask.size(); ++leg)
        if (mask[leg]) bits |= 1 << static_cast<int>(leg);
    return bits;
}
void Emit(const char *scenario, int step, const Mask &raw,
          const Mask &filtered, const Mask &planned,
          const go2_control::ContactFusionResult &result)
{
    std::cout << scenario << "," << step << "," << Bits(raw) << ","
              << Bits(filtered) << "," << Bits(planned) << ","
              << Bits(result.fused_contact) << "," << result.reason << ","
              << static_cast<int>(result.fallback_stage) << ","
              << (result.guard_active ? 1 : 0) << ","
              << result.grace_remaining_ticks << "\n";
}
}

int main()
{
    const Mask stable{true, true, true, false};
    const Mask reduced{true, true, false, false};
    const Mask planned_only{false, false, false, true};
    const Mask none{false, false, false, false};
    go2_control::MeasuredContactFusion fusion;
    std::cout << "scenario,step,raw_mask,filtered_mask,planned_mask,fused_mask,reason,fallback_stage,guard,grace_remaining\n";

    auto run = [&](const char *name, int step, const Mask &raw,
                   const Mask &filtered, const Mask &planned, bool valid = true) {
        Emit(name, step, raw, filtered, planned,
             fusion.Update(filtered, valid, true));
    };
    run("stable", 0, stable, stable, planned_only);
    run("noisy-off", 1, reduced, reduced, planned_only);
    run("recovery", 2, stable, stable, planned_only);
    run("early-touchdown", 3, stable, reduced, planned_only);
    run("late-touchdown", 4, reduced, reduced, planned_only);
    run("grace-expiry", 5, reduced, reduced, planned_only);
    for (int age = 6; age <= 30; ++age)
        run("grace-expiry", age, none, reduced, planned_only);
    run("reset-before", 31, none, none, planned_only);
    fusion.Reset();
    const auto reset = fusion.Update(none, true, true);
    Emit("reset-after", 32, none, none, planned_only, reset);
    const auto planned_never_measured = fusion.Update(none, true, true);
    Emit("planned-never-measured", 33, planned_only, none,
         planned_only, planned_never_measured);
    std::cout << "identity-mismatch,34,0,0," << Bits(planned_only)
              << ",0,adapter-invalid-or-provenance-mismatch,0,1,0\n";
    return 0;
}
