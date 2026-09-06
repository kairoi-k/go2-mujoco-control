#include "terrain_commitment_lifecycle.h"

#include <cmath>

namespace
{

bool Check(bool value)
{
    return value;
}

}  // namespace

int main()
{
    using go2_trot::DecideTerrainCommitment;

    // Initial swing: a usable plan may prepare a new target.
    const auto initial = DecideTerrainCommitment(
        true, true, false, false, false, false);
    if (!Check(initial.clear_latched_target && initial.prepare_allowed &&
               !initial.apply_swing_target && !initial.hold_stance_target))
        return 1;

    // Expiry during the same swing: keep applying the in-flight target.
    const auto expired = DecideTerrainCommitment(
        true, false, false, true, true, false);
    if (!Check(expired.apply_swing_target && !expired.prepare_allowed &&
               !expired.clear_latched_target))
        return 1;

    // A conflicting new plan cannot replace the in-flight target.
    const auto conflict = DecideTerrainCommitment(
        true, true, false, true, true, false);
    if (!Check(conflict.apply_swing_target && !conflict.prepare_allowed))
        return 1;

    // Without a committed target, an expired plan falls back to kernel feet.
    const auto no_plan = DecideTerrainCommitment(
        true, false, false, false, false, false);
    if (!Check(no_plan.clear_latched_target && !no_plan.prepare_allowed &&
               !no_plan.apply_swing_target))
        return 1;

    // Stance retains the target and records completion only on measured contact.
    const auto stance_wait = DecideTerrainCommitment(
        false, false, false, true, true, false);
    const auto stance_done = DecideTerrainCommitment(
        false, false, true, true, true, false);
    if (!Check(stance_wait.hold_stance_target && !stance_wait.completion &&
               stance_done.hold_stance_target && stance_done.completion))
        return 1;

    // The next liftoff clears the completed target before fallback/prepare.
    const auto next_swing = DecideTerrainCommitment(
        true, false, false, true, false, true);
    if (!Check(next_swing.clear_latched_target &&
               !next_swing.prepare_allowed &&
               !next_swing.apply_swing_target))
        return 1;

    // Independent pitch rotation: desired world Z remains fixed even though
    // target X differs from the currently commanded horizontal foot path.
    const double pitch = 0.25;
    const std::array<double,4> q{std::cos(pitch/2),0,std::sin(pitch/2),0};
    const go2::Vec3 base{1,2,.35}, foot{.15,-.10,-.30};
    go2::Vec3 held{};
    if (!go2_trot::HoldTerrainWorldHeight(base,q,foot,.075,held)) return 2;
    const double expected_x = base.x+std::cos(pitch)*foot.x+std::sin(pitch)*foot.z;
    const double actual_x = base.x+std::cos(pitch)*held.x+std::sin(pitch)*held.z;
    const double actual_z = base.z-std::sin(pitch)*held.x+std::cos(pitch)*held.z;
    if (std::abs(actual_x-expected_x)>1e-12 || std::abs(actual_z-.075)>1e-12 ||
        std::abs(held.y-foot.y)>1e-12) return 3;
    const auto wrong = go2_control::WorldToBody(base,q,{1.5,2,.075});
    const double wrong_world_z = base.z-std::sin(pitch)*foot.x+std::cos(pitch)*wrong.z;
    if (std::abs(wrong_world_z-.075)<.02) return 5;
    if (go2_trot::HoldTerrainWorldHeight(base,{0,0,0,0},foot,.075,held) ||
        go2_trot::HoldTerrainWorldHeight(base,q,foot,NAN,held)) return 4;
    return 0;
}
