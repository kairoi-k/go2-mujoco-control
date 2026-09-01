#include "terrain_plan_before_motion.h"
#include <iostream>

using go2_terrain::PlanBeforeMotionGate;
using go2_terrain::PlanBeforeMotionObservation;

namespace {
bool Check(bool value, const char *message)
{
    if (!value) std::cerr << "FAIL: " << message << "\n";
    return value;
}
PlanBeforeMotionObservation Ready(std::uint64_t boundary = 1)
{
    PlanBeforeMotionObservation observation;
    observation.captured_stand = true;
    observation.zero_motion = true;
    observation.map_valid = true;
    observation.coverage_valid = true;
    observation.filtered_measured_support_valid = true;
    observation.filtered_measured_support = {true, true, true, true};
    observation.family_a_complete_timed_plan = true;
    observation.publishable_identity_epoch = true;
    observation.published_identity = {7, 8, 9};
    observation.whole_snapshot_published = true;
    observation.adapter_exactly_adopted = true;
    observation.adopted_identity = {7, 8, 9};
    observation.gait_same_identity = true;
    observation.gait_identity = {7, 8, 9};
    observation.srbd_same_identity = true;
    observation.srbd_identity = {7, 8, 9};
    observation.boundary_id = boundary;
    observation.nonzero_request = true;
    return observation;
}
}
int main()
{
    PlanBeforeMotionGate gate(true);
    if (!Check(gate.state() == go2_terrain::PlanBeforeMotionState::kWarmHold,
               "E1 did not start in warm hold") ||
        !Check(!gate.motion_allowed(), "warm hold allowed motion")) return 1;
    gate.ObserveWarmHold(true, true);
    if (!Check(gate.state() == go2_terrain::PlanBeforeMotionState::kReadyToArm,
               "captured stand did not become ready")) return 1;
    auto zero = Ready(0);
    zero.nonzero_request = false;
    if (!Check(!gate.OnCommandBoundary(zero) &&
                   gate.state() == go2_terrain::PlanBeforeMotionState::kReadyToArm,
               "zero/no-command boundary armed or stopped")) return 1;
    if (!Check(gate.OnCommandBoundary(Ready()), "ready plan did not arm") ||
        !Check(gate.motion_allowed(), "armed plan disallowed motion")) return 1;
    if (!Check(gate.OnCommandBoundary(Ready()),
               "duplicate boundary was not idempotent")) return 1;
    auto armed_replacement = Ready(2);
    armed_replacement.published_identity.plan_id++;
    if (!Check(!gate.OnCommandBoundary(armed_replacement) && gate.safe_stop() &&
                   gate.reason() == "replacement-identity-mismatch",
               "replacement after arm was accepted")) return 1;

    auto mismatch = Ready(2);
    mismatch.adopted_identity.plan_epoch++;
    PlanBeforeMotionGate mismatch_gate(true);
    mismatch_gate.ObserveWarmHold(true, true);
    if (!Check(!mismatch_gate.OnCommandBoundary(mismatch) &&
                   mismatch_gate.safe_stop() &&
                   mismatch_gate.reason() == "adapter-identity-mismatch",
               "identity/epoch mismatch was accepted")) return 1;
    auto replacement = Ready(2);
    replacement.published_identity.plan_id++;
    PlanBeforeMotionGate replacement_gate(true);
    replacement_gate.ObserveWarmHold(true, true);
    if (!Check(!replacement_gate.OnCommandBoundary(replacement) &&
                   replacement_gate.safe_stop(),
               "replacement snapshot was accepted")) return 1;
    for (int failure = 0; failure < 4; ++failure)
    {
        auto bad = Ready(static_cast<std::uint64_t>(10 + failure));
        if (failure == 0) bad.filtered_measured_support = {false, false, true, false};
        if (failure == 1) bad.map_valid = false;
        if (failure == 2) bad.family_a_complete_timed_plan = false;
        if (failure == 3) bad.deadline_ok = false;
        PlanBeforeMotionGate bad_gate(true);
        bad_gate.ObserveWarmHold(true, true);
        if (!Check(!bad_gate.OnCommandBoundary(bad) && bad_gate.safe_stop(),
                   "unknown/negative/deadline gate did not safe-stop")) return 1;
    }
    PlanBeforeMotionGate no_stand(true);
    no_stand.ObserveWarmHold(false, true);
    if (!Check(!no_stand.OnCommandBoundary(Ready(30)) && no_stand.safe_stop(),
               "motion started without captured stand")) return 1;
    PlanBeforeMotionGate p1(true);
    p1.ObserveWarmHold(true, true);
    if (!Check(p1.OnCommandBoundary(Ready(40)) && p1.motion_allowed(),
               "P1 could not arm") ||
        !Check(p1.ValidateOnlyAfterArm() && p1.safe_stop() &&
                   !p1.motion_allowed(),
               "P1 validate-only did not stop before command application")) return 1;
    PlanBeforeMotionGate off(false);
    off.ObserveWarmHold(false, false);
    auto off_observation = Ready(50);
    if (!Check(off.OnCommandBoundary(off_observation) && !off.safe_stop(), "flag-off was not Phase1-equivalent")) return 1;
    std::cout << "Plan-before-motion E1 checks passed.\n";
    return 0;
}
