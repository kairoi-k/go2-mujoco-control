#include <iostream>

#include "terrain_bootstrap_coordinator.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

go2_terrain::TerrainBootstrapPlanKey Key(std::uint64_t plan_id = 11)
{
    go2_terrain::TerrainBootstrapPlanKey key;
    key.plan_id = plan_id;
    key.plan_epoch = 7;
    key.map_epoch = 19;
    key.input_hash = 0x1234abcdULL;
    key.frame_id = "world";
    return key;
}

go2_terrain::TerrainC0Readiness ReadyC0()
{
    return {true, true, true};
}

go2_terrain::TerrainBootstrapCandidate Candidate(
    const go2_terrain::TerrainBootstrapPlanKey &key)
{
    go2_terrain::TerrainBootstrapCandidate candidate;
    candidate.roi_ready = true;
    candidate.family_a_complete = true;
    candidate.key = key;
    return candidate;
}

go2_terrain::TerrainC1AdoptionEvidence Coherent(
    const go2_terrain::TerrainBootstrapPlanKey &key)
{
    go2_terrain::TerrainC1AdoptionEvidence evidence;
    evidence.publish_succeeded = true;
    evidence.adapter_adopted = true;
    evidence.deadline_ok = true;
    evidence.published = key;
    evidence.adopted = key;
    evidence.gait = key;
    evidence.srbd = key;
    return evidence;
}

} // namespace

int main()
{
    using go2_terrain::TerrainBootstrapOwner;
    using go2_terrain::TerrainBootstrapState;

    const auto key = Key();
    go2_terrain::TerrainBootstrapCoordinator coordinator;
    go2_terrain::TerrainBootstrapInput input;
    input.enabled = true;
    input.c0 = ReadyC0();
    input.observation_motion_requested = true;

    // C0 must not wait for C1. With a valid local certificate and no C1
    // candidate, bounded observation motion remains available.
    auto out = coordinator.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kObserve &&
                   out.owner == TerrainBootstrapOwner::kC0 &&
                   out.allow_observation_motion &&
                   !out.allow_c1_transition && !out.request_brake_hold,
               "C0 observation incorrectly waited for C1"))
        return 1;

    // A complete C1 candidate stops observation but cannot take ownership
    // before an explicit legal transfer boundary.
    input.candidate = Candidate(key);
    out = coordinator.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kTransferHold &&
                   out.owner == TerrainBootstrapOwner::kC0 &&
                   out.request_hold && !out.allow_observation_motion &&
                   !out.allow_c1_transition,
               "C1 candidate bypassed the legal transfer boundary"))
        return 1;

    // Reaching the boundary without an adoption witness is still a hold, not
    // permission to cross and not a fabricated adoption.
    input.legal_transfer_boundary = true;
    out = coordinator.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kTransferHold &&
                   out.owner == TerrainBootstrapOwner::kC0 &&
                   out.request_hold && !out.allow_c1_transition &&
                   !out.request_brake_hold,
               "missing C1 adoption evidence did not hold at boundary"))
        return 1;

    // The exact same immutable identity must be observed at publish, adapter,
    // gait and SRBD before the owner changes C0 -> C1.
    input.c1 = Coherent(key);
    out = coordinator.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kC1Armed &&
                   out.owner == TerrainBootstrapOwner::kC1 &&
                   out.allow_c1_transition &&
                   go2_terrain::SameTerrainBootstrapKey(out.armed_plan, key),
               "coherent C1 adoption did not atomically arm transition"))
        return 1;

    input.crossing_started = true;
    out = coordinator.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kCrossing &&
                   out.owner == TerrainBootstrapOwner::kC1 &&
                   out.allow_c1_transition,
               "armed C1 did not retain transition ownership"))
        return 1;

    // C0 remains the safety sentinel during C1 execution. Losing the local
    // certificate must immediately reclaim control into BRAKE_HOLD.
    input.c0.swept_volume_clear = false;
    out = coordinator.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kBrakeHold &&
                   out.owner == TerrainBootstrapOwner::kC0 &&
                   out.request_brake_hold && !out.allow_c1_transition,
               "C0 loss during crossing did not preempt C1"))
        return 1;

    // An identity mismatch is a safety fault once publication/adoption has
    // begun; it may never degrade into a permissive hold or cross command.
    go2_terrain::TerrainBootstrapCoordinator mismatch;
    input = {};
    input.enabled = true;
    input.c0 = ReadyC0();
    input.candidate = Candidate(key);
    input.legal_transfer_boundary = true;
    input.c1 = Coherent(key);
    input.c1.srbd = Key(12);
    out = mismatch.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kBrakeHold &&
                   out.owner == TerrainBootstrapOwner::kC0 &&
                   out.request_brake_hold && !out.allow_c1_transition,
               "mixed gait/SRBD identity did not fail closed"))
        return 1;

    // Deadline failure after publication is also a fail-closed mismatch.
    go2_terrain::TerrainBootstrapCoordinator late;
    input.c1 = Coherent(key);
    input.c1.deadline_ok = false;
    out = late.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kBrakeHold &&
                   out.request_brake_hold,
               "late C1 adoption did not fail closed"))
        return 1;

    // Disabling the development path resets the latch and exposes no motion.
    input.enabled = false;
    out = late.Update(input);
    if (!Check(out.state == TerrainBootstrapState::kDisabled &&
                   out.owner == TerrainBootstrapOwner::kNone &&
                   !out.allow_observation_motion &&
                   !out.allow_c1_transition && !out.request_brake_hold,
               "disabled bootstrap did not reset to inert state"))
        return 1;

    return 0;
}
