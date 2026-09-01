#pragma once

#include <cstdint>
#include <string>

#include "terrain_motion_plan.h"

namespace go2_terrain
{

struct TerrainBootstrapPlanKey
{
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    std::uint64_t map_epoch = 0;
    std::uint64_t input_hash = 0;
    std::string frame_id;

    bool valid() const noexcept
    {
        return plan_id != 0 && plan_epoch != 0 && map_epoch != 0 &&
            input_hash != 0 && !frame_id.empty();
    }
};

inline TerrainBootstrapPlanKey TerrainBootstrapKey(
    const TerrainMotionPlan &plan)
{
    return TerrainBootstrapPlanKey{
        plan.plan_id, plan.plan_epoch, plan.map_epoch,
        plan.input_hash, plan.frame_id};
}

inline bool SameTerrainBootstrapKey(
    const TerrainBootstrapPlanKey &a,
    const TerrainBootstrapPlanKey &b) noexcept
{
    return a.valid() && b.valid() &&
        a.plan_id == b.plan_id &&
        a.plan_epoch == b.plan_epoch &&
        a.map_epoch == b.map_epoch &&
        a.input_hash == b.input_hash &&
        a.frame_id == b.frame_id;
}

struct TerrainBootstrapCandidate
{
    // These readiness bits come from the existing map/planner contracts. The
    // coordinator does not reinterpret planner margins or change thresholds.
    bool roi_ready = false;
    bool family_a_complete = false;
    TerrainBootstrapPlanKey key{};

    bool valid() const noexcept
    {
        return roi_ready && family_a_complete && key.valid();
    }
};

enum class TerrainBootstrapState : std::uint8_t
{
    kDisabled = 0,
    kHold,
    kObserve,
    kTransferHold,
    kC1Armed,
    kCrossing,
    kBrakeHold,
};

enum class TerrainBootstrapOwner : std::uint8_t
{
    kNone = 0,
    kC0,
    kC1,
};

struct TerrainC0Readiness
{
    // Existing perception/safety surfaces certify these conditions. No new
    // margins or scene-specific checks are introduced here.
    bool local_known_flat = false;
    bool swept_volume_clear = false;
    bool dstop_valid = false;

    bool valid() const noexcept
    {
        return local_known_flat && swept_volume_clear && dstop_valid;
    }
};

struct TerrainC1AdoptionEvidence
{
    bool publish_succeeded = false;
    bool adapter_adopted = false;
    bool deadline_ok = false;
    TerrainBootstrapPlanKey published{};
    TerrainBootstrapPlanKey adopted{};
    TerrainBootstrapPlanKey gait{};
    TerrainBootstrapPlanKey srbd{};

    bool coherent_with(const TerrainBootstrapPlanKey &candidate) const noexcept
    {
        return publish_succeeded && adapter_adopted && deadline_ok &&
            SameTerrainBootstrapKey(candidate, published) &&
            SameTerrainBootstrapKey(candidate, adopted) &&
            SameTerrainBootstrapKey(candidate, gait) &&
            SameTerrainBootstrapKey(candidate, srbd);
    }
};

struct TerrainBootstrapInput
{
    bool enabled = false;
    TerrainC0Readiness c0{};
    bool observation_motion_requested = false;
    TerrainBootstrapCandidate candidate{};
    bool legal_transfer_boundary = false;
    TerrainC1AdoptionEvidence c1{};
    bool crossing_started = false;
    bool crossing_complete = false;
};

struct TerrainBootstrapDecision
{
    TerrainBootstrapState state = TerrainBootstrapState::kDisabled;
    TerrainBootstrapOwner owner = TerrainBootstrapOwner::kNone;
    bool allow_observation_motion = false;
    bool request_hold = false;
    bool allow_c1_transition = false;
    bool request_brake_hold = false;
    TerrainBootstrapPlanKey armed_plan{};
    std::string reason;
};

// Minimal development-only ownership seam for the C0/C1 bootstrap contract.
// C0 never waits for C1: while C0 is valid it may perform bounded observation
// motion. C1 can take ownership only at a legal boundary after one complete
// Family-A snapshot is published/adopted and gait/SRBD report the exact same
// identity. Any C0 loss or identity disagreement fails closed to BRAKE_HOLD.
class TerrainBootstrapCoordinator final
{
public:
    TerrainBootstrapDecision Update(const TerrainBootstrapInput &input)
    {
        TerrainBootstrapDecision out;
        if (!input.enabled)
        {
            Reset();
            out.reason = "bootstrap_disabled";
            return out;
        }

        if (state_ == TerrainBootstrapState::kBrakeHold)
            return Brake("brake_hold_latched");

        if (!input.c0.valid())
        {
            state_ = TerrainBootstrapState::kBrakeHold;
            owner_ = TerrainBootstrapOwner::kC0;
            armed_plan_ = {};
            return Brake("c0_invalid");
        }

        if (state_ == TerrainBootstrapState::kC1Armed ||
            state_ == TerrainBootstrapState::kCrossing)
        {
            if (!armed_plan_.valid())
            {
                state_ = TerrainBootstrapState::kBrakeHold;
                owner_ = TerrainBootstrapOwner::kC0;
                return Brake("armed_identity_missing");
            }
            if (input.crossing_complete)
            {
                state_ = TerrainBootstrapState::kHold;
                owner_ = TerrainBootstrapOwner::kC0;
                armed_plan_ = {};
                return Hold("crossing_complete");
            }
            if (input.crossing_started)
                state_ = TerrainBootstrapState::kCrossing;
            return C1("c1_owns_transition");
        }

        if (!input.candidate.valid())
        {
            state_ = input.observation_motion_requested
                ? TerrainBootstrapState::kObserve
                : TerrainBootstrapState::kHold;
            owner_ = TerrainBootstrapOwner::kC0;
            if (input.observation_motion_requested)
                return Observe("c0_observation");
            return Hold("waiting_for_c1_candidate");
        }

        if (!input.legal_transfer_boundary)
        {
            state_ = TerrainBootstrapState::kTransferHold;
            owner_ = TerrainBootstrapOwner::kC0;
            return Hold("waiting_for_legal_transfer_boundary");
        }

        // At the transfer boundary, absent adoption evidence is a hold. Once
        // any publication/adoption witness appears, disagreement fails closed.
        const bool any_c1_evidence = input.c1.publish_succeeded ||
            input.c1.adapter_adopted || input.c1.published.valid() ||
            input.c1.adopted.valid() || input.c1.gait.valid() ||
            input.c1.srbd.valid();
        if (!input.c1.coherent_with(input.candidate.key))
        {
            if (!any_c1_evidence)
            {
                state_ = TerrainBootstrapState::kTransferHold;
                owner_ = TerrainBootstrapOwner::kC0;
                return Hold("awaiting_atomic_c1_adoption");
            }
            state_ = TerrainBootstrapState::kBrakeHold;
            owner_ = TerrainBootstrapOwner::kC0;
            armed_plan_ = {};
            return Brake("c1_identity_or_deadline_mismatch");
        }

        armed_plan_ = input.candidate.key;
        state_ = TerrainBootstrapState::kC1Armed;
        owner_ = TerrainBootstrapOwner::kC1;
        return C1("c1_armed");
    }

    void Reset() noexcept
    {
        state_ = TerrainBootstrapState::kDisabled;
        owner_ = TerrainBootstrapOwner::kNone;
        armed_plan_ = {};
    }

    TerrainBootstrapState state() const noexcept { return state_; }
    TerrainBootstrapOwner owner() const noexcept { return owner_; }
    const TerrainBootstrapPlanKey &armed_plan() const noexcept
    {
        return armed_plan_;
    }

private:
    TerrainBootstrapDecision Observe(const char *reason) const
    {
        TerrainBootstrapDecision out;
        out.state = state_;
        out.owner = owner_;
        out.allow_observation_motion = true;
        out.reason = reason;
        return out;
    }

    TerrainBootstrapDecision Hold(const char *reason) const
    {
        TerrainBootstrapDecision out;
        out.state = state_;
        out.owner = owner_;
        out.request_hold = true;
        out.armed_plan = armed_plan_;
        out.reason = reason;
        return out;
    }

    TerrainBootstrapDecision C1(const char *reason) const
    {
        TerrainBootstrapDecision out;
        out.state = state_;
        out.owner = TerrainBootstrapOwner::kC1;
        out.allow_c1_transition = true;
        out.armed_plan = armed_plan_;
        out.reason = reason;
        return out;
    }

    TerrainBootstrapDecision Brake(const char *reason) const
    {
        TerrainBootstrapDecision out;
        out.state = TerrainBootstrapState::kBrakeHold;
        out.owner = TerrainBootstrapOwner::kC0;
        out.request_brake_hold = true;
        out.reason = reason;
        return out;
    }

    TerrainBootstrapState state_ = TerrainBootstrapState::kDisabled;
    TerrainBootstrapOwner owner_ = TerrainBootstrapOwner::kNone;
    TerrainBootstrapPlanKey armed_plan_{};
};

} // namespace go2_terrain
