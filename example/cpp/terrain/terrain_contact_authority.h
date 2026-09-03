#pragma once

// Single contact authority for the Stage-C execution window.
//
// Gait, SRBD-MPC and ID-WBC must consume the same contact topology at every
// control instant. This module is the one producer of that topology inside
// the terrain transfer window. Outside the window it reports kPhaseOne and
// every consumer keeps its established Phase-1 path.
//
// Rules (deliberately small, matching the accepted fallback chain):
//  - !stage_c_window                        -> kPhaseOne (consumers unchanged)
//  - usable adopted plan, guard inactive    -> kPlanned
//  - guard active, age < N+5                -> kPlanned with knot-0 safety
//    boundary (topology() = fused contact)
//  - guard active, age >= N+5               -> kSafeHold: topology stays on
//    the fused/robust support set and nominal Phase-1 gait topology is
//    forbidden from reinjecting into any consumer until measured support is
//    restored. When support is restored the authority returns to kPlanned if
//    the adapter still owns a usable plan, otherwise to kPhaseOne.
//  - no usable plan but guard active        -> kSafeHold (same fused hold)
//  - no usable plan, no guard               -> kPhaseOne
//
// kSafeHold also carries the plan identity it retired (safe_hold_plan_id /
// safe_hold_epoch) so MPC can atomically invalidate its prior plan state
// instead of mixing a stale horizon with the hold topology.

#include <array>
#include <cstddef>
#include <cstdint>

#include "go2_forward_kinematics.h"
#include "terrain_control_interface.h"
#include "terrain_motion_plan.h"

namespace go2_terrain
{

enum class ContactAuthorityMode : int
{
    kPhaseOne = 0,
    kPlanned = 1,
    kSafeHold = 2,
};

class TerrainContactAuthority final
{
public:
    // Matches MeasuredContactFusion::kSafeStopTicks.
    static constexpr std::size_t kSafeStopAgeTicks = 5;

    void Reset() noexcept
    {
        mode_ = ContactAuthorityMode::kPhaseOne;
        topology_.fill(false);
        safe_hold_plan_id_ = 0;
        safe_hold_epoch_ = 0;
    }

    void Update(
        bool stage_c_window,
        bool adapter_using_plan,
        bool guard_active,
        std::size_t guard_age_ticks,
        const std::array<bool, go2::kLegCount> &fused_contact,
        const std::array<bool, go2::kLegCount> &measured_contact,
        const TerrainMotionPlan *adopted_plan,
        double now_s) noexcept
    {
        if (!stage_c_window)
        {
            Reset();
            return;
        }
        const bool plan_usable = adapter_using_plan && adopted_plan != nullptr &&
            adopted_plan->identity.usable_at(now_s) &&
            adopted_plan->contact_schedule.valid(
                adopted_plan->horizon_knots);
        const bool safe_stop = guard_active &&
            guard_age_ticks >= kSafeStopAgeTicks;
        if (safe_stop)
        {
            // N+5: atomically retire the plan from the contact authority and
            // freeze the fused safe topology. Nominal reinjection is
            // forbidden by construction: FillHorizon keeps this topology for
            // the whole preview and the gait adapter keeps stance hold.
            mode_ = ContactAuthorityMode::kSafeHold;
            topology_ = fused_contact;
            safe_hold_plan_id_ =
                plan_usable ? adopted_plan->plan_id : 0;
            safe_hold_epoch_ =
                plan_usable ? adopted_plan->plan_epoch : 0;
            return;
        }
        if (plan_usable)
        {
            mode_ = ContactAuthorityMode::kPlanned;
            // While the guard is active (N..N+1) knot zero is the fused
            // measured support set; future knots stay on the atomic plan.
            topology_ = guard_active
                ? fused_contact
                : adopted_plan->contact_schedule.planned_contact[0];
            safe_hold_plan_id_ = 0;
            safe_hold_epoch_ = 0;
            return;
        }
        if (guard_active)
        {
            // No usable plan but the window is still protecting support:
            // keep the fused hold instead of falling back to Phase-1 nominal.
            mode_ = ContactAuthorityMode::kSafeHold;
            topology_ = fused_contact;
            safe_hold_plan_id_ = 0;
            safe_hold_epoch_ = 0;
            return;
        }
        mode_ = ContactAuthorityMode::kPhaseOne;
        topology_ = measured_contact;
        safe_hold_plan_id_ = 0;
        safe_hold_epoch_ = 0;
    }

    ContactAuthorityMode mode() const noexcept { return mode_; }

    // Knot-0 topology: planned row, or the fused safety boundary while the
    // guard is active, or the frozen fused hold in kSafeHold.
    const std::array<bool, go2::kLegCount> &topology() const noexcept
    {
        return topology_;
    }

    // Fill a consumer horizon. kSafeHold fills the whole horizon with the
    // frozen topology. kPlanned fills from the adopted plan's atomic contact
    // schedule and re-applies topology() as the knot-0 authority. kPhaseOne
    // returns false and the caller keeps its Phase-1 path.
    template <std::size_t Horizon>
    bool FillHorizon(
        std::array<std::array<bool, go2::kLegCount>, Horizon> &contact,
        int horizon,
        double now_s,
        double plan_knot_dt_s,
        double consumer_knot_dt_s,
        const TerrainMotionPlan *adopted_plan) const noexcept
    {
        if (horizon <= 0 ||
            horizon > static_cast<int>(Horizon))
            return false;
        if (mode_ == ContactAuthorityMode::kSafeHold)
        {
            for (int k = 0; k < horizon; ++k)
                contact[static_cast<std::size_t>(k)] = topology_;
            return true;
        }
        if (mode_ == ContactAuthorityMode::kPlanned &&
            adopted_plan != nullptr &&
            adopted_plan->contact_schedule.valid(
                adopted_plan->horizon_knots))
        {
            std::array<std::size_t, kTerrainPlanMaxKnots> indices{};
            if (!BuildTerrainPlanHorizonIndices(
                    *adopted_plan, now_s, plan_knot_dt_s,
                    consumer_knot_dt_s,
                    static_cast<std::size_t>(horizon), indices))
                return false;
            for (int k = 0; k < horizon; ++k)
            {
                contact[static_cast<std::size_t>(k)] =
                    adopted_plan->contact_schedule.planned_contact[
                        indices[static_cast<std::size_t>(k)]];
            }
            contact[0] = topology_;
            return true;
        }
        return false;
    }

    std::uint64_t safe_hold_plan_id() const noexcept
    {
        return safe_hold_plan_id_;
    }
    std::uint64_t safe_hold_epoch() const noexcept
    {
        return safe_hold_epoch_;
    }

private:
    ContactAuthorityMode mode_ = ContactAuthorityMode::kPhaseOne;
    std::array<bool, go2::kLegCount> topology_{};
    std::uint64_t safe_hold_plan_id_ = 0;
    std::uint64_t safe_hold_epoch_ = 0;
};

} // namespace go2_terrain
