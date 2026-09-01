#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include "terrain_motion_plan.h"
namespace go2_terrain {
enum class PlanBeforeMotionState : std::uint8_t { kDisabled=0, kWarmHold, kReadyToArm, kArmed, kSafeStop };
inline const char *PlanBeforeMotionStateName(PlanBeforeMotionState s) {
    switch (s) { case PlanBeforeMotionState::kWarmHold:return "warm-hold"; case PlanBeforeMotionState::kReadyToArm:return "ready-to-arm"; case PlanBeforeMotionState::kArmed:return "armed"; case PlanBeforeMotionState::kSafeStop:return "safe-stop"; default:return "disabled"; }
}
struct PlanBeforeMotionIdentity {
    std::uint64_t plan_id=0, plan_epoch=0, map_epoch=0;
    bool valid() const noexcept { return plan_id && plan_epoch && map_epoch; }
};
inline bool SamePlanBeforeMotionIdentity(const PlanBeforeMotionIdentity&a,const PlanBeforeMotionIdentity&b) noexcept { return a.valid()&&b.valid()&&a.plan_id==b.plan_id&&a.plan_epoch==b.plan_epoch&&a.map_epoch==b.map_epoch; }
struct PlanBeforeMotionObservation {
    bool captured_stand=false, zero_motion=false, map_valid=false, coverage_valid=false;
    bool filtered_measured_support_valid=false;
    std::array<bool,go2::kLegCount> filtered_measured_support{};
    bool family_a_complete_timed_plan=false, publishable_identity_epoch=false;
    PlanBeforeMotionIdentity published_identity{};
    bool whole_snapshot_published=false, adapter_exactly_adopted=false;
    PlanBeforeMotionIdentity adopted_identity{};
    bool gait_same_identity=false; PlanBeforeMotionIdentity gait_identity{};
    bool srbd_same_identity=false; PlanBeforeMotionIdentity srbd_identity{};
    bool deadline_ok=true; std::uint64_t boundary_id=0; bool nonzero_request=false;
};
class PlanBeforeMotionGate final {
public:
    explicit PlanBeforeMotionGate(bool enabled=false) { SetEnabled(enabled); }
    void SetEnabled(bool enabled) noexcept { enabled_=enabled; Reset(); }
    void Reset() noexcept { state_=enabled_?PlanBeforeMotionState::kWarmHold:PlanBeforeMotionState::kDisabled; reason_.clear(); have_boundary_=false; last_boundary_id_=0; armed_identity_={}; }
    bool enabled()const noexcept{return enabled_;} bool motion_allowed()const noexcept{return state_==PlanBeforeMotionState::kArmed;} bool safe_stop()const noexcept{return state_==PlanBeforeMotionState::kSafeStop;}
    PlanBeforeMotionState state()const noexcept{return state_;} const std::string& reason()const noexcept{return reason_;} PlanBeforeMotionIdentity armed_identity()const noexcept{return armed_identity_;}
    void ObserveWarmHold(bool captured_stand,bool zero_motion) noexcept { if(!enabled_||safe_stop()||motion_allowed())return; state_=(captured_stand&&zero_motion)?PlanBeforeMotionState::kReadyToArm:PlanBeforeMotionState::kWarmHold; }
    bool OnCommandBoundary(const PlanBeforeMotionObservation&o) {
        if(!enabled_) return true; if(safe_stop()) return false; if(!o.nonzero_request)return motion_allowed();
        if(have_boundary_&&o.boundary_id==last_boundary_id_)
        {
            if(motion_allowed() && !SamePlanBeforeMotionIdentity(
                    o.published_identity, armed_identity_))
                return Fail("replacement-identity-mismatch");
            return motion_allowed();
        }
        if(motion_allowed())
        {
            if(!SamePlanBeforeMotionIdentity(o.published_identity, armed_identity_))
                return Fail("replacement-identity-mismatch");
            return true;
        }
        have_boundary_=true; last_boundary_id_=o.boundary_id;
        if(state_!=PlanBeforeMotionState::kReadyToArm)return Fail("warm-hold-not-ready");
        if(!o.map_valid||!o.coverage_valid)return Fail("map-or-coverage-not-ready");
        if(!o.filtered_measured_support_valid||Count(o.filtered_measured_support)<3)return Fail("filtered-measured-support-not-ready");
        if(!o.family_a_complete_timed_plan)return Fail("family-a-complete-timed-plan-not-ready");
        if(!o.publishable_identity_epoch||!o.published_identity.valid())return Fail("publishable-identity-epoch-not-ready");
        if(!o.whole_snapshot_published)return Fail("whole-snapshot-not-published");
        if(!o.adapter_exactly_adopted||!SamePlanBeforeMotionIdentity(o.published_identity,o.adopted_identity))return Fail("adapter-identity-mismatch");
        if(!o.gait_same_identity||!SamePlanBeforeMotionIdentity(o.published_identity,o.gait_identity))return Fail("gait-identity-mismatch");
        if(!o.srbd_same_identity||!SamePlanBeforeMotionIdentity(o.published_identity,o.srbd_identity))return Fail("srbd-identity-mismatch");
        if(!o.deadline_ok)return Fail("deadline-safe-stop");
        armed_identity_=o.published_identity; state_=PlanBeforeMotionState::kArmed; reason_="armed"; return true;
    }
    bool ValidateOnlyAfterArm(bool request_safe_stop=true) noexcept { if(!enabled_)return true; if(!motion_allowed())return false; if(request_safe_stop){state_=PlanBeforeMotionState::kSafeStop;reason_="validate-only-safe-stop";} return true; }
private:
    static std::size_t Count(const std::array<bool,go2::kLegCount>&s)noexcept{std::size_t n=0;for(bool v:s)n+=v?1U:0U;return n;}
    bool Fail(const char*r)noexcept{state_=PlanBeforeMotionState::kSafeStop;reason_=r;return false;}
    bool enabled_=false; PlanBeforeMotionState state_=PlanBeforeMotionState::kDisabled; std::string reason_; std::uint64_t last_boundary_id_=0; bool have_boundary_=false; PlanBeforeMotionIdentity armed_identity_{};
};
inline bool PlanBeforeMotionFamilyAComplete(const TerrainMotionPlan&plan) {
    if(plan.v3_c_shadow||!plan.has_stage_c_timing||!plan.valid()||!plan.contact_timing.valid(plan.timing_bounds,&plan.contact_schedule.measured_contact))return false;
    for(std::size_t k=0;k<plan.horizon_knots;++k){std::size_t n=0;for(bool c:plan.contact_schedule.planned_contact[k])n+=c?1U:0U;if(n<3)return false;} return true;
}
inline PlanBeforeMotionIdentity MakePlanBeforeMotionIdentity(const TerrainPlanIdentity&i)noexcept{return {i.plan_id,i.plan_epoch,i.map_epoch};}
}
