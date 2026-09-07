#pragma once
#include "foot_trajectory.h"
#include "terrain_candidates.h"
namespace go2_terrain { namespace stage_c {
// Bind explicit discrete choices from observed terrain candidates. No target
// search, prediction-validity extension, commitment, or dynamics certificate.
// Output is atomic: a failure leaves the caller's request unchanged.
inline JointPlannerFailure BindTerminalSwingCandidates(
    FootTrajectoryRequest &request, const TouchdownEventTable &events,
    const TerrainCandidateGenerationResult &generated,
    const std::vector<std::size_t> &choices) {
    if (!generated.valid)
        return generated.failure==JointPlannerFailure::kNone
            ? JointPlannerFailure::kInvalidInput : generated.failure;
    if (!request.problem ||
        events.events.size()!=generated.sets.size() ||
        choices.size()!=events.events.size())
        return JointPlannerFailure::kInvalidInput;
    auto bound=request;
    std::array<bool,4> seen{};
    const auto &identity=request.problem->request.input.identity;
    for(std::size_t e=0;e<events.events.size();++e) {
        const auto &event=events.events[e];
        const auto leg=static_cast<std::size_t>(event.id.leg);
        const auto &set=generated.sets[e];
        if(leg>=4 || seen[leg] || bound.continuation[leg].valid ||
           event.id.schedule_epoch!=identity.schedule_epoch ||
           !(set.event_set.event_id==event.id) || !set.valid() ||
           choices[e]>=set.event_set.candidates.size() ||
           !event.liftoff_valid || event.liftoff_time>=request.end ||
           event.touchdown_time<request.end)
            return JointPlannerFailure::kInvalidInput;
        seen[leg]=true;
        const auto &candidate=set.event_set.candidates[choices[e]];
        const auto &match=set.matched_surfaces[choices[e]];
        const auto &p=candidate.target_world;
        const auto &q=match.surface_world;
        if(p.source_time!=q.source_time || p.frame!=q.frame || p.role!=q.role ||
           p.value.x!=q.value.x || p.value.y!=q.value.y || p.value.z!=q.value.z ||
           match.contact_surface.map_epoch!=identity.map_epoch ||
           match.collision_radius_m!=request.collision_radius_m[leg])
            return JointPlannerFailure::kInvalidInput;
        if(event.target_world.valid &&
           (event.target_world.value.x!=p.value.x ||
            event.target_world.value.y!=p.value.y ||
            event.target_world.value.z!=p.value.z))
            return JointPlannerFailure::kInitialConditionConflict;
        if(p.source_time>identity.source_state_time ||
           match.contact_surface.valid_until<event.contact_interval_end)
            return JointPlannerFailure::kCoverageIncomplete;
        auto &tail=bound.continuation[leg];
        tail.valid=true;tail.event=event;tail.candidate=candidate;
        tail.surface=match.contact_surface;
        // Retain original observation timestamps and the complete contact tail.
    }
    const auto check=SampleFootTrajectoryAt(bound,bound.start);
    if(!check.valid) return check.failure;
    request=std::move(bound);
    return JointPlannerFailure::kNone;
}
}} // namespace
