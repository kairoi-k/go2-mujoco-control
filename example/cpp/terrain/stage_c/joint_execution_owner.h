#pragma once
// Atomic selected-reference owner; motor authority remains with the caller.
// It deliberately owns a selected CentroidalJointProposal and one existing
// FootTrajectoryRequest. It does not own contact policy, body reconstruction,
// force schema, LowCmdWrite, or actuator output.
#include "centroidal_joint_proposal.h"
#include "foot_trajectory.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
namespace go2_terrain
{
namespace stage_c
{
namespace joint_execution
{
enum class OwnerStatus : std::uint8_t
{
    kAdopted = 0,
    kRetained,
    kNoPending,
    kStale,
    kInvalid,
    kCommitmentConflict,
    kSampleUnavailable,
    kMissingCommandedSeed,
    // Command p/v cannot be sampled continuously with this proposal.
    kCommandHandoverConflict,
};
struct OwnerConfig
{
    // Explicit source freshness policy; caller may tighten it for runtime.
    std::int64_t max_source_age_ns = 250000000;
    // Zero means the caller has not supplied a current epoch constraint.
    std::uint64_t expected_schedule_epoch = 0;
    // If set, this is the current observation epoch only; never pin it to
    // the prior accepted proposal because terrain_map_epoch_ advances per
    // planning observation.
    std::uint64_t expected_map_epoch = 0;
};
// Handover-only command provenance. This is not measured contact and is never
// written into TerrainPlanningInput::feet or ContactEvidence.
struct CommandedFootSeed
{
    // Monotonic command lineage from the existing Phase-1 shaper/owner.
    std::uint64_t command_epoch = 0;
    TimeNs source_time{};
    std::array<TimedPoint, go2::kLegCount> center_world{};
    std::array<go2::Vec3, go2::kLegCount> velocity_world{};
    std::array<bool, go2::kLegCount> valid{};
    bool valid_for_handover() const
    {
        if (command_epoch == 0 || source_time.value < 0)
            return false;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!valid[leg] || !TimedPointValidAt(
                    center_world[leg], PointRole::kFootCollisionCenter,
                    Frame::kWorld, source_time) ||
                !FinitePointValue(velocity_world[leg]))
                return false;
        }
        return true;
    }
};
// One immutable transport object. selected owns the exact selected problem
// and result retained by CentroidalJointProposal. foot_request.problem MUST
// point to selected->selected_problem; no copied problem/result is allowed.
struct JointExecutionProposal
{
    std::uint64_t proposal_id = 0;
    PlanningIdentity identity{};
    TimeNs valid_until{};
    std::shared_ptr<const CentroidalJointProposal> selected{};
    FootTrajectoryRequest foot_request{};
    // Bind an explicit command boundary at owner-thread adoption time.
    bool first_handover_from_commanded = false;
    CommandedFootSeed commanded_seed{};
};
struct ActiveCurveLease
{
    bool valid = false;
    TouchdownEvent event{};
    // The old body horizon may expire while this swing is still airborne.
    // Cache the already validated polynomial inputs so curve sampling does
    // not revalidate the expired body/force bundle or silently restart it.
    std::shared_ptr<const JointExecutionProposal> source{};
    foot_trajectory_detail::PreparedEvent prepared_event{};
    TimeNs reference_source_time{};
    double clearance_m = 0.0;
    bool add_clearance_to_inflight_continuation = true;
    bool curve_prepared = false;
};
inline bool PrepareActiveCurveLease(
    const std::shared_ptr<const JointExecutionProposal> &source,
    const TouchdownEvent &resolved, ActiveCurveLease &lease)
{
    if (!source || !source->selected || !source->selected->selected_valid ||
        !source->selected->search.feasible ||
        !source->selected->selected_result.certificate.feasible ||
        source->foot_request.problem !=
            &source->selected->selected_problem ||
        source->foot_request.end <= resolved.liftoff_time)
        return false;
    const auto &problem = source->selected->selected_problem;
    std::size_t event_index = problem.request.events.events.size();
    for (std::size_t i = 0; i < problem.request.events.events.size(); ++i)
        if (problem.request.events.events[i].id == resolved.id)
        {
            event_index = i;
            break;
        }
    if (event_index == problem.request.events.events.size())
        return false;
    foot_trajectory_detail::PreparedTrajectory prepared;
    if (foot_trajectory_detail::Prepare(source->foot_request, prepared) !=
        JointPlannerFailure::kNone)
        return false;
    const auto it = std::find_if(
        prepared.events.begin(), prepared.events.end(),
        [event_index](const foot_trajectory_detail::PreparedEvent &candidate) {
            return !candidate.continuation &&
                candidate.event_index == event_index;
        });
    if (it == prepared.events.end() ||
        static_cast<std::size_t>(resolved.id.leg) >= go2::kLegCount ||
        it->leg != static_cast<std::size_t>(resolved.id.leg) ||
        it->liftoff != resolved.liftoff_time ||
        it->touchdown != resolved.touchdown_time ||
        it->contact_end != resolved.contact_interval_end)
        return false;
    ActiveCurveLease cached{};
    cached.valid = true;
    cached.event = resolved;
    cached.source = source;
    cached.prepared_event = *it;
    cached.reference_source_time = prepared.initial_reference_source_time;
    cached.clearance_m = prepared.clearance_m;
    cached.add_clearance_to_inflight_continuation =
        prepared.add_clearance_to_inflight_continuation;
    cached.curve_prepared = true;
    lease = std::move(cached);
    return true;
}
inline bool SampleActiveCurveLease(
    const ActiveCurveLease &lease, TimeNs now, FootTrajectorySample &sample)
{
    if (!lease.valid || !lease.curve_prepared ||
        static_cast<std::size_t>(lease.event.id.leg) >= go2::kLegCount ||
        now < lease.event.liftoff_time ||
        now < lease.prepared_event.interpolation_start ||
        now >= lease.event.contact_interval_end)
        return false;
    const std::size_t leg = static_cast<std::size_t>(lease.event.id.leg);
    Eigen::Vector3d position = lease.prepared_event.p1;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    if (now < lease.event.touchdown_time)
    {
        const double clearance = lease.prepared_event.starts_in_flight &&
                !lease.add_clearance_to_inflight_continuation
            ? 0.0 : lease.clearance_m;
        foot_trajectory_detail::EvaluateSwing(
            lease.prepared_event, now, clearance, position, velocity,
            acceleration);
    }
    if (!position.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite())
        return false;
    sample = FootTrajectorySample{};
    sample.time = now;
    sample.center_world[leg] = {
        foot_trajectory_detail::ToVec3(position), Frame::kWorld,
        lease.reference_source_time, true, PointRole::kFootCollisionCenter};
    sample.velocity_world[leg] =
        foot_trajectory_detail::ToVec3(velocity);
    sample.acceleration_world[leg] =
        foot_trajectory_detail::ToVec3(acceleration);
    sample.leg_valid[leg] = true;
    sample.valid = true;
    return true;
}
struct AcceptedJointReference
{
    std::uint64_t execution_version = 0;
    std::shared_ptr<const JointExecutionProposal> proposal{};
    std::array<ActiveCurveLease, go2::kLegCount> in_flight{};
    bool valid = false;
};
struct SampledJointReference
{
    std::uint64_t execution_version = 0;
    // This remains the Stage-C collision-center role. WBC uses this center
    // reference against dyn.foot_pos_world. Gait IK must separately perform
    // a model-consistent center-to-foot-site lift.
    FootTrajectorySample center_reference{};
    std::array<bool, go2::kLegCount> curve_valid{};
    // bundle_valid covers current centroidal/body/force validity. A retained
    // in-flight curve may be available after it expires, but that does not
    // make the whole accepted bundle executable.
    bool bundle_valid = false;
    bool valid = false;
};
struct OwnerResult
{
    OwnerStatus status = OwnerStatus::kNoPending;
    std::shared_ptr<const AcceptedJointReference> accepted{};
};
inline bool SameTimedPointExact(const TimedPoint &a, const TimedPoint &b)
{
    return a.valid == b.valid && a.role == b.role && a.frame == b.frame &&
        a.source_time == b.source_time &&
        (!a.valid || (a.value.x == b.value.x && a.value.y == b.value.y &&
                      a.value.z == b.value.z));
}
inline bool SameCommittedEventCore(
    const TouchdownEvent &a, const TouchdownEvent &b)
{
    return a.id == b.id && a.liftoff_valid == b.liftoff_valid &&
        (!a.liftoff_valid || a.liftoff_time == b.liftoff_time) &&
        a.touchdown_time == b.touchdown_time &&
        a.contact_interval_end == b.contact_interval_end &&
        SameTimedPointExact(a.target_world, b.target_world);
}
inline const TouchdownEvent *FindEvent(
    const CentroidalProblem &problem, const TouchdownEventId &id)
{
    for (const auto &event : problem.request.events.events)
        if (event.id == id)
            return &event;
    return nullptr;
}
inline bool ResolveSelectedEvent(const CentroidalProblem &problem,
    const TouchdownEvent &event, TouchdownEvent &resolved)
{
    for (std::size_t i=0; i<problem.request.events.events.size(); ++i)
    {
        if (!(problem.request.events.events[i].id == event.id)) continue;
        if (i>=problem.combination.size() || i>=problem.request.candidate_sets.size()) return false;
        const auto &set=problem.request.candidate_sets[i];
        if (problem.combination[i]>=set.candidates.size()) return false;
        const auto &point=set.candidates[problem.combination[i]].target_world;
        if (!TimedPointValidForRole(point,PointRole::kSurfaceContactPoint,Frame::kWorld) ||
            (event.target_world.valid && !SameTimedPointExact(event.target_world,point))) return false;
        resolved=event;
        resolved.target_world=point;
        resolved.source_plan_id=problem.request.input.identity.source_plan_id;
        return true;
    }
    return false;
}
inline const TouchdownEvent *FindActiveEvent(
    const CentroidalProblem &problem, std::size_t leg, TimeNs now)
{
    const TouchdownEvent *found = nullptr;
    for (const auto &event : problem.request.events.events)
    {
        if (static_cast<std::size_t>(event.id.leg) != leg ||
            !event.liftoff_valid || event.liftoff_time > now ||
            now >= event.contact_interval_end)
            continue;
        if (found != nullptr)
            return nullptr; // two active events for one leg is malformed
        found = &event;
    }
    return found;
}
inline bool SameIdentity(const PlanningIdentity &a, const PlanningIdentity &b)
{
    return a.source_state_tick == b.source_state_tick &&
        a.source_state_time == b.source_state_time &&
        a.map_epoch == b.map_epoch &&
        a.schedule_epoch == b.schedule_epoch &&
        a.source_plan_id == b.source_plan_id;
}
class AtomicJointExecutionOwner final
{
public:
    explicit AtomicJointExecutionOwner(OwnerConfig config = {})
        : config_(config)
    {
    }
    // Worker side. The shared_ptr free-function atomic overload is C++17-safe;
    // std::atomic<shared_ptr<T>> is intentionally not required.
    void Publish(std::shared_ptr<const JointExecutionProposal> proposal)
    {
        std::atomic_store_explicit(
            &pending_, std::move(proposal), std::memory_order_release);
    }
    std::shared_ptr<const JointExecutionProposal> Pending() const
    {
        return std::atomic_load_explicit(&pending_, std::memory_order_acquire);
    }
    // Owner side only. No LowCmdWrite dependency is present in this draft.
    // Call once per control tick before any consumer samples the reference.
    OwnerResult Adopt(TimeNs now, std::uint64_t current_state_tick,
                      const CommandedFootSeed *handover_seed = nullptr)
    {
        OwnerResult out;
        RefreshActiveLeases(now);
        out.accepted = accepted_;
        auto pending = Pending(); // exactly one atomic load per call
        if (!pending)
        {
            out.status = OwnerStatus::kNoPending;
            return out;
        }
        if (accepted_ && accepted_->proposal->proposal_id == pending->proposal_id &&
            SameIdentity(accepted_->proposal->identity, pending->identity))
        {
            if (now < accepted_->proposal->valid_until)
                out.status = OwnerStatus::kRetained;
            else
                out.status = OwnerStatus::kStale;
            return out;
        }
        if (pending->first_handover_from_commanded)
        {
            if (handover_seed == nullptr || !handover_seed->valid_for_handover() ||
                handover_seed->source_time != now)
            {
                out.status = OwnerStatus::kMissingCommandedSeed;
                return out;
            }
            // Bind only reference boundary. The selected physical input and
            // optimization result remain immutable and retain their timestamps.
            auto bound = std::make_shared<JointExecutionProposal>(*pending);
            bound->commanded_seed = *handover_seed;
            bound->foot_request.start = now;
            auto &boundary = bound->foot_request.commanded_initial;
            boundary.enabled = true;
            boundary.command_epoch = handover_seed->command_epoch;
            boundary.source_time = now;
            boundary.valid_until = now;
            boundary.center_world = handover_seed->center_world;
            boundary.velocity_world = handover_seed->velocity_world;
            boundary.valid = handover_seed->valid;
            bound->foot_request.add_clearance_to_inflight_continuation = false;
            // A commanded boundary may carry nonzero stance velocity. Opt into
            // the bounded C1 settling prefix explicitly; this changes only the
            // generated reference and never rewrites measured input.
            bound->foot_request.enable_commanded_stance_settling = true;
            bound->foot_request.commanded_stance_settling_duration_s = 0.020;
            const auto sample = SampleFootTrajectoryAt(bound->foot_request, now);
            if (!sample.valid || sample.samples.size() != 1)
            {
                out.status = OwnerStatus::kCommandHandoverConflict;
                return out;
            }
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto &actual = sample.samples.front();
                const auto &p = actual.center_world[leg].value;
                const auto &q = handover_seed->center_world[leg].value;
                const auto &v = actual.velocity_world[leg];
                const auto &w = handover_seed->velocity_world[leg];
                if (!actual.leg_valid[leg] ||
                    std::abs(p.x-q.x)>1e-9 || std::abs(p.y-q.y)>1e-9 ||
                    std::abs(p.z-q.z)>1e-9 || std::abs(v.x-w.x)>1e-9 ||
                    std::abs(v.y-w.y)>1e-9 || std::abs(v.z-w.z)>1e-9)
                {
                    out.status = OwnerStatus::kCommandHandoverConflict;
                    return out;
                }
            }
            pending = std::move(bound);
        }
        const OwnerStatus valid = ValidateProposal(
            *pending, now, current_state_tick);
        if (valid != OwnerStatus::kAdopted)
        {
            out.status = valid;
            return out;
        }
        if (accepted_ && !CommittedLeasesCompatible(*accepted_, *pending, now))
        {
            out.status = OwnerStatus::kCommitmentConflict;
            return out;
        }
        auto next = std::make_shared<AcceptedJointReference>();
        next->execution_version = next_version_ + 1;
        next->proposal = pending;
        next->valid = true;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (accepted_ && accepted_->in_flight[leg].valid &&
                now < accepted_->in_flight[leg].event.contact_interval_end)
            {
                next->in_flight[leg] = accepted_->in_flight[leg];
                continue;
            }
            const auto *event = FindActiveEvent(
                pending->selected->selected_problem, leg, now);
            if (event != nullptr)
            {
                TouchdownEvent resolved;
                if (!ResolveSelectedEvent(
                        pending->selected->selected_problem, *event, resolved) ||
                    !PrepareActiveCurveLease(pending, resolved,
                                             next->in_flight[leg]))
                {
                    out.status = OwnerStatus::kSampleUnavailable;
                    return out;
                }
            }
        }
        next_version_ = next->execution_version;
        accepted_ = std::move(next);
        out.accepted = accepted_;
        out.status = OwnerStatus::kAdopted;
        return out;
    }
    // Control-thread only. Do not call this from the worker; copy the result
    // under the existing terrain snapshot/work mutex before worker transport.
    std::shared_ptr<const AcceptedJointReference> Accepted() const
    {
        return accepted_;
    }
    // Control-thread only producer input for the next solve. This exports only committed event
    // metadata from active leases; it does not export measured/applied contact
    // and it does not claim a curve token that the current contracts lack.
    TouchdownEventTable CommittedEvents(TimeNs now) const
    {
        TouchdownEventTable result;
        if (!accepted_)
            return result;
        for (const auto &lease : accepted_->in_flight)
        {
            if (!lease.valid || now >= lease.event.touchdown_time)
                continue;
            auto event = lease.event;
            event.committed = true;
            result.events.push_back(event);
        }
        std::sort(result.events.begin(), result.events.end(),
                  [](const TouchdownEvent &left, const TouchdownEvent &right) {
                      return left.touchdown_time < right.touchdown_time;
                  });
        return result;
    }

    // Consumer-side sampling. It returns collision-center p/v/a. The same
    // accepted version is visible to gait and WBC; active leases override only
    // their leg and retain the old request/curve through contact_interval_end.
    SampledJointReference SampleAt(TimeNs now) const
    {
        SampledJointReference out;
        if (!accepted_ || !accepted_->valid || !accepted_->proposal ||
            now.value < accepted_->proposal->foot_request.start.value)
            return out;
        // Lease lifetime does not extend the current body/centroidal/force
        // validity. Expose no full reference after expiry; a separate
        // curve-only query below may retain a committed in-flight swing.
        if (now >= accepted_->proposal->valid_until)
            return SampleCurveOnlyAt(now);
        if (now >= accepted_->proposal->foot_request.end)
            return out;
        auto current = SampleFootTrajectoryAt(
            accepted_->proposal->foot_request, now);
        if (!current.valid || current.samples.size() != 1)
            return out;
        out.execution_version = accepted_->execution_version;
        out.center_reference = current.samples.front();
        out.bundle_valid = true;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            out.curve_valid[leg] = out.center_reference.leg_valid[leg];
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &lease = accepted_->in_flight[leg];
            if (!lease.valid || now >= lease.event.contact_interval_end)
                continue;
            FootTrajectorySample old_sample;
            if (!SampleActiveCurveLease(lease, now, old_sample) ||
                !old_sample.leg_valid[leg])
            {
                out = SampledJointReference{};
                return out;
            }
            out.center_reference.center_world[leg] = old_sample.center_world[leg];
            out.center_reference.velocity_world[leg] =
                old_sample.velocity_world[leg];
            out.center_reference.acceleration_world[leg] =
                old_sample.acceleration_world[leg];
            out.center_reference.leg_valid[leg] = old_sample.leg_valid[leg];
            out.curve_valid[leg] = old_sample.leg_valid[leg];
        }
        out.valid = out.bundle_valid && out.center_reference.valid;
        return out;
    }

    // Control-thread only. This is deliberately curve-only: it can retain an
    // old in-flight center curve after bundle expiry, but exposes no current
    // centroidal/body/force validity and therefore cannot drive a full tick.
    SampledJointReference SampleCurveOnlyAt(TimeNs now) const
    {
        SampledJointReference out;
        if (!accepted_ || !accepted_->valid || !accepted_->proposal ||
            now < accepted_->proposal->foot_request.start)
            return out;
        out.execution_version = accepted_->execution_version;
        out.center_reference.time = now;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const auto &lease = accepted_->in_flight[leg];
            FootTrajectorySample old_sample;
            if (!SampleActiveCurveLease(lease, now, old_sample) ||
                !old_sample.leg_valid[leg])
                continue;
            out.center_reference.center_world[leg] = old_sample.center_world[leg];
            out.center_reference.velocity_world[leg] =
                old_sample.velocity_world[leg];
            out.center_reference.acceleration_world[leg] =
                old_sample.acceleration_world[leg];
            out.center_reference.leg_valid[leg] = old_sample.leg_valid[leg];
            out.curve_valid[leg] = old_sample.leg_valid[leg];
        }
        for (const bool valid : out.curve_valid)
            out.center_reference.valid =
                out.center_reference.valid || valid;
        return out;
    }
private:
    void RefreshActiveLeases(TimeNs now)
    {
        if (!accepted_ || !accepted_->valid || !accepted_->proposal)
            return;
        // Preserve immutability of previously returned accepted snapshots.
        accepted_ = std::make_shared<AcceptedJointReference>(*accepted_);
        const auto &problem = accepted_->proposal->selected->selected_problem;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            auto &lease = accepted_->in_flight[leg];
            if (lease.valid && now >= lease.event.contact_interval_end)
                lease = ActiveCurveLease{};
            if (lease.valid)
            {
                if (!lease.curve_prepared &&
                    !PrepareActiveCurveLease(
                        lease.source, lease.event, lease))
                    lease = ActiveCurveLease{};
                if (lease.valid)
                    continue;
            }
            const auto *event = FindActiveEvent(problem, leg, now);
            if (event != nullptr)
            {
                TouchdownEvent resolved;
                if (ResolveSelectedEvent(problem, *event, resolved) &&
                    PrepareActiveCurveLease(
                        accepted_->proposal, resolved, lease))
                    continue;
                lease = ActiveCurveLease{};
            }
        }
    }

    OwnerStatus ValidateProposal(
        const JointExecutionProposal &proposal, TimeNs now,
        std::uint64_t current_state_tick) const
    {
        if (!proposal.selected || !proposal.selected->selected_valid ||
            !proposal.selected->search.feasible ||
            !proposal.selected->selected_result.certificate.feasible ||
            !proposal.identity.valid() || proposal.proposal_id == 0 ||
            proposal.valid_until.value <= now.value ||
            proposal.foot_request.end <= proposal.foot_request.start ||
            proposal.foot_request.start < proposal.identity.source_state_time ||
            proposal.valid_until > proposal.foot_request.end ||
            proposal.identity.source_state_tick > current_state_tick ||
            proposal.identity.source_state_time > now ||
            proposal.identity.map_epoch == 0 ||
            proposal.identity.schedule_epoch == 0)
            return proposal.identity.source_state_time > now ||
                    proposal.identity.source_state_tick > current_state_tick ||
                    proposal.valid_until.value <= now.value
                ? OwnerStatus::kStale : OwnerStatus::kInvalid;
        const auto &problem = proposal.selected->selected_problem;
        const auto &input = problem.request.input;
        if (!SameIdentity(proposal.identity, input.identity) ||
            problem.schedule_epoch != proposal.identity.schedule_epoch ||
            proposal.foot_request.problem != &problem ||
            (config_.expected_schedule_epoch != 0 &&
             config_.expected_schedule_epoch != proposal.identity.schedule_epoch) ||
            (config_.expected_map_epoch != 0 &&
             config_.expected_map_epoch != proposal.identity.map_epoch) ||
            config_.max_source_age_ns < 0 ||
            now.value - proposal.identity.source_state_time.value >
                config_.max_source_age_ns)
            return OwnerStatus::kInvalid;
        if (!problem.request.events.events.empty() &&
            !problem.request.events.valid(false))
            return OwnerStatus::kInvalid;
        if (!problem.request.accepted_commitments.events.empty() &&
            !problem.request.accepted_commitments.valid(false))
            return OwnerStatus::kCommitmentConflict;
        const auto sample = SampleFootTrajectoryAt(
            proposal.foot_request, now);
        if (!sample.valid || sample.samples.size() != 1)
            return OwnerStatus::kSampleUnavailable;
        return OwnerStatus::kAdopted;
    }
    bool HasActiveLease(TimeNs now) const
    {
        if (!accepted_)
            return false;
        for (const auto &lease : accepted_->in_flight)
            if (lease.valid && now < lease.event.contact_interval_end)
                return true;
        return false;
    }
    static bool CommittedLeasesCompatible(
        const AcceptedJointReference &old,
        const JointExecutionProposal &next, TimeNs now)
    {
        for (const auto &lease : old.in_flight)
        {
            if (!lease.valid || now >= lease.event.contact_interval_end)
                continue;
            const auto *candidate = FindEvent(
                next.selected->selected_problem, lease.event.id);
            // Past touchdowns are initial stance in the next preview, not
            // future events. Keep their old curve without inventing an event.
            if ((now < lease.event.touchdown_time &&
                 (candidate == nullptr || !candidate->committed ||
                  !SameCommittedEventCore(lease.event, *candidate))) ||
                !lease.curve_prepared)
                return false;
            FootTrajectorySample old_sample;
            if (!SampleActiveCurveLease(lease, now, old_sample))
                return false;
        }
        return true;
    }
    OwnerConfig config_{};
    std::shared_ptr<const JointExecutionProposal> pending_{};
    // Mutated only by the control-thread owner during Adopt/refresh.
    // Public access is returned as shared_ptr<const ...> below.
    std::shared_ptr<AcceptedJointReference> accepted_{};
    std::uint64_t next_version_ = 0;
};
} // namespace joint_execution
} // namespace stage_c
} // namespace go2_terrain
