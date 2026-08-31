#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "terrain_crawl_script.h"
#include "terrain_crawl_state_machine.h"

namespace go2_terrain
{

// Window-local event-driven crawl owner. It has no planner identity, expiry,
// candidate stream, or transaction mask: every update reads current map and
// measured state, and every event advances the sequence.
enum class TerrainCrawlSequencerState : unsigned char
{
    kInactive = 0, kStage, kShift, kSwing, kCommit, kAdvance,
    kClear, kResume, kAbort
};

inline const char *TerrainCrawlSequencerStateName(
    TerrainCrawlSequencerState state) noexcept
{
    switch (state)
    {
    case TerrainCrawlSequencerState::kStage: return "STAGE";
    case TerrainCrawlSequencerState::kShift: return "SHIFT";
    case TerrainCrawlSequencerState::kSwing: return "SWING";
    case TerrainCrawlSequencerState::kCommit: return "COMMIT";
    case TerrainCrawlSequencerState::kAdvance: return "ADVANCE";
    case TerrainCrawlSequencerState::kClear: return "CLEAR";
    case TerrainCrawlSequencerState::kResume: return "RESUME";
    case TerrainCrawlSequencerState::kAbort: return "ABORT";
    default: return "INACTIVE";
    }
}

struct TerrainCrawlSequencerInput
{
    bool transfer_window_active = false;
    // Harness-only isolation mode. It deliberately has no terrain/map
    // dependency and is disabled for every normal caller.
    bool flat_ground_mode = false;
    double flat_step_length_m = 0.08;
    // The running-trot phase is retained as an attribution witness. After
    // staging stops the gait, measured stance is the equivalent boundary.
    bool trot_full_contact_able = false;
    // The legacy state machine enters STAGE after authority. Keep the two
    // owners from launching the first swing on different ticks.
    bool legacy_stage_ready = true;
    // Terrain SWING must wait for the measured legacy COM-shift completion
    // witness. The caller sets this only after force balance and COM margin
    // have passed; flat isolation does not use the terrain gate.
    bool legacy_shift_ready = false;
    const TerrainModel *terrain = nullptr;
    go2::Vec3 base_position_world{};
    double base_yaw_rad = 0.0;
    double nominal_front_foot_x_m = 0.0;
    std::array<go2::Vec3, go2::kLegCount> measured_feet_world{};
    bool measured_feet_valid = false;
    std::array<bool, go2::kLegCount> measured_contact{};
    bool measured_contact_valid = false;
    bool measured_force_valid = false;
    std::array<double, go2::kLegCount> measured_normal_force_n{};
    go2::Vec3 measured_com_world{};
    bool measured_com_valid = false;
    double measured_velocity_mps = 0.0;
    bool measured_posture_valid = false;
    double measured_roll_rad = 0.0;
    double measured_pitch_rad = 0.0;
    bool rear_targets_fk_reachable = false;
    // Harness-only staged isolation target, already sampled by the terrain
    // execution adapter before its asynchronous plan expiry.
    bool staged_target_valid = false;
    go2::Vec3 staged_target_world{};
    bool base_clear = false;
    bool all_feet_clear = false;
    bool stable = false;
    // Measured support margin from the legacy SHIFT_COM witness.
    double com_margin_m = -std::numeric_limits<double>::infinity();
    double now_s = 0.0;
};

// Project the four-contact preload onto the three stance legs before a
// terrain swing.  Keeping the total normal load and each stance leg's
// share avoids a contact-mask switch from asking one diagonal leg to
// absorb the entire unloaded leg in a single WBC solve.
inline std::array<double, go2::kLegCount>
TerrainStanceForceHandoffReference(
    const std::array<double, go2::kLegCount> &preload_n,
    std::size_t swing_leg) noexcept
{
    std::array<double, go2::kLegCount> reference{};
    if (swing_leg >= go2::kLegCount)
        return reference;
    double total = 0.0;
    double stance_total = 0.0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const double force = preload_n[leg];
        if (!std::isfinite(force) || force <= 0.0)
            continue;
        total += force;
        if (leg != swing_leg)
            stance_total += force;
    }
    if (!(total > 0.0) || !(stance_total > 0.0))
        return reference;
    const double scale = total / stance_total;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (leg == swing_leg)
            continue;
        const double force = preload_n[leg];
        if (std::isfinite(force) && force > 0.0)
            reference[leg] = force * scale;
    }
    return reference;
}

struct TerrainCrawlSequencerOutput
{
    TerrainCrawlSequencerState state = TerrainCrawlSequencerState::kInactive;
    std::size_t active_leg = go2::kLegCount;
    bool target_valid = false;
    go2::Vec3 swing_start_world{};
    go2::Vec3 target_world{};
    go2::Vec3 swing_position_world{};
    go2::Vec3 swing_velocity_world{};
    double swing_phase = 0.0;
    double swing_lift_m = 0.03;
    go2::Vec3 com_reference_world{};
    bool com_reference_valid = false;
    // False while the window is merely armed; consumers must keep trot
    // contacts, swing targets, and COM authority unchanged in that phase.
    bool control_authority_active = false;
    // Per-tick authority witness. These fields make the handoff boundary
    // attributable without inferring predicates from downstream behavior.
    bool authority_trot_full_contact_able = false;
    bool authority_measured_contacts_ready = false;
    bool authority_velocity_ready = false;
    bool authority_posture_ready = false;
    bool authority_stand_transition_seen = false;
    int authority_block_reason = 0; // 1=trot phase, 2=contacts, 3=speed, 4=posture
    double authority_velocity_mps = 0.0;
    double authority_roll_rad = 0.0;
    double authority_pitch_rad = 0.0;
    int stage_abort_reason = 0; // 1=STAGE timeout
    // Identifies the non-contract flat-ground isolation harness.
    bool flat_ground_mode = false;
    // A phase-respecting stop may be requested while still armed; this is
    // the trot pipeline slowing down, not sequencer contact ownership.
    bool stand_transition_requested = false;
    std::array<bool, go2::kLegCount> contact_schedule{};
    int measured_contact_count = 0;
    double com_margin_m = -std::numeric_limits<double>::infinity();
    // Measured endpoint commits, retained across the following leg event.
    std::array<bool, go2::kLegCount> committed{};
};

class TerrainCrawlSequencer
{
public:
    // The historical compile-time alias remains available to old callers.
    static constexpr std::array<std::size_t, 4> kLegOrder =
        kLegacyFrontFirstLegOrder;
    static constexpr std::array<std::size_t, 4> kLateralOrder =
        kLateralLegOrder;
    static constexpr double kStandoffM = 0.12;
    // V2-A entry braking remains under the running trot authority. The
    // deceleration is deliberately below the command shaper limit so the
    // measured three-contact stance can remain reachable while speed falls.
    static constexpr double kApproachMaxSpeedMps = 0.30;
    static constexpr double kApproachAllowedDecelMps2 = 1.20;
    static constexpr double kApproachSafetyMarginM = 0.10;
    static constexpr double kApproachBrakingDistanceM =
        kApproachMaxSpeedMps * kApproachMaxSpeedMps /
        (2.0 * kApproachAllowedDecelMps2);
    // TransferActivationReady receives distance to the staging target. Keep
    // enough distance for a full-speed stop, the canonical standoff, and a
    // map/foot measurement margin before the leading feet reach the riser.
    static constexpr double kTransferActivationDistanceM =
        kApproachBrakingDistanceM + kStandoffM + kApproachSafetyMarginM;
    static constexpr double kStageToleranceM = 0.015;
    // Leave one legacy state-machine entry dwell after authority handoff so
    // both owners observe the same four-contact STAGE before SHIFT.
    static constexpr double kStageDwellS = 0.30;
    static constexpr double kShiftDwellS = 0.12;
    static constexpr double kSwingDurationS = 0.60;
    // A force-filter contact bit can drop briefly as the unloaded leg clears
    // the stance. Keep the existing fixed swing deadline as the safety bound;
    // a persistent loss still aborts at that deadline.
    static constexpr double kSwingSupportLossDeadlineS = kSwingDurationS;
    static constexpr double kCommitToleranceM = 0.045;
    static constexpr double kResumeDwellS = 0.45;
    static constexpr double kStageTimeoutS = 4.0;
    // Order-060 measured entry basin: edge-minus-base target is the
    // midpoint of the observed 0.318--0.330 m band.
    static constexpr double kBasinMarginM = 0.020;
    static constexpr double kBasinHalfWidthM =
        0.5 * (kMeasuredBasinEdgeMinusBaseMaxM -
               kMeasuredBasinEdgeMinusBaseMinM);
    static constexpr double kStableForceMinN = 10.0;
    static constexpr double kStableForceTotalMinN = 50.0;
    static constexpr double kStableForceImbalanceRatio = 4.0;

    // Adaptive speed envelope for the approach leg. The outer envelope
    // starts bleeding speed as soon as the window arms; the stopping envelope
    // guarantees that the remaining distance can absorb the current speed.
    static double ApproachSpeedCapMps(double staging_error_m) noexcept
    {
        if (!std::isfinite(staging_error_m) || staging_error_m <= 0.0)
            return 0.0;
        const double profile_cap = kApproachMaxSpeedMps * std::sqrt(
            std::min(1.0, staging_error_m /
                kTransferActivationDistanceM));
        const double stopping_cap = std::sqrt(
            2.0 * kApproachAllowedDecelMps2 * staging_error_m);
        return std::min({kApproachMaxSpeedMps, profile_cap, stopping_cap});
    }

    static bool TransferActivationReady(
        const TerrainModel &terrain, const go2::Vec3 &base_position_world,
        double base_yaw_rad, double nominal_front_foot_x_m,
        double activation_distance_m = kTransferActivationDistanceM) noexcept
    {
        const auto staging = MeasureTerrainStagingReference(
            terrain, base_position_world, base_yaw_rad,
            nominal_front_foot_x_m, kStandoffM);
        return staging.valid && std::isfinite(staging.error_m) &&
            staging.error_m <= activation_distance_m;
    }

    std::size_t order_index() const noexcept { return order_index_; }

    void SetLegOrder(TerrainCrawlLegOrder order) noexcept
    {
        leg_order_ = order;
        leg_order_values_ = order == TerrainCrawlLegOrder::kLateral
            ? kLateralLegOrder : kLegacyFrontFirstLegOrder;
    }

    void SetAdvancePolicy(TerrainCrawlAdvancePolicy policy) noexcept
    {
        advance_policy_ = policy;
    }

    TerrainCrawlLegOrder leg_order() const noexcept { return leg_order_; }
    TerrainCrawlAdvancePolicy advance_policy() const noexcept
    {
        return advance_policy_;
    }

    void Reset() noexcept
    {
        state_ = TerrainCrawlSequencerState::kInactive;
        order_index_ = 0;
        state_enter_s_ = 0.0;
        stable_start_s_ = std::numeric_limits<double>::infinity();
        stage_stable_start_s_ = std::numeric_limits<double>::infinity();
        target_ = {};
        swing_start_ = {};
        target_valid_ = false;
        output_ = {};
        authority_active_ = false;
        authority_com_reference_ = {};
        stage_abort_reason_ = 0;
        authority_com_reference_valid_ = false;
        stand_transition_seen_ = false;
        committed_.fill(false);
    }

    TerrainCrawlSequencerState Update(
        const TerrainCrawlSequencerInput &input) noexcept
    {
        if (!input.transfer_window_active)
        {
            Reset();
            return state_;
        }
        if (state_ == TerrainCrawlSequencerState::kInactive)
        {
            state_ = TerrainCrawlSequencerState::kStage;
            state_enter_s_ = input.now_s;
        }
        const int contacts = input.measured_contact_valid
            ? static_cast<int>(std::count(input.measured_contact.begin(),
                                          input.measured_contact.end(), true))
            : 0;
        // Arming is deliberately side-effect free. Do not publish a crawl
        // topology or COM target until the trot phase itself can carry four
        // contacts and the measured plant still has at least three anchors.
        if (!authority_active_)
        {
            const bool stand_boundary = input.flat_ground_mode ||
                (input.measured_contact_valid && contacts >= 3);
            if (stand_boundary && !stand_transition_seen_)
            {
                // Give the running trot one complete boundary tick to accept
                // the zero-velocity stand request before ownership changes.
                stand_transition_seen_ = true;
                return Publish(input);
            }
            // After the one-tick running-trot boundary request, staging owns
            // a measured standstill. Requiring the trot phase again is
            // unreachable when braking has already stopped that gait.
            if (stand_transition_seen_ && input.measured_contact_valid &&
                contacts >= 3 && std::isfinite(input.measured_velocity_mps) &&
                input.measured_velocity_mps <= 0.04 &&
                input.measured_posture_valid &&
                std::abs(input.measured_roll_rad) <= 0.08 &&
                std::abs(input.measured_pitch_rad) <= 0.08)
            {
                authority_active_ = true;
                // STAGE dwell starts at authority handoff, not while the
                // window is merely armed under running-trot authority.
                stage_stable_start_s_ = std::numeric_limits<double>::infinity();
                if (input.measured_com_valid &&
                    std::isfinite(input.measured_com_world.x) &&
                    std::isfinite(input.measured_com_world.y) &&
                    std::isfinite(input.measured_com_world.z))
                {
                    authority_com_reference_ = input.measured_com_world;
                    authority_com_reference_valid_ = true;
                }
            }
            else
                return Publish(input);
        }
        if (state_ == TerrainCrawlSequencerState::kAbort)
            return Publish(input);
        const bool three_contacts = input.measured_contact_valid && contacts >= 3;
        const bool finite_time = std::isfinite(input.now_s);
        switch (state_)
        {
        case TerrainCrawlSequencerState::kStage:
        {
            const auto staging = input.terrain != nullptr
                ? (input.flat_ground_mode
                    ? TerrainStagingReference{}
                    : MeasureTerrainBasinStagingReference(
                          *input.terrain, input.base_position_world,
                          input.base_yaw_rad))
                : TerrainStagingReference{};
            // The base-x band is an observed correlation, not a causal
            // gate. The measured support margin below remains the sole
            // terrain release condition; staging stays edge-anchored.
            // Once authority owns a measured four-foot stand, a transient
            // map-edge invalidation must not strand STAGE. The measured basin
            // margin remains the release witness below.
            const bool at_standoff = input.flat_ground_mode || staging.valid ||
                (input.measured_feet_valid && contacts ==
                     static_cast<int>(go2::kLegCount));
            // STAGE retains all four measured contacts. Its basin witness is
            // the same convex polygon used by the state machine; FL's
            // lifted triangle is selected only after SHIFT chooses a leg.
            const bool measured_basin_ready = input.flat_ground_mode ||
                (input.measured_feet_valid && input.measured_com_valid &&
                 contacts == static_cast<int>(go2::kLegCount) &&
                 std::isfinite(TerrainMeasuredSupportMargin(
                     input.measured_feet_world, input.measured_contact,
                     go2::kLegCount, input.measured_com_world)) &&
                 TerrainMeasuredSupportMargin(
                     input.measured_feet_world, input.measured_contact,
                     go2::kLegCount, input.measured_com_world) >= kBasinMarginM);
            const bool settled = at_standoff && measured_basin_ready &&
                (input.flat_ground_mode || contacts ==
                    static_cast<int>(go2::kLegCount)) &&
                std::isfinite(input.measured_velocity_mps) &&
                input.measured_velocity_mps <=
                    (input.flat_ground_mode ? 0.12 : 0.04) &&
                input.measured_posture_valid &&
                std::abs(input.measured_roll_rad) <= 0.08 &&
                std::abs(input.measured_pitch_rad) <= 0.08;
            // Flat isolation has no terrain plan to break the legacy
            // APPROACH wait (its plan_valid is the sequencer target that is
            // only published after STAGE), so the legacy_stage_ready
            // handshake is unreachable there; keep the measured STAGE dwell
            // as the flat release instead of the Order-061 legacy gate.
            if (settled &&
                (input.flat_ground_mode || input.legacy_stage_ready))
            {
                if (!std::isfinite(stage_stable_start_s_))
                    stage_stable_start_s_ = input.now_s;
                const double stage_dwell_s = input.flat_ground_mode
                    ? 0.30 : kStageDwellS;
                if (finite_time && input.now_s - stage_stable_start_s_ + 1e-9 >=
                        stage_dwell_s)
                    SetState(TerrainCrawlSequencerState::kShift, input.now_s);
            }
            else
                stage_stable_start_s_ = std::numeric_limits<double>::infinity();
            if (!finite_time || input.now_s - state_enter_s_ + 1.0e-9 >=
                    kStageTimeoutS)
            {
                stage_abort_reason_ = 1;
                SetState(TerrainCrawlSequencerState::kAbort, input.now_s);
            }
            break;
        }
        case TerrainCrawlSequencerState::kShift:
            if (three_contacts && input.measured_feet_valid &&
                input.measured_com_valid && CurrentTarget(input))
            {
                const double measured_margin =
                    TerrainMeasuredSupportMargin(
                        input.measured_feet_world, input.measured_contact,
                        active_leg(), input.measured_com_world);
                const bool measured_margin_valid =
                    std::isfinite(measured_margin);
                if (measured_margin_valid &&
                    (measured_margin >= 0.0 ||
                     input.flat_ground_mode) &&
                    (input.flat_ground_mode || input.legacy_shift_ready) &&
                    finite_time && input.now_s - state_enter_s_ + 1e-9 >=
                        kShiftDwellS)
                {
                    CaptureTarget(input);
                    SetState(TerrainCrawlSequencerState::kSwing, input.now_s);
                }
            }
            break;
        case TerrainCrawlSequencerState::kSwing:
        {
            if (active_leg() >= go2::kLegCount)
                break;
            // COMMIT is a measured event: endpoint position and the
            // active-leg contact bit are insufficient without a force-backed
            // three-leg support witness. The same rule applies to flat mode.
            const bool force_supported =
                ForceSupportReady(input, active_leg());
            if (MeasuredTargetAtEndpoint(input))
                SetState(TerrainCrawlSequencerState::kCommit, input.now_s);
            else if ((!three_contacts && !force_supported &&
                      (!input.flat_ground_mode || !finite_time ||
                       input.now_s - state_enter_s_ + 1e-9 >=
                           kSwingSupportLossDeadlineS)) ||
                     (finite_time &&
                      input.now_s - state_enter_s_ + 1e-9 >= kSwingDurationS))
                SetState(TerrainCrawlSequencerState::kAbort, input.now_s);
            break;
        }
        case TerrainCrawlSequencerState::kCommit:
            if (MeasuredTargetAtEndpoint(input))
            {
                committed_[active_leg()] = true;
                if (ShouldAdvanceBodyAfterCommit())
                    SetState(TerrainCrawlSequencerState::kAdvance, input.now_s);
                else if (order_index_ + 1 < leg_order_values_.size())
                {
                    ++order_index_;
                    SetState(TerrainCrawlSequencerState::kShift, input.now_s);
                }
                else
                    SetState(TerrainCrawlSequencerState::kClear, input.now_s);
            }
            break;
        case TerrainCrawlSequencerState::kAdvance:
            // Body advance is an observed event: wait until the measured
            // support has translated into the rear-leg workspace. The
            // caller supplies that reachability through live FK, never via a
            // planner phase.
            if (three_contacts && input.measured_feet_valid &&
                input.rear_targets_fk_reachable &&
                ForceSupportReady(input, go2::kLegCount))
            {
                ++order_index_;
                SetState(TerrainCrawlSequencerState::kShift, input.now_s);
            }
            break;
        case TerrainCrawlSequencerState::kClear:
            if (input.base_clear && input.all_feet_clear)
            {
                if (input.flat_ground_mode)
                {
                    // Flat isolation is a continuous gait, not a transfer
                    // transaction. Re-arm the first leg directly while the
                    // four-foot landing support is still authoritative;
                    // RESUME would unnecessarily hand control back to the
                    // stochastic trot boundary between crawl cycles.
                    order_index_ = 0;
                    committed_.fill(false);
                    SetState(TerrainCrawlSequencerState::kShift, input.now_s);
                }
                else
                {
                    stable_start_s_ = input.now_s;
                    SetState(TerrainCrawlSequencerState::kResume, input.now_s);
                }
            }
            break;
        case TerrainCrawlSequencerState::kResume:
            if (input.stable && finite_time &&
                input.now_s - stable_start_s_ + 1e-9 >= kResumeDwellS)
            {
                if (input.flat_ground_mode)
                {
                    // Start the next flat cycle without dropping authority
                    // for one tick. A full Reset re-enters the running trot
                    // boundary and can lose the force/contact witness.
                    state_ = TerrainCrawlSequencerState::kStage;
                    order_index_ = 0;
                    state_enter_s_ = input.now_s;
                    stable_start_s_ = std::numeric_limits<double>::infinity();
                    stage_stable_start_s_ = std::numeric_limits<double>::infinity();
                    target_ = {};
                    swing_start_ = {};
                    target_valid_ = false;
                    committed_.fill(false);
                }
                else
                    Reset();
            }
            break;
        default: break;
        }
        return Publish(input);
    }

    TerrainCrawlSequencerState state() const noexcept { return state_; }
    std::size_t active_leg() const noexcept
    {
        return state_ == TerrainCrawlSequencerState::kShift ||
                state_ == TerrainCrawlSequencerState::kSwing ||
                state_ == TerrainCrawlSequencerState::kCommit
            ? (order_index_ < leg_order_values_.size()
                ? leg_order_values_[order_index_] : go2::kLegCount)
            : go2::kLegCount;
    }
    const TerrainCrawlSequencerOutput &output() const noexcept { return output_; }
    double state_enter_time_s() const noexcept { return state_enter_s_; }

private:
    bool ShouldAdvanceBodyAfterCommit() const noexcept
    {
        const std::size_t advance_index = advance_policy_ ==
            TerrainCrawlAdvancePolicy::kBeforeSecondStep ? 0 : 1;
        return order_index_ == advance_index;
    }

    void SetState(TerrainCrawlSequencerState state, double now_s) noexcept
    {
        state_ = state;
        state_enter_s_ = now_s;
        if (state == TerrainCrawlSequencerState::kShift ||
            state == TerrainCrawlSequencerState::kStage)
            target_valid_ = false;
        stage_stable_start_s_ = std::numeric_limits<double>::infinity();
    }

    bool CurrentTarget(const TerrainCrawlSequencerInput &input) noexcept
    {
        if (active_leg() >= go2::kLegCount ||
            !input.measured_feet_valid)
            return false;
        if (input.flat_ground_mode)
            return std::isfinite(input.flat_step_length_m) &&
                std::abs(input.flat_step_length_m) > 1.0e-4;
        if (input.staged_target_valid)
            return std::isfinite(input.staged_target_world.x) &&
                std::isfinite(input.staged_target_world.y) &&
                std::isfinite(input.staged_target_world.z);
        if (input.terrain == nullptr)
            return false;
        const double c = std::cos(input.base_yaw_rad);
        const double s = std::sin(input.base_yaw_rad);
        const auto &foot = input.measured_feet_world[active_leg()];
        const go2::Vec3 current_base{
            c * (foot.x - input.base_position_world.x) +
                s * (foot.y - input.base_position_world.y),
            -s * (foot.x - input.base_position_world.x) +
                c * (foot.y - input.base_position_world.y),
            foot.z - input.base_position_world.z};
        return MeasureTerrainScriptTarget(
            *input.terrain, static_cast<go2::Leg>(active_leg()), current_base).valid;
    }

    void CaptureTarget(const TerrainCrawlSequencerInput &input) noexcept
    {
        if (active_leg() >= go2::kLegCount || !input.measured_feet_valid)
            return;
        const auto &foot = input.measured_feet_world[active_leg()];
        if (input.flat_ground_mode)
        {
            const double c = std::cos(input.base_yaw_rad);
            const double s = std::sin(input.base_yaw_rad);
            target_ = {foot.x + c * input.flat_step_length_m,
                       foot.y + s * input.flat_step_length_m, foot.z};
            swing_start_ = foot;
            target_valid_ = std::isfinite(target_.x) &&
                std::isfinite(target_.y) && std::isfinite(target_.z);
            return;
        }
        if (input.terrain == nullptr && !input.staged_target_valid)
            return;
        if (input.staged_target_valid)
        {
            target_ = input.staged_target_world;
            swing_start_ = foot;
            target_valid_ = std::isfinite(target_.x) &&
                std::isfinite(target_.y) && std::isfinite(target_.z);
            return;
        }
        const double c = std::cos(input.base_yaw_rad);
        const double s = std::sin(input.base_yaw_rad);
        const go2::Vec3 current_base{
            c * (foot.x - input.base_position_world.x) +
                s * (foot.y - input.base_position_world.y),
            -s * (foot.x - input.base_position_world.x) +
                c * (foot.y - input.base_position_world.y),
            foot.z - input.base_position_world.z};
        const auto measured = MeasureTerrainScriptTarget(
            *input.terrain, static_cast<go2::Leg>(active_leg()), current_base);
        if (!measured.valid)
        {
            target_valid_ = false;
            return;
        }
        // Lidar elevations are contact-patch coordinates; the sequencer
        // output is consumed by FK/WBC as the foot-site center.  Apply the
        // calibrated patch-to-site offset at this direct handoff, matching
        // the planner execution path without changing flat mode.
        const auto measured_site = go2::ContactPatchToFootSite(
            measured.position_base);
        target_ = {
            input.base_position_world.x + c * measured_site.x -
                s * measured_site.y,
            input.base_position_world.y + s * measured_site.x +
                c * measured_site.y,
            input.base_position_world.z + measured_site.z};
        swing_start_ = foot;
        target_valid_ = true;
    }

    bool ForceSupportReady(const TerrainCrawlSequencerInput &input,
                           std::size_t excluded_leg) const noexcept
    {
        if (!input.measured_force_valid)
            return false;
        double total = 0.0;
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = 0.0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (leg == excluded_leg)
                continue;
            const double force = input.measured_normal_force_n[leg];
            if (!std::isfinite(force) || force < kStableForceMinN)
                return false;
            total += force;
            minimum = std::min(minimum, force);
            maximum = std::max(maximum, force);
        }
        return total >= kStableForceTotalMinN && minimum > 0.0 &&
            maximum / minimum <= kStableForceImbalanceRatio;
    }

    bool TargetPositionAtEndpoint(const TerrainCrawlSequencerInput &input) const noexcept
    {
        if (!target_valid_ || active_leg() >= go2::kLegCount ||
            !input.measured_feet_valid)
            return false;
        const auto &p = input.measured_feet_world[active_leg()];
        return std::hypot(std::hypot(p.x - target_.x, p.y - target_.y),
                          p.z - target_.z) <= kCommitToleranceM;
    }

    bool MeasuredTargetAtEndpoint(const TerrainCrawlSequencerInput &input) const noexcept
    {
        return TargetPositionAtEndpoint(input) && input.measured_contact_valid &&
            input.measured_contact[active_leg()] &&
            ForceSupportReady(input, active_leg());
    }

    TerrainCrawlSequencerState Publish(
        const TerrainCrawlSequencerInput &input) noexcept
    {
        output_ = {};
        output_.state = state_;
        output_.active_leg = active_leg();
        output_.target_valid = target_valid_;
        output_.control_authority_active = authority_active_ &&
            state_ != TerrainCrawlSequencerState::kInactive &&
            state_ != TerrainCrawlSequencerState::kAbort;
        output_.flat_ground_mode = input.flat_ground_mode;
        output_.measured_contact_count = input.measured_contact_valid
            ? static_cast<int>(std::count(input.measured_contact.begin(),
                                          input.measured_contact.end(), true))
            : 0;
        output_.authority_trot_full_contact_able = input.trot_full_contact_able;
        output_.authority_measured_contacts_ready = input.measured_contact_valid &&
            output_.measured_contact_count >= 3;
        output_.authority_velocity_mps = input.measured_velocity_mps;
        output_.authority_velocity_ready = std::isfinite(input.measured_velocity_mps) &&
            input.measured_velocity_mps <= 0.04;
        output_.authority_roll_rad = input.measured_roll_rad;
        output_.authority_pitch_rad = input.measured_pitch_rad;
        output_.authority_posture_ready = input.measured_posture_valid &&
            std::isfinite(input.measured_roll_rad) &&
            std::isfinite(input.measured_pitch_rad) &&
            std::abs(input.measured_roll_rad) <= 0.08 &&
            std::abs(input.measured_pitch_rad) <= 0.08;
        output_.authority_stand_transition_seen = stand_transition_seen_;
        if (!authority_active_ && !input.flat_ground_mode) {
            if (!stand_transition_seen_ && !input.trot_full_contact_able)
                output_.authority_block_reason = 1;
            else if (!output_.authority_measured_contacts_ready)
                output_.authority_block_reason = 2;
            else if (!output_.authority_velocity_ready)
                output_.authority_block_reason = 3;
            else if (!output_.authority_posture_ready)
                output_.authority_block_reason = 4;
        }
        output_.stage_abort_reason = stage_abort_reason_;
        output_.stand_transition_requested = !output_.control_authority_active &&
            input.measured_contact_valid &&
            std::count(input.measured_contact.begin(),
                       input.measured_contact.end(), true) >= 3;
        output_.swing_start_world = swing_start_;
        output_.target_world = target_;
        // The terrain target is a foot-site point above a contact patch.
        // Keep flat isolation unchanged, but lift the transfer apex above
        // the 5 cm surface step plus the calibrated foot-site offset.
        output_.swing_lift_m = input.flat_ground_mode ? 0.015 : 0.08;
        if ((output_.active_leg < go2::kLegCount ||
             state_ == TerrainCrawlSequencerState::kStage) &&
            input.measured_feet_valid && input.measured_com_valid)
        {
            // STAGE has no lifted leg: publish the full measured support
            // witness so the state-machine handoff never sees -inf solely
            // because the sequencer has not selected a swing leg yet.
            const std::size_t support_lifted_leg =
                state_ == TerrainCrawlSequencerState::kStage
                    ? go2::kLegCount : output_.active_leg;
            output_.com_margin_m = TerrainMeasuredSupportMargin(
                input.measured_feet_world, input.measured_contact,
                support_lifted_leg, input.measured_com_world);
        }
        output_.committed = committed_;
        output_.contact_schedule.fill(true);
        if (!output_.control_authority_active)
            output_.contact_schedule.fill(false);
        if (state_ == TerrainCrawlSequencerState::kSwing &&
            output_.active_leg < go2::kLegCount)
            output_.contact_schedule[output_.active_leg] = false;
        if (state_ == TerrainCrawlSequencerState::kInactive ||
            state_ == TerrainCrawlSequencerState::kAbort)
            output_.contact_schedule.fill(false);
        if (output_.control_authority_active &&
            authority_com_reference_valid_ &&
            (state_ == TerrainCrawlSequencerState::kStage ||
             state_ == TerrainCrawlSequencerState::kAdvance))
        {
            output_.com_reference_world = authority_com_reference_;
            output_.com_reference_valid = true;
        }
        else if (output_.control_authority_active && input.measured_feet_valid)
        {
            if (output_.active_leg < go2::kLegCount)
            {
                const auto triangle = ComputeTerrainSupportTriangle(
                    input.measured_feet_world, output_.active_leg);
                if (triangle.valid)
                {
                    output_.com_reference_world =
                        TerrainSupportTriangleIncenter(triangle);
                    output_.com_reference_valid = true;
                }
            }
            else if (state_ == TerrainCrawlSequencerState::kStage ||
                     state_ == TerrainCrawlSequencerState::kAdvance)
            {
                for (const auto &foot : input.measured_feet_world)
                {
                    output_.com_reference_world.x += foot.x;
                    output_.com_reference_world.y += foot.y;
                    output_.com_reference_world.z += foot.z;
                }
                output_.com_reference_world.x /= go2::kLegCount;
                output_.com_reference_world.y /= go2::kLegCount;
                output_.com_reference_world.z /= go2::kLegCount;
                output_.com_reference_valid = true;
            }
        }
        if (finite(input.now_s) && state_ == TerrainCrawlSequencerState::kSwing)
        {
            output_.swing_phase = std::clamp(
                (input.now_s - state_enter_s_) / kSwingDurationS, 0.0, 1.0);
            const double u = output_.swing_phase;
            // The FR event is the mixed-height touchdown crux; preserve
            // the proven FL transfer path and change only this descent.
            const bool terrain_swing = !input.flat_ground_mode &&
                active_leg() == 0;
            // Keep horizontal progress unchanged: the measured failure was
            // a touchdown wrench impulse, not an edge-corner collision.
            const double progress = u * u * (3.0 - 2.0 * u);
            const double progress_rate = 6.0 * u * (1.0 - u) /
                kSwingDurationS;
            // Ease the endpoint elevation as well as the lift arch. This
            // makes touchdown velocity zero instead of carrying the linear
            // height delta into the captured support.
            const double vertical_progress = terrain_swing
                ? u * u * (3.0 - 2.0 * u) : u;
            const double vertical_progress_rate = terrain_swing
                ? 6.0 * u * (1.0 - u) / kSwingDurationS :
                1.0 / kSwingDurationS;
            const double arch = terrain_swing
                ? output_.swing_lift_m * 16.0 * u * u *
                    (1.0 - u) * (1.0 - u)
                : output_.swing_lift_m * 4.0 * u * (1.0 - u);
            const double arch_rate = terrain_swing
                ? output_.swing_lift_m * 32.0 * u * (1.0 - u) *
                    (1.0 - 2.0 * u) / kSwingDurationS
                : output_.swing_lift_m * 4.0 * (1.0 - 2.0 * u) /
                    kSwingDurationS;
            output_.swing_position_world = {
                swing_start_.x + progress * (target_.x - swing_start_.x),
                swing_start_.y + progress * (target_.y - swing_start_.y),
                swing_start_.z + vertical_progress *
                    (target_.z - swing_start_.z) + arch};
            output_.swing_velocity_world = {
                progress_rate * (target_.x - swing_start_.x),
                progress_rate * (target_.y - swing_start_.y),
                vertical_progress_rate * (target_.z - swing_start_.z) +
                    arch_rate};
        }
        return state_;
    }

    static bool finite(double value) noexcept { return std::isfinite(value); }

    TerrainCrawlSequencerState state_ = TerrainCrawlSequencerState::kInactive;
    TerrainCrawlLegOrder leg_order_ = TerrainCrawlLegOrder::kLegacyFrontFirst;
    TerrainCrawlAdvancePolicy advance_policy_ =
        TerrainCrawlAdvancePolicy::kAfterSecondStep;
    std::array<std::size_t, 4> leg_order_values_ = kLegacyFrontFirstLegOrder;
    std::size_t order_index_ = 0;
    double state_enter_s_ = 0.0;
    double stable_start_s_ = std::numeric_limits<double>::infinity();
    double stage_stable_start_s_ = std::numeric_limits<double>::infinity();
    go2::Vec3 swing_start_{};
    go2::Vec3 target_{};
    bool target_valid_ = false;
    bool authority_active_ = false;
    go2::Vec3 authority_com_reference_{};
    bool authority_com_reference_valid_ = false;
    bool stand_transition_seen_ = false;
    int stage_abort_reason_ = 0;
    std::array<bool, go2::kLegCount> committed_{};
    TerrainCrawlSequencerOutput output_{};
};

}  // namespace go2_terrain
