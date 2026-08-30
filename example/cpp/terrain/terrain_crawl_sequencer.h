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
    // The window is armed before this boundary. Authority is granted only
    // when the running trot reports a four-contact-able phase.
    bool trot_full_contact_able = false;
    const TerrainModel *terrain = nullptr;
    go2::Vec3 base_position_world{};
    double base_yaw_rad = 0.0;
    double nominal_front_foot_x_m = 0.0;
    std::array<go2::Vec3, go2::kLegCount> measured_feet_world{};
    bool measured_feet_valid = false;
    std::array<bool, go2::kLegCount> measured_contact{};
    bool measured_contact_valid = false;
    go2::Vec3 measured_com_world{};
    bool measured_com_valid = false;
    double measured_velocity_mps = 0.0;
    bool measured_posture_valid = false;
    double measured_roll_rad = 0.0;
    double measured_pitch_rad = 0.0;
    bool rear_targets_fk_reachable = false;
    bool base_clear = false;
    bool all_feet_clear = false;
    bool stable = false;
    double now_s = 0.0;
};

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
    // A phase-respecting stop may be requested while still armed; this is
    // the trot pipeline slowing down, not sequencer contact ownership.
    bool stand_transition_requested = false;
    std::array<bool, go2::kLegCount> contact_schedule{};
    int measured_contact_count = 0;
};

class TerrainCrawlSequencer
{
public:
    // FL, FR, ADVANCE, RR, RL. Leg indices use the existing Go2 order.
    static constexpr std::array<std::size_t, 4> kLegOrder = {1, 0, 2, 3};
    static constexpr double kStandoffM = 0.25;
    static constexpr double kStageToleranceM = 0.015;
    static constexpr double kStageDwellS = 0.30;
    static constexpr double kShiftDwellS = 0.12;
    static constexpr double kSwingDurationS = 0.60;
    static constexpr double kCommitToleranceM = 0.045;
    static constexpr double kResumeDwellS = 0.45;

    static bool TransferActivationReady(
        const TerrainModel &terrain, const go2::Vec3 &base_position_world,
        double base_yaw_rad, double nominal_front_foot_x_m,
        double activation_distance_m = 0.45) noexcept
    {
        const auto staging = MeasureTerrainStagingReference(
            terrain, base_position_world, base_yaw_rad,
            nominal_front_foot_x_m, kStandoffM);
        return staging.valid && std::isfinite(staging.error_m) &&
            staging.error_m <= activation_distance_m;
    }

    std::size_t order_index() const noexcept { return order_index_; }

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
        authority_com_reference_valid_ = false;
        stand_transition_seen_ = false;
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
            const bool stand_boundary = input.trot_full_contact_able &&
                input.measured_contact_valid && contacts >= 3;
            if (stand_boundary && !stand_transition_seen_)
            {
                // Give the running trot one complete boundary tick to accept
                // the zero-velocity stand request before ownership changes.
                stand_transition_seen_ = true;
                return Publish(input);
            }
            if (stand_boundary && stand_transition_seen_ &&
                std::isfinite(input.measured_velocity_mps) &&
                input.measured_velocity_mps <= 0.04 &&
                input.measured_posture_valid &&
                std::abs(input.measured_roll_rad) <= 0.08 &&
                std::abs(input.measured_pitch_rad) <= 0.08)
            {
                authority_active_ = true;
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
                ? MeasureTerrainStagingReference(
                      *input.terrain, input.base_position_world,
                      input.base_yaw_rad, input.nominal_front_foot_x_m,
                      kStandoffM)
                : TerrainStagingReference{};
            const bool already_past = staging.valid && staging.error_m < 0.0;
            const bool at_standoff = staging.valid &&
                (already_past || std::abs(staging.error_m) <= kStageToleranceM);
            const bool settled = at_standoff && three_contacts &&
                std::isfinite(input.measured_velocity_mps) &&
                input.measured_velocity_mps <= 0.04 &&
                input.measured_posture_valid &&
                std::abs(input.measured_roll_rad) <= 0.08 &&
                std::abs(input.measured_pitch_rad) <= 0.08;
            if (settled)
            {
                if (!std::isfinite(stage_stable_start_s_))
                    stage_stable_start_s_ = input.now_s;
                if (finite_time && input.now_s - stage_stable_start_s_ + 1e-9 >=
                        kStageDwellS)
                    SetState(TerrainCrawlSequencerState::kShift, input.now_s);
            }
            else
                stage_stable_start_s_ = std::numeric_limits<double>::infinity();
            break;
        }
        case TerrainCrawlSequencerState::kShift:
            if (three_contacts && input.measured_feet_valid &&
                input.measured_com_valid && CurrentTarget(input))
            {
                const auto triangle = ComputeTerrainSupportTriangle(
                    input.measured_feet_world, active_leg());
                const auto metrics = triangle.valid
                    ? MeasureTerrainSupportTriangle(triangle,
                                                     input.measured_com_world)
                    : TerrainSupportTriangleMetrics{};
                if (metrics.valid && metrics.signed_margin_m >= 0.0 &&
                    finite_time && input.now_s - state_enter_s_ + 1e-9 >=
                        kShiftDwellS)
                {
                    CaptureTarget(input);
                    SetState(TerrainCrawlSequencerState::kSwing, input.now_s);
                }
            }
            break;
        case TerrainCrawlSequencerState::kSwing:
            if (active_leg() >= go2::kLegCount)
                break;
            if (MeasuredTargetAtEndpoint(input))
                SetState(TerrainCrawlSequencerState::kCommit, input.now_s);
            else if (!three_contacts || (finite_time &&
                     input.now_s - state_enter_s_ + 1e-9 >= kSwingDurationS))
                SetState(TerrainCrawlSequencerState::kAbort, input.now_s);
            break;
        case TerrainCrawlSequencerState::kCommit:
            if (MeasuredTargetAtEndpoint(input))
            {
                if (order_index_ == 1)
                    SetState(TerrainCrawlSequencerState::kAdvance, input.now_s);
                else if (order_index_ + 1 < kLegOrder.size())
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
                input.rear_targets_fk_reachable)
            {
                ++order_index_;
                SetState(TerrainCrawlSequencerState::kShift, input.now_s);
            }
            break;
        case TerrainCrawlSequencerState::kClear:
            if (input.base_clear && input.all_feet_clear)
            {
                stable_start_s_ = input.now_s;
                SetState(TerrainCrawlSequencerState::kResume, input.now_s);
            }
            break;
        case TerrainCrawlSequencerState::kResume:
            if (input.stable && finite_time &&
                input.now_s - stable_start_s_ + 1e-9 >= kResumeDwellS)
                Reset();
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
            ? (order_index_ < kLegOrder.size() ? kLegOrder[order_index_]
                                                : go2::kLegCount)
            : go2::kLegCount;
    }
    const TerrainCrawlSequencerOutput &output() const noexcept { return output_; }
    double state_enter_time_s() const noexcept { return state_enter_s_; }

private:
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
        if (active_leg() >= go2::kLegCount || input.terrain == nullptr ||
            !input.measured_feet_valid)
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
        if (active_leg() >= go2::kLegCount || input.terrain == nullptr ||
            !input.measured_feet_valid)
            return;
        const double c = std::cos(input.base_yaw_rad);
        const double s = std::sin(input.base_yaw_rad);
        const auto &foot = input.measured_feet_world[active_leg()];
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
        target_ = {
            input.base_position_world.x + c * measured.position_base.x -
                s * measured.position_base.y,
            input.base_position_world.y + s * measured.position_base.x +
                c * measured.position_base.y,
            input.base_position_world.z + measured.position_base.z};
        swing_start_ = foot;
        target_valid_ = true;
    }

    bool MeasuredTargetAtEndpoint(const TerrainCrawlSequencerInput &input) const noexcept
    {
        if (!target_valid_ || active_leg() >= go2::kLegCount ||
            !input.measured_feet_valid || !input.measured_contact_valid ||
            !input.measured_contact[active_leg()])
            return false;
        const auto &p = input.measured_feet_world[active_leg()];
        return std::hypot(std::hypot(p.x - target_.x, p.y - target_.y),
                          p.z - target_.z) <= kCommitToleranceM;
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
        output_.stand_transition_requested = !output_.control_authority_active &&
            input.trot_full_contact_able && input.measured_contact_valid &&
            std::count(input.measured_contact.begin(),
                       input.measured_contact.end(), true) >= 3;
        output_.swing_start_world = swing_start_;
        output_.target_world = target_;
        output_.measured_contact_count = input.measured_contact_valid
            ? static_cast<int>(std::count(input.measured_contact.begin(),
                                          input.measured_contact.end(), true))
            : 0;
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
                        TerrainSupportTriangleCentroid(triangle);
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
            const double progress = u * u * (3.0 - 2.0 * u);
            const double progress_rate = 6.0 * u * (1.0 - u) /
                kSwingDurationS;
            const double arch = output_.swing_lift_m * 4.0 * u * (1.0 - u);
            const double arch_rate = output_.swing_lift_m * 4.0 *
                (1.0 - 2.0 * u) / kSwingDurationS;
            output_.swing_position_world = {
                swing_start_.x + progress * (target_.x - swing_start_.x),
                swing_start_.y + progress * (target_.y - swing_start_.y),
                swing_start_.z + progress * (target_.z - swing_start_.z) + arch};
            output_.swing_velocity_world = {
                progress_rate * (target_.x - swing_start_.x),
                progress_rate * (target_.y - swing_start_.y),
                progress_rate * (target_.z - swing_start_.z) + arch_rate};
        }
        return state_;
    }

    static bool finite(double value) noexcept { return std::isfinite(value); }

    TerrainCrawlSequencerState state_ = TerrainCrawlSequencerState::kInactive;
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
    TerrainCrawlSequencerOutput output_{};
};

}  // namespace go2_terrain
