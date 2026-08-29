#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "go2_forward_kinematics.h"

namespace go2_terrain
{

enum class TerrainCrawlState : std::uint8_t
{
    kInactive = 0,
    kApproach,
    kDecelerateToCreep,
    kCrawlStep,
    kAdvanceBody,
    kClear,
    kResume,
    kAbort,
};

inline const char *TerrainCrawlStateName(TerrainCrawlState state) noexcept
{
    switch (state)
    {
    case TerrainCrawlState::kApproach: return "APPROACH";
    case TerrainCrawlState::kDecelerateToCreep: return "DECELERATE_TO_CREEP";
    case TerrainCrawlState::kCrawlStep: return "CRAWL_STEP";
    case TerrainCrawlState::kAdvanceBody: return "ADVANCE_BODY";
    case TerrainCrawlState::kClear: return "CLEAR";
    case TerrainCrawlState::kResume: return "RESUME";
    case TerrainCrawlState::kAbort: return "ABORT";
    default: return "INACTIVE";
    }
}

struct TerrainCrawlSignals
{
    bool transfer_window_active = false;
    bool plan_valid = false;
    bool measured_contact_valid = false;
    std::array<bool, go2::kLegCount> measured_contact{};
    double measured_velocity_mps = 0.0;
    std::array<bool, go2::kLegCount> target_valid{};
    std::array<bool, go2::kLegCount> committed{};
    bool rear_targets_fk_reachable = false;
    bool base_clear = false;
    bool all_feet_clear = false;
    bool stable = false;
    bool step_failed = false;
    double now_s = 0.0;
};

inline int TerrainCrawlContactCount(
    const std::array<bool, go2::kLegCount> &contact) noexcept
{
    return static_cast<int>(std::count(contact.begin(), contact.end(), true));
}

class TerrainCrawlStateMachine
{
public:
    static constexpr std::array<std::size_t, go2::kLegCount> kLegOrder =
        {1, 0, 2, 3};
    static constexpr int kMaxRetries = 2;

    void Reset() noexcept
    {
        state_ = TerrainCrawlState::kInactive;
        order_index_ = 0;
        retry_count_ = 0;
        state_enter_time_s_ = 0.0;
        stable_start_time_s_ = 0.0;
        transition_count_ = 0;
    }

    void Enter(double now_s) noexcept
    {
        state_ = TerrainCrawlState::kApproach;
        order_index_ = 0;
        retry_count_ = 0;
        state_enter_time_s_ = now_s;
        stable_start_time_s_ = 0.0;
        ++transition_count_;
    }

    TerrainCrawlState Update(const TerrainCrawlSignals &signals) noexcept
    {
        if (!signals.transfer_window_active)
        {
            if (state_ != TerrainCrawlState::kInactive &&
                state_ != TerrainCrawlState::kAbort)
                Reset();
            return state_;
        }
        if (state_ == TerrainCrawlState::kInactive)
            Enter(signals.now_s);
        if (state_ == TerrainCrawlState::kAbort)
            return state_;

        const int contacts = signals.measured_contact_valid
            ? TerrainCrawlContactCount(signals.measured_contact) : 0;
        const bool three_contacts = signals.measured_contact_valid &&
            contacts >= 3;
        const bool valid_time = std::isfinite(signals.now_s);
        switch (state_)
        {
        case TerrainCrawlState::kApproach:
            if (signals.plan_valid && three_contacts)
                SetState(TerrainCrawlState::kDecelerateToCreep,
                         signals.now_s);
            break;
        case TerrainCrawlState::kDecelerateToCreep:
            if (signals.plan_valid && three_contacts &&
                std::isfinite(signals.measured_velocity_mps) &&
                signals.measured_velocity_mps >= 0.05 &&
                signals.measured_velocity_mps <= 0.30 + 1.0e-6)
                SetState(TerrainCrawlState::kCrawlStep, signals.now_s);
            break;
        case TerrainCrawlState::kCrawlStep:
        {
            const std::size_t leg = ActiveLeg();
            // Losing the three-contact invariant is a state-machine abort,
            // not permission for the trot scheduler to create a flight phase.
            if (!three_contacts)
            {
                SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            if (signals.step_failed)
            {
                if (retry_count_ < kMaxRetries)
                    ++retry_count_;
                else
                    SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            if (leg >= go2::kLegCount || !three_contacts ||
                !signals.plan_valid || !signals.target_valid[leg] ||
                !signals.committed[leg])
                break;
            retry_count_ = 0;
            if (order_index_ == 1)
                SetState(TerrainCrawlState::kAdvanceBody, signals.now_s);
            else if (order_index_ + 1 < kLegOrder.size())
            {
                ++order_index_;
                SetState(TerrainCrawlState::kCrawlStep, signals.now_s);
            }
            else
                SetState(TerrainCrawlState::kClear, signals.now_s);
            break;
        }
        case TerrainCrawlState::kAdvanceBody:
            if (signals.plan_valid && three_contacts &&
                signals.rear_targets_fk_reachable)
            {
                ++order_index_;
                SetState(TerrainCrawlState::kCrawlStep, signals.now_s);
            }
            break;
        case TerrainCrawlState::kClear:
            if (three_contacts && signals.base_clear &&
                signals.all_feet_clear)
            {
                stable_start_time_s_ = valid_time ? signals.now_s : 0.0;
                SetState(TerrainCrawlState::kResume, signals.now_s);
            }
            break;
        case TerrainCrawlState::kResume:
            if (signals.stable && valid_time &&
                signals.now_s - stable_start_time_s_ >= 0.45)
                Reset();
            break;
        default:
            break;
        }
        return state_;
    }

    TerrainCrawlState state() const noexcept { return state_; }
    int retry_count() const noexcept { return retry_count_; }
    std::size_t order_index() const noexcept { return order_index_; }
    std::size_t ActiveLeg() const noexcept
    {
        return state_ == TerrainCrawlState::kCrawlStep &&
                order_index_ < kLegOrder.size()
            ? kLegOrder[order_index_]
            : go2::kLegCount;
    }
    double state_enter_time_s() const noexcept { return state_enter_time_s_; }
    double stable_start_time_s() const noexcept { return stable_start_time_s_; }
    std::uint64_t transition_count() const noexcept { return transition_count_; }
    bool aborted() const noexcept { return state_ == TerrainCrawlState::kAbort; }

private:
    void SetState(TerrainCrawlState state, double now_s) noexcept
    {
        if (state_ != state)
            ++transition_count_;
        state_ = state;
        state_enter_time_s_ = now_s;
    }

    TerrainCrawlState state_ = TerrainCrawlState::kInactive;
    std::size_t order_index_ = 0;
    int retry_count_ = 0;
    double state_enter_time_s_ = 0.0;
    double stable_start_time_s_ = 0.0;
    std::uint64_t transition_count_ = 0;
};

}  // namespace go2_terrain
