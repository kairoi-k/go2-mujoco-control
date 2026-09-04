#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "go2_forward_kinematics.h"

namespace go2_control
{

struct HystereticContactParams
{
    double engage_force_n = 5.0;
    double release_force_n = 3.0;
};

inline bool UpdateHystereticContact(
    bool previous_contact,
    double force_n,
    const HystereticContactParams &params,
    bool &current_contact)
{
    if (!std::isfinite(force_n) ||
        !std::isfinite(params.engage_force_n) ||
        !std::isfinite(params.release_force_n) ||
        params.release_force_n > params.engage_force_n)
    {
        return false;
    }

    current_contact = previous_contact;
    if (previous_contact)
    {
        if (force_n <= params.release_force_n)
            current_contact = false;
    }
    else if (force_n >= params.engage_force_n)
    {
        current_contact = true;
    }
    return true;
}

// The force-filter output is the only measured-contact source. A planned
// contact is intentionally not an input here: prediction cannot promote a
// contact or satisfy a support gate.
template <std::size_t N>
inline bool UpdateHystereticContactArray(
    const std::array<double, N> &force_n,
    const HystereticContactParams &params,
    std::array<bool, N> &state,
    std::array<bool, N> &current)
{
    current = state;
    for (std::size_t leg = 0; leg < N; ++leg)
    {
        bool next = false;
        if (!UpdateHystereticContact(state[leg], force_n[leg], params, next))
            return false;
        current[leg] = next;
    }
    state = current;
    return true;
}

// Combine the gait schedule with force-supported early/late touchdown without
// turning a transient single measured contact into the WBC support model.
// Running trot intentionally permits zero-contact flight, but its supported
// intervals require at least the scheduled diagonal pair.
inline std::array<bool, go2::kLegCount> MergeRunningTrotContact(
    const std::array<bool, go2::kLegCount> &scheduled,
    const std::array<bool, go2::kLegCount> &measured,
    int mode) noexcept
{
    if (mode <= 0)
        return scheduled;

    std::array<bool, go2::kLegCount> merged = measured;
    if (mode >= 2)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            merged[leg] = merged[leg] || scheduled[leg];
    }
    else
    {
        std::size_t active = static_cast<std::size_t>(std::count(
            merged.begin(), merged.end(), true));
        for (std::size_t leg = 0;
             leg < go2::kLegCount && active < 2; ++leg)
        {
            if (!scheduled[leg] || merged[leg])
                continue;
            merged[leg] = true;
            ++active;
        }
    }

    const std::size_t active = static_cast<std::size_t>(std::count(
        merged.begin(), merged.end(), true));
    return active == 1 ? scheduled : merged;
}

enum class ContactFusionFallbackStage : int
{
    kNone = 0,
    kN = 1,
    kNPlus1 = 2,
    kNPlus5 = 5,
    kNPlus25 = 25,
};

struct ContactFusionResult
{
    std::array<bool, go2::kLegCount> measured_contact{};
    // Fused is measured contact plus only bounded last robust support.
    std::array<bool, go2::kLegCount> fused_contact{};
    std::array<bool, go2::kLegCount> robust_support{};
    bool measured_valid = false;
    bool guard_active = false;
    bool recovery = false;
    std::size_t measured_count = 0;
    std::size_t low_support_age_ticks = 0;
    std::size_t grace_remaining_ticks = 0;
    ContactFusionFallbackStage fallback_stage =
        ContactFusionFallbackStage::kNone;
    const char *reason = "none";
};

// Runtime planned/measured fusion for Stage-C. The one-tick grace is the
// existing filter-latency allowance; after it expires no support is
// fabricated and callers must use the braking/safe-stop chain.
class MeasuredContactFusion final
{
public:
    static constexpr std::size_t kMinimumMeasuredContacts = 3;
    static constexpr std::size_t kRobustSupportGraceTicks = 1;
    static constexpr std::size_t kSafeStopTicks = 5;
    static constexpr std::size_t kPostureStopTicks = 25;

    void Reset() noexcept
    {
        robust_support_.fill(false);
        have_robust_support_ = false;
        low_support_age_ticks_ = 0;
        guard_active_ = false;
    }

    ContactFusionResult Update(
        const std::array<bool, go2::kLegCount> &measured_contact,
        bool measured_valid, bool stage_c_active) noexcept
    {
        ContactFusionResult result;
        result.measured_contact = measured_contact;
        result.measured_valid = measured_valid;
        result.measured_count = measured_valid
            ? static_cast<std::size_t>(std::count(
                  measured_contact.begin(), measured_contact.end(), true))
            : 0;
        if (!stage_c_active)
        {
            Reset();
            result.fused_contact = measured_contact;
            result.robust_support = measured_contact;
            result.reason = "stage_c_inactive";
            return result;
        }
        if (!measured_valid)
        {
            AdvanceLowSupport();
            result.fused_contact = measured_contact;
            result.robust_support = robust_support_;
            result.guard_active = true;
            result.low_support_age_ticks = low_support_age_ticks_;
            result.fallback_stage = StageForAge(low_support_age_ticks_);
            result.reason = "measured-contact-invalid";
            return result;
        }
        if (result.measured_count >= kMinimumMeasuredContacts)
        {
            robust_support_ = measured_contact;
            have_robust_support_ = true;
            low_support_age_ticks_ = 0;
            const bool recovered = guard_active_;
            guard_active_ = false;
            result.fused_contact = measured_contact;
            result.robust_support = robust_support_;
            result.recovery = recovered;
            result.reason = recovered
                ? "measured-support-restored" : "measured-support-stable";
            return result;
        }

        AdvanceLowSupport();
        result.fused_contact = measured_contact;
        if (have_robust_support_ &&
            low_support_age_ticks_ <= kRobustSupportGraceTicks)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                result.fused_contact[leg] = measured_contact[leg] ||
                    robust_support_[leg];
            result.grace_remaining_ticks =
                kRobustSupportGraceTicks - low_support_age_ticks_;
            result.reason = "measured-support-grace";
        }
        else
            result.reason = "measured-support-below-three";
        result.robust_support = robust_support_;
        result.guard_active = true;
        result.low_support_age_ticks = low_support_age_ticks_;
        result.fallback_stage = StageForAge(low_support_age_ticks_);
        return result;
    }

    bool guard_active() const noexcept { return guard_active_; }
    std::size_t low_support_age_ticks() const noexcept
    { return low_support_age_ticks_; }

private:
    void AdvanceLowSupport() noexcept
    {
        if (!guard_active_)
            low_support_age_ticks_ = 0;
        else if (low_support_age_ticks_ < kPostureStopTicks)
            ++low_support_age_ticks_;
        guard_active_ = true;
    }

    static ContactFusionFallbackStage StageForAge(std::size_t age) noexcept
    {
        if (age == 0) return ContactFusionFallbackStage::kN;
        if (age == 1) return ContactFusionFallbackStage::kNPlus1;
        if (age < kSafeStopTicks) return ContactFusionFallbackStage::kNPlus1;
        if (age < kPostureStopTicks) return ContactFusionFallbackStage::kNPlus5;
        return ContactFusionFallbackStage::kNPlus25;
    }

    std::array<bool, go2::kLegCount> robust_support_{};
    bool have_robust_support_ = false;
    std::size_t low_support_age_ticks_ = 0;
    bool guard_active_ = false;
};

} // namespace go2_control
