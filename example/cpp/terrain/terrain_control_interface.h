#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "go2_forward_kinematics.h"

namespace go2_terrain
{

// Shared, bounded interface storage.  Keep enough absolute-time knots for
// asynchronous planner latency plus the consumer horizon; this is storage
// capacity only and does not change the gait or MPC horizon.
constexpr std::size_t kTerrainContactMaxKnots = 48;

struct TerrainPlanIdentity
{
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    std::uint64_t map_epoch = 0;
    double generated_at_s = std::numeric_limits<double>::quiet_NaN();
    double valid_until_s = std::numeric_limits<double>::quiet_NaN();

    bool valid() const
    {
        return plan_id != 0 && plan_epoch != 0 && map_epoch != 0 &&
            std::isfinite(generated_at_s) &&
            std::isfinite(valid_until_s) &&
            valid_until_s >= generated_at_s;
    }

    bool usable_at(double now_s) const
    {
        return valid() && std::isfinite(now_s) && now_s <= valid_until_s;
    }
};

// Bounds are an input-only description of a future Stage-C timing window.
// They are intentionally inert until a later order enables the timing path.
struct TerrainTimingBounds
{
    double current_period_s = 0.8;
    double current_duty_factor = 0.58;
    double min_period_s = 0.10;
    double max_period_s = 2.00;
    double min_duty_factor = 0.50;
    double max_duty_factor = 0.95;
    double window_start_s = 0.0;
    double window_end_s = 0.0;
    double knot_dt_s = 0.020;
    std::array<double, go2::kLegCount> next_touchdown_time_s{};
    std::array<bool, go2::kLegCount> next_touchdown_time_valid{};
    std::array<double, go2::kLegCount> earliest_touchdown_time_s{};
    std::array<double, go2::kLegCount> latest_touchdown_time_s{};
    std::array<bool, go2::kLegCount> touchdown_window_valid{};

    bool valid() const
    {
        if (!std::isfinite(current_period_s) ||
            !std::isfinite(current_duty_factor) ||
            !std::isfinite(min_period_s) || !std::isfinite(max_period_s) ||
            !std::isfinite(min_duty_factor) ||
            !std::isfinite(max_duty_factor) ||
            !std::isfinite(window_start_s) || !std::isfinite(window_end_s) ||
            !std::isfinite(knot_dt_s) || knot_dt_s <= 0.0 ||
            min_period_s <= 0.0 || min_period_s > max_period_s ||
            min_duty_factor <= 0.0 || min_duty_factor >= max_duty_factor ||
            max_duty_factor >= 1.0 || window_end_s < window_start_s)
            return false;
        if (current_period_s < min_period_s ||
            current_period_s > max_period_s ||
            current_duty_factor < min_duty_factor ||
            current_duty_factor > max_duty_factor)
            return false;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (next_touchdown_time_valid[leg] &&
                (!std::isfinite(next_touchdown_time_s[leg]) ||
                 next_touchdown_time_s[leg] < window_start_s ||
                 next_touchdown_time_s[leg] > window_end_s))
                return false;
            if (touchdown_window_valid[leg] &&
                (!std::isfinite(earliest_touchdown_time_s[leg]) ||
                 !std::isfinite(latest_touchdown_time_s[leg]) ||
                 earliest_touchdown_time_s[leg] >
                     latest_touchdown_time_s[leg] ||
                 earliest_touchdown_time_s[leg] < window_start_s ||
                 latest_touchdown_time_s[leg] > window_end_s))
                return false;
        }
        return true;
    }
};

struct TerrainContactSchedule
{
    // Both measured and every planned knot belong to one immutable snapshot.
    TerrainPlanIdentity provenance{};
    // Measured contact is the current estimator/sensor observation.  Planned
    // contact is a future schedule supplied by the planner.  Neither field is
    // an implicit replacement for the applied WBC contact mask.
    std::array<bool, go2::kLegCount> measured_contact{};
    std::array<std::array<bool, go2::kLegCount>, kTerrainContactMaxKnots>
        planned_contact{};
    bool measured_valid = false;
    bool planned_valid = false;

    bool valid(std::size_t horizon_knots) const
    {
        return measured_valid && planned_valid && horizon_knots > 0 &&
            horizon_knots <= kTerrainContactMaxKnots;
    }
};

// Stretch the stance row immediately before a pending rear transition.  This
// is S1 timing: the command velocity and gait topology remain unchanged, while
// the already-loaded front stance is retained long enough for the body to
// translate into the rear-leg FK envelope.  Return false unless the requested
// advance fits in the bounded atomic schedule.
inline bool StretchTerrainFrontStanceSchedule(
    TerrainContactSchedule &schedule,
    const std::array<bool, go2::kLegCount> &transition_required,
    const std::array<bool, go2::kLegCount> &transition_committed,
    std::array<double, go2::kLegCount> &next_touchdown_time_s,
    const std::array<bool, go2::kLegCount> &next_touchdown_time_valid,
    double state_stamp_s, double commanded_vx_mps, double knot_dt_s,
    std::size_t horizon_knots, double advance_distance_m = 0.13,
    double minimum_advance_until_s = std::numeric_limits<double>::quiet_NaN())
{
    if (!schedule.valid(horizon_knots) || horizon_knots == 0 ||
        horizon_knots > kTerrainContactMaxKnots ||
        !std::isfinite(state_stamp_s) || !std::isfinite(commanded_vx_mps) ||
        std::abs(commanded_vx_mps) < 0.05 || !std::isfinite(knot_dt_s) ||
        knot_dt_s <= 0.0 || !std::isfinite(advance_distance_m) ||
        advance_distance_m <= 0.0)
        return false;

    bool front_committed = false;
    int rear_event = -1;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (leg < 2)
        {
            front_committed = front_committed || transition_committed[leg];
            continue;
        }
        // The rear requirement is discovered only after this preview
        // makes its foothold reachable; do not wait for that bit here.
        // A rear leg already committed to the upper surface is excluded.
        if (transition_committed[leg])
            continue;
        bool previous = schedule.measured_contact[leg];
        for (std::size_t k = 0; k < horizon_knots; ++k)
        {
            const bool planned = schedule.planned_contact[k][leg];
            if (planned && !previous)
            {
                const double knot_time = state_stamp_s +
                    static_cast<double>(k) * knot_dt_s;
                if (!next_touchdown_time_valid[leg] ||
                    !std::isfinite(next_touchdown_time_s[leg]) ||
                    knot_time + 0.5 * knot_dt_s >=
                        next_touchdown_time_s[leg])
                {
                    if (rear_event < 0 ||
                        static_cast<int>(k) < rear_event)
                        rear_event = static_cast<int>(k);
                    break;
                }
            }
            previous = planned;
        }
    }
    if (!front_committed || rear_event <= 0)
        return false;

    const int delay_knots = static_cast<int>(std::ceil(
        (std::isfinite(minimum_advance_until_s)
             ? std::max(0.0, minimum_advance_until_s -
                                (state_stamp_s +
                                 static_cast<double>(rear_event) * knot_dt_s))
             : advance_distance_m / (std::abs(commanded_vx_mps) * knot_dt_s))));
    if (delay_knots <= 0)
        return false;

    const auto original_schedule = schedule.planned_contact;
    auto &stretched = schedule.planned_contact;
    int dst = 0;
    for (int k = 0; k < static_cast<int>(horizon_knots); ++k)
    {
        if (k == rear_event)
        {
            const int source = std::max(0, k - 1);
            // A crawl advance can exceed one consumer horizon. Keep the
            // current horizon on the captured stance; the next snapshot
            // repeats this bounded operation until the fixed deadline enters
            // view, without publishing a partially shifted contact row.
            for (int j = 0; j < delay_knots && dst <
                 static_cast<int>(horizon_knots); ++j)
                stretched[static_cast<std::size_t>(dst++)] =
                    original_schedule[static_cast<std::size_t>(source)];
        }
        if (dst >= static_cast<int>(horizon_knots))
            break;
        stretched[static_cast<std::size_t>(dst++)] =
            original_schedule[static_cast<std::size_t>(k)];
    }
    const double event_time = state_stamp_s +
        static_cast<double>(rear_event) * knot_dt_s;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (next_touchdown_time_valid[leg] &&
            std::isfinite(next_touchdown_time_s[leg]) &&
            next_touchdown_time_s[leg] >= event_time - 0.5 * knot_dt_s)
            next_touchdown_time_s[leg] +=
                static_cast<double>(delay_knots) * knot_dt_s;
    }
    return true;
}

inline std::array<bool, go2::kLegCount> TerrainTransferPreviewContact(
    const std::array<bool, go2::kLegCount> &planned_contact,
    const std::array<bool, go2::kLegCount> &held_support,
    const std::array<bool, go2::kLegCount> &active_target)
{
    auto contact = planned_contact;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (held_support[leg])
            contact[leg] = true;
        if (active_target[leg])
            contact[leg] = false;
    }
    return contact;
}

} // namespace go2_terrain
