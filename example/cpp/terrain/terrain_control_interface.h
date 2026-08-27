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

struct TerrainContactSchedule
{
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
