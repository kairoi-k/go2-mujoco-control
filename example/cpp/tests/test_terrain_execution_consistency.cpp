#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "terrain_motion_plan.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

go2_terrain::TerrainMotionPlan FixturePlan()
{
    go2_terrain::TerrainMotionPlan plan;
    plan.plan_id = 7;
    plan.plan_epoch = 7;
    plan.map_epoch = 3;
    plan.state_stamp_s = 1.0;
    plan.generated_at_s = 1.0;
    plan.valid_until_s = 2.0;
    plan.knot_dt_s = 0.02;
    plan.frame_id = "base_link";
    plan.status = go2_terrain::TerrainPlanStatus::kValid;
    plan.horizon_knots = 8;
    plan.contact_schedule.measured_valid = true;
    plan.contact_schedule.planned_valid = true;
    plan.contact_schedule.measured_contact = {true, false, true, false};
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
    {
        plan.body_reference[k].valid = true;
        plan.contact_schedule.planned_contact[k] = {true, false, true, false};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            plan.predicted_foothold[k][leg].valid = true;
            plan.predicted_foothold[k][leg].position_world =
                {0.2 + 0.01 * static_cast<double>(k),
                 -0.1 + 0.02 * static_cast<double>(leg), -0.25};
        }
    }
    plan.predicted_foothold[1][0].touchdown = true;
    plan.predicted_foothold[1][0].touchdown_time_s = 1.02;
    plan.predicted_foothold[1][0].position_world = {0.31, -0.10, -0.24};
    plan.predicted_foothold[5][0].touchdown = true;
    plan.predicted_foothold[5][0].touchdown_time_s = 1.10;
    plan.predicted_foothold[5][0].position_world = {0.35, -0.10, -0.23};
    return plan;
}

// These four functions intentionally mirror the pre-H7 consumer behavior.
// The executable is expected to fail until the consistency helpers replace it.
bool LegacyOverrunMarksCurrentFootValid(
    const go2_terrain::TerrainMotionPlan &plan, double now_s)
{
    const auto lookup = go2_terrain::TerrainPlanKnotAtTime(plan, now_s);
    return !lookup.valid;
}

std::size_t LegacyFirstTouchdown(
    const go2_terrain::TerrainMotionPlan &plan, std::size_t leg)
{
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
        if (plan.predicted_foothold[k][leg].touchdown)
            return k;
    return plan.horizon_knots;
}

bool LegacyGaitMpcCoherent(std::uint64_t committed_plan_id,
                           std::uint64_t latest_mpc_plan_id)
{
    (void)committed_plan_id;
    return latest_mpc_plan_id != 0;
}

bool LegacySupportUsesOneSource(const go2_terrain::TerrainMotionPlan &plan)
{
    return plan.body_reference[0].position.x ==
        plan.body_reference[0].position.y &&
        plan.contact_schedule.measured_contact ==
            plan.contact_schedule.planned_contact[0];
}

} // namespace

int main()
{
    const auto plan = FixturePlan();
    if (!Check(!LegacyOverrunMarksCurrentFootValid(plan, 1.16),
               "legacy overrun still fabricates a valid current foot") ||
        !Check(LegacyFirstTouchdown(plan, 0) != 5,
               "legacy execution can reuse the first touchdown across cycles") ||
        !Check(!LegacyGaitMpcCoherent(7, 8),
               "legacy gait commitment was not checked against MPC plan") ||
        !Check(!LegacySupportUsesOneSource(plan),
               "legacy support path silently mixed time, COM, and contact sources"))
        return 1;
    std::cout << "legacy terrain execution defect witnesses unexpectedly passed.\n";
    return 0;
}
