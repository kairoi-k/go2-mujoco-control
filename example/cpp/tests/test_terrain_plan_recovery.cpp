#include <array>
#include <cstddef>
#include <iostream>

#include "terrain_plan_execution_adapter.h"

namespace {

go2_terrain::TerrainMotionPlan MakeTimingPlan()
{
    go2_terrain::TerrainMotionPlan plan;
    plan.plan_id = 11;
    plan.input_hash = 101;
    plan.plan_epoch = 11;
    plan.map_epoch = 1;
    plan.state_stamp_s = 1.0;
    plan.generated_at_s = 1.0;
    plan.valid_until_s = 1.20;
    plan.identity.plan_id = plan.plan_id;
    plan.identity.plan_epoch = plan.plan_epoch;
    plan.identity.map_epoch = plan.map_epoch;
    plan.identity.generated_at_s = plan.generated_at_s;
    plan.identity.valid_until_s = plan.valid_until_s;
    plan.frame_id = "base_link";
    plan.status = go2_terrain::TerrainPlanStatus::kValid;
    plan.horizon_knots = 3;
    plan.has_stage_c_timing = true;
    plan.timing_bounds.window_start_s = 1.0;
    plan.timing_bounds.window_end_s = 1.20;
    plan.timing_bounds.knot_dt_s = 0.10;
    plan.contact_timing.identity = plan.identity;
    plan.contact_timing.horizon_knots = 3;
    plan.contact_timing.knot_dt_s = 0.10;
    plan.contact_timing.period_s = 0.80;
    plan.contact_timing.duty_factor = 0.75;
    plan.contact_timing.provenance =
        go2_terrain::TerrainTimingProvenance::kStageCPlanner;
    plan.contact_schedule.provenance = plan.identity;
    plan.contact_schedule.measured_contact = {false, true, true, true};
    plan.contact_schedule.measured_valid = true;
    plan.contact_schedule.planned_valid = true;
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
    {
        plan.contact_schedule.planned_contact[k] =
            k == 0 ? std::array<bool, go2::kLegCount>{false, true, true, true}
                   : std::array<bool, go2::kLegCount>{true, true, true, true};
        plan.body_reference[k].valid = true;
        plan.body_reference[k].provenance = plan.identity;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            auto &foot = plan.predicted_foothold[k][leg];
            foot.valid = true;
            foot.provenance = plan.identity;
            foot.position_world = {0.2, 0.1, 0.0};
        }
    }
    auto &touchdown = plan.predicted_foothold[1][0];
    touchdown.touchdown = true;
    touchdown.touchdown_time_s = 1.10;
    plan.contact_timing.touchdown_time_s[0] = 1.10;
    plan.contact_timing.touchdown_time_valid[0] = true;
    touchdown.swing_duration_s = 0.10;
    touchdown.swing_start_position_valid = true;
    touchdown.swing_start_position_world = {0.1, 0.1, 0.0};
    return plan;
}

go2_terrain::TerrainMotionPlan MakeReplacement(
    const go2_terrain::TerrainMotionPlan &source)
{
    auto plan = source;
    plan.plan_id = 12;
    plan.plan_epoch = 12;
    plan.map_epoch = 2;
    plan.identity.plan_id = plan.plan_id;
    plan.identity.plan_epoch = plan.plan_epoch;
    plan.identity.map_epoch = plan.map_epoch;
    plan.contact_timing.identity = plan.identity;
    plan.contact_schedule.provenance = plan.identity;
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
    {
        plan.body_reference[k].provenance = plan.identity;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            plan.predicted_foothold[k][leg].provenance = plan.identity;
    }
    return plan;
}

int Fail(const char *message)
{
    std::cerr << message << '\n';
    return 1;
}

} // namespace

int main()
{
    auto plan_a = MakeTimingPlan();
    auto plan_b = MakeReplacement(plan_a);
    if (!plan_a.valid() || !plan_b.valid())
        return Fail("recovery fixtures are not valid Stage-C plans");

    go2_terrain::TerrainPlanExecutionAdapter adapter(true, 0.10);
    const std::array<bool, go2::kLegCount> measured{
        false, true, true, true};

    const auto first = adapter.Update(
        &plan_a, 1.0, adapter.IsLegalBoundary(1.0), measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (!first.adopted || !first.using_plan ||
        first.request.plan_id != plan_a.plan_id)
        return Fail("initial Stage-C plan was not adopted");

    adapter.SetContactGuard(true, 5);
    const auto stopped = adapter.Update(
        &plan_a, 1.05, adapter.IsLegalBoundary(1.05), measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (stopped.using_plan || !stopped.request.fallback ||
        adapter.adopted_plan_id() != plan_a.plan_id)
        return Fail("safe-stop did not retire active execution");
    if (!adapter.IsLegalBoundary(1.05))
        return Fail("retired execution did not expose a recovery boundary");

    adapter.SetContactGuard(false, 0);
    const auto blocked = adapter.Update(
        &plan_b, 1.05, false, measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (blocked.using_plan || !blocked.request.fallback ||
        !blocked.rejected || adapter.adopted_plan_id() != plan_a.plan_id)
        return Fail("stale execution resumed before recovery adoption");

    const auto recovered = adapter.Update(
        &plan_b, 1.05, adapter.IsLegalBoundary(1.05), measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (!recovered.adopted || !recovered.using_plan ||
        recovered.request.fallback ||
        recovered.request.plan_id != plan_b.plan_id ||
        adapter.adopted_plan_id() != plan_b.plan_id)
        return Fail("fresh Stage-C plan was not atomically adopted in recovery");

    return 0;
}
