#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "terrain_execution_consistency.h"

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
    plan.body_reference[0].position = {0.0, 0.0, 0.42};
    plan.contact_schedule.measured_valid = true;
    plan.contact_schedule.planned_valid = true;
    plan.contact_schedule.measured_contact = {true, false, true, false};
    for (std::size_t k = 0; k < plan.horizon_knots; ++k)
    {
        plan.body_reference[k].valid = true;
        plan.body_reference[k].position = {0.0, 0.0, 0.42};
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

} // namespace

int main()
{
    const auto plan = FixturePlan();
    const auto first = go2_terrain::TerrainPlanNextTouchdown(plan, 0, 1.0);
    const auto next_cycle = go2_terrain::TerrainPlanNextTouchdown(plan, 0, 1.08);
    if (!Check(first.valid && first.knot == 1 &&
                   std::abs(first.event_time_s - 1.02) < 1.0e-12,
               "absolute-time touchdown lookup lost the first event") ||
        !Check(next_cycle.valid && next_cycle.knot == 5 &&
                   std::abs(next_cycle.event_time_s - 1.10) < 1.0e-12,
               "touchdown lookup reused the first event across cycles"))
        return 1;

    go2_terrain::TerrainExecutionCommitment commitment;
    commitment.valid = true;
    commitment.in_flight = true;
    commitment.source_plan_id = plan.plan_id;
    commitment.source_plan_epoch = plan.plan_epoch;
    commitment.target_time_s = next_cycle.event_time_s;
    commitment.target_world = next_cycle.target_world;
    std::array<go2_terrain::TerrainExecutionCommitment, go2::kLegCount>
        commitments{};
    commitments[0] = commitment;
    const std::array<bool, go2::kLegCount> measured{
        true, false, true, false};
    const std::array<bool, go2::kLegCount> applied{
        true, false, false, false};
    go2_terrain::TerrainExecutionSnapshot snapshot;
    if (!Check(go2_terrain::BuildTerrainExecutionSnapshot(
                   plan, 1.0, 0.02, 6, {0.05, 0.01, 0.33}, measured, true,
                   applied, true, commitments, snapshot),
               "complete execution snapshot was rejected") ||
        !Check(snapshot.valid && snapshot.horizon_covered &&
                   snapshot.model_com_world.x == 0.05 &&
                   snapshot.planned_contact[0][2] &&
                   snapshot.measured_contact[2] &&
                   !snapshot.applied_contact[2] &&
                   snapshot.touchdown_events[1][0].valid &&
                   snapshot.touchdown_events[5][0].valid &&
                   snapshot.touchdown_events[1][0].event_time_s == 1.02,
               "snapshot mixed time, COM, or contact provenance"))
        return 1;

    go2_terrain::TerrainPlanHorizonCoverage coverage;
    if (!Check(!go2_terrain::TerrainPlanCoversMpcHorizon(
                   plan, 1.04, 0.03, 8, &coverage) && !coverage.valid,
               "MPC overrun was silently accepted") ||
        !Check(!go2_terrain::BuildTerrainExecutionSnapshot(
                   plan, 1.04, 0.03, 8, {0.05, 0.01, 0.33}, measured, true,
                   applied, true, commitments, snapshot) && !snapshot.valid,
               "overrun fabricated a current foot and marked it valid"))
        return 1;

    auto inherited = commitments;
    inherited[0].source_plan_id = plan.plan_id + 1;
    inherited[0].source_plan_epoch = plan.plan_epoch + 1;
    if (!Check(go2_terrain::BuildTerrainExecutionSnapshot(
                   plan, 1.0, 0.02, 6, {0.05, 0.01, 0.33}, measured, true,
                   applied, true, inherited, snapshot) &&
                   snapshot.commitment_inherited[0],
               "matching touchdown was rejected solely because plan ID changed"))
        return 1;

    auto wrong_target = inherited;
    wrong_target[0].target_world.x += 0.02;
    if (!Check(!go2_terrain::BuildTerrainExecutionSnapshot(
                   plan, 1.0, 0.02, 6, {0.05, 0.01, 0.33}, measured, true,
                   applied, true, wrong_target, snapshot),
               "commitment target mismatch was not rejected"))
        return 1;

    auto previous = commitments;
    const auto inherited_update =
        go2_terrain::AdvanceTerrainExecutionCommitments(
            plan, 1.0, measured, true, previous);
    if (!Check(inherited_update.valid &&
                   inherited_update.commitments[0].target_time_s == 1.10,
               "initial touchdown commitment was not prepared"))
        return 1;
    auto replanned = plan;
    replanned.plan_id = 8;
    replanned.plan_epoch = 8;
    const auto replan_update =
        go2_terrain::AdvanceTerrainExecutionCommitments(
            replanned, 1.04, measured, true,
            inherited_update.commitments);
    if (!Check(replan_update.valid &&
                   (replan_update.inherited_mask & 1u) != 0,
               "cross-replan touchdown commitment was not inherited"))
        return 1;

    auto expired = plan;
    expired.valid_until_s = 1.05;
    if (!Check(!go2_terrain::BuildTerrainExecutionSnapshot(
                   expired, 1.0, 0.02, 6, {0.05, 0.01, 0.33}, measured, true,
                   applied, true, commitments, snapshot),
               "expired plan was marked as a complete snapshot"))
        return 1;

    std::cout << "terrain execution consistency checks passed.\n";
    return 0;
}
