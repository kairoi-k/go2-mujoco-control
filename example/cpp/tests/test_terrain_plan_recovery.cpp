#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include "terrain_plan_execution_adapter.h"

namespace {

class CaptureKernel final : public go2_control::LocomotionKernel
{
public:
    const char *Name() const noexcept override { return "capture"; }
    bool Compute(const go2_control::GaitKernelRequest &,
                 go2_control::GaitKernelResult &) override
    {
        return true;
    }

    void SetGaitPattern(go2_control::GaitPattern) override { ++setter_calls; }
    void SetGaitPeriod(double) override { ++setter_calls; }
    void SetGaitDuty(double) override { ++setter_calls; }
    void SetGaitStepLength(double) override { ++setter_calls; }
    void SetGaitFootLift(double) override { ++setter_calls; }

    int setter_calls = 0;
};

bool Near(double actual, double expected, double tolerance = 1.0e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

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
    plan.contact_schedule.measured_contact = {true, true, true, true};
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
    plan.contact_timing.liftoff_time_s[0] = 1.0;
    plan.contact_timing.liftoff_time_valid[0] = true;
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
    // Deliberately differs from the snapshot's planning-time observation.
    const std::array<bool, go2::kLegCount> measured{
        false, true, true, true};

    const auto first = adapter.Update(
        &plan_a, 1.0, adapter.IsLegalBoundary(1.0), measured,
        go2_control::GaitPattern::kRunningTrot,
        0.8, 0.75, 0.12, 0.05);
    if (!first.adopted || !first.using_plan ||
        first.request.plan_id != plan_a.plan_id ||
        first.request.pattern != go2_control::GaitPattern::kRunningTrot)
        return Fail("initial Stage-C plan was not adopted");

    adapter.SetContactGuard(true, 5);
    const auto stopped = adapter.Update(
        &plan_a, 1.05, adapter.IsLegalBoundary(1.05), measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (stopped.using_plan || !stopped.request.fallback ||
        adapter.adopted_plan_id() != plan_a.plan_id)
        return Fail("safe-stop did not retire active execution");
    if (adapter.IsLegalBoundary(1.05))
        return Fail("active contact guard exposed a recovery boundary");

    adapter.SetContactGuard(false, 0);
    if (!adapter.IsLegalBoundary(1.05))
        return Fail("cleared guard did not expose the recovery boundary");
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
        recovered.request.pattern != go2_control::GaitPattern::kDiagonalTrot ||
        adapter.adopted_plan_id() != plan_b.plan_id)
        return Fail("fresh Stage-C plan was not atomically adopted in recovery");

    CaptureKernel kernel;
    go2_control::GaitKernelRequest kernel_request{};
    std::array<go2::Vec3, go2::kLegCount> neutral_feet{};
    const double diagonal = std::sqrt(0.5);
    if (!adapter.ApplyToKernel(
        kernel, kernel_request, 1.05, neutral_feet, go2::Vec3{},
        std::array<double, 4>{diagonal, diagonal, 0.0, 0.0}, true))
        return Fail("adapter rejected a valid world-to-body conversion");
    const auto &execution = kernel_request.execution;
    if (!kernel_request.has_execution_request || !execution.frame_valid ||
        execution.measured_support != measured ||
        !execution.scheduled_support_valid || execution.scheduled_support[0])
        return Fail("adapter did not preserve current measured/frame validity");
    if (!Near(execution.swing_start[0].x, 0.10) ||
        !Near(execution.swing_start[0].y, 0.0) ||
        !Near(execution.swing_start[0].z, -0.10))
        return Fail("adapter did not transform the world swing start");
    if (!Near(execution.swing_endpoint[0].x, 0.20) ||
        !Near(execution.swing_endpoint[0].y,
              go2::kFootSiteToContactPatchOffsetM) ||
        !Near(execution.swing_endpoint[0].z, -0.10))
        return Fail("adapter did not transform the contact patch to foot site");
    if (!Near(execution.world_up_base.x, 0.0) ||
        !Near(execution.world_up_base.y, 1.0) ||
        !Near(execution.world_up_base.z, 0.0))
        return Fail("adapter did not rotate world-up into the body frame");

    CaptureKernel missing_pose_kernel;
    go2_control::GaitKernelRequest missing_pose_request{};
    if (adapter.ApplyToKernel(
            missing_pose_kernel, missing_pose_request, 1.05, neutral_feet) ||
        !missing_pose_request.has_execution_request ||
        missing_pose_request.execution.frame_valid ||
        missing_pose_kernel.setter_calls != 0)
        return Fail("legacy adapter overload silently accepted active plan");

    CaptureKernel invalid_pose_kernel;
    go2_control::GaitKernelRequest invalid_pose_request{};
    if (adapter.ApplyToKernel(
            invalid_pose_kernel, invalid_pose_request, 1.05, neutral_feet,
            go2::Vec3{},
            std::array<double, 4>{diagonal, std::numeric_limits<double>::quiet_NaN(),
                                  0.0, 0.0},
            true) || !invalid_pose_request.has_execution_request ||
        invalid_pose_request.execution.frame_valid ||
        invalid_pose_kernel.setter_calls != 0)
        return Fail("invalid pose silently reached nominal gait setters");

    CaptureKernel expired_kernel;
    go2_control::GaitKernelRequest expired_request{};
    if (adapter.ApplyToKernel(
            expired_kernel, expired_request, 1.30, neutral_feet, go2::Vec3{},
            std::array<double, 4>{1.0, 0.0, 0.0, 0.0}, true) ||
        !expired_request.has_execution_request ||
        expired_request.execution.frame_valid || expired_kernel.setter_calls != 0)
        return Fail("expired horizon index silently reached nominal gait");

    auto bad_endpoint_plan = MakeReplacement(plan_a);
    bad_endpoint_plan.plan_id = 13;
    bad_endpoint_plan.plan_epoch = 13;
    bad_endpoint_plan.map_epoch = 3;
    bad_endpoint_plan.BindIdentity();
    bad_endpoint_plan.predicted_foothold[1][0].position_world.x =
        std::numeric_limits<double>::max();
    if (!bad_endpoint_plan.valid())
        return Fail("invalid-endpoint fixture unexpectedly became invalid");
    go2_terrain::TerrainPlanExecutionAdapter bad_endpoint_adapter(true, 0.10);
    const auto bad_endpoint_handoff = bad_endpoint_adapter.Update(
        &bad_endpoint_plan, 1.0, true, measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (!bad_endpoint_handoff.adopted || !bad_endpoint_handoff.using_plan)
        return Fail("invalid-endpoint fixture was not adopted for the test");
    CaptureKernel bad_endpoint_kernel;
    go2_control::GaitKernelRequest bad_endpoint_request{};
    if (bad_endpoint_adapter.ApplyToKernel(
            bad_endpoint_kernel, bad_endpoint_request, 1.05, neutral_feet,
            go2::Vec3{-std::numeric_limits<double>::max(), 0.0, 0.0},
            std::array<double, 4>{1.0, 0.0, 0.0, 0.0}, true) ||
        !bad_endpoint_request.has_execution_request ||
        bad_endpoint_request.execution.frame_valid ||
        bad_endpoint_kernel.setter_calls != 0)
        return Fail("invalid endpoint silently reached nominal gait");

    auto single_event_plan = plan_a;
    single_event_plan.contact_schedule.measured_contact[0] = false;
    single_event_plan.contact_timing.liftoff_time_valid[0] = false;
    if (!single_event_plan.valid())
        return Fail("single-event fixture unexpectedly became invalid");
    go2_terrain::TerrainPlanExecutionAdapter single_event_adapter(true, 0.10);
    const auto single_event_handoff = single_event_adapter.Update(
        &single_event_plan, 1.05, true, measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (!single_event_handoff.adopted || !single_event_handoff.using_plan ||
        single_event_handoff.rejected ||
        single_event_handoff.request.plan_id != single_event_plan.plan_id)
        return Fail("single-event legal plan was not adopted");
    CaptureKernel single_event_kernel;
    go2_control::GaitKernelRequest single_event_request{};
    if (single_event_adapter.ApplyToKernel(
            single_event_kernel, single_event_request, 1.05, neutral_feet,
            go2::Vec3{}, std::array<double, 4>{1.0, 0.0, 0.0, 0.0}, true) ||
        single_event_request.execution.frame_valid ||
        single_event_kernel.setter_calls != 0)
        return Fail("single-event request reached the kernel");
    const auto expired_single_event = single_event_adapter.Update(
        nullptr, 1.21, false, measured,
        go2_control::GaitPattern::kDiagonalTrot,
        0.8, 0.75, 0.12, 0.05);
    if (expired_single_event.using_plan ||
        !expired_single_event.request.fallback)
        return Fail("expired single-event plan did not fall back");

    return 0;
}
