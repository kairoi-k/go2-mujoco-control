from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise SystemExit(f"{label}: expected exactly one seam, found {text.count(old)}")
    return text.replace(old, new)


# 1) Carry the exact current planner schedule row through the immutable
# execution request. Measured support stays separate.
path = Path("example/cpp/gait/locomotion_kernel.h")
text = path.read_text()
old = """    std::array<std::uint64_t, go2::kLegCount> endpoint_identity{};
    std::array<bool, go2::kLegCount> endpoint_valid{};
    std::array<bool, go2::kLegCount> measured_support{};
"""
new = """    std::array<std::uint64_t, go2::kLegCount> endpoint_identity{};
    std::array<bool, go2::kLegCount> endpoint_valid{};
    // Exact planner-owned support row for this control instant. This remains
    // planned contact only; measured_support is the independent safety input.
    std::array<bool, go2::kLegCount> scheduled_support{};
    bool scheduled_support_valid = false;
    std::array<bool, go2::kLegCount> measured_support{};
"""
if "scheduled_support_valid" not in text:
    text = replace_once(text, old, new, "GaitExecutionRequest")
path.write_text(text)

# 2) The adapter samples the same immutable plan timeline used by WBC/MPC.
path = Path("example/cpp/terrain/terrain_plan_execution_adapter.h")
text = path.read_text()
old = """        request.execution = last_request_;
        if (!request.has_execution_request)
            request.execution = {};
        if (last_request_.valid)
"""
new = """        request.execution = last_request_;
        if (!request.has_execution_request)
            request.execution = {};
        if (request.has_execution_request && adopted_)
        {
            std::array<std::size_t, kTerrainPlanMaxKnots> plan_index{};
            const double knot_dt_s = adopted_->contact_timing.knot_dt_s;
            if (BuildTerrainPlanHorizonIndices(
                    *adopted_, gait_time_s, knot_dt_s, knot_dt_s, 1,
                    plan_index))
            {
                request.execution.scheduled_support =
                    adopted_->contact_schedule.planned_contact[plan_index[0]];
                request.execution.scheduled_support_valid = true;
            }
        }
        if (last_request_.valid)
"""
if "plan_index{}" not in text:
    text = replace_once(text, old, new, "adapter ApplyToKernel")
path.write_text(text)

# 3) Raibert is the frozen B1 kernel. It must actually use the planner row for
# stance/swing selection rather than merely acknowledging it.
path = Path("example/cpp/gait/raibert_trot_kernel.h")
text = path.read_text()
old = """            return false;
        }

        // [Fix 2026-08-13] 连续相位累积: phase += dt / current_period
"""
new = """            return false;
        }

        const auto &execution = request.execution;
        const bool timed_execution =
            request.has_execution_request && execution.valid &&
            execution.scheduled_support_valid && !execution.fallback &&
            execution.plan_id != 0 && execution.plan_epoch != 0 &&
            execution.map_epoch != 0 && execution.input_hash != 0 &&
            std::isfinite(execution.valid_from_s) &&
            std::isfinite(execution.valid_until_s) &&
            request.gait_time_s + 1.0e-9 >= execution.valid_from_s &&
            request.gait_time_s <= execution.valid_until_s + 1.0e-9;

        // [Fix 2026-08-13] 连续相位累积: phase += dt / current_period
"""
if "const bool timed_execution" not in text:
    text = replace_once(text, old, new, "Raibert execution gate")

old = """        result.period_s = params_.gait.period_s;
        result.duty_factor = params_.gait.duty_factor;
        result.step_length_m = params_.gait.step_length_m;

        const RaibertFootstepPlannerParams planner_params{
"""
new = """        result.period_s = params_.gait.period_s;
        result.duty_factor = params_.gait.duty_factor;
        result.step_length_m = params_.gait.step_length_m;
        if (timed_execution)
        {
            // This acknowledgement is emitted only by the branch that below
            // consumes scheduled_support for the actual stance/swing choice.
            result.execution_request_valid = true;
            result.execution_plan_id = execution.plan_id;
            result.execution_plan_epoch = execution.plan_epoch;
            result.execution_map_epoch = execution.map_epoch;
            result.execution_input_hash = execution.input_hash;
        }

        const RaibertFootstepPlannerParams planner_params{
"""
if "acknowledgement is emitted only" not in text:
    text = replace_once(text, old, new, "Raibert identity")

old = """            double x_offset = 0.0;
            double y_offset = 0.0;
            double z_offset = 0.0;
            if (leg_phase < stance_duration)
            {
                const double stance_phase = leg_phase / stance_duration;
"""
new = """            double x_offset = 0.0;
            double y_offset = 0.0;
            double z_offset = 0.0;
            const bool scheduled_stance = timed_execution
                ? execution.scheduled_support[leg]
                : leg_phase < stance_duration;
            double execution_swing_phase = 0.0;
            if (timed_execution && !scheduled_stance)
            {
                if (!execution.liftoff_time_valid[leg] ||
                    !execution.touchdown_time_valid[leg] ||
                    !std::isfinite(execution.liftoff_time_s[leg]) ||
                    !std::isfinite(execution.touchdown_time_s[leg]) ||
                    execution.touchdown_time_s[leg] <=
                        execution.liftoff_time_s[leg])
                    return false;
                execution_swing_phase = std::clamp(
                    (request.gait_time_s - execution.liftoff_time_s[leg]) /
                        (execution.touchdown_time_s[leg] -
                         execution.liftoff_time_s[leg]),
                    0.0, 1.0);
            }
            if (scheduled_stance)
            {
                const double stance_phase = leg_phase / stance_duration;
"""
if "const bool scheduled_stance" not in text:
    text = replace_once(text, old, new, "Raibert stance/swing")

old = """            else
            {
                const double swing_phase =
                    (leg_phase - stance_duration) / swing_duration;
                const double swing_start_x =
"""
new = """            else
            {
                const double swing_phase = timed_execution
                    ? execution_swing_phase
                    : (leg_phase - stance_duration) / swing_duration;
                const double swing_start_x =
"""
if "? execution_swing_phase" not in text:
    text = replace_once(text, old, new, "Raibert swing phase")

old = """                z_offset =
                    params_.gait.foot_lift_m *
                    std::sin(kPi * swing_z_phase) *
"""
new = """                const double active_foot_lift_m = timed_execution
                    ? execution.foot_lift_m : params_.gait.foot_lift_m;
                z_offset =
                    active_foot_lift_m *
                    std::sin(kPi * swing_z_phase) *
"""
if "active_foot_lift_m" not in text:
    text = replace_once(text, old, new, "Raibert foot lift")
path.write_text(text)

# 4) Unit-test the actual control branch, not just the identity plumbing.
path = Path("example/cpp/tests/test_raibert_trot_kernel.cpp")
text = path.read_text()
if "CheckTimedExecutionOwnsSwingSupport" not in text:
    marker = """bool CheckSpeedAdaptiveStanceUsesMeasuredTravel()
{
"""
    insert = r'''bool CheckTimedExecutionOwnsSwingSupport()
{
    auto kernel = MakeKernel();
    auto request = Request(0.20, 0.105);
    request.has_execution_request = true;
    request.execution.valid = true;
    request.execution.plan_id = 41;
    request.execution.plan_epoch = 42;
    request.execution.map_epoch = 43;
    request.execution.input_hash = 44;
    request.execution.valid_from_s = 0.0;
    request.execution.valid_until_s = 1.0;
    request.execution.period_s = 0.8;
    request.execution.duty_factor = 0.75;
    request.execution.foot_lift_m = 0.05;
    request.execution.scheduled_support_valid = true;
    request.execution.scheduled_support = {false, true, true, true};
    const std::size_t fr = static_cast<std::size_t>(go2::Leg::FR);
    request.execution.liftoff_time_valid[fr] = true;
    request.execution.touchdown_time_valid[fr] = true;
    request.execution.liftoff_time_s[fr] = 0.10;
    request.execution.touchdown_time_s[fr] = 0.30;

    go2_control::GaitKernelResult planned{};
    if (!kernel.Compute(request, planned))
        return false;
    if (!planned.execution_request_valid ||
        planned.execution_plan_id != 41 ||
        planned.execution_plan_epoch != 42 ||
        planned.execution_map_epoch != 43 ||
        planned.execution_input_hash != 44 ||
        !(planned.feet[fr].z > 1.0e-4))
        return false;

    auto nominal_kernel = MakeKernel();
    auto nominal_request = Request(0.20, 0.105);
    go2_control::GaitKernelResult nominal{};
    if (!nominal_kernel.Compute(nominal_request, nominal))
        return false;
    return !nominal.execution_request_valid && Near(nominal.feet[fr].z, 0.0);
}

'''
    text = replace_once(text, marker, insert + marker, "Raibert test insertion")
    old_main = """        !CheckPreviewHorizonClosesLoop() ||
        !CheckPreviewPersistsWithinCycle() ||
        !CheckSpeedAdaptiveStanceUsesMeasuredTravel())
"""
    new_main = """        !CheckPreviewHorizonClosesLoop() ||
        !CheckPreviewPersistsWithinCycle() ||
        !CheckTimedExecutionOwnsSwingSupport() ||
        !CheckSpeedAdaptiveStanceUsesMeasuredTravel())
"""
    text = replace_once(text, old_main, new_main, "Raibert test main")
path.write_text(text)

# 5) Persist the previously probed C0->C1 handoff rule and exact consumer
# identity telemetry instead of relying on dynamic workflow-only patches.
path = Path("example/cpp/trot/trot_experiment_gait.cpp")
text = path.read_text()
old = """            activate = terrain_bootstrap_c0_ready_.load(
                           std::memory_order_acquire) &&
                candidate_plan_id != 0 && candidate &&
"""
new = """            // C0 readiness authorizes observation motion only. Once a complete
            // C1 candidate is ready and the body has settled at the boundary,
            // C0 is expected to have gone false and must not veto handoff.
            activate = candidate_plan_id != 0 && candidate &&
"""
if old in text:
    text = replace_once(text, old, new, "C0/C1 handoff")
elif new not in text:
    raise SystemExit("C0/C1 handoff seam not recognized")

old = """    if (!locomotion_kernel_->Compute(gait_request, gait_result))
    {
        std::cerr << \"Locomotion kernel failed at gait_time=\"
                  << gait_time_s << std::endl;
        return false;
    }
    kernel_footstep_plan_valid_ = gait_result.footstep_plan_valid;
"""
new = """    if (!locomotion_kernel_->Compute(gait_request, gait_result))
    {
        std::cerr << \"Locomotion kernel failed at gait_time=\"
                  << gait_time_s << std::endl;
        return false;
    }
    if (gait_result.execution_request_valid &&
        gait_result.execution_plan_id != 0)
    {
        static std::uint64_t last_logged_stage_c_gait_plan_id = 0;
        if (gait_result.execution_plan_id != last_logged_stage_c_gait_plan_id)
        {
            const auto adopted = terrain_plan_execution_adapter_.adopted_plan();
            const bool identity_match = adopted &&
                adopted->plan_id == gait_result.execution_plan_id &&
                adopted->plan_epoch == gait_result.execution_plan_epoch &&
                adopted->map_epoch == gait_result.execution_map_epoch &&
                adopted->input_hash == gait_result.execution_input_hash;
            std::cout << \"STAGE_C_IDENTITY consumer=gait plan_id=\"
                      << gait_result.execution_plan_id
                      << \" plan_epoch=\" << gait_result.execution_plan_epoch
                      << \" map_epoch=\" << gait_result.execution_map_epoch
                      << \" input_hash=\" << gait_result.execution_input_hash
                      << \" adapter_plan_id=\" << (adopted ? adopted->plan_id : 0)
                      << \" match=\" << (identity_match ? 1 : 0) << \"\\n\";
            last_logged_stage_c_gait_plan_id = gait_result.execution_plan_id;
        }
    }
    kernel_footstep_plan_valid_ = gait_result.footstep_plan_valid;
"""
if "STAGE_C_IDENTITY consumer=gait" not in text:
    text = replace_once(text, old, new, "gait identity proof")
path.write_text(text)

path = Path("example/cpp/trot/trot_experiment_wbc.cpp")
text = path.read_text()
old = """                ++terrain_mpc_plan_consumed_count_;
"""
new = """                ++terrain_mpc_plan_consumed_count_;
                static std::uint64_t last_logged_stage_c_mpc_plan_id = 0;
                if (terrain_plan->plan_id != last_logged_stage_c_mpc_plan_id)
                {
                    const auto adopted = terrain_plan_execution_adapter_.adopted_plan();
                    const bool identity_match = adopted &&
                        adopted->plan_id == terrain_plan->plan_id &&
                        adopted->plan_epoch == terrain_plan->plan_epoch &&
                        adopted->map_epoch == terrain_plan->map_epoch &&
                        adopted->input_hash == terrain_plan->input_hash;
                    std::cout << \"STAGE_C_IDENTITY consumer=mpc plan_id=\"
                              << terrain_plan->plan_id
                              << \" plan_epoch=\" << terrain_plan->plan_epoch
                              << \" map_epoch=\" << terrain_plan->map_epoch
                              << \" input_hash=\" << terrain_plan->input_hash
                              << \" adapter_plan_id=\" << (adopted ? adopted->plan_id : 0)
                              << \" match=\" << (identity_match ? 1 : 0) << \"\\n\";
                    last_logged_stage_c_mpc_plan_id = terrain_plan->plan_id;
                }
"""
if "STAGE_C_IDENTITY consumer=mpc" not in text:
    text = replace_once(text, old, new, "MPC identity proof")
path.write_text(text)

print("Applied Stage-C gait contact-ownership slice")
