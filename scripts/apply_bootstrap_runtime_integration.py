#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def replace_once(path: str, old: str, new: str) -> None:
    p = ROOT / path
    text = p.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: expected exactly one patch anchor, found {count}")
    p.write_text(text.replace(old, new, 1))


# 1) C0 corridor direction must follow the runtime command direction.
replace_once(
    "example/cpp/terrain/terrain_bootstrap_c0.h",
    "    double forward_acceleration_mps2 = 0.0;\n"
    "    go2_trot::VelocityCommandShaperParams shaper{};",
    "    double forward_acceleration_mps2 = 0.0;\n"
    "    double forward_direction_sign = 1.0;\n"
    "    go2_trot::VelocityCommandShaperParams shaper{};",
)
replace_once(
    "example/cpp/terrain/terrain_bootstrap_c0.h",
    "        const double dx = alpha * out.stop.distance_m;\n"
    "        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)",
    "        const double direction = input.forward_direction_sign < 0.0 ? -1.0 : 1.0;\n"
    "        const double dx = direction * alpha * out.stop.distance_m;\n"
    "        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)",
)

# 2) Planner input explicitly records the pre-transfer bootstrap phase. It is
# part of the immutable input hash so a pre-armed snapshot cannot be confused
# with a later in-window replan.
replace_once(
    "example/cpp/terrain/terrain_planner.h",
    "    TerrainTimingBounds terrain_timing_bounds{};\n"
    "    bool has_stage_c_timing = false;",
    "    TerrainTimingBounds terrain_timing_bounds{};\n"
    "    bool has_stage_c_timing = false;\n"
    "    bool bootstrap_pretransfer = false;",
)
replace_once(
    "example/cpp/terrain/terrain_planner.h",
    "    add(&input.has_stage_c_timing, sizeof(input.has_stage_c_timing));\n"
    "    add(&input.terrain_timing_bounds, sizeof(input.terrain_timing_bounds));",
    "    add(&input.has_stage_c_timing, sizeof(input.has_stage_c_timing));\n"
    "    add(&input.bootstrap_pretransfer, sizeof(input.bootstrap_pretransfer));\n"
    "    add(&input.terrain_timing_bounds, sizeof(input.terrain_timing_bounds));",
)

# 3) Runtime owns only two cross-thread bootstrap facts: whether C0 currently
# certifies observation motion, and the exact latest publishable C1 plan id.
# The frozen shared_ptr is control-thread only and bridges the first adoption.
replace_once(
    "example/cpp/trot/trot_experiment.h",
    "    go2_terrain::TerrainPlanExecutionAdapter terrain_plan_execution_adapter_{};\n"
    "    // Stage-C contact truth is filtered once, then fused only with bounded",
    "    go2_terrain::TerrainPlanExecutionAdapter terrain_plan_execution_adapter_{};\n"
    "    std::atomic<bool> terrain_bootstrap_c0_ready_{false};\n"
    "    std::atomic<std::uint64_t> terrain_bootstrap_candidate_plan_id_{0};\n"
    "    std::shared_ptr<const go2_terrain::TerrainMotionPlan>\n"
    "        terrain_bootstrap_prearmed_plan_;\n"
    "    // Stage-C contact truth is filtered once, then fused only with bounded",
)

# 4) Pre-transfer planner work may build Stage-C timing in shadow while Phase 1
# still owns motion. This is opt-in and default-off.
replace_once(
    "example/cpp/trot/trot_experiment_control.cpp",
    '#include "full2_campaign_env.h"\n',
    '#include "full2_campaign_env.h"\n#include "terrain_bootstrap_c0.h"\n',
)
replace_once(
    "example/cpp/trot/trot_experiment_control.cpp",
    "    input.has_stage_c_timing = params_.stage_c_execution &&\n"
    "        control.terrain_transfer_window_active;",
    "    const bool bootstrap_dev =\n"
    "        Full2EnvDouble(\"TROT_TERRAIN_BOOTSTRAP_DEV\", 0.0) > 0.5;\n"
    "    input.bootstrap_pretransfer = bootstrap_dev &&\n"
    "        params_.stage_c_execution && params_.terrain_actuation &&\n"
    "        !params_.terrain_sensor_only &&\n"
    "        !control.terrain_transfer_window_active;\n"
    "    input.has_stage_c_timing = params_.stage_c_execution &&\n"
    "        (control.terrain_transfer_window_active ||\n"
    "         input.bootstrap_pretransfer);",
)
replace_once(
    "example/cpp/trot/trot_experiment_control.cpp",
    "        work.input.terrain = model.get();\n"
    "        const auto result = terrain_planner_.Build(work.input, work.plan_id);",
    "        work.input.terrain = model.get();\n"
    "\n"
    "        bool bootstrap_c0_ready = false;\n"
    "        bool bootstrap_roi_ready = false;\n"
    "        if (work.input.bootstrap_pretransfer && model)\n"
    "        {\n"
    "            go2_terrain::TerrainBootstrapC0Input c0;\n"
    "            c0.terrain = model.get();\n"
    "            c0.feasibility = terrain_planner_.config().feasibility;\n"
    "            c0.current_feet_base = work.input.current_feet_base;\n"
    "            const auto body_velocity = go2_terrain::RotateWorldVectorToBase(\n"
    "                work.input.base_yaw_rad, work.input.base_velocity_world);\n"
    "            c0.forward_speed_mps = std::abs(body_velocity.x);\n"
    "            // Use the maximum permitted positive shaper acceleration as\n"
    "            // a conservative discrete-stop initial condition.\n"
    "            c0.forward_acceleration_mps2 =\n"
    "                params_.velocity_command_shaper.max_accel_mps2;\n"
    "            c0.forward_direction_sign = params_.direction_sign;\n"
    "            c0.shaper = params_.velocity_command_shaper;\n"
    "            c0.dt_s = dt_;\n"
    "            const auto c0_result = go2_terrain::EvaluateTerrainBootstrapC0(c0);\n"
    "            bootstrap_c0_ready = c0_result.readiness.valid();\n"
    "\n"
    "            const auto nominal_feet = go2::AllFootPositions(\n"
    "                task_.stand_up_joint_pos_);\n"
    "            const double nominal_front_x =\n"
    "                0.5 * (nominal_feet[0].x + nominal_feet[1].x);\n"
    "            bootstrap_roi_ready =\n"
    "                go2_terrain::TerrainCrawlSequencer::TransferActivationReady(\n"
    "                    *model, work.input.base_position_world,\n"
    "                    work.input.base_yaw_rad, nominal_front_x);\n"
    "        }\n"
    "        terrain_bootstrap_c0_ready_.store(\n"
    "            bootstrap_c0_ready, std::memory_order_release);\n"
    "\n"
    "        const auto result = terrain_planner_.Build(work.input, work.plan_id);",
)

# After normal publish arbitration, pre-transfer bootstrap overrides only the
# velocity cap. Candidate present => transfer hold; no candidate + C0 valid =>
# observation motion; C0 invalid => brake/hold. The existing shaper remains
# the sole velocity writer.
replace_once(
    "example/cpp/trot/trot_experiment_control.cpp",
    "        else if (publish_allowed)\n"
    "        {\n"
    "            const auto previous = terrain_plan_store_.LoadUsable(\n"
    "                work.input.state_stamp_s);\n"
    "            if (!previous)\n"
    "            {\n"
    "                terrain_velocity_cap_mps_.store(0.0);\n"
    "                terrain_safe_stop_requested_.store(true);\n"
    "            }\n"
    "        }\n"
    "    }\n"
    "}",
    "        else if (publish_allowed)\n"
    "        {\n"
    "            const auto previous = terrain_plan_store_.LoadUsable(\n"
    "                work.input.state_stamp_s);\n"
    "            if (!previous)\n"
    "            {\n"
    "                terrain_velocity_cap_mps_.store(0.0);\n"
    "                terrain_safe_stop_requested_.store(true);\n"
    "            }\n"
    "        }\n"
    "\n"
    "        if (work.input.bootstrap_pretransfer)\n"
    "        {\n"
    "            const auto prearmed = terrain_plan_store_.LoadUsable(\n"
    "                work.input.state_stamp_s);\n"
    "            const bool candidate_ready = bootstrap_roi_ready && prearmed &&\n"
    "                prearmed->valid() && prearmed->has_stage_c_timing &&\n"
    "                !prearmed->v3_c_shadow;\n"
    "            terrain_bootstrap_candidate_plan_id_.store(\n"
    "                candidate_ready ? prearmed->plan_id : 0,\n"
    "                std::memory_order_release);\n"
    "            if (candidate_ready)\n"
    "            {\n"
    "                terrain_velocity_cap_mps_.store(0.0);\n"
    "                terrain_safe_stop_requested_.store(false);\n"
    "            }\n"
    "            else if (bootstrap_c0_ready)\n"
    "            {\n"
    "                terrain_velocity_cap_mps_.store(\n"
    "                    std::numeric_limits<double>::infinity());\n"
    "                terrain_safe_stop_requested_.store(false);\n"
    "            }\n"
    "            else\n"
    "            {\n"
    "                terrain_velocity_cap_mps_.store(0.0);\n"
    "                terrain_safe_stop_requested_.store(true);\n"
    "            }\n"
    "        }\n"
    "        else\n"
    "        {\n"
    "            terrain_bootstrap_candidate_plan_id_.store(\n"
    "                0, std::memory_order_release);\n"
    "        }\n"
    "    }\n"
    "}",
)

# 5) The transfer window may open in bootstrap mode only after C0 is valid,
# a sensor-derived ROI has a complete prepublished plan, the Phase-1 shaper
# has brought measured speed to a hold, and the adapter reports a legal first
# boundary. Freeze that exact store snapshot for the initial C1 adoption.
replace_once(
    "example/cpp/trot/trot_experiment_gait.cpp",
    "        if (activate)\n"
    "        {\n"
    "            terrain_transfer_window_active_ = true;",
    "        if (activate &&\n"
    "            Full2EnvDouble(\"TROT_TERRAIN_BOOTSTRAP_DEV\", 0.0) > 0.5)\n"
    "        {\n"
    "            const double bootstrap_now_s =\n"
    "                static_cast<double>(state_snapshot.tick()) * 1.0e-3;\n"
    "            const auto candidate =\n"
    "                terrain_plan_store_.LoadUsable(bootstrap_now_s);\n"
    "            const std::uint64_t candidate_plan_id =\n"
    "                terrain_bootstrap_candidate_plan_id_.load(\n"
    "                    std::memory_order_acquire);\n"
    "            const double measured_forward_speed = have_filtered_body_velocity_\n"
    "                ? std::abs(latest_filtered_body_velocity_[0])\n"
    "                : std::numeric_limits<double>::infinity();\n"
    "            activate = terrain_bootstrap_c0_ready_.load(\n"
    "                           std::memory_order_acquire) &&\n"
    "                candidate_plan_id != 0 && candidate &&\n"
    "                candidate->plan_id == candidate_plan_id &&\n"
    "                candidate->valid() && candidate->has_stage_c_timing &&\n"
    "                !candidate->v3_c_shadow &&\n"
    "                measured_forward_speed <= 0.05 &&\n"
    "                terrain_plan_execution_adapter_.IsLegalBoundary(\n"
    "                    bootstrap_now_s);\n"
    "            if (activate)\n"
    "                terrain_bootstrap_prearmed_plan_ = candidate;\n"
    "        }\n"
    "        if (activate)\n"
    "        {\n"
    "            terrain_transfer_window_active_ = true;",
)

# Initial C1 adoption consumes the frozen pretransfer snapshot rather than a
# newer asynchronous store value. Once adopted, the adapter's immutable copy
# remains authoritative for gait and SRBD/WBC exactly as in C-004.
replace_once(
    "example/cpp/trot/trot_experiment_gait.cpp",
    "        const auto timed_plan = terrain_plan_store_.LoadUsable(adapter_now_s);",
    "        const bool bootstrap_dev =\n"
    "            Full2EnvDouble(\"TROT_TERRAIN_BOOTSTRAP_DEV\", 0.0) > 0.5;\n"
    "        const auto timed_plan = bootstrap_dev &&\n"
    "                terrain_bootstrap_prearmed_plan_ &&\n"
    "                terrain_bootstrap_prearmed_plan_->usable_at(adapter_now_s)\n"
    "            ? terrain_bootstrap_prearmed_plan_\n"
    "            : terrain_plan_store_.LoadUsable(adapter_now_s);",
)
replace_once(
    "example/cpp/trot/trot_experiment_gait.cpp",
    "        const auto handoff = terrain_plan_execution_adapter_.Update(\n"
    "            timed_plan.get(), adapter_now_s, adapter_boundary,",
    "        const auto handoff = terrain_plan_execution_adapter_.Update(\n"
    "            timed_plan.get(), adapter_now_s, adapter_boundary,",
)
# no-op anchor above intentionally asserts the adoption call still exists.

print("bootstrap runtime integration applied")
