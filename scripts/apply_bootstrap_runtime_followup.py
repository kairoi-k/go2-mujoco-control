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


# The invalid sentinel is infinity, but a valid integration must start at zero.
# Release-mode assert removal previously hid this defect in the C0 test.
replace_once(
    "example/cpp/terrain/terrain_bootstrap_c0.h",
    "    double speed = std::clamp(speed_mps, 0.0, params.max_speed_mps);",
    "    out.distance_m = 0.0;\n"
    "    out.time_s = 0.0;\n"
    "    double speed = std::clamp(speed_mps, 0.0, params.max_speed_mps);",
)

# Do not read TrotTask-owned storage from the planner worker. The control
# snapshot already copied the nominal feet into TerrainPlannerInput.
replace_once(
    "example/cpp/trot/trot_experiment_control.cpp",
    "            const auto nominal_feet = go2::AllFootPositions(\n"
    "                task_.stand_up_joint_pos_);\n"
    "            const double nominal_front_x =\n"
    "                0.5 * (nominal_feet[0].x + nominal_feet[1].x);",
    "            const double nominal_front_x =\n"
    "                0.5 * (work.input.nominal_feet_base[0].x +\n"
    "                       work.input.nominal_feet_base[1].x);",
)

# Publishing a pre-transfer candidate into the shared plan store is only an
# ownership handoff mechanism. While C0 still owns locomotion, WBC must not
# consume that candidate's contact schedule before the adapter has adopted it.
replace_once(
    "example/cpp/trot/trot_experiment_wbc.cpp",
    "    const auto terrain_contact_plan =\n"
    "        params_.terrain_actuation && !params_.terrain_sensor_only\n"
    "            ? (stage_c_window",
    "    const bool bootstrap_c0_owner =\n"
    "        Full2EnvDouble(\"TROT_TERRAIN_BOOTSTRAP_DEV\", 0.0) > 0.5 &&\n"
    "        params_.stage_c_execution && !stage_c_window;\n"
    "    const auto terrain_contact_plan =\n"
    "        params_.terrain_actuation && !params_.terrain_sensor_only &&\n"
    "                !bootstrap_c0_owner\n"
    "            ? (stage_c_window",
)

# This test must remain effective under the project's Release CMake build.
replace_once(
    "example/cpp/tests/test_terrain_bootstrap_c0.cpp",
    "#include <cassert>\n#include <cmath>",
    "#ifdef NDEBUG\n#undef NDEBUG\n#endif\n#include <cassert>\n#include <cmath>",
)

# Stage-C already has a planner-owned velocity request/cap that feeds the
# Phase-1 jerk/acceleration shaper. Once C1 owns the transfer window, do not
# re-enter the legacy crawl/approach speed FSM: that would create an implicit
# second terrain policy and could restart creep immediately after the atomic
# zero-speed handoff. Keep only the plan cap on the existing shaper path.
replace_once(
    "example/cpp/trot/trot_experiment_gait.cpp",
    "    if ((params_.terrain_actuation && !params_.terrain_sensor_only) ||\n"
    "        flat_crawl_debug)\n"
    "    {\n"
    "        if (terrain_window_active)\n"
    "        {",
    "    const bool stage_c_velocity_owner = params_.stage_c_execution &&\n"
    "        params_.terrain_actuation && !params_.terrain_sensor_only &&\n"
    "        terrain_window_active;\n"
    "    if ((params_.terrain_actuation && !params_.terrain_sensor_only) ||\n"
    "        flat_crawl_debug)\n"
    "    {\n"
    "        if (stage_c_velocity_owner)\n"
    "        {\n"
    "            terrain_deceleration_active_ = false;\n"
    "            terrain_deceleration_target_mps_ = 0.0;\n"
    "            const double plan_cap_mps = terrain_velocity_cap_mps_.load();\n"
    "            if (std::isfinite(plan_cap_mps))\n"
    "                requested_mps = std::min(\n"
    "                    std::max(0.0, requested_mps),\n"
    "                    std::max(0.0, plan_cap_mps));\n"
    "            runtime_gait_regime_ = std::isfinite(plan_cap_mps)\n"
    "                ? \"terrain-stage-c-plan-cap\"\n"
    "                : \"terrain-stage-c-plan\";\n"
    "        }\n"
    "        else if (terrain_window_active)\n"
    "        {",
)

# The old staging-error micro-adjust is another legacy crawl speed override.
# It must remain unavailable while Stage-C owns the transfer window.
replace_once(
    "example/cpp/trot/trot_experiment_gait.cpp",
    "    if (terrain_window_active && !flat_crawl_debug &&\n"
    "        !terrain_crawl_sequencer_output_.body_advance_requested &&\n"
    "        !full_v2_shift &&\n"
    "        std::isfinite(terrain_staging_error_m_) &&",
    "    if (!stage_c_velocity_owner && terrain_window_active &&\n"
    "        !flat_crawl_debug &&\n"
    "        !terrain_crawl_sequencer_output_.body_advance_requested &&\n"
    "        !full_v2_shift &&\n"
    "        std::isfinite(terrain_staging_error_m_) &&",
)

print("bootstrap runtime follow-up applied")
