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

# This test must remain effective under the project's Release CMake build.
replace_once(
    "example/cpp/tests/test_terrain_bootstrap_c0.cpp",
    "#include <cassert>\n#include <cmath>",
    "#ifdef NDEBUG\n#undef NDEBUG\n#endif\n#include <cassert>\n#include <cmath>",
)

print("bootstrap runtime follow-up applied")
