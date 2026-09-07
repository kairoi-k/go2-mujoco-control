#!/usr/bin/env python3
"""V4 B1 traversal evidence analyzer.
V4 keeps the V3 physical, topology, speed, collision and stop-tail gates.  Its
versioned changes are the registered 5/10 cm scene height and finite running
period/duty constraints; the old V3 verdict is retained for comparison.
"""
from __future__ import annotations
import argparse
import bisect
import csv
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any
import analyze_b1_dynamic_v3 as legacy_v3
import analyze_b1_physical as physical
import b1_dynamic_protocol as protocol
CONTRACT_VERSION = "b1-dynamic-traversal-v4"
ALLOWED_HEIGHTS_M = (0.05, 0.10)
SUPERSEDED_LEGACY_GATES = (
    "registered_v3_scene",
    "stable_running_approach",
    "nominal_running_through_interaction",
)
RUNNING_STAGE = 2
RUNNING_ACTIVE = 1

def _finite(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))

def _safe(value: Any) -> Any:
    """Make diagnostics JSON-safe; non-finite evidence remains visibly null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(v) for v in value]
    return value

def _height_value(height: Any) -> float:
    h = float(height)
    if not math.isfinite(h) or not any(abs(h - x) <= 1e-9 for x in ALLOWED_HEIGHTS_M):
        raise ValueError("--height must be exactly 0.05 or 0.10 m")
    return h

def parse_scene(path: Path, expected_height: float) -> dict[str, float]:
    """Parse the ground-based step and bind its measured height to --height."""
    expected = _height_value(expected_height)
    root = ET.parse(path).getroot()
    boxes = []
    for geom in root.findall("./worldbody/geom"):
        if geom.get("type") != "box" or not str(geom.get("name", "")).startswith("phase2_step"):
            continue
        try:
            pos = [float(x) for x in str(geom.get("pos", "")).split()]
            size = [float(x) for x in str(geom.get("size", "")).split()]
        except (TypeError, ValueError):
            raise ValueError("step geom has invalid pos/size")
        if len(pos) != 3 or len(size) != 3 or not all(_finite(x) for x in pos + size):
            raise ValueError("step geom pos/size must be finite xyz")
        if not all(x > 0.0 for x in size):
            raise ValueError("step geom size must be positive")
        if abs(pos[2] - size[2]) > 1e-9:
            raise ValueError("step geom must be ground-based with centerz=half-height")
        top = pos[2] + size[2]
        if abs(top - expected) > 1e-9:
            raise ValueError("scene height does not match --height")
        boxes.append({
            "front": pos[0] - size[0], "rear": pos[0] + size[0],
            "left": pos[1] - size[1], "right": pos[1] + size[1],
            "top": top, "height_m": 2.0 * size[2],
            "center_x": pos[0], "center_y": pos[1], "center_z": pos[2],
        })
    if len(boxes) != 1:
        raise ValueError("scene must contain exactly one phase2_step box")
    return boxes[0]

def _time_order(rows: list[dict[str, Any]], field: str) -> bool:
    values = [physical.f(row, field) for row in rows]
    return bool(values) and all(_finite(x) for x in values) and all(
        b >= a for a, b in zip(values, values[1:]))


def _joined_rows(truth: list[dict[str, Any]], control: list[dict[str, Any]], start: float, end: float):
    ct = [physical.f(row, "state_tick_s") for row in control]
    if not ct or not all(_finite(x) for x in ct) or any(b < a for a, b in zip(ct, ct[1:])):
        return [], []
    joined, uncovered = [], []
    for row in truth:
        t = physical.f(row, "time_s")
        if not _finite(t) or not start - 0.8 <= t <= end:
            continue
        i = bisect.bisect_right(ct, t) - 1
        if i < 0 or t - ct[i] > 0.020000001:
            uncovered.append(t)
            continue
        joined.append(dict(control[i], **row))
    return joined, uncovered

def _stable_approach_v4(rows: list[dict[str, Any]], first_contact_s: float) -> dict[str, Any]:
    """V3 approach thresholds with only period/duty fixed constraints removed."""
    start = first_contact_s - 0.8
    selected = [r for r in rows if start <= physical.f(r, "time_s") < first_contact_s]
    reasons = []
    if not selected:
        reasons.append("missing approach rows")
    times = [physical.f(r, "time_s") for r in selected]
    if not times or not all(_finite(x) for x in times):
        reasons.append("nonfinite approach time")
    elif times[0] > start + 0.010000001 or times[-1] < first_contact_s - 0.010000001:
        reasons.append("approach truth coverage")
    periods = [physical.f(r, "velocity_command_gait_period_s") for r in selected]
    duties = [physical.f(r, "velocity_command_gait_duty") for r in selected]
    if not periods or not all(_finite(x) and x > 0.0 for x in periods):
        reasons.append("period must be finite and positive")
    if not duties or not all(_finite(x) and 0.0 < x < 0.5 for x in duties):
        reasons.append("duty must be finite and in (0,0.5)")
    request = [physical.f(r, "velocity_command_requested_mps") for r in selected]
    if not request or not all(_finite(x) and abs(x - 1.0) <= 0.02 for x in request):
        reasons.append("requested speed")
    stages = [physical.f(r, "motion_stage") for r in selected]
    active = [physical.f(r, "velocity_command_active") for r in selected]
    if not stages or any(x != RUNNING_STAGE for x in stages) or any(x != RUNNING_ACTIVE for x in active):
        reasons.append("running phase")
    speeds = [physical.f(r, "base_qvel_world_x_mps") for r in selected]
    if not speeds or not all(_finite(x) for x in speeds):
        reasons.append("nonfinite approach speed")
        speed_fraction = None
    else:
        speed_fraction = sum(0.75 <= x <= 1.25 for x in speeds) / len(speeds)
        if speed_fraction < 0.95:
            reasons.append("95 percent speed range")
    return {
        "status": "PASS" if not reasons else "NOT_CERTIFIED",
        "start_s": start, "end_s": first_contact_s,
        "period_s": {"min": min(periods) if periods else None, "max": max(periods) if periods else None},
        "duty": {"min": min(duties) if duties else None, "max": max(duties) if duties else None},
        "speed_in_band_fraction": speed_fraction, "reasons": reasons,
    }

def _running_period_duty(control: list[dict[str, Any]]) -> tuple[bool, int]:
    rows = [r for r in control if physical.f(r, "velocity_command_active") == RUNNING_ACTIVE]
    bad = sum(not (_finite(physical.f(r, "velocity_command_gait_period_s"))
                 and physical.f(r, "velocity_command_gait_period_s") > 0.0
                 and _finite(physical.f(r, "velocity_command_gait_duty"))
                 and 0.0 < physical.f(r, "velocity_command_gait_duty") < 0.5)
              for r in rows)
    return bool(rows) and bad == 0, bad

def _accepted(gates: dict[str, Any]) -> bool:
    return bool(gates) and all(bool(v) for k, v in gates.items() if k not in {"registered_v3_scene"})

def analyze(truth: list[dict[str, Any]], control: list[dict[str, Any]], box: dict[str, Any],
            height: float, log: str = "", statuses: dict[str, Any] | None = None,
            legacy_v3_verdict: dict[str, Any] | None = None) -> dict[str, Any]:
    """Analyze one run while preserving the old V3 result for audit comparison."""
    registered_height = _height_value(height)
    measured = box.get("height_m", box.get("height", box.get("top", math.nan)))
    height_ok = _finite(measured) and abs(float(measured) - registered_height) <= 1e-9
    base = legacy_v3.analyze(truth, control, box, log, statuses or {})
    gates = dict(base.get("gates", {}))
    detail = dict(base.get("detail", {}))
    issues = list(base.get("issues", []))
    gates["registered_scene_height"] = height_ok
    gates["v4_input_rows_nonempty"] = bool(truth) and bool(control)
    gates["v4_truth_time_order"] = _time_order(truth, "time_s")
    gates["v4_control_time_order"] = _time_order(control, "state_tick_s")
    expected_xy = {"front": 5.0, "rear": 5.5, "left": -0.75, "right": 0.75}
    gates["registered_stable_approach_geometry"] = all(
        _finite(box.get(key, math.nan)) and abs(float(box[key]) - value) <= 1e-9
        for key, value in expected_xy.items())
    period_ok, bad_period_rows = _running_period_duty(control)
    gates["running_period_duty_valid"] = period_ok
    detail["running_period_duty_bad_rows"] = bad_period_rows
    interaction = detail.get("interaction_s")
    if interaction and len(interaction) == 2 and all(_finite(x) for x in interaction):
        joined, uncovered = _joined_rows(truth, control, float(interaction[0]), float(interaction[1]))
        approach = _stable_approach_v4(joined, float(interaction[0]))
        interaction_rows = [r for r in joined if float(interaction[0]) <= physical.f(r, "time_s") <= float(interaction[1])]
        flexible_phase = bool(interaction_rows) and all(
            physical.f(r, "motion_stage") == RUNNING_STAGE
            and physical.f(r, "velocity_command_active") == RUNNING_ACTIVE
            and _finite(physical.f(r, "velocity_command_gait_period_s"))
            and physical.f(r, "velocity_command_gait_period_s") > 0.0
            and _finite(physical.f(r, "velocity_command_gait_duty"))
            and 0.0 < physical.f(r, "velocity_command_gait_duty") < 0.5
            for r in interaction_rows)
        coverage_ok = bool(joined) and not uncovered
        detail["approach_v4"] = approach
        detail["v4_uncovered_truth_rows"] = len(uncovered)
        # Reassert the inherited gate with the V4 join result: every required
        # truth row must be covered by a controller row, with no silent drops.
        gates["approach_interaction_join_coverage"] = coverage_ok
        gates["v4_required_truth_control_coverage"] = coverage_ok
        gates["stable_running_approach"] = approach["status"] == "PASS"
        gates["nominal_running_through_interaction"] = flexible_phase
    else:
        gates["approach_interaction_join_coverage"] = False
        gates["v4_required_truth_control_coverage"] = False
        gates["stable_running_approach"] = False
        gates["nominal_running_through_interaction"] = False
        issues.append("missing interaction for V4 running-phase checks")
    # V4 excludes only the legacy fixed scene gate from acceptance; the two
    # fixed period/duty gates retain their names for direct V3 comparisons but
    # use the flexible V4 semantics above.
    status = "PASS" if _accepted(gates) else "NOT_CERTIFIED"
    if legacy_v3_verdict is None:
        legacy_v3_verdict = ({"status": "SUPERSEDED", "reason": "legacy V3 hardcodes 5 cm bounds and fixed period/duty"}
                             if registered_height > 0.05 else
                             {"status": "NOT_RUN", "reason": "analyze() preserves a supplied V3 verdict; run() obtains it"})
    result = {
        "schema": CONTRACT_VERSION,
        "contract_version": CONTRACT_VERSION,
        "status": status,
        "claim": "single-run empirical evidence; B1 acceptance still requires the registered campaign and coverage review",
        "gates": gates,
        "issues": issues,
        "detail": detail,
        "scene_height_m": registered_height,
        "scene_bounds": dict(box),
        "superseded_legacy_gates": list(SUPERSEDED_LEGACY_GATES),
        "legacy_v3_verdict": legacy_v3_verdict,
        "architecture_claim": {
            "status": "NOT_ESTABLISHED",
            "basis": "joint backend adoption requires independent architecture evidence; terrain_actuation flags are not proof",
        },
        "physical_v2_diagnostic": base.get("physical_v2_diagnostic"),
        "legacy_statuses": statuses or {},
    }
    return _safe(result)

def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def _load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))

def _provenance(manifest: dict[str, Any], inputs: dict[str, str], scene_sha: str) -> dict[str, Any]:
    artifacts = manifest.get("artifacts", {})
    repo = manifest.get("repository", {})
    hashes_ok = all(re.fullmatch(r"[0-9a-f]{64}", str(artifacts.get(k, "")))
                    for k in ("controller_sha256", "simulator_sha256"))
    scene_match = artifacts.get("scenario_sha256") == scene_sha
    repo_ok = re.fullmatch(r"[0-9a-f]{40}", str(repo.get("git_commit", ""))) is not None and str(repo.get("git_dirty")).lower() == "false"
    return {
        "manifest_source": manifest,
        "recorded_run_binding": bool(scene_match and hashes_ok and repo_ok),
        "scene_hash_match": scene_match,
        "artifact_hashes_well_formed": hashes_ok,
        "repository_clean_and_committed": repo_ok,
        "input_sha256": inputs,
    }

def run(run_dir: Path, scene: Path, height: float) -> dict[str, Any]:
    height = _height_value(height)
    truth_path = run_dir / "contact_ground_truth.csv"
    data_path = run_dir / "data.csv"
    log_path = run_dir / "controller.log"
    manifest_path = run_dir / "run_manifest.json"
    inputs_paths = [truth_path, data_path, log_path, manifest_path, scene]
    if not all(p.is_file() for p in inputs_paths):
        missing = [str(p) for p in inputs_paths if not p.is_file()]
        raise FileNotFoundError("missing run input: " + ", ".join(missing))
    truth, control = _load_csv(truth_path), _load_csv(data_path)
    manifest = json.loads(manifest_path.read_text())
    box = parse_scene(scene, height)
    if abs(height - 0.05) <= 1e-9:
        try:
            legacy = legacy_v3.run(run_dir, scene)
        except Exception as exc:  # preserve failure as explicit non-verdict
            legacy = {"status": "NOT_RUN", "reason": f"legacy V3 execution failed: {exc}"}
    else:
        legacy = {"status": "SUPERSEDED", "reason": "legacy V3 hardcodes 5 cm bounds and fixed period/duty"}
    input_hashes = {str(p): _sha256(p) for p in inputs_paths}
    result = analyze(truth, control, box, height, log_path.read_text(), manifest.get("statuses", {}), legacy)
    repo_root = Path(__file__).resolve().parents[3]
    dependency_paths = {
        "v4": Path(__file__).resolve(),
        "legacy_v3": Path(legacy_v3.__file__).resolve(),
        "physical": Path(physical.__file__).resolve(),
        "legacy_protocol": Path(protocol.__file__).resolve(),
    }
    result["input_sha256"] = input_hashes
    result["analyzer_hashes"] = {name: _sha256(path) for name, path in dependency_paths.items()}
    result["provenance"] = _provenance(manifest, input_hashes, input_hashes[str(scene)])
    result["gates"]["recorded_run_binding"] = result["provenance"]["recorded_run_binding"]
    result["status"] = "PASS" if _accepted(result["gates"]) else "NOT_CERTIFIED"
    result["repository_root"] = str(repo_root)
    return _safe(result)

def write_exclusive(path: Path, result: dict[str, Any]) -> None:
    with path.open("x") as handle:
        json.dump(_safe(result), handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--height", type=float, required=True, choices=ALLOWED_HEIGHTS_M)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error(f"refusing to overwrite existing output: {args.out}")
    try:
        result = run(args.run_dir, args.scene, args.height)
        write_exclusive(args.out, result)
    except (OSError, ValueError, json.JSONDecodeError, csv.Error) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": result["status"], "gates": result["gates"]}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1

if __name__ == "__main__":
    sys.exit(main())
