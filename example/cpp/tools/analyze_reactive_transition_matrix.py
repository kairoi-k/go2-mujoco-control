#!/usr/bin/env python3
"""Analyze a directed reactive transition matrix with explicit QA gates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path


EVENT_NAMES = {
    0: "none", 1: "emergency_stop", 2: "obstacle_left", 3: "obstacle_right",
    4: "turn_left", 5: "turn_right", 6: "slip", 7: "low_friction", 8: "impact",
}


def now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [value for row in rows if math.isfinite(value := number(row, key))]


def median(rows: list[dict[str, str]], key: str) -> float:
    data = values(rows, key)
    return statistics.median(data) if data else math.nan


def max_abs(rows: list[dict[str, str]], key: str) -> float:
    data = values(rows, key)
    return max((abs(value) for value in data), default=math.nan)


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    metadata = path / "run_metadata.txt"
    if metadata.exists():
        for line in metadata.read_text(errors="replace").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                result[key] = value
    return result


def read_rows(path: Path) -> list[dict[str, str]]:
    with (path / "data.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def ground_truth_support(path: Path, start: float, end: float) -> tuple[int, float, float, int]:
    csv_path = path / "contact_ground_truth.csv"
    if not csv_path.exists():
        return 0, math.nan, math.nan, 0
    count = 0
    minimum = math.inf
    positive_minimum = math.inf
    zero_count = 0
    max_zero_run = 0
    zero_run = 0
    with csv_path.open(newline="", encoding="utf-8", errors="replace") as stream:
        for row in csv.DictReader(stream):
            time_s = number(row, "time_s")
            force_z = number(row, "total_contact_grf_world_z_N")
            if start <= time_s <= end and math.isfinite(force_z):
                count += 1
                minimum = min(minimum, force_z)
                unsupported = force_z <= 1.0
                if unsupported:
                    zero_count += 1
                    zero_run += 1
                    max_zero_run = max(max_zero_run, zero_run)
                else:
                    zero_run = 0
                    positive_minimum = min(positive_minimum, force_z)
    return (
        count,
        minimum if count else math.nan,
        positive_minimum if positive_minimum < math.inf else math.nan,
        max_zero_run,
    )


def slice_rows(rows: list[dict[str, str]], start: float, end: float) -> list[dict[str, str]]:
    return [row for row in rows if start <= number(row, "cmd_time_s") <= end]


def transition_sequence(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    previous: int | None = None
    for row in rows:
        event = int(number(row, "event_type", 0.0))
        if event != previous:
            result.append({
                "time_s": round(number(row, "cmd_time_s", 0.0), 4),
                "type": EVENT_NAMES.get(event, f"unknown_{event}"),
                "priority": int(number(row, "event_priority", 0.0)),
            })
            previous = event
    return result


def gait_start(rows: list[dict[str, str]]) -> float:
    data = [
        number(row, "cmd_time_s")
        for row in rows
        if int(number(row, "motion_stage", -1.0)) == 2
    ]
    return min(data) if data else math.nan


def max_reference_rate(rows: list[dict[str, str]], key: str,
                       start: float, end: float) -> float:
    selected = [row for row in rows if start <= number(row, "cmd_time_s") <= end]
    maximum = 0.0
    previous: tuple[float, float] | None = None
    for row in selected:
        time_s = number(row, "cmd_time_s")
        value = number(row, key)
        if not math.isfinite(time_s) or not math.isfinite(value):
            previous = None
            continue
        if previous is not None:
            dt = time_s - previous[0]
            if dt > 1.0e-4:
                maximum = max(maximum, abs(value - previous[1]) / dt)
        previous = (time_s, value)
    return maximum


def max_velocity_jump(rows: list[dict[str, str]], start: float, end: float) -> float:
    selected = [row for row in rows if start <= number(row, "cmd_time_s") <= end]
    maximum = 0.0
    previous: tuple[float, float, float] | None = None
    for row in selected:
        time_s = number(row, "cmd_time_s")
        vx = number(row, "world_velocity_x_mps")
        vy = number(row, "world_velocity_y_mps")
        if not all(math.isfinite(value) for value in (time_s, vx, vy)):
            previous = None
            continue
        if previous is not None and time_s - previous[0] > 1.0e-4:
            maximum = max(maximum, math.hypot(vx - previous[1], vy - previous[2]))
        previous = (time_s, vx, vy)
    return maximum


def signed_window_delta(rows: list[dict[str, str]], key: str,
                        start: float, end: float) -> float:
    selected = slice_rows(rows, start, end)
    if len(selected) < 4:
        return math.nan
    edge = max(2, len(selected) // 10)
    return median(selected[-edge:], key) - median(selected[:edge], key)


def signed_tail_median(rows: list[dict[str, str]], key: str,
                       start: float, end: float) -> float:
    selected = slice_rows(rows, start, end)
    if len(selected) < 4:
        return math.nan
    begin = max(0, int(len(selected) * 0.70))
    return median(selected[begin:], key)


def expected_target_ok(event: str, event_rows: list[dict[str, str]]) -> bool:
    if not event_rows:
        return False
    target_vx = median(event_rows, "event_target_vx_mps")
    target_vy = median(event_rows, "event_target_vy_mps")
    target_yaw = median(event_rows, "event_target_yaw_rate_radps")
    if event == "turn_left":
        return target_yaw >= 0.20
    if event == "turn_right":
        return target_yaw <= -0.20
    if event == "obstacle_left":
        return target_vy >= 0.20 and target_yaw >= 0.10
    if event == "obstacle_right":
        return target_vy <= -0.20 and target_yaw <= -0.10
    if event in {"emergency_stop", "impact"}:
        return abs(target_vx) <= 0.02
    if event in {"slip", "low_friction"}:
        return math.isfinite(target_vx) and 0.0 <= target_vx <= 0.13
    return False


def actual_event_response_ok(event: str, rows: list[dict[str, str]],
                            start: float, end: float) -> bool:
    if event in {"turn_left", "turn_right"}:
        delta = signed_window_delta(rows, "imu_yaw_rad", start, end)
        tail_rate = signed_tail_median(rows, "imu_gyro_z_radps", start, end)
        sign = 1.0 if event == "turn_left" else -1.0
        return (math.isfinite(delta) and math.isfinite(tail_rate)
                and sign * delta >= 0.02 and sign * tail_rate >= 0.02)
    if event in {"obstacle_left", "obstacle_right"}:
        delta_y = signed_window_delta(rows, "world_base_y_m", start, end)
        tail_velocity = signed_tail_median(rows, "world_velocity_y_mps", start, end)
        sign = 1.0 if event == "obstacle_left" else -1.0
        return (math.isfinite(delta_y) and math.isfinite(tail_velocity)
                and (sign * delta_y >= 0.005
                     or sign * tail_velocity >= 0.005))
    if event == "emergency_stop":
        hold = [int(number(row, "event_hold_stance", 0.0)) for row in rows]
        return bool(hold) and max(hold) == 1
    return True


def analyze_run(root: Path, manifest_path: Path,
                protocol: dict[str, object]) -> dict[str, object]:
    manifest = read_json(manifest_path)
    run_dir = Path(str(manifest["run_directory"]))
    source = str(manifest["source_event"])
    target = str(manifest["target_event"])
    result: dict[str, object] = {
        "pair_index": manifest.get("pair_index"),
        "run_id": manifest.get("run_id"),
        "source_event": source, "target_event": target,
        "run_directory": str(run_dir),
        "controller_config_fingerprint": manifest.get("controller_config_fingerprint"),
    }
    if not (run_dir / "data.csv").exists():
        result.update({"strict_pass": False, "failure": "missing data.csv"})
        return result
    rows = read_rows(run_dir)
    metadata = read_metadata(run_dir)
    statuses = {
        key: int(metadata.get(key, -1))
        for key in (
            "controller_status", "safety_status", "quality_status",
            "analysis_status", "ground_truth_status", "dynamics_status",
            "completion_status",
        )
    }
    times = [number(row, "cmd_time_s") for row in rows]
    positive_dt = [b - a for a, b in zip(times, times[1:]) if b - a > 1.0e-4]
    duplicate_rows = sum(1 for a, b in zip(times, times[1:]) if b - a <= 1.0e-4)
    start_gait = gait_start(rows)
    event_start_rel = float(protocol["event_start_s"])
    event_duration = float(protocol["event_duration_s"])
    first_start = start_gait + event_start_rel
    first_end = first_start + event_duration
    second_start = first_end
    second_end = second_start + event_duration
    post_end = second_end + float(protocol["post_event_window_s"])
    first_rows = slice_rows(rows, first_start, first_end)
    second_rows = slice_rows(rows, second_start, second_end)
    transition_rows = slice_rows(rows, first_start, post_end)
    (ground_truth_rows, min_ground_truth_contact_z,
     min_positive_ground_truth_contact_z, max_ground_truth_zero_run) = ground_truth_support(
        run_dir, first_start, post_end)
    observed = transition_sequence(rows)
    observed_types = [item["type"] for item in observed]
    expected_types = ["none", source, target]
    if target != "emergency_stop":
        expected_types.append("none")
    if target == "emergency_stop":
        log_text = (run_dir / "controller.log").read_text(errors="replace")
        terminal_hold_ok = "Emergency stop hold complete; ending in WBC stance" in log_text
    else:
        terminal_hold_ok = True
    expected_sequence_ok = observed_types == expected_types
    stage_values = [int(number(row, "motion_stage", -1.0)) for row in transition_rows]
    stage_stable = bool(stage_values) and set(stage_values) == {2}
    limits = dict(protocol["reference_rate_limits"])
    ref_rates = {
        "vx_mps_per_s": max_reference_rate(rows, "event_ref_vx_mps", first_start, post_end),
        "vy_mps_per_s": max_reference_rate(rows, "event_ref_vy_mps", first_start, post_end),
        "yaw_rate_radps_per_s": max_reference_rate(rows, "event_ref_yaw_rate_radps", first_start, post_end),
    }
    rate_ok = all(ref_rates[key] <= float(limit) + 0.10 for key, limit in limits.items())
    velocity_jump = max_velocity_jump(rows, first_start, post_end)
    roll = max_abs(transition_rows, "imu_roll_rad")
    pitch = max_abs(transition_rows, "imu_pitch_rad")
    contact_values = [int(number(row, "contact_count", 0.0)) for row in transition_rows]
    residual_values = values(transition_rows, "wbc_full_eq_residual")
    solver_residual = max(residual_values, default=math.nan)
    finite_required = all(
        math.isfinite(number(row, key))
        for row in transition_rows
        for key in (
            "event_ref_vx_mps", "event_ref_vy_mps", "event_ref_yaw_rate_radps",
            "world_velocity_x_mps", "world_velocity_y_mps", "imu_roll_rad",
            "imu_pitch_rad",
        )
    )
    first_target_ok = expected_target_ok(source, first_rows)
    second_target_ok = expected_target_ok(target, second_rows)
    first_response_ok = actual_event_response_ok(source, rows, first_start, first_end)
    second_response_ok = actual_event_response_ok(target, rows, second_start, second_end)
    if target == "emergency_stop":
        hold_tail = [
            int(number(row, "event_hold_stance", 0.0))
            for row in rows
            if second_end <= number(row, "cmd_time_s") <= post_end
        ]
        second_response_ok = bool(hold_tail) and max(hold_tail) == 1
    status_ok = bool(rows) and all(value == 0 for value in statuses.values())
    csv_ok = (
        len(rows) >= 4000 and math.isfinite(start_gait) and len(positive_dt) >= 100
        and all(b + 1.0e-6 >= a for a, b in zip(times, times[1:]))
        and len(first_rows) >= 300 and len(second_rows) >= 300
    )
    acceptance_limits = dict(protocol["acceptance_limits"])
    posture_ok = (
        math.isfinite(roll) and math.isfinite(pitch)
        and roll <= float(acceptance_limits["max_abs_roll_rad"])
        and pitch <= float(acceptance_limits["max_abs_pitch_rad"])
    )
    dynamic_ok = (
        rate_ok
        and velocity_jump <= float(acceptance_limits["max_actual_velocity_jump_mps"])
        and ground_truth_rows >= 100
        and math.isfinite(min_positive_ground_truth_contact_z)
        and min_positive_ground_truth_contact_z >= 1.0
        and max_ground_truth_zero_run <= 5
        and math.isfinite(solver_residual)
        and solver_residual <= float(acceptance_limits["max_solver_residual"])
    )
    result.update({
        "rows": len(rows), "max_time_s": max(times, default=math.nan),
        "duplicate_time_rows": duplicate_rows, "statuses": statuses,
        "gait_start_s": start_gait, "expected_sequence": expected_types,
        "observed_sequence": observed, "expected_sequence_ok": expected_sequence_ok,
        "stage_stable": stage_stable, "terminal_hold_ok": terminal_hold_ok,
        "reference_rates": ref_rates, "reference_rate_limits": limits,
        "reference_rate_ok": rate_ok,
        "max_actual_velocity_jump_mps": velocity_jump,
        "max_abs_roll_rad": roll, "max_abs_pitch_rad": pitch,
        "min_contact_count": min(contact_values, default=math.nan),
        "ground_truth_support_rows": ground_truth_rows,
        "min_ground_truth_contact_grf_z_N": min_ground_truth_contact_z,
        "min_positive_ground_truth_contact_grf_z_N": min_positive_ground_truth_contact_z,
        "max_ground_truth_zero_run": max_ground_truth_zero_run,
        "ground_truth_support_ok": (
            ground_truth_rows >= 100
            and math.isfinite(min_positive_ground_truth_contact_z)
            and min_positive_ground_truth_contact_z >= 1.0
            and max_ground_truth_zero_run <= 5
        ),
        "max_wbc_full_eq_residual": solver_residual,
        "finite_required_columns": finite_required,
        "first_event_rows": len(first_rows), "second_event_rows": len(second_rows),
        "first_target_ok": first_target_ok, "second_target_ok": second_target_ok,
        "first_response_ok": first_response_ok, "second_response_ok": second_response_ok,
        "status_ok": status_ok, "csv_ok": csv_ok, "posture_ok": posture_ok,
        "dynamic_ok": dynamic_ok,
        "strict_pass": bool(
            status_ok and csv_ok and expected_sequence_ok and stage_stable
            and terminal_hold_ok and finite_required and first_target_ok
            and second_target_ok and first_response_ok and second_response_ok
            and posture_ok and dynamic_ok
        ),
    })
    return result


def write_report(root: Path, protocol: dict[str, object], results: list[dict[str, object]]) -> None:
    report_dir = root / "reports"
    report_dir.mkdir(parents=True, exist_ok=True)
    passed = sum(bool(item.get("strict_pass")) for item in results)
    fingerprints = {str(item.get("controller_config_fingerprint")) for item in results}
    summary = {
        "generated_at": now(), "protocol": protocol,
        "expected_pairs": len(protocol.get("pair_specs", [])),
        "analyzed_pairs": len(results), "passed_pairs": passed,
        "failed_pairs": len(results) - passed,
        "coverage_fraction": passed / len(protocol.get("pair_specs", []))
        if protocol.get("pair_specs") else 0.0,
        "unique_controller_config_fingerprints": sorted(fingerprints),
        "pair_specific_tuning_detected": len(fingerprints) > 1,
        "overall_strict_pass": bool(
            results and len(results) == len(protocol.get("selected_pairs", []))
            and passed == len(results)
        ),
        "results": results,
    }
    (report_dir / "transition_matrix_metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    fields = [
        "pair_index", "run_id", "source_event", "target_event", "strict_pass",
        "expected_sequence_ok", "stage_stable", "reference_rate_ok",
        "first_target_ok", "second_target_ok", "first_response_ok",
        "second_response_ok", "posture_ok", "dynamic_ok", "max_abs_roll_rad",
        "max_abs_pitch_rad", "max_actual_velocity_jump_mps",
        "max_wbc_full_eq_residual", "duplicate_time_rows",
    ]
    with (report_dir / "transition_matrix_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n"
        )
        writer.writeheader()
        for item in results:
            writer.writerow(item)
    lines = [
        "# Reactive transition matrix — validation report", "",
        f"Generated: {summary['generated_at']}", "", "## Overall assessment", "",
        ("**Ready to share** — all analyzed directed transitions passed the same-controller "
         "acceptance gates." if summary["overall_strict_pass"] else
         "**Needs revision** — one or more transitions or quality gates failed."), "",
        f"Coverage: {passed}/{len(results)} analyzed pairs passed; protocol matrix contains "
        f"{len(protocol.get('pair_specs', []))} pairs.",
        f"Unique controller configuration fingerprints: {len(fingerprints)} "
        f"(pair-specific tuning detected: {summary['pair_specific_tuning_detected']}).", "",
        "## Question tested", "",
        "Can one bounded continuous-reference transition layer and one WBC/MPC plant "
        "handle directed event changes without hand-written pairwise action stitching? "
        "Each nonterminal run contains `none -> A -> B -> none`; an emergency target "
        "ends in the absorbing WBC stance hold. Only the two-line event script changes "
        "between runs.", "", "## Protocol", "",
        f"- Events: {', '.join(protocol['events'])}",
        f"- Sources: {', '.join(protocol['source_events'])}",
        f"- Event windows: start={protocol['event_start_s']} s, duration={protocol['event_duration_s']} s, adjacent A→B",
        f"- Controller duration: {protocol['controller_duration_s']} s; post window: {protocol['post_event_window_s']} s",
        f"- Event source: scheduled scripts only (automatic sensor events enabled: {protocol.get('sensor_events_enabled', False)})",
        f"- Infrastructure policy: up to {protocol.get('max_attempts', 1)} attempts with {protocol.get('retry_delay_s', 0)} s cooldown and alternate DDS domains",
        "- Safety policy: `emergency_stop` is absorbing; incoming transitions are tested, outgoing transitions are intentionally not required.",
        "", "## Gates", "",
        "1. CSV completeness and monotonic time; both event windows contain data.",
        "2. Observed event sequence exactly matches `none -> A -> B -> none`.",
        "3. WBC/MPC remains in gait stage 2 during both events; no controller reset.",
        "4. Reference rates stay within shared limits; target jumps are not mistaken for reference discontinuities.",
        "5. Solver/status gates, Ground Truth contact support, velocity jumps, roll and pitch pass; up to 5 consecutive 2 ms contact-unloading samples are tolerated.",
        "6. Event-specific target signs and response checks pass; emergency includes the terminal WBC stance-hold marker.",
        "", "## Pair results", "",
        "| pair | pass | ref-rate | posture | dynamic | sequence |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for item in results:
        pair = f"{item.get('source_event')} → {item.get('target_event')}"
        lines.append(
            f"| {pair} | {'PASS' if item.get('strict_pass') else 'FAIL'} | "
            f"{'PASS' if item.get('reference_rate_ok') else 'FAIL'} | "
            f"{'PASS' if item.get('posture_ok') else 'FAIL'} | "
            f"{'PASS' if item.get('dynamic_ok') else 'FAIL'} | "
            f"{'PASS' if item.get('expected_sequence_ok') else 'FAIL'} |"
        )
    failed = [item for item in results if not item.get("strict_pass")]
    lines.extend(["", "## Failed runs", ""])
    if failed:
        for item in failed:
            lines.append(
                f"- `{item.get('run_id')}`: sequence={item.get('observed_sequence')}; "
                f"target=({item.get('first_target_ok')}, {item.get('second_target_ok')}); "
                f"response=({item.get('first_response_ok')}, {item.get('second_response_ok')}); "
                f"status={item.get('status_ok')}, dynamic={item.get('dynamic_ok')}"
            )
    else:
        lines.append("None.")
    lines.extend([
        "", "## Scope and caveats", "",
        "This matrix demonstrates the shared reference/transition/WBC-MPC path under "
        "scripted events. It does not prove autonomous perception, local obstacle planning, "
        "or every possible physical disturbance. The physical obstacle acceptance run "
        "remains a separate scene-level test.", "",
        "Raw CSV, simulator/controller logs, per-pair event scripts, and manifests are "
        "retained under this experiment directory for reproduction. Duplicate CSV "
        "timestamps are preserved; rate gates use positive time intervals only.", "",
    ])
    (report_dir / "transition_matrix_report.md").write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, nargs="?", default=Path(
        "example/cpp/experiments/go2_reactive_transition_matrix_2026-08-20"))
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    manifest_path = root / "matrix_manifest.json"
    if not manifest_path.exists():
        raise SystemExit(f"missing {manifest_path}")
    protocol = read_json(manifest_path)
    results = [analyze_run(root, path, protocol)
               for path in sorted((root / "manifests").glob("*.json"))]
    write_report(root, protocol, results)
    passed = sum(bool(item.get("strict_pass")) for item in results)
    failed = len(results) - passed
    summary = {
        "root": str(root), "analyzed": len(results), "passed": passed,
        "failed": failed, "report": str(root / "reports/transition_matrix_report.md"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2) if args.json else
          f"transition-matrix: {passed}/{len(results)} PASS; report={summary['report']}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
