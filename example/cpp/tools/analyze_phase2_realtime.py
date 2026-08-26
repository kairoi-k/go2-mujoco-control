#!/usr/bin/env python3
"""Summarize wall-clock performance telemetry for Phase 2 realtime runs."""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def path_for(spec: str, name: str = "data.csv") -> Path:
    path = Path(spec)
    if path.is_dir():
        path /= name
    if not path.is_file():
        raise SystemExit(f"missing {name}: {path}")
    return path


def number(value: str | None) -> float | None:
    try:
        result = float(value or "")
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def values(rows: list[dict[str, str]], field: str) -> list[float]:
    result = [number(row.get(field)) for row in rows]
    return [item for item in result if item is not None]


def percentile(items: list[float], fraction: float) -> float | None:
    if not items:
        return None
    ordered = sorted(items)
    index = fraction * (len(ordered) - 1)
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return ordered[low]
    weight = index - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def scalar_stats(items: list[float]) -> dict[str, float | int | None]:
    if not items:
        return {
            "count": 0,
            "mean": None,
            "min": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    return {
        "count": len(items),
        "mean": statistics.fmean(items),
        "min": min(items),
        "p50": percentile(items, 0.50),
        "p95": percentile(items, 0.95),
        "p99": percentile(items, 0.99),
        "max": max(items),
    }


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit(f"empty CSV: {path}")
    return rows


def trace_summary(run: Path) -> dict[str, Any]:
    trace_path = run / "bridge_trace.csv"
    if not trace_path.is_file():
        return {"present": False}
    rows = read_rows(trace_path)
    ages = values(rows, "command_age_s")
    wall = values(rows, "wall_time_ns")
    intervals = [
        (wall[index] - wall[index - 1]) * 1.0e-9
        for index in range(1, len(wall))
        if wall[index] > wall[index - 1]
    ]
    sequences = values(rows, "command_sequence")
    changes = sum(
        1
        for index in range(1, len(sequences))
        if sequences[index] != sequences[index - 1]
    )
    result: dict[str, Any] = {
        "present": True,
        "row_count": len(rows),
        "command_sequence_changes": changes,
        "command_age_s": scalar_stats(ages),
        "bridge_interval_s": scalar_stats(intervals),
        "bridge_hz": (
            1.0 / statistics.fmean(intervals) if intervals else None
        ),
        "wall_span_s": (
            (wall[-1] - wall[0]) * 1.0e-9
            if len(wall) >= 2 and wall[-1] >= wall[0]
            else None
        ),
        "first_simulation_tick": number(rows[0].get("simulation_tick")),
        "last_simulation_tick": number(rows[-1].get("simulation_tick")),
    }
    return result


def run_summary(spec: str) -> dict[str, Any]:
    data_path = path_for(spec)
    run = data_path.parent
    rows = read_rows(data_path)
    dt = values(rows, "motion_clock_wall_dt_s")
    if not dt:
        jitter = values(rows, "controller_wall_jitter_s")
        dt = [0.002 + item for item in jitter if 0.002 + item > 0.0]
    jitter_abs = [abs(item - 0.002) for item in dt]
    state_age = values(rows, "controller_input_low_state_age_s")
    high_age = values(rows, "controller_input_high_state_age_s")
    sim_lag = []
    for row in rows:
        physics = number(row.get("physics_sequence"))
        input_tick = number(row.get("controller_input_sim_tick"))
        if physics is not None and input_tick is not None:
            sim_lag.append(physics - input_tick)
    lidar_age = values(rows, "terrain_lidar_wall_age_s")
    lidar_sequence = values(rows, "terrain_lidar_rx_sequence")
    return {
        "spec": spec,
        "csv": str(data_path.resolve()),
        "row_count": len(rows),
        "controller_dt_s": scalar_stats(dt),
        "controller_hz": 1.0 / statistics.fmean(dt) if dt else None,
        "controller_jitter_abs_s": scalar_stats(jitter_abs),
        "state_age_s": scalar_stats(state_age),
        "high_state_age_s": scalar_stats(high_age),
        "controller_input_sim_tick_lag": scalar_stats(sim_lag),
        "lidar_wall_age_s": scalar_stats(lidar_age),
        "lidar_sequence_last": lidar_sequence[-1] if lidar_sequence else None,
        "lidar_updates": len(set(lidar_sequence)),
        "bridge": trace_summary(run),
    }


def metric_values(runs: list[dict[str, Any]], key: str) -> list[float]:
    result = []
    for run in runs:
        value = run.get(key)
        if isinstance(value, (int, float)) and math.isfinite(value):
            result.append(float(value))
    return result


def aggregate(runs: list[dict[str, Any]]) -> dict[str, Any]:
    keys = (
        "controller_hz",
        "lidar_sequence_last",
        "lidar_updates",
    )
    result: dict[str, Any] = {}
    for key in keys:
        items = metric_values(runs, key)
        result[key] = scalar_stats(items)
    bridge_hz = [
        run["bridge"]["bridge_hz"]
        for run in runs
        if run["bridge"].get("bridge_hz") is not None
    ]
    bridge_age = [
        run["bridge"]["command_age_s"]["max"]
        for run in runs
        if run["bridge"].get("present")
        and run["bridge"]["command_age_s"]["max"] is not None
    ]
    result["bridge_hz"] = scalar_stats(bridge_hz)
    result["bridge_command_age_max_s"] = scalar_stats(bridge_age)
    return result


def report(result: dict[str, Any]) -> str:
    lines = [
        "Phase 2 realtime performance summary",
        f"runs={len(result['runs'])}",
    ]
    for run in result["runs"]:
        bridge = run["bridge"]
        lines.append(
            "run={} samples={} controller_hz={} dt_p95={} "
            "state_age_max={} lidar_age_max={} bridge_hz={} "
            "bridge_command_age_max={}".format(
                run["spec"],
                run["row_count"],
                run["controller_hz"],
                run["controller_dt_s"]["p95"],
                run["state_age_s"]["max"],
                run["lidar_wall_age_s"]["max"],
                bridge.get("bridge_hz"),
                bridge.get("command_age_s", {}).get("max"),
            )
        )
    lines.append("aggregate=" + json.dumps(result["aggregate"], sort_keys=True))
    return "\n".join(lines) + "\n"


def main() -> None:
    options = args()
    runs = [run_summary(spec) for spec in options.run]
    result = {
        "format": "phase2-realtime-performance-v1",
        "runs": runs,
        "aggregate": aggregate(runs),
        "deadline_reference_s": 0.002,
        "deadline_miss_policy": "diagnostic only; no acceptance gate",
    }
    output = Path(options.output)
    if output.suffix.lower() != ".json":
        output = output.with_suffix(".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    text_path = output.with_suffix(".txt")
    text_path.write_text(report(result), encoding="utf-8")
    print(report(result), end="")
    print(f"json={output}")
    print(f"report={text_path}")


if __name__ == "__main__":
    main()
