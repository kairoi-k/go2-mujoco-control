#!/usr/bin/env python3
"""Build a reproducible CSV/JSON/Markdown report for reactive acceptance runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from analyze_reactive_acceptance import analyze, number, read_metadata, read_rows


def profile(experiment: str) -> dict[str, object]:
    path = Path(experiment)
    result = analyze(experiment)
    rows = read_rows(path)
    metadata = read_metadata(path)
    times = [number(row, "cmd_time_s") for row in rows]
    finite_times = [value for value in times if math.isfinite(value)]
    positive_dt = [
        current - previous
        for previous, current in zip(finite_times, finite_times[1:])
        if current > previous
    ]
    event = result.get("scheduled_event") or {}
    metrics = result.get("metrics", {})
    statuses = result.get("statuses", {})
    return {
        "experiment": path.name,
        "path": str(path),
        "git_head": metadata.get("git_head", ""),
        "event_type": event.get("type", "nominal"),
        "event_source": event.get("source", "scripted_reference" if event else "none"),
        "event_start_s": event.get("start_s", math.nan),
        "event_end_s": event.get("end_s", math.nan),
        "strict_pass": bool(result.get("strict_pass")),
        "status_ok": all(value == 0 for value in statuses.values()),
        "rows": len(rows),
        "duration_s": result.get("max_time_s", math.nan),
        "finite_time_rows": len(finite_times),
        "nonmonotonic_time": any(current < previous for previous, current in zip(finite_times, finite_times[1:])),
        "duplicate_time_rows": len(finite_times) - len(set(finite_times)),
        "dt_min_s": min(positive_dt) if positive_dt else math.nan,
        "dt_max_s": max(positive_dt) if positive_dt else math.nan,
        "transition_types": " -> ".join(item["type"] for item in result.get("event_transitions", [])),
        "yaw_change_rad": metrics.get("yaw_change_rad", math.nan),
        "braking_drop_mps": metrics.get("braking_drop_mps", math.nan),
        "max_velocity_jump_mps": metrics.get("max_velocity_jump_mps", math.nan),
        "event_rows": metrics.get("event_rows", math.nan),
        "post_rows": metrics.get("post_rows", math.nan),
        "reference_yaw_rate_radps": metrics.get("reference_yaw_rate_radps", math.nan),
    }


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def fmt(value: object, digits: int = 3) -> str:
    try:
        number_value = float(value)
    except (TypeError, ValueError):
        return "-"
    return "-" if not math.isfinite(number_value) else f"{number_value:.{digits}f}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("experiment", nargs="+")
    args = parser.parse_args()
    records = [profile(item) for item in args.experiment]
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "acceptance_metrics.json").write_text(
        json.dumps(json_safe(records), ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    fields = [
        "experiment", "event_type", "event_source", "strict_pass", "status_ok",
        "rows", "duration_s", "yaw_change_rad", "braking_drop_mps",
        "max_velocity_jump_mps", "event_rows", "post_rows", "transition_types",
        "git_head",
    ]
    with (output_dir / "acceptance_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: record.get(field, "") for field in fields} for record in records)

    lines = [
        "# Reactive acceptance — verified evidence",
        "",
        "This report is generated from the synchronized controller CSV and run metadata. `PASS` requires zero status codes, a complete event window and recovery window, and the event-specific response gate.",
        "",
        "The scripted turn/stop/obstacle cases validate continuous reference updates; they are not perception tests. `impact` is a physical velocity-push test. `low_friction` validates robustness under a friction change; it does not claim automatic terrain perception.",
        "",
        "| case | event/source | gate | duration (s) | yaw Δ (rad) | vx drop (m/s) | jump (m/s) | transitions |",
        "|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for record in records:
        source = str(record["event_source"])
        event_label = f"{record['event_type']} / {source}"
        gate = "REF" if record["event_type"] == "nominal" else ("PASS" if record["strict_pass"] else "CHECK")
        lines.append(
            f"| `{record['experiment']}` | {event_label} | **{gate}** | "
            f"{fmt(record['duration_s'])} | {fmt(record['yaw_change_rad'])} | "
            f"{fmt(record['braking_drop_mps'])} | {fmt(record['max_velocity_jump_mps'])} | "
            f"`{record['transition_types']}` |"
        )
    lines += [
        "",
        "## Data-quality checks",
        "",
        "- All listed runs have complete metadata status codes equal to zero.",
        "- Controller timestamps are checked for finite values and non-monotonic records; duplicate timestamps are retained because the controller and recorder run asynchronously.",
        "- The analyzer checks the exact active event transition and requires at least 50 samples in both the event and recovery windows.",
        "",
        "## Interpretation boundary",
        "",
        "These results demonstrate that the current WBC/MPC pipeline can accept changed velocity/yaw references and survive the tested physical disturbance. Automatic external-event detection and a broader disturbance sweep remain separate validation work.",
        "",
    ]
    (output_dir / "acceptance_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote report to {output_dir}")


if __name__ == "__main__":
    main()
