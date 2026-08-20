#!/usr/bin/env python3
"""Summarize long-window transition runs into a compact, auditable table."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def number(row: dict[str, str], key: str) -> float:
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value


def mean(rows: list[dict[str, str]], key: str) -> float:
    values = [number(row, key) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    return sum(values) / len(values) if values else math.nan


def maximum_abs(rows: list[dict[str, str]], key: str) -> float:
    values = [abs(number(row, key)) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    return max(values) if values else math.nan


def minimum(rows: list[dict[str, str]], key: str) -> float:
    values = [number(row, key) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    return min(values) if values else math.nan


def segment_metrics(rows: list[dict[str, str]], start: float, end: float, suffix: str) -> dict[str, object]:
    segment = [row for row in rows if start <= number(row, "cmd_time_s") <= end]
    ok = [row for row in segment if number(row, "wbc_full_srbd_ok") >= 1 and number(row, "wbc_full_id_ok") >= 1]
    result: dict[str, object] = {
        f"{suffix}_rows": len(segment),
        f"{suffix}_event_active_ratio": round(sum(number(row, "event_active") >= 1 for row in segment) / len(segment), 4) if segment else math.nan,
        f"{suffix}_wbc_ok_ratio": round(len(ok) / len(segment), 4) if segment else math.nan,
        f"{suffix}_ref_vx_mean_mps": round(mean(segment, "event_ref_vx_mps"), 5),
        f"{suffix}_ref_vy_mean_mps": round(mean(segment, "event_ref_vy_mps"), 5),
        f"{suffix}_ref_yaw_mean_radps": round(mean(segment, "event_ref_yaw_rate_radps"), 5),
        f"{suffix}_world_vx_mean_mps": round(mean(segment, "world_velocity_x_mps"), 5),
        f"{suffix}_world_vy_mean_mps": round(mean(segment, "world_velocity_y_mps"), 5),
        f"{suffix}_gyro_z_mean_radps": round(mean(segment, "imu_gyro_z_radps"), 5),
        f"{suffix}_max_abs_roll_deg": round(math.degrees(maximum_abs(segment, "imu_roll_rad")), 3),
        f"{suffix}_max_abs_pitch_deg": round(math.degrees(maximum_abs(segment, "imu_pitch_rad")), 3),
        f"{suffix}_min_base_z_m": round(minimum(segment, "world_base_z_m"), 5),
        f"{suffix}_max_eq_residual": round(maximum_abs(segment, "wbc_full_eq_residual"), 6),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--clip-start", type=float, default=3.5)
    args = parser.parse_args()

    records = json.loads(args.manifest.read_text(encoding="utf-8"))
    rows_out: list[dict[str, object]] = []
    for item in records:
        result: dict[str, object] = {
            "pair_index": item.get("pair_index"),
            "run_id": item.get("run_id"),
            "source_event": item.get("source_event"),
            "target_event": item.get("target_event"),
            "status": item.get("status"),
            "duration_s": item.get("duration_s"),
        }
        spans = item.get("event_spans", [])
        if len(spans) == 2:
            result.update({"source_video_start_s": spans[0]["start_s"], "source_video_end_s": spans[0]["end_s"], "target_video_start_s": spans[1]["start_s"], "target_video_end_s": spans[1]["end_s"]})
        run_dir = Path(str(item.get("run_directory", "")))
        data_path = run_dir / "data.csv"
        if data_path.exists() and len(spans) == 2:
            with data_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            source_start, source_end = float(spans[0]["start_s"]) + args.clip_start, float(spans[0]["end_s"]) + args.clip_start
            target_start, target_end = float(spans[1]["start_s"]) + args.clip_start, float(spans[1]["end_s"]) + args.clip_start
            result.update(segment_metrics(rows, source_start, source_end, "source"))
            result.update(segment_metrics(rows, target_start, target_end, "target"))
        rows_out.append(result)

    fieldnames = list(rows_out[0]) if rows_out else []
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows_out)

    durations = [float(row["duration_s"]) for row in rows_out if row.get("duration_s") is not None]
    source_ok = [float(row["source_wbc_ok_ratio"]) for row in rows_out if isinstance(row.get("source_wbc_ok_ratio"), (float, int)) and math.isfinite(float(row["source_wbc_ok_ratio"]))]
    target_ok = [float(row["target_wbc_ok_ratio"]) for row in rows_out if isinstance(row.get("target_wbc_ok_ratio"), (float, int)) and math.isfinite(float(row["target_wbc_ok_ratio"]))]
    report = [
        "# Long-window transition matrix QA",
        "",
        f"- Records: {len(rows_out)}; pass: {sum(row.get('status') == 'pass' for row in rows_out)}.",
        f"- Clip duration: {min(durations):.3f}–{max(durations):.3f} s." if durations else "- Clip duration: unavailable.",
        f"- WBC full OK ratio, source windows: {min(source_ok):.4f}–{max(source_ok):.4f}." if source_ok else "- WBC source-window ratio: unavailable.",
        f"- WBC full OK ratio, target windows: {min(target_ok):.4f}–{max(target_ok):.4f}." if target_ok else "- WBC target-window ratio: unavailable.",
        "- Each row is a nominal-floor reference handoff; physical obstacle, impact, and low-friction claims are kept in the representative acceptance package.",
        "- `transition_metrics.csv` reports reference/feedback means, orientation bounds, base height, WBC validity, and event-active coverage for both windows.",
    ]
    args.output_md.write_text("\n".join(report) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
