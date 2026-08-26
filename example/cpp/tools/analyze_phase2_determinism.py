#!/usr/bin/env python3
"""Compare Phase 2 runs by simulation tick and locate the first divergence."""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare phase2 CSV runs by simulation tick."
    )
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        help="run directory or data.csv; repeat for each run",
    )
    parser.add_argument(
        "--output",
        default="phase2_determinism_analysis.json",
        help="JSON output path; a sibling .txt report is also written",
    )
    parser.add_argument("--abs-tolerance", type=float, default=1.0e-12)
    parser.add_argument("--rel-tolerance", type=float, default=1.0e-12)
    parser.add_argument(
        "--exclude",
        action="append",
        default=[],
        help="numeric field to exclude; repeat as needed",
    )
    return parser.parse_args()


def resolve_csv(spec: str) -> Path:
    path = Path(spec)
    if path.is_dir():
        path /= "data.csv"
    if not path.is_file():
        raise SystemExit(f"missing run CSV: {path}")
    return path


def read_metadata(run_dir: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for name in ("run_metadata.txt", "run_manifest.txt"):
        path = run_dir / name
        if not path.is_file():
            continue
        for line in path.read_text(
            encoding="utf-8", errors="replace"
        ).splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                result[key.strip()] = value.strip()
    return result


def numeric_value(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def tick_value(row: dict[str, str], row_number: int) -> int | float:
    for key in ("controller_tick", "simulation_tick", "physics_sequence", "tick"):
        if key not in row:
            continue
        try:
            number = float(row[key])
        except ValueError:
            continue
        if math.isfinite(number):
            return int(number) if number.is_integer() else number
    return row_number


def load_run(spec: str) -> dict[str, Any]:
    csv_path = resolve_csv(spec)
    rows: dict[int | float, dict[str, float | None]] = {}
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise SystemExit(f"CSV has no header: {csv_path}")
        fields = list(reader.fieldnames)
        for row_number, raw in enumerate(reader):
            key = tick_value(raw, row_number)
            if key in rows:
                raise SystemExit(f"duplicate simulation tick {key} in {csv_path}")
            rows[key] = {
                field: numeric_value(raw.get(field)) for field in fields
            }
    if not rows:
        raise SystemExit(f"CSV has no data rows: {csv_path}")
    return {
        "spec": spec,
        "csv": str(csv_path.resolve()),
        "run_dir": str(csv_path.parent.resolve()),
        "metadata": read_metadata(csv_path.parent),
        "fields": fields,
        "rows": rows,
    }


def sort_ticks(values: set[int | float]) -> list[int | float]:
    return sorted(values, key=float)


def common_ticks(runs: list[dict[str, Any]]) -> list[int | float]:
    keys = set(runs[0]["rows"])
    for run in runs[1:]:
        keys.intersection_update(run["rows"])
    return sort_ticks(keys)


def common_numeric_fields(
    runs: list[dict[str, Any]], excluded: set[str]
) -> list[str]:
    result = []
    for field in runs[0]["fields"]:
        if field in excluded:
            continue
        if all(
            field in run["fields"]
            and any(row[field] is not None for row in run["rows"].values())
            for run in runs
        ):
            result.append(field)
    return result


def field_class(field: str) -> str:
    scheduling_prefixes = (
        "controller_",
        "physics_",
        "published_",
        "state_",
        "motion_",
        "terrain_",
        "queue_",
    )
    if field.startswith(scheduling_prefixes) or field.endswith("_age_s"):
        return "scheduling"
    if "deadline" in field or "elapsed_us" in field:
        return "scheduling"
    return "trajectory"


def allowed_difference(
    a: float, b: float, abs_tolerance: float, rel_tolerance: float
) -> float:
    return abs_tolerance + rel_tolerance * max(abs(a), abs(b))


def compare_to_reference(
    reference: dict[str, Any],
    candidate: dict[str, Any],
    ticks: list[int | float],
    fields: list[str],
    abs_tolerance: float,
    rel_tolerance: float,
) -> dict[str, Any]:
    max_abs = {field: 0.0 for field in fields}
    first: dict[str, Any] | None = None
    first_by_class: dict[str, dict[str, Any] | None] = {"trajectory": None, "scheduling": None}
    compared = 0
    missing_values = 0
    for tick in ticks:
        ref_row = reference["rows"][tick]
        candidate_row = candidate["rows"][tick]
        for field in fields:
            a = ref_row[field]
            b = candidate_row[field]
            if a is None or b is None:
                missing_values += 1
                continue
            compared += 1
            difference = abs(a - b)
            max_abs[field] = max(max_abs[field], difference)
            limit = allowed_difference(
                a, b, abs_tolerance, rel_tolerance
            )
            if difference > limit:
                event = {
                    "alignment_key": tick,
                    "controller_tick": ref_row.get("controller_tick"),
                    "simulation_tick": ref_row.get("simulation_tick"),
                    "physics_sequence": ref_row.get("physics_sequence"),
                    "controller_input_sim_tick": ref_row.get("controller_input_sim_tick"),
                    "simulation_time_s": ref_row.get("simulation_time_s"),
                    "field": field,
                    "reference": a,
                    "candidate": b,
                    "absolute_difference": difference,
                    "allowed_difference": limit,
                }
                if first is None:
                    first = event
                category = field_class(field)
                if first_by_class[category] is None:
                    first_by_class[category] = event
    ranked = sorted(max_abs.items(), key=lambda item: item[1], reverse=True)
    return {
        "candidate": candidate["spec"],
        "common_tick_count": len(ticks),
        "compared_values": compared,
        "missing_values": missing_values,
        "first_divergence": first,
        "first_trajectory_divergence": first_by_class["trajectory"],
        "first_scheduling_divergence": first_by_class["scheduling"],
        "max_absolute_difference": dict(ranked[:20]),
        "within_tolerance": first is None and missing_values == 0,
    }

def variance_summary(
    runs: list[dict[str, Any]],
    ticks: list[int | float],
    fields: list[str],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    if not ticks:
        return result
    last_tick = ticks[-1]
    for field in fields:
        variances: list[float] = []
        final_values: list[float] = []
        for tick in ticks:
            values = [
                run["rows"][tick][field]
                for run in runs
                if run["rows"][tick][field] is not None
            ]
            if not values:
                continue
            mean = statistics.fmean(values)
            variances.append(
                statistics.fmean((value - mean) ** 2 for value in values)
            )
            if tick == last_tick:
                final_values = values
        if not variances:
            continue
        result[field] = {
            "max_population_variance": max(variances),
            "mean_population_variance": statistics.fmean(variances),
            "final_population_variance": (
                variances[-1] if final_values else None
            ),
            "final_min": min(final_values) if final_values else None,
            "final_max": max(final_values) if final_values else None,
            "final_mean": (
                statistics.fmean(final_values) if final_values else None
            ),
        }
    return result


def selected_final_values(
    runs: list[dict[str, Any]], tick: int | float, fields: list[str]
) -> dict[str, dict[str, float | None]]:
    selected = [
        field
        for field in fields
        if field.startswith(("qpos", "qvel", "joint_tau", "joint_target"))
        or field
        in {
            "base_x_m",
            "base_y_m",
            "base_z_m",
            "base_roll_rad",
            "base_pitch_rad",
            "base_yaw_rad",
            "base_vx_mps",
            "base_vy_mps",
            "base_vz_mps",
            "body_vx_mps",
            "filtered_body_vx_mps",
            "requested_v_cmd_mps",
            "shaped_v_cmd_mps",
            "applied_v_cmd_mps",
            "gait_phase",
            "planned_contact_mask",
            "measured_contact_mask",
            "srbd_first_force_x_n",
            "srbd_first_force_z_n",
            "id_force_x_n",
            "torque_max_abs_nm",
            "torque_saturation_count",
            "terrain_map_epoch",
            "terrain_lidar_sequence",
            "terrain_plan_epoch",
        }
    ]
    return {
        field: {
            str(run["spec"]): run["rows"][tick].get(field)
            for run in runs
        }
        for field in selected
    }


def make_report(result: dict[str, Any]) -> str:
    lines = [
        "Phase 2 determinism comparison",
        f"runs={result['run_count']} common_ticks={result['common_tick_count']}",
        f"tolerance_abs={result['abs_tolerance']} tolerance_rel={result['rel_tolerance']}",
        f"deterministic_within_tolerance={result['deterministic_within_tolerance']}",
    ]
    for item in result["comparisons"]:
        first = item["first_divergence"]
        if first is None:
            lines.append(f"run={item['candidate']} first_divergence=none")
        else:
            simulation_tick = first.get("simulation_tick")
            if simulation_tick is None:
                simulation_tick = first.get("controller_input_sim_tick")
            lines.append(
                "run={} first_divergence=alignment_key {} controller_tick={} simulation_tick={} field={} abs_diff={:.17g}".format(
                    item["candidate"],
                    first["alignment_key"],
                    first.get("controller_tick"),
                    simulation_tick,
                    first["field"],
                    first["absolute_difference"],
                )
            )
        for label in ("first_trajectory_divergence", "first_scheduling_divergence"):
            event = item[label]
            if event is None:
                lines.append(f"  {label}=none")
            else:
                lines.append(
                    f"  {label}=tick {event['simulation_tick']} "
                    f"field={event['field']} abs_diff={event['absolute_difference']:.17g}"
                )
        top = list(item["max_absolute_difference"].items())[:5]
        lines.append(
            "  max_abs=" + ", ".join(
                f"{name}:{value:.6g}" for name, value in top
            )
        )
    variance_items = sorted(
        result["variance"].items(),
        key=lambda item: item[1]["max_population_variance"],
        reverse=True,
    )
    lines.append("largest_population_variance:")
    for field, summary in variance_items[:10]:
        final = summary["final_population_variance"]
        lines.append(
            "  {} max={:.6g} final={}".format(
                field,
                summary["max_population_variance"],
                f"{final:.6g}" if final is not None else "n/a",
            )
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    if args.abs_tolerance < 0.0 or args.rel_tolerance < 0.0:
        raise SystemExit("tolerances must be non-negative")
    runs = [load_run(spec) for spec in args.run]
    ticks = common_ticks(runs)
    if not ticks:
        raise SystemExit("runs have no common simulation ticks")
    fields = common_numeric_fields(runs, set(args.exclude))
    if not fields:
        raise SystemExit("runs have no common numeric fields")
    reference = runs[0]
    comparisons = [
        compare_to_reference(
            reference,
            run,
            ticks,
            fields,
            args.abs_tolerance,
            args.rel_tolerance,
        )
        for run in runs[1:]
    ]
    result: dict[str, Any] = {
        "format": "phase2-determinism-analysis-v1",
        "run_count": len(runs),
        "runs": [
            {
                "spec": run["spec"],
                "csv": run["csv"],
                "row_count": len(run["rows"]),
                "metadata": run["metadata"],
            }
            for run in runs
        ],
        "key": "controller_tick|simulation_tick|physics_sequence|tick|row_index",
        "common_tick_count": len(ticks),
        "first_common_tick": ticks[0],
        "last_common_tick": ticks[-1],
        "numeric_field_count": len(fields),
        "numeric_fields": fields,
        "excluded_fields": sorted(set(args.exclude)),
        "abs_tolerance": args.abs_tolerance,
        "rel_tolerance": args.rel_tolerance,
        "comparisons": comparisons,
        "deterministic_within_tolerance": all(
            item["within_tolerance"] for item in comparisons
        ),
        "variance": variance_summary(runs, ticks, fields),
        "final_values": selected_final_values(runs, ticks[-1], fields),
    }
    output = Path(args.output)
    if output.suffix.lower() != ".json":
        output = output.with_suffix(".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    report_path = output.with_suffix(".txt")
    report_path.write_text(make_report(result), encoding="utf-8")
    print(make_report(result), end="")
    print(f"json={output}")
    print(f"report={report_path}")


if __name__ == "__main__":
    main()
