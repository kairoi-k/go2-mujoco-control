#!/usr/bin/env python3
"""Audit Phase-1 transition settling with complete observed tail coverage.

This is an independent, offline diagnostic. It does not change the frozen
analyzer, its thresholds, or any acceptance status. A transition is observed
only when a candidate has a finite measured-velocity tail covering the full
one-second window used by the legacy metric. The first sample at or after the
window end is retained as a bracketing sample so irregular wall-clock samples
do not look short by one period. Missing coverage, sampling gaps, non-finite
samples, and late settling are reported explicitly and fail closed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

TAIL_WINDOW_S = 1.0
TIME_EPS_S = 1.0e-6
# Diagnostic data-quality guard; it does not alter the frozen acceptance gate.
MAX_SAMPLE_GAP_S = 0.050
SETTLING_LIMITS_S = {
    "steps": 8.2,
    "accel_1_to_3": 10.0,
    "brake_3_to_0": 1.5,
    "ramp": 2.0,
    "varying": 8.2,
}


def settling_limit_for_scenario(scenario: str) -> float:
    try:
        return SETTLING_LIMITS_S[scenario]
    except KeyError as exc:
        raise ValueError(f"unknown quantitative scenario: {scenario}") from exc

def _value(row: Mapping[str, Any], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def _finite_or_none(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return value


def read_profile(path: Path) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = [field.strip() for field in line.replace(",", " ").split()]
        if len(fields) < 2:
            raise ValueError(f"invalid profile line: {raw!r}")
        point = (float(fields[0]), float(fields[1]))
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"non-finite profile line: {raw!r}")
        points.append(point)
    if not points:
        raise ValueError("empty velocity profile")
    if any(right[0] <= left[0] for left, right in zip(points, points[1:])):
        raise ValueError("profile times must be strictly increasing")
    return points


def _active_rows(rows: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for row in rows:
        if row.get("velocity_command_gait_regime") != "continuous-trot":
            continue
        active = _value(row, "velocity_command_active")
        # Retain a non-finite active flag for the input-validation failure
        # instead of silently treating it as inactive.
        if (math.isfinite(active) and active > 0.5) or not math.isfinite(active):
            result.append(dict(row))
    return result


def load_active_rows(csv_path: Path) -> list[dict[str, Any]]:
    with csv_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"cmd_time_s", "velocity_command_measured_mps",
                    "velocity_command_active", "velocity_command_gait_regime"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("missing diagnostic columns: " + ", ".join(sorted(missing)))
        return _active_rows(reader)


def _candidate_summary(
    candidate_time: float,
    transition_start: float,
    span_s: float,
    tail_rows: int,
    max_error: float | None,
    max_gap_s: float | None,
) -> dict[str, float | int | None]:
    return {
        "candidate_time_s": candidate_time,
        "settling_time_s": candidate_time - transition_start,
        "tail_observed_span_s": _finite_or_none(span_s),
        "tail_rows": tail_rows,
        "tail_max_abs_error_mps": _finite_or_none(max_error),
        "tail_max_sample_gap_s": _finite_or_none(max_gap_s),
    }


def _max_gap(rows: Sequence[tuple[float, float]]) -> float:
    if len(rows) < 2:
        return 0.0
    return max(right[0] - left[0] for left, right in zip(rows, rows[1:]))


def audit_transitions(
    rows: Sequence[Mapping[str, Any]],
    profile: Sequence[tuple[float, float]],
    active_start: float | None = None,
    *,
    tail_window_s: float = TAIL_WINDOW_S,
    time_epsilon_s: float = TIME_EPS_S,
    settling_limit_s: float | None = None,
    max_sample_gap_s: float = MAX_SAMPLE_GAP_S,
) -> dict[str, Any]:
    """Return per-transition settling proof and an all-transitions verdict.

    Transition windows and tolerance are intentionally the same as legacy
    ``transition_metrics``. The diagnostic adds finite input validation, a
    full observed tail, a sample-gap guard, and an optional per-transition
    settling deadline copied from the frozen quantitative limits.
    """
    if not rows:
        raise ValueError("no evaluation rows")
    profile = list(profile)
    if len(profile) < 2:
        raise ValueError("profile must contain at least two points")
    if any(
        not (math.isfinite(time_s) and math.isfinite(target))
        for time_s, target in profile
    ):
        raise ValueError("profile times and targets must be finite")
    if any(right[0] <= left[0] for left, right in zip(profile, profile[1:])):
        raise ValueError("profile times must be strictly increasing")
    if not math.isfinite(tail_window_s) or tail_window_s <= 0.0:
        raise ValueError("tail_window_s must be positive and finite")
    if not math.isfinite(max_sample_gap_s) or max_sample_gap_s <= 0.0:
        raise ValueError("max_sample_gap_s must be positive and finite")

    timed_rows = [
        (_value(row, "cmd_time_s"), _value(row, "velocity_command_measured_mps"))
        for row in rows
    ]
    invalid_time_rows = sum(not math.isfinite(time_s) for time_s, _ in timed_rows)
    invalid_measured_rows = sum(
        not math.isfinite(measured) for _, measured in timed_rows
    )
    invalid_active_rows = sum(
        "velocity_command_active" in row
        and not math.isfinite(_value(row, "velocity_command_active"))
        for row in rows
    )
    finite_times = [time_s for time_s, _ in timed_rows if math.isfinite(time_s)]
    if active_start is None:
        if not finite_times:
            raise ValueError("evaluation rows have no finite command time")
        active_start = finite_times[0]
    if not math.isfinite(active_start):
        raise ValueError("active_start must be finite")
    end_time = finite_times[-1] - active_start if finite_times else float("nan")
    time_order_errors = sum(
        right < left
        for left, right in zip(finite_times, finite_times[1:])
    )
    # Preserve legacy chronological semantics while making any source-order
    # violation visible in input_validation below.
    relative_rows = sorted(
        [
            (time_s - active_start, measured)
            for time_s, measured in timed_rows
            if math.isfinite(time_s)
        ],
        key=lambda item: item[0],
    )

    transitions: list[dict[str, Any]] = []
    for index in range(1, len(profile)):
        transition_start, target = profile[index]
        previous = profile[index - 1][1]
        transition_end = (
            profile[index + 1][0] if index + 1 < len(profile) else end_time
        )
        if abs(target - previous) < 1.0e-9 or transition_end - transition_start < 1.0:
            continue

        window = [
            (time_s, measured)
            for time_s, measured in relative_rows
            if transition_start <= time_s <= transition_end
        ]
        tolerance = max(0.15, 0.05 * max(abs(target), 1.0))
        legacy_candidate: dict[str, Any] | None = None
        observed_candidate: dict[str, Any] | None = None
        complete_finite_candidates = 0
        nonfinite_full_candidates = 0
        sampling_gap_candidates = 0
        partial_candidates = 0
        best_complete_error: float | None = None
        best_complete_time: float | None = None

        for candidate_time, _ in window:
            endpoint = candidate_time + tail_window_s
            legacy_tail = [
                (time_s, measured)
                for time_s, measured in window
                if candidate_time <= time_s <= endpoint
            ]
            if not legacy_tail:
                continue
            legacy_finite_tail = [
                (time_s, measured)
                for time_s, measured in legacy_tail
                if math.isfinite(measured)
            ]
            legacy_max_error = (
                max(
                    abs(measured - target)
                    for _, measured in legacy_finite_tail
                )
                if legacy_finite_tail
                else float("nan")
            )
            legacy_gap = _max_gap(legacy_tail)
            legacy_summary = _candidate_summary(
                candidate_time,
                transition_start,
                legacy_tail[-1][0] - candidate_time,
                len(legacy_tail),
                legacy_max_error,
                legacy_gap,
            )
            # This finite-only equivalent is intentionally reported only for
            # comparison with the legacy finite aggregation; NaN never proves.
            if legacy_candidate is None and (
                len(legacy_finite_tail) == len(legacy_tail)
                and legacy_max_error <= tolerance
            ):
                legacy_candidate = legacy_summary

            bracket = next(
                (sample for sample in window if sample[0] >= endpoint),
                None,
            )
            if bracket is None:
                partial_candidates += 1
                continue
            proof_tail = [
                (time_s, measured)
                for time_s, measured in window
                if candidate_time <= time_s <= endpoint
            ]
            if not proof_tail or proof_tail[-1][0] < bracket[0]:
                proof_tail.append(bracket)
            span_s = bracket[0] - candidate_time
            has_full_coverage = span_s >= tail_window_s - time_epsilon_s
            finite_proof_tail = [
                (time_s, measured)
                for time_s, measured in proof_tail
                if math.isfinite(measured)
            ]
            all_finite = len(finite_proof_tail) == len(proof_tail)
            max_error = (
                max(abs(measured - target) for _, measured in finite_proof_tail)
                if finite_proof_tail
                else float("nan")
            )
            gap_s = _max_gap(proof_tail)
            candidate = _candidate_summary(
                candidate_time,
                transition_start,
                span_s,
                len(proof_tail),
                max_error,
                gap_s,
            )
            if not has_full_coverage:
                partial_candidates += 1
                continue
            if not all_finite:
                nonfinite_full_candidates += 1
                continue
            complete_finite_candidates += 1
            if best_complete_error is None or max_error < best_complete_error:
                best_complete_error = max_error
                best_complete_time = candidate_time
            if gap_s > max_sample_gap_s:
                sampling_gap_candidates += 1
                continue
            if observed_candidate is None and max_error <= tolerance:
                observed_candidate = candidate

        if observed_candidate is not None:
            status = "observed"
            if (
                settling_limit_s is not None
                and observed_candidate["settling_time_s"]
                > settling_limit_s + time_epsilon_s
            ):
                status = "deadline_missed"
        elif nonfinite_full_candidates:
            status = "nonfinite_missing"
        elif sampling_gap_candidates:
            status = "sampling_gap"
        elif complete_finite_candidates:
            status = "unsettled"
        else:
            status = "coverage_missing"

        transitions.append(
            {
                "transition": f"{previous:g}->{target:g}",
                "from_mps": previous,
                "to_mps": target,
                "time_s": transition_start,
                "window_end_s": transition_end,
                "settling_deadline_s": (
                    transition_start + settling_limit_s
                    if settling_limit_s is not None
                    else None
                ),
                "window_rows": len(window),
                "settling_tolerance_mps": tolerance,
                "status": status,
                "legacy_settling": legacy_candidate,
                "observed_settling": observed_candidate,
                "complete_finite_candidate_count": complete_finite_candidates,
                "nonfinite_full_candidate_count": nonfinite_full_candidates,
                "sampling_gap_candidate_count": sampling_gap_candidates,
                "partial_candidate_count": partial_candidates,
                "best_complete_tail_max_abs_error_mps": _finite_or_none(best_complete_error),
                "best_complete_candidate_time_s": _finite_or_none(best_complete_time),
            }
        )

    legacy_times = [
        item["legacy_settling"]["settling_time_s"]
        for item in transitions
        if item["legacy_settling"] is not None
    ]
    legacy_max = max(legacy_times, default=float("nan"))
    input_validation = {
        "invalid_time_rows": invalid_time_rows,
        "invalid_measured_rows": invalid_measured_rows,
        "invalid_active_rows": invalid_active_rows,
        "time_order_errors": time_order_errors,
    }
    input_valid = all(value == 0 for value in input_validation.values())
    all_observed = bool(transitions) and all(
        item["status"] == "observed" for item in transitions
    )
    legacy_aggregate_pass = (
        settling_limit_s is not None
        and math.isfinite(legacy_max)
        and legacy_max <= settling_limit_s
    )
    return {
        "diagnostic": "phase1-settling-coverage-v1",
        "tail_window_s": tail_window_s,
        "time_epsilon_s": time_epsilon_s,
        "max_sample_gap_s": max_sample_gap_s,
        "max_sample_gap_scope": "diagnostic data coverage only; not a frozen acceptance threshold",
        "settling_limit_s": settling_limit_s,
        "active_start_cmd_time_s": active_start,
        "active_end_relative_time_s": _finite_or_none(end_time),
        "input_validation": input_validation,
        "input_validation_status": "PASS" if input_valid else "FAIL",
        "transitions": transitions,
        "legacy_finite_settling_time_max_s": _finite_or_none(legacy_max),
        "legacy_finite_transition_count": len(legacy_times),
        "legacy_finite_aggregate_pass": legacy_aggregate_pass,
        "all_transitions_observed": all_observed,
        "audit_status": "PASS" if input_valid and all_observed else "FAIL",
    }


def metadata_for(run_dir: Path) -> dict[str, str]:
    metadata: dict[str, str] = {}
    path = run_dir / "run_metadata.txt"
    if not path.exists():
        return metadata
    for line in path.read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            metadata[key] = value
    return metadata


def audit_run(
    run_dir: Path,
    profile_path: Path,
    *,
    max_sample_gap_s: float = MAX_SAMPLE_GAP_S,
) -> dict[str, Any]:
    active_rows = load_active_rows(run_dir / "data.csv")
    profile = read_profile(profile_path)
    scenario = profile_path.stem.removeprefix("phase1_velocity_")
    settling_limit_s = settling_limit_for_scenario(scenario)
    report = audit_transitions(
        active_rows,
        profile,
        settling_limit_s=settling_limit_s,
        max_sample_gap_s=max_sample_gap_s,
    )
    metadata = metadata_for(run_dir)
    report["source"] = {
        "run_dir": str(run_dir),
        "data_csv": str(run_dir / "data.csv"),
        "profile": str(profile_path),
        "profile_sha256_recorded": metadata.get("profile_sha256", ""),
        "git_head": metadata.get("git_head", ""),
        "git_dirty": metadata.get("git_dirty", ""),
    }
    report["scenario"] = scenario
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Offline diagnostic for complete observed Phase-1 settling tails; "
            "does not change the frozen analyzer or acceptance contract."
        )
    )
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument(
        "--max-sample-gap-s",
        type=float,
        default=MAX_SAMPLE_GAP_S,
        help="diagnostic coverage guard; default 0.05 s",
    )
    args = parser.parse_args()
    report = audit_run(
        args.run_dir,
        args.profile,
        max_sample_gap_s=args.max_sample_gap_s,
    )
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
    print(encoded)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(encoded + "\n")
    return 0 if report["audit_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
