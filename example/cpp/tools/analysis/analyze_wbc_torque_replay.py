#!/usr/bin/env python3
"""Compare replayed contact torques with the controller's current PD torque."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
MOTORS = tuple(f"{leg}_{joint}" for leg in LEGS for joint in JOINTS)


def finite(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("percentile of empty sequence")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def cosine(left: list[float], right: list[float]) -> float:
    dot = sum(a * b for a, b in zip(left, right))
    left_norm = math.sqrt(sum(a * a for a in left))
    right_norm = math.sqrt(sum(b * b for b in right))
    if left_norm <= 1e-12 or right_norm <= 1e-12:
        return math.nan
    return dot / (left_norm * right_norm)


def relation(
    candidates: list[list[float]],
    references: list[list[float]],
) -> tuple[float, int, int]:
    flat_candidates = [value for row in candidates for value in row]
    flat_references = [value for row in references for value in row]
    eligible = [
        (candidate, reference)
        for candidate, reference in zip(flat_candidates, flat_references)
        if abs(candidate) > 0.1 and abs(reference) > 0.1
    ]
    same_sign = sum(candidate * reference > 0 for candidate, reference in eligible)
    return cosine(flat_candidates, flat_references), same_sign, len(eligible)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        return list(reader)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-csv", required=True)
    parser.add_argument("--replay-csv", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--scales",
        nargs="+",
        type=float,
        default=[0.10, 0.25],
        help="candidate torque scales to audit",
    )
    parser.add_argument("--joint-limit", type=float, default=23.7)
    args = parser.parse_args()

    if (
        not args.scales
        or any(not math.isfinite(scale) or scale <= 0.0 for scale in args.scales)
        or not math.isfinite(args.joint_limit)
        or args.joint_limit <= 0.0
    ):
        print("validation=FAIL: invalid scale or joint limit", file=sys.stderr)
        return 2

    try:
        state_rows = read_rows(Path(args.state_csv))
        replay_rows = read_rows(Path(args.replay_csv))
        required_state = {
            "motion_stage",
            "cycle_index",
            *(f"{motor}_{suffix}" for motor in MOTORS for suffix in (
                "q_target",
                "dq_target",
                "kp",
                "kd",
                "tau_ff",
                "q_state",
                "dq_state",
                "tau_est",
            )),
        }
        required_replay = {
            "row_number",
            *(f"{motor}_tau_ff_candidate" for motor in MOTORS),
        }
        missing_state = sorted(required_state.difference(state_rows[0] if state_rows else {}))
        missing_replay = sorted(required_replay.difference(replay_rows[0] if replay_rows else {}))
        if missing_state:
            raise ValueError("state CSV missing columns: " + ",".join(missing_state))
        if missing_replay:
            raise ValueError("replay CSV missing columns: " + ",".join(missing_replay))
    except (OSError, ValueError, KeyError, IndexError) as exc:
        print(f"validation=FAIL: {exc}", file=sys.stderr)
        return 2

    joined: list[dict[str, object]] = []
    seen_rows: set[int] = set()
    try:
        for replay_row in replay_rows:
            state_row_number = int(replay_row["row_number"])
            if state_row_number in seen_rows:
                raise ValueError(f"duplicate replay row_number {state_row_number}")
            if not 1 <= state_row_number <= len(state_rows):
                raise ValueError(f"replay row_number out of range: {state_row_number}")
            seen_rows.add(state_row_number)
            state_row = state_rows[state_row_number - 1]
            pd_torque = []
            candidate_torque = []
            estimated_torque = []
            for motor in MOTORS:
                pd_torque.append(
                    finite(state_row, f"{motor}_tau_ff")
                    + finite(state_row, f"{motor}_kp")
                    * (
                        finite(state_row, f"{motor}_q_target")
                        - finite(state_row, f"{motor}_q_state")
                    )
                    + finite(state_row, f"{motor}_kd")
                    * (
                        finite(state_row, f"{motor}_dq_target")
                        - finite(state_row, f"{motor}_dq_state")
                    )
                )
                candidate_torque.append(
                    finite(replay_row, f"{motor}_tau_ff_candidate")
                )
                estimated_torque.append(finite(state_row, f"{motor}_tau_est"))
            joined.append(
                {
                    "pd": pd_torque,
                    "candidate": candidate_torque,
                    "estimated": estimated_torque,
                    "walking": (
                        int(finite(state_row, "motion_stage")) == 2
                        and int(finite(state_row, "cycle_index")) >= 0
                    ),
                }
            )
    except (ValueError, KeyError) as exc:
        print(f"validation=FAIL: {exc}", file=sys.stderr)
        return 2

    if not joined:
        print("validation=FAIL: no replay rows joined to state rows", file=sys.stderr)
        return 1

    candidate_rows = [row["candidate"] for row in joined]
    pd_rows = [row["pd"] for row in joined]
    estimated_rows = [row["estimated"] for row in joined]
    walking = [row for row in joined if row["walking"]]
    lines = [
        "wbc torque replay audit",
        f"state_rows={len(state_rows)}",
        f"replay_rows={len(replay_rows)}",
        f"joined_rows={len(joined)}",
        f"walking_rows={len(walking)}",
        "pd_definition=tau_ff+kp*(q_target-q_state)+kd*(dq_target-dq_state)",
        f"candidate_max_abs={max(abs(value) for row in candidate_rows for value in row):.6f}",
        f"candidate_p95_abs={percentile([abs(value) for row in candidate_rows for value in row], 0.95):.6f}",
        f"pd_max_abs={max(abs(value) for row in pd_rows for value in row):.6f}",
        f"pd_p95_abs={percentile([abs(value) for row in pd_rows for value in row], 0.95):.6f}",
    ]

    for label, subset in (("all", joined), ("walking", walking)):
        candidate_subset = [row["candidate"] for row in subset]
        pd_subset = [row["pd"] for row in subset]
        estimated_subset = [row["estimated"] for row in subset]
        pd_cosine, pd_same, pd_eligible = relation(candidate_subset, pd_subset)
        est_cosine, est_same, est_eligible = relation(
            candidate_subset, estimated_subset
        )
        lines.extend(
            [
                f"{label}_candidate_vs_pd_cosine={pd_cosine:.6f}",
                f"{label}_candidate_vs_pd_same_sign={pd_same}/{pd_eligible}",
                f"{label}_candidate_vs_tau_est_cosine={est_cosine:.6f}",
                f"{label}_candidate_vs_tau_est_same_sign={est_same}/{est_eligible}",
            ]
        )

    for scale in args.scales:
        for sign in (1.0, -1.0):
            combined = [
                [
                    pd + sign * scale * candidate
                    for pd, candidate in zip(row["pd"], row["candidate"])
                ]
                for row in joined
            ]
            combined_abs = [abs(value) for row in combined for value in row]
            rows_over_limit = sum(
                max(abs(value) for value in row) > args.joint_limit
                for row in combined
            )
            delta_abs = [
                abs(sign * scale * candidate)
                for row in joined
                for candidate in row["candidate"]
            ]
            sign_label = "plus" if sign > 0 else "minus"
            prefix = f"scale_{scale:.3f}_{sign_label}"
            lines.extend(
                [
                    f"{prefix}_combined_max_abs={max(combined_abs):.6f}",
                    f"{prefix}_combined_p95_abs={percentile(combined_abs, 0.95):.6f}",
                    f"{prefix}_rows_over_joint_limit={rows_over_limit}",
                    f"{prefix}_candidate_delta_max_abs={max(delta_abs):.6f}",
                    f"{prefix}_candidate_delta_mean_abs={sum(delta_abs) / len(delta_abs):.6f}",
                ]
            )

    for index, motor in enumerate(MOTORS):
        values = [abs(row["candidate"][index]) for row in joined]
        lines.append(
            f"candidate_{motor}_max_abs={max(values):.6f}"
        )
        lines.append(
            f"candidate_{motor}_p95_abs={percentile(values, 0.95):.6f}"
        )

    lines.append("validation=PASS")
    report = "\n".join(lines) + "\n"
    try:
        output_path = Path(args.out)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(report, encoding="utf-8")
    except OSError as exc:
        print(f"validation=FAIL: cannot write report: {exc}", file=sys.stderr)
        return 2
    print(report, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
