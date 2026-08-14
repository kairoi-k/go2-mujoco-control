#!/usr/bin/env python3
"""Compare gravity-only and live state-wrench contact torque replays."""

import argparse
import csv
import math
from pathlib import Path


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("%s has no CSV header" % path)
        rows = {}
        for row in reader:
            key = int(row["row_number"])
            if key in rows:
                raise ValueError("%s has duplicate row_number=%d" % (path, key))
            rows[key] = row
    return rows


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def cosine(left, right):
    left_norm = norm(left)
    right_norm = norm(right)
    if left_norm == 0.0 or right_norm == 0.0:
        return 0.0
    return sum(a * b for a, b in zip(left, right)) / (
        left_norm * right_norm
    )


def percentile(values, probability):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (
        position - lower
    )


def pearson(left, right):
    if not left or len(left) != len(right):
        return 0.0
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    numerator = sum(
        (a - left_mean) * (b - right_mean)
        for a, b in zip(left, right)
    )
    denominator = math.sqrt(
        sum((a - left_mean) ** 2 for a in left)
        * sum((b - right_mean) ** 2 for b in right)
    )
    return numerator / denominator if denominator else 0.0


def vector_lines(label, vectors):
    norms = [norm(vector) for vector in vectors]
    max_abs = [max(abs(value) for value in vector) for vector in vectors]
    return [
        "%s_max_norm=%.6f" % (label, max(norms, default=0.0)),
        "%s_p95_norm=%.6f" % (label, percentile(norms, 0.95)),
        "%s_max_abs=%.6f" % (label, max(max_abs, default=0.0)),
        "%s_p95_abs=%.6f" % (label, percentile(max_abs, 0.95)),
        "%s_mean_abs=%.6f" % (
            label,
            sum(max_abs) / len(max_abs) if max_abs else 0.0,
        ),
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gravity-replay", type=Path, required=True)
    parser.add_argument("--state-replay", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--force-epsilon", type=float, default=1e-9)
    args = parser.parse_args()

    gravity_rows = load_rows(args.gravity_replay)
    state_rows = load_rows(args.state_replay)
    common = sorted(set(gravity_rows) & set(state_rows))
    if not common:
        raise ValueError("replays have no common row_number")
    torque_columns = [
        name for name in state_rows[common[0]]
        if name.endswith("_tau_ff_candidate")
    ]
    if not torque_columns:
        raise ValueError("state replay has no candidate torque columns")
    if "desired_force_x_n" not in state_rows[common[0]]:
        raise ValueError("state replay has no desired_force_x_n column")

    gravity = []
    state_total = []
    delta = []
    desired_force = []
    active_contacts = []
    for key in common:
        gravity_vector = [
            float(gravity_rows[key][name]) for name in torque_columns
        ]
        state_vector = [
            float(state_rows[key][name]) for name in torque_columns
        ]
        force = float(state_rows[key]["desired_force_x_n"])
        if not all(
            math.isfinite(value)
            for value in gravity_vector + state_vector + [force]
        ):
            raise ValueError("non-finite value at row_number=%d" % key)
        gravity.append(gravity_vector)
        state_total.append(state_vector)
        delta.append(
            [state_value - gravity_value
             for state_value, gravity_value
             in zip(state_vector, gravity_vector)]
        )
        desired_force.append(force)
        active_contacts.append(
            int(float(state_rows[key]["selected_contact_count"]))
        )

    nonzero = [
        index for index, force in enumerate(desired_force)
        if abs(force) > args.force_epsilon
    ]
    delta_abs = [
        max(abs(value) for value in delta[index])
        for index in nonzero
    ]
    lines = [
        "wbc wrench component replay audit",
        "gravity_rows=%d" % len(gravity_rows),
        "state_rows=%d" % len(state_rows),
        "joined_rows=%d" % len(common),
        "state_desired_force_nonzero_rows=%d" % len(nonzero),
        "state_desired_force_min_n=%.6f" % (
            min((desired_force[index] for index in nonzero), default=0.0)
        ),
        "state_desired_force_max_n=%.6f" % (
            max((desired_force[index] for index in nonzero), default=0.0)
        ),
        "state_total_vs_gravity_cosine=%.6f" % (
            sum(
                cosine(state, gravity_vector)
                for state, gravity_vector in zip(state_total, gravity)
            ) / len(common)
        ),
    ]
    for label, vectors in (
        ("gravity", gravity),
        ("state_total", state_total),
        ("state_minus_gravity", delta),
    ):
        lines.extend(vector_lines(label, vectors))
    lines.extend(
        [
            "nonzero_force_delta_rows=%d" % len(delta_abs),
            "nonzero_force_delta_max_abs=%.6f" % max(delta_abs, default=0.0),
            "nonzero_force_delta_p95_abs=%.6f" % percentile(delta_abs, 0.95),
            "nonzero_force_delta_mean_abs=%.6f" % (
                sum(delta_abs) / len(delta_abs) if delta_abs else 0.0
            ),
        ]
    )

    for contact_count in (2, 4):
        subset_positions = [
            position for position, index in enumerate(nonzero)
            if active_contacts[index] == contact_count
        ]
        subset_abs = [delta_abs[position] for position in subset_positions]
        lines.extend(
            [
                "contact_%d_nonzero_force_rows=%d" % (
                    contact_count, len(subset_positions)
                ),
                "contact_%d_delta_p95_abs=%.6f" % (
                    contact_count, percentile(subset_abs, 0.95)
                ),
            ]
        )

    force_values = [desired_force[index] for index in nonzero]
    correlations = []
    for column_index, column in enumerate(torque_columns):
        values = [delta[index][column_index] for index in nonzero]
        correlations.append(
            (abs(pearson(force_values, values)), column,
             pearson(force_values, values))
        )
    correlations.sort(reverse=True)
    for _, column, correlation in correlations[:3]:
        lines.append(
            "strongest_delta_force_correlation_%s=%.6f"
            % (column, correlation)
        )

    lines.append("validation=PASS")
    report = "\n".join(lines) + "\n"
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")


if __name__ == "__main__":
    main()
