#!/usr/bin/env python3
"""Audit MuJoCo's 6x6 free-base composite mass matrix."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

MATRIX_SIZE = 6
BLOCK_SIZE = 3
MATRIX_FIELDS = tuple(
    f"base_mass_matrix_qcoord_r{row}c{column}"
    for row in range(MATRIX_SIZE)
    for column in range(MATRIX_SIZE)
)


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def symmetrize(matrix: list[list[float]]) -> list[list[float]]:
    return [
        [
            0.5 * (matrix[row][column] + matrix[column][row])
            for column in range(len(matrix))
        ]
        for row in range(len(matrix))
    ]


def max_abs_difference(
    left: list[list[float]], right: list[list[float]]
) -> float:
    return max(
        abs(left[row][column] - right[row][column])
        for row in range(len(left))
        for column in range(len(left))
    )


def matrix_multiply(
    left: list[list[float]], right: list[list[float]]
) -> list[list[float]]:
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def matrix_subtract(
    left: list[list[float]], right: list[list[float]]
) -> list[list[float]]:
    return [
        [left[row][column] - right[row][column] for column in range(3)]
        for row in range(3)
    ]


def inverse_3x3(matrix: list[list[float]]) -> list[list[float]]:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(determinant) <= 1e-12:
        raise ValueError("translation block is singular")
    return [
        [(e * i - f * h) / determinant,
         (c * h - b * i) / determinant,
         (b * f - c * e) / determinant],
        [(f * g - d * i) / determinant,
         (a * i - c * g) / determinant,
         (c * d - a * f) / determinant],
        [(d * h - e * g) / determinant,
         (b * g - a * h) / determinant,
         (a * e - b * d) / determinant],
    ]


def jacobi_eigenvalues(matrix: list[list[float]]) -> list[float]:
    """Return eigenvalues for a small real symmetric matrix."""
    size = len(matrix)
    work = [row[:] for row in symmetrize(matrix)]
    for _ in range(100 * size * size):
        pivot = max(
            (
                (abs(work[row][column]), row, column)
                for row in range(size)
                for column in range(row + 1, size)
            ),
            default=(0.0, 0, 0),
        )
        magnitude, row, column = pivot
        if magnitude <= 1e-12:
            break
        tau = (
            (work[column][column] - work[row][row])
            / (2.0 * work[row][column])
        )
        tangent = (
            1.0 / (abs(tau) + math.sqrt(1.0 + tau * tau))
            if tau >= 0.0
            else -1.0 / (abs(tau) + math.sqrt(1.0 + tau * tau))
        )
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine
        diagonal_row = work[row][row]
        diagonal_column = work[column][column]
        off_diagonal = work[row][column]
        for index in range(size):
            if index in (row, column):
                continue
            row_value = work[index][row]
            column_value = work[index][column]
            work[index][row] = cosine * row_value - sine * column_value
            work[row][index] = work[index][row]
            work[index][column] = sine * row_value + cosine * column_value
            work[column][index] = work[index][column]
        work[row][row] = (
            cosine * cosine * diagonal_row
            - 2.0 * sine * cosine * off_diagonal
            + sine * sine * diagonal_column
        )
        work[column][column] = (
            sine * sine * diagonal_row
            + 2.0 * sine * cosine * off_diagonal
            + cosine * cosine * diagonal_column
        )
        work[row][column] = 0.0
        work[column][row] = 0.0
    return sorted(work[index][index] for index in range(size))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--symmetry-tolerance",
        type=float,
        default=1e-9,
    )
    parser.add_argument(
        "--positive-definite-tolerance",
        type=float,
        default=1e-9,
    )
    args = parser.parse_args()

    if args.symmetry_tolerance < 0.0 or args.positive_definite_tolerance < 0.0:
        print("validation=FAIL: tolerances must be non-negative")
        return 2

    required = {"time_s", *MATRIX_FIELDS}
    matrices: list[list[list[float]]] = []
    previous_time = -math.inf
    try:
        with args.ground_truth_csv.open(
            newline="", encoding="utf-8"
        ) as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames or []
            if len(fields) != len(set(fields)):
                raise ValueError("duplicate CSV fields")
            missing = sorted(required - set(fields))
            if missing:
                raise ValueError("missing fields: " + ",".join(missing))
            for row_number, row in enumerate(reader, start=2):
                time_s = finite(row, "time_s")
                if time_s <= previous_time:
                    raise ValueError(
                        f"row {row_number}: time is not strictly increasing"
                    )
                previous_time = time_s
                matrices.append(
                    [
                        [
                            finite(
                                row,
                                f"base_mass_matrix_qcoord_r{row_index}c{column}",
                            )
                            for column in range(MATRIX_SIZE)
                        ]
                        for row_index in range(MATRIX_SIZE)
                    ]
                )
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not matrices:
        print("validation=FAIL: empty ground-truth CSV")
        return 1

    symmetry_errors: list[float] = []
    full_eigenvalues: list[list[float]] = []
    translation_eigenvalues: list[list[float]] = []
    rotation_eigenvalues: list[list[float]] = []
    schur_eigenvalues: list[list[float]] = []

    try:
        for matrix in matrices:
            symmetric = symmetrize(matrix)
            symmetry_errors.append(max_abs_difference(matrix, symmetric))
            full_eigenvalues.append(jacobi_eigenvalues(symmetric))
            translation = [
                row[:BLOCK_SIZE] for row in symmetric[:BLOCK_SIZE]
            ]
            rotation = [
                row[BLOCK_SIZE:] for row in symmetric[BLOCK_SIZE:]
            ]
            cross_tr = [
                row[BLOCK_SIZE:] for row in symmetric[:BLOCK_SIZE]
            ]
            cross_rt = [
                row[:BLOCK_SIZE] for row in symmetric[BLOCK_SIZE:]
            ]
            translation_eigenvalues.append(
                jacobi_eigenvalues(translation)
            )
            rotation_eigenvalues.append(jacobi_eigenvalues(rotation))
            schur = matrix_subtract(
                rotation,
                matrix_multiply(
                    matrix_multiply(cross_rt, inverse_3x3(translation)),
                    cross_tr,
                ),
            )
            schur_eigenvalues.append(jacobi_eigenvalues(schur))
    except ValueError as exc:
        print(f"validation=FAIL: {exc}")
        return 1

    def minimum(eigenvalues: list[list[float]]) -> float:
        return min(values[0] for values in eigenvalues)

    def maximum(eigenvalues: list[list[float]]) -> float:
        return max(values[-1] for values in eigenvalues)

    symmetry_pass = max(symmetry_errors) <= args.symmetry_tolerance
    positive_definite_pass = all(
        minimum(eigenvalues) > args.positive_definite_tolerance
        for eigenvalues in (
            full_eigenvalues,
            translation_eigenvalues,
            rotation_eigenvalues,
            schur_eigenvalues,
        )
    )
    lines = [
        "base composite mass matrix audit",
        f"rows={len(matrices)}",
        "matrix_dimension=6",
        "q_coordinate_note=free_joint_translation_then_rotation",
        "symmetry_tolerance=%.9g" % args.symmetry_tolerance,
        "positive_definite_tolerance=%.9g"
        % args.positive_definite_tolerance,
        "symmetry_max_abs=%.9g" % max(symmetry_errors),
        "symmetry_p95_abs=%.9g" % percentile(symmetry_errors, 0.95),
        "full_mass_eig_min=%.9g" % minimum(full_eigenvalues),
        "full_mass_eig_max=%.9g" % maximum(full_eigenvalues),
        "translation_block_eig_min=%.9g"
        % minimum(translation_eigenvalues),
        "translation_block_eig_max=%.9g"
        % maximum(translation_eigenvalues),
        "rotation_block_eig_min=%.9g" % minimum(rotation_eigenvalues),
        "rotation_block_eig_max=%.9g" % maximum(rotation_eigenvalues),
        "rotation_schur_eig_min=%.9g" % minimum(schur_eigenvalues),
        "rotation_schur_eig_max=%.9g" % maximum(schur_eigenvalues),
        "symmetry_validation=" + ("PASS" if symmetry_pass else "FAIL"),
        "positive_definite_validation="
        + ("PASS" if positive_definite_pass else "FAIL"),
        "interpretation=qcoord_composite_inertia_not_body_frame_inertia",
        "validation="
        + ("PASS" if symmetry_pass and positive_definite_pass else "FAIL"),
    ]
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if symmetry_pass and positive_definite_pass else 1


if __name__ == "__main__":
    sys.exit(main())
