#!/usr/bin/env python3
"""Replay CSV evidence for the opt-in terrain planner-consumer shadow."""
import argparse
import collections
import csv
import json
import math
import pathlib
import sys

REQUIRED = (
    "state_tick_s", "terrain_plan_id", "terrain_plan_epoch",
    "terrain_plan_valid", "terrain_model_com_valid",
    "terrain_model_com_state_stamp_s",
    "motion_stage",
    "wbc_terrain_execution_shadow_enabled",
    "wbc_terrain_execution_shadow_checked",
    "wbc_terrain_execution_snapshot_valid",
    "wbc_terrain_execution_shadow_rejection_code",
    "wbc_terrain_execution_shadow_failure_reason",
    "wbc_terrain_execution_shadow_plan_id",
    "wbc_terrain_execution_shadow_plan_epoch",
)
LEGS = ("FR", "FL", "RR", "RL")
COMMITMENT_SUFFIXES = (
    "_commitment_valid", "_commitment_in_flight", "_source_plan_id",
    "_source_plan_epoch", "_target_time_s", "_target_x_m",
    "_target_y_m", "_target_z_m",
)


def number(row, name):
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(name + " is not finite")
    return value


def inspect_csv(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        missing = [name for name in REQUIRED if name not in fields]
        for leg in LEGS:
            missing.extend(
                "terrain_shadow_" + leg + suffix
                for suffix in COMMITMENT_SUFFIXES
                if "terrain_shadow_" + leg + suffix not in fields)
        if missing:
            return {"status": "missing_fields", "path": str(path),
                    "missing": missing}
        rows = 0
        com_valid = 0
        com_timestamp_mismatch = 0
        shadow_enabled = 0
        shadow_checked = 0
        shadow_valid = 0
        shadow_rejected = 0
        rejection_codes = collections.Counter()
        failure_reasons = collections.Counter()
        stable_rows = 0
        stable_checked = 0
        stable_valid = 0
        stable_invalid_checked = 0
        stable_unchecked = 0
        errors = []
        for row in reader:
            rows += 1
            state_time = number(row, "state_tick_s")
            stable = int(number(row, "motion_stage")) == 2
            if stable:
                stable_rows += 1
            if int(number(row, "terrain_model_com_valid")):
                com_valid += 1
                com_time = number(row, "terrain_model_com_state_stamp_s")
                if abs(com_time - state_time) > 1.0e-6:
                    com_timestamp_mismatch += 1
            if not int(number(
                    row, "wbc_terrain_execution_shadow_enabled")):
                continue
            shadow_enabled += 1
            checked = int(number(
                row, "wbc_terrain_execution_shadow_checked"))
            valid = int(number(
                row, "wbc_terrain_execution_snapshot_valid"))
            rejection = int(number(
                row, "wbc_terrain_execution_shadow_rejection_code"))
            reason = int(number(
                row, "wbc_terrain_execution_shadow_failure_reason"))
            rejection_codes[str(rejection)] += 1
            failure_reasons[str(reason)] += 1
            if checked:
                shadow_checked += 1
                if stable:
                    stable_checked += 1
            if valid:
                shadow_valid += 1
                if stable:
                    stable_valid += 1
                if not checked:
                    errors.append("valid sample was not checked")
                if number(row, "wbc_terrain_execution_shadow_plan_id") <= 0:
                    errors.append("valid sample has no plan id")
                if number(row, "wbc_terrain_execution_shadow_plan_epoch") <= 0:
                    errors.append("valid sample has no plan epoch")
                for leg in LEGS:
                    prefix = "terrain_shadow_" + leg
                    if not int(number(row, prefix + "_commitment_valid")):
                        continue
                    if number(row, prefix + "_source_plan_id") <= 0 or                             number(row, prefix + "_source_plan_epoch") <= 0:
                        errors.append("valid commitment has no source plan")
                    for suffix in ("_target_time_s", "_target_x_m",
                                   "_target_y_m", "_target_z_m"):
                        number(row, prefix + suffix)
            elif checked:
                shadow_rejected += 1
                if stable:
                    stable_invalid_checked += 1
                if rejection == 0:
                    errors.append("checked invalid sample has no rejection")
            elif stable:
                stable_unchecked += 1
            elif valid:
                errors.append("unchecked sample was marked valid")
        if stable_invalid_checked:
            stable_gate = "fail"
        elif stable_rows == 0 or stable_unchecked:
            stable_gate = "incomplete"
        else:
            stable_gate = "pass"
        result = {
            "status": "pass" if not errors and
                com_timestamp_mismatch == 0 else "fail",
            "path": str(path), "rows": rows,
            "model_com_valid_rows": com_valid,
            "model_com_timestamp_mismatch_rows": com_timestamp_mismatch,
            "shadow_enabled_rows": shadow_enabled,
            "shadow_checked_rows": shadow_checked,
            "shadow_valid_rows": shadow_valid,
            "shadow_rejected_rows": shadow_rejected,
            "shadow_rejection_code_counts": dict(rejection_codes),
            "shadow_failure_reason_counts": dict(failure_reasons),
            "stable_rows": stable_rows,
            "stable_checked_rows": stable_checked,
            "stable_valid_rows": stable_valid,
            "stable_invalid_checked_rows": stable_invalid_checked,
            "stable_unchecked_rows": stable_unchecked,
            "stable_checked_valid_rate": (
                stable_valid / stable_checked if stable_checked else None),
            "stable_shadow_gate": stable_gate,
            "errors": errors,
        }
        if shadow_enabled and shadow_valid == 0:
            result["status"] = "fail"
            result["errors"].append(
                "shadow enabled but no checked sample passed")
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=pathlib.Path)
    parser.add_argument("--expect-missing-model-com", action="store_true")
    args = parser.parse_args()
    result = inspect_csv(args.csv)
    print(json.dumps(result, sort_keys=True))
    if result["status"] == "missing_fields":
        return 0 if args.expect_missing_model_com else 2
    return 0 if result["status"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
