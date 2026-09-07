#!/usr/bin/env python3
"""Audit current-model WBC physical-certificate telemetry.
This read-only diagnostic evaluates active motion rows, keeps incomplete rows
visible, and never treats the certificate as command selection or B1 evidence.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

CERT_PREFIXES = ("attempt", "selected")
CERT_FIELDS = (
    "checked",
    "valid",
    "input_valid",
    "feasible",
    "failure_mask",
    "assumed_flat_mask",
    "force_residual_n",
    "moment_residual_nm",
    "joint_residual_nm",
    "friction_violation_n",
    "normal_violation_n",
    "swing_violation_n",
    "torque_violation_nm",
    "stance_acc_residual_mps2",
)
COMMON_FIELDS = (
    "motion_stage",
    "velocity_command_active",
    "state_tick_s",
    "wbc_full_id_ok",
    "wbc_full_id_attempt_qp_converged",
    "wbc_full_cert_selected_reused",
    "wbc_full_certificate_elapsed_us",
)
FLAG_FIELDS = {"checked", "valid", "input_valid", "feasible"}
MASK_FIELDS = {"failure_mask", "assumed_flat_mask"}
RESIDUAL_FIELDS = tuple(
    field for field in CERT_FIELDS if field not in FLAG_FIELDS | MASK_FIELDS
)
FAILURE_BITS = {
    "invalid_configuration": 1 << 0,
    "invalid_dynamics": 1 << 1,
    "input_conflict": 1 << 2,
    "nonfinite_proposal": 1 << 3,
    "dynamics_residual": 1 << 4,
    "normal_force": 1 << 5,
    "friction": 1 << 6,
    "swing_force": 1 << 7,
    "torque": 1 << 8,
    "hard_stance_acceleration": 1 << 9,
}
FAILURE_MASK_LIMIT = (1 << 10) - 1
ASSUMED_FLAT_MASK_LIMIT = (1 << 4) - 1
COMMON_FLAG_FIELDS = {
    "velocity_command_active",
    "wbc_full_id_ok",
    "wbc_full_id_attempt_qp_converged",
    "wbc_full_cert_selected_reused",
}
REQUIRED_FIELDS = COMMON_FIELDS + tuple(
    f"wbc_full_cert_{prefix}_{field}"
    for prefix in CERT_PREFIXES
    for field in CERT_FIELDS
)

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

def _number(row: Mapping[str, Any], name: str) -> tuple[float | None, str | None]:
    if name not in row or row[name] is None or str(row[name]).strip() == "":
        return None, "missing"
    try:
        value = float(row[name])
    except (TypeError, ValueError):
        return None, "invalid"
    if not math.isfinite(value):
        return None, "nonfinite"
    return value, None

def _flag(row: Mapping[str, Any], name: str) -> tuple[bool | None, str | None]:
    value, issue = _number(row, name)
    if issue is not None or value is None:
        return None, issue
    if value not in (0.0, 1.0):
        return None, "invalid"
    return value == 1.0, None

def _nonnegative(
    row: Mapping[str, Any], name: str
) -> tuple[float | None, str | None]:
    value, issue = _number(row, name)
    if issue is not None or value is None:
        return None, issue
    if value < 0.0:
        return None, "invalid"
    return value, None

def _mask(
    row: Mapping[str, Any], name: str, limit: int
) -> tuple[float | None, str | None]:
    value, issue = _number(row, name)
    if issue is not None or value is None:
        return None, issue
    if value < 0.0 or not value.is_integer() or value > limit:
        return None, "invalid"
    return value, None

def _common_value(
    row: Mapping[str, Any], field: str
) -> tuple[float | None, str | None]:
    if field in COMMON_FLAG_FIELDS:
        value, issue = _flag(row, field)
        return (float(value) if issue is None and value is not None else None), issue
    if field == "wbc_full_certificate_elapsed_us":
        return _nonnegative(row, field)
    return _number(row, field)

def _cert_value(
    row: Mapping[str, Any], prefix: str, field: str
) -> tuple[float | None, str | None]:
    name = f"wbc_full_cert_{prefix}_{field}"
    if field in FLAG_FIELDS:
        value, issue = _flag(row, name)
        return (float(value) if issue is None and value is not None else None), issue
    if field == "failure_mask":
        return _mask(row, name, FAILURE_MASK_LIMIT)
    if field == "assumed_flat_mask":
        return _mask(row, name, ASSUMED_FLAT_MASK_LIMIT)
    return _nonnegative(row, name)

def _percentile(values: Sequence[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])

def _coverage(
    rows: Sequence[Mapping[str, Any]],
    field: str,
    parser: Callable[[Mapping[str, Any], str], tuple[float | None, str | None]] = _number,
) -> dict[str, Any]:
    counts = Counter()
    for row in rows:
        _, issue = parser(row, field)
        counts[issue or "finite"] += 1
    total = len(rows)
    finite = counts["finite"]
    return {
        "rows": total,
        "finite": finite,
        "missing": counts["missing"],
        "nonfinite": counts["nonfinite"],
        "invalid": counts["invalid"],
        "coverage": finite / total if total else None,
    }

def _numeric_stats(
    rows: Sequence[Mapping[str, Any]],
    field: str,
    parser: Callable[[Mapping[str, Any], str], tuple[float | None, str | None]] = _number,
) -> dict[str, Any]:
    values: list[float] = []
    counts = Counter()
    for row in rows:
        value, issue = parser(row, field)
        if issue is None and value is not None:
            values.append(value)
        else:
            counts[issue or "invalid"] += 1
    return {
        "rows": len(rows),
        "finite": len(values),
        "missing": counts["missing"],
        "nonfinite": counts["nonfinite"],
        "invalid": counts["invalid"],
        "coverage": len(values) / len(rows) if rows else None,
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "max": max(values) if values else None,
    }

def _mask_key(value: float) -> str:
    return str(int(value)) if value.is_integer() else format(value, ".15g")

def _counter_dict(values: Iterable[str]) -> dict[str, int]:
    return dict(sorted(Counter(values).items()))

def _status_for(row: Mapping[str, Any], prefix: str) -> str:
    if any(_cert_value(row, prefix, field)[1] is not None for field in CERT_FIELDS):
        return "UNKNOWN"
    checked, _ = _cert_value(row, prefix, "checked")
    valid, _ = _cert_value(row, prefix, "valid")
    input_valid, _ = _cert_value(row, prefix, "input_valid")
    feasible, _ = _cert_value(row, prefix, "feasible")
    failure_mask, _ = _cert_value(row, prefix, "failure_mask")
    if not checked:
        return "UNKNOWN"
    if not input_valid or not valid or not feasible:
        return "NOT_CERTIFIED"
    if failure_mask != 0.0:
        return "NOT_CERTIFIED"
    return "CERTIFIED"

def _failure_category_coverage(
    rows: Sequence[Mapping[str, Any]], prefix: str
) -> dict[str, Any]:
    field = f"wbc_full_cert_{prefix}_failure_mask"
    known_mask = sum(FAILURE_BITS.values())
    bit_rows = {name: 0 for name in FAILURE_BITS}
    finite_rows = 0
    invalid_rows = 0
    unknown_bit_rows = 0
    for row in rows:
        value, issue = _mask(row, field, FAILURE_MASK_LIMIT)
        if issue is not None or value is None:
            invalid_rows += 1
            continue
        finite_rows += 1
        mask = int(value)
        for name, bit in FAILURE_BITS.items():
            if mask & bit:
                bit_rows[name] += 1
        if mask & ~known_mask:
            unknown_bit_rows += 1
    result: dict[str, Any] = {
        name: {
            "bit": bit,
            "rows": bit_rows[name],
            "coverage": bit_rows[name] / len(rows) if rows else None,
        }
        for name, bit in FAILURE_BITS.items()
    }
    result.update(
        {
            "mask_finite_rows": finite_rows,
            "mask_invalid_or_unknown_rows": invalid_rows,
            "unknown_bit_rows": unknown_bit_rows,
            "known_mask": known_mask,
        }
    )
    return result

def _certificate_report(
    rows: Sequence[Mapping[str, Any]], prefix: str
) -> dict[str, Any]:
    statuses = [_status_for(row, prefix) for row in rows]
    status_counts = Counter(statuses)
    parser = lambda row, field: _cert_value(
        row, prefix, field.removeprefix(f"wbc_full_cert_{prefix}_")
    )
    fields = {
        field: _coverage(rows, f"wbc_full_cert_{prefix}_{field}", parser)
        for field in CERT_FIELDS
    }
    flag_counts: dict[str, Any] = {}
    for field in sorted(FLAG_FIELDS):
        values = []
        issues = Counter()
        for row in rows:
            value, issue = _cert_value(row, prefix, field)
            if issue is None and value is not None:
                values.append(value >= 0.5)
            else:
                issues[issue or "invalid"] += 1
        flag_counts[field] = {
            "true": sum(values),
            "false": len(values) - sum(values),
            "missing": issues["missing"],
            "nonfinite": issues["nonfinite"],
            "invalid": issues["invalid"],
        }
    feasible = flag_counts["feasible"]
    checked = flag_counts["checked"]
    input_valid = flag_counts["input_valid"]
    valid = flag_counts["valid"]
    feasible_true = int(feasible["true"])
    checked_true = int(checked["true"])
    valid_true = int(valid["true"])
    input_valid_true = int(input_valid["true"])
    masks: dict[str, dict[str, int]] = {}
    for field in sorted(MASK_FIELDS):
        values = []
        for row in rows:
            value, issue = _cert_value(row, prefix, field)
            if issue is None and value is not None:
                values.append(_mask_key(value))
        masks[field] = _counter_dict(values)
    return {
        "active_rows": len(rows),
        "status_counts": dict(sorted(status_counts.items())),
        "field_coverage": fields,
        "flag_counts": flag_counts,
        "mask_counts": masks,
        "failure_category_coverage": _failure_category_coverage(rows, prefix),
        "residuals": {
            field: _numeric_stats(
                rows, f"wbc_full_cert_{prefix}_{field}", parser
            )
            for field in RESIDUAL_FIELDS
        },
        "checked_rows": checked_true,
        "valid_rows": valid_true,
        "input_valid_rows": input_valid_true,
        "feasible_rows": feasible_true,
        "feasible_ratio": feasible_true / len(rows) if rows else None,
        "known_feasible_ratio": (
            feasible_true / (feasible_true + int(feasible["false"]))
            if feasible_true + int(feasible["false"])
            else None
        ),
        "checked_feasible_ratio": (
            feasible_true / checked_true if checked_true else None
        ),
        "input_valid_feasible_ratio": (
            feasible_true / input_valid_true if input_valid_true else None
        ),
        "feasible_with_failure_mask_rows": sum(
            _cert_value(row, prefix, "feasible")[0] == 1.0
            and (_cert_value(row, prefix, "failure_mask")[0] or 0.0) != 0.0
            for row in rows
        ),
    }

def _selection(
    rows: Sequence[Mapping[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    active: list[dict[str, Any]] = []
    inactive = 0
    unknown = 0
    invalid = 0
    reasons: Counter[str] = Counter()
    for index, row in enumerate(rows, start=1):
        stage, stage_issue = _number(row, "motion_stage")
        command, command_issue = _number(row, "velocity_command_active")
        if stage_issue is not None or command_issue is not None:
            unknown += 1
            reasons[stage_issue or command_issue or "unknown"] += 1
            continue
        if stage is None or command is None:
            unknown += 1
            reasons["unknown"] += 1
            continue
        if not stage.is_integer() or command not in (0.0, 1.0):
            unknown += 1
            invalid += 1
            reasons["invalid_selector"] += 1
            continue
        if int(stage) in (2, 3) and command == 1.0:
            active.append({"index": index, "row": row})
        else:
            inactive += 1
    return active, {
        "total_rows": len(rows),
        "active_rows": len(active),
        "inactive_rows": inactive,
        "unknown_rows": unknown,
        "invalid_rows": invalid,
        "unknown_reasons": dict(sorted(reasons.items())),
        "predicate": "motion_stage in {2,3} and velocity_command_active == 1",
    }

def _group_summary(entries: Sequence[dict[str, Any]]) -> dict[str, Any]:
    attempt_status = [item["attempt_status"] for item in entries]
    selected_status = [item["selected_status"] for item in entries]
    selected_certified = sum(status == "CERTIFIED" for status in selected_status)
    return {
        "rows": len(entries),
        "legacy_accepted_rows": sum(
            item["legacy_accepted"] is True for item in entries
        ),
        "attempt_status_counts": dict(sorted(Counter(attempt_status).items())),
        "selected_status_counts": dict(sorted(Counter(selected_status).items())),
        "selected_feasible_rows": selected_certified,
        "selected_feasible_ratio": (
            selected_certified / len(entries) if entries else None
        ),
        "legacyaccepted_but_selected_cert_not_pass_rows": sum(
            item["legacy_accepted"] is True
            and item["selected_status"] != "CERTIFIED"
            for item in entries
        ),
        "attempt_failure_mask_counts": _counter_dict(
            item["attempt_failure_mask"]
            for item in entries
            if item["attempt_failure_mask"] is not None
        ),
        "selected_failure_mask_counts": _counter_dict(
            item["selected_failure_mask"]
            for item in entries
            if item["selected_failure_mask"] is not None
        ),
    }

def _required_coverage(
    rows: Sequence[Mapping[str, Any]], field: str
) -> dict[str, Any]:
    if field in COMMON_FIELDS:
        return _coverage(rows, field, _common_value)
    rest = field.removeprefix("wbc_full_cert_")
    prefix, leaf = rest.split("_", 1)
    parser = lambda row, _field: _cert_value(row, prefix, leaf)
    return _coverage(rows, field, parser)

def audit_rows(
    rows: Sequence[Mapping[str, Any]],
    fields: Iterable[str] | None = None,
    *,
    manifest_source: str | None = None,
    manifest_sha256: str | None = None,
    manifest: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    rows = list(rows)
    field_set = set(fields or (key for row in rows for key in row))
    missing_fields = sorted(set(REQUIRED_FIELDS) - field_set)
    selected, selection = _selection(rows)
    active_rows = [item["row"] for item in selected]
    certificates = {
        prefix: _certificate_report(active_rows, prefix)
        for prefix in CERT_PREFIXES
    }
    coverage_by_class = {
        prefix: certificates[prefix]["field_coverage"] for prefix in CERT_PREFIXES
    }
    entries: list[dict[str, Any]] = []
    for item in selected:
        row = item["row"]
        accepted, accepted_issue = _flag(row, "wbc_full_id_ok")
        converged, converged_issue = _flag(
            row, "wbc_full_id_attempt_qp_converged"
        )
        attempt_mask, attempt_mask_issue = _mask(
            row, "wbc_full_cert_attempt_failure_mask", FAILURE_MASK_LIMIT
        )
        selected_mask, selected_mask_issue = _mask(
            row, "wbc_full_cert_selected_failure_mask", FAILURE_MASK_LIMIT
        )
        entries.append(
            {
                "index": item["index"],
                "attempt_status": _status_for(row, "attempt"),
                "selected_status": _status_for(row, "selected"),
                "legacy_accepted": None if accepted_issue else accepted,
                "converged": None if converged_issue else converged,
                "attempt_failure_mask": (
                    _mask_key(attempt_mask)
                    if attempt_mask_issue is None and attempt_mask is not None
                    else None
                ),
                "selected_failure_mask": (
                    _mask_key(selected_mask)
                    if selected_mask_issue is None and selected_mask is not None
                    else None
                ),
            }
        )
    groups: dict[str, list[dict[str, Any]]] = {
        "converged": [],
        "nonconverged": [],
        "unknown": [],
    }
    for entry in entries:
        if entry["converged"] is True:
            groups["converged"].append(entry)
        elif entry["converged"] is False:
            groups["nonconverged"].append(entry)
        else:
            groups["unknown"].append(entry)
    convergence_groups = {
        name: _group_summary(group) for name, group in groups.items()
    }
    legacy_accepted_rows = sum(
        item["legacy_accepted"] is True for item in entries
    )
    legacy_unknown_rows = sum(item["legacy_accepted"] is None for item in entries)
    legacy_rejected_rows = sum(item["legacy_accepted"] is False for item in entries)
    legacy_but_not_certified = sum(
        item["legacy_accepted"] is True
        and item["selected_status"] != "CERTIFIED"
        for item in entries
    )
    reuse_values = []
    for row in active_rows:
        value, issue = _flag(row, "wbc_full_cert_selected_reused")
        if issue is None and value is not None:
            reuse_values.append(value)
    overhead = _numeric_stats(
        active_rows,
        "wbc_full_certificate_elapsed_us",
        _common_value,
    )
    required_coverage = {
        field: _required_coverage(rows, field) for field in REQUIRED_FIELDS
    }
    input_issues = {
        field: required_coverage[field]
        for field in COMMON_FIELDS
        if required_coverage[field]["missing"]
        or required_coverage[field]["nonfinite"]
        or required_coverage[field]["invalid"]
    }
    all_statuses = [
        status
        for prefix in CERT_PREFIXES
        for status in certificates[prefix]["status_counts"]
        for _ in range(certificates[prefix]["status_counts"][status])
    ]
    complete_and_certified = (
        bool(active_rows)
        and not missing_fields
        and not selection["unknown_rows"]
        and not input_issues
        and all(status == "CERTIFIED" for status in all_statuses)
    )
    overall_status = "PASS" if complete_and_certified else "NOT_CERTIFIED"
    return {
        "schema": "wbc-physical-certificate-audit-v1",
        "status": overall_status,
        "acceptance_scope": {
            "diagnostic_only": True,
            "b1_acceptance": False,
            "b1_acceptance_status": "NOT_APPLICABLE",
            "physical_cone_certificate": True,
            "final_pd_included": False,
            "note": "The physical-cone certificate excludes finalPD torque and does not establish B1 acceptance.",
        },
        "selection": selection,
        "required_fields": list(REQUIRED_FIELDS),
        "missing_header_fields": missing_fields,
        "input_validation": {
            "status": (
                "PASS" if not missing_fields and not input_issues else "FAIL"
            ),
            "required_field_coverage": required_coverage,
            "common_field_issues": input_issues,
        },
        "certificates": certificates,
        "failure_category_coverage": {
            prefix: certificates[prefix]["failure_category_coverage"]
            for prefix in CERT_PREFIXES
        },
        "legacy": {
            "accepted_rows": legacy_accepted_rows,
            "rejected_rows": legacy_rejected_rows,
            "unknown_rows": legacy_unknown_rows,
            "legacyaccepted_but_cert_not_pass_rows": legacy_but_not_certified,
        },
        "convergence_groups": convergence_groups,
        "validator_overhead_us": overhead,
        "reuse": {
            "reused_rows": sum(reuse_values),
            "not_reused_rows": len(reuse_values) - sum(reuse_values),
            "unknown_rows": len(active_rows) - len(reuse_values),
            "count": sum(reuse_values),
        },
        "source": {
            "manifest_source": manifest_source,
            "manifest_sha256": manifest_sha256,
            "run_id": (manifest or {}).get("run_id"),
            "schema_version": (manifest or {}).get("schema_version"),
            "repository": (manifest or {}).get("repository", {}),
        },
    }

def audit(
    rows: Sequence[Mapping[str, Any]], fields: Iterable[str] | None = None
) -> dict[str, Any]:
    return audit_rows(rows, fields)

def _read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = list(reader.fieldnames or [])
        if not fields:
            raise ValueError(f"CSV has no header: {path}")
        if len(fields) != len(set(fields)):
            raise ValueError(f"CSV has duplicate fields: {path}")
        rows = list(reader)
    return rows, fields

def _read_manifest(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"run manifest must be a JSON object: {path}")
    return value

def audit_run(
    run_dir: Path,
    *,
    manifest_path: Path | None = None,
    analyzer_path: Path | None = None,
) -> dict[str, Any]:
    run_dir = Path(run_dir)
    data_path = run_dir / "data.csv"
    if manifest_path is None:
        candidates = (run_dir / "run_manifest.json", run_dir / "run_manifest")
        manifest_path = next(
            (path for path in candidates if path.exists()), candidates[0]
        )
    rows, fields = _read_csv(data_path)
    manifest = _read_manifest(manifest_path)
    analyzer = analyzer_path or Path(__file__)
    data_hash = sha256(data_path)
    manifest_hash = sha256(manifest_path)
    analyzer_hash = sha256(analyzer)
    report = audit_rows(
        rows,
        fields,
        manifest_source=str(manifest_path),
        manifest_sha256=manifest_hash,
        manifest=manifest,
    )
    report["input_sha256"] = {
        str(data_path): data_hash,
        str(manifest_path): manifest_hash,
    }
    report["analyzer_sha256"] = {str(analyzer): analyzer_hash}
    report["script_sha256"] = analyzer_hash
    report["source"]["data_csv"] = str(data_path)
    return report

def _write_exclusive(path: Path, report: Mapping[str, Any]) -> str:
    text = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    return text

def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--manifest",
        "--run-manifest",
        dest="manifest_path",
        type=Path,
        default=None,
    )
    args = parser.parse_args(argv)
    if args.out.exists():
        print(
            f"validation=FAIL: output already exists: {args.out}",
            file=sys.stderr,
        )
        return 2
    try:
        report = audit_run(args.run_dir, manifest_path=args.manifest_path)
        text = _write_exclusive(args.out, report)
    except (OSError, ValueError, json.JSONDecodeError, csv.Error) as exc:
        print(f"validation=FAIL: {exc}", file=sys.stderr)
        return 2
    print(text, end="")
    return 0 if report["status"] == "PASS" else 1

if __name__ == "__main__":
    raise SystemExit(main())
