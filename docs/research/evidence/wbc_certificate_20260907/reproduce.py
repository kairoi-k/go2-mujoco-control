#!/usr/bin/env python3
"""Read-only reproduction packet for the registered flat WBC certificate run."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any
PACKET = Path(__file__).resolve().parent
REPO = PACKET.parents[3]
DEFAULT_RUN = Path(
    "example/cpp/experiments/_runs/"
    "wbc_cert_flat_71d242a_20260907_0001"
)
DEFAULT_BINDING = PACKET / "pre_run_binding.json"
CERTIFICATE_SCRIPT = Path("example/cpp/tools/analysis/audit_wbc_certificate.py")
CYCLE_SCRIPT = Path("example/cpp/tools/analysis/audit_running_cycle_truth.py")
RUNTIME_SHA = "71d242a80e84356cf9dd786830a01ff425a1a838"

def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def rel(path: Path) -> str:
    return str(path.resolve().relative_to(REPO.resolve()))

def load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)

def finite_number(row: dict[str, str], field: str) -> float | None:
    value = row.get(field)
    if value is None or value.strip() == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None

def strict_flag(row: dict[str, str], field: str) -> bool | None:
    value = finite_number(row, field)
    if value is None or value not in (0.0, 1.0):
        return None
    return value == 1.0

def active(row: dict[str, str]) -> bool:
    stage = finite_number(row, "motion_stage")
    command = finite_number(row, "velocity_command_active")
    return (
        stage is not None
        and stage.is_integer()
        and int(stage) in (2, 3)
        and command == 1.0
    )

def worst_swing_row(data_path: Path) -> dict[str, Any]:
    candidates: list[tuple[float, int, dict[str, str]]] = []
    with data_path.open(newline="", encoding="utf-8") as stream:
        for index, row in enumerate(csv.DictReader(stream), start=1):
            if not active(row):
                continue
            swing = finite_number(
                row, "wbc_full_cert_selected_swing_violation_n"
            )
            if swing is not None:
                candidates.append((swing, index, row))
    if not candidates:
        return {
            "status": "UNKNOWN",
            "reason": "no active row has a finite selected swing violation",
        }
    swing, index, row = max(candidates, key=lambda item: (item[0], -item[1]))
    numeric_fields = (
        "motion_stage",
        "velocity_command_active",
        "state_tick_s",
        "cmd_time_s",
        "telemetry_running_time_s",
        "telemetry_gait_time_s",
        "velocity_command_gait_period_s",
        "velocity_command_gait_duty",
    )
    return {
        "status": "OBSERVED",
        "row_index": index,
        "selected_swing_violation_n": swing,
        "state": {
            field: finite_number(row, field)
            for field in numeric_fields
            if field in row
        },
        "period_s": finite_number(row, "velocity_command_gait_period_s"),
        "duty": finite_number(row, "velocity_command_gait_duty"),
        "legacy": {
            "wbc_full_id_ok": strict_flag(row, "wbc_full_id_ok"),
            "raw_wbc_full_id_ok": row.get("wbc_full_id_ok"),
        },
        "converged": {
            "wbc_full_id_attempt_qp_converged": strict_flag(
                row, "wbc_full_id_attempt_qp_converged"
            ),
            "raw_wbc_full_id_attempt_qp_converged": row.get(
                "wbc_full_id_attempt_qp_converged"
            ),
        },
    }

def run_analyzer(
    script: Path,
    run_rel: Path,
    output: Path,
    extra: list[str],
    expected_exit: int | None,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(script),
        str(run_rel),
        *extra,
        "--out",
        str(output),
    ]
    process = subprocess.run(
        command,
        cwd=REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    if not output.is_file():
        raise RuntimeError(
            f"analyzer did not create its exclusive output: {output}"
        )
    report = load_json(output)
    if expected_exit is not None and process.returncode != expected_exit:
        raise RuntimeError(
            f"unexpected analyzer exit {process.returncode}, expected {expected_exit}"
        )
    return {
        "output": output.name,
        "sha256": digest(output),
        "returncode": process.returncode,
        "json": report,
    }

def failure_family_summary(report: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for prefix in ("attempt", "selected"):
        source = report["certificates"][prefix]["failure_category_coverage"]
        result[prefix] = {
            name: {
                "bit": values["bit"],
                "rows": values["rows"],
                "coverage": values["coverage"],
            }
            for name, values in source.items()
            if isinstance(values, dict) and "bit" in values
        }
    return result

def raw_file_hashes(run_dir: Path) -> dict[str, dict[str, Any]]:
    files: dict[str, dict[str, Any]] = {}
    for path in sorted(run_dir.rglob("*")):
        if path.is_file():
            files[rel(path)] = {
                "bytes": path.stat().st_size,
                "sha256": digest(path),
            }
    return files

def git_blob_sha256(path: str) -> str | None:
    result = subprocess.run(
        ["git", "-C", str(REPO), "show", f"{RUNTIME_SHA}:{path}"],
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        return None
    return hashlib.sha256(result.stdout).hexdigest()

def manifest_binding(
    manifest: dict[str, Any], binding: dict[str, Any]
) -> dict[str, Any]:
    repository = manifest.get("repository")
    if not isinstance(repository, dict):
        raise RuntimeError("manifest repository metadata is missing")
    manifest_commit = repository.get("git_commit")
    manifest_dirty = repository.get("git_dirty")
    repository_clean = (
        manifest_dirty is False
        or (isinstance(manifest_dirty, str) and manifest_dirty.lower() == "false")
    )
    if manifest_commit != RUNTIME_SHA or not repository_clean:
        raise RuntimeError("manifest runtime SHA or clean flag is not verified")
    source_binding = binding.get("runtime_source_sha256")
    if not isinstance(source_binding, dict) or not source_binding:
        raise RuntimeError("pre-run source binding is missing")
    source_checks: dict[str, dict[str, Any]] = {}
    for path, expected in sorted(source_binding.items()):
        valid_expected = (
            isinstance(path, str)
            and isinstance(expected, str)
            and len(expected) == 64
            and all(char in "0123456789abcdef" for char in expected)
        )
        actual = git_blob_sha256(path) if valid_expected else None
        source_checks[path] = {
            "expected": expected,
            "git_show_sha256": actual,
            "match": valid_expected and actual == expected,
        }
    if not all(item["match"] for item in source_checks.values()):
        raise RuntimeError("pre-run source binding does not match runtime commit")
    artifacts = manifest.get("artifacts")
    binary_binding = binding.get("binary_sha256")
    if not isinstance(artifacts, dict) or not isinstance(binary_binding, dict):
        raise RuntimeError("manifest or pre-run binary binding is missing")
    pairs = {
        "controller_sha256": "example/cpp/build/real_trot_go2",
        "simulator_sha256": "simulate/build/unitree_mujoco",
    }
    checks: dict[str, dict[str, Any]] = {}
    for manifest_key, binding_key in pairs.items():
        manifest_value = artifacts.get(manifest_key)
        binding_value = binary_binding.get(binding_key)
        match = (
            isinstance(manifest_value, str)
            and isinstance(binding_value, str)
            and len(manifest_value) == 64
            and len(binding_value) == 64
            and manifest_value == binding_value
        )
        checks[manifest_key] = {
            "manifest": manifest_value,
            "pre_run_binding": binding_value,
            "match": match,
        }
    if not all(item["match"] for item in checks.values()):
        raise RuntimeError("manifest binary does not match pre-run binding")
    return {
        "manifest_source": {
            "run_id": manifest.get("run_id"),
            "repository": repository,
            "effective_argv": manifest.get("effective_argv", []),
            "profile": manifest.get("profile"),
            "seed": manifest.get("seed"),
        },
        "repository_verification": {
            "manifest_git_commit": manifest_commit,
            "expected_runtime_sha": RUNTIME_SHA,
            "exact_sha_match": manifest_commit == RUNTIME_SHA,
            "manifest_git_dirty": manifest_dirty,
            "clean": repository_clean,
        },
        "manifest_binaries": {
            key: artifacts.get(key) for key in sorted(pairs)
        },
        "pre_run_binding_binary_sha256": binary_binding,
        "source_hash_checks": source_checks,
        "all_source_matches": True,
        "binary_matches": checks,
        "all_binary_matches": True,
    }

def build_result(
    run_rel: Path,
    run_dir: Path,
    manifest_path: Path,
    manifest: dict[str, Any],
    binding_path: Path,
    binding: dict[str, Any],
    certificate: dict[str, Any],
    cycles: dict[str, Any],
    certificate_call: dict[str, Any],
    cycle_call: dict[str, Any],
    start: float,
    end: float,
) -> dict[str, Any]:
    return {
        "schema": "wbc-certificate-reproduction-v1",
        "runtime_sha": RUNTIME_SHA,
        "acceptance": {
            "status": "NOT_APPLICABLE",
            "b1_pass": False,
            "diagnostic_only": True,
            "physical_cone_certificate": True,
            "final_pd_included": False,
            "note": "This packet is a read-only diagnostic and is not a B1 pass; the physical-cone certificate excludes finalPD.",
        },
        "run": {
            "run_dir": str(run_rel),
            "run_id": manifest.get("run_id"),
            "repository": manifest.get("repository", {}),
            "statuses": manifest.get("statuses", {}),
            "effective_argv": manifest.get("effective_argv", []),
        },
        "binding": {
            "run_manifest": {
                "path": rel(manifest_path),
                "sha256": digest(manifest_path),
            },
            "pre_run_binding": {
                "path": rel(binding_path),
                "sha256": digest(binding_path),
                "json": binding,
            },
            "manifest_to_pre_run_binding": manifest_binding(manifest, binding),
        },
        "inputs": {"raw_files": raw_file_hashes(run_dir)},
        "scripts": {
            "reproduce": {
                "path": rel(Path(__file__)),
                "sha256": digest(Path(__file__)),
            },
            "audit_wbc_certificate": {
                "path": str(CERTIFICATE_SCRIPT),
                "sha256": digest(REPO / CERTIFICATE_SCRIPT),
            },
            "audit_running_cycle_truth": {
                "path": str(CYCLE_SCRIPT),
                "sha256": digest(REPO / CYCLE_SCRIPT),
            },
        },
        "analysis": {
            "certificate": certificate,
            "running_cycle_truth": cycles,
        },
        "diagnostics": {
            "failure_family_summary": failure_family_summary(certificate),
            "nonconverged_groups": certificate["convergence_groups"][
                "nonconverged"
            ],
            "worst_swing_row": worst_swing_row(run_dir / "data.csv"),
        },
        "reproduction": {
            "gait_interval_s": [start, end],
            "commands": [
                {
                    "program": "python3",
                    "script": str(CERTIFICATE_SCRIPT),
                    "args": [str(run_rel)],
                    "output": certificate_call["output"],
                    "returncode": certificate_call["returncode"],
                },
                {
                    "program": "python3",
                    "script": str(CYCLE_SCRIPT),
                    "args": [str(run_rel), "--start", str(start), "--end", str(end)],
                    "output": cycle_call["output"],
                    "returncode": cycle_call["returncode"],
                },
            ],
            "output_files": {
                "certificate": {
                    "name": certificate_call["output"],
                    "sha256": certificate_call["sha256"],
                },
                "running_cycle_truth": {
                    "name": cycle_call["output"],
                    "sha256": cycle_call["sha256"],
                },
            },
        },
    }

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--pre-run-binding", type=Path, default=DEFAULT_BINDING)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--start", type=float, default=17.0)
    parser.add_argument("--end", type=float, default=24.0)
    args = parser.parse_args(argv)
    run_dir = (REPO / args.run_dir).resolve() if not args.run_dir.is_absolute() else args.run_dir.resolve()
    run_rel = run_dir.relative_to(REPO.resolve())
    binding_path = args.pre_run_binding.resolve()
    manifest_path = run_dir / "run_manifest.json"
    out_dir = args.out_dir.resolve()
    if out_dir.exists():
        raise SystemExit(f"--out-dir must be new: {out_dir}")
    out_dir.mkdir(parents=False, exist_ok=False)
    if not run_dir.is_dir() or not manifest_path.is_file():
        raise SystemExit(f"missing run or run_manifest.json: {run_dir}")
    if not binding_path.is_file():
        raise SystemExit(f"missing pre-run binding: {binding_path}")
    if not math.isfinite(args.start + args.end) or args.end <= args.start:
        raise SystemExit("invalid gait interval")
    manifest = load_json(manifest_path)
    binding = load_json(binding_path)
    cert_output = out_dir / "certificate.json"
    cycle_output = out_dir / "running_cycle_truth.json"
    # The certificate analyzer returns 1 for a valid NOT_CERTIFIED diagnostic.
    cert_call = run_analyzer(
        CERTIFICATE_SCRIPT,
        run_rel,
        cert_output,
        [],
        None,
    )
    cert_report = cert_call["json"]
    cert_expected_actual = 0 if cert_report.get("status") == "PASS" else 1
    if cert_call["returncode"] != cert_expected_actual:
        raise RuntimeError("certificate analyzer status/exit mismatch")
    cycle_call = run_analyzer(
        CYCLE_SCRIPT,
        run_rel,
        cycle_output,
        ["--start", str(args.start), "--end", str(args.end)],
        0,
    )
    result = build_result(
        run_rel,
        run_dir,
        manifest_path,
        manifest,
        binding_path,
        binding,
        cert_report,
        cycle_call["json"],
        cert_call,
        cycle_call,
        args.start,
        args.end,
    )
    result_path = out_dir / "results.json"
    with result_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": result_path.name, "b1_pass": False}))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
