#!/usr/bin/env python3
"""Read-only replay adapter for registered active-contact flat runs."""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
ADAPTER = Path(__file__).resolve()
REPO = ADAPTER.parent.parents[3]
PREDECESSOR = REPO / "docs/research/evidence/wbc_certificate_20260907/reproduce.py"
PACKET = REPO / "docs/research/evidence/wbc_active_forces_20260907"
RUN_0001 = Path("example/cpp/experiments/_runs/wbc_active_flat_43c5f5a_20260907_0001")
RUN_0002 = Path("example/cpp/experiments/_runs/wbc_active_flat_43c5f5a_20260907_0002")
RUNTIME_SHA = "43c5f5a91e5c075a69170bc8af7d5582d1031474"
CERTIFICATE = Path("example/cpp/tools/analysis/audit_wbc_certificate.py")
CYCLES = Path("example/cpp/tools/analysis/audit_running_cycle_truth.py")

def load_predecessor():
    spec = importlib.util.spec_from_file_location("wbc_certificate_predecessor", PREDECESSOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot import predecessor replay script")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.PACKET = PACKET
    module.REPO = REPO
    module.DEFAULT_RUN = RUN_0001
    module.DEFAULT_BINDING = PACKET / "pre_run_binding.json"
    module.CERTIFICATE_SCRIPT = CERTIFICATE
    module.CYCLE_SCRIPT = CYCLES
    module.RUNTIME_SHA = RUNTIME_SHA
    module.__file__ = str(ADAPTER)
    predecessor_rel = module.rel
    def rel_any(path: Path) -> str:
        try:
            return predecessor_rel(path)
        except ValueError:
            return str(path.resolve())
    module.rel = rel_any
    return module

def run_one(old, run_arg: Path, out_dir: Path, start: float, end: float) -> dict:
    run_dir = (REPO / run_arg).resolve() if not run_arg.is_absolute() else run_arg.resolve()
    run_rel = run_dir.relative_to(REPO.resolve())
    manifest_path = run_dir / "run_manifest.json"
    binding_path = PACKET / "pre_run_binding.json"
    manifest = old.load_json(manifest_path)
    binding = old.load_json(binding_path)
    label = run_dir.name
    cert_output = out_dir / f"{label}_certificate.json"
    cycle_output = out_dir / f"{label}_running_cycle_truth.json"
    cert_call = old.run_analyzer(old.CERTIFICATE_SCRIPT, run_rel, cert_output, [], None)
    certificate = cert_call["json"]
    expected_cert_exit = 0 if certificate.get("status") == "PASS" else 1
    if cert_call["returncode"] != expected_cert_exit:
        raise RuntimeError("certificate analyzer status/exit mismatch")
    cycle_call = old.run_analyzer(
        old.CYCLE_SCRIPT,
        run_rel,
        cycle_output,
        ["--start", str(start), "--end", str(end)],
        0,
    )
    item = old.build_result(
        run_rel,
        run_dir,
        manifest_path,
        manifest,
        binding_path,
        binding,
        certificate,
        cycle_call["json"],
        cert_call,
        cycle_call,
        start,
        end,
    )
    item.pop("schema", None)
    item.pop("runtime_sha", None)
    item["scripts"]["imported_predecessor_reproduce"] = {
        "path": old.rel(PREDECESSOR),
        "sha256": old.digest(PREDECESSOR),
    }
    if label.endswith("_0001"):
        item["timing_contamination"] = {
            "status": "PRESENT",
            "concurrent_archive_wall_duration_s": 8.6,
            "simulation_interval_s": None,
            "note": "The run had approximately 8.6 s of root archival old-raw concurrent CPU/IO activity; no exact simulation interval is asserted, and validator latency is not an improvement claim.",
            "latency_improvement_claim": False,
        }
    elif label.endswith("_0002"):
        item["timing_contamination"] = {
            "status": "ABSENT",
            "concurrent_archive_wall_duration_s": 0.0,
            "simulation_interval_s": None,
            "note": "The orchestrator recorded no concurrent build/archive activity for this isolated run; validator latency remains diagnostic and is not a B1 claim.",
            "latency_improvement_claim": False,
        }
    else:
        item["timing_contamination"] = {
            "status": "UNKNOWN",
            "concurrent_archive_wall_duration_s": None,
            "simulation_interval_s": None,
            "note": "No contamination record is registered for this run name; validator latency remains diagnostic.",
            "latency_improvement_claim": False,
        }
    return item

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", action="append", type=Path, default=None)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--start", type=float, default=17.0)
    parser.add_argument("--end", type=float, default=24.0)
    args = parser.parse_args(argv)
    if not args.end > args.start:
        raise SystemExit("invalid gait interval")
    runs = args.run_dir or [RUN_0001]
    out_dir = args.out_dir.resolve()
    if out_dir.exists():
        raise SystemExit(f"--out-dir must be new: {out_dir}")
    out_dir.mkdir(parents=False, exist_ok=False)
    old = load_predecessor()
    result = {
        "schema": "wbc-active-forces-reproduction-v1",
        "runtime_sha": RUNTIME_SHA,
        "b1_pass": False,
        "diagnostic_only": True,
        "physical_cone_certificate_final_pd_included": False,
        "runs": {},
        "reproduction": {
            "gait_interval_s": [args.start, args.end],
            "run_dirs": [str(path) for path in runs],
            "predecessor_reproduce": old.rel(PREDECESSOR),
        },
    }
    for run_arg in runs:
        item = run_one(old, run_arg, out_dir, args.start, args.end)
        key = item["run"]["run_id"]
        if key in result["runs"]:
            raise SystemExit(f"duplicate run_id: {key}")
        result["runs"][key] = item
    result_path = out_dir / "results.json"
    with result_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": result_path.name, "b1_pass": False, "runs": len(result["runs"])}))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
