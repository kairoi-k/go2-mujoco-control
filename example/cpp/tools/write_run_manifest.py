#!/usr/bin/env python3
"""Create the versioned, machine-readable manifest for one simulation run."""

import argparse
import hashlib
import json
import pathlib
import shlex


def read_kv(path: pathlib.Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key] = value
    return values


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_dir", type=pathlib.Path)
    parser.add_argument("--repo", type=pathlib.Path, required=True)
    parser.add_argument("--cpp-dir", type=pathlib.Path, required=True)
    options = parser.parse_args()

    metadata_path = options.experiment_dir / "run_metadata.txt"
    environment_path = options.experiment_dir / "environment.txt"
    metadata = read_kv(metadata_path)
    environment = read_kv(environment_path) if environment_path.exists() else {}
    profile_path = pathlib.Path(metadata["profile_path"]) if metadata.get("profile_path") else None
    profile_hash = sha256(profile_path) if profile_path and profile_path.is_file() else ""
    controller_argv = shlex.split(metadata.get("controller_argv_shell", ""))
    analyzer_names = [
        "analyze_contact_ground_truth.py",
        "analyze_contact_dynamics.py",
    ]
    analyzers = {}
    for name in analyzer_names:
        analyzer_path = options.cpp_dir / "tools" / "analysis" / name
        analyzers[name] = sha256(analyzer_path) if analyzer_path.is_file() else ""
    phase2_contract_path = options.repo / "docs" / "research" / \
        "PHASE2_B0_ACCEPTANCE_CONTRACT.md"
    phase2_analyzer_path = options.cpp_dir / "tools" / "analyze_phase2_b0.py"
    phase2_terrain_contract_path = options.repo / "docs" / "research" / \
        "PHASE2_B123_ACCEPTANCE_CONTRACT.md"
    phase2_b1_contract_path = options.repo / "docs" / "research" / \
        "PHASE2_B1_ACCEPTANCE_CONTRACT.md"
    phase2_terrain_analyzer_path = options.cpp_dir / "tools" / \
        "analyze_phase2_terrain.py"
    sustained_analyzer_path = options.cpp_dir / "tools" / "analysis" / \
        "analyze_sustained_running.py"

    manifest = {
        "schema_version": 1,
        "run_id": options.experiment_dir.name,
        "timestamps": {
            "started_at": metadata.get("started_at", ""),
            "finished_at": metadata.get("finished_at", ""),
        },
        "repository": {
            "git_commit": metadata.get("git_head", ""),
            "git_branch": metadata.get("git_branch", ""),
            "git_dirty": metadata.get("git_dirty", ""),
        },
        "profile": {
            "path": metadata.get("profile_path", ""),
            "sha256": profile_hash,
        },
        "effective_argv": controller_argv,
        "semantic_env_snapshot": environment,
        "seed": metadata.get("seed", ""),
        "artifacts": {
            "simulator_sha256": metadata.get("simulator_sha256", ""),
            "controller_sha256": metadata.get("controller_sha256", ""),
            "scenario_sha256": metadata.get("scene_sha256", ""),
            "event_script_sha256": metadata.get("event_script_sha256", ""),
            "phase2_b0_contract_sha256": sha256(phase2_contract_path)
            if phase2_contract_path.is_file() else "",
            "phase2_b0_analyzer_sha256": sha256(phase2_analyzer_path)
            if phase2_analyzer_path.is_file() else "",
            "phase2_b123_contract_sha256": sha256(phase2_terrain_contract_path)
            if phase2_terrain_contract_path.is_file() else "",
            "phase2_b1_contract_sha256": sha256(phase2_b1_contract_path)
            if phase2_b1_contract_path.is_file() else "",
            "phase2_b123_analyzer_sha256": sha256(phase2_terrain_analyzer_path)
            if phase2_terrain_analyzer_path.is_file() else "",
            "phase2_fixed_3mps_analyzer_sha256": sha256(sustained_analyzer_path)
            if sustained_analyzer_path.is_file() else "",
            "phase2_contract_sha256": sha256(pathlib.Path(environment["TROT_PHASE2_CONTRACT"]))
            if environment.get("TROT_PHASE2_CONTRACT") and
            pathlib.Path(environment["TROT_PHASE2_CONTRACT"]).is_file() else "",
            "phase2_analyzer_sha256": sha256(pathlib.Path(environment["TROT_PHASE2_ANALYZER"]))
            if environment.get("TROT_PHASE2_ANALYZER") and
            pathlib.Path(environment["TROT_PHASE2_ANALYZER"]).is_file() else "",
        },
        "analyzers": analyzers,
        "statuses": {
            key: metadata.get(key, "")
            for key in (
                "controller_status",
                "safety_status",
                "quality_status",
                "analysis_status",
                "ground_truth_status",
                "dynamics_status",
                "completion_status",
                "phase1_quantitative_status",
                "terrain_analysis_status",
            )
        },
        # C-000 publishes schema/provenance only.  Timing remains disabled
        # until a later order explicitly enables a terrain execution path.
        "terrain_timing": {
            "has_stage_c_timing": False,
            "timing_provenance": "none",
            "period_s": None,
            "duty_factor": None,
            "window_start_s": None,
            "window_end_s": None,
        },
        "run": {
            "domain_id": metadata.get("domain_id", ""),
            "controller_duration_s": metadata.get("controller_duration_s", ""),
            "wall_timeout_s": metadata.get("wall_timeout_s", ""),
            "run_mode": metadata.get("run_mode", ""),
            "headless": metadata.get("headless", ""),
            "phase2_milestone": metadata.get("phase2_milestone", ""),
            "scene_file": metadata.get("scene_file", ""),
            "initial_x_m": environment.get("TROT_INITIAL_X_M", metadata.get("initial_x_m", "0.0")),
            "initial_y_m": environment.get("TROT_INITIAL_Y_M", metadata.get("initial_y_m", "0.0")),
        },
    }
    output_path = options.experiment_dir / "run_manifest.json"
    output_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
