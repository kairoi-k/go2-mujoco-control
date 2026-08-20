#!/usr/bin/env python3
"""Run a reproducible directed transition matrix for reactive locomotion.

Every run changes only a two-line event script. Controller gains, gait
parameters, scene, and WBC/MPC code path are identical across pairs. Emergency
stop is an absorbing safety state and is a destination, not a source, by
default.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


EVENTS = (
    "emergency_stop", "obstacle_left", "obstacle_right", "turn_left",
    "turn_right", "slip", "low_friction", "impact",
)
ABSORBING_EVENTS = ("emergency_stop",)
NONTERMINAL_EVENTS = tuple(e for e in EVENTS if e not in ABSORBING_EVENTS)
EVENT_MAGNITUDES = {
    "emergency_stop": 0.0, "obstacle_left": 0.48,
    "obstacle_right": 0.48, "turn_left": 0.55, "turn_right": 0.55,
    "slip": 0.0, "low_friction": 0.0, "impact": 0.0,
}


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def slug(value: str) -> str:
    return value.replace("_", "-")


def pair_specs(include_terminal_source: bool) -> list[tuple[str, str]]:
    sources = EVENTS if include_terminal_source else NONTERMINAL_EVENTS
    return [(a, b) for a in sources for b in EVENTS if a != b]


def build_protocol(args: argparse.Namespace, repo: Path) -> dict[str, object]:
    controller = repo / "example/cpp/build/real_trot_go2"
    scene = repo / "unitree_robots/go2/scene_leg_lift_demo.xml"
    base_args = ["--wbc-full"]
    if args.enable_sensor_events:
        base_args.append("--reactive-events")
    base_args += [
        "--step-length", "0.091",
        "--period", "0.60", "--duty", "0.75", "--foot-lift", "0.020",
        "--kernel", "raibert-trot", "--raibert-velocity-gain", "0.05",
        "--raibert-max-adjustment", "0.010", "--tau-limit", "35",
        "--controller-duration", str(args.controller_duration_s), "--headless",
    ]
    return {
        "protocol_name": "go2_reactive_transition_matrix",
        "protocol_version": "1.1", "created_at": iso_now(),
        "events": list(EVENTS),
        "source_events": list(EVENTS if args.include_terminal_source else NONTERMINAL_EVENTS),
        "absorbing_events": list(ABSORBING_EVENTS),
        "pair_count": len(pair_specs(args.include_terminal_source)),
        "event_start_s": args.event_start_s,
        "event_duration_s": args.event_duration_s,
        "second_event_start_s": args.event_start_s + args.event_duration_s,
        "controller_duration_s": args.controller_duration_s,
        "post_event_window_s": args.post_event_window_s,
        "transition_tolerance_s": args.transition_tolerance_s,
        "event_magnitudes": EVENT_MAGNITUDES,
        "controller_args": base_args,
        "sensor_events_enabled": bool(args.enable_sensor_events),
        "scene": str(scene),
        "scene_sha256": sha256_file(scene) if scene.exists() else None,
        "controller": str(controller),
        "controller_sha256": sha256_file(controller) if controller.exists() else None,
        "reference_rate_limits": {
            "vx_mps_per_s": 2.50, "vy_mps_per_s": 1.50,
            "yaw_rate_radps_per_s": 2.00,
        },
        "acceptance_limits": {
            "max_abs_roll_rad": 0.25, "max_abs_pitch_rad": 0.25,
            "max_actual_velocity_jump_mps": 0.08,
            "min_contact_count": 1, "max_solver_residual": 1.0e-3,
        },
        "terminal_policy": (
            "emergency_stop is absorbing: any event may transition into it, "
            "but no transition out is required; WBC stance hold ends the run."
        ),
    }


def make_event_script(source: str, target: str, protocol: dict[str, object]) -> str:
    start = float(protocol["event_start_s"])
    duration = float(protocol["event_duration_s"])
    second_start = start + duration
    magnitudes = dict(protocol["event_magnitudes"])
    return (
        "# Reactive transition-matrix protocol v1.0\n"
        "# Adjacent events share one controller and one transition layer.\n"
        "# Times are relative to gait start; no pair-specific gains are used.\n"
        f"{start:.3f} {duration:.3f} {source} {magnitudes[source]:.6f}\n"
        f"{second_start:.3f} {duration:.3f} {target} {magnitudes[target]:.6f}\n"
    )


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def run_one(repo: Path, root: Path, protocol: dict[str, object], index: int,
            source: str, target: str, args: argparse.Namespace) -> dict[str, object]:
    run_id = f"{index:03d}_{slug(source)}__to__{slug(target)}"
    relative_experiment = f"{root.name}/runs/{run_id}"
    run_dir = root / "runs" / run_id
    script_path = root / "event_scripts" / f"{run_id}.txt"
    manifest_path = root / "manifests" / f"{run_id}.json"
    runner_log = root / "runner_logs" / f"{run_id}.log"
    run_dir.mkdir(parents=True, exist_ok=True)
    script_path.parent.mkdir(parents=True, exist_ok=True)
    script_path.write_text(make_event_script(source, target, protocol), encoding="utf-8")
    command = [
        "bash", str(repo / "example/cpp/scripts/run_trot.sh"),
        str(args.wall_timeout_s), relative_experiment,
        *list(protocol["controller_args"]), "--event-script", str(script_path),
    ]
    manifest: dict[str, object] = {
        "protocol_name": protocol["protocol_name"],
        "protocol_version": protocol["protocol_version"],
        "pair_index": index, "run_id": run_id,
        "source_event": source, "target_event": target,
        "event_script": str(script_path),
        "event_script_sha256": sha256_file(script_path),
        "run_directory": str(run_dir),
        "command": command,
        "controller_config_fingerprint": hashlib.sha256(
            json.dumps(protocol["controller_args"], sort_keys=True).encode()
        ).hexdigest(),
        "created_at": iso_now(),
    }
    write_json(manifest_path, manifest)
    if args.resume and (run_dir / "run_metadata.txt").exists():
        manifest["status"] = "resumed_existing"
        write_json(manifest_path, manifest)
        print(f"[{index:03d}] RESUME {source} -> {target}", flush=True)
        return manifest
    if args.dry_run:
        manifest["status"] = "dry_run"
        write_json(manifest_path, manifest)
        print(f"[{index:03d}] DRY-RUN {source} -> {target}", flush=True)
        return manifest
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    runner_log.parent.mkdir(parents=True, exist_ok=True)
    print(f"[{index:03d}] START {source} -> {target}", flush=True)
    attempts: list[dict[str, object]] = []
    for attempt in range(1, args.max_attempts + 1):
        if attempt > 1:
            time.sleep(args.retry_delay_s)
        domain_id = args.domain_base + index + (attempt - 1) * args.domain_stride
        attempt_command = [*command, "--domain-id", str(domain_id)]
        attempt_log = runner_log if attempt == 1 else root / "runner_logs" / (
            f"{run_id}.attempt{attempt}.log")
        with attempt_log.open("w", encoding="utf-8") as stream:
            completed = subprocess.run(
                attempt_command, cwd=repo, env=env, stdout=stream,
                stderr=subprocess.STDOUT, check=False)
        attempts.append({
            "attempt": attempt, "domain_id": domain_id,
            "returncode": completed.returncode, "log": str(attempt_log),
        })
        if completed.returncode == 0:
            break
    manifest["attempts"] = attempts
    manifest["domain_id"] = attempts[-1]["domain_id"]
    manifest["command"] = [
        *command, "--domain-id", str(attempts[-1]["domain_id"])
    ]
    manifest["returncode"] = attempts[-1]["returncode"]
    manifest["finished_at"] = iso_now()
    manifest["status"] = "completed" if manifest["returncode"] == 0 else "failed"
    write_json(manifest_path, manifest)
    print(f"[{index:03d}] {manifest['status'].upper()} {source} -> {target} "
          f"rc={manifest['returncode']} attempts={len(attempts)}", flush=True)
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=None)
    parser.add_argument("--event-start-s", type=float, default=1.5)
    parser.add_argument("--event-duration-s", type=float, default=2.0)
    parser.add_argument("--controller-duration-s", type=float, default=7.5)
    parser.add_argument("--post-event-window-s", type=float, default=1.5)
    parser.add_argument("--transition-tolerance-s", type=float, default=0.08)
    parser.add_argument("--wall-timeout-s", type=float, default=25.0)
    parser.add_argument("--domain-base", type=int, default=150)
    parser.add_argument("--domain-stride", type=int, default=49)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--retry-delay-s", type=float, default=5.0)
    parser.add_argument("--enable-sensor-events", action="store_true")
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--count", type=int, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--include-terminal-source", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo = repo_root()
    root = (args.root or repo / "example/cpp/experiments/go2_reactive_transition_matrix_2026-08-20").resolve()
    pairs = pair_specs(args.include_terminal_source)
    if not 1 <= args.start_index <= len(pairs):
        raise SystemExit(f"start-index must be in [1, {len(pairs)}]")
    if args.count is None:
        selected = pairs[args.start_index - 1:]
    else:
        if args.count <= 0:
            raise SystemExit("count must be positive")
        selected = pairs[args.start_index - 1:args.start_index - 1 + args.count]
    if args.max_attempts <= 0 or args.domain_stride <= 0:
        raise SystemExit("max-attempts and domain-stride must be positive")
    if args.domain_base < 0 or (
        args.domain_base + len(pairs) + (args.max_attempts - 1) * args.domain_stride
        > 232
    ):
        raise SystemExit("domain retry range must stay within DDS domain 0..232")
    if args.event_start_s < 0 or args.event_duration_s <= 0:
        raise SystemExit("event timing must be non-negative and duration must be positive")
    if args.controller_duration_s <= args.event_start_s + 2 * args.event_duration_s:
        raise SystemExit("controller duration must include both events")
    root.mkdir(parents=True, exist_ok=True)
    protocol = build_protocol(args, repo)
    protocol["max_attempts"] = args.max_attempts
    protocol["retry_delay_s"] = args.retry_delay_s
    protocol["domain_base"] = args.domain_base
    protocol["domain_stride"] = args.domain_stride
    protocol["selected_start_index"] = args.start_index
    protocol["selected_count"] = len(selected)
    protocol["pair_specs"] = [
        {"index": i, "source": source, "target": target}
        for i, (source, target) in enumerate(pairs, start=1)
    ]
    git_head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    matrix_manifest = {
        **protocol, "git_head": git_head, "repo": str(repo),
        "run_command": str(Path(__file__).resolve()),
        "selected_pairs": [
            {"index": args.start_index + offset, "source": source, "target": target}
            for offset, (source, target) in enumerate(selected)
        ],
    }
    write_json(root / "matrix_manifest.json", matrix_manifest)
    (root / "README.md").write_text(
        "# Reactive transition matrix\n\n"
        "Generated by `run_reactive_transition_matrix.py`. Each run changes "
        "only the two-line event script; controller gains, scene, gait parameters, "
        "and WBC/MPC code path are held constant. Emergency stop is an absorbing "
        "safety state and is only tested as a destination by default.\n",
        encoding="utf-8",
    )
    results = []
    for offset, (source, target) in enumerate(selected):
        results.append(run_one(repo, root, protocol, args.start_index + offset,
                               source, target, args))
    write_json(root / "runner_results.json", results)
    failed = [item for item in results if item.get("status") == "failed"]
    print(f"Transition matrix runner finished: {len(results) - len(failed)}/{len(results)} subprocesses returned 0.", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
