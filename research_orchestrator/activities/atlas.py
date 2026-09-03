"""Allowlisted Atlas activities for bounded, provenance-first experiments.

The Atlas worker is the only place where a physical MuJoCo/controller process
may run. Requests are structured experiment specs; no request can provide a
shell command, executable path, environment assignment, or arbitrary argv.
Formal campaigns are autonomous, but remain fail-closed on the source-pinned
manifest, analyzer, prerequisite chain, and process lifecycle.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, NoReturn

from temporalio import activity
from temporalio.exceptions import ApplicationError

from .local_llm import local_llm_is_active
from ..schemas.models import (
    ActivityReceipt,
    ArtifactRef,
    CampaignMember,
    CampaignReceipt,
    EvidencePoint,
    ExperimentSpec,
    FailureWindow,
    Result,
    ResultMetrics,
    RunStatus,
    Verdict,
)


EXPECTED_REPOSITORY = "kairoi-k/go2-mujoco-control"
ATLAS_ACTIVITY_NAMES = (
    "build_source",
    "run_unit_tests",
    "run_dev_probe",
    "run_b0_member",
    "run_b0_holdout",
    "run_b1_probe",
    "extract_failure_window",
    "diagnose_with_local_llm",
)

# These are the existing reviewed velocity profiles. Their durations are part
# of the runner contract; an arbitrary duration would turn a reproducible
# profile into an under-specified partial run.
DEV_SCENARIO_DURATIONS: dict[str, float] = {
    "steps": 96.0,
    "accel_1_to_3": 40.0,
    "brake_3_to_0": 44.0,
    "ramp": 62.0,
    "varying": 86.0,
}

_MAX_ARTIFACT_BYTES = 256 * 1024 * 1024
_MAX_ARTIFACT_TOTAL_BYTES = 512 * 1024 * 1024
_MAX_LOG_READ_BYTES = 8 * 1024 * 1024
_SAFE_DOMAIN_RANGE = range(190, 200)

B0_HOLDOUT_PROFILES = ("steps", "accel_1_to_3", "brake_3_to_0", "ramp", "varying")
B0_HOLDOUT_REPEATS = ((1, 200, 180), (2, 201, 181), (3, 202, 182))
B0_FIXED_HOLDOUT_REPEATS = ((1, 203, 183), (2, 204, 184), (3, 205, 185))
B0_CONTRACT = "b0-contract-v1.2"
B1_CONTRACT = "b1-contract-v1.0"
B1_PROFILE = "example/cpp/configs/phase1_velocity_steps.csv"
B1_DURATION_S = 96.0
B1_CASES = (
    {
        "case": "dev_centered",
        "scene": "unitree_robots/go2/phase2_step_5cm.xml",
        "initial_x_m": 0.0,
        "initial_y_m": 0.0,
        "gait_phase_offset": 0.0,
        "seed": 11,
        "domain_id": 209,
    },
    {
        "case": "holdout_a",
        "scene": "unitree_robots/go2/phase2_step_5cm_holdout_a.xml",
        "obstacle_center_x_m": 0.85,
        "obstacle_center_y_m": 0.10,
        "initial_x_m": -0.10,
        "initial_y_m": 0.0,
        "gait_phase_offset": 0.17,
        "seed": 101,
        "domain_id": 210,
    },
    {
        "case": "holdout_b",
        "scene": "unitree_robots/go2/phase2_step_5cm_holdout_b.xml",
        "obstacle_center_x_m": 1.05,
        "obstacle_center_y_m": -0.12,
        "initial_x_m": 0.08,
        "initial_y_m": 0.0,
        "gait_phase_offset": 0.41,
        "seed": 102,
        "domain_id": 211,
    },
    {
        "case": "holdout_c",
        "scene": "unitree_robots/go2/phase2_step_5cm_holdout_c.xml",
        "obstacle_center_x_m": 0.95,
        "obstacle_center_y_m": 0.16,
        "initial_x_m": -0.04,
        "initial_y_m": 0.0,
        "gait_phase_offset": 0.68,
        "seed": 103,
        "domain_id": 212,
    },
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _spec(payload: Any) -> ExperimentSpec:
    candidate = payload.get("spec", payload) if isinstance(payload, dict) else payload
    try:
        return ExperimentSpec.model_validate(candidate)
    except Exception as exc:  # Do not expose the whole validation payload over the wire.
        raise ApplicationError(
            "Atlas Activity payload does not contain a valid experiment.v1 spec",
            type="INVALID_ATLAS_PAYLOAD",
            non_retryable=True,
        ) from exc


def _invalid(message: str, error_type: str = "ATLAS_CONFIGURATION_INVALID") -> NoReturn:
    raise ApplicationError(message, type=error_type, non_retryable=True)


def _workspace() -> Path:
    raw = os.environ.get("ATLAS_WORKSPACE", "").strip()
    if not raw:
        _invalid("ATLAS_WORKSPACE is required", "ATLAS_WORKSPACE_MISSING")
    path = Path(raw).expanduser().resolve()
    if not path.is_dir() or not (path / ".git").exists():
        _invalid("ATLAS_WORKSPACE must be an existing Git checkout", "ATLAS_WORKSPACE_INVALID")
    return path


def _artifact_root(workspace: Path) -> Path:
    configured = os.environ.get("ATLAS_ARTIFACT_ROOT", "").strip()
    root = (
        Path(configured).expanduser().resolve()
        if configured
        else (workspace / ".orchestration" / "artifacts").resolve()
    )
    if root in {Path("/"), Path.home().resolve()} or root == workspace:
        _invalid("ATLAS_ARTIFACT_ROOT is too broad or aliases the source checkout")
    root.mkdir(parents=True, exist_ok=True)
    return root


def _git_output(workspace: Path, *args: str) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", str(workspace), *args],
            check=True,
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        raise ApplicationError(
            "Atlas Git state could not be read",
            type="ATLAS_SOURCE_STATE_UNAVAILABLE",
            non_retryable=True,
        ) from exc
    return completed.stdout.strip()


def _git_state(workspace: Path) -> tuple[str, bool]:
    head = _git_output(workspace, "rev-parse", "HEAD")
    dirty = bool(_git_output(workspace, "status", "--porcelain"))
    return head, dirty


def _ensure_source(spec: ExperimentSpec) -> Path:
    if spec.source.repository != EXPECTED_REPOSITORY:
        _invalid(
            f"unexpected Atlas source repository {spec.source.repository!r}",
            "ATLAS_SOURCE_REPOSITORY_MISMATCH",
        )
    workspace = _workspace()
    if local_llm_is_active():
        _invalid(
            "Atlas physical Activities refuse to run while the reserved local LLM port is active",
            "ATLAS_LOCAL_LLM_ACTIVE",
        )
    head, dirty = _git_state(workspace)
    if head != spec.source.git_sha:
        _invalid(
            "Atlas checkout HEAD does not match experiment source.git_sha",
            "ATLAS_SOURCE_SHA_MISMATCH",
        )
    if dirty or spec.source.dirty:
        _invalid(
            "physical Atlas activities require a clean source checkout",
            "ATLAS_SOURCE_DIRTY",
        )
    return workspace


def _ensure_mujoco_link(workspace: Path) -> Path:
    configured = os.environ.get("ATLAS_MUJOCO_ROOT", "").strip()
    target = (
        Path(configured).expanduser().resolve()
        if configured
        else (Path.home() / ".mujoco" / "mujoco-3.3.6").resolve()
    )
    if not target.is_dir():
        _invalid(f"MuJoCo root is missing: {target}", "ATLAS_MUJOCO_MISSING")
    link = workspace / "simulate" / "mujoco"
    if link.exists() or link.is_symlink():
        if not link.is_symlink() or link.resolve() != target:
            _invalid(
                "simulate/mujoco exists but does not point to the configured MuJoCo root",
                "ATLAS_MUJOCO_LINK_MISMATCH",
            )
    else:
        link.symlink_to(target, target_is_directory=True)
    return target


def _kind(path: Path) -> str:
    if path.suffix == ".json":
        return "json"
    if path.suffix == ".csv":
        return "csv"
    if path.suffix in {".log", ".txt"}:
        return "log"
    if path.suffix in {".trace", ".dat"}:
        return "trace"
    if path.suffix in {".so", ".bin", ".exe"}:
        return "binary"
    return "other"


def _artifact_ref(path: Path, root: Path) -> ArtifactRef:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        _invalid("artifact escaped ATLAS_ARTIFACT_ROOT", "ATLAS_ARTIFACT_PATH_INVALID")
    if not relative or any(part in {"", ".", ".."} for part in relative.split("/")):
        _invalid("artifact path is not normalized", "ATLAS_ARTIFACT_PATH_INVALID")
    return ArtifactRef(
        kind=_kind(resolved),
        relative_path=relative,
        sha256=hashlib.sha256(resolved.read_bytes()).hexdigest(),
        size_bytes=resolved.stat().st_size,
    )


def _copy_file(source: Path, target: Path, root: Path) -> ArtifactRef | None:
    if not source.is_file() or source.is_symlink():
        return None
    size = source.stat().st_size
    if size > _MAX_ARTIFACT_BYTES:
        return None
    if source.resolve() == target.resolve():
        return _artifact_ref(source, root)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return _artifact_ref(target, root)


def _copy_run_artifacts(
    run_dir: Path,
    process_log: Path,
    root: Path,
    experiment_id: str,
    artifact_subdir: str = "run",
) -> list[ArtifactRef]:
    if not artifact_subdir or artifact_subdir.startswith("/") or ".." in artifact_subdir.split("/"):
        _invalid("artifact_subdir is not normalized", "ATLAS_ARTIFACT_PATH_INVALID")
    artifact_dir = root / experiment_id / artifact_subdir
    refs: list[ArtifactRef] = []
    total_bytes = 0
    allowed_suffixes = {".csv", ".json", ".log", ".txt", ".trace", ".yaml"}
    if run_dir.is_dir():
        for source in sorted(run_dir.rglob("*")):
            if not source.is_file() or source.is_symlink() or source.suffix not in allowed_suffixes:
                continue
            size = source.stat().st_size
            if size > _MAX_ARTIFACT_BYTES or total_bytes + size > _MAX_ARTIFACT_TOTAL_BYTES:
                continue
            ref = _copy_file(source, artifact_dir / source.relative_to(run_dir), root)
            if ref is not None:
                refs.append(ref)
                total_bytes += size
    process_target = (
        root / experiment_id / "atlas_process.log"
        if artifact_subdir == "run"
        else artifact_dir / "atlas_process.log"
    )
    process_ref = _copy_file(process_log, process_target, root)
    if process_ref is not None:
        refs.append(process_ref)
    return refs


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _receipt(
    spec: ExperimentSpec,
    activity_name: str,
    status: str,
    message: str,
    root: Path,
    artifacts: list[ArtifactRef],
) -> dict[str, Any]:
    value = ActivityReceipt(
        activity_name=activity_name,
        experiment_id=spec.experiment_id,
        status=status,
        source=spec.source,
        message=message,
        artifacts=artifacts,
    )
    receipt_path = root / spec.experiment_id / "preflight" / f"{activity_name}_receipt.json"
    _write_json(receipt_path, value.model_dump(mode="json"))
    receipt_artifact = _artifact_ref(receipt_path, root)
    final = ActivityReceipt(
        activity_name=activity_name,
        experiment_id=spec.experiment_id,
        status=status,
        source=spec.source,
        message=message,
        artifacts=[*artifacts, receipt_artifact],
    )
    return final.model_dump(mode="json")


async def _run_fixed_command(
    argv: list[str],
    cwd: Path,
    env: dict[str, str],
    log_path: Path,
    timeout_s: float,
    heartbeat_name: str,
) -> int | None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("wb") as log_file:
        try:
            process = await asyncio.create_subprocess_exec(
                *argv,
                cwd=str(cwd),
                env=env,
                stdout=log_file,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as exc:
            log_file.write(f"failed to start fixed command: {exc}\n".encode())
            return None

        deadline = asyncio.get_running_loop().time() + timeout_s
        try:
            while True:
                remaining = deadline - asyncio.get_running_loop().time()
                if remaining <= 0:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        await asyncio.wait_for(process.wait(), timeout=10)
                    except asyncio.TimeoutError:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        await process.wait()
                    return None
                try:
                    returncode = await asyncio.wait_for(process.wait(), timeout=min(5.0, remaining))
                    return returncode
                except asyncio.TimeoutError:
                    activity.heartbeat({"activity": heartbeat_name, "pid": process.pid})
        except asyncio.CancelledError:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(process.wait(), timeout=10)
            except asyncio.TimeoutError:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await process.wait()
            raise


def _fixed_env(spec: ExperimentSpec) -> dict[str, str]:
    env = os.environ.copy()
    # Atlas never needs model/API credentials.
    env.pop("OPENAI_API_KEY", None)
    env.pop("OPENAI_BASE_URL", None)
    env.update(
        {
            "TROT_CPU_AUTOPIN": "1",
            "TROT_DYNAMICS_TOLERANCE_N": "20",
            "TROT_HS_START_PERIOD": "0.20",
            "TROT_HS_START_DUTY": "0.50",
            "TROT_HS_SPEED_LEAD": "0.25",
            "TROT_HS_ACC_GAIN": "10",
            "TROT_HS_ACC_LIMIT": "4",
            "TROT_HS_STEP_CAP": "0.52",
            "TROT_HS_SWING_REACH": "0.90",
            "TROT_HS_HYBRID_CONTACT": "2",
            "TROT_HS_PITCH_GAIN": "24",
            "TROT_HS_PITCH_DAMP": "6",
            "TROT_HS_ROLL_GAIN": "20",
            "TROT_HS_ROLL_DAMP": "10",
            "TROT_HS_STABILITY_GOV": "1",
            "TROT_SEED": str(spec.seed),
        }
    )
    return env


async def _preflight_commands(
    spec: ExperimentSpec,
    workspace: Path,
    root: Path,
    activity_name: str,
    commands: list[tuple[str, list[str]]],
    timeout_s: float,
) -> dict[str, Any]:
    artifact_dir = root / spec.experiment_id / "preflight"
    env = _fixed_env(spec)
    refs: list[ArtifactRef] = []
    statuses: list[int | None] = []
    for label, argv in commands:
        log_path = artifact_dir / f"{label}.log"
        status = await _run_fixed_command(argv, workspace, env, log_path, timeout_s, activity_name)
        statuses.append(status)
        refs.append(_artifact_ref(log_path, root))
        if status != 0:
            break
    ok = bool(statuses) and all(status == 0 for status in statuses)
    message = (
        f"{activity_name} completed using {len(commands)} fixed command(s)"
        if ok
        else f"{activity_name} failed; inspect the preserved preflight logs"
    )
    return _receipt(spec, activity_name, "completed" if ok else "failed", message, root, refs)


@activity.defn(name="build_source")
async def build_source(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    workspace = _ensure_source(spec)
    _ensure_mujoco_link(workspace)
    cmake = shutil.which("cmake") or "/usr/bin/cmake"
    parallel = str(max(1, min(os.cpu_count() or 1, 4)))
    commands = [
        (
            "build_simulator",
            [
                cmake,
                "-S",
                str(workspace / "simulate"),
                "-B",
                str(workspace / "simulate" / "build"),
                "-DCMAKE_BUILD_TYPE=Release",
            ],
        ),
        (
            "compile_simulator",
            [cmake, "--build", str(workspace / "simulate" / "build"), "--parallel", parallel],
        ),
        (
            "build_controller",
            [
                cmake,
                "-S",
                str(workspace / "example" / "cpp"),
                "-B",
                str(workspace / "example" / "cpp" / "build"),
                "-DCMAKE_BUILD_TYPE=Release",
            ],
        ),
        (
            "compile_controller",
            [cmake, "--build", str(workspace / "example" / "cpp" / "build"), "--parallel", parallel],
        ),
    ]
    root = _artifact_root(workspace)
    receipt = await _preflight_commands(spec, workspace, root, "build_source", commands, 900.0)
    if receipt["status"] != "completed":
        return receipt

    controller = workspace / "example" / "cpp" / "build" / "real_trot_go2"
    if not controller.is_file():
        return _receipt(
            spec,
            "build_source",
            "failed",
            "build completed without producing the required controller binary",
            root,
            [],
        )
    help_result = subprocess.run(
        [str(controller)],
        cwd=str(workspace),
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    if "--terrain-sensor-only" not in (help_result.stdout + help_result.stderr):
        _invalid(
            "Atlas source/controller does not expose the reviewed sensor-only B0 entry point",
            "ATLAS_SOURCE_UNSUPPORTED",
        )
    return receipt


@activity.defn(name="run_unit_tests")
async def run_unit_tests(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    workspace = _ensure_source(spec)
    root = _artifact_root(workspace)
    ctest = shutil.which("ctest") or "/usr/bin/ctest"
    commands = [
        (
            "ctest_controller",
            [
                ctest,
                "--test-dir",
                str(workspace / "example" / "cpp" / "build"),
                "--output-on-failure",
            ],
        ),
        (
            "ctest_simulator",
            [
                ctest,
                "--test-dir",
                str(workspace / "simulate" / "build"),
                "--output-on-failure",
            ],
        ),
    ]
    return await _preflight_commands(spec, workspace, root, "run_unit_tests", commands, 900.0)


def _scenario(spec: ExperimentSpec) -> tuple[str, float, int]:
    raw_scenario = spec.parameters.get("scenario", "accel_1_to_3")
    if not isinstance(raw_scenario, str) or raw_scenario not in DEV_SCENARIO_DURATIONS:
        _invalid("scenario is not in the fixed development profile map", "ATLAS_SCENARIO_INVALID")
    expected_duration = DEV_SCENARIO_DURATIONS[raw_scenario]
    if abs(spec.duration_s - expected_duration) > 1e-6:
        _invalid(
            f"duration_s must equal the fixed {raw_scenario} profile duration {expected_duration:g}s",
            "ATLAS_DURATION_INVALID",
        )
    raw_domain = spec.parameters.get("domain_id", 190)
    if (
        isinstance(raw_domain, bool)
        or not isinstance(raw_domain, int)
        or raw_domain not in _SAFE_DOMAIN_RANGE
    ):
        _invalid("development domain_id must be an integer in 190..199", "ATLAS_DOMAIN_INVALID")
    if spec.wall_timeout_s < spec.duration_s + 30.0:
        _invalid(
            "wall_timeout_s must include the fixed 30s startup/teardown margin",
            "ATLAS_TIMEOUT_INVALID",
        )
    return raw_scenario, expected_duration, raw_domain


def _read_text(path: Path) -> str:
    if not path.is_file():
        return ""
    with path.open("rb") as handle:
        data = handle.read(_MAX_LOG_READ_BYTES)
    return data.decode("utf-8", errors="replace")


def _manifest(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _require_autonomous(spec: ExperimentSpec) -> None:
    if not spec.autonomous:
        _invalid(
            "formal Atlas campaigns are available only through autonomous mode",
            "ATLAS_AUTONOMOUS_MODE_REQUIRED",
        )


def _load_contract_manifest(
    workspace: Path, stage: str
) -> tuple[Path, dict[str, Any], str]:
    if stage == "b0":
        relative = Path("docs/research/PHASE2_B0_HOLDOUT_MANIFEST.json")
        expected_contract = B0_CONTRACT
    elif stage == "b1":
        relative = Path("docs/research/PHASE2_B1_HOLDOUT_MANIFEST.json")
        expected_contract = B1_CONTRACT
    else:
        _invalid("unknown formal campaign stage", "ATLAS_CAMPAIGN_INVALID")
    path = workspace / relative
    manifest = _manifest(path)
    if manifest is None or manifest.get("contract") != expected_contract:
        _invalid(
            f"{stage.upper()} source manifest is missing or has an unexpected contract",
            "ATLAS_CONTRACT_INVALID",
        )
    return path, manifest, hashlib.sha256(path.read_bytes()).hexdigest()


def _validate_b0_manifest(manifest: dict[str, Any]) -> None:
    if tuple(manifest.get("profiles", ())) != B0_HOLDOUT_PROFILES:
        _invalid("B0 profile membership differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    repeats = manifest.get("repeats")
    expected_repeats = [
        {"repeat": repeat, "terrain_domain": terrain, "baseline_domain": baseline}
        for repeat, terrain, baseline in B0_HOLDOUT_REPEATS
    ]
    if repeats != expected_repeats:
        _invalid("B0 repeat/domain membership differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    fixed = manifest.get("fixed_3mps_repeats")
    expected_fixed = [
        {"repeat": repeat, "terrain_domain": terrain, "baseline_domain": baseline}
        for repeat, terrain, baseline in B0_FIXED_HOLDOUT_REPEATS
    ]
    if fixed != expected_fixed:
        _invalid("B0 fixed-speed membership differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    if manifest.get("terrain_mode") != "sensor_only":
        _invalid("B0 autonomous execution requires sensor_only terrain mode", "ATLAS_CONTRACT_INVALID")


def _validate_b1_manifest(manifest: dict[str, Any]) -> None:
    if manifest.get("terrain_mode") != "planner_actuation":
        _invalid("B1 autonomous execution requires planner_actuation terrain mode", "ATLAS_CONTRACT_INVALID")
    if manifest.get("profile") != B1_PROFILE:
        _invalid("B1 profile differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    development = manifest.get("development")
    if not isinstance(development, list) or len(development) != 1:
        _invalid("B1 development membership differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    expected_development = {
        "case": "dev_centered",
        "scene": "unitree_robots/go2/phase2_step_5cm.xml",
        "initial_x_m": 0.0,
        "initial_y_m": 0.0,
        "gait_phase_offset": 0.0,
        "seed": 11,
    }
    if any(development[0].get(key) != value for key, value in expected_development.items()):
        _invalid("B1 development case differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    holdout = manifest.get("holdout")
    expected_holdout = (
        ("holdout_a", "unitree_robots/go2/phase2_step_5cm_holdout_a.xml", 0.85, 0.10, -0.10, 0.0, 0.17, 101, 210),
        ("holdout_b", "unitree_robots/go2/phase2_step_5cm_holdout_b.xml", 1.05, -0.12, 0.08, 0.0, 0.41, 102, 211),
        ("holdout_c", "unitree_robots/go2/phase2_step_5cm_holdout_c.xml", 0.95, 0.16, -0.04, 0.0, 0.68, 103, 212),
    )
    if not isinstance(holdout, list) or len(holdout) != len(expected_holdout):
        _invalid("B1 holdout membership differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")
    for item, expected in zip(holdout, expected_holdout):
        keys = ("case", "scene", "obstacle_center_x_m", "obstacle_center_y_m", "initial_x_m", "initial_y_m", "gait_phase_offset", "seed", "domain_id")
        values = tuple(item.get(key) for key in keys)
        if values != expected:
            _invalid("B1 holdout case differs from the frozen manifest", "ATLAS_CONTRACT_INVALID")


def _copy_contract_manifest(
    manifest_path: Path, root: Path, spec: ExperimentSpec, stage: str
) -> ArtifactRef:
    target = root / spec.experiment_id / "formal" / stage / "contracts" / manifest_path.name
    reference = _copy_file(manifest_path, target, root)
    if reference is None:
        _invalid("formal source manifest could not be preserved", "ATLAS_CONTRACT_INVALID")
    return reference


def _b0_holdout_members(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    _validate_b0_manifest(manifest)
    members: list[dict[str, Any]] = []
    for scenario in B0_HOLDOUT_PROFILES:
        for repeat, terrain_domain, baseline_domain in B0_HOLDOUT_REPEATS:
            members.append(
                {
                    "member_id": f"{scenario}-r{repeat}",
                    "case": scenario,
                    "scenario": scenario,
                    "set_name": "holdout",
                    "repeat": repeat,
                    "terrain_domain": terrain_domain,
                    "baseline_domain": baseline_domain,
                    "fixed_3mps": False,
                }
            )
    for repeat, terrain_domain, baseline_domain in B0_FIXED_HOLDOUT_REPEATS:
        members.append(
            {
                "member_id": f"fixed-3mps-r{repeat}",
                "case": "fixed_3mps",
                "scenario": "fixed_3mps",
                "set_name": "holdout",
                "repeat": repeat,
                "terrain_domain": terrain_domain,
                "baseline_domain": baseline_domain,
                "fixed_3mps": True,
            }
        )
    return members


def _b0_development_member(spec: ExperimentSpec) -> dict[str, Any]:
    scenario, _, domain_id = _scenario(spec)
    return {
        "member_id": f"development-{scenario}",
        "case": scenario,
        "scenario": scenario,
        "set_name": "development",
        "repeat": 0,
        "terrain_domain": domain_id,
        "baseline_domain": 220,
        "fixed_3mps": False,
    }


def _b1_members(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    _validate_b1_manifest(manifest)
    return [
        {
            "member_id": item["case"],
            "case": item["case"],
            "scene": item["scene"],
            "initial_x_m": item["initial_x_m"],
            "initial_y_m": item["initial_y_m"],
            "gait_phase_offset": item["gait_phase_offset"],
            "seed": item["seed"],
            "terrain_domain": item["domain_id"],
        }
        for item in (
            {
                "case": "dev_centered",
                "scene": "unitree_robots/go2/phase2_step_5cm.xml",
                "initial_x_m": 0.0,
                "initial_y_m": 0.0,
                "gait_phase_offset": 0.0,
                "seed": 11,
                "domain_id": 209,
            },
            *B1_CASES[1:],
        )
    ]


def _new_run_dirs(parent: Path, prefix: str, before: set[str]) -> list[Path]:
    return sorted(
        [path for path in parent.glob(prefix + "*") if path.is_dir() and path.name not in before],
        key=lambda path: path.stat().st_mtime_ns,
    )


def _analyzer_failure_reasons(
    analyzer: dict[str, Any] | None, prefix: str
) -> tuple[str, list[str]]:
    if analyzer is None:
        return "UNAVAILABLE", [f"{prefix}_analyzer_missing"]
    acceptance = str(analyzer.get("acceptance_status", "UNAVAILABLE"))
    checks = analyzer.get("checks", {})
    reasons = [
        f"{prefix}_check_failed:{name}"
        for name, passed in sorted(checks.items())
        if passed is not True
    ]
    if acceptance != "PASS" and not reasons:
        reasons.append(f"{prefix}_acceptance_status:{acceptance}")
    return ("PASS" if acceptance == "PASS" else "FAIL"), reasons


def _status(manifest: dict[str, Any], name: str) -> int:
    value = manifest.get("statuses", {}).get(name, 1)
    try:
        return int(value)
    except (TypeError, ValueError):
        return 1


def _verdict(manifest: dict[str, Any], controller_log: str) -> Verdict:
    if _status(manifest, "safety_status") != 0:
        return Verdict.FAIL_SAFE_STOP
    if _status(manifest, "quality_status") != 0:
        return Verdict.FAIL_TIMING
    if _status(manifest, "terrain_analysis_status") != 0:
        return Verdict.FAIL_PLANNER
    lifecycle_names = (
        "controller_status",
        "analysis_status",
        "ground_truth_status",
        "dynamics_status",
        "completion_status",
    )
    if any(_status(manifest, name) != 0 for name in lifecycle_names):
        return Verdict.RUNNER_FAILURE
    if "Trot hard safety limit" in controller_log or "Trot hard posture limit" in controller_log:
        return Verdict.FAIL_SAFE_STOP
    return Verdict.PASS_DEV


def _max_signal(text: str, name: str) -> float | None:
    values: list[float] = []
    for match in re.finditer(rf"\b{re.escape(name)}=([-+0-9.eE]+)", text):
        try:
            values.append(float(match.group(1)))
        except ValueError:
            continue
    return max(values) if values else None


def _first_failure_s(text: str) -> float | None:
    failure_line = re.compile(
        r"quality guard rejected|hard safety|hard posture|IK failed|planner deadline", re.I
    )
    time_value = re.compile(r"(?:time_s|sim_time|t|time)[=: ]+([-+0-9.eE]+)")
    for line in text.splitlines():
        if not failure_line.search(line):
            continue
        match = time_value.search(line)
        if match:
            try:
                return max(0.0, float(match.group(1)))
            except ValueError:
                pass
    return None


def _csv_time_span(path: Path) -> tuple[float | None, float | None]:
    if not path.is_file():
        return None, None
    try:
        import csv

        with path.open(newline="", encoding="utf-8", errors="replace") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                return None, None
            time_key = next(
                (key for key in reader.fieldnames if key in {"time", "time_s", "sim_time_s"}),
                None,
            )
            if time_key is None:
                return None, None
            values: list[float] = []
            for row in reader:
                try:
                    values.append(float(row[time_key]))
                except (KeyError, TypeError, ValueError):
                    continue
    except OSError:
        return None, None
    if len(values) < 2 or values[-1] <= values[0]:
        return None, None
    span = values[-1] - values[0]
    return span, (len(values) - 1) / span


def _make_failure_window(
    result: Result, controller_artifact: ArtifactRef | None
) -> FailureWindow | None:
    failure_s = result.metrics.first_failure_s
    if failure_s is None:
        return None
    return FailureWindow(
        experiment_id=result.experiment_id,
        start_s=max(0.0, failure_s - 2.0),
        end_s=min(result.duration_s, failure_s + 2.0),
        evidence=[
            EvidencePoint(signal="result.verdict", value=result.verdict.value, source="result"),
            EvidencePoint(
                signal="metrics.first_failure_s",
                value=f"{failure_s:.6f}",
                source="result",
            ),
        ],
        source_artifact=controller_artifact,
    )


@activity.defn(name="run_dev_probe")
async def run_dev_probe(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    workspace = _ensure_source(spec)
    scenario, _, domain_id = _scenario(spec)
    _ensure_mujoco_link(workspace)
    simulator = workspace / "simulate" / "build" / "unitree_mujoco"
    controller = workspace / "example" / "cpp" / "build" / "real_trot_go2"
    runner = workspace / "example" / "cpp" / "scripts" / "run_trot.sh"
    profile = workspace / "example" / "cpp" / "configs" / f"phase1_velocity_{scenario}.csv"
    if not all(path.is_file() for path in (simulator, controller, runner, profile)):
        _invalid(
            "Atlas development probe requires the completed fixed build and profile",
            "ATLAS_BUILD_MISSING",
        )

    root = _artifact_root(workspace)
    run_name = f"orchestration_{spec.experiment_id}"
    run_dir = workspace / "example" / "cpp" / "experiments" / "_runs" / run_name
    if run_dir.exists():
        _invalid("experiment_id already has a preserved Atlas run directory", "ATLAS_RUN_ALREADY_EXISTS")
    process_log = root / spec.experiment_id / "atlas_process.log"
    argv = [
        "bash",
        str(runner),
        f"{spec.wall_timeout_s:g}",
        f"_runs/{run_name}",
        "--headless",
        "--wall-clock-motion",
        "--controller-duration",
        f"{spec.duration_s:g}",
        "--wbc-full",
        "--gait-pattern",
        "running-trot",
        "--kernel",
        "raibert-trot",
        "--period",
        "0.14",
        "--duty",
        "0.44",
        "--step-length",
        "0.50",
        "--foot-lift",
        "0.20",
        "--tau-limit",
        "45",
        "--raibert-velocity-gain",
        "0.010",
        "--raibert-max-adjustment",
        "0.06",
        "--preview-horizon",
        "4",
        "--support-anchor-feedback",
        "--support-anchor-gain",
        "0.35",
        "--velocity-max-accel",
        "0.80",
        "--velocity-max-decel",
        "1.20",
        "--velocity-max-jerk",
        "4.0",
        "--velocity-command-script",
        str(profile),
        "--velocity-max-tracking-lead",
        "0.20",
        "--terrain-sensor-only",
        "--domain-id",
        str(domain_id),
    ]
    started_clock = time.monotonic()
    process_exit = await _run_fixed_command(
        argv,
        workspace,
        _fixed_env(spec),
        process_log,
        spec.wall_timeout_s + 60.0,
        "run_dev_probe",
    )
    elapsed_s = max(0.0, time.monotonic() - started_clock)
    manifest = _manifest(run_dir / "run_manifest.json")
    refs = _copy_run_artifacts(run_dir, process_log, root, spec.experiment_id)
    controller_log = _read_text(run_dir / "controller.log")
    if manifest is None:
        status = RunStatus.TIMEOUT if process_exit is None else RunStatus.FAILED
        result = Result(
            experiment_id=spec.experiment_id,
            status=status,
            verdict=Verdict.RUNNER_FAILURE,
            runner="atlas",
            started_at=_now_iso(),
            finished_at=_now_iso(),
            duration_s=elapsed_s,
            source=spec.source,
            metrics=ResultMetrics(
                safe_stop=False,
                extra={"process_exit_code": -1 if process_exit is None else process_exit},
            ),
            artifacts=refs,
            notes=[
                "Atlas fixed runner did not produce a readable run_manifest.json.",
                "The preserved process log and partial run files are diagnostic evidence only.",
            ],
        )
        return result.model_dump(mode="json")

    verdict = _verdict(manifest, controller_log)
    run_values = manifest.get("run", {}) if isinstance(manifest.get("run"), dict) else {}
    timestamp_values = (
        manifest.get("timestamps", {}) if isinstance(manifest.get("timestamps"), dict) else {}
    )
    started_at = str(timestamp_values.get("started_at") or _now_iso())
    finished_at = str(timestamp_values.get("finished_at") or _now_iso())
    data_duration, wall_rate = _csv_time_span(run_dir / "data.csv")
    metrics = ResultMetrics(
        safe_stop=verdict == Verdict.FAIL_SAFE_STOP,
        first_failure_s=_first_failure_s(controller_log),
        plan_published=bool(re.search(r"plan (?:published|available)", controller_log, re.I)),
        plan_consumed=bool(re.search(r"plan consumed", controller_log, re.I)),
        terrain_actuation=bool(re.search(r"terrain_actuation=on", controller_log, re.I)),
        q_error_max_rad=_max_signal(controller_log, "q_error"),
        foot_error_max_m=_max_signal(controller_log, "foot_error"),
        wall_clock_rate_hz=wall_rate,
        extra={
            "process_exit_code": -1 if process_exit is None else process_exit,
            "quality_status": _status(manifest, "quality_status"),
            "safety_status": _status(manifest, "safety_status"),
            "controller_status": _status(manifest, "controller_status"),
            "source_git_dirty": str(manifest.get("repository", {}).get("git_dirty", "true")) == "true",
            "domain_id": int(run_values.get("domain_id", domain_id)),
        },
    )
    result_status = RunStatus.TIMEOUT if process_exit is None else RunStatus.COMPLETED
    if verdict == Verdict.RUNNER_FAILURE:
        result_status = RunStatus.FAILED
    result = Result(
        experiment_id=spec.experiment_id,
        status=result_status,
        verdict=verdict,
        runner="atlas",
        started_at=started_at,
        finished_at=finished_at,
        duration_s=data_duration if data_duration is not None else max(spec.duration_s, elapsed_s),
        source=spec.source,
        metrics=metrics,
        artifacts=refs,
        notes=[
            "Development probe executed through the fixed existing run_trot.sh runner.",
            "This result is diagnostic and does not establish B0/B1 acceptance.",
            f"runner_sha256={hashlib.sha256(runner.read_bytes()).hexdigest()}",
        ],
    )
    controller_artifact = next(
        (ref for ref in refs if ref.relative_path.endswith("/controller.log")),
        None,
    )
    failure_window = _make_failure_window(result, controller_artifact)
    if failure_window is not None:
        result = result.model_copy(update={"failure_window": failure_window})
    return result.model_dump(mode="json")


async def _run_b0_member_impl(
    spec: ExperimentSpec,
    member: dict[str, Any],
    workspace: Path,
    root: Path,
) -> CampaignMember:
    runs_root = workspace / "example" / "cpp" / "experiments" / "_runs"
    set_name = str(member["set_name"])
    repeat = int(member["repeat"])
    fixed_3mps = bool(member.get("fixed_3mps", False))
    label = "fixed_3mps" if fixed_3mps else str(member["scenario"])
    prefix = f"phase2_b0_{set_name}_{label}_r{repeat}_"
    before = {path.name for path in runs_root.glob(prefix + "*") if path.is_dir()}
    process_log = root / spec.experiment_id / "formal" / "b0" / f"{member['member_id']}.log"
    if fixed_3mps:
        runner = workspace / "example" / "cpp" / "scripts" / "run_phase2_b0_fixed_pair.sh"
        argv = ["bash", str(runner), set_name, str(repeat)]
    else:
        runner = workspace / "example" / "cpp" / "scripts" / "run_phase2_b0_pair.sh"
        argv = ["bash", str(runner), str(member["scenario"]), set_name, str(repeat)]
    if not runner.is_file():
        _invalid("B0 fixed pair runner is missing", "ATLAS_RUNNER_MISSING")

    started_clock = time.monotonic()
    exit_code = await _run_fixed_command(
        argv,
        workspace,
        _fixed_env(spec),
        process_log,
        420.0,
        "run_b0_member",
    )
    elapsed_s = max(0.0, time.monotonic() - started_clock)
    new_dirs = _new_run_dirs(runs_root, prefix, before)
    baseline_dir = next((path for path in new_dirs if path.name.endswith("_baseline")), None)
    terrain_dir = next((path for path in new_dirs if path.name.endswith("_terrain")), None)
    refs: list[ArtifactRef] = []
    for label_name, run_dir in (("baseline", baseline_dir), ("terrain", terrain_dir)):
        if run_dir is not None:
            refs.extend(
                _copy_run_artifacts(
                    run_dir,
                    Path("/__no_process_log__"),
                    root,
                    spec.experiment_id,
                    f"formal/b0/{member['member_id']}/{label_name}",
                )
            )
    process_ref = _copy_file(
        process_log,
        root / spec.experiment_id / "formal" / "b0" / member["member_id"] / "atlas_process.log",
        root,
    )
    if process_ref is not None:
        refs.append(process_ref)

    analyzer = _manifest(terrain_dir / "b0_analyzer.json") if terrain_dir is not None else None
    acceptance_status, failure_reasons = _analyzer_failure_reasons(analyzer, "b0")
    if exit_code is None:
        failure_reasons.append("runner_timeout")
    elif exit_code != 0:
        failure_reasons.append(f"runner_exit_code:{exit_code}")
    if baseline_dir is None or terrain_dir is None:
        failure_reasons.append("paired_run_directory_missing")
    passed = exit_code == 0 and acceptance_status == "PASS" and not failure_reasons
    if passed:
        status = "completed"
    elif exit_code is None:
        status = "timeout"
    else:
        status = "failed"
    member_receipt = CampaignMember(
        member_id=str(member["member_id"]),
        stage="b0",
        case=str(member["case"]),
        profile=(
            None
            if fixed_3mps
            else f"example/cpp/configs/phase1_velocity_{member['scenario']}.csv"
        ),
        repeat=repeat,
        baseline_domain=int(member["baseline_domain"]),
        terrain_domain=int(member["terrain_domain"]),
        status=status,
        passed=passed,
        exit_code=exit_code,
        acceptance_status=acceptance_status,
        failure_reasons=failure_reasons[:30],
        artifacts=refs[:100],
    )
    receipt_path = (
        root / spec.experiment_id / "formal" / "b0" / member["member_id"] / "member_receipt.json"
    )
    _write_json(receipt_path, member_receipt.model_dump(mode="json"))
    receipt_ref = _artifact_ref(receipt_path, root)
    return member_receipt.model_copy(update={"artifacts": [*member_receipt.artifacts, receipt_ref][:100]})


def _finalize_campaign(
    spec: ExperimentSpec,
    workspace: Path,
    root: Path,
    stage: str,
    contract: str,
    manifest_sha256: str,
    contract_ref: ArtifactRef,
    members: list[CampaignMember],
    prerequisite_passed: bool | None,
    blocked: bool = False,
) -> CampaignReceipt:
    members_total = len(members)
    members_passed = sum(member.passed for member in members)
    members_failed = members_total - members_passed
    failure_reasons = [
        f"{member.member_id}:{reason}"
        for member in members
        for reason in member.failure_reasons
    ][:100]
    if blocked:
        status = "skipped"
        acceptance_status = "BLOCKED"
    else:
        status = "completed" if all(member.status in {"completed", "failed"} for member in members) else "failed"
        acceptance_status = "PASS" if members_total > 0 and members_passed == members_total else "FAIL"
    artifact_refs = [contract_ref]
    artifact_refs.extend(
        reference
        for member in members
        for reference in member.artifacts
        if reference.relative_path.endswith("/member_receipt.json")
    )
    campaign = CampaignReceipt(
        campaign_id=f"{spec.experiment_id}-{stage}",
        experiment_id=spec.experiment_id,
        stage=stage,
        contract=contract,
        manifest_sha256=manifest_sha256,
        status=status,
        acceptance_status=acceptance_status,
        prerequisite_passed=prerequisite_passed,
        members_total=members_total,
        members_passed=members_passed,
        members_failed=members_failed,
        source=spec.source,
        members=members,
        failure_reasons=failure_reasons or ([f"{stage}_prerequisite_not_passed"] if blocked else []),
        artifacts=artifact_refs[:100],
    )
    receipt_path = root / spec.experiment_id / "formal" / stage / "campaign_receipt.json"
    _write_json(receipt_path, campaign.model_dump(mode="json"))
    receipt_ref = _artifact_ref(receipt_path, root)
    return campaign.model_copy(update={"artifacts": [*campaign.artifacts, receipt_ref][:100]})


@activity.defn(name="run_b0_member")
async def run_b0_member(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    _require_autonomous(spec)
    workspace = _ensure_source(spec)
    root = _artifact_root(workspace)
    manifest_path, manifest, _ = _load_contract_manifest(workspace, "b0")
    _validate_b0_manifest(manifest)
    expected = _b0_holdout_members(manifest)
    member = payload.get("member") if isinstance(payload, dict) else None
    if member is None:
        member = _b0_development_member(spec)
    if not isinstance(member, dict):
        _invalid("run_b0_member requires a structured member descriptor", "ATLAS_MEMBER_INVALID")
    if member != _b0_development_member(spec) and member not in expected:
        _invalid("B0 member is not present in the frozen manifest", "ATLAS_MEMBER_INVALID")
    result = await _run_b0_member_impl(spec, member, workspace, root)
    return result.model_dump(mode="json")


@activity.defn(name="run_b0_holdout")
async def run_b0_holdout(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    _require_autonomous(spec)
    workspace = _ensure_source(spec)
    root = _artifact_root(workspace)
    manifest_path, manifest, manifest_sha256 = _load_contract_manifest(workspace, "b0")
    members = _b0_holdout_members(manifest)
    contract_ref = _copy_contract_manifest(manifest_path, root, spec, "b0")
    receipts: list[CampaignMember] = []
    for index, member in enumerate(members, start=1):
        try:
            activity.heartbeat(
                {"stage": "b0", "member": member["member_id"], "index": index, "total": len(members)}
            )
        except RuntimeError:
            pass
        receipts.append(await _run_b0_member_impl(spec, member, workspace, root))
    campaign = _finalize_campaign(
        spec,
        workspace,
        root,
        "b0",
        B0_CONTRACT,
        manifest_sha256,
        contract_ref,
        receipts,
        prerequisite_passed=True,
    )
    return campaign.model_dump(mode="json")


async def _run_b1_case_impl(
    spec: ExperimentSpec,
    member: dict[str, Any],
    workspace: Path,
    root: Path,
) -> CampaignMember:
    runs_root = workspace / "example" / "cpp" / "experiments" / "_runs"
    case = str(member["case"])
    prefix = f"phase2_b1_{case}_"
    before = {path.name for path in runs_root.glob(prefix + "*") if path.is_dir()}
    simulator = workspace / "simulate" / "build" / "unitree_mujoco"
    controller = workspace / "example" / "cpp" / "build" / "real_trot_go2"
    runner = workspace / "example" / "cpp" / "scripts" / "run_trot.sh"
    profile = workspace / B1_PROFILE
    scene = (workspace / str(member["scene"])).resolve()
    if not all(path.is_file() for path in (simulator, controller, runner, profile, scene)):
        _invalid("B1 probe requires the fixed build, profile, runner, and scene files", "ATLAS_BUILD_MISSING")
    try:
        scene.relative_to(workspace.resolve())
    except ValueError:
        _invalid("B1 scene escaped the source checkout", "ATLAS_SCENE_INVALID")
    process_log = root / spec.experiment_id / "formal" / "b1" / f"{case}.log"
    argv = [
        "bash",
        str(runner),
        "140",
        f"_runs/{prefix}orchestration",
        "--headless",
        "--wall-clock-motion",
        "--controller-duration",
        f"{B1_DURATION_S:g}",
        "--wbc-full",
        "--gait-pattern",
        "running-trot",
        "--gait-phase-offset",
        f"{float(member['gait_phase_offset']):g}",
        "--kernel",
        "raibert-trot",
        "--period",
        "0.14",
        "--duty",
        "0.44",
        "--step-length",
        "0.50",
        "--foot-lift",
        "0.20",
        "--tau-limit",
        "45",
        "--raibert-velocity-gain",
        "0.010",
        "--raibert-max-adjustment",
        "0.06",
        "--preview-horizon",
        "4",
        "--support-anchor-feedback",
        "--support-anchor-gain",
        "0.35",
        "--velocity-max-accel",
        "0.80",
        "--velocity-max-decel",
        "1.20",
        "--velocity-max-jerk",
        "4.0",
        "--velocity-command-script",
        str(profile),
        "--velocity-max-tracking-lead",
        "0.20",
        "--terrain-planner",
        "--phase2-milestone",
        "B1",
        "--scene-file",
        str(scene),
        "--initial-x",
        f"{float(member['initial_x_m']):g}",
        "--initial-y",
        f"{float(member['initial_y_m']):g}",
        "--domain-id",
        str(int(member["terrain_domain"])),
    ]
    env = _fixed_env(spec)
    env["TROT_SEED"] = str(int(member["seed"]))
    exit_code = await _run_fixed_command(
        argv,
        workspace,
        env,
        process_log,
        240.0,
        "run_b1_probe",
    )
    run_dirs = _new_run_dirs(runs_root, prefix, before)
    run_dir = run_dirs[-1] if run_dirs else None
    refs: list[ArtifactRef] = []
    if run_dir is not None:
        refs.extend(
            _copy_run_artifacts(
                run_dir,
                Path("/__no_process_log__"),
                root,
                spec.experiment_id,
                f"formal/b1/{case}/run",
            )
        )
    process_ref = _copy_file(
        process_log,
        root / spec.experiment_id / "formal" / "b1" / case / "atlas_process.log",
        root,
    )
    if process_ref is not None:
        refs.append(process_ref)
    analyzer = _manifest(run_dir / "phase2_terrain_analysis.json") if run_dir is not None else None
    acceptance_status, failure_reasons = _analyzer_failure_reasons(analyzer, "b1")
    if exit_code is None:
        failure_reasons.append("runner_timeout")
    elif exit_code != 0:
        failure_reasons.append(f"runner_exit_code:{exit_code}")
    if run_dir is None:
        failure_reasons.append("run_directory_missing")
    passed = exit_code == 0 and acceptance_status == "PASS" and not failure_reasons
    member_receipt = CampaignMember(
        member_id=case,
        stage="b1",
        case=case,
        profile=B1_PROFILE,
        repeat=0,
        terrain_domain=int(member["terrain_domain"]),
        status="completed" if passed else ("timeout" if exit_code is None else "failed"),
        passed=passed,
        exit_code=exit_code,
        acceptance_status=acceptance_status,
        failure_reasons=failure_reasons[:30],
        artifacts=refs[:100],
    )
    receipt_path = root / spec.experiment_id / "formal" / "b1" / case / "member_receipt.json"
    _write_json(receipt_path, member_receipt.model_dump(mode="json"))
    receipt_ref = _artifact_ref(receipt_path, root)
    return member_receipt.model_copy(update={"artifacts": [*member_receipt.artifacts, receipt_ref][:100]})


@activity.defn(name="run_b1_probe")
async def run_b1_probe(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    _require_autonomous(spec)
    workspace = _ensure_source(spec)
    root = _artifact_root(workspace)
    manifest_path, manifest, manifest_sha256 = _load_contract_manifest(workspace, "b1")
    members = _b1_members(manifest)
    contract_ref = _copy_contract_manifest(manifest_path, root, spec, "b1")
    raw_b0 = payload.get("b0_campaign") if isinstance(payload, dict) else None
    if not isinstance(raw_b0, dict):
        _invalid("B1 probe requires the preceding B0 campaign receipt", "ATLAS_B1_PREREQUISITE_MISSING")
    try:
        b0_campaign = CampaignReceipt.model_validate(raw_b0)
    except Exception as exc:
        raise ApplicationError(
            "B1 prerequisite is not a valid campaign.v1 receipt",
            type="ATLAS_B1_PREREQUISITE_INVALID",
            non_retryable=True,
        ) from exc
    if (
        b0_campaign.experiment_id != spec.experiment_id
        or b0_campaign.stage != "b0"
        or b0_campaign.acceptance_status != "PASS"
    ):
        campaign = _finalize_campaign(
            spec,
            workspace,
            root,
            "b1",
            B1_CONTRACT,
            manifest_sha256,
            contract_ref,
            [],
            prerequisite_passed=False,
            blocked=True,
        )
        return campaign.model_dump(mode="json")
    receipts: list[CampaignMember] = []
    for index, member in enumerate(members, start=1):
        try:
            activity.heartbeat(
                {"stage": "b1", "member": member["member_id"], "index": index, "total": len(members)}
            )
        except RuntimeError:
            pass
        receipts.append(await _run_b1_case_impl(spec, member, workspace, root))
    campaign = _finalize_campaign(
        spec,
        workspace,
        root,
        "b1",
        B1_CONTRACT,
        manifest_sha256,
        contract_ref,
        receipts,
        prerequisite_passed=True,
    )
    return campaign.model_dump(mode="json")


@activity.defn(name="extract_failure_window")
async def extract_failure_window(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    if not isinstance(payload, dict) or not isinstance(payload.get("result"), dict):
        _invalid("extract_failure_window requires a result.v1 payload", "INVALID_RESULT_PAYLOAD")
    try:
        result = Result.model_validate(payload["result"])
    except Exception as exc:
        raise ApplicationError(
            "extract_failure_window received an invalid result.v1 payload",
            type="INVALID_RESULT_PAYLOAD",
            non_retryable=True,
        ) from exc
    if result.experiment_id != spec.experiment_id:
        _invalid("result and spec experiment_id differ", "INVALID_RESULT_PAYLOAD")
    if result.failure_window is None:
        _invalid(
            "no timestamped failure was present; refusing to invent a failure window",
            "FAILURE_TIME_UNAVAILABLE",
        )
    return result.failure_window.model_dump(mode="json")
