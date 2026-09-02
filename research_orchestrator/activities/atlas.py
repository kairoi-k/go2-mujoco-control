"""Allowlisted Atlas activities for bounded, provenance-first experiments.

The Atlas worker is the only place where a physical MuJoCo/controller process
may run. Requests are structured experiment specs; no request can provide a
shell command, executable path, environment assignment, or arbitrary argv.
Formal holdouts remain fail-closed until a separate approval workflow exists.
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

from ..schemas.models import (
    ActivityReceipt,
    ArtifactRef,
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
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return _artifact_ref(target, root)


def _copy_run_artifacts(
    run_dir: Path, process_log: Path, root: Path, experiment_id: str
) -> list[ArtifactRef]:
    artifact_dir = root / experiment_id / "run"
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
    process_ref = _copy_file(process_log, root / experiment_id / "atlas_process.log", root)
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


def _formal_not_ready(name: str, payload: Any) -> NoReturn:
    _spec(payload)
    raise ApplicationError(
        f"{name} is reserved for a separate human-approved checkpoint; no physical command was executed",
        type="ATLAS_FORMAL_APPROVAL_REQUIRED",
        non_retryable=True,
    )


@activity.defn(name="run_b0_member")
async def run_b0_member(payload: dict[str, Any]) -> dict[str, Any]:
    _formal_not_ready("run_b0_member", payload)


@activity.defn(name="run_b0_holdout")
async def run_b0_holdout(payload: dict[str, Any]) -> dict[str, Any]:
    _formal_not_ready("run_b0_holdout", payload)


@activity.defn(name="run_b1_probe")
async def run_b1_probe(payload: dict[str, Any]) -> dict[str, Any]:
    _formal_not_ready("run_b1_probe", payload)


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
