"""Base-side activities: validation, deterministic diagnosis, and optional Codex."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pydantic import ValidationError
from temporalio import activity
from temporalio.exceptions import ApplicationError

from ..policies import load_policy, validate_spec_against_policy
from ..schemas.models import (
    ActionType,
    Diagnosis,
    DiagnosisSource,
    EvidencePoint,
    ExperimentSpec,
    FailureClass,
    NextAction,
    ProbeProfile,
    Result,
    ResultMetrics,
    RunStatus,
    Verdict,
)


EXPECTED_REPOSITORY = "kairoi-k/go2-mujoco-control"
MAX_CODEX_PROMPT_BYTES = 32_000
MAX_CODEX_OUTPUT_BYTES = 100_000


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _spec(payload: Any) -> ExperimentSpec:
    try:
        return ExperimentSpec.model_validate(payload)
    except ValidationError as exc:
        raise ApplicationError(
            "invalid experiment.v1 payload",
            type="INVALID_EXPERIMENT_SPEC",
            non_retryable=True,
            details=[str(exc)],
        ) from exc


def _result(payload: Any) -> Result:
    try:
        return Result.model_validate(payload)
    except ValidationError as exc:
        raise ApplicationError(
            "invalid result.v1 payload",
            type="INVALID_RESULT",
            non_retryable=True,
            details=[str(exc)],
        ) from exc


def _diagnosis(payload: Any) -> Diagnosis:
    try:
        return Diagnosis.model_validate(payload)
    except ValidationError as exc:
        raise ApplicationError(
            "invalid diagnosis.v1 payload",
            type="INVALID_DIAGNOSIS",
            non_retryable=True,
            details=[str(exc)],
        ) from exc


def _local_source_state(repo_root: Path) -> tuple[str, bool]:
    try:
        head = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "-C", str(repo_root), "status", "--porcelain"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ApplicationError(
            "agent worker repository state could not be read",
            type="SOURCE_STATE_UNAVAILABLE",
            non_retryable=True,
        ) from exc
    return head, dirty


@activity.defn(name="validate_experiment")
async def validate_experiment(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    if spec.source.repository != EXPECTED_REPOSITORY:
        raise ApplicationError(
            f"unexpected source repository {spec.source.repository!r}",
            type="SOURCE_REPOSITORY_MISMATCH",
            non_retryable=True,
        )
    repo_root = Path(os.environ.get("RESEARCH_REPO_ROOT", Path.cwd())).expanduser().resolve()
    if not (repo_root / ".git").exists():
        raise ApplicationError(
            "RESEARCH_REPO_ROOT is not a Git checkout",
            type="SOURCE_STATE_UNAVAILABLE",
            non_retryable=True,
        )
    local_sha, local_dirty = _local_source_state(repo_root)
    if local_sha != spec.source.git_sha:
        raise ApplicationError(
            "worker checkout HEAD does not match experiment source.git_sha",
            type="SOURCE_SHA_MISMATCH",
            non_retryable=True,
        )
    if local_dirty != spec.source.dirty:
        raise ApplicationError(
            "worker checkout dirty state does not match experiment source.dirty",
            type="SOURCE_DIRTY_MISMATCH",
            non_retryable=True,
        )
    policy = load_policy(spec.policy_id.value)
    try:
        validate_spec_against_policy(spec, policy)
    except ValueError as exc:
        raise ApplicationError(str(exc), type="POLICY_REJECTED", non_retryable=True) from exc
    if spec.execution_mode.value == "atlas" and spec.atlas_task_queue != "atlas":
        raise ApplicationError(
            "the first Atlas integration accepts only the fixed 'atlas' task queue",
            type="ATLAS_QUEUE_REJECTED",
            non_retryable=True,
        )
    return spec.model_dump(mode="json")


def make_fixture_result(payload: dict[str, Any]) -> dict[str, Any]:
    spec = _spec(payload)
    started_at = _now_iso()
    raw_verdict = spec.parameters.get("fixture_verdict", Verdict.PASS_DEV.value)
    try:
        verdict = Verdict(str(raw_verdict))
    except ValueError as exc:
        raise ApplicationError(
            f"unsupported fixture_verdict {raw_verdict!r}",
            type="INVALID_FIXTURE_VERDICT",
            non_retryable=True,
        ) from exc
    status = (
        RunStatus.COMPLETED
        if verdict in {Verdict.PASS_DEV, Verdict.UNKNOWN}
        else RunStatus.FAILED
    )
    finished_at = _now_iso()
    first_failure_s = None if verdict == Verdict.PASS_DEV else min(spec.duration_s, 1.0)
    result = Result(
        experiment_id=spec.experiment_id,
        status=status,
        verdict=verdict,
        runner="fixture",
        started_at=started_at,
        finished_at=finished_at,
        duration_s=0.001,
        source=spec.source,
        metrics=ResultMetrics(
            safe_stop=verdict == Verdict.FAIL_SAFE_STOP,
            first_failure_s=first_failure_s,
            plan_published=False,
            plan_consumed=False,
            terrain_actuation=False,
            extra={"fixture": True, "controller_executed": False},
        ),
        notes=[
            "Synthetic fixture only; no controller, MuJoCo process, or Atlas host was used.",
            "This result validates orchestration and schema plumbing, not locomotion behavior.",
        ],
    )
    return result.model_dump(mode="json")


@activity.defn(name="run_fixture_probe")
async def run_fixture_probe(payload: dict[str, Any]) -> dict[str, Any]:
    return make_fixture_result(payload)


def classify_result_payload(payload: dict[str, Any]) -> dict[str, Any]:
    result = _result(payload)
    if result.status != RunStatus.COMPLETED:
        failure_class = FailureClass.RUNNER_FAILURE
        action = ActionType.ESCALATE
        summary = "The runner did not produce a completed result; preserve the failure bundle and inspect the worker path."
        requires_codex = False
        requires_human_review = True
    elif result.verdict == Verdict.PASS_DEV:
        failure_class = FailureClass.PASS_DEV
        action = ActionType.CHECKPOINT
        summary = "The deterministic development gates passed; a formal holdout remains a separate checkpoint."
        requires_codex = False
        requires_human_review = False
    else:
        try:
            failure_class = FailureClass(result.verdict.value)
        except ValueError:
            failure_class = FailureClass.UNKNOWN
        action = (
            ActionType.ESCALATE
            if failure_class in {FailureClass.UNKNOWN, FailureClass.FAIL_SAFE_STOP}
            else ActionType.COLLECT_EVIDENCE
        )
        summary = f"Deterministic classifier matched result verdict {result.verdict.value}."
        requires_codex = failure_class == FailureClass.UNKNOWN
        requires_human_review = failure_class in {FailureClass.UNKNOWN, FailureClass.FAIL_SAFE_STOP}

    evidence = [
        EvidencePoint(signal="result.status", value=result.status.value, source="result"),
        EvidencePoint(signal="result.verdict", value=result.verdict.value, source="result"),
    ]
    if result.metrics.first_failure_s is not None:
        evidence.append(
            EvidencePoint(
                signal="metrics.first_failure_s",
                value=f"{result.metrics.first_failure_s:.6f}",
                source="result",
            )
        )
    diagnosis = Diagnosis(
        experiment_id=result.experiment_id,
        source=DiagnosisSource.DETERMINISTIC,
        failure_class=failure_class,
        confidence=1.0 if failure_class != FailureClass.UNKNOWN else 0.0,
        summary=summary,
        evidence=evidence,
        recommended_action=action,
        requires_codex=requires_codex,
        requires_human_review=requires_human_review,
    )
    return diagnosis.model_dump(mode="json")


@activity.defn(name="classify_result")
async def classify_result(payload: dict[str, Any]) -> dict[str, Any]:
    return classify_result_payload(payload)


def _codex_failure_diagnosis(result: Result, error_code: str, summary: str) -> Diagnosis:
    return Diagnosis(
        experiment_id=result.experiment_id,
        source=DiagnosisSource.CODEX_ERROR,
        failure_class=FailureClass.UNKNOWN,
        confidence=0.0,
        summary=summary,
        evidence=[EvidencePoint(signal="codex.status", value=error_code, source="result")],
        recommended_action=ActionType.ESCALATE,
        requires_human_review=True,
        error_code=error_code,
    )


def _json_object(text: str) -> dict[str, Any]:
    candidate = text.strip()
    if candidate.startswith("```"):
        lines = candidate.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        candidate = "\n".join(lines).strip()
    value = json.loads(candidate)
    if not isinstance(value, dict):
        raise ValueError("Codex output must be a JSON object")
    return value


def _normalize_codex_output(result: Result, raw: dict[str, Any]) -> Diagnosis:
    try:
        failure_class = FailureClass(str(raw.get("failure_class", FailureClass.UNKNOWN.value)))
    except ValueError:
        failure_class = FailureClass.UNKNOWN
    try:
        action = ActionType(str(raw.get("recommended_action", ActionType.ESCALATE.value)))
    except ValueError:
        action = ActionType.ESCALATE
    try:
        confidence = float(raw.get("confidence", 0.0))
    except (TypeError, ValueError):
        confidence = 0.0
    confidence = max(0.0, min(1.0, confidence))

    evidence: list[EvidencePoint] = []
    raw_evidence = raw.get("evidence", [])
    if isinstance(raw_evidence, list):
        for item in raw_evidence[:20]:
            if isinstance(item, dict):
                signal = str(item.get("signal", "codex.evidence"))[:120]
                value = str(item.get("value", ""))[:500]
                evidence.append(EvidencePoint(signal=signal, value=value, source="codex"))
    if not evidence:
        evidence.append(EvidencePoint(signal="codex.output", value="validated JSON response", source="codex"))

    recommended_profile = None
    raw_profile = raw.get("recommended_profile")
    if raw_profile is not None:
        try:
            recommended_profile = ProbeProfile(str(raw_profile))
        except ValueError:
            recommended_profile = None
    parameters = raw.get("recommended_parameters", {})
    if not isinstance(parameters, dict):
        parameters = {}
    summary = str(raw.get("summary", "Codex returned no summary."))[:2000]
    hypothesis = raw.get("hypothesis")
    if hypothesis is not None:
        hypothesis = str(hypothesis)[:2000]
    requires_review = bool(raw.get("needs_human_review", True))
    if failure_class in {FailureClass.UNKNOWN, FailureClass.FAIL_SAFE_STOP} or action in {
        ActionType.ESCALATE,
        ActionType.STOP,
    }:
        requires_review = True
    return Diagnosis(
        experiment_id=result.experiment_id,
        source=DiagnosisSource.CODEX,
        failure_class=failure_class,
        confidence=confidence,
        summary=summary,
        evidence=evidence,
        hypothesis=hypothesis,
        recommended_action=action,
        recommended_profile=recommended_profile,
        recommended_parameters=parameters,
        requires_codex=False,
        requires_human_review=requires_review,
    )


def diagnose_with_codex_payload(
    result_payload: dict[str, Any],
    baseline_payload: dict[str, Any],
    spec_payload: dict[str, Any] | None = None,
) -> dict[str, Any]:
    result = _result(result_payload)
    baseline = _diagnosis(baseline_payload)
    spec = _spec(spec_payload) if spec_payload is not None else None
    repo_root = Path(os.environ.get("RESEARCH_REPO_ROOT", Path.cwd())).expanduser().resolve()
    if not (repo_root / ".git").exists():
        return _codex_failure_diagnosis(
            result,
            "REPO_ROOT_MISSING",
            "Codex diagnosis was not run because RESEARCH_REPO_ROOT is not a Git checkout.",
        ).model_dump(mode="json")
    codex_bin = os.environ.get("CODEX_BIN", "codex")
    codex_path = Path(codex_bin)
    if codex_path.is_absolute():
        executable = codex_path if codex_path.is_file() else None
    else:
        executable = Path(shutil.which(codex_bin) or "")
    if executable is None or not executable.is_file():
        return _codex_failure_diagnosis(
            result,
            "CODEX_NOT_FOUND",
            "Codex CLI is not available to the Base agent worker.",
        ).model_dump(mode="json")

    bundle = {
        "experiment": {"experiment_id": result.experiment_id, "source": result.source.model_dump(mode="json")},
        "result": result.model_dump(mode="json"),
        "deterministic_diagnosis": baseline.model_dump(mode="json"),
    }
    prompt = (
        "You are a bounded research-diagnosis assistant for a Go2 MuJoCo study. "
        "Use only the JSON bundle below and, if needed, read repository files. "
        "Do not edit files, run experiments, change controller algorithms, or claim hardware validity. "
        "Return only JSON matching the supplied schema. Keep evidence tied to observed signals; "
        "unknown causes must remain UNKNOWN and require human review.\n\n"
        + json.dumps(bundle, ensure_ascii=False, sort_keys=True)
    )
    if len(prompt.encode("utf-8")) > MAX_CODEX_PROMPT_BYTES:
        return _codex_failure_diagnosis(
            result,
            "PROMPT_TOO_LARGE",
            "Codex diagnosis was skipped because the bounded diagnosis bundle exceeded its size limit.",
        ).model_dump(mode="json")

    schema_path = Path(__file__).resolve().parents[1] / "schemas" / "codex_diagnosis.schema.json"
    configured_timeout = spec.codex_timeout_s if spec is not None else int(os.environ.get("CODEX_TIMEOUT_S", "90"))
    timeout_s = max(10, min(600, configured_timeout))
    codex_model = os.environ.get("CODEX_MODEL", "gpt-5.5")
    codex_reasoning = os.environ.get("CODEX_REASONING_EFFORT", "medium")
    if codex_reasoning not in {"none", "low", "medium", "high", "xhigh"}:
        codex_reasoning = "xhigh"
    with tempfile.TemporaryDirectory(prefix="go2-codex-") as temp_dir:
        temp = Path(temp_dir)
        output_path = temp / "last-message.json"
        stdout_path = temp / "stdout.log"
        stderr_path = temp / "stderr.log"
        argv = [
            str(executable),
            "exec",
            "--ephemeral",
            "--model",
            codex_model,
            "-c",
            f'model_reasoning_effort="{codex_reasoning}"',
            "--sandbox",
            "read-only",
            "--output-schema",
            str(schema_path),
            "--output-last-message",
            str(output_path),
            "--cd",
            str(repo_root),
            "-",
        ]
        env = os.environ.copy()
        if env.get("CODEX_ALLOW_API_KEY", "0") != "1":
            env.pop("OPENAI_API_KEY", None)
            env.pop("OPENAI_BASE_URL", None)
        try:
            with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
                completed = subprocess.run(
                    argv,
                    input=prompt.encode("utf-8"),
                    stdout=stdout,
                    stderr=stderr,
                    cwd=repo_root,
                    env=env,
                    timeout=timeout_s,
                    check=False,
                )
        except subprocess.TimeoutExpired:
            return _codex_failure_diagnosis(
                result,
                "CODEX_TIMEOUT",
                f"Codex diagnosis exceeded the bounded {timeout_s}s timeout.",
            ).model_dump(mode="json")
        except OSError:
            return _codex_failure_diagnosis(
                result,
                "CODEX_EXEC_ERROR",
                "Codex CLI could not be started by the Base agent worker.",
            ).model_dump(mode="json")

        if completed.returncode != 0:
            return _codex_failure_diagnosis(
                result,
                f"CODEX_EXIT_{abs(completed.returncode)}",
                "Codex CLI returned a non-zero status; no automatic action was taken.",
            ).model_dump(mode="json")
        try:
            raw_bytes = output_path.read_bytes()
            if len(raw_bytes) > MAX_CODEX_OUTPUT_BYTES:
                raise ValueError("output exceeds bounded size")
            raw = _json_object(raw_bytes.decode("utf-8"))
            diagnosis = _normalize_codex_output(result, raw)
        except (OSError, UnicodeDecodeError, ValueError, json.JSONDecodeError):
            return _codex_failure_diagnosis(
                result,
                "CODEX_INVALID_JSON",
                "Codex returned no valid diagnosis.v1-compatible JSON; no automatic action was taken.",
            ).model_dump(mode="json")
    return diagnosis.model_dump(mode="json")


@activity.defn(name="diagnose_with_codex")
async def diagnose_with_codex(
    result_payload: dict[str, Any],
    baseline_payload: dict[str, Any],
    spec_payload: dict[str, Any],
) -> dict[str, Any]:
    return diagnose_with_codex_payload(result_payload, baseline_payload, spec_payload)


def decide_next_action_payload(
    spec_payload: dict[str, Any], diagnosis_payload: dict[str, Any]
) -> dict[str, Any]:
    spec = _spec(spec_payload)
    diagnosis = _diagnosis(diagnosis_payload)
    guardrails = [
        "No controller algorithm or acceptance threshold mutation is authorized.",
        "Do not run formal B0/B1 without explicit human approval.",
        "Any proposed parameters must pass the policy validator before execution.",
        "Atlas commands must come from an allowlisted adapter operation, never the experiment JSON.",
    ]
    if diagnosis.source == DiagnosisSource.CODEX_ERROR or diagnosis.failure_class in {
        FailureClass.UNKNOWN,
        FailureClass.RUNNER_FAILURE,
        FailureClass.FAIL_SAFE_STOP,
    }:
        action = ActionType.ESCALATE
        rationale = diagnosis.summary
        proposed_profile = None
    elif diagnosis.failure_class == FailureClass.PASS_DEV:
        action = ActionType.CHECKPOINT
        rationale = "Development probe passed; stop at the formal checkpoint until a human approves the holdout."
        proposed_profile = None
    else:
        action = diagnosis.recommended_action
        if action not in {ActionType.RERUN, ActionType.COLLECT_EVIDENCE}:
            action = ActionType.COLLECT_EVIDENCE
        rationale = diagnosis.summary
        proposed_profile = spec.profile
    next_action = NextAction(
        experiment_id=spec.experiment_id,
        action=action,
        rationale=rationale,
        proposed_profile=proposed_profile,
        proposed_parameters=diagnosis.recommended_parameters,
        requires_approval=True,
        guardrails=guardrails,
    )
    return next_action.model_dump(mode="json")


@activity.defn(name="decide_next_action")
async def decide_next_action(
    spec_payload: dict[str, Any], diagnosis_payload: dict[str, Any]
) -> dict[str, Any]:
    return decide_next_action_payload(spec_payload, diagnosis_payload)
