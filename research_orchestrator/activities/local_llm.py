"""Bounded local-LLM diagnosis on Atlas.

The model server is native Windows CUDA software launched from the Atlas WSL
worker only for this Activity. The same Atlas worker has
``max_concurrent_activities=1`` and every physical Activity checks that the
server is stopped, so GPU inference cannot overlap MuJoCo/controller work.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import os
import re
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib import error as urlerror
from urllib import request as urlrequest

from pydantic import ValidationError
from temporalio import activity
from temporalio.exceptions import ApplicationError

from ..schemas.models import (
    ActionType,
    Diagnosis,
    DiagnosisSource,
    EvidencePoint,
    ExperimentSpec,
    FailureClass,
    InferenceReceipt,
    ProbeProfile,
    Result,
    Verdict,
)


LOCAL_LLM_ACTIVITY_NAME = "diagnose_with_local_llm"
LOCAL_LLM_SERVER_MARKER = "go2-local-llm"
MAX_LOCAL_PROMPT_BYTES = 32_000
MAX_LOCAL_RESPONSE_BYTES = 100_000
MAX_LOCAL_SERVER_LOG_BYTES = 8 * 1024 * 1024

_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "schemas" / "local_diagnosis.schema.json"
LOCAL_DIAGNOSIS_SCHEMA: dict[str, Any] = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))


@dataclass(frozen=True)
class LocalLLMSettings:
    server_exe: Path
    model_path: Path
    host: str
    port: int
    model_id: str
    model_revision: str
    quantization: str
    runtime_version: str
    model_sha256: str | None
    verify_model_hash: bool
    context_size: int
    batch_size: int
    ubatch_size: int
    reasoning_budget: int
    reasoning_mode: str
    reasoning_format: str
    start_timeout_s: float
    request_timeout_s: float
    min_confidence: float
    process_marker: str

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"


@dataclass
class _RunningServer:
    process: subprocess.Popen[bytes]
    log_handle: Any


def _int_env(name: str, default: int, low: int, high: int) -> int:
    raw = os.environ.get(name, str(default)).strip()
    try:
        value = int(raw)
    except ValueError as exc:
        raise ApplicationError(
            f"{name} must be an integer",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        ) from exc
    if not low <= value <= high:
        raise ApplicationError(
            f"{name} is outside its safe range",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        )
    return value


def _float_env(name: str, default: float, low: float, high: float) -> float:
    raw = os.environ.get(name, str(default)).strip()
    try:
        value = float(raw)
    except ValueError as exc:
        raise ApplicationError(
            f"{name} must be numeric",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        ) from exc
    if not low <= value <= high:
        raise ApplicationError(
            f"{name} is outside its safe range",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        )
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def settings_from_env() -> LocalLLMSettings:
    server_exe = Path(
        os.environ.get(
            "ATLAS_LLM_SERVER_EXE",
            "/mnt/c/Users/w1881/go2-local-llm/bin/llama-server.exe",
        )
    ).expanduser()
    model_path = Path(
        os.environ.get(
            "ATLAS_LLM_MODEL_PATH",
            "/mnt/c/Users/w1881/go2-local-llm/models/gpt-oss-20b-MXFP4.gguf",
        )
    ).expanduser()
    host = os.environ.get("ATLAS_LLM_HOST", "127.0.0.1").strip()
    if host != "127.0.0.1":
        raise ApplicationError(
            "Atlas local LLM must bind to loopback",
            type="LOCAL_LLM_BIND_INVALID",
            non_retryable=True,
        )
    process_marker = os.environ.get("ATLAS_LLM_PROCESS_MARKER", LOCAL_LLM_SERVER_MARKER).strip()
    if not process_marker or any(char in process_marker for char in "'\"\r\n"):
        raise ApplicationError(
            "ATLAS_LLM_PROCESS_MARKER is invalid",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        )
    model_sha256 = os.environ.get("ATLAS_LLM_MODEL_SHA256", "").strip().lower() or None
    if model_sha256 is not None and not re.fullmatch(r"[0-9a-f]{64}", model_sha256):
        raise ApplicationError(
            "ATLAS_LLM_MODEL_SHA256 must be a lowercase SHA-256 value",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        )
    reasoning_mode = os.environ.get("ATLAS_LLM_REASONING", "on").strip().lower()
    reasoning_format = os.environ.get("ATLAS_LLM_REASONING_FORMAT", "deepseek").strip().lower()
    if reasoning_mode not in {"on", "off", "auto"} or reasoning_format not in {
        "none",
        "deepseek",
        "deepseek-legacy",
    }:
        raise ApplicationError(
            "ATLAS_LLM_REASONING or ATLAS_LLM_REASONING_FORMAT is invalid",
            type="LOCAL_LLM_CONFIG_INVALID",
            non_retryable=True,
        )
    return LocalLLMSettings(
        server_exe=server_exe,
        model_path=model_path,
        host=host,
        port=_int_env("ATLAS_LLM_PORT", 8090, 1024, 65535),
        model_id=os.environ.get("ATLAS_LLM_MODEL_ID", "gpt-oss-20b-MXFP4").strip(),
        model_revision=os.environ.get(
            "ATLAS_LLM_MODEL_REVISION",
            "ggml-org/gpt-oss-20b-GGUF@ef9b12f2ff56c69cf32153a02784e7a3c88bf524",
        ).strip(),
        quantization=os.environ.get("ATLAS_LLM_QUANTIZATION", "MXFP4").strip(),
        runtime_version=os.environ.get("ATLAS_LLM_RUNTIME_VERSION", "llama.cpp-b10766-cuda13.3").strip(),
        model_sha256=model_sha256,
        verify_model_hash=os.environ.get("ATLAS_LLM_VERIFY_MODEL_HASH", "1").strip() == "1",
        context_size=_int_env("ATLAS_LLM_CTX_SIZE", 32768, 2048, 131072),
        batch_size=_int_env("ATLAS_LLM_BATCH_SIZE", 4096, 256, 16384),
        ubatch_size=_int_env("ATLAS_LLM_UBATCH_SIZE", 4096, 128, 16384),
        reasoning_budget=_int_env("ATLAS_LLM_REASONING_BUDGET", 2048, 0, 16384),
        reasoning_mode=reasoning_mode,
        reasoning_format=reasoning_format,
        start_timeout_s=_float_env("ATLAS_LLM_START_TIMEOUT_S", 240.0, 10.0, 900.0),
        request_timeout_s=_float_env("ATLAS_LLM_REQUEST_TIMEOUT_S", 180.0, 10.0, 900.0),
        min_confidence=_float_env("ATLAS_LLM_MIN_CONFIDENCE", 0.65, 0.0, 1.0),
        process_marker=process_marker,
    )


def _port_is_open(settings: LocalLLMSettings) -> bool:
    try:
        with socket.create_connection((settings.host, settings.port), timeout=0.3):
            return True
    except OSError:
        return False


def local_llm_is_active() -> bool:
    """Return whether the reserved Atlas loopback port is occupied."""

    try:
        return _port_is_open(settings_from_env())
    except ApplicationError:
        return False


def _health(settings: LocalLLMSettings, timeout_s: float = 1.0) -> int | None:
    try:
        with urlrequest.urlopen(f"{settings.base_url}/health", timeout=timeout_s) as response:
            return int(response.status)
    except urlerror.HTTPError as exc:
        return int(exc.code)
    except (OSError, ValueError):
        return None


def _heartbeat(details: dict[str, Any]) -> None:
    try:
        activity.heartbeat(details)
    except RuntimeError:
        # The same bounded server helper is also used by the manual benchmark
        # launcher, which is not itself a Temporal Activity.
        pass


def _process_cleanup(settings: LocalLLMSettings) -> None:
    """Stop only llama-server processes whose command line carries our marker."""

    if os.name == "nt":
        return
    powershell = "powershell.exe"
    if not shutil_which(powershell):
        return
    marker = settings.process_marker.replace("'", "")
    command = (
        "$procs = Get-CimInstance Win32_Process -Filter \"Name = 'llama-server.exe'\"; "
        f"$procs | Where-Object {{ $_.CommandLine -like '*{marker}*' }} | "
        "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"
    )
    try:
        subprocess.run(
            [powershell, "-NoProfile", "-NonInteractive", "-Command", command],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        pass


def shutil_which(name: str) -> str | None:
    # Keep this helper local so importing the Activity does not execute any
    # shell command and so tests can patch it without affecting other modules.
    import shutil

    return shutil.which(name)


def _start_command(settings: LocalLLMSettings) -> list[str]:
    schema = json.dumps(LOCAL_DIAGNOSIS_SCHEMA, ensure_ascii=False, separators=(",", ":"))
    model_arg = str(settings.model_path)
    # WSL can inspect a Windows file through /mnt/c, but a native Windows
    # executable needs a Windows drive path in its argv.
    if model_arg.startswith("/mnt/"):
        drive, _, rest = model_arg[len("/mnt/") :].partition("/")
        model_arg = drive.upper() + ":\\" + rest.replace("/", "\\")
    return [
        str(settings.server_exe),
        "-m",
        model_arg,
        "--host",
        settings.host,
        "--port",
        str(settings.port),
        "--no-webui",
        "--jinja",
        "--reasoning",
        settings.reasoning_mode,
        "--reasoning-format",
        settings.reasoning_format,
        "--reasoning-budget",
        str(settings.reasoning_budget),
        "--ctx-size",
        str(settings.context_size),
        "--batch-size",
        str(settings.batch_size),
        "--ubatch-size",
        str(settings.ubatch_size),
        "--flash-attn",
        "on",
        "--gpu-layers",
        "99",
        "--alias",
        settings.model_id,
        "--json-schema",
        schema,
    ]


async def _start_server(settings: LocalLLMSettings, log_path: Path) -> _RunningServer:
    if not settings.server_exe.is_file():
        raise ApplicationError(
            "Atlas local LLM server executable is missing",
            type="LOCAL_LLM_SERVER_MISSING",
            non_retryable=True,
        )
    if not settings.model_path.is_file():
        raise ApplicationError(
            "Atlas local LLM model file is missing",
            type="LOCAL_LLM_MODEL_MISSING",
            non_retryable=True,
        )
    if settings.verify_model_hash and settings.model_sha256 is not None:
        if _sha256_file(settings.model_path) != settings.model_sha256:
            raise ApplicationError(
                "Atlas local LLM model SHA-256 does not match the pinned deployment value",
                type="LOCAL_LLM_MODEL_HASH_MISMATCH",
                non_retryable=True,
            )
    if _port_is_open(settings):
        raise ApplicationError(
            "reserved Atlas local LLM port is already occupied",
            type="LOCAL_LLM_PORT_BUSY",
            non_retryable=True,
        )

    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("wb")
    env = os.environ.copy()
    # The local model must not inherit credentials or a proxy configuration.
    for name in ("OPENAI_API_KEY", "OPENAI_BASE_URL", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY"):
        env.pop(name, None)
    try:
        process = subprocess.Popen(
            _start_command(settings),
            cwd=str(settings.model_path.parent),
            stdin=subprocess.DEVNULL,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=env,
        )
    except OSError as exc:
        log_handle.close()
        raise ApplicationError(
            "Atlas local LLM server could not be started",
            type="LOCAL_LLM_START_FAILED",
            non_retryable=True,
        ) from exc

    deadline = time.monotonic() + settings.start_timeout_s
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                log_handle.close()
                raise ApplicationError(
                    "Atlas local LLM server exited during startup",
                    type="LOCAL_LLM_START_FAILED",
                    non_retryable=True,
                )
            status = _health(settings)
            if status == 200:
                return _RunningServer(process=process, log_handle=log_handle)
            _heartbeat({"phase": "local_llm_start", "pid": process.pid, "health": status})
            await asyncio.sleep(1.0)
    except asyncio.CancelledError:
        await _stop_server(settings, _RunningServer(process, log_handle))
        raise
    await _stop_server(settings, _RunningServer(process, log_handle))
    raise ApplicationError(
        "Atlas local LLM server did not become healthy before the startup deadline",
        type="LOCAL_LLM_START_TIMEOUT",
        non_retryable=True,
    )


async def _stop_server(settings: LocalLLMSettings, running: _RunningServer | None) -> None:
    if running is not None:
        process = running.process
        if process.poll() is None:
            process.terminate()
            try:
                await asyncio.to_thread(process.wait, 10)
            except subprocess.TimeoutExpired:
                process.kill()
                try:
                    await asyncio.to_thread(process.wait, 10)
                except subprocess.TimeoutExpired:
                    pass
        running.log_handle.close()
    # WSL process handles do not always own the lifetime of a native Windows
    # child. Always perform the marker-scoped Windows cleanup, even when the
    # loopback forwarding socket has already disappeared.
    _process_cleanup(settings)


def _compact_bundle(result: Result, baseline: Diagnosis, spec: ExperimentSpec) -> dict[str, Any]:
    result_payload = result.model_dump(mode="json")
    result_payload["artifacts"] = [
        {
            "kind": artifact.kind,
            "relative_path": artifact.relative_path,
            "sha256": artifact.sha256,
            "size_bytes": artifact.size_bytes,
        }
        for artifact in result.artifacts[:40]
    ]
    return {
        "experiment": {
            "experiment_id": spec.experiment_id,
            "question": spec.question,
            "profile": spec.profile.value,
            "duration_s": spec.duration_s,
            "seed": spec.seed,
            "parameters": spec.parameters,
        },
        "result": result_payload,
        "deterministic_diagnosis": baseline.model_dump(mode="json"),
    }


def _prompt(result: Result, baseline: Diagnosis, spec: ExperimentSpec) -> str:
    bundle = json.dumps(_compact_bundle(result, baseline, spec), ensure_ascii=False, sort_keys=True)
    prompt = (
        "You are the local, offline diagnosis model for a reproducible Go2 MuJoCo research run. "
        "Use only the observed JSON bundle. Do not invent signals, files, causes, or successful "
        "experiments. The deterministic diagnosis is a prior, not permission to ignore evidence. "
        "Return only the final JSON object required by the response schema. Keep the summary and "
        "hypothesis concise. If evidence is insufficient or signals conflict, use UNKNOWN, "
        "recommended_action=escalate, and needs_human_review=true. Never propose changing a "
        "controller algorithm, acceptance threshold, or physics parameter.\n\n"
        + bundle
    )
    if len(prompt.encode("utf-8")) > MAX_LOCAL_PROMPT_BYTES:
        raise ApplicationError(
            "local diagnosis bundle exceeds the bounded prompt size",
            type="LOCAL_LLM_PROMPT_TOO_LARGE",
            non_retryable=True,
        )
    return prompt


def _post_chat(settings: LocalLLMSettings, prompt: str) -> tuple[str, dict[str, Any]]:
    body = {
        "model": settings.model_id,
        "messages": [
            {
                "role": "system",
                "content": "Output only schema-valid JSON. Reason internally but do not expose a narrative outside the JSON object.",
            },
            {"role": "user", "content": prompt},
        ],
        "temperature": 0.1,
        "top_p": 0.9,
        "seed": 42,
        "max_tokens": 1536,
        "stream": False,
        "response_format": {
            "type": "json_schema",
            "json_schema": {
                "name": "go2_local_diagnosis",
                "strict": True,
                "schema": LOCAL_DIAGNOSIS_SCHEMA,
            },
        },
    }
    encoded = json.dumps(body, ensure_ascii=False).encode("utf-8")
    request = urlrequest.Request(
        f"{settings.base_url}/v1/chat/completions",
        data=encoded,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlrequest.urlopen(request, timeout=settings.request_timeout_s) as response:
            raw = response.read(MAX_LOCAL_RESPONSE_BYTES + 1)
    except (OSError, urlerror.URLError, TimeoutError) as exc:
        raise ApplicationError(
            "Atlas local LLM request failed",
            type="LOCAL_LLM_REQUEST_FAILED",
            non_retryable=True,
        ) from exc
    if len(raw) > MAX_LOCAL_RESPONSE_BYTES:
        raise ApplicationError(
            "Atlas local LLM response exceeded the bounded size",
            type="LOCAL_LLM_RESPONSE_TOO_LARGE",
            non_retryable=True,
        )
    try:
        response_payload = json.loads(raw.decode("utf-8"))
        message = response_payload["choices"][0]["message"]
        content = message.get("content", "")
        if isinstance(content, list):
            content = "".join(
                str(item.get("text", "")) if isinstance(item, dict) else str(item)
                for item in content
            )
        if not isinstance(content, str) or not content.strip():
            raise ValueError("empty assistant content")
    except (KeyError, IndexError, TypeError, UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        raise ApplicationError(
            "Atlas local LLM returned an invalid OpenAI-compatible response",
            type="LOCAL_LLM_RESPONSE_INVALID",
            non_retryable=True,
        ) from exc
    return content, response_payload


def _json_object(text: str) -> dict[str, Any]:
    candidate = text.strip()
    if candidate.startswith("```"):
        lines = candidate.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        candidate = "\n".join(lines).strip()
    try:
        value = json.loads(candidate)
    except json.JSONDecodeError:
        # Some compatible servers may leave a short delimiter after the
        # constrained object. Decode the first object without accepting prose
        # as a diagnosis.
        start = candidate.find("{")
        if start < 0:
            raise
        value, end = json.JSONDecoder().raw_decode(candidate[start:])
        if candidate[start + end :].strip():
            raise ValueError("trailing non-JSON content")
    if not isinstance(value, dict):
        raise ValueError("local LLM output must be a JSON object")
    return value


def _safe_confidence(raw: Any) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, min(1.0, value))


def normalize_local_output(
    result: Result,
    baseline: Diagnosis,
    raw: dict[str, Any],
    receipt: InferenceReceipt,
    min_confidence: float,
) -> Diagnosis:
    try:
        failure_class = FailureClass(str(raw.get("failure_class", FailureClass.UNKNOWN.value)))
    except ValueError:
        failure_class = FailureClass.UNKNOWN
    try:
        action = ActionType(str(raw.get("recommended_action", ActionType.ESCALATE.value)))
    except ValueError:
        action = ActionType.ESCALATE
    confidence = _safe_confidence(raw.get("confidence", 0.0))
    if result.verdict == Verdict.PASS_DEV:
        failure_class = FailureClass.PASS_DEV
        action = ActionType.CHECKPOINT
    elif result.verdict == Verdict.RUNNER_FAILURE:
        failure_class = FailureClass.RUNNER_FAILURE
        action = ActionType.ESCALATE

    evidence: list[EvidencePoint] = []
    raw_evidence = raw.get("evidence", [])
    if isinstance(raw_evidence, list):
        for item in raw_evidence[:20]:
            if isinstance(item, dict):
                signal = str(item.get("signal", "local_llm.evidence"))[:120]
                value = str(item.get("value", ""))[:500]
                evidence.append(EvidencePoint(signal=signal, value=value, source="local_llm"))
    if not evidence:
        evidence.append(
            EvidencePoint(signal="local_llm.output", value="validated structured response", source="local_llm")
        )

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
    summary = str(raw.get("summary", "Local model returned no summary."))[:2000]
    hypothesis = raw.get("hypothesis")
    if hypothesis is not None:
        hypothesis = str(hypothesis)[:2000]
    needs_review = bool(raw.get("needs_human_review", False))
    unresolved = failure_class == FailureClass.UNKNOWN or confidence < min_confidence
    if failure_class in {FailureClass.UNKNOWN, FailureClass.FAIL_SAFE_STOP, FailureClass.RUNNER_FAILURE}:
        needs_review = True
    if unresolved:
        action = ActionType.ESCALATE
    return Diagnosis(
        experiment_id=result.experiment_id,
        source=DiagnosisSource.LOCAL_LLM,
        failure_class=failure_class,
        confidence=confidence,
        summary=summary,
        evidence=evidence,
        hypothesis=hypothesis,
        recommended_action=action,
        recommended_profile=recommended_profile,
        recommended_parameters=parameters,
        requires_codex=unresolved or baseline.requires_codex and failure_class == FailureClass.UNKNOWN,
        requires_human_review=needs_review,
        inference=receipt,
    )


def local_llm_failure_diagnosis(result: Result, error_code: str, summary: str) -> dict[str, Any]:
    return Diagnosis(
        experiment_id=result.experiment_id,
        source=DiagnosisSource.LOCAL_LLM_ERROR,
        failure_class=FailureClass.UNKNOWN,
        confidence=0.0,
        summary=summary,
        evidence=[EvidencePoint(signal="local_llm.status", value=error_code, source="result")],
        recommended_action=ActionType.ESCALATE,
        requires_codex=True,
        requires_human_review=True,
        error_code=error_code,
    ).model_dump(mode="json")


def _write_exchange(settings: LocalLLMSettings, spec: ExperimentSpec, prompt: str, content: str) -> None:
    raw_root = os.environ.get("ATLAS_ARTIFACT_ROOT", "").strip()
    if not raw_root:
        return
    root = Path(raw_root).expanduser().resolve()
    if root == Path("/"):
        return
    path = root / spec.experiment_id / "local_llm" / "exchange.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": "local_llm_exchange.v1",
        "model_id": settings.model_id,
        "model_revision": settings.model_revision,
        "quantization": settings.quantization,
        "runtime_version": settings.runtime_version,
        "prompt_sha256": hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
        "response_sha256": hashlib.sha256(content.encode("utf-8")).hexdigest(),
        "response": content,
    }
    encoded = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    if len(encoded) <= MAX_LOCAL_RESPONSE_BYTES:
        path.write_bytes(encoded + b"\n")


@activity.defn(name=LOCAL_LLM_ACTIVITY_NAME)
async def diagnose_with_local_llm(payload: dict[str, Any]) -> dict[str, Any]:
    try:
        result = Result.model_validate(payload.get("result"))
        baseline = Diagnosis.model_validate(payload.get("baseline"))
        spec = ExperimentSpec.model_validate(payload.get("spec"))
    except (AttributeError, TypeError, ValidationError) as exc:
        raise ApplicationError(
            "local LLM Activity payload is invalid",
            type="INVALID_LOCAL_LLM_PAYLOAD",
            non_retryable=True,
        ) from exc
    if spec.execution_mode.value != "atlas" or not spec.allow_local_llm:
        return local_llm_failure_diagnosis(
            result,
            "LOCAL_LLM_NOT_ALLOWED",
            "Local inference was not enabled for this Atlas experiment.",
        )

    try:
        settings = settings_from_env()
        prompt = _prompt(result, baseline, spec)
    except ApplicationError as exc:
        return local_llm_failure_diagnosis(result, str(exc.type or "LOCAL_LLM_ERROR"), str(exc))
    root = Path(os.environ.get("ATLAS_ARTIFACT_ROOT", ".")).expanduser().resolve()
    log_path = root / spec.experiment_id / "local_llm" / "server.log"
    running: _RunningServer | None = None
    try:
        running = await _start_server(settings, log_path)
        started = time.monotonic()
        content, response_payload = await asyncio.to_thread(_post_chat, settings, prompt)
        latency_ms = max(0, int(round((time.monotonic() - started) * 1000)))
        raw = _json_object(content)
        usage = response_payload.get("usage", {})
        if not isinstance(usage, dict):
            usage = {}
        prompt_tokens = int(usage.get("prompt_tokens", 0) or 0)
        completion_tokens = int(usage.get("completion_tokens", 0) or 0)
        details = usage.get("completion_tokens_details", {})
        reasoning_tokens = int(details.get("reasoning_tokens", 0) or 0) if isinstance(details, dict) else 0
        receipt = InferenceReceipt(
            engine="llama.cpp",
            model_id=settings.model_id,
        model_revision=settings.model_revision,
        quantization=settings.quantization,
        runtime_version=settings.runtime_version,
        model_sha256=settings.model_sha256,
            prompt_sha256=hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
            response_sha256=hashlib.sha256(content.encode("utf-8")).hexdigest(),
            context_tokens=prompt_tokens + completion_tokens,
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens,
            reasoning_tokens=reasoning_tokens,
            latency_ms=latency_ms,
        )
        _write_exchange(settings, spec, prompt, content)
        return normalize_local_output(result, baseline, raw, receipt, settings.min_confidence).model_dump(
            mode="json"
        )
    except ApplicationError as exc:
        return local_llm_failure_diagnosis(result, str(exc.type or "LOCAL_LLM_ERROR"), str(exc))
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        return local_llm_failure_diagnosis(
            result,
            "LOCAL_LLM_OUTPUT_INVALID",
            "Local inference did not produce a validated diagnosis; no automatic action was taken.",
        )
    finally:
        await _stop_server(settings, running)
