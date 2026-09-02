"""Versioned, JSON-safe contracts shared by Temporal workers.

The workflow passes plain JSON dictionaries to keep the Temporal workflow
sandbox deterministic. These Pydantic models validate every activity boundary
and are also the public contract for future Atlas adapters.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class SchemaBase(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class PolicyId(str, Enum):
    B0 = "b0"
    B1 = "b1"


class ProbeProfile(str, Enum):
    B0_DEVELOPMENT = "b0-development"
    B0_HOLDOUT = "b0-holdout"
    B1_PROBE = "b1-probe"


class ExecutionMode(str, Enum):
    FIXTURE = "fixture"
    ATLAS = "atlas"


class RunStatus(str, Enum):
    COMPLETED = "completed"
    FAILED = "failed"
    TIMEOUT = "timeout"
    REJECTED = "rejected"
    DEFERRED = "deferred"


class Verdict(str, Enum):
    PASS_DEV = "PASS_DEV"
    FAIL_TIMING = "FAIL_TIMING"
    FAIL_CONTACT = "FAIL_CONTACT"
    FAIL_WBC = "FAIL_WBC"
    FAIL_PLANNER = "FAIL_PLANNER"
    FAIL_SAFE_STOP = "FAIL_SAFE_STOP"
    RUNNER_FAILURE = "RUNNER_FAILURE"
    UNKNOWN = "UNKNOWN"


class FailureClass(str, Enum):
    PASS_DEV = "PASS_DEV"
    FAIL_TIMING = "FAIL_TIMING"
    FAIL_CONTACT = "FAIL_CONTACT"
    FAIL_WBC = "FAIL_WBC"
    FAIL_PLANNER = "FAIL_PLANNER"
    FAIL_SAFE_STOP = "FAIL_SAFE_STOP"
    RUNNER_FAILURE = "RUNNER_FAILURE"
    UNKNOWN = "UNKNOWN"


class ActionType(str, Enum):
    ADVANCE = "advance"
    RERUN = "rerun"
    COLLECT_EVIDENCE = "collect_evidence"
    CHECKPOINT = "checkpoint"
    ESCALATE = "escalate"
    STOP = "stop"


class DiagnosisSource(str, Enum):
    DETERMINISTIC = "deterministic"
    CODEX = "codex"
    CODEX_ERROR = "codex_error"


def _validate_scalar_map(values: dict[str, Any]) -> dict[str, Any]:
    for key, value in values.items():
        if not key or len(key) > 100 or key.startswith("_"):
            raise ValueError(f"invalid parameter name: {key!r}")
        if not isinstance(value, (str, int, float, bool)) or isinstance(value, (list, dict)):
            raise ValueError(f"parameter {key!r} must be a JSON scalar")
    return values


class SourceRevision(SchemaBase):
    repository: str = Field(default="kairoi-k/go2-mujoco-control", min_length=1, max_length=200)
    git_sha: str = Field(pattern=r"^[0-9a-f]{40}$")
    git_ref: str = Field(default="main", pattern=r"^[A-Za-z0-9._/-]{1,200}$")
    dirty: bool = False


class ArtifactRef(SchemaBase):
    kind: Literal["log", "csv", "json", "trace", "binary", "other"]
    relative_path: str = Field(min_length=1, max_length=400)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(ge=0)

    @field_validator("relative_path")
    @classmethod
    def relative_path_must_be_safe(cls, value: str) -> str:
        if (
            value.startswith(("/", "~"))
            or "\\" in value
            or any(part in {"", ".", ".."} for part in value.split("/"))
        ):
            raise ValueError("artifact paths must be normalized repository-relative paths")
        return value


class ExperimentSpec(SchemaBase):
    schema_version: Literal["experiment.v1"] = "experiment.v1"
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{2,63}$")
    question: str = Field(min_length=1, max_length=1000)
    policy_id: PolicyId
    profile: ProbeProfile
    execution_mode: ExecutionMode = ExecutionMode.FIXTURE
    duration_s: float = Field(gt=0, le=3600)
    wall_timeout_s: float = Field(gt=0, le=7200)
    seed: int = Field(ge=0, le=2_147_483_647)
    source: SourceRevision
    parameters: dict[str, Any] = Field(default_factory=dict)
    atlas_task_queue: str = Field(default="atlas", pattern=r"^[a-z][a-z0-9._-]{0,63}$")
    allow_codex: bool = False
    codex_timeout_s: int = Field(default=90, ge=10, le=600)
    requested_at: str = Field(min_length=1, max_length=80)

    @field_validator("parameters")
    @classmethod
    def parameters_must_be_scalars(cls, value: dict[str, Any]) -> dict[str, Any]:
        return _validate_scalar_map(value)

    @model_validator(mode="after")
    def validate_policy_and_timeout(self) -> "ExperimentSpec":
        expected_policy = PolicyId.B0 if self.profile.value.startswith("b0-") else PolicyId.B1
        if self.policy_id != expected_policy:
            raise ValueError(f"profile {self.profile.value!r} requires policy {expected_policy.value!r}")
        if self.wall_timeout_s < self.duration_s:
            raise ValueError("wall_timeout_s must be at least duration_s")
        return self


class ResultMetrics(SchemaBase):
    safe_stop: bool = False
    first_failure_s: float | None = Field(default=None, ge=0)
    plan_published: bool | None = None
    plan_consumed: bool | None = None
    terrain_actuation: bool | None = None
    q_error_max_rad: float | None = Field(default=None, ge=0)
    foot_error_max_m: float | None = Field(default=None, ge=0)
    wbc_saturation_fraction: float | None = Field(default=None, ge=0, le=1)
    wall_clock_rate_hz: float | None = Field(default=None, ge=0)
    extra: dict[str, Any] = Field(default_factory=dict)

    @field_validator("extra")
    @classmethod
    def extra_must_be_scalars(cls, value: dict[str, Any]) -> dict[str, Any]:
        return _validate_scalar_map(value)


class EvidencePoint(SchemaBase):
    signal: str = Field(min_length=1, max_length=120)
    value: str = Field(max_length=500)
    source: Literal["result", "artifact", "codex"]


class FailureWindow(SchemaBase):
    schema_version: Literal["failure_window.v1"] = "failure_window.v1"
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{2,63}$")
    start_s: float = Field(ge=0)
    end_s: float = Field(ge=0)
    evidence: list[EvidencePoint] = Field(default_factory=list, max_length=50)
    source_artifact: ArtifactRef | None = None

    @model_validator(mode="after")
    def window_must_be_ordered(self) -> "FailureWindow":
        if self.end_s < self.start_s:
            raise ValueError("failure window end_s must be >= start_s")
        return self


class Result(SchemaBase):
    schema_version: Literal["result.v1"] = "result.v1"
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{2,63}$")
    status: RunStatus
    verdict: Verdict
    runner: Literal["fixture", "atlas"]
    started_at: str = Field(min_length=1, max_length=80)
    finished_at: str = Field(min_length=1, max_length=80)
    duration_s: float = Field(ge=0)
    source: SourceRevision
    metrics: ResultMetrics
    artifacts: list[ArtifactRef] = Field(default_factory=list, max_length=100)
    failure_window: FailureWindow | None = None
    notes: list[str] = Field(default_factory=list, max_length=50)


class Diagnosis(SchemaBase):
    schema_version: Literal["diagnosis.v1"] = "diagnosis.v1"
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{2,63}$")
    source: DiagnosisSource
    failure_class: FailureClass
    confidence: float = Field(ge=0, le=1)
    summary: str = Field(min_length=1, max_length=2000)
    evidence: list[EvidencePoint] = Field(default_factory=list, max_length=20)
    hypothesis: str | None = Field(default=None, max_length=2000)
    recommended_action: ActionType
    recommended_profile: ProbeProfile | None = None
    recommended_parameters: dict[str, Any] = Field(default_factory=dict)
    requires_codex: bool = False
    requires_human_review: bool = False
    error_code: str | None = Field(default=None, pattern=r"^[A-Z0-9_-]{1,80}$")

    @field_validator("recommended_parameters")
    @classmethod
    def recommended_parameters_must_be_scalars(cls, value: dict[str, Any]) -> dict[str, Any]:
        return _validate_scalar_map(value)


class NextAction(SchemaBase):
    schema_version: Literal["next_action.v1"] = "next_action.v1"
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{2,63}$")
    action: ActionType
    rationale: str = Field(min_length=1, max_length=2000)
    proposed_profile: ProbeProfile | None = None
    proposed_parameters: dict[str, Any] = Field(default_factory=dict)
    requires_approval: bool = True
    guardrails: list[str] = Field(default_factory=list, max_length=20)

    @field_validator("proposed_parameters")
    @classmethod
    def proposed_parameters_must_be_scalars(cls, value: dict[str, Any]) -> dict[str, Any]:
        return _validate_scalar_map(value)


class ActivityReceipt(SchemaBase):
    schema_version: Literal["activity.v1"] = "activity.v1"
    activity_name: str = Field(min_length=1, max_length=100)
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{2,63}$")
    status: Literal["completed", "failed", "deferred"]
    source: SourceRevision
    message: str = Field(min_length=1, max_length=1000)
    artifacts: list[ArtifactRef] = Field(default_factory=list, max_length=100)


class WorkflowResult(SchemaBase):
    schema_version: Literal["research_run.v1"] = "research_run.v1"
    experiment: ExperimentSpec
    result: Result
    diagnosis: Diagnosis
    next_action: NextAction
