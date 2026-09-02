"""Load versioned policy files without accepting command strings from input."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field

from ..schemas.models import ExperimentSpec, ProbeProfile


class Policy(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    schema_version: Literal["policy.v1"]
    policy_id: Literal["b0", "b1"]
    status: Literal["draft", "wip", "accepted"]
    purpose: str = Field(min_length=1, max_length=1000)
    allowed_profiles: list[ProbeProfile] = Field(min_length=1)
    allowed_activity_names: list[str] = Field(min_length=1)
    max_duration_s: float = Field(gt=0, le=3600)
    controller_mutation_allowed: bool
    acceptance_claim_allowed: bool
    requires_human_approval_for_formal: bool
    notes: list[str] = Field(default_factory=list)


def _policy_dir() -> Path:
    return Path(__file__).resolve().parent


def load_policy(policy_id: str) -> Policy:
    path = _policy_dir() / f"{policy_id}.yaml"
    if not path.is_file():
        raise ValueError(f"unknown research policy: {policy_id!r}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return Policy.model_validate(data)


def validate_spec_against_policy(spec: ExperimentSpec, policy: Policy) -> None:
    if spec.policy_id.value != policy.policy_id:
        raise ValueError(f"spec policy {spec.policy_id.value!r} does not match loaded policy")
    if spec.profile not in policy.allowed_profiles:
        raise ValueError(f"profile {spec.profile.value!r} is not allowed by {policy.policy_id}")
    if spec.duration_s > policy.max_duration_s:
        raise ValueError(
            f"duration_s={spec.duration_s} exceeds {policy.policy_id} limit {policy.max_duration_s}"
        )
    if policy.controller_mutation_allowed:
        raise ValueError("research policies must not enable controller mutation in this phase")
    if policy.acceptance_claim_allowed:
        raise ValueError("local orchestration policies cannot grant an acceptance claim")
