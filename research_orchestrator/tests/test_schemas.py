from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from research_orchestrator.schemas.models import (
    ExecutionMode,
    ExperimentSpec,
    PolicyId,
    ProbeProfile,
    SourceRevision,
)


def _spec(**overrides):
    data = dict(
        experiment_id="schema-smoke",
        question="check schema",
        policy_id=PolicyId.B0,
        profile=ProbeProfile.B0_DEVELOPMENT,
        execution_mode=ExecutionMode.FIXTURE,
        duration_s=1.0,
        wall_timeout_s=2.0,
        seed=0,
        source=SourceRevision(git_sha="a" * 40),
        requested_at=datetime.now(timezone.utc).isoformat(),
    )
    data.update(overrides)
    return ExperimentSpec(**data)


def test_experiment_is_json_safe_and_versioned():
    spec = _spec(parameters={"fixture_verdict": "PASS_DEV", "repeat": 2})
    payload = spec.model_dump(mode="json")
    assert payload["schema_version"] == "experiment.v1"
    assert payload["source"]["git_sha"] == "a" * 40


def test_experiment_rejects_mismatched_policy():
    with pytest.raises(ValidationError):
        _spec(policy_id=PolicyId.B1)


def test_experiment_rejects_nested_parameters():
    with pytest.raises(ValidationError):
        _spec(parameters={"argv": ["arbitrary", "shell"]})
