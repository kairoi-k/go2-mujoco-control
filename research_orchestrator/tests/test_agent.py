from research_orchestrator.activities.agent import (
    classify_campaign_payload,
    classify_result_payload,
    decide_next_action_payload,
    make_fixture_result,
)
from research_orchestrator.schemas.models import ExperimentSpec, SourceRevision


def _spec(verdict="PASS_DEV"):
    return ExperimentSpec(
        experiment_id="agent-smoke",
        question="exercise deterministic fixture path",
        policy_id="b0",
        profile="b0-development",
        execution_mode="fixture",
        duration_s=1.0,
        wall_timeout_s=2.0,
        seed=0,
        source=SourceRevision(git_sha="b" * 40),
        parameters={"fixture_verdict": verdict},
        requested_at="2026-09-03T00:00:00Z",
    )


def test_pass_stops_at_human_checkpoint():
    spec = _spec()
    result = make_fixture_result(spec.model_dump(mode="json"))
    diagnosis = classify_result_payload(result)
    action = decide_next_action_payload(spec.model_dump(mode="json"), diagnosis)
    assert diagnosis["failure_class"] == "PASS_DEV"
    assert action["action"] == "checkpoint"
    assert action["requires_approval"] is True


def test_unknown_failure_escalates_without_codex():
    spec = _spec("UNKNOWN")
    result = make_fixture_result(spec.model_dump(mode="json"))
    diagnosis = classify_result_payload(result)
    action = decide_next_action_payload(spec.model_dump(mode="json"), diagnosis)
    assert diagnosis["requires_codex"] is True
    assert action["action"] == "escalate"
    assert action["requires_approval"] is True


def test_autonomous_pass_advances_without_approval():
    spec = _spec().model_copy(update={"autonomous": True})
    result = make_fixture_result(spec.model_dump(mode="json"))
    diagnosis = classify_result_payload(result)
    action = decide_next_action_payload(spec.model_dump(mode="json"), diagnosis)
    assert action["action"] == "advance"
    assert action["proposed_profile"] == "b0-holdout"
    assert action["requires_approval"] is False


def test_autonomous_campaign_pass_advances_b0_to_b1():
    spec = _spec().model_copy(update={"autonomous": True})
    campaign = {
        "campaign_id": "agent-smoke-b0",
        "experiment_id": spec.experiment_id,
        "stage": "b0",
        "contract": "b0-contract-v1.2",
        "manifest_sha256": "a" * 64,
        "status": "completed",
        "acceptance_status": "PASS",
        "prerequisite_passed": True,
        "members_total": 18,
        "members_passed": 18,
        "members_failed": 0,
        "source": spec.source.model_dump(mode="json"),
        "members": [],
        "failure_reasons": [],
        "artifacts": [],
    }
    diagnosis = classify_campaign_payload({"spec": spec.model_dump(mode="json"), "campaign": campaign})
    action = decide_next_action_payload(spec.model_dump(mode="json"), diagnosis)
    assert action["action"] == "advance"
    assert action["proposed_profile"] == "b1-probe"
    assert action["requires_approval"] is False
