from research_orchestrator.activities.agent import classify_result_payload, make_fixture_result
from research_orchestrator.activities.local_llm import _json_object, normalize_local_output
from research_orchestrator.schemas.models import (
    Diagnosis,
    DiagnosisSource,
    ExperimentSpec,
    InferenceReceipt,
    Result,
    SourceRevision,
)


def _spec(verdict: str = "UNKNOWN") -> ExperimentSpec:
    return ExperimentSpec(
        experiment_id="local-llm-smoke",
        question="exercise local diagnosis normalization",
        policy_id="b0",
        profile="b0-development",
        execution_mode="atlas",
        duration_s=1.0,
        wall_timeout_s=31.0,
        seed=42,
        source=SourceRevision(git_sha="a" * 40),
        control_plane=SourceRevision(git_sha="b" * 40),
        parameters={"scenario": "steps", "domain_id": 190},
        allow_local_llm=True,
        requested_at="2026-09-03T00:00:00Z",
    )


def _receipt() -> InferenceReceipt:
    return InferenceReceipt(
        engine="llama.cpp",
        model_id="test-model",
        model_revision="test@sha",
        quantization="Q4_K_M",
        runtime_version="llama.cpp-test",
        prompt_sha256="1" * 64,
        response_sha256="2" * 64,
        context_tokens=20,
        prompt_tokens=12,
        completion_tokens=8,
        latency_ms=123,
    )


def test_json_object_accepts_fenced_json_only():
    assert _json_object('```json\n{"failure_class":"UNKNOWN"}\n```') == {
        "failure_class": "UNKNOWN"
    }


def test_low_confidence_local_result_escalates():
    fixture_spec = _spec().model_dump(mode="json")
    fixture_spec["execution_mode"] = "fixture"
    spec = ExperimentSpec.model_validate(fixture_spec)
    result = make_fixture_result(spec.model_dump(mode="json"))
    result = Result.model_validate(result)
    baseline = Diagnosis.model_validate(classify_result_payload(result.model_dump(mode="json")))
    diagnosis = normalize_local_output(
        result,
        baseline,
        {
            "failure_class": "UNKNOWN",
            "confidence": 0.4,
            "summary": "Signals conflict.",
            "evidence": [{"signal": "result.verdict", "value": "UNKNOWN"}],
            "hypothesis": None,
            "recommended_action": "rerun",
            "recommended_profile": None,
            "recommended_parameters": {},
            "needs_human_review": True,
        },
        _receipt(),
        0.65,
    )
    assert diagnosis.source == DiagnosisSource.LOCAL_LLM
    assert diagnosis.requires_codex is True
    assert diagnosis.recommended_action.value == "escalate"
