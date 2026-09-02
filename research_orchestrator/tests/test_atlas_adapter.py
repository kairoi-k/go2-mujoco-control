import pytest
from temporalio.exceptions import ApplicationError

from research_orchestrator.activities.atlas import DEV_SCENARIO_DURATIONS, _copy_file, _scenario
from research_orchestrator.schemas.models import ExperimentSpec


def _spec(**overrides):
    data = dict(
        experiment_id="atlas-adapter-smoke",
        question="validate fixed Atlas operation selection",
        policy_id="b0",
        profile="b0-development",
        execution_mode="atlas",
        duration_s=DEV_SCENARIO_DURATIONS["accel_1_to_3"],
        wall_timeout_s=100.0,
        seed=0,
        source={"git_sha": "c" * 40, "git_ref": "phase2-b1-b3", "dirty": False},
        control_plane={"git_sha": "d" * 40, "git_ref": "wip/control-plane", "dirty": False},
        parameters={"scenario": "accel_1_to_3", "domain_id": 190},
        requested_at="2026-09-03T00:00:00Z",
    )
    data.update(overrides)
    return ExperimentSpec.model_validate(data)


def test_development_profile_maps_to_fixed_duration_and_domain():
    assert _scenario(_spec()) == ("accel_1_to_3", 40.0, 190)


def test_development_profile_rejects_partial_duration():
    with pytest.raises(ApplicationError):
        _scenario(_spec(duration_s=10.0))


def test_development_profile_rejects_non_development_domain():
    with pytest.raises(ApplicationError):
        _scenario(_spec(parameters={"scenario": "accel_1_to_3", "domain_id": 22}))


def test_copy_file_accepts_a_log_already_in_the_artifact_root(tmp_path):
    root = tmp_path / "artifacts"
    source = root / "atlas-adapter-smoke" / "atlas_process.log"
    source.parent.mkdir(parents=True)
    source.write_text("ok\n", encoding="utf-8")

    ref = _copy_file(source, source, root)

    assert ref is not None
    assert ref.relative_path == "atlas-adapter-smoke/atlas_process.log"
