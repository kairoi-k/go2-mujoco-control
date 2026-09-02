from research_orchestrator.activities.atlas import ATLAS_ACTIVITY_NAMES


def test_atlas_activity_set_is_explicit_and_minimal():
    assert ATLAS_ACTIVITY_NAMES == (
        "build_source",
        "run_unit_tests",
        "run_dev_probe",
        "run_b0_member",
        "run_b0_holdout",
        "run_b1_probe",
        "extract_failure_window",
    )
