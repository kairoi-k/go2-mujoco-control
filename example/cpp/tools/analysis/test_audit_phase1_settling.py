#!/usr/bin/env python3
"""Focused synthetic tests for audit_phase1_settling.py."""

from __future__ import annotations

import importlib.util
import math
import unittest
import tempfile
from pathlib import Path


ANALYSIS_DIR = Path(__file__).parent

def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_audit = load_module("audit_phase1_settling", ANALYSIS_DIR / "audit_phase1_settling.py")
_legacy = load_module(
    "analyze_phase1_velocity",
    ANALYSIS_DIR.parent.parent / "scripts" / "analyze_phase1_velocity.py",
)


def synthetic_rows(
    end_s: float,
    measured,
    *,
    start_s: float = 0.0,
    step_s: float = 0.01,
):
    rows = []
    sample_count = int(round((end_s - start_s) / step_s))
    for index in range(sample_count + 1):
        time_s = start_s + index * step_s
        rows.append(
            {
                "cmd_time_s": str(time_s),
                "velocity_command_measured_mps": str(measured(time_s)),
            }
        )
    return rows


class SettlingAuditTests(unittest.TestCase):
    @staticmethod
    def profile_one_transition(end_s: float = 4.0):
        return [(0.0, 0.0), (2.0, 1.0), (end_s, 1.0)]

    def test_csv_missing_active_column_cannot_prove_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "data.csv"
            path.write_text("cmd_time_s,velocity_command_measured_mps,velocity_command_gait_regime\n0,1,continuous-trot\n")
            with self.assertRaisesRegex(ValueError, "velocity_command_active"):
                _audit.load_active_rows(path)

    def test_unsettled_with_complete_tail_fails(self):
        rows = synthetic_rows(4.0, lambda _: 1.2)
        report = _audit.audit_transitions(
            rows, self.profile_one_transition(), settling_limit_s=8.2
        )
        transition = report["transitions"][0]
        self.assertEqual(transition["status"], "unsettled")
        self.assertIsNone(transition["observed_settling"])
        self.assertGreaterEqual(transition["complete_finite_candidate_count"], 1)
        self.assertEqual(report["audit_status"], "FAIL")

    def test_last_window_without_full_second_is_coverage_missing(self):
        rows = synthetic_rows(2.5, lambda _: 1.0)
        report = _audit.audit_transitions(
            rows, self.profile_one_transition(end_s=3.0), settling_limit_s=8.2
        )
        transition = report["transitions"][0]
        self.assertEqual(transition["status"], "coverage_missing")
        self.assertEqual(transition["complete_finite_candidate_count"], 0)
        self.assertEqual(report["audit_status"], "FAIL")

    def test_nonfinite_sample_cannot_prove_settling(self):
        def measured(time_s: float) -> float:
            return float("nan") if math.isclose(time_s, 2.5) else 1.0

        rows = synthetic_rows(3.0, measured)
        report = _audit.audit_transitions(
            rows, self.profile_one_transition(), settling_limit_s=8.2
        )
        transition = report["transitions"][0]
        self.assertEqual(transition["status"], "nonfinite_missing")
        self.assertEqual(transition["complete_finite_candidate_count"], 0)
        self.assertGreater(transition["nonfinite_full_candidate_count"], 0)
        self.assertEqual(report["input_validation"]["invalid_measured_rows"], 1)
        self.assertEqual(report["audit_status"], "FAIL")

    def test_mixed_pass_fail_matches_legacy_but_fails_closed(self):
        profile = [(0.0, 0.0), (2.0, 1.0), (4.0, 2.0), (6.0, 2.0)]

        def measured(time_s: float) -> float:
            if time_s < 2.0:
                return 0.0
            if time_s < 4.0:
                return 1.0
            return 2.2

        rows = synthetic_rows(6.0, measured)
        legacy = _legacy.transition_metrics(rows, profile, 0.0)
        report = _audit.audit_transitions(rows, profile, settling_limit_s=8.2)
        statuses = [item["status"] for item in report["transitions"]]
        legacy_times = [
            item["settling_time_s"]
            for item in legacy
            if math.isfinite(item["settling_time_s"])
        ]
        self.assertEqual(statuses, ["observed", "unsettled"])
        self.assertEqual(len(legacy), 2)
        self.assertEqual(sum(math.isfinite(item["settling_time_s"]) for item in legacy), 1)
        self.assertLessEqual(max(legacy_times), 8.2)
        self.assertTrue(report["legacy_finite_aggregate_pass"])
        self.assertEqual(report["audit_status"], "FAIL")

    def test_irregular_two_millisecond_samples_use_bracketing_sample(self):
        rows = []
        time_s = 0.0
        while time_s < 4.0:
            rows.append(
                {
                    "cmd_time_s": str(time_s),
                    "velocity_command_measured_mps": "1.0",
                }
            )
            time_s += 0.002 if len(rows) % 2 else 0.003
        report = _audit.audit_transitions(
            rows, self.profile_one_transition(), settling_limit_s=8.2
        )
        transition = report["transitions"][0]
        self.assertEqual(transition["status"], "observed")
        self.assertEqual(report["input_validation_status"], "PASS")
        self.assertGreaterEqual(
            transition["observed_settling"]["tail_observed_span_s"], 1.0 - 1.0e-6
        )

    def test_large_sampling_gap_fails_closed(self):
        rows = [
            {"cmd_time_s": str(time_s), "velocity_command_measured_mps": "1.0"}
            for time_s in (0.0, 2.0, 3.0, 4.0)
        ]
        report = _audit.audit_transitions(
            rows, self.profile_one_transition(), settling_limit_s=8.2
        )
        self.assertEqual(report["transitions"][0]["status"], "sampling_gap")
        self.assertEqual(report["audit_status"], "FAIL")

    def test_settling_after_per_transition_deadline_fails(self):
        profile = [(0.0, 0.0), (2.0, 1.0), (12.0, 1.0)]
        rows = synthetic_rows(12.0, lambda t: 1.0 if t >= 11.0 else 1.2)
        report = _audit.audit_transitions(rows, profile, settling_limit_s=8.2)
        transition = report["transitions"][0]
        self.assertEqual(transition["status"], "deadline_missed")
        self.assertEqual(report["audit_status"], "FAIL")

    def test_profile_bounds_and_unknown_scenario_fail_closed(self):
        rows = synthetic_rows(4.0, lambda _: 1.0)
        with self.assertRaises(ValueError):
            _audit.audit_transitions(
                rows, [(0.0, 0.0), (2.0, math.nan), (4.0, 1.0)]
            )
        with self.assertRaises(ValueError):
            _audit.audit_transitions(
                rows, [(0.0, 0.0), (2.0, 1.0), (2.0, 1.0)]
            )
        with self.assertRaises(ValueError):
            _audit.settling_limit_for_scenario("unknown")
    def test_nonfinite_time_is_input_failure_not_dropped(self):
        rows = synthetic_rows(4.0, lambda _: 1.0)
        rows.insert(15, {
            "cmd_time_s": "nan",
            "velocity_command_measured_mps": "1.0",
        })
        report = _audit.audit_transitions(
            rows, self.profile_one_transition(), settling_limit_s=8.2
        )
        self.assertEqual(report["input_validation"]["invalid_time_rows"], 1)
        self.assertEqual(report["input_validation_status"], "FAIL")
        self.assertEqual(report["audit_status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
