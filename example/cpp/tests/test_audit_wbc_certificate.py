#!/usr/bin/env python3
"""Synthetic tests for the read-only WBC certificate audit."""
from __future__ import annotations
import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

ANALYZER_PATH = (
    Path(__file__).parents[1]
    / "tools"
    / "analysis"
    / "audit_wbc_certificate.py"
)
SPEC = importlib.util.spec_from_file_location(
    "audit_wbc_certificate", ANALYZER_PATH
)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import {ANALYZER_PATH}")
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)

def row(
    *,
    stage=2,
    active=1,
    attempt_feasible=1,
    selected_feasible=1,
    attempt_checked=1,
    selected_checked=1,
    attempt_valid=1,
    selected_valid=1,
    legacy_ok=1,
    converged=1,
    reused=0,
    selected_failure=0,
    selected_force=0.0,
    selected_joint=0.4,
):
    result = {
        "motion_stage": str(stage),
        "velocity_command_active": str(active),
        "state_tick_s": "1.0",
        "wbc_full_id_ok": str(legacy_ok),
        "wbc_full_id_attempt_qp_converged": str(converged),
        "wbc_full_cert_selected_reused": str(reused),
        "wbc_full_certificate_elapsed_us": "12.0",
    }
    for prefix, feasible, checked, valid, failure in (
        ("attempt", attempt_feasible, attempt_checked, attempt_valid, 0),
        ("selected", selected_feasible, selected_checked, selected_valid, selected_failure),
    ):
        result.update(
            {
                f"wbc_full_cert_{prefix}_checked": str(checked),
                f"wbc_full_cert_{prefix}_valid": str(valid),
                f"wbc_full_cert_{prefix}_input_valid": "1",
                f"wbc_full_cert_{prefix}_feasible": str(feasible),
                f"wbc_full_cert_{prefix}_failure_mask": str(failure),
                f"wbc_full_cert_{prefix}_assumed_flat_mask": "0",
                f"wbc_full_cert_{prefix}_force_residual_n": (
                    str(selected_force) if prefix == "selected" else "0.1"
                ),
                f"wbc_full_cert_{prefix}_moment_residual_nm": "0.2",
                f"wbc_full_cert_{prefix}_joint_residual_nm": (
                    str(selected_joint) if prefix == "selected" else "0.3"
                ),
                f"wbc_full_cert_{prefix}_friction_violation_n": "0.0",
                f"wbc_full_cert_{prefix}_normal_violation_n": "0.0",
                f"wbc_full_cert_{prefix}_swing_violation_n": "0.0",
                f"wbc_full_cert_{prefix}_torque_violation_nm": "0.0",
                f"wbc_full_cert_{prefix}_stance_acc_residual_mps2": "0.3",
            }
        )
    return result

class AuditWbcCertificateTests(unittest.TestCase):
    def write_run(self, rows, *, remove_field=None):
        temporary = tempfile.TemporaryDirectory()
        run_dir = Path(temporary.name)
        fields = list(rows[0])
        if remove_field is not None:
            fields.remove(remove_field)
            for item in rows:
                item.pop(remove_field, None)
        with (run_dir / "data.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        (run_dir / "run_manifest.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "run_id": "synthetic",
                    "repository": {
                        "git_commit": "deadbeef",
                        "git_dirty": "false",
                    },
                }
            ),
            encoding="utf-8",
        )
        return temporary, run_dir
    def test_legal_rows_are_certified_but_scope_is_not_b1(self):
        holder, run_dir = self.write_run([row(), row(stage=3, reused=1)])
        self.addCleanup(holder.cleanup)
        report = AUDIT.audit_run(run_dir)
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["certificates"]["selected"]["feasible_ratio"], 1.0)
        self.assertEqual(report["reuse"]["count"], 1)
        self.assertFalse(report["acceptance_scope"]["b1_acceptance"])
        self.assertFalse(report["acceptance_scope"]["final_pd_included"])
        self.assertEqual(report["source"]["run_id"], "synthetic")
        self.assertEqual(len(report["input_sha256"]), 2)
        self.assertTrue(report["script_sha256"])
    def test_violation_and_legacy_acceptance_mismatch_are_counted(self):
        holder, run_dir = self.write_run(
            [
                row(),
                row(
                    selected_feasible=0,
                    selected_failure=4,
                    selected_force=2.0,
                    converged=0,
                ),
            ]
        )
        self.addCleanup(holder.cleanup)
        report = AUDIT.audit_run(run_dir)
        self.assertEqual(report["status"], "NOT_CERTIFIED")
        self.assertEqual(report["certificates"]["selected"]["feasible_ratio"], 0.5)
        self.assertEqual(report["legacy"]["legacyaccepted_but_cert_not_pass_rows"], 1)
        self.assertEqual(
            report["convergence_groups"]["nonconverged"]["rows"], 1
        )
        self.assertEqual(
            report["certificates"]["selected"]["residuals"][
                "force_residual_n"
            ]["max"],
            2.0,
        )
        self.assertEqual(
            report["certificates"]["selected"]["residuals"][
                "joint_residual_nm"
            ]["max"],
            0.4,
        )
    def test_unchecked_and_nonfinite_rows_are_unknown_and_visible(self):
        unchecked = row(selected_checked=0)
        nonfinite = row()
        nonfinite["wbc_full_cert_selected_force_residual_n"] = "nan"
        holder, run_dir = self.write_run([unchecked, nonfinite])
        self.addCleanup(holder.cleanup)
        report = AUDIT.audit_run(run_dir)
        self.assertEqual(report["status"], "NOT_CERTIFIED")
        self.assertEqual(
            report["certificates"]["selected"]["status_counts"]["UNKNOWN"],
            2,
        )
        coverage = report["certificates"]["selected"]["field_coverage"]["force_residual_n"]
        self.assertEqual(coverage["nonfinite"], 1)
        self.assertEqual(report["selection"]["active_rows"], 2)
    def test_invalid_flags_masks_residuals_and_contradiction_fail_closed(self):
        invalid_flag = row()
        invalid_flag["wbc_full_cert_selected_feasible"] = "2"
        invalid_mask = row()
        invalid_mask["wbc_full_cert_selected_failure_mask"] = "1024"
        invalid_residual = row()
        invalid_residual["wbc_full_cert_selected_force_residual_n"] = "-1"
        contradiction = row(selected_failure=1)
        holder, run_dir = self.write_run(
            [invalid_flag, invalid_mask, invalid_residual, contradiction]
        )
        self.addCleanup(holder.cleanup)
        report = AUDIT.audit_run(run_dir)
        self.assertEqual(report["status"], "NOT_CERTIFIED")
        selected = report["certificates"]["selected"]
        self.assertEqual(selected["status_counts"]["UNKNOWN"], 3)
        self.assertEqual(selected["status_counts"]["NOT_CERTIFIED"], 1)
        self.assertEqual(selected["flag_counts"]["feasible"]["invalid"], 1)
        self.assertEqual(
            selected["field_coverage"]["failure_mask"]["invalid"], 1
        )
        self.assertEqual(
            selected["residuals"]["force_residual_n"]["invalid"], 1
        )
        self.assertEqual(selected["feasible_with_failure_mask_rows"], 1)
    def test_missing_field_is_reported_and_does_not_disappear(self):
        field = "wbc_full_cert_attempt_torque_violation_nm"
        holder, run_dir = self.write_run([row()], remove_field=field)
        self.addCleanup(holder.cleanup)
        report = AUDIT.audit_run(run_dir)
        self.assertEqual(report["status"], "NOT_CERTIFIED")
        self.assertIn(field, report["missing_header_fields"])
        self.assertEqual(
            report["certificates"]["attempt"]["field_coverage"]["torque_violation_nm"][
                "missing"
            ],
            1,
        )
    def test_selector_excludes_inactive_rows_and_records_unknown_selector(self):
        unknown = row()
        unknown["motion_stage"] = "nan"
        holder, run_dir = self.write_run([row(stage=1, active=0), unknown])
        self.addCleanup(holder.cleanup)
        report = AUDIT.audit_run(run_dir)
        self.assertEqual(report["selection"]["inactive_rows"], 1)
        self.assertEqual(report["selection"]["unknown_rows"], 1)
        self.assertEqual(report["selection"]["active_rows"], 0)
        self.assertEqual(report["status"], "NOT_CERTIFIED")
    def test_output_is_exclusive(self):
        holder, run_dir = self.write_run([row()])
        self.addCleanup(holder.cleanup)
        output = run_dir / "report.json"
        output.write_text("keep this", encoding="utf-8")
        self.assertEqual(
            AUDIT.main([str(run_dir), "--out", str(output)]),
            2,
        )
        self.assertEqual(output.read_text(encoding="utf-8"), "keep this")

if __name__ == "__main__":
    unittest.main()
