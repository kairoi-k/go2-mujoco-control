#!/usr/bin/env python3
"""Counterexample fixtures for the two small V3 protocol helpers."""
from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


PATH = Path(__file__).with_name("b1_dynamic_protocol.py")
SPEC = importlib.util.spec_from_file_location("b1_dynamic_protocol", PATH)
assert SPEC and SPEC.loader
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def merged(time_s, *, cycle=0, phase=0.5, mask=9, total=(0, 0, 100),
           period=0.14, duty=0.44, requested=1.0, active=1):
    row = {
        "state_tick_s": str(time_s), "time_s": str(time_s),
        "motion_stage": "2", "velocity_command_active": str(active),
        "velocity_command_requested_mps": str(requested),
        "velocity_command_gait_period_s": str(period),
        "velocity_command_gait_duty": str(duty), "cycle_index": str(cycle),
        "phase": str(phase), "base_qvel_world_x_mps": "1.0",
        "terrain_execution_planned_contact_mask": str(mask),
        "total_contact_grf_world_x_N": str(total[0]),
        "total_contact_grf_world_y_N": str(total[1]),
        "total_contact_grf_world_z_N": str(total[2]),
    }
    for bit, leg in enumerate(V3.LEGS):
        force = 20.0 if mask & (1 << bit) else 0.0
        row.update({
            f"{leg}_foot_contact_grf_world_x_N": "0",
            f"{leg}_foot_contact_grf_world_y_N": "0",
            f"{leg}_foot_contact_grf_world_z_N": str(force),
        })
    return row


def cycles_fixture(count=4, bad_cycle=None):
    control = []
    truth = []
    # Add one real predecessor and successor wrap around the requested
    # interaction cycles.  The helper must not manufacture either edge.
    for cycle in range(count + 2):
        base = cycle * 0.14
        for sample in range(70):
            offset = sample * 0.002
            time_s = base + offset
            phase = offset / 0.14
            if offset < 0.02:
                mask, total = 9, (0, 0, 100)
            elif offset < 0.04:
                mask, total = 0, (20, 0, 0) if bad_cycle == cycle else (0, 0, 0)
            elif offset < 0.06:
                mask, total = 6, (0, 0, 100)
            else:
                mask, total = 0, (20, 0, 0) if bad_cycle == cycle else (0, 0, 0)
            row = merged(f"{time_s:.3f}", cycle=cycle, phase=f"{phase:.6f}", mask=mask,
                         total=total)
            control.append(row)
            truth.append(dict(row))
    return control, truth


class V3PureProtocolTests(unittest.TestCase):
    def test_stable_approach_rejects_transition(self):
        rows = [merged(f"{i * 0.002:.3f}", period=0.20, duty=0.50) for i in range(401)]
        result = V3.stable_approach(rows, 0.80)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("approach_period_out_of_band", result["reasons"])

    def test_stable_approach_uses_each_gt_sample_with_reused_state_tick(self):
        rows = [merged(f"{i * 0.002:.3f}") for i in range(401)]
        for i, row in enumerate(rows):
            row["state_tick_s"] = f"{(i // 2) * 0.004:.3f}"
        # The first-contact row is outside the half-open approach window.
        rows[-1]["base_qvel_world_x_mps"] = "0.0"
        # Exact duplicate GT timestamps are tolerated by the pure helper;
        # the full wrapper rejects them in its strict global GT gate.
        rows.insert(101, dict(rows[100]))
        result = V3.stable_approach(rows, 0.80)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["evidence"]["rows"], 400)
        self.assertEqual(result["evidence"]["speed_in_band_fraction"], 1.0)
    def test_stable_approach_counts_bad_gt_speed_sharing_a_state_tick(self):
        rows = [merged(f"{i * 0.002:.3f}") for i in range(401)]
        for i, row in enumerate(rows):
            row["state_tick_s"] = f"{(i // 2) * 0.004:.3f}"
            if i < 42 and i % 2 == 0:
                row["base_qvel_world_x_mps"] = "0.0"
        result = V3.stable_approach(rows, 0.80)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("approach_measured_speed_fraction", result["reasons"])
        self.assertAlmostEqual(result["evidence"]["speed_in_band_fraction"], 379 / 400)

    def test_stable_approach_requires_first_contact_tail(self):
        rows = [merged(f"{i * 0.002:.3f}") for i in range(371)]  # ends at 0.740 s
        result = V3.stable_approach(rows, 0.80)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("approach_first_contact_truth_tail_coverage", result["reasons"])
    def test_stable_approach_requires_gt_gap_coverage(self):
        rows = [merged(f"{i * 0.002:.3f}") for i in range(401)]
        rows = [row for row in rows if not 0.400 <= float(row["time_s"]) < 0.414]
        result = V3.stable_approach(rows, 0.80)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("time_s_gap_gt_10ms", result["reasons"])

    def test_stable_approach_rejects_nonfinite_and_descending_ticks(self):
        nonfinite = [merged(f"{i * 0.002:.3f}") for i in range(401)]
        nonfinite[10]["state_tick_s"] = "nan"
        result = V3.stable_approach(nonfinite, 0.80)
        self.assertIn("nonfinite_state_tick_s", result["reasons"])

        descending = [merged(f"{i * 0.002:.3f}") for i in range(401)]
        descending[200]["state_tick_s"] = "0.100"
        result = V3.stable_approach(descending, 0.80)
        self.assertIn("descending_state_tick_s", result["reasons"])

    def test_four_cycle_edge_rule_allows_three_good(self):
        control, truth = cycles_fixture(bad_cycle=2)
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["evidence"]["complete_cycles"], 4)
        self.assertEqual(result["evidence"]["windows"][0]["good_cycles"], 3)

    def test_five_cycle_sliding_rule_does_not_use_four_cycle_edge(self):
        control, truth = cycles_fixture(count=5, bad_cycle=2)
        for row in truth:
            cycle = int(row["cycle_index"])
            offset = float(row["time_s"]) - cycle * 0.14
            if cycle in (1, 3) and (0.02 <= offset < 0.04 or offset >= 0.06):
                row["total_contact_grf_world_x_N"] = "20"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.84)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("sliding_cycle_topology_below_three_good", result["reasons"])

    def test_total_grf_prevents_false_aerial_from_foot_mask(self):
        control, truth = cycles_fixture(bad_cycle=1)
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        cycle = next(item for item in result["evidence"]["cycles"] if item["cycle_index"] == 1)
        self.assertEqual(cycle["aerial_s"], 0.0)

    def test_all_four_contact_is_not_both_diagonals(self):
        control, truth = cycles_fixture()
        for row in truth:
            cycle = int(row["cycle_index"])
            offset = float(row["time_s"]) - cycle * 0.14
            if cycle == 1 and ((offset < 0.02) or (0.04 <= offset < 0.06)):
                for leg in V3.LEGS:
                    row[f"{leg}_foot_contact_grf_world_z_N"] = "20"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        cycle = next(item for item in result["evidence"]["cycles"] if item["cycle_index"] == 1)
        self.assertEqual(cycle["diagonal_support_s"]["9"], 0.0)
        self.assertEqual(cycle["diagonal_support_s"]["6"], 0.0)
        self.assertFalse(cycle["good_cycle"])

    def test_complete_topology_rejects_nonfinite_and_descending_truth_time(self):
        control, truth = cycles_fixture()
        truth[3]["time_s"] = "nan"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("nonfinite_time_s", result["reasons"])

        control, truth = cycles_fixture()
        truth[3]["time_s"] = "0.00"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("descending_time_s", result["reasons"])

        control, truth = cycles_fixture()
        control[2]["cycle_index"] = "nan"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("nonfinite_cycle_index", result["reasons"])

    def test_controller_and_truth_sampling_gaps_fail_closed(self):
        control, truth = cycles_fixture()
        control = [
            row for row in control
            if not 0.18 <= float(row["time_s"]) < 0.21
        ]
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("state_tick_s_gap_gt_20ms", result["reasons"])

        control, truth = cycles_fixture()
        truth = [
            row for row in truth
            if not 0.30 <= float(row["time_s"]) < 0.314
        ]
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("time_s_gap_gt_10ms", result["reasons"])

    def test_cycle_index_must_be_integer_nonnegative(self):
        control, truth = cycles_fixture()
        control[100]["cycle_index"] = "1.5"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("invalid_cycle_index", result["reasons"])

        control, truth = cycles_fixture()
        control[100]["cycle_index"] = "-1"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("invalid_cycle_index", result["reasons"])

    def test_phase_must_be_bounded_and_monotonic_within_cycle(self):
        control, truth = cycles_fixture()
        control[100]["phase"] = "1.1"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("phase_out_of_range", result["reasons"])

        control, truth = cycles_fixture()
        control[100]["phase"] = "0.9"
        control[101]["phase"] = "0.8"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("phase_reversal_cycle_1", result["reasons"])

    def test_each_cycle_requires_truth_edge_coverage(self):
        control, truth = cycles_fixture()
        truth = [
            row for row in truth
            if not (
                row["cycle_index"] == "1"
                and (float(row["time_s"]) - 0.14 < 0.014
                     or float(row["time_s"]) - 0.14 >= 0.126)
            )
        ]
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertIn("cycle_1_truth_edge_coverage", result["reasons"])

    def test_cycle_index_gap_is_not_silently_dropped(self):
        control, truth = cycles_fixture(count=5)
        control = [row for row in control if row["cycle_index"] != "2"]
        truth = [row for row in truth if row["cycle_index"] != "2"]
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.84)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("cycle_index_gap", result["reasons"])

    def test_phase_samples_do_not_define_cycle_completeness(self):
        control, truth = cycles_fixture()
        for row in control:
            row["phase"] = "0.5"
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertEqual(result["status"], "PASS")

    def test_partial_or_missing_cycle_is_not_complete(self):
        control, truth = cycles_fixture()
        control = [row for row in control if row["cycle_index"] != "2"]
        truth = [row for row in truth if row["cycle_index"] != "2"]
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.70)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("complete_interaction_cycles_lt_4", result["reasons"])

    def test_final_half_cycle_has_no_successor_wrap(self):
        control, truth = cycles_fixture()
        result = V3.complete_cycle_topology(control, truth, 0.14, 0.74)
        self.assertEqual(result["status"], "NOT_CERTIFIED")
        self.assertIn("cycle_5_wrap_missing", result["reasons"])


if __name__ == "__main__":
    unittest.main()
