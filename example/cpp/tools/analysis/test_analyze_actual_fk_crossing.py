#!/usr/bin/env python3
"""Synthetic contract tests for analyze_actual_fk_crossing."""
import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("actual_fk", HERE / "analyze_actual_fk_crossing.py")
actual_fk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(actual_fk)


class ActualFkCrossingTest(unittest.TestCase):
    def write_fixture(self, directory):
        fields = ["state_tick_s", "world_base_x_m", "world_base_y_m", "world_base_z_m",
                  "imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad",
                  "wbc_measured_contact_mask", "foot_force_FR", "foot_force_FL",
                  "foot_force_RR", "foot_force_RL", "terrain_exec_FR_target_world_x_m"]
        for leg in actual_fk.LEGS:
            fields.extend(f"{leg}_{joint}_q_state" for joint in actual_fk.JOINTS)
            fields.extend(f"measured_fk_{leg}_foot_world_{axis}" for axis in "xyz")
        path = directory / "synthetic.csv"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for i, base_x in enumerate((0.0, .4, .6, .8, 1.0, 1.3, 1.5)):
                row = {"state_tick_s": str(i * .02), "world_base_x_m": str(base_x),
                       "world_base_y_m": "0", "world_base_z_m": ".176",
                       "imu_roll_rad": "0", "imu_pitch_rad": "0", "imu_yaw_rad": "0",
                       # Front feet touch after the front crossing; rear feet later.
                       "wbc_measured_contact_mask": str(3 if i >= 3 and i < 5 else 15 if i >= 5 else 0),
                       "terrain_exec_FR_target_world_x_m": "999"}
                for leg in actual_fk.LEGS:
                    q = (0.0, 1.2, -2.4)
                    for joint, value in zip(actual_fk.JOINTS, q):
                        row[f"{leg}_{joint}_q_state"] = str(value)
                predicted = actual_fk.fk(row)
                for leg, point in zip(actual_fk.LEGS, predicted):
                    for axis, value in zip("xyz", point):
                        row[f"measured_fk_{leg}_foot_world_{axis}"] = str(value)
                writer.writerow(row)
        return path

    def test_synthetic_crossing_and_fk_cross_check(self):
        with tempfile.TemporaryDirectory() as raw:
            path = self.write_fixture(Path(raw))
            report = actual_fk.analyze(path)
        self.assertEqual(report["overall_status"], "observed")
        self.assertEqual(report["measured_fk_cross_check"]["status"], "checked")
        self.assertLess(report["measured_fk_cross_check"]["max_abs_error_m"], 1e-12)
        self.assertTrue(report["phases"]["front_ascent_first_touchdown"]["measured_contact_witness_total"] > 0)
        self.assertIn("terrain_exec_FR_target_world_x_m", report["forbidden_fields_present_but_ignored"])
        # A target cannot create a crossing: only q/base FK is used for phases.
        self.assertNotEqual(report["phases"]["approach"]["status"], "missing")

    def test_missing_phase_is_honest(self):
        with tempfile.TemporaryDirectory() as raw:
            path = self.write_fixture(Path(raw))
            with path.open(newline="") as source:
                rows = list(csv.DictReader(source))[:2]
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
                writer.writeheader(); writer.writerows(rows)
            report = actual_fk.analyze(path)
        self.assertEqual(report["phases"]["rear_ascent_support_exchange"]["status"], "missing")
        self.assertEqual(report["body_posture_com"]["com_progression"].split()[0], "unavailable")

    def test_invalid_and_shuffled_state_ticks_are_reported(self):
        with tempfile.TemporaryDirectory() as raw:
            path = self.write_fixture(Path(raw))
            with path.open(newline="") as source:
                rows = list(csv.DictReader(source))
            rows[2]["state_tick_s"] = "not-a-number"
            rows[3]["state_tick_s"], rows[4]["state_tick_s"] = rows[4]["state_tick_s"], rows[3]["state_tick_s"]
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
                writer.writeheader(); writer.writerows(rows)
            report = actual_fk.analyze(path)
        self.assertEqual(report["state_tick_quality"]["invalid"], 1)
        self.assertFalse(report["state_tick_quality"]["all_analyzed_rows_finite"])
        self.assertFalse(report["state_tick_quality"]["monotonic"])

    def test_gt_over_tolerance_is_unavailable_and_measured_is_not_overridden(self):
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw); data = self.write_fixture(directory)
            gt = directory / "gt.csv"
            fields = ["time_s", "phase2_terrain_foot_contact_mask"]
            with gt.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow({"time_s": "10", "phase2_terrain_foot_contact_mask": "0"})
                writer.writerow({"time_s": "10", "phase2_terrain_foot_contact_mask": "0"})
                writer.writerow({"time_s": "9", "phase2_terrain_foot_contact_mask": "0"})
            report = actual_fk.analyze(data, gt)
        self.assertEqual(report["gt_time_quality"]["duplicates"], 1)
        self.assertTrue(report["gt_time_quality"]["input_nonmonotonic"])
        self.assertTrue(report["gt_time_quality"]["sorted_non_decreasing"])
        self.assertEqual(report["gt_alignment"]["status"], "unavailable")
        self.assertGreater(report["gt_alignment"]["unmatched"], 0)
        self.assertEqual(report["phases"]["front_ascent_first_touchdown"]["contact_witness_status"], "observed")

    def test_missing_measured_contact_is_ambiguous_not_gt_promoted(self):
        with tempfile.TemporaryDirectory() as raw:
            path = self.write_fixture(Path(raw))
            with path.open(newline="") as source:
                rows = list(csv.DictReader(source))
            for row in rows:
                row["wbc_measured_contact_mask"] = ""
                for leg in actual_fk.LEGS:
                    row["foot_force_" + leg] = ""
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
                writer.writeheader(); writer.writerows(rows)
            report = actual_fk.analyze(path)
        self.assertEqual(report["phases"]["front_ascent_first_touchdown"]["contact_witness_status"], "unavailable")
        self.assertEqual(report["phases"]["front_ascent_first_touchdown"]["status"], "ambiguous")

    def test_tick_quality_and_contact_penetration_are_separate(self):
        with tempfile.TemporaryDirectory() as raw:
            path = self.write_fixture(Path(raw))
            with path.open(newline="") as source:
                rows = list(csv.DictReader(source))
            rows[2]["state_tick_s"] = rows[1]["state_tick_s"]  # duplicate
            rows[3]["state_tick_s"] = "0.20"  # a gap
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
                writer.writeheader(); writer.writerows(rows)
            report = actual_fk.analyze(path)
        quality = report["state_tick_quality"]
        self.assertEqual(quality["duplicates"], 1)
        self.assertGreaterEqual(quality["gaps"], 1)
        points = [(0.0, 0.0, actual_fk.PATCH_OFFSET - .01),
                  (0.0, 0.0, actual_fk.PATCH_OFFSET - .10),
                  (0.0, 0.0, actual_fk.PATCH_OFFSET),
                  (0.0, 0.0, actual_fk.PATCH_OFFSET)]
        row = {"wbc_measured_contact_mask": "1"}
        phase = actual_fk.phase_report("fixture", [row], [points], [None])
        self.assertEqual(phase["contact_penetration"]["contact_samples"], 1)
        self.assertEqual(phase["swing_clearance_collision"]["collision_samples"], 1)


if __name__ == "__main__":
    unittest.main()
