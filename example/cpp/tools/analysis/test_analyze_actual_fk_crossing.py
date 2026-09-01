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
        self.assertEqual(report["body_posture_com"]["com_progression"].split()[0], "missing")


if __name__ == "__main__":
    unittest.main()
