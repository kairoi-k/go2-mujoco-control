"""Synthetic contract tests for V4; no simulation, build, or acceptance claim."""
import json
import tempfile
import unittest
from pathlib import Path
import analyze_b1_dynamic_v3 as v3
import analyze_dynamic_traversal_v4 as v4
from analyze_b1_physical import LEGS
BOX5 = dict(front=5.0, rear=5.5, left=-.75, right=.75, top=.05, height_m=.05)
STATUS = dict(controller_status=0, completion_status=0, safety_status=0,
              quality_status=0, ground_truth_status=0, dynamics_status=0)

def fixture():
    truth, control = [], []
    for i in range(16002):
        t = i * .002
        x = max(0.0, min(t - 14.0, 12.0))
        phase = (i % 70) / 70
        mask = 9 if phase < .3 or phase >= .9 else (6 if .4 <= phase < .8 else 0)
        row = dict(time_s=t, base_pos_world_x_m=x, base_pos_world_y_m=0.0,
                   base_pos_world_z_m=.35, base_qvel_world_y_mps=0.0,
                   base_qvel_world_z_mps=0.0,
                   base_qvel_world_x_mps=1.0 if 14 < t < 26 else 0.0,
                   base_quat_w=1.0, base_quat_x=0.0, base_quat_y=0.0,
                   base_quat_z=0.0, robot_collision_rear_bound_world_x_m=x-.4,
                   phase2_terrain_nonfoot_contact_count=0,
                   phase2_terrain_nonfoot_contact_force_N=0.0)
        total, topmask = 0, 0
        for j, leg in enumerate(LEGS):
            foot_x = x + (.2 if j < 2 else -.2)
            top = 5 <= foot_x <= 5.5
            force = 100 if mask & (1 << j) else 0
            for axis, value in zip('xyz', (foot_x, .1 if j % 2 else -.1,
                                           .072 if top else .022)):
                row[leg + '_pos_world_' + axis + '_m'] = value
            for axis in 'xyz':
                row[leg + '_foot_contact_grf_world_' + axis + '_N'] = force if axis == 'z' else 0
            row[leg + '_terrain_top_grf_world_z_N'] = force if top else 0
            row[leg + '_terrain_nontop_contact_force_N'] = 0
            if top and force:
                topmask |= 1 << j
            total += force
        row['phase2_terrain_foot_contact_mask'] = topmask
        for axis in 'xyz':
            row['total_contact_grf_world_' + axis + '_N'] = total if axis == 'z' else 0
        truth.append(row)
        request = 0 if t < 8 or t >= 26 else ((t-8)/8 if t < 16 else (1 if t < 24 else (26-t)/2))
        control.append(dict(state_tick_s=t, telemetry_gait_time_s=t, motion_stage=2,
                            velocity_command_active=1,
                            velocity_command_requested_mps=request,
                            velocity_command_gait_period_s=.14,
                            velocity_command_gait_duty=.44,
                            cycle_index=i//70, phase=phase, terrain_actuation=0,
                            terrain_sensor_only=1, terrain_safe_stop_requested=0,
                            wbc_full_id_ok=1))
    return truth, control

class DynamicV4Tests(unittest.TestCase):
    def test_old_5cm_non_superseded_gates_match_v3(self):
        truth, control = fixture()
        old = v3.analyze(truth, control, BOX5, 'controlled stop', STATUS)
        new = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS, old)
        for key, value in old['gates'].items():
            if key not in v4.SUPERSEDED_LEGACY_GATES:
                self.assertEqual(new['gates'][key], value, key)
        self.assertEqual(old['status'], 'PASS')
        self.assertEqual(new['status'], 'PASS')
        self.assertEqual(new['legacy_v3_verdict']['status'], 'PASS')
    def test_10cm_height_is_registered_while_legacy_scene_gate_is_superseded(self):
        truth, control = fixture()
        box = dict(BOX5, top=.10, height_m=.10)
        result = v4.analyze(truth, control, box, .10, 'controlled stop', STATUS)
        self.assertTrue(result['gates']['registered_scene_height'])
        self.assertFalse(result['gates']['registered_v3_scene'])
        self.assertEqual(result['legacy_v3_verdict']['status'], 'SUPERSEDED')
        self.assertTrue(result['gates']['registered_stable_approach_geometry'])
        self.assertEqual(result['status'], 'PASS')
        altered = v4.analyze(truth, control, dict(box, front=4.9), .10, 'controlled stop', STATUS)
        self.assertFalse(altered['gates']['registered_stable_approach_geometry'])
        self.assertEqual(altered['status'], 'NOT_CERTIFIED')
    def test_period_and_duty_are_flexible_but_must_be_valid(self):
        truth, control = fixture()
        for row in control:
            row['velocity_command_gait_period_s'] = .20
            row['velocity_command_gait_duty'] = .30
        old = v3.analyze(truth, control, BOX5, 'controlled stop', STATUS)
        new = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS, old)
        self.assertFalse(old['gates']['nominal_running_through_interaction'])
        self.assertTrue(new['gates']['nominal_running_through_interaction'])
        self.assertTrue(new['gates']['running_period_duty_valid'])
        self.assertEqual(new['status'], 'PASS')
        control[100]['velocity_command_gait_period_s'] = float('nan')
        bad = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertFalse(bad['gates']['running_period_duty_valid'])
        self.assertEqual(bad['status'], 'NOT_CERTIFIED')
    def test_missing_control_span_fails_required_join_coverage(self):
        truth, control = fixture()
        control = [r for r in control if not 19.0 <= r['state_tick_s'] < 20.0]
        result = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertFalse(result['gates']['v4_required_truth_control_coverage'])
        self.assertFalse(result['gates']['approach_interaction_join_coverage'])
        self.assertGreater(result['detail']['v4_uncovered_truth_rows'], 0)
        self.assertEqual(result['status'], 'NOT_CERTIFIED')

    def test_nonfinite_or_nonmonotonic_timestamps_fail_closed(self):
        truth, control = fixture()
        control[100]['state_tick_s'] = float('nan')
        nonfinite = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertFalse(nonfinite['gates']['v4_control_time_order'])
        self.assertFalse(nonfinite['gates']['v4_required_truth_control_coverage'])
        self.assertEqual(nonfinite['status'], 'NOT_CERTIFIED')

        truth, control = fixture()
        truth[100]['time_s'] = truth[99]['time_s'] - .001
        nonmonotonic = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertFalse(nonmonotonic['gates']['v4_truth_time_order'])
        self.assertEqual(nonmonotonic['status'], 'NOT_CERTIFIED')

        truth, control = fixture()
        control[100]['state_tick_s'] = control[99]['state_tick_s'] - .001
        nonmonotonic = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertFalse(nonmonotonic['gates']['v4_control_time_order'])
        self.assertEqual(nonmonotonic['status'], 'NOT_CERTIFIED')

    def test_empty_input_is_not_certified(self):
        result = v4.analyze([], [], BOX5, .05, '', STATUS)
        self.assertFalse(result['gates']['v4_input_rows_nonempty'])
        self.assertFalse(result['gates']['v4_truth_time_order'])
        self.assertFalse(result['gates']['v4_control_time_order'])
        self.assertFalse(result['gates']['v4_required_truth_control_coverage'])
        self.assertEqual(result['status'], 'NOT_CERTIFIED')

    def test_missing_and_nonfinite_truth_are_not_filtered(self):
        truth, control = fixture()
        truth[100].pop('base_pos_world_z_m')
        missing = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertEqual(missing['status'], 'NOT_CERTIFIED')
        self.assertFalse(missing['gates']['whole_trace_required_truth_finite'])
        truth, control = fixture()
        truth[-1]['base_quat_w'] = float('nan')
        nonfinite = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertEqual(nonfinite['status'], 'NOT_CERTIFIED')
        self.assertFalse(nonfinite['gates']['whole_trace_required_truth_finite'])
        self.assertNotIn('NaN', json.dumps(nonfinite, allow_nan=False))
    def test_architecture_claim_does_not_follow_terrain_actuation_flag(self):
        truth, control = fixture()
        for row in control:
            row['terrain_actuation'] = 1
            row['terrain_sensor_only'] = 0
        result = v4.analyze(truth, control, BOX5, .05, 'controlled stop', STATUS)
        self.assertEqual(result['architecture_claim']['status'], 'NOT_ESTABLISHED')
    def test_scene_parser_binds_height_and_center(self):
        scene = Path(__file__).resolve().parents[4] / 'unitree_robots/go2/b1_running_step_10cm.xml'
        box = v4.parse_scene(scene, .10)
        self.assertAlmostEqual(box['top'], .10)
        self.assertAlmostEqual(box['center_z'], .05)
        with self.assertRaises(ValueError):
            v4.parse_scene(scene, .05)
    def test_required_height_and_exclusive_output(self):
        with self.assertRaises(SystemExit):
            v4.main(['missing-run', '--scene', 'missing.xml', '--out', 'missing.json'])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'result.json'
            value = {'status': 'NOT_CERTIFIED', 'value': float('nan')}
            v4.write_exclusive(output, value)
            self.assertIn('null', output.read_text())
            with self.assertRaises(FileExistsError):
                v4.write_exclusive(output, value)

if __name__ == '__main__':
    unittest.main()
