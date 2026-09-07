"""Tiny XML independent replay, hard-limit, provenance and tamper checks."""
import copy
import hashlib
import importlib.util
import json
import pathlib
import tempfile
import unittest
import mujoco
import numpy as np
path = pathlib.Path(__file__).resolve().parents[1] / "tools/research/verify_whole_body_shooting.py"
spec = importlib.util.spec_from_file_location("verify_whole_body_shooting", path)
verifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verifier)
XML = '''<mujoco><option timestep="0.002"/><worldbody>
<geom name="ground" type="plane" size="5 5 .1"/>
<body name="robot" pos="0 0 1"><freejoint/>
<geom name="BODY" type="sphere" size=".01" mass="1"/>
<geom name="FR" type="sphere" pos=".1 -.1 -.2" size=".04" mass=".1"/>
<geom name="FL" type="sphere" pos=".1 .1 -.2" size=".04" mass=".1"/>
<geom name="RR" type="sphere" pos="-.1 -.1 -.2" size=".04" mass=".1"/>
<geom name="RL" type="sphere" pos="-.1 .1 -.2" size=".04" mass=".1"/>
<body pos="0 0 .05"><joint name="hinge" axis="0 1 0" range="-2 2"/>
<geom type="capsule" fromto="0 0 0 .1 0 0" size=".01" mass=".1"/></body>
</body></worldbody><actuator><motor joint="hinge"/></actuator></mujoco>'''

class CertificateTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = pathlib.Path(self.temp.name)
        self.scene = self.directory / 'tiny.xml'
        self.scene.write_text(XML)
        self.result_path = self.directory / 'result.json'
    def tearDown(self):
        self.temp.cleanup()
    def fixture(self, contact=False, external=False):
        model = mujoco.MjModel.from_xml_path(str(self.scene))
        initial = mujoco.MjData(model)
        initial.time = 3.0
        if contact:
            initial.qpos[2] = .195
        if external:
            initial.xfrc_applied[1] = [.2, -.1, .3, .02, .01, -.03]
            initial.qfrc_applied[-1] = .1
        state_spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
        state = np.empty(mujoco.mj_stateSize(model, state_spec))
        mujoco.mj_getState(model, initial, state, state_spec)
        data = copy.copy(initial)
        commands = np.zeros((2, 1))
        rows = []
        for command in commands:
            data.ctrl[:] = command
            mujoco.mj_step(model, data)
            observed = copy.copy(data)
            mujoco.mj_forward(model, observed)
            forces = np.zeros(4)
            gids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ('FR', 'FL', 'RR', 'RL')]
            for i in range(observed.ncon):
                contact_info = observed.contact[i]
                f = np.zeros(6)
                mujoco.mj_contactForce(model, observed, i, f)
                for leg, gid in enumerate(gids):
                    if gid == contact_info.geom1 or gid == contact_info.geom2:
                        forces[leg] += f[0]
            rows.append({'time': float(observed.time), 'qpos': observed.qpos.tolist(),
                         'qvel': observed.qvel.tolist(), 'normal_forces': forces.tolist(), 'torque': command.tolist()})
        return {'schema': 'whole-body-shooting-v1', 'scene': str(self.scene),
                'input_hashes': {str(self.scene): hashlib.sha256(self.scene.read_bytes()).hexdigest()},
                'source_sha': 'a'*40, 'mujoco_version': mujoco.__version__,
                'integration_state_spec': state_spec, 'initial_integration_state': state.tolist(),
                'times_are_absolute': True, 'timestep_s': .002, 'period_s': .004,
                'initial_phase': .486194, 'duty': .44, 'leg_offsets': [0., .46, .46, 0.],
                'controls': commands.tolist(), 'rows': rows, 'command_vx': 0.,
                'constraints': dict(verifier.LIMITS),
                'terminal_reference': {'qpos': initial.qpos.tolist(), 'qvel': initial.qvel.tolist()}}
    def run_certificate(self, result):
        self.result_path.write_text(json.dumps(result))
        return verifier.verify_result(self.result_path)
    def test_exact_replay_and_external_force_balance(self):
        for external in (False, True):
            report = self.run_certificate(self.fixture(external=external))
            self.assertTrue(report['diagnostic_success'], report)
            self.assertEqual(report['maxima']['state_error'], 0.)
            self.assertLess(report['maxima']['dynamics_residual'], 1e-10)
            self.assertEqual(report['periods_covered'], 1.)
            self.assertIn('not B1', report['scope'])
            self.assertTrue(report['contact_episodes'])
    def test_tampered_saved_state_and_control(self):
        result = self.fixture()
        result['rows'][0]['qpos'][0] += .01
        self.assertFalse(self.run_certificate(result)['checks']['state_reproduction'])
        result = self.fixture()
        result['controls'][0][0] = 36.
        report = self.run_certificate(result)
        self.assertFalse(report['diagnostic_success'])
        self.assertFalse(report['checks']['torque_bound'])
        result = self.fixture()
        result['rows'][0]['normal_forces'][0] = 1.
        self.assertFalse(self.run_certificate(result)['checks']['force_reproduction'])
    def test_missing_coverage_and_malformed_dimensions(self):
        for mutate in (lambda r: r['rows'].pop(),
                       lambda r: r.update(period_s=.006),
                       lambda r: r.update(controls=[[0., 1.], [0., 1.]]),
                       lambda r: r['initial_integration_state'].pop(),
                       lambda r: r['terminal_reference'].update(qvel=[0.]),
                       lambda r: r.update(integration_state_spec=1)):
            result = self.fixture()
            mutate(result)
            with self.assertRaises(ValueError):
                self.run_certificate(result)
    def test_nonfinite_and_tampered_hash(self):
        result = self.fixture()
        result['controls'][0][0] = float('nan')
        with self.assertRaises(ValueError):
            self.run_certificate(result)
        result = self.fixture()
        result['input_hashes'][str(self.scene)] = '0'*64
        with self.assertRaises(ValueError):
            self.run_certificate(result)
        result = self.fixture()
        result['input_hashes'] = {}
        with self.assertRaises(ValueError):
            self.run_certificate(result)
    def test_limits_cannot_be_relaxed(self):
        result = self.fixture()
        result['constraints']['normal_force_limit_n'] = 181.
        with self.assertRaises(ValueError):
            self.run_certificate(result)
    def test_real_contact_failure_is_not_hidden_by_reproduction(self):
        report = self.run_certificate(self.fixture(contact=True))
        self.assertTrue(report['checks']['state_reproduction'])
        self.assertTrue(report['checks']['force_reproduction'])
        self.assertGreater(report['maxima']['normal_force_n'], 0.)
        self.assertFalse(report['checks']['base_height_bound'])
        self.assertFalse(report['diagnostic_success'])
    def test_terminal_reference_cannot_be_forged(self):
        result = self.fixture()
        result['terminal_reference']['qpos'][0] += .001
        with self.assertRaises(ValueError):
            self.run_certificate(result)
    def test_unhashed_include(self):
        part = self.directory / 'part.xml'
        part.write_text(XML)
        self.scene.write_text('<mujoco><include file="part.xml"/></mujoco>')
        result = self.fixture()
        with self.assertRaises(ValueError):
            self.run_certificate(result)
        result['input_hashes'][str(part)] = hashlib.sha256(part.read_bytes()).hexdigest()
        self.assertTrue(self.run_certificate(result)['diagnostic_success'])

if __name__ == '__main__':
    unittest.main()
