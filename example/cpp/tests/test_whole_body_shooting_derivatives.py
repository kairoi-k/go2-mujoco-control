"""Tiny independent whole-rollout finite-difference and ownership checks."""
import copy
import importlib.util
import pathlib
import unittest
import mujoco
import numpy as np
path = pathlib.Path(__file__).resolve().parents[1] / "tools/research/whole_body_shooting_derivatives.py"
spec = importlib.util.spec_from_file_location("whole_body_shooting_derivatives", path)
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)

def make_model(free=False, activation=False):
    root = '<freejoint/>' if free else ''
    actuator = ('<general joint="hinge" dyntype="filter" dynprm="0.03"/>'
                if activation else '<motor joint="hinge"/>')
    return mujoco.MjModel.from_xml_string('''<mujoco><option timestep="0.002" gravity="0 0 -9.81"/>
    <worldbody><body pos="0 0 1">''' + root + '''
    <geom type="sphere" size="0.1" mass="1"/>
    <body pos="0.1 0 0"><joint name="hinge" axis="0 1 0" damping="0.1"/>
    <geom type="capsule" size="0.025" fromto="0 0 0 0.3 0 0" mass="0.2"/>
    </body></body></worldbody><actuator>''' + actuator + '</actuator></mujoco>')

def independent_rollout(model, initial, controls, block_steps):
    data = copy.copy(initial)
    rows = []
    for command in controls:
        for _ in range(block_steps):
            data.ctrl[:] = command
            mujoco.mj_step(model, data)
            rows.append(copy.copy(data))
    return rows

class ShootingDerivativesTest(unittest.TestCase):
    def check_derivative(self, free=False, activation=False):
        model = make_model(free, activation)
        initial = mujoco.MjData(model)
        initial.qpos[-1] = 0.17
        initial.qvel[:] = np.linspace(-0.08, 0.13, model.nv)
        initial.qacc_warmstart[:] = 0.123
        if model.na:
            initial.act[:] = 0.2
        controls = np.array([[0.15], [-0.1], [0.25]])
        rows, jac = helper.rollout(model, initial, controls, 2, True)
        expected = independent_rollout(model, initial, controls, 2)
        self.assertEqual(jac.shape, (6, 2 * model.nv + model.na, 3))
        for row, want in zip(rows, expected):
            np.testing.assert_array_equal(row.qpos, want.qpos)
            np.testing.assert_array_equal(row.qvel, want.qvel)
            np.testing.assert_array_equal(row.act, want.act)
        for eps in (2e-5, 5e-6):
            for column in range(3):
                plus = controls.copy(); plus.flat[column] += eps
                minus = controls.copy(); minus.flat[column] -= eps
                high = independent_rollout(model, initial, plus, 2)
                low = independent_rollout(model, initial, minus, 2)
                for k, (hi, lo) in enumerate(zip(high, low)):
                    tangent = np.empty(model.nv)
                    mujoco.mj_differentiatePos(model, tangent, 2 * eps, lo.qpos, hi.qpos)
                    fd = np.r_[tangent, (hi.qvel - lo.qvel) / (2 * eps),
                               (hi.act - lo.act) / (2 * eps)]
                    np.testing.assert_allclose(jac[k, :, column], fd, atol=2e-7, rtol=2e-4)
    def test_one_joint(self):
        self.check_derivative()
    def test_free_body_quaternion_tangent(self):
        self.check_derivative(free=True)
    def test_activation_state(self):
        self.check_derivative(free=True, activation=True)
    def test_no_mutation_and_geometry(self):
        model = make_model(True)
        initial = mujoco.MjData(model)
        initial.qvel[:] = 0.1
        initial.qacc_warmstart[:] = 0.3
        initial.ctrl[:] = 0.7
        initial.time = 2.0
        state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        before = np.empty(mujoco.mj_stateSize(model, state_spec))
        mujoco.mj_getState(model, initial, before, state_spec)
        controls = np.array([[0.2], [0.1]])
        original_controls = controls.copy()
        timestep = model.opt.timestep
        rows, jac = helper.rollout(model, initial, controls, 2, True)
        after = before.copy()
        mujoco.mj_getState(model, initial, after, state_spec)
        np.testing.assert_array_equal(before, after)
        np.testing.assert_array_equal(controls, original_controls)
        self.assertEqual(model.opt.timestep, timestep)
        ordinary, absent = helper.rollout(model, initial, controls, 2, False)
        self.assertIsNone(absent)
        for row, other in zip(rows, ordinary):
            np.testing.assert_array_equal(row.qpos, other.qpos)
            forwarded = copy.copy(row)
            mujoco.mj_forward(model, forwarded)
            np.testing.assert_array_equal(row.geom_xpos, forwarded.geom_xpos)
        saved_next = rows[1].qpos.copy()
        rows[0].qpos[:] = 10
        np.testing.assert_array_equal(rows[1].qpos, saved_next)
    def test_invalid_inputs(self):
        model = make_model()
        initial = mujoco.MjData(model)
        for controls in ([1.], np.zeros((2, 2)), np.zeros((0, 1)), [[np.nan]]):
            with self.assertRaises(ValueError):
                helper.rollout(model, initial, controls)
        for block_steps in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                helper.rollout(model, initial, [[0.]], block_steps)
        initial.qvel[0] = np.inf
        with self.assertRaises(ValueError):
            helper.rollout(model, initial, [[0.]])

if __name__ == '__main__':
    unittest.main()
