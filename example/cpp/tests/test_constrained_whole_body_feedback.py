"""Tiny actual compliant-contact checks, independent fresh force verification."""
import copy,importlib.util,pathlib,unittest
import mujoco
import numpy as np
path=pathlib.Path(__file__).resolve().parents[1]/'tools/research/constrained_whole_body_feedback.py'
spec=importlib.util.spec_from_file_location('constrained_feedback',path);helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
def model(mass=16):
 return mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep=".002" solver="Newton" tolerance="1e-12"/><worldbody><geom type="plane" size="1 1 .1"/><body pos="0 0 .1"><joint name="vertical" type="slide" axis="0 0 1"/><geom name="foot" type="sphere" size=".1" mass="{mass}"/></body></worldbody><actuator><motor joint="vertical"/></actuator></mujoco>''')
def forces(m,d,tau):
 initial=copy.copy(d);initial.ctrl[:]=tau;pre=copy.copy(initial);mujoco.mj_forward(m,pre);post=copy.copy(initial);mujoco.mj_step(m,post);mujoco.mj_forward(m,post)
 result=[]
 for state in (pre,post):
  total=0.
  for i in range(state.ncon):
   f=np.empty(6);mujoco.mj_contactForce(m,state,i,f);total+=f[0]
  result.append(total)
 return np.array(result)
class ConstrainedTest(unittest.TestCase):
 def test_actual_correction_and_preservation(self):
  m=model();d=mujoco.MjData(m);d.qpos[0]=-.001
  state_spec=mujoco.mjtState.mjSTATE_INTEGRATION;state_before=np.empty(mujoco.mj_stateSize(m,state_spec));mujoco.mj_getState(m,d,state_before,state_spec)
  before=d.qpos.copy();velocity=d.qvel.copy();warm=d.qacc_warmstart.copy();options=(m.opt.timestep,m.opt.tolerance,m.opt.iterations)
  self.assertGreater(max(forces(m,d,[-35.])),180.)
  tau,report=helper.constrained_tracking(m,d,[-35.],[1])
  self.assertGreater(tau[0],-35.);self.assertTrue(np.all(forces(m,d,tau)<=180.));self.assertIn(report['status'],('constrained_solution','strictly_feasible_evaluated_witness'))
  state_after=np.empty_like(state_before);mujoco.mj_getState(m,d,state_after,state_spec);np.testing.assert_array_equal(state_after,state_before)
  np.testing.assert_array_equal(d.qpos,before);np.testing.assert_array_equal(d.qvel,velocity);np.testing.assert_array_equal(d.qacc_warmstart,warm);self.assertEqual(options,(m.opt.timestep,m.opt.tolerance,m.opt.iterations))
 def test_fast_path(self):
  m=model(1);d=mujoco.MjData(m)
  tau,report=helper.constrained_tracking(m,d,[0.],[1]);np.testing.assert_array_equal(tau,[0.]);self.assertEqual(report['status'],'box_optimum_feasible')
 def test_irreducible_force_rejected(self):
  m=model(100);d=mujoco.MjData(m);d.qpos[0]=-.001
  self.assertGreater(min(forces(m,d,[35.])),180.)
  with self.assertRaises(helper.ConstrainedFeedbackFailure):helper.constrained_tracking(m,d,[0.],[1])
if __name__=='__main__':unittest.main()
