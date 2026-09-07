"""Focused algebra, quaternion convention, and portable provenance tests."""
import hashlib
import importlib.util
import pathlib
import tempfile
import sys
import unittest
import mujoco
import numpy as np
from scipy.linalg import solve_discrete_are
path=pathlib.Path(__file__).resolve().parents[1]/'tools/research/periodic_whole_body_feedback.py'
sys.path.insert(0,str(path.parent))
spec=importlib.util.spec_from_file_location('periodic_feedback',path)
helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
class PeriodicFeedbackTest(unittest.TestCase):
 def test_scalar_matches_independent_dare(self):
  A=np.array([[1.1]]);B=np.array([[.3]]);Q=np.array([[2.]]);R=np.array([[.2]])
  gains,iterations,residual=helper.periodic_lqr([A],[B],Q,R)
  P=solve_discrete_are(A,B,Q,R)
  expected=np.linalg.solve(R+B.T@P@B,B.T@P@A)
  np.testing.assert_allclose(gains[0],expected,rtol=1e-9)
  self.assertLess(residual,1e-10)
 def test_nonconvergence_rejected(self):
  with self.assertRaisesRegex(ValueError,'did not converge'):
   helper.periodic_lqr([np.array([[1.1]])],[np.zeros((1,1))],np.eye(1),np.eye(1),max_iterations=3)
 def test_periodic_gain_order(self):
  As=[np.array([[.8]]),np.array([[1.2]])];Bs=[np.array([[.4]]),np.array([[.7]])]
  gains,_,_=helper.periodic_lqr(As,Bs,np.eye(1),np.eye(1))
  # Independent finite-horizon recursion for 2000 alternating stages.
  P=np.eye(1);expected={}
  for k in range(1999,-1,-1):
   A,B=As[k%2],Bs[k%2]
   K=np.linalg.solve(np.eye(1)+B.T@P@B,B.T@P@A)
   P=np.eye(1)+A.T@P@A-A.T@P@B@K
   if k<2:expected[k]=K
  for k in range(2):np.testing.assert_allclose(gains[k],expected[k],rtol=1e-9)
 def test_quaternion_tangent_and_sign(self):
  m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
  d=mujoco.MjData(m);q=d.qpos.copy();v=np.array([.03,-.02,.01,.11,-.07,.04])
  mujoco.mj_integratePos(m,q,v,1.)
  error=helper.state_error(m,d.qpos,d.qvel,q,v)
  np.testing.assert_allclose(error,np.r_[v,v],atol=1e-14)
  q[3:7]*=-1
  np.testing.assert_allclose(helper.state_error(m,d.qpos,d.qvel,q,v),error,atol=1e-14)
 def test_nominal_and_perturbed_terminal_are_distinct(self):
  m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
  nominal=mujoco.MjData(m);perturbed=mujoco.MjData(m)
  perturbed.qvel[1]=.2
  delta=np.zeros(m.nv);delta[3]=.1
  mujoco.mj_integratePos(m,perturbed.qpos,delta,1.)
  target=helper.translated_reference(nominal,.5)
  frozen=helper.translated_reference(perturbed,.5)
  self.assertEqual(target['qpos'][0],.5);self.assertEqual(frozen['qpos'][0],.5)
  self.assertEqual(target['qvel'][1],0);self.assertEqual(frozen['qvel'][1],.2)
  self.assertNotEqual(target['qpos'][3:7],frozen['qpos'][3:7])
 def test_portable_hashes_and_tampering(self):
  with tempfile.TemporaryDirectory() as td:
   base=pathlib.Path(td);scene=base/'scene.xml';scene.write_text('<mujoco/>')
   h=hashlib.sha256(scene.read_bytes()).hexdigest()
   result={'scene':'scene.xml','input_hashes':{'scene.xml':h}}
   resolved,hashes=helper.resolve_inputs(result,base/'result.json')
   self.assertEqual(resolved,scene);self.assertEqual(hashes,{str(scene):h})
   scene.write_text('tamper')
   with self.assertRaisesRegex(ValueError,'hash mismatch'):helper.resolve_inputs(result,base/'result.json')
 def test_unbound_scene_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   with self.assertRaisesRegex(ValueError,'not hash bound'):
    helper.resolve_inputs({'scene':'scene.xml','input_hashes':{}},pathlib.Path(td)/'result.json')
if __name__=='__main__':unittest.main()
