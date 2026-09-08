"""Absolute coverage boundary tests for the offline horizon probe (no physics)."""
import copy,importlib.util,pathlib,unittest
path=pathlib.Path(__file__).resolve().parents[1]/'tools/research/whole_body_horizon_probe.py'
spec=importlib.util.spec_from_file_location('probe',path);probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
class Coverage(unittest.TestCase):
    def setUp(self): self.plan={'complete':True,'completed_steps':10,'stages':[{'time':21.02+k*.002} for k in range(10)]}
    def test_start_included(self): self.assertTrue(probe.temporal_admission(self.plan,21.02)[0])
    def test_delayed_still_covered(self): self.assertTrue(probe.temporal_admission(self.plan,21.026)[0])
    def test_expiration_excluded(self): self.assertEqual(probe.temporal_admission(self.plan,21.04),(False,'expired'))
    def test_before_coverage(self): self.assertEqual(probe.temporal_admission(self.plan,21.018),(False,'before_coverage'))
    def test_partial_candidate_rejected(self):
        self.plan['complete']=False;self.assertEqual(probe.temporal_admission(self.plan,21.02),(False,'missing_coverage'))
    def test_missing_sample_rejected(self):
        self.plan['stages'][3]['time']+=.002;self.assertEqual(probe.temporal_admission(self.plan,21.026),(False,'missing_sample'))
if __name__=='__main__': unittest.main()
