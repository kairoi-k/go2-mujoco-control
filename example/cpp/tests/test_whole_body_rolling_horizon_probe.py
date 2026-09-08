"""Synthetic scheduler records isolate epoch, deadline, and commitment regressions."""
import copy,pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'tools/research'))
from whole_body_rolling_horizon_probe import audit_schedule
def candidate(version,obs,source,adopted=True,elapsed=4.):
    return {'version':version,'observation_tick':obs,'source_version':source,'start_tick':obs+6,'end_tick':obs+16,'generation_ms':elapsed,'late':elapsed>12,'valid':True,'adopted':adopted,'stages':[{'tick':k} for k in range(obs,obs+16)]}
class Scheduler(unittest.TestCase):
    def setUp(self):
        self.run={'candidates':[candidate(1,0,0),candidate(2,6,1)],'rows':[{'tick':k,'active_version':0 if k<6 else 1 if k<12 else 2} for k in range(18)]}
    def test_ordered_overlap(self):self.assertEqual(audit_schedule(self.run)['adopted_versions'],[1,2])
    def test_late_discard_stops_before_missing_prefix(self):
        self.run['candidates'][1]=candidate(2,6,1,False,13.);self.run['rows']=self.run['rows'][:12]
        self.assertEqual(audit_schedule(self.run)['adopted_versions'],[1])
    def test_late_candidate_cannot_publish(self):
        self.run['candidates'][1]['generation_ms']=13.;self.run['candidates'][1]['late']=True
        with self.assertRaises(ValueError):audit_schedule(self.run)
    def test_oldlaw_may_not_extend_for_next_commitment(self):
        self.run['candidates'][1]=candidate(2,6,1,False,13.);self.run['rows']=self.run['rows'][:12];self.run['candidates'].append(candidate(3,12,1,False))
        with self.assertRaises(ValueError):audit_schedule(self.run)
    def test_observation_cannot_bind_superseded_epoch(self):
        self.run['candidates'][1]['source_version']=0
        with self.assertRaises(ValueError):audit_schedule(self.run)
    def test_missing_optimized_sample_rejected(self):
        self.run['candidates'][0]['stages'].pop()
        with self.assertRaises(ValueError):audit_schedule(self.run)
    def test_publish_must_wait_for_commit_boundary(self):
        self.run['rows'][5]['active_version']=1
        with self.assertRaises(ValueError):audit_schedule(self.run)
if __name__=='__main__':unittest.main()
