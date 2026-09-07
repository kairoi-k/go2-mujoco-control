"""Independent synthetic gait windows: phase coverage and total GRF semantics."""
import pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'tools/research'))
from analyze_shooting_contact_cycles import complete_cycle_diagnostics
class CycleWindowsTest(unittest.TestCase):
 def rows(self,count=140):
  rows=[]
  for k in range(count):
   slot=k%70
   mask=9 if slot<25 else 0 if slot<30 else 6
   rows.append({'time':(k+1)*.002,'body_x_m':k*.002,'mask_10':mask,'aerial_below_10n':mask==0})
  return rows
 def test_both_diagonals_and_real_aerial(self):
  result=complete_cycle_diagnostics(self.rows(),0,0,.14,.002)
  self.assertEqual(len(result),2)
  self.assertTrue(all(r['running_contact_diagnostic'] for r in result))
 def test_no_foot_mask_does_not_prove_total_robot_aerial(self):
  rows=self.rows()
  for row in rows:row['aerial_below_10n']=False
  self.assertFalse(any(r['running_contact_diagnostic'] for r in complete_cycle_diagnostics(rows,0,0,.14,.002)))
 def test_partial_cycles_are_excluded(self):
  result=complete_cycle_diagnostics(self.rows(),0,.5,.14,.002)
  self.assertEqual(len(result),1)
  self.assertEqual(result[0]['samples'],70)
  self.assertAlmostEqual(result[0]['start_s'],.07)
if __name__=='__main__':unittest.main()
