import pathlib,sys,unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'tools/research'))
from swing_surface_reference import surface_envelope,height_at
class GeometryEnvelope(unittest.TestCase):
 def make(self,x,h0,h1,y=0):return surface_envelope([0,1],[[x[0],y],[x[1],y]],[[0,-.5],[.5,.5]],.05,.02,h0,h1)
 def test_ascending(self):
  e=self.make([-.2,.2],0,.05);self.assertAlmostEqual(e['entry'],.45);self.assertEqual(height_at(e,.45),.05);self.assertEqual(height_at(e,0),0);self.assertEqual(height_at(e,1),.05)
 def test_descending(self):
  e=self.make([.3,.8],.05,0);self.assertAlmostEqual(e['exit'],.44);self.assertEqual(height_at(e,.44),.05);self.assertEqual(height_at(e,1),0)
 def test_crossing(self):
  e=self.make([-.2,.8],0,0);self.assertAlmostEqual(e['entry'],.18);self.assertAlmostEqual(e['exit'],.72);self.assertEqual(height_at(e,.5),.05)
 def test_outside(self):self.assertEqual(height_at(self.make([-.2,.8],0,0,1),.5),0)
 def test_c1(self):
  e=self.make([-.2,.8],0,0)
  for s in (e['entry'],e['exit']):self.assertLess(abs(height_at(e,s+1e-7)-height_at(e,s-1e-7))/2e-7,1e-6)
 def test_conflicting_start(self):
  with self.assertRaises(ValueError):self.make([.2,.8],0,0)
 def test_nonfinite(self):
  with self.assertRaises(ValueError):self.make([np.nan,.8],0,0)
if __name__=='__main__':unittest.main()
