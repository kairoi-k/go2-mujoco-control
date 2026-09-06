import unittest
from audit_b1_impulse import audit

def fixture(aerial=False):
    rows=[];v=[0.,0.,0.];dt=.002;m=15.
    for i in range(101):
        force=[0.,0.,0.] if aerial else [2.*i,0.,m*9.81+10.]
        row={"time_s":i*dt,"total_mass_kg":m}
        for j,a in enumerate("xyz"):
            g=-9.81 if a=="z" else 0.
            row["subtree_linvel_world_"+a+"_mps"]=v[j]
            row["total_contact_grf_world_"+a+"_N"]=force[j]
            row["gravity_world_"+a+"_mps2"]=g
            v[j]+=(force[j]/m+g)*dt
        rows.append(row)
    return rows
class TestImpulse(unittest.TestCase):
    def test_known_force_and_aerial(self):
        for air in (False,True):
            r=audit(fixture(air),0,.2)
            self.assertLess(max(w["residual_norm_Ns_p50_p95_max"][-1] for w in r["windows"].values()),1e-11)
    def test_injected_momentum(self):
        rows=fixture(); rows[50]["subtree_linvel_world_x_mps"]+=1
        self.assertGreater(audit(rows,0,.2)["windows"]["1"]["residual_norm_Ns_p50_p95_max"][-1],14.99)
    def test_unknown_and_gaps(self):
        for key,value in [("time_s",float("nan")),("total_mass_kg",0),("total_contact_grf_world_z_N",float("nan"))]:
            rows=fixture();rows[50][key]=value
            with self.assertRaises(ValueError):audit(rows,0,.2)
        rows=fixture(); del rows[10:20]
        with self.assertRaises(ValueError):audit(rows,0,.2)
    def test_missing_interval(self):
        with self.assertRaises(ValueError):audit(fixture(),-.1,.2)
if __name__=="__main__":unittest.main()
