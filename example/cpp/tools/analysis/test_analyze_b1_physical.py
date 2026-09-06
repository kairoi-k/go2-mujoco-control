import copy
import math
import unittest
from analyze_b1_physical import analyze, LEGS

BOX=dict(front=.7,rear=1.2,left=-.75,right=.75,top=.05)

def fixture():
    # Synthetic bookkeeping stream, not a dynamic-feasibility oracle.
    truth=[]; control=[]
    for i in range(1001):
        t=i*.002; x=1.2*t
        r=dict(time_s=t,base_pos_world_x_m=x,base_pos_world_z_m=.35,
            base_qvel_world_x_mps=1.2,base_quat_w=1,base_quat_x=0,
            base_quat_y=0,base_quat_z=0,robot_collision_rear_bound_world_x_m=x-.4,
            phase2_terrain_nonfoot_contact_count=0,phase2_terrain_nonfoot_contact_force_N=0)
        phase=i%100
        mask=9 if phase<40 else (6 if 50<=phase<90 else 0)
        topmask=0; total=0
        for j,l in enumerate(LEGS):
            foot_x=x+(.2 if j<2 else -.2); on_top=.7<=foot_x<=1.2
            force=100 if mask&(1<<j) else 0
            for a,v in zip('xyz',(foot_x, .1 if j%2 else -.1,.072 if on_top else .022)):
                r[l+'_pos_world_'+a+'_m']=v
            for a in 'xyz': r[l+'_foot_contact_grf_world_'+a+'_N']=force if a=='z' else 0
            r[l+'_terrain_top_grf_world_z_N']=force if on_top else 0
            r[l+'_terrain_nontop_contact_force_N']=0
            if on_top and force: topmask|=1<<j
            total+=force
        r['phase2_terrain_foot_contact_mask']=topmask
        for a in 'xyz':r['total_contact_grf_world_'+a+'_N']=total if a=='z' else 0
        truth.append(r)
        c=dict(state_tick_s=t,has_state=1,terrain_model_com_valid=1,
            terrain_model_com_state_stamp_s=t,terrain_map_source='lidar',
            telemetry_lidar_stamp_s=t,motion_stage=2,velocity_command_active=1,
            terrain_actuation=1,terrain_sensor_only=0,velocity_command_gait_duty=.44,
            wbc_full_id_ok=1)
        control.append(c)
    return truth,control

class PhysicalEvidenceTests(unittest.TestCase):
    def test_complete_witness(self):
        t,c=fixture();self.assertEqual(analyze(t,c,BOX)['status'],'PASS')
    def test_no_crossing(self):
        t,c=fixture();self.assertFalse(analyze(t[:600],c,BOX)['gates']['complete_body_and_feet_exit'])
    def test_missing_body_bound_never_passes_proxy(self):
        t,c=fixture()
        for r in t:r.pop('robot_collision_rear_bound_world_x_m')
        a=analyze(t,c,BOX);self.assertEqual(a['status'],'NOT_CERTIFIED');self.assertTrue(a['metrics']['base_proxy_rear_crossed'])
    def test_riser_force_cannot_be_top_support(self):
        t,c=fixture()
        for r in t:
            r['FR_terrain_nontop_contact_force_N']=r['FR_terrain_top_grf_world_z_N']
            r['FR_terrain_top_grf_world_z_N']=0
        self.assertFalse(analyze(t,c,BOX)['gates']['each_leg_force_supported_on_top'])
    def test_nonfoot_collision(self):
        t,c=fixture();t[400]['phase2_terrain_nonfoot_contact_count']=1
        self.assertFalse(analyze(t,c,BOX)['gates']['no_nonfoot_step_collision'])
    def test_missing_controller_crossing_span(self):
        t,c=fixture();c=[r for r in c if not .7<r['state_tick_s']<.8]
        self.assertFalse(analyze(t,c,BOX)['gates']['controller_window_coverage'])
    def test_asynchronous_start_and_duplicate_state_are_legal(self):
        t,c=fixture();c=c[10:]+[dict(c[-1])]
        self.assertEqual(analyze(t,c,BOX)['status'],'PASS')
    def test_step_mask_is_not_aerial_mask(self):
        t,c=fixture()
        for r in t:r['total_contact_grf_world_z_N']=200
        self.assertFalse(analyze(t,c,BOX)['gates']['dynamic_contact_witness'])
    def test_stale_lidar(self):
        t,c=fixture();c[400]['telemetry_lidar_stamp_s']-=1
        self.assertFalse(analyze(t,c,BOX)['gates']['estimated_state_and_lidar'])
    def test_truth_gap_and_nan(self):
        t,c=fixture();self.assertFalse(analyze(t[:400]+t[410:],c,BOX)['gates']['truth_time_coverage'])
        t,c=fixture();t[400]['FR_terrain_top_grf_world_z_N']=math.nan
        self.assertFalse(analyze(t,c,BOX)['gates']['finite_physical_window'])

if __name__=='__main__':unittest.main()
