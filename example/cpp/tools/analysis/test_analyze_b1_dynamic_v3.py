"""Synthetic bookkeeping streams; no dynamics feasibility claim."""
import unittest
import analyze_b1_dynamic_v3 as v3
from analyze_b1_physical import LEGS
BOX=dict(front=5.,rear=5.5,left=-.75,right=.75,top=.05)
STATUS=dict(controller_status=0,completion_status=0,safety_status=0)
def fixture():
    truth=[];control=[]
    for i in range(16002):
        t=i*.002;x=max(0.,min(t-14.,12.))
        phase=(i%70)/70;mask=9 if phase<.3 or phase>=.9 else (6 if .4<=phase<.8 else 0)
        r=dict(time_s=t,base_pos_world_x_m=x,base_pos_world_z_m=.35,base_qvel_world_x_mps=1. if 14<t<26 else 0.,base_quat_w=1.,base_quat_x=0.,base_quat_y=0.,base_quat_z=0.,robot_collision_rear_bound_world_x_m=x-.4,phase2_terrain_nonfoot_contact_count=0,phase2_terrain_nonfoot_contact_force_N=0)
        total=0;topmask=0
        for j,l in enumerate(LEGS):
            foot_x=x+(.2 if j<2 else -.2);top=5<=foot_x<=5.5;force=100 if mask&(1<<j) else 0
            for a,val in zip('xyz',(foot_x,.1 if j%2 else -.1,.072 if top else .022)):r[l+'_pos_world_'+a+'_m']=val
            for a in 'xyz':r[l+'_foot_contact_grf_world_'+a+'_N']=force if a=='z' else 0
            r[l+'_terrain_top_grf_world_z_N']=force if top else 0;r[l+'_terrain_nontop_contact_force_N']=0
            if top and force:topmask|=1<<j
            total+=force
        r['phase2_terrain_foot_contact_mask']=topmask
        for a in 'xyz':r['total_contact_grf_world_'+a+'_N']=total if a=='z' else 0
        truth.append(r)
        request=0 if t<8 or t>=26 else ((t-8)/8 if t<16 else (1 if t<24 else (26-t)/2))
        control.append(dict(state_tick_s=t,telemetry_gait_time_s=t,motion_stage=2,velocity_command_active=1,velocity_command_requested_mps=request,velocity_command_gait_period_s=.14,velocity_command_gait_duty=.44,cycle_index=i//70,phase=phase,terrain_actuation=0,terrain_sensor_only=1,terrain_safe_stop_requested=0,wbc_full_id_ok=1))
    return truth,control
class DynamicV3Tests(unittest.TestCase):
    def test_full_bookkeeping_stream_can_pass_without_planner_route(self):
        t,c=fixture();r=v3.analyze(t,c,BOX,'controlled stop',STATUS)
        self.assertEqual(r['status'],'PASS',r['gates'])
    def test_process_success_does_not_repair_missing_tail(self):
        t,c=fixture();c=[r for r in c if r['telemetry_gait_time_s']<29]
        r=v3.analyze(t,c,BOX,'controlled stop',STATUS)
        self.assertFalse(r['gates']['complete_profile_and_tail_coverage'])
    def test_late_collision_outside_passage_window_is_rejected(self):
        t,c=fixture();t[-10]['FR_terrain_nontop_contact_force_N']=1
        r=v3.analyze(t,c,BOX,'controlled stop',STATUS)
        self.assertFalse(r['gates']['whole_trace_step_collision_free'])
if __name__=='__main__':unittest.main()
