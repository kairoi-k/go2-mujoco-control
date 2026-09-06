"""Synthetic bookkeeping streams; no dynamics feasibility claim."""
import unittest
import analyze_b1_dynamic_v3 as v3
from analyze_b1_physical import LEGS
BOX=dict(front=5.,rear=5.5,left=-.75,right=.75,top=.05)
STATUS=dict(controller_status=0,completion_status=0,safety_status=0,quality_status=0,ground_truth_status=0,dynamics_status=0)
def fixture():
    truth=[];control=[]
    for i in range(16002):
        t=i*.002;x=max(0.,min(t-14.,12.))
        phase=(i%70)/70;mask=9 if phase<.3 or phase>=.9 else (6 if .4<=phase<.8 else 0)
        r=dict(time_s=t,base_pos_world_x_m=x,base_pos_world_y_m=0.,base_pos_world_z_m=.35,base_qvel_world_y_mps=0.,base_qvel_world_z_mps=0.,base_qvel_world_x_mps=1. if 14<t<26 else 0.,base_quat_w=1.,base_quat_x=0.,base_quat_y=0.,base_quat_z=0.,robot_collision_rear_bound_world_x_m=x-.4,phase2_terrain_nonfoot_contact_count=0,phase2_terrain_nonfoot_contact_force_N=0)
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
    def test_truncated_truth_cannot_borrow_controller_tail(self):
        t,c=fixture();t=[r for r in t if r['time_s']<25]
        self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',STATUS)['gates']['profile_truth_coverage'])
    def test_nonfoot_force_without_count_is_rejected(self):
        t,c=fixture();t[-10]['phase2_terrain_nonfoot_contact_force_N']=1
        self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',STATUS)['gates']['whole_trace_step_collision_free'])
    def test_negative_contact_norm_is_input_conflict(self):
        t,c=fixture();t[-10]['FR_terrain_nontop_contact_force_N']=-1
        self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',STATUS)['gates']['contact_input_consistency'])
    def test_unregistered_scene_is_not_v3(self):
        t,c=fixture();box=dict(BOX,front=4.9)
        self.assertFalse(v3.analyze(t,c,box,'controlled stop',STATUS)['gates']['registered_v3_scene'])
    def test_runtime_integrity_failure_is_not_legacy_threshold_failure(self):
        t,c=fixture()
        for key in ['quality_status','ground_truth_status','dynamics_status']:
            self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',dict(STATUS,**{key:1}))['gates']['runtime_integrity'])
    def test_braking_stage_after_profile_is_valid_tail(self):
        t,c=fixture()
        for r in c:
            if r['telemetry_gait_time_s']>=30:r['motion_stage']=3
        self.assertEqual(v3.analyze(t,c,BOX,'controlled stop',STATUS)['status'],'PASS')
    def test_compressed_state_clock_cannot_fabricate_full_profile(self):
        t,c=fixture()
        for r in c:
            if r['telemetry_gait_time_s']>21:r['state_tick_s']=21+(r['telemetry_gait_time_s']-21)*.001
        result=v3.analyze(t,c,BOX,'controlled stop',STATUS)
        self.assertFalse(result['gates']['profile_state_clock_consistency'])
        self.assertFalse(result['gates']['whole_profile_state_truth_join'])
    def test_gait_change_during_crossing_is_not_nominal_running(self):
        t,c=fixture()
        for r in c:
            if 19<r['state_tick_s']<19.4:r['velocity_command_gait_period_s']=.2;r['velocity_command_gait_duty']=.5
        self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',STATUS)['gates']['nominal_running_through_interaction'])
    def test_nonfinite_tail_field_cannot_hide_after_exit(self):
        t,c=fixture();t[-1]['base_quat_w']=float('nan')
        result=v3.analyze(t,c,BOX,'controlled stop',STATUS)
        self.assertFalse(result['gates']['whole_trace_required_truth_finite'])
    def test_post_exit_collapse_is_not_a_complete_run(self):
        t,c=fixture();t[-10]['base_pos_world_z_m']=.1
        self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',STATUS)['gates']['whole_profile_posture_and_height'])
    def test_explicit_inactive_row_is_not_filtered_as_missing_sample(self):
        t,c=fixture();c[15500]['motion_stage']=1;c[15500]['velocity_command_active']=0
        self.assertFalse(v3.analyze(t,c,BOX,'controlled stop',STATUS)['gates']['profile_motion_lifecycle'])
    def test_fatal_log_overrides_success_status(self):
        self.assertIsNotNone(v3.FATAL.search('Trot hard joint limit'))
        self.assertIsNotNone(v3.FATAL.search('Trot safety rejected'))
        t,c=fixture()
        self.assertFalse(v3.analyze(t,c,BOX,'Trot hard joint limit',STATUS)['gates']['no_fatal_or_safety_stop'])
if __name__=='__main__':unittest.main()
