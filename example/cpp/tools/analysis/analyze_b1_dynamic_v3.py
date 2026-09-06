#!/usr/bin/env python3
"""B1 V3 empirical gates. Historical V2 and frozen analyzers are unchanged."""
import argparse,bisect,csv,hashlib,json,math,re
from pathlib import Path
import analyze_b1_physical as physical
import b1_dynamic_protocol as protocol
FATAL=re.compile(r'Trot IK failed|Trot hard safety limit reached|Trot hard posture limit|Trot hard joint limit|Requested motion|Trot cycle quality guard rejected|Trot safety rejected|Terrain gait lift unavailable|terminate called|Segmentation fault|Aborted|FATAL')
PHYSICAL_GATES=('truth_time_coverage','complete_body_and_feet_exit','started_before_step','finite_physical_window','each_leg_force_supported_on_top','no_nonfoot_step_collision','posture_and_height','controller_window_coverage')

def analyze(truth,control,box,log,statuses):
    f=physical.f
    baseline=physical.analyze(truth,control,box)
    gates={k:baseline.get('gates',{}).get(k,False) for k in PHYSICAL_GATES}
    issues=[];detail={}
    expected_box=dict(front=5.,rear=5.5,left=-.75,right=.75,top=.05)
    gates['registered_v3_scene']=all(math.isfinite(box.get(k,math.nan)) and abs(box[k]-v)<=1e-9 for k,v in expected_box.items())
    fields=['phase2_terrain_nonfoot_contact_count','phase2_terrain_nonfoot_contact_force_N']+[l+s for l in physical.LEGS for s in ['_terrain_top_grf_world_z_N','_terrain_nontop_contact_force_N']]
    finite_contact=bool(truth) and all(math.isfinite(f(r,k)) for r in truth for k in fields)
    norm_fields=['phase2_terrain_nonfoot_contact_force_N']+[l+'_terrain_nontop_contact_force_N' for l in physical.LEGS]
    contact_input_ok=finite_contact and all(f(r,k)>=0 for r in truth for k in norm_fields) and all(f(r,'phase2_terrain_nonfoot_contact_count')>=0 and f(r,'phase2_terrain_nonfoot_contact_count').is_integer() for r in truth)
    gates['contact_input_consistency']=contact_input_ok
    required_truth=fields+['time_s','robot_collision_rear_bound_world_x_m']+['base_pos_world_'+axis+'_m' for axis in 'xyz']+['base_qvel_world_'+axis+'_mps' for axis in 'xyz']+['base_quat_'+axis for axis in 'wxyz']+['total_contact_grf_world_'+axis+'_N' for axis in 'xyz']
    required_truth += [l+'_pos_world_'+axis+'_m' for l in physical.LEGS for axis in 'xyz']+[l+'_foot_contact_grf_world_'+axis+'_N' for l in physical.LEGS for axis in 'xyz']
    gates['whole_trace_required_truth_finite']=bool(truth) and all(math.isfinite(f(r,k)) for r in truth for k in required_truth)
    touching=[r for r in truth if f(r,'phase2_terrain_nonfoot_contact_count')>0 or f(r,'phase2_terrain_nonfoot_contact_force_N')>1e-6 or any(abs(f(r,l+'_terrain_top_grf_world_z_N'))>1e-6 or f(r,l+'_terrain_nontop_contact_force_N')>1e-6 for l in physical.LEGS)]
    gates['whole_trace_step_collision_free']=contact_input_ok and all(f(r,'phase2_terrain_nonfoot_contact_count')==0 and f(r,'phase2_terrain_nonfoot_contact_force_N')<=1e-6 and all(f(r,l+'_terrain_nontop_contact_force_N')<=1e-6 for l in physical.LEGS) for r in truth)
    ct=[f(r,'state_tick_s') for r in control]
    ordered=bool(ct) and all(math.isfinite(t) for t in ct) and all(b>=a for a,b in zip(ct,ct[1:]))
    if touching and ordered:
        start,end=f(touching[0],'time_s'),f(touching[-1],'time_s')
        detail['interaction_s']=[start,end]
        joined=[];uncovered=[]
        for r in truth:
            t=f(r,'time_s')
            if not start-.8<=t<=end:continue
            i=bisect.bisect_right(ct,t)-1
            if i<0 or t-ct[i]>.020000001:uncovered.append(t);continue
            joined.append(dict(control[i],**r))
        gates['approach_interaction_join_coverage']=not uncovered and bool(joined)
        interaction_control=[r for r in joined if start<=f(r,'time_s')<=end]
        gates['nominal_running_through_interaction']=bool(interaction_control) and all(abs(f(r,'velocity_command_gait_period_s')-.14)<=.005 and abs(f(r,'velocity_command_gait_duty')-.44)<=.01 and f(r,'motion_stage')==2 and f(r,'velocity_command_active')==1 for r in interaction_control)
        detail['uncovered_truth_rows']=len(uncovered)
        detail['approach']=protocol.stable_approach(joined,start)
        local_control=[r for r in control if start-.3<=f(r,'state_tick_s')<=end+.3]
        local_truth=[r for r in truth if start-.3<=f(r,'time_s')<=end+.3]
        detail['topology']=protocol.complete_cycle_topology(local_control,local_truth,start,end)
        gates['stable_running_approach']=detail['approach']['status']=='PASS'
        gates['complete_cycle_topology']=detail['topology']['status']=='PASS'
        motion=[r for r in truth if start<=f(r,'time_s')<=end]
        v=[f(r,'base_qvel_world_x_mps') for r in motion]
        p05,p50=physical.quantile(v,.05),physical.quantile(v,.5)
        detail['interaction_speed']={'p05':p05,'median':p50}
        gates['interaction_speed']=bool(v) and all(math.isfinite(x) for x in v) and p05>=.5 and .8<=p50<=1.2
    else:
        issues.append('missing interaction or invalid controller time order')
        for k in ['nominal_running_through_interaction','approach_interaction_join_coverage','stable_running_approach','complete_cycle_topology','interaction_speed']:gates[k]=False
    active=[r for r in control if f(r,'motion_stage') in (2,3) and f(r,'velocity_command_active')==1]
    gait=[f(r,'telemetry_gait_time_s') for r in active]
    profile_coverage=bool(gait) and all(math.isfinite(t) for t in gait) and gait[0]<=.02 and gait[-1]>=31.99 and all(0<=b-a<=.020000001 for a,b in zip(gait,gait[1:]))
    gates['complete_profile_and_tail_coverage']=profile_coverage
    offset=f(active[0],'state_tick_s')-gait[0] if active else math.nan
    gates['profile_state_clock_consistency']=bool(active) and math.isfinite(offset) and all(abs((f(r,'state_tick_s')-f(r,'telemetry_gait_time_s'))-offset)<=.020000001 for r in active)
    start_index=next((i for i,r in enumerate(control) if f(r,'motion_stage') in (2,3) and f(r,'velocity_command_active')==1),len(control))
    profile_observed=[r for r in control[start_index:] if f(r,'state_tick_s')<=offset+32.020000001]
    gates['profile_motion_lifecycle']=bool(profile_observed) and all(f(r,'velocity_command_active')==1 and (f(r,'motion_stage')==2 if f(r,'telemetry_gait_time_s')<29.99 else f(r,'motion_stage') in (2,3)) for r in profile_observed)
    required_active=[r for r in active if f(r,'telemetry_gait_time_s')<=32.000001]
    truth_times=[f(r,'time_s') for r in truth]
    gates['profile_truth_coverage']=profile_coverage and bool(required_active) and bool(truth_times) and all(math.isfinite(t) for t in truth_times) and truth_times[0]<=f(required_active[0],'state_tick_s')+.010000001 and truth_times[-1]>=f(required_active[-1],'state_tick_s')-.010000001
    profile_truth=[r for r in truth if math.isfinite(offset) and offset<=f(r,'time_s')<=offset+32.]
    full_join_ok=bool(profile_truth) and ordered
    if full_join_ok:
        for r in profile_truth:
            t=f(r,'time_s');i=bisect.bisect_right(ct,t)-1
            if i<0 or t-ct[i]>.020000001:full_join_ok=False;break
    gates['whole_profile_state_truth_join']=full_join_ok
    gates['whole_profile_posture_and_height']=bool(profile_truth) and all(all(math.isfinite(x) and abs(x)<=15 for x in physical.angles(r)) and f(r,'base_pos_world_z_m')>=.28 for r in profile_truth)
    gates['runtime_integrity']=all(str(statuses.get(k))=='0' for k in ['quality_status','ground_truth_status','dynamics_status'])
    detail['active_gait_interval_s']=[gait[0],gait[-1]] if gait else None
    knots=[(0.,0.),(8.,0.),(16.,1.),(24.,1.),(26.,0.),(30.,0.)]
    def expected(t):
        if t<=0:return 0.
        for (a,x),(b,y) in zip(knots,knots[1:]):
            if t<=b:return x+(y-x)*(t-a)/(b-a)
        return 0.
    # Telemetry is rounded to ns/1e-9 m/s; this checks the selected profile,
    # not a tracking tolerance or a new physical acceptance threshold.
    mismatches=[r for r in active if not math.isfinite(f(r,'velocity_command_requested_mps')) or abs(f(r,'velocity_command_requested_mps')-expected(f(r,'telemetry_gait_time_s')))>.000001]
    gates['registered_requested_profile']=bool(active) and not mismatches
    detail['profile_mismatch_rows']=len(mismatches)
    gates['no_fatal_or_safety_stop']=not FATAL.search(log) and all(str(statuses.get(k))=='0' for k in ['controller_status','completion_status','safety_status']) and bool(active) and all(f(r,'terrain_safe_stop_requested')==0 for r in active)
    # Sensor-only or a replacement controller can establish physical traversal;
    # map provenance and actuation-route flags remain explicit diagnostics.
    detail['route_flags']={k:sorted({r.get(k,'MISSING') for r in active}) for k in ['terrain_actuation','terrain_sensor_only']}
    return physical.safe({'schema':'b1-dynamic-traversal-v3','status':'PASS' if all(gates.values()) else 'NOT_CERTIFIED','claim':'single-run empirical evidence; repetitions and holdouts still required','gates':gates,'issues':issues,'detail':detail,'physical_v2_diagnostic':baseline,'legacy_statuses':statuses})

def run(path,scene):
    inputs=[path/'contact_ground_truth.csv',path/'data.csv',path/'controller.log',path/'run_manifest.json',scene]
    with inputs[0].open() as h:t=list(csv.DictReader(h))
    with inputs[1].open() as h:c=list(csv.DictReader(h))
    manifest=json.loads(inputs[3].read_text())
    result=analyze(t,c,physical.box_scene(scene),inputs[2].read_text(),manifest.get('statuses',{}))
    result['input_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    artifacts=manifest.get('artifacts',{});repo=manifest.get('repository',{})
    # Bind the supplied scene and fixed profile to the recorded run. This does
    # not replace the campaign's independent source/binary build audit.
    result['gates']['recorded_run_binding']=(
        artifacts.get('scenario_sha256')==result['input_sha256'][str(scene)]
        and manifest.get('profile',{}).get('sha256')=='62f2ff9701b2f2d6189cad97877d7891177a32b17ded255677434eb1d8f41ac6'
        and str(repo.get('git_dirty')).lower()=='false'
        and re.fullmatch(r'[0-9a-f]{40}',str(repo.get('git_commit',''))) is not None
        and all(re.fullmatch(r'[0-9a-f]{64}',str(artifacts.get(k,''))) is not None for k in ['controller_sha256','simulator_sha256']))
    result['status']='PASS' if all(result['gates'].values()) else 'NOT_CERTIFIED'
    result['analyzer_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(physical.__file__),Path(protocol.__file__)]}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--scene',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=run(a.run_dir,a.scene)
    with a.out.open('x') as h:json.dump(result,h,indent=2,allow_nan=False)
    print(json.dumps({'status':result['status'],'gates':result['gates']}))
    raise SystemExit(0 if result['status']=='PASS' else 1)
