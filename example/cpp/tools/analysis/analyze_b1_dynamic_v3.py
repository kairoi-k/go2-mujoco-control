#!/usr/bin/env python3
"""B1 V3 empirical gates. Historical V2 and frozen analyzers are unchanged."""
import argparse,bisect,csv,hashlib,json,math,re
from pathlib import Path
import analyze_b1_physical as physical
import b1_dynamic_protocol as protocol
FATAL=re.compile(r'Trot IK failed|Trot hard safety limit reached|Trot hard posture limit|Terrain gait lift unavailable|terminate called|Segmentation fault|Aborted|FATAL')
PHYSICAL_GATES=('truth_time_coverage','complete_body_and_feet_exit','started_before_step','finite_physical_window','each_leg_force_supported_on_top','no_nonfoot_step_collision','posture_and_height','controller_window_coverage')

def analyze(truth,control,box,log,statuses):
    f=physical.f
    baseline=physical.analyze(truth,control,box)
    gates={k:baseline.get('gates',{}).get(k,False) for k in PHYSICAL_GATES}
    issues=[];detail={}
    fields=['phase2_terrain_nonfoot_contact_count']+[l+s for l in physical.LEGS for s in ['_terrain_top_grf_world_z_N','_terrain_nontop_contact_force_N']]
    finite_contact=bool(truth) and all(math.isfinite(f(r,k)) for r in truth for k in fields)
    touching=[r for r in truth if f(r,'phase2_terrain_nonfoot_contact_count')>0 or any(abs(f(r,l+'_terrain_top_grf_world_z_N'))>1e-6 or f(r,l+'_terrain_nontop_contact_force_N')>1e-6 for l in physical.LEGS)]
    gates['whole_trace_step_collision_free']=finite_contact and all(f(r,'phase2_terrain_nonfoot_contact_count')==0 and all(f(r,l+'_terrain_nontop_contact_force_N')<=1e-6 for l in physical.LEGS) for r in truth)
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
        for k in ['approach_interaction_join_coverage','stable_running_approach','complete_cycle_topology','interaction_speed']:gates[k]=False
    active=[r for r in control if f(r,'motion_stage')==2 and f(r,'velocity_command_active')==1]
    gait=[f(r,'telemetry_gait_time_s') for r in active]
    profile_coverage=bool(gait) and all(math.isfinite(t) for t in gait) and gait[0]<=.02 and gait[-1]>=31.99 and all(0<=b-a<=.020000001 for a,b in zip(gait,gait[1:]))
    gates['complete_profile_and_tail_coverage']=profile_coverage
    detail['active_gait_interval_s']=[gait[0],gait[-1]] if gait else None
    knots=[(0.,0.),(8.,0.),(16.,1.),(24.,1.),(26.,0.),(30.,0.)]
    def expected(t):
        if t<=0:return 0.
        for (a,x),(b,y) in zip(knots,knots[1:]):
            if t<=b:return x+(y-x)*(t-a)/(b-a)
        return 0.
    # Telemetry is rounded to ns/1e-9 m/s; this checks the selected profile,
    # not a tracking tolerance or a new physical acceptance threshold.
    mismatches=[r for r in active if not math.isfinite(f(r,'velocity_command_requested_mps')) or abs(f(r,'velocity_command_requested_mps')-expected(f(r,'telemetry_gait_time_s')))>.00001]
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
    result['analyzer_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(physical.__file__),Path(protocol.__file__)]}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--scene',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=run(a.run_dir,a.scene)
    with a.out.open('x') as h:json.dump(result,h,indent=2,allow_nan=False)
    print(json.dumps({'status':result['status'],'gates':result['gates']}))
    raise SystemExit(0 if result['status']=='PASS' else 1)
