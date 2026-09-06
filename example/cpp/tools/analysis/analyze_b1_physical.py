#!/usr/bin/env python3
"""Physical B1 evidence v2. No frozen-analyzer verdict is an input.

This module produces empirical evidence, not a planner feasibility certificate.
Only static, axis-aligned, world-attached 5 cm box scenes are covered.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

LEGS = ('FR', 'FL', 'RR', 'RL')
SCHEMA = 'b1-physical-evidence-v2'
POLICY = dict(force_witness_N=10.0, top_witness_s=0.020, stable_exit_s=0.45,
              rear_clearance_m=0.02, truth_gap_s=0.010, controller_gap_s=0.020,
              posture_max_deg=15.0, minimum_base_height_m=0.28,
              aerial_witness_s=0.004, diagonal_witness_s=0.004)

def f(row, key):
    try:
        x = float(row[key])
        return x if math.isfinite(x) else math.nan
    except (KeyError, ValueError, TypeError):
        return math.nan

def quantile(xs, q):
    xs = sorted(x for x in xs if math.isfinite(x))
    if not xs: return None
    p = (len(xs)-1)*q; lo = int(p); hi = min(lo+1, len(xs)-1)
    return xs[lo] + (xs[hi]-xs[lo])*(p-lo)

def norm(row, prefix):
    return math.sqrt(sum(f(row, prefix+a+'_N')**2 for a in 'xyz'))

def spans(rows, predicate):
    """Observed intervals only: no invented duration after the last sample."""
    result = []; start = last = None
    for row in rows:
        t = f(row, 'time_s')
        good = math.isfinite(t) and predicate(row)
        if good and start is not None and t-last <= POLICY['truth_gap_s']:
            last = t
        else:
            if start is not None: result.append((start, last))
            start = last = t if good else None
    if start is not None: result.append((start, last))
    return result

def longest(rows, predicate):
    return max((b-a for a,b in spans(rows,predicate)), default=0.0)

def angles(row):
    w,x,y,z = (f(row, 'base_quat_'+a) for a in 'wxyz')
    n = math.sqrt(w*w+x*x+y*y+z*z)
    if not math.isfinite(n) or abs(n-1)>0.01: return math.nan,math.nan
    w,x,y,z = (v/n for v in (w,x,y,z))
    return (math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
            math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x))))))

def box_scene(path):
    root = ET.parse(path).getroot()
    boxes = []
    for geom in root.findall('./worldbody/geom'):
        if not geom.get('name','').startswith('phase2_step'): continue
        if geom.get('type') != 'box' or any(k in geom.attrib for k in ('quat','euler','axisangle','xyaxes','zaxis')):
            raise ValueError('unsupported step shape or frame')
        pos = [float(x) for x in geom.get('pos','').split()]
        size = [float(x) for x in geom.get('size','').split()]
        if len(pos)!=3 or len(size)!=3 or not all(math.isfinite(x) for x in pos+size):
            raise ValueError('invalid box geometry')
        if min(size)<=0 or abs(pos[2]+size[2]-.05)>1e-9 or abs(pos[2]-size[2])>1e-9:
            raise ValueError('not a ground-based 5 cm box')
        boxes.append(dict(name=geom.get('name'),front=pos[0]-size[0],rear=pos[0]+size[0],
                          left=pos[1]-size[1],right=pos[1]+size[1],top=pos[2]+size[2]))
    if len(boxes)!=1: raise ValueError('this evidence version covers one box only')
    return boxes[0]

def analyze(truth, control, box):
    issues = []; metrics = {}; gates = {}
    times = [f(r,'time_s') for r in truth]
    required_truth = ['phase2_terrain_foot_contact_mask','time_s','base_pos_world_x_m','base_pos_world_z_m',
        'robot_collision_rear_bound_world_x_m','phase2_terrain_nonfoot_contact_count',
        'phase2_terrain_nonfoot_contact_force_N','base_qvel_world_x_mps']
    required_truth += ['base_quat_'+a for a in 'wxyz']
    required_truth += ['total_contact_grf_world_'+a+'_N' for a in 'xyz']
    for l in LEGS:
        required_truth += [l+'_pos_world_'+a+'_m' for a in 'xyz']
        required_truth += [l+'_foot_contact_grf_world_'+a+'_N' for a in 'xyz']
        required_truth += [l+'_terrain_top_grf_world_z_N',l+'_terrain_nontop_contact_force_N']
    missing = sorted(set(required_truth)-set(truth[0] if truth else {}))
    if missing: issues.append(dict(kind='coverage',missing_truth_fields=missing))
    time_ok = len(times)>1 and all(math.isfinite(t) for t in times) and all(0<b-a<=POLICY['truth_gap_s']+1e-9 for a,b in zip(times,times[1:]))
    gates['truth_time_coverage'] = time_ok
    if not truth or not time_ok:
        return dict(schema=SCHEMA,status='NOT_CERTIFIED',gates=gates,issues=issues,metrics=metrics)
    # Approach begins with the base 0.5 m before the observed step front.
    approach = next((f(r,'time_s') for r in truth if f(r,'base_pos_world_x_m')>=box['front']-.5),None)
    clear = lambda r: (f(r,'robot_collision_rear_bound_world_x_m')>box['rear']+POLICY['rear_clearance_m'] and
        all(f(r,l+'_pos_world_x_m')>box['rear']+POLICY['rear_clearance_m'] for l in LEGS))
    exits = [(a,b) for a,b in spans(truth,clear) if b-a>=POLICY['stable_exit_s']-1e-9]
    exit_end = exits[0][0]+POLICY['stable_exit_s'] if exits else None
    proxy_clear = lambda r: (f(r,'base_pos_world_x_m')>box['rear']+.2 and
        all(f(r,l+'_pos_world_x_m')>box['rear']+.02 for l in LEGS))
    proxy_exits=[(a,b) for a,b in spans(truth,proxy_clear) if b-a>=POLICY['stable_exit_s']-1e-9]
    proxy_end=proxy_exits[0][0]+POLICY['stable_exit_s'] if proxy_exits else None
    # Older traces can still supply bounded diagnostics. This proxy can never
    # satisfy the full-geometry exit gate or repair missing coverage.
    end = exit_end if exit_end is not None else (proxy_end if proxy_end is not None else times[-1])
    metrics['window_end_source']='collision_geometry' if exit_end is not None else ('uncertified_base_feet_proxy' if proxy_end is not None else 'trace_end')
    window = [r for r in truth if approach is not None and approach<=f(r,'time_s')<=end+1e-9]
    gates['complete_body_and_feet_exit'] = bool(exits)
    gates['started_before_step'] = f(truth[0],'base_pos_world_x_m') < box['front']-.5
    gates['finite_physical_window'] = bool(window) and not missing and all(math.isfinite(f(r,k)) for r in window for k in required_truth)
    metrics.update(approach_time_s=approach,exit_end_time_s=exit_end,
                   max_base_x_m=max(f(r,'base_pos_world_x_m') for r in truth),
                   base_proxy_rear_crossed=any(f(r,'base_pos_world_x_m')>box['rear']+.2 for r in truth))
    support = {}
    for l in LEGS:
        key=l+'_terrain_top_grf_world_z_N'
        predicate=lambda r,l=l,key=key: (f(r,key)>=POLICY['force_witness_N'] and
            box['front']<=f(r,l+'_pos_world_x_m')<=box['rear'] and
            box['left']<=f(r,l+'_pos_world_y_m')<=box['right'])
        support[l] = dict(longest_support_s=longest(window,predicate),
                         max_top_force_N=max((f(r,key) for r in window),default=math.nan))
    gates['each_leg_force_supported_on_top'] = all(v['longest_support_s']>=POLICY['top_witness_s']-1e-9 for v in support.values())
    metrics['top_support']=support
    nonfoot=[r for r in window if f(r,'phase2_terrain_nonfoot_contact_count')>0]
    gates['no_nonfoot_step_collision']=not nonfoot and bool(window)
    metrics['nonfoot_collision_rows']=len(nonfoot)
    metrics['first_nonfoot_collision_time_s']=f(nonfoot[0],'time_s') if nonfoot else None
    metrics['max_nontop_foot_force_N']={l:max((f(r,l+'_terrain_nontop_contact_force_N') for r in window),default=math.nan) for l in LEGS}
    metrics['foot_riser_contact_free']=bool(window) and all(v<=1e-6 for v in metrics['max_nontop_foot_force_N'].values())
    rp=[angles(r) for r in window]
    metrics['roll_abs_max_deg']=max((abs(x) for x,y in rp),default=math.nan)
    metrics['pitch_abs_max_deg']=max((abs(y) for x,y in rp),default=math.nan)
    gates['posture_and_height'] = bool(window) and all(math.isfinite(x) and math.isfinite(y) and max(abs(x),abs(y))<=POLICY['posture_max_deg'] for x,y in rp) and all(f(r,'base_pos_world_z_m')>=POLICY['minimum_base_height_m'] for r in window)
    interaction=[r for r in window if f(r,'phase2_terrain_foot_contact_mask')>0]
    motion=[]
    if interaction:
        a,b=f(interaction[0],'time_s'),f(interaction[-1],'time_s')
        motion=[r for r in window if a<=f(r,'time_s')<=b]
        metrics['obstacle_interaction_time_s']=[a,b]
    contact_mask=lambda r: sum(1<<i for i,l in enumerate(LEGS) if norm(r,l+'_foot_contact_grf_world_')>=POLICY['force_witness_N'])
    metrics['diagonal_witness_s']={str(m):longest(motion,lambda r,m=m:contact_mask(r)==m) for m in (9,6)}
    metrics['aerial_witness_s']=longest(motion,lambda r:norm(r,'total_contact_grf_world_')<POLICY['force_witness_N'])
    metrics['forward_speed_p05_mps']=quantile([f(r,'base_qvel_world_x_mps') for r in motion],.05)
    metrics['forward_speed_p50_mps']=quantile([f(r,'base_qvel_world_x_mps') for r in motion],.5)
    gates['dynamic_contact_witness']=bool(motion) and metrics['aerial_witness_s']>=POLICY['aerial_witness_s']-1e-9 and all(v>=POLICY['diagonal_witness_s']-1e-9 for v in metrics['diagonal_witness_s'].values())
    # LowState.tick is round(MuJoCo time / 1 ms); duplicates are legal DDS reuse.
    ctimes=[f(r,'state_tick_s') for r in control]
    ctime_ok=bool(ctimes) and all(math.isfinite(t) for t in ctimes) and all(b>=a for a,b in zip(ctimes,ctimes[1:]))
    assoc=[]; gaps=[]
    if ctime_ok:
        for r in window:
            t=f(r,'time_s'); i=bisect.bisect_right(ctimes,t)-1
            if i<0 or t-ctimes[i]>POLICY['controller_gap_s']+1e-9: gaps.append(t)
            else: assoc.append(control[i])
    gates['controller_window_coverage']=bool(window) and ctime_ok and not gaps and len(assoc)==len(window)
    metrics['uncovered_truth_rows']=len(gaps) if ctime_ok else len(window)
    gates['estimated_state_and_lidar']=bool(assoc) and all(f(r,'has_state')==1 and f(r,'terrain_model_com_valid')==1 and abs(f(r,'terrain_model_com_state_stamp_s')-f(r,'state_tick_s'))<=.004 and r.get('terrain_map_source')=='lidar' and 0<=f(r,'state_tick_s')-f(r,'telemetry_lidar_stamp_s')<=.20 for r in assoc)
    gates['continuous_active_execution']=bool(assoc) and all(f(r,'motion_stage')==2 and f(r,'velocity_command_active')==1 and f(r,'terrain_actuation')==1 and f(r,'terrain_sensor_only')==0 for r in assoc)
    metrics['effective_duty_range']=[quantile([f(r,'velocity_command_gait_duty') for r in assoc],0),quantile([f(r,'velocity_command_gait_duty') for r in assoc],1)]
    metrics['id_wbc_ok_fraction']=sum(f(r,'wbc_full_id_ok')==1 for r in assoc)/len(assoc) if assoc else None
    return dict(schema=SCHEMA,status='PASS' if all(gates.values()) else 'NOT_CERTIFIED',
                claim='empirical physical traversal only; not planner feasibility or campaign acceptance',
                policy=POLICY,gates=gates,issues=issues,metrics=metrics)

def safe(value):
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {k:safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [safe(v) for v in value]
    return value

def run(path, scene):
    files=[path/'contact_ground_truth.csv',path/'data.csv',scene]
    with files[0].open() as h: truth=list(csv.DictReader(h))
    with files[1].open() as h: control=list(csv.DictReader(h))
    result=analyze(truth,control,box_scene(scene))
    result['input_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    result['analyzer_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    return safe(result)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--scene',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    result=run(a.run_dir,a.scene)
    # New output only: preserve earlier raw verdicts.
    with a.out.open('x',encoding='utf-8') as h: json.dump(result,h,indent=2,allow_nan=False)
    print(json.dumps({'status':result['status'],'gates':result.get('gates',{})}))
    sys.exit(0 if result['status']=='PASS' else 1)
