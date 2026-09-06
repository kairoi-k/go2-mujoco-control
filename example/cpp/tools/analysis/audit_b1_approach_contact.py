#!/usr/bin/env python3
"""Descriptive first-contact audit; not a V3 acceptance verdict."""
import argparse, bisect, csv, hashlib, json, math
from pathlib import Path
import analyze_b1_physical as physical

def run(path):
    files=[path/'contact_ground_truth.csv',path/'data.csv']
    with files[0].open() as h: truth=list(csv.DictReader(h))
    with files[1].open() as h: control=list(csv.DictReader(h))
    f=physical.f
    if not truth or not control: raise ValueError('empty input')
    for rows,key,strict in [(truth,'time_s',True),(control,'state_tick_s',False)]:
        ts=[f(r,key) for r in rows]
        if not all(math.isfinite(t) for t in ts) or any(b<a or (strict and b==a) for a,b in zip(ts,ts[1:])):
            raise ValueError('invalid time order or coverage')
    fields=['phase2_terrain_nonfoot_contact_count']
    fields += [l+s for l in physical.LEGS for s in ['_terrain_top_grf_world_z_N','_terrain_nontop_contact_force_N']]
    if not all(math.isfinite(f(r,k)) for r in truth for k in fields): raise ValueError('missing contact evidence')
    contact=lambda r: f(r,'phase2_terrain_nonfoot_contact_count')>0 or any(abs(f(r,l+'_terrain_top_grf_world_z_N'))>1e-6 or f(r,l+'_terrain_nontop_contact_force_N')>1e-6 for l in physical.LEGS)
    touching=[r for r in truth if contact(r)]
    first=f(touching[0],'time_s') if touching else None
    result={'schema':'b1-approach-contact-diagnostic-v1','acceptance_claim':False,'first_step_contact_s':first}
    if first is not None:
        pre=[r for r in control if first-.8<=f(r,'state_tick_s')<first]
        pre_truth=[r for r in truth if first-.8<=f(r,'time_s')<first]
        v=[f(r,'base_qvel_world_x_mps') for r in pre_truth]
        result['pre_contact_0_8s']={'controller_rows':len(pre),'truth_rows':len(pre_truth),'speed_range_fraction':sum(.75<=x<=1.25 for x in v)/len(v) if v else None,'speed_p05':physical.quantile(v,.05),'speed_median':physical.quantile(v,.5)}
        for k in ['velocity_command_gait_period_s','velocity_command_gait_duty','velocity_command_requested_mps','kernel_effective_foot_lift_m']:
            result['pre_contact_0_8s'][k]=[physical.quantile([f(r,k) for r in pre],q) for q in [0,1]]
        result['first_riser_contact']={}
        for l in physical.LEGS:
            r=next((r for r in truth if f(r,l+'_terrain_nontop_contact_force_N')>1e-6),None)
            result['first_riser_contact'][l]=None if r is None else {k:f(r,k) for k in ['time_s','base_pos_world_x_m','base_pos_world_z_m',l+'_pos_world_x_m',l+'_pos_world_z_m',l+'_terrain_nontop_contact_force_N']}
    result['input_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    result['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    return physical.safe(result)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=run(a.run_dir)
    with a.out.open('x') as h: json.dump(result,h,indent=2,allow_nan=False)
    print(json.dumps(result))
