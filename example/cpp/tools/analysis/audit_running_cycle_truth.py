#!/usr/bin/env python3
"""Independent descriptive cycle audit using raw total and per-foot forces."""
import argparse,bisect,csv,hashlib,json,math
from pathlib import Path
import analyze_b1_physical as physical

def audit(path,start,end):
    files=[path/'data.csv',path/'contact_ground_truth.csv']
    with files[0].open() as h: c=list(csv.DictReader(h))
    with files[1].open() as h: t=list(csv.DictReader(h))
    f=physical.f
    if not c or not t or not math.isfinite(start+end) or end<=start: raise ValueError('invalid interval/input')
    bounds=[];last=None
    for r in c:
        value=f(r,'cycle_index')
        if not math.isfinite(value) or value!=int(value): raise ValueError('invalid cycle index')
        idx=int(value)
        if f(r,'motion_stage')==2 and idx!=last:
            bounds.append((idx,f(r,'state_tick_s'),f(r,'telemetry_gait_time_s')));last=idx
    times=[f(r,'time_s') for r in t]
    if any(not math.isfinite(x) for x in times) or any(not 0<b-a<=.010000001 for a,b in zip(times,times[1:])): raise ValueError('truth time coverage')
    mask=lambda r:sum(1<<i for i,l in enumerate(physical.LEGS) if physical.norm(r,l+'_foot_contact_grf_world_')>=10)
    results=[]
    for x,y in zip(bounds,bounds[1:]):
        if x[2]<start or y[2]>end: continue
        if y[0]!=x[0]+1:raise ValueError('cycle gap')
        w=t[bisect.bisect_left(times,x[1]):bisect.bisect_left(times,y[1])]
        if not w or f(w[0],'time_s')-x[1]>.01 or y[1]-f(w[-1],'time_s')>.01:raise ValueError('cycle coverage')
        if any(not math.isfinite(physical.norm(r,prefix)) for r in w for prefix in ['total_contact_grf_world_']+[l+'_foot_contact_grf_world_' for l in physical.LEGS]):raise ValueError('force coverage')
        diagonal=[physical.longest(w,lambda r,m=m:mask(r)==m) for m in [9,6]]
        aerial=physical.longest(w,lambda r:physical.norm(r,'total_contact_grf_world_')<10)
        results.append({'cycle':x[0],'time_s':[x[1],y[1]],'diagonal_s':diagonal,'aerial_s':aerial,'good':min(diagonal)>=.01-1e-9 and aerial>=.004-1e-9})
    return {'schema':'running-cycle-truth-diagnostic-v1','acceptance_claim':False,'gait_interval_s':[start,end],'complete_cycles':len(results),'good_cycles':sum(x['good'] for x in results),'five_cycle_min_good':min((sum(x['good'] for x in results[i:i+5]) for i in range(len(results)-4)),default=None),'aerial_p50_s':physical.quantile([x['aerial_s'] for x in results],.5),'cycles':results,'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--start',type=float,required=True);p.add_argument('--end',type=float,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    r=audit(a.run_dir,a.start,a.end)
    with a.out.open('x') as h:json.dump(r,h,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in r.items() if k not in ['cycles','input_sha256']}))
