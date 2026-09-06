#!/usr/bin/env python3
"""Independent momentum/force integral diagnostic; no acceptance threshold.

Uses measured subtree velocity and summed external contact forces, not a
logged qacc/residual. Left-end quadrature follows explicit force-to-velocity
step semantics. Derived MuJoCo quantities logged after mj_step retain the
pre-integration kinematics (one physics step); no claim of exact pose/force
synchronization with controller telemetry is made here.
"""
import argparse, csv, hashlib, json, math
from pathlib import Path
import numpy as np

def audit(rows, start, end, windows=(1,10,50)):
    if len(rows)<2: raise ValueError("insufficient coverage")
    def col(key): return np.asarray([float(r[key]) for r in rows])
    t=col("time_s"); mass=col("total_mass_kg")
    v=np.column_stack([col("subtree_linvel_world_"+a+"_mps") for a in "xyz"])
    g=np.column_stack([col("gravity_world_"+a+"_mps2") for a in "xyz"])
    force=np.column_stack([col("total_contact_grf_world_"+a+"_N") for a in "xyz"])
    if not all(np.isfinite(x).all() for x in (t,mass,v,g,force)):
        raise ValueError("nonfinite input")
    if np.min(mass)<=0 or np.ptp(mass)>1e-9: raise ValueError("invalid/changing mass")
    dt=np.diff(t)
    if np.min(dt)<=0 or np.max(dt)>.01: raise ValueError("time coverage gap")
    if not math.isfinite(start) or not math.isfinite(end) or start>=end or t[0]>start or t[-1]<end:
        raise ValueError("uncovered requested window")
    net=force+mass[:,None]*g
    impulse=np.vstack([np.zeros(3),np.cumsum(net[:-1]*dt[:,None],axis=0)])
    out={"schema":"b1-impulse-diagnostic-v1","claim":"integral consistency diagnostic; no acceptance verdict", "window_s":[start,end],"quadrature":"left-end external force plus gravity", "total_mass_kg":float(mass[0]),"windows":{}}
    for n in windows:
        if n<=0 or n>=len(t): raise ValueError("uncovered integration length")
        mask=(t[:-n]>=start)&(t[n:]<=end)
        if not np.any(mask): raise ValueError("uncovered integration window")
        residual=mass[0]*(v[n:]-v[:-n])-(impulse[n:]-impulse[:-n])
        norms=np.linalg.norm(residual[mask],axis=1)
        out["windows"][str(n)]={"samples":int(mask.sum()),"duration_min_s":float((t[n:]-t[:-n])[mask].min()),"duration_max_s":float((t[n:]-t[:-n])[mask].max()),"residual_norm_Ns_p50_p95_max":np.percentile(norms,[50,95,100]).tolist()}
    return out

def main():
    ap=argparse.ArgumentParser();ap.add_argument("csv",type=Path);ap.add_argument("--start",type=float,required=True);ap.add_argument("--end",type=float,required=True);ap.add_argument("--out",type=Path,required=True);a=ap.parse_args()
    with a.csv.open() as f: result=audit(list(csv.DictReader(f)),a.start,a.end)
    result["input_sha256"]=hashlib.sha256(a.csv.read_bytes()).hexdigest();result["analyzer_sha256"]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with a.out.open("x") as f: json.dump(result,f,indent=2,allow_nan=False);f.write("\n")
    print(json.dumps(result,allow_nan=False))
if __name__=="__main__":main()
