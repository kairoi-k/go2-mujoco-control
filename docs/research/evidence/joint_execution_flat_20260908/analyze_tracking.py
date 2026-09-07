#!/usr/bin/env python3
"""Executor-task evidence only; no physical feasibility or causal verdict."""
import argparse, hashlib, json, re
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('tracking',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
if a.out.exists(): raise SystemExit('refusing to overwrite output')
rows=[]
for line in a.tracking.read_text().splitlines():
 if not line.startswith('JointExecutionTracking '): continue
 d=dict(re.findall(r'(\w+)=([^\s]+)',line))
 v=lambda k: np.fromstring(d[k],sep=',')
 vectors={k:v(k) for k in ('ref_p','ref_v','ref_a','actual_p','actual_v','task_a','solved_a','solved_force')}
 if any(x.shape!=(3,) or not np.isfinite(x).all() for x in vectors.values()): raise ValueError('invalid vector')
 rows.append(dict(time=float(d['state']),version=int(d['version']),leg=int(d['leg']),
   planned=int(d['planned_contact']),measured=int(d['measured_contact']),
   position_error=float(np.linalg.norm(vectors['ref_p']-vectors['actual_p'])),
   velocity_error=float(np.linalg.norm(vectors['ref_v']-vectors['actual_v'])),
   task_acceleration_error=float(np.linalg.norm(vectors['task_a']-vectors['solved_a'])),
   **{k:x.tolist() for k,x in vectors.items()}))
if not rows: raise ValueError('no executor tracking rows')
first=rows[0]['time'];initial=[r for r in rows if r['time']==first]
stance=[r for r in rows if r['planned']]
out=dict(kind='sampled_executor_tracking_audit',b1_claim=False,source_sha256=hashlib.sha256(a.tracking.read_bytes()).hexdigest(),
 sample_leg_rows=len(rows),first_sample=initial,
 first_sensor_schedule_mismatch=next((r for r in rows if r['planned']!=r['measured']),None),
 max_stance_reference_acceleration=max(float(np.linalg.norm(r['ref_a'])) for r in stance),
 note='Task residuals quantify optimization compromise. Sensor masks are not collision truth. Sampling cannot establish the exact first physical divergence.')
a.out.write_text(json.dumps(out,indent=2)+'\n')
print('saved',a.out)
