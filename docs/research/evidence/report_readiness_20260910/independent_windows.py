"""Independent NumPy raw-window check; no imports from the settling auditor."""
import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
repo=Path(__file__).resolve().parents[4]
root=Path('/tmp/go2-report-20260910'); report=json.loads((root/'phase1-audit/summary.json').read_text());out=[]
for item in report['results']:
 p=Path(item['audit']['source']['data_csv']); times=[];vel=[];requested=[];shaped=[]
 with p.open() as f:
  for r in csv.DictReader(f):
   if r['velocity_command_gait_regime']=='continuous-trot' and float(r['velocity_command_active'])>.5:
    times.append(float(r['cmd_time_s']));vel.append(float(r['velocity_command_measured_mps']));requested.append(float(r['velocity_command_requested_mps']));shaped.append(float(r['velocity_command_shaped_mps']))
 t=np.array(times);t-=t[0];v=np.array(vel);checks=[]
 for tr in item['audit']['transitions']:
  mask=(t>=tr['time_s'])&(t<=tr['window_end_s']);tt=t[mask];err=np.abs(v[mask]-tr['to_mps']);witness=None
  for i in np.flatnonzero(err<=tr['settling_tolerance_mps']):
   j=np.searchsorted(tt,tt[i]+1.0)
   if j>=len(tt):break
   if np.max(err[i:j+1])<=tr['settling_tolerance_mps'] and np.max(np.diff(tt[i:j+1]))<=.05:
    witness=float(tt[i]-tr['time_s']);break
  observed=witness is not None and witness<=item['audit']['settling_limit_s'] if 'settling_limit_s' in item['audit'] else witness is not None and witness<=tr['settling_deadline_s']-tr['time_s']
  checks.append({'transition':tr['transition'],'observed':bool(observed),'auditor_observed':tr['status']=='observed','settling_s':witness,'min_point_error_mps':float(np.min(err)),'in_tolerance_samples':int(np.sum(err<=tr['settling_tolerance_mps'])),'samples':len(tt)})
 assert all(c['observed']==c['auditor_observed'] for c in checks)
 np.savez_compressed(root/(item['run']+'.npz'),time=t,measured=v,requested=np.array(requested),shaped=np.array(shaped))
 out.append({'run':item['run'],'checks':checks,'raw_sha256':hashlib.sha256(p.read_bytes()).hexdigest()});print(item['run'],'independent agreement',flush=True)
(root/'independent_windows.json').write_text(json.dumps(out,indent=2)+'\n')
