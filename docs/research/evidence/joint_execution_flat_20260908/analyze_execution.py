#!/usr/bin/env python3
"""Read-only actual joint adoption audit; no traversal acceptance verdict."""
import argparse,csv,hashlib,json,re
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',required=True,type=Path);a=p.parse_args()
if a.out.exists():raise SystemExit('refusing to overwrite output')
log=(a.run/'controller.log').read_text();lines=log.splitlines()
def fields(line):return dict(re.findall(r'(\w+)=([^\s]+)',line))
executions=[fields(x) for x in lines if x.startswith('JointExecution ')]
applied=[x for x in executions if x.get('applied')=='1']
stops=[x for x in executions if x.get('stop_requested')=='1']
shadows=[fields(x) for x in lines if x.startswith('JointShadow ')]
start=float(applied[0]['state']) if applied else None
end=float(stops[0]['state']) if stops else (float(applied[-1]['state']) if applied else None)
rows=[]
with (a.run/'data.csv').open() as f:
 reader=csv.DictReader(f);keys=reader.fieldnames
 gains=[k for k in keys if k.endswith('_kp') or k.endswith('_kd')]
 for row in reader:
  t=float(row['state_tick_s'])
  if start is not None and start<=t<end:rows.append(row)
q=lambda key: [float(x[key]) for x in applied if key in x]
lat=q('elapsed_us')
out={'runtime_manifest_repository':json.loads((a.run/'run_manifest.json').read_text())['repository'],
 'b1_claim':False,'first_applied_state_s':start,'first_stop':stops[0] if stops else None,
 'sampled_applied_log_rows':len(applied),'logged_applied_count_lower_bound':max([int(x['count']) for x in applied],default=0),
 'accepted_versions':sorted(set(int(x['execution_version']) for x in applied)),
 'first_post_adoption_proposal_failure':next((x for x in shadows if start is not None and float(x.get('state','-1'))>=start and x.get('feasible')=='0'),None),
 'csv_active_window_rows':len(rows),'csv_zero_extra_pd_rows':sum(all(float(r[k])==0 for k in gains) for r in rows),
 'latency_sampled_us':dict(zip(['p50','p95','max'],map(float,np.percentile(lat,[50,95,100])))) if lat else None,
 'sampled_maxima':{k:max(q(k),default=None) for k in ['com_error_m','foot_error_m','force_residual_N','moment_residual_Nm','joint_residual_Nm']},
 'raw_hashes':{n:hashlib.sha256((a.run/n).read_bytes()).hexdigest() for n in ['controller.log','data.csv','contact_ground_truth.csv','run_manifest.json','run_metadata.txt','environment.txt']}}
a.out.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
