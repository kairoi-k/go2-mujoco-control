#!/usr/bin/env python3
"""Raw replay audit; deliberately never grants traversal acceptance."""
import argparse,csv,hashlib,json,math,pathlib
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--out',required=True);a=p.parse_args()
run=pathlib.Path(a.run);manifest=json.loads((run/'manifest.json').read_text());result=json.loads((run/'result.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name,digest in result['output_hashes'].items():
 if sha(run/name)!=digest:raise SystemExit('raw hash mismatch: '+name)
rows=[]
if (run/'feedback.csv').exists():
 with (run/'feedback.csv').open() as f:rows=list(csv.DictReader(line for line in f if not line.startswith('#')))
if any(None in row for row in rows):raise SystemExit('CSV width mismatch')
def number(row,key):return float(row[key])
def maximum(key):
 v=[number(r,key) for r in rows if key in r and math.isfinite(number(r,key))];return max(v) if v else None
def first(pred):return next(({'step':r['step'],'time':r['reference_time_s'],'status':r['status']} for r in rows if pred(r)),None)
applied=[r for r in rows if r['status']=='applied']
clock=max((abs(number(r,'reference_time_s')-number(rows[0],'reference_time_s')-number(r,'plant_time_s')) for r in rows),default=None)
torque=max((abs(number(r,f'requested_tau{j}')-number(r,f'applied_tau{j}')) for r in applied for j in range(12)),default=None)
feet=max((math.sqrt(sum(number(r,f'foot_error{l}_{axis}')**2 for axis in 'xyz')) for r in applied for l in range(4)),default=None)
summary={'runtime_sha':manifest['runtime_sha'],'snapshot_runtime_sha':manifest['snapshot_runtime_sha'],'b1_claim':False,'rows':len(rows),'applied_rows':len(applied),'returncode':result['returncode'],'input_files_changed':result['input_files_changed'],'clock_residual_s':clock,'requested_applied_residual_nm':torque,'max_foot_error_m':feet,'first_failure':first(lambda r:r['status'] not in ['applied','terminal']),'first_nonfoot_contact':first(lambda r:float(r['nonfoot_contact_count'])>0),'nominal_geom_disagreement_rows':sum(r['nominal_contact_mask']!=r['mujoco_geom_contact_mask'] for r in applied),'solver_unconverged_rows':sum(r['solver_converged']!='1' for r in applied),'maxima':{k:maximum(k) for k in ['com_error_m','com_velocity_error_mps','momentum_error_nms','max_tau_nm','max_motor_saturation_nm','certificate_force_residual_n','certificate_moment_residual_nm','certificate_joint_residual_nm','gt_contact_couple_norm_nm']},'raw_hashes':result['output_hashes'],'analyzer_sha256':sha(pathlib.Path(__file__))}
latency=sorted(number(r,'solve_us') for r in rows if 'solve_us' in r and math.isfinite(number(r,'solve_us')))
def percentile(q):
 if not latency:return None
 t=(len(latency)-1)*q;i=int(t);return latency[i]+(latency[min(i+1,len(latency)-1)]-latency[i])*(t-i)
summary['qp_latency_us']={'count':len(latency),'p50':percentile(.5),'p95':percentile(.95),'max':max(latency) if latency else None}
pathlib.Path(a.out).write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='raw_hashes'},indent=2))
