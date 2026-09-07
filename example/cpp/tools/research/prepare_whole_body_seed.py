"""Extract previous-cycle initialization from immutable telemetry; never acceptance."""
import argparse,csv,json,pathlib,hashlib
import numpy as np
def main():
 p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--run',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 src=pathlib.Path(a.source);run=pathlib.Path(a.run);t=src.read_text().split();t0=float(t[3]);period=float(t[5]);start=t0-period
 names=[leg+'_'+joint for leg in ['FR','FL','RR','RL'] for joint in ['hip','thigh','calf']]
 with (run/'data.csv').open() as f:
  rows=[r for r in csv.DictReader(f) if start-.004<=float(r['state_tick_s'])<=t0+.000001]
 # Duplicate state snapshots are resolved by first row, not averaged commands.
 unique={}
 for r in rows:unique.setdefault(float(r['state_tick_s']),r)
 times=sorted(unique);qs=[[float(unique[t][n+'_q_state']) for n in names] for t in times];vs=[[float(unique[t][n+'_dq_state']) for n in names] for t in times]
 with (run/'contact_ground_truth.csv').open() as f:
  gt=[r for r in csv.DictReader(f) if start-.004<=float(r['time_s'])<=t0+.000001]
 cols=['base_pos_world_'+s+'_m' for s in 'xyz']+['base_quat_'+s for s in ['w','x','y','z']]+['base_qvel_world_'+s+'_mps' for s in 'xyz']+['base_angvel_body_'+s+'_radps' for s in 'xyz']
 report={'schema':'previous-cycle-seed-v1','scope':'privileged historical initialization, not future controller input or acceptance','source_time':t0,'period_s':period,'times':times,'joint_q':qs,'joint_v':vs,'gt_times':[float(r['time_s']) for r in gt],'base_state':[[float(r[c]) for c in cols] for r in gt],'torque':[[float(r['full_qfrc_actuator_qcoord_'+n+'_joint']) for n in names] for r in gt],'input_hashes':{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [src,run/'data.csv',run/'contact_ground_truth.csv',pathlib.Path(__file__)]}}
 if not times or times[0]>start or times[-1]<t0-1e-6:raise ValueError('incomplete previous-cycle state coverage')
 with open(a.out,'x') as f:json.dump(report,f,indent=2);f.write('\n')
 print(json.dumps({'states':len(times),'gt_rows':len(gt),'start':start,'end':t0,'max_source_torque':float(np.max(np.abs(report['torque'])))}))
if __name__=='__main__':main()
