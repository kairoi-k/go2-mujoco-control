#!/usr/bin/env python3
"""Independent Python MuJoCo replay of saved torque commands and actual state."""
import argparse,csv,fcntl,hashlib,json,pathlib
import numpy as np,mujoco
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--out',required=True);a=p.parse_args();run=pathlib.Path(a.run)
manifest=json.loads((run/'manifest.json').read_text());cmd=manifest['command'];snapshot=pathlib.Path(cmd[1]);scene=pathlib.Path(cmd[3])
for path in [snapshot,scene]:
 if hashlib.sha256(path.read_bytes()).hexdigest()!=manifest['hashes'][str(path)]:raise SystemExit('input hash mismatch')
t=snapshot.read_text().split();offset=9
p0=np.array(t[offset:offset+3],float);offset+=3;q0=np.array(t[offset:offset+4],float);offset+=4
v0=np.array(t[offset:offset+3],float);offset+=3;w0=np.array(t[offset:offset+3],float);offset+=3
joint=np.array(t[offset:offset+12],float);offset+=12;vel=np.array(t[offset:offset+12],float)
with (run/'feedback.csv').open() as f:rows=list(csv.DictReader(l for l in f if not l.startswith('#')))
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m)
 d.qpos[:3]=p0;d.qpos[3:7]=q0/np.linalg.norm(q0);d.qvel[:3]=v0;d.qvel[3:6]=w0
 ids=[m.actuator_trnid[j,0] for j in range(12)];qa=[m.jnt_qposadr[j] for j in ids];va=[m.jnt_dofadr[j] for j in ids]
 d.qpos[qa]=joint;d.qvel[va]=vel;mujoco.mj_forward(m,d)
 maximum_state=maximum_torque=maximum_time=0.;steps=0
 for row in rows:
  observed=np.r_[d.qpos[:7],d.qpos[qa],d.qvel[va]]
  recorded=np.array([float(row[k]) for k in ['base_px','base_py','base_pz','base_qw','base_qx','base_qy','base_qz']+[f'q{j}' for j in range(12)]+[f'dq{j}' for j in range(12)]])
  maximum_state=max(maximum_state,float(np.max(np.abs(observed-recorded))))
  maximum_time=max(maximum_time,abs(d.time-float(row['plant_time_s'])))
  if row['status']!='applied':break
  tau=np.array([float(row[f'requested_tau{j}']) for j in range(12)])
  d.ctrl[:]=tau;mujoco.mj_step(m,d)
  maximum_torque=max(maximum_torque,float(np.max(np.abs(d.qfrc_actuator[va]-tau))))
  mujoco.mj_forward(m,d);steps+=1
report={'schema':'independent-plant-command-replay-v1','runtime_sha':manifest['runtime_sha'],'mujoco_version':mujoco.__version__,'steps':steps,'max_recorded_state_residual':maximum_state,'max_actual_actuator_torque_residual_nm':maximum_torque,'max_plant_clock_residual_s':maximum_time,'b1_claim':False,'scope':'saved torque commands reproduce nonlinear plant; not WBC or planner optimality','script_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()}
pathlib.Path(a.out).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if maximum_state>1e-8 or maximum_torque>1e-9 or maximum_time>1e-12:raise SystemExit(2)
