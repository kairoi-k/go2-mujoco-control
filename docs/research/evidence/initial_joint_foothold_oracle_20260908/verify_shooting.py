#!/usr/bin/env python3
"""Independent replay of saved nonlinear shooting controls and full state."""
import argparse,json,pathlib,hashlib,fcntl
import numpy as np,mujoco
p=argparse.ArgumentParser();p.add_argument('result');a=p.parse_args();r=json.loads(pathlib.Path(a.result).read_text());files=r['hashes']
source=next(pathlib.Path(p) for p in files if p.endswith('initial_source.txt'));scene=next(pathlib.Path(p) for p in files if p.endswith('phase2_flat.xml'))
for f in (source,scene):assert hashlib.sha256(f.read_bytes()).hexdigest()==files[str(f)]
t=source.read_text().split();o=9;initial=np.array(t[o:o+49],float)
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m);j=m.actuator_trnid[:,0];qa=m.jnt_qposadr[j];va=m.jnt_dofadr[j]
 d.qpos[:3]=initial[:3];d.qpos[3:7]=initial[3:7]/np.linalg.norm(initial[3:7]);d.qvel[:3]=initial[7:10];d.qvel[3:6]=initial[10:13];d.qpos[qa]=initial[13:25];d.qvel[va]=initial[25:37]
 maximum=0.;clock_error=0.;torque_error=0.
 for row in r['rows']:
  d.ctrl[:]=row['torque'];mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
  maximum=max(maximum,float(np.max(abs(d.qpos-row['qpos']))),float(np.max(abs(d.qvel-row['qvel']))))
  clock_error=max(clock_error,abs(d.time-row['t']));torque_error=max(torque_error,float(np.max(abs(d.qfrc_actuator[va]-row['torque']))))
assert maximum<1e-9 and clock_error<1e-12 and torque_error<1e-9
print(json.dumps({'state_residual':maximum,'clock_residual':clock_error,'torque_residual':torque_error,'steps':len(r['rows']),'result_sha256':hashlib.sha256(pathlib.Path(a.result).read_bytes()).hexdigest(),'max_recorded_normal_force_N':max(max(x['normal_forces']) for x in r['rows']),'scope':'full-state saved-control reproduction; not acceptance'},indent=2))
