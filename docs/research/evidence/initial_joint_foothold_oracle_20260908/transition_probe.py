#!/usr/bin/env python3
"""20ms zero-order-held oracle torque probe, not trajectory execution."""
import argparse,json,pathlib,hashlib,fcntl,subprocess
import numpy as np,mujoco
p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--torque',required=True);p.add_argument('--scene',required=True);p.add_argument('--out',required=True);a=p.parse_args()
paths=[pathlib.Path(x) for x in (a.source,a.torque,a.scene,__file__)]
t=paths[0].read_text().split();offset=9
p0=np.array(t[offset:offset+3],float);offset+=3;q0=np.array(t[offset:offset+4],float);offset+=4
v0=np.array(t[offset:offset+3],float);offset+=3;w0=np.array(t[offset:offset+4-1],float);offset+=3
joint=np.array(t[offset:offset+12],float);offset+=12;vel=np.array(t[offset:offset+12],float)
tau=np.array(json.loads(paths[1].read_text())['torque']);assert np.max(abs(tau))<=35+1e-9
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 m=mujoco.MjModel.from_xml_path(a.scene);d=mujoco.MjData(m)
 ids=m.actuator_trnid[:,0];qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
 d.qpos[:3]=p0;d.qpos[3:7]=q0/np.linalg.norm(q0);d.qvel[:3]=v0;d.qvel[3:6]=w0;d.qpos[qa]=joint;d.qvel[va]=vel;d.ctrl[:]=tau
 gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
 rows=[]
 for i in range(11):
  mujoco.mj_forward(m,d);forces=np.zeros(4)
  for j in range(d.ncon):
   ct=d.contact[j];f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f)
   for leg,gid in enumerate(gids):
    if gid in (ct.geom1,ct.geom2):forces[leg]+=f[0]
  speeds=[]
  for gid in gids:
   J=np.zeros((3,18));R=np.zeros((3,18));mujoco.mj_jacGeom(m,d,J,R,gid);speeds.append((J@d.qvel).tolist())
  rows.append({'time':float(d.time),'normal_forces':forces.tolist(),'foot_velocity':speeds,'body_angular_velocity':d.qvel[3:6].tolist(),'body_angular_acceleration':d.qacc[3:6].tolist(),'actuator_torque_error':float(np.max(abs(d.qfrc_actuator[va]-tau)))})
  if i<10:mujoco.mj_step(m,d)
report={'scope':'20ms fixed torque from unchanged actual state, not a planned torque trajectory or B1','runtime_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},'timestep':float(m.opt.timestep),'foot_solref':[m.geom_solref[g].tolist() for g in gids],'rows':rows}
with pathlib.Path(a.out).open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({'initial':rows[0],'final':rows[-1]},indent=2))
