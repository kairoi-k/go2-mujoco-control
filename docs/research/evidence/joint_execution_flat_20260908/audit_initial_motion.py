#!/usr/bin/env python3
"""Independent actual-source kinematics; no integration or state projection."""
import argparse, json, hashlib
from pathlib import Path
import numpy as np, mujoco
p=argparse.ArgumentParser();p.add_argument('log',type=Path);p.add_argument('scene',type=Path);p.add_argument('--out',required=True,type=Path);a=p.parse_args()
if a.out.exists():raise SystemExit('refusing overwrite')
line=next(l for l in a.log.read_text().splitlines() if l.startswith('JointSnapshot '));s=json.loads(line[len('JointSnapshot '):])
m=mujoco.MjModel.from_xml_path(str(a.scene));d=mujoco.MjData(m)
legs=['FR','FL','RR','RL'];expected=[f'{leg}_{joint}' for leg in legs for joint in ['hip','thigh','calf']]
assert [m.actuator(i).name for i in range(m.nu)]==expected
assert m.nq==19 and m.nv==18
q=np.array(s['quaternion_wxyz']);d.qpos[:3]=s['position'];d.qpos[3:7]=q/np.linalg.norm(q)
d.qvel[:3]=s['linear_velocity_world'];d.qvel[3:6]=s['angular_velocity_body']
for i in range(12):
 j=int(m.actuator_trnid[i,0]);d.qpos[m.jnt_qposadr[j]]=s['q'][i];d.qvel[m.jnt_dofadr[j]]=s['dq'][i]
original_qpos=d.qpos.copy();original_qvel=d.qvel.copy();mujoco.mj_forward(m,d)
results=[]
for i,leg in enumerate(legs):
 J=np.zeros((3,m.nv));R=J.copy();geom=m.geom(leg).id
 assert m.geom_type[geom]==mujoco.mjtGeom.mjGEOM_SPHERE
 mujoco.mj_jacGeom(m,d,J,R,geom);v=J@d.qvel
 results.append(dict(leg=leg,measured_contact=s['measured_contact'][i],center_velocity_world=v.tolist(),speed_mps=float(np.linalg.norm(v))))
assert np.array_equal(original_qpos,d.qpos) and np.array_equal(original_qvel,d.qvel)
out=dict(b1_claim=False,source_time=s['time'],mujoco_version=mujoco.__version__,results=results,
 source_log_sha256=hashlib.sha256(a.log.read_bytes()).hexdigest(),scene_sha256=hashlib.sha256(a.scene.read_bytes()).hexdigest(),
 snapshot={k:s[k] for k in ['time','position','quaternion_wxyz','linear_velocity_world','angular_velocity_body','q','dq','measured_contact']},
 model_xml_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(a.scene.parent.glob('*.xml'))},scope='actual source model velocity; no mj_step; measured contacts do not imply stationary collision centers')
a.out.write_text(json.dumps(out,indent=2)+'\n');print('saved',a.out)
