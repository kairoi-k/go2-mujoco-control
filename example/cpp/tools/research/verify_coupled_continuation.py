"""Independent continuous saved-torque replay; does not import producer checks."""
import argparse,copy,fcntl,hashlib,json,pathlib
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
def main():
 p=argparse.ArgumentParser();p.add_argument('audit');p.add_argument('--head',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 audit=json.loads(pathlib.Path(a.audit).read_text());head=json.loads(pathlib.Path(a.head).read_text())
 sha=lambda f:hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()
 for f,h in audit['input_hashes'].items():
  if sha(f)!=h:raise ValueError('input hash mismatch '+f)
 if sha(a.head) not in audit['input_hashes'].values():raise ValueError('head not bound')
 for f,h in head['hashes'].items():
  if sha(f)!=h:raise ValueError('head source mismatch '+f)
 m=mujoco.MjModel.from_xml_path(head['scene']);gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')];ids=m.actuator_trnid[:,0];qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
 reports={}
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  for name,case in audit['results'].items():
   d=mujoco.MjData(m);mujoco.mj_setState(m,d,np.array(head['initial_integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION);t0=d.time
   residual=force_error=clock_error=0.;peak_force=peak_tau=peak_speed=peak_pose=peak_bad=0.;min_height=1e9;joint_violation=0.;negative=False
   rows=case['rows']
   if len(rows)!=156:raise ValueError('incomplete continuation')
   for k,row in enumerate(rows):
    tau=np.asarray(row['tau']);pre=copy.copy(d);pre.ctrl[:]=tau;mujoco.mj_forward(m,pre);d.ctrl[:]=tau;mujoco.mj_step(m,d);post=copy.copy(d);mujoco.mj_forward(m,post)
    if row['tick']!=k:raise ValueError('phase ordering')
    residual=max(residual,float(np.max(abs(d.qpos-row['qpos']))),float(np.max(abs(d.qvel-row['qvel']))));clock_error=max(clock_error,abs(d.time-row['time']),abs(d.time-t0-(k+1)*.002));samples=[]
    for obs in (pre,post):
     if not np.all(np.isfinite(np.r_[obs.qpos,obs.qvel,obs.qacc])):raise ValueError('nonfinite')
     if any(obs.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL)):raise ValueError('numerical reset')
     forces=np.zeros(4);bad=0.
     for i in range(obs.ncon):
      c=obs.contact[i];cf=np.zeros(6);mujoco.mj_contactForce(m,obs,i,cf);legs=[l for l,g in enumerate(gids) if g==c.geom1 or g==c.geom2]
      for l in legs:forces[l]+=cf[0]
      if len(legs)!=1 or m.body_rootid[m.geom_bodyid[c.geom1]]==m.body_rootid[m.geom_bodyid[c.geom2]]:bad+=float(np.linalg.norm(cf[:3]))
     samples.extend(forces);peak_bad=max(peak_bad,bad);negative|=bool(np.min(forces)<0)
     pose=Rotation.from_quat(obs.qpos[[4,5,6,3]]).as_euler('xyz');peak_pose=max(peak_pose,float(max(abs(pose[:2]))));min_height=min(min_height,float(obs.qpos[2]));peak_speed=max(peak_speed,float(max(abs(obs.qvel[va]))));joint_violation=max(joint_violation,float(max(m.jnt_range[ids,0]-obs.qpos[qa])),float(max(obs.qpos[qa]-m.jnt_range[ids,1])))
    force_error=max(force_error,float(max(abs(np.asarray(samples)-row['forces']))));peak_force=max(peak_force,max(samples));peak_tau=max(peak_tau,float(max(abs(tau))))
   checks={'state_replay':residual<=1e-9,'force_replay':force_error<=1e-8,'clock':clock_error<=1e-10,'force':peak_force<=180 and not negative,'torque':peak_tau<=35,'joint_speed':peak_speed<=30,'joint_range':joint_violation<=0,'height':min_height>=.28,'pose':peak_pose<=np.pi/12,'nonfoot':peak_bad<=1e-6}
   reports[name]={'checks':checks,'passed':bool(all(checks.values())),'state_residual':residual,'force_residual':force_error,'clock_residual':clock_error,'peak_force':peak_force,'peak_torque':peak_tau,'peak_abs_roll_pitch_rad':peak_pose,'min_height':min_height,'terminal_velocity':d.qvel[:6].tolist()}
 result={'scope':'independent312ms replay and sampled hard constraints; no global recovery-region or B1 certificate','cases':reports,'audit_sha256':sha(a.audit),'verifier_sha256':sha(__file__)}
 with open(a.out,'x') as f:json.dump(result,f,indent=2,allow_nan=False)
 print(json.dumps(result,indent=2))
 if not all(r['passed'] for r in reports.values()):raise SystemExit(1)
if __name__=='__main__':main()
