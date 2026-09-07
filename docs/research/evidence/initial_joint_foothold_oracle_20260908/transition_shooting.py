#!/usr/bin/env python3
"""Bounded nonlinear torque-sequence diagnostic using actual MuJoCo contact."""
import argparse,json,pathlib,time,hashlib,fcntl,subprocess
import numpy as np,mujoco
from scipy.optimize import least_squares
p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--seed',required=True);p.add_argument('--scene',required=True);p.add_argument('--out',required=True);a=p.parse_args()
paths=[pathlib.Path(x) for x in (a.source,a.seed,a.scene,__file__)]
t=paths[0].read_text().split();o=9
pos=np.array(t[o:o+3],float);o+=3;quat=np.array(t[o:o+4],float);o+=4
vel=np.array(t[o:o+3],float);o+=3;omega=np.array(t[o:o+3],float);o+=3;q=np.array(t[o:o+12],float);o+=12;dq=np.array(t[o:o+12],float)
seed=json.loads(paths[1].read_text());target=np.array(seed['targets']);tau=np.array(seed['torque'])
m=mujoco.MjModel.from_xml_path(a.scene);assert abs(m.opt.timestep-.002)<1e-12
ids=m.actuator_trnid[:,0];qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
started=time.perf_counter();evaluations=0;best=None
class TimeBudget(Exception):pass
def simulate(x,record=False):
 global evaluations,best
 if time.perf_counter()-started>45 and not record:raise TimeBudget()
 d=mujoco.MjData(m);d.qpos[:3]=pos;d.qpos[3:7]=quat/np.linalg.norm(quat);d.qvel[:3]=vel;d.qvel[3:6]=omega;d.qpos[qa]=q;d.qvel[va]=dq
 costs=[];rows=[];u=x.reshape(5,12)
 for k in range(10):
  d.ctrl[:]=u[k//2];mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
  forces=np.zeros(4)
  for j in range(d.ncon):
   ct=d.contact[j];f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f)
   for leg,gid in enumerate(gids):
    if gid in (ct.geom1,ct.geom2):forces[leg]+=f[0]
  # Fixed diagnostic weights; no safety or B1 thresholds are changed.
  costs.extend((d.qvel[3:6]-omega)/.15)
  costs.extend((d.qvel[:3]-vel)/.3)
  costs.extend((d.qpos[3:7]-quat)/.025)
  costs.extend(np.maximum(forces-180,0)/30)
  if k>=3:
   for leg in (1,2):costs.extend((d.geom_xpos[gids[leg]]-target[leg])/.015)
  costs.extend(np.maximum(abs(d.qvel[va])-30,0)/3)
  if record:rows.append({'t':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'torque':d.ctrl.tolist(),'normal_forces':forces.tolist()})
 costs.extend((np.diff(u,axis=0)/20).ravel());costs.extend((u/350).ravel())
 r=np.array(costs);evaluations+=1;cost=float(r@r)
 if best is None or cost<best[0]:best=(cost,x.copy())
 return (r,rows) if record else r
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 x0=np.tile(np.clip(tau,-34.999,34.999),5);initial,initial_rows=simulate(x0,True)
 timed_out=False;success=False
 try:
  fit=least_squares(simulate,x0,bounds=(-35,35),max_nfev=30,ftol=1e-5,xtol=1e-5,gtol=1e-5)
  success=bool(fit.success)
 except TimeBudget:timed_out=True
 final,rows=simulate(best[1],True)
report={'scope':'20ms nonlinear multiple-control shooting diagnostic; not MPC runtime or B1','runtime_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},'solver_success':success,'time_budget_hit':timed_out,'evaluations':evaluations,'elapsed_s':time.perf_counter()-started,'initial_cost':float(initial@initial),'final_cost':float(final@final),'initial_final_body_omega':initial_rows[-1]['qvel'][3:6],'final_body_omega':rows[-1]['qvel'][3:6],'rows':rows}
with pathlib.Path(a.out).open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('hashes','rows')},indent=2))
