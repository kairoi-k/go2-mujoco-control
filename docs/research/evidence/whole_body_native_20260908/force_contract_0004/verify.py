"""Independent Python fresh-copy physical evaluation of native saved torques."""
import copy,fcntl,hashlib,json,pathlib,time
import mujoco,numpy as np
base=pathlib.Path(__file__).resolve().parent;r=json.loads((base/'result.json').read_text());text=(base/'input.txt').read_text().splitlines();m=mujoco.MjModel.from_xml_path(text[0]);state=np.fromstring(text[2],sep=' ');desired=np.fromstring(text[3],sep=' ');d=mujoco.MjData(m);mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_INTEGRATION);gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
old=json.loads((base.parent/'whole_body_feedback_scratch_benchmark_20260908_0001/report.json').read_text())['results']['failed_predecessor']['pairs'][0]['new'];checks=[]
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 for run in r['runs']:
  tau=np.array(run['control']);pre=copy.copy(d);pre.ctrl[:]=tau;post=copy.copy(pre);mujoco.mj_forward(m,pre);mujoco.mj_step(m,post);mujoco.mj_forward(m,post);values=[]
  for data in [pre,post]:
   f=np.zeros(4)
   for i,c in enumerate(data.contact):
    force=np.empty(6);mujoco.mj_contactForce(m,data,i,force)
    for leg,gid in enumerate(gids):
     if gid in (c.geom1,c.geom2):f[leg]+=force[0]
   values.extend(f.tolist())
  checks.append({'force_reproduction_max_n':float(np.max(abs(np.array(values)-run['forces']))),'physical_force_max_n':max(values),'torque_max_nm':float(max(abs(tau))),'strict_feasible':bool(max(values)<=180 and max(abs(tau))<=35),'python_solution_control_max_delta_nm':float(max(abs(tau-np.array(old['control'])))),'native_cost':float(.5*np.sum((tau-desired)**2)),'python_cost':float(.5*np.sum((np.array(old['control'])-desired)**2))})
report={'checks':checks,'median_ms':float(np.median([x['elapsed_ms'] for x in r['runs']])),'inputs':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [base/'input.txt',base/'result.json',pathlib.Path(__file__)]},'scope':'one-state native diagnostic; no horizon safety or realtime acceptance'}
with (base/'verification.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
