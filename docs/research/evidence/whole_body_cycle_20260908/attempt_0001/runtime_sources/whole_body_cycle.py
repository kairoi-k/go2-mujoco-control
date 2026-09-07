"""Full-model torque shooting challenger. Explicit privileged offline diagnostic.
No controller integration or traversal claim. All physical constraints are
independently re-evaluated; least-squares penalties never constitute feasibility.
"""
import argparse,copy,fcntl,hashlib,json,pathlib,subprocess,time,xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import least_squares
from whole_body_shooting_derivatives import rollout
LEGS=('FR','FL','RR','RL')
def dependencies(scene):
 # Bind recursive XML and every referenced file asset, including meshes.
 found=set()
 def visit(path,assetroot=None):
  path=path.resolve()
  if path in found:return
  found.add(path);tree=ET.parse(path);root=tree.getroot();comp=root.find('compiler')
  meshdir=path.parent/(comp.get('meshdir','') if comp is not None else '')
  texturedir=path.parent/(comp.get('texturedir','') if comp is not None else '')
  for node in root.iter():
   f=node.get('file')
   if not f:continue
   if node.tag=='include':visit(path.parent/f)
   else:
    candidate=(meshdir if node.tag=='mesh' else texturedir if node.tag=='texture' else path.parent)/f
    if not candidate.is_file():raise ValueError('missing model dependency '+str(candidate))
    found.add(candidate.resolve())
 visit(pathlib.Path(scene));return found
def forces(m,d,gids):
 out=np.zeros(4)
 for i in range(d.ncon):
  c=d.contact[i];f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
  for l,g in enumerate(gids):
   if g in (c.geom1,c.geom2):out[l]+=f[0]
 return out
def validate_seed(seed,source,t0,period):
 if seed.get('schema')!='previous-cycle-seed-v1' or abs(seed['source_time']-t0)>1e-10 or abs(seed['period_s']-period)>1e-10:raise ValueError('seed source/calendar mismatch')
 source_hash=hashlib.sha256(pathlib.Path(source).read_bytes()).hexdigest()
 if source_hash not in seed['input_hashes'].values():raise ValueError('seed source hash mismatch')
 for key,arrays,widths in [('times',['joint_q','joint_v'],[12,12]),('gt_times',['base_state','torque'],[13,12])]:
  ts=np.asarray(seed[key],float)
  if ts.ndim!=1 or len(ts)<2 or not np.all(np.isfinite(ts)) or np.any(np.diff(ts)<=0) or ts[0]>t0-period+1e-10 or ts[-1]<t0-1e-10:raise ValueError('seed time coverage invalid '+key)
  for name,width in zip(arrays,widths):
   values=np.asarray(seed[name],float)
   if values.shape!=(len(ts),width) or not np.all(np.isfinite(values)):raise ValueError('invalid seed '+name)
class CycleProblem:
 def __init__(self,source,seed,scene,cycles=1,block_steps=2,force_scale=10.):
  self.m=mujoco.MjModel.from_xml_path(str(scene));m=self.m
  if m.na or m.nu!=12 or m.nv!=18 or abs(m.opt.timestep-.002)>1e-12:raise ValueError('unsupported model')
  t=pathlib.Path(source).read_text().split()
  if t[0]!='joint-shadow-snapshot-v2':raise ValueError('source schema')
  self.t0=float(t[3]);self.phase=float(t[4]);self.period=float(t[5]);self.duty=float(t[6]);self.vcmd=float(t[7]);s=np.array(t[9:46],float)
  if not np.all(np.isfinite(s)):raise ValueError('nonfinite source')
  self.qa=m.jnt_qposadr[m.actuator_trnid[:,0]];self.va=m.jnt_dofadr[m.actuator_trnid[:,0]];self.gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in LEGS]
  self.initial=mujoco.MjData(m);d=self.initial;d.time=self.t0;d.qpos[:3]=s[:3];d.qpos[3:7]=s[3:7]/np.linalg.norm(s[3:7]);d.qvel[:3]=s[7:10];d.qvel[3:6]=s[10:13];d.qpos[self.qa]=s[13:25];d.qvel[self.va]=s[25:37]
  # Source only records q/dq: omitted integration memory starts at model defaults.
  self.state_spec=int(mujoco.mjtState.mjSTATE_INTEGRATION);self.initial_state=np.empty(mujoco.mj_stateSize(m,self.state_spec));mujoco.mj_getState(m,d,self.initial_state,self.state_spec)
  self.steps=int(round(cycles*self.period/m.opt.timestep));self.bs=block_steps
  if self.steps%block_steps:raise ValueError('horizon does not divide control blocks')
  self.blocks=self.steps//block_steps;self.force_scale=force_scale
  old=json.loads(pathlib.Path(seed).read_text());validate_seed(old,source,self.t0,self.period);times=np.array(old['times']);gt=np.array(old['gt_times']);qs=np.array(old['joint_q']);vs=np.array(old['joint_v']);base=np.array(old['base_state']);taus=np.array(old['torque'])
  def interp(t,ts,a):return np.array([np.interp(t,ts,a[:,i]) for i in range(a.shape[1])])
  qstart=interp(self.t0-self.period,times,qs);vstart=interp(self.t0-self.period,times,vs);zstart=interp(self.t0-self.period,gt,base)[2]
  self.refs=[];self.feet=[];self.us=[]
  for k in range(self.steps):
   elapsed=(k+1)*m.opt.timestep;phase_elapsed=elapsed%self.period
   if abs(phase_elapsed)<1e-10:phase_elapsed=self.period
   source_t=self.t0-self.period+phase_elapsed;frac=phase_elapsed/self.period
   ref=copy.copy(d);ref.time=self.t0+elapsed;ref.qpos[:3]=d.qpos[:3]+[self.vcmd*elapsed,0,0];ref.qpos[2]+=interp(source_t,gt,base)[2]-zstart
   ref.qpos[self.qa]=interp(source_t,times,qs)+(d.qpos[self.qa]-qstart)*(1-frac)
   ref.qvel[self.va]=interp(source_t,times,vs)+(d.qvel[self.va]-vstart)*(1-frac)
   ref.qvel[:3]=[self.vcmd,0,interp(source_t,gt,base)[9]];ref.qvel[3:6]=0
   if (k+1)%round(self.period/m.opt.timestep)==0:
    ref.qpos[:]=d.qpos;ref.qpos[0]+=self.vcmd*elapsed;ref.qvel[:]=d.qvel
   mujoco.mj_forward(m,ref);self.refs.append(ref);self.feet.append(ref.geom_xpos[self.gids].copy());self.us.append(interp(source_t-m.opt.timestep,gt,taus))
  self.u0=np.clip(np.array(self.us).reshape(self.blocks,self.bs,12).mean(axis=1),-34.99,34.99)
  self.qscale=np.r_[[.04,.025,.025],[.08]*3,[.35]*12];self.vscale=np.r_[[.3,.3,.4],[.6]*3,[5.]*12]
  self.cache_x=None;self.cache_r=None;self.cache_j=None;self.calls=0;self.jcalls=0;self.history=[];self.best=(float('inf'),self.u0.ravel().copy());self.started=time.perf_counter()
 def local(self,d,k):
  m=self.m;delta=np.zeros(m.nv);mujoco.mj_differentiatePos(m,delta,1,self.refs[k].qpos,d.qpos)
  r=[delta/self.qscale,(d.qvel-self.refs[k].qvel)/self.vscale,(d.geom_xpos[self.gids]-self.feet[k]).ravel()/.025,np.maximum(forces(m,d,self.gids)-180,0)/self.force_scale,np.maximum(abs(d.qvel[self.va])-30,0)/.5]
  ids=m.actuator_trnid[:,0];q=d.qpos[self.qa];r.extend([np.maximum(m.jnt_range[ids,0]-q,0)/.02,np.maximum(q-m.jnt_range[ids,1],0)/.02])
  if k==self.steps-1:r.extend([delta/np.r_[[.01]*3,[.025]*3,[.075]*12],(d.qvel-self.refs[k].qvel)/np.r_[[.075]*3,[.15]*3,[1.5]*12]])
  return np.concatenate(r)
 def evaluate(self,x,jac=False):
  if self.cache_x is not None and np.array_equal(x,self.cache_x) and (not jac or self.cache_j is not None):return self.cache_j if jac else self.cache_r
  u=x.reshape(self.blocks,12);ds,S=rollout(self.m,self.initial,u,self.bs,jac);res=[];jacs=[];m=self.m;eps=1e-6
  for k,d in enumerate(ds):
   r=self.local(d,k);res.append(r)
   if jac:
    # Differentiate all observations incl compliant forces in the same full model.
    D=np.empty((len(r),2*m.nv));U=np.zeros((len(r),m.nu))
    for col in range(2*m.nv):
     plus=copy.copy(d);minus=copy.copy(d)
     if col<m.nv:
      v=np.zeros(m.nv);v[col]=1;mujoco.mj_integratePos(m,plus.qpos,v,eps);mujoco.mj_integratePos(m,minus.qpos,v,-eps)
     else:plus.qvel[col-m.nv]+=eps;minus.qvel[col-m.nv]-=eps
     mujoco.mj_forward(m,plus);mujoco.mj_forward(m,minus);D[:,col]=(self.local(plus,k)-self.local(minus,k))/(2*eps)
    if np.any(forces(m,d,self.gids)>180):
     for col in range(m.nu):
      plus=copy.copy(d);minus=copy.copy(d);plus.ctrl[col]+=eps;minus.ctrl[col]-=eps;mujoco.mj_forward(m,plus);mujoco.mj_forward(m,minus);U[:,col]=(self.local(plus,k)-self.local(minus,k))/(2*eps)
    J=D@S[k];J[:,(k//self.bs)*12:(k//self.bs+1)*12]+=U;jacs.append(J)
  # Regularization is an objective, actuator bounds remain explicit solver bounds.
  res.extend([(np.diff(u,axis=0)/40).ravel(),(u/350).ravel()]);r=np.concatenate(res)
  if jac:
   R=np.zeros(((self.blocks-1)*12,self.blocks*12))
   for i in range(self.blocks-1):R[12*i:12*(i+1),12*i:12*(i+1)]=-np.eye(12)/40;R[12*i:12*(i+1),12*(i+1):12*(i+2)]=np.eye(12)/40
   jacs.extend([R,np.eye(self.blocks*12)/350]);J=np.vstack(jacs);self.jcalls+=1
  else:J=None;self.calls+=1
  cost=float(r@r)
  if np.all(abs(x)<=35) and cost<self.best[0]:self.best=(cost,x.copy());self.history.append({'elapsed_s':time.perf_counter()-self.started,'cost':cost})
  self.cache_x=x.copy();self.cache_r=r;self.cache_j=J
  return J if jac else r
def main():
 p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--seed',required=True);p.add_argument('--scene',required=True);p.add_argument('--out',required=True);p.add_argument('--warm-start');p.add_argument('--cycles',type=int,default=1);p.add_argument('--block-steps',type=int,default=2);p.add_argument('--max-nfev',type=int,default=35);p.add_argument('--force-scale',type=float,default=10);p.add_argument('--jac',choices=['chain','finite'],default='chain');p.add_argument('--check-jac',action='store_true');p.add_argument('--wall-budget-s',type=float,default=240);a=p.parse_args()
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);pb=CycleProblem(a.source,a.seed,a.scene,a.cycles,a.block_steps,a.force_scale);x=pb.u0.ravel()
  if a.warm_start:
   w=json.loads(pathlib.Path(a.warm_start).read_text());ctrl=np.array(w.get('controls',[r['torque'] for r in w.get('rows',[])]));n=min(len(ctrl)//pb.bs,pb.blocks);x[:n*12]=ctrl[:n*pb.bs].reshape(n,pb.bs,12).mean(axis=1).ravel();x=np.clip(x,-34.999999,34.999999)
  started=time.perf_counter();checks=[]
  if a.check_jac:
   J=pb.evaluate(x,True);rng=np.random.default_rng(1301);v=rng.normal(size=x.size);v/=np.linalg.norm(v)
   for eps in [1e-4,1e-5]:
    fd=(pb.evaluate(x+eps*v)-pb.evaluate(x-eps*v))/(2*eps);err=np.linalg.norm(fd-J@v)/max(np.linalg.norm(fd),1e-12);checks.append({'epsilon':eps,'relative_error':float(err),'max_abs':float(np.max(abs(fd-J@v)))})
   print(json.dumps({'derivative_check':checks}),flush=True)
  effective_jac=a.jac
  if checks and max(c['relative_error'] for c in checks)>.03:effective_jac='finite'
  class WallBudget(Exception):pass
  def objective(z,jac=False):
   if time.perf_counter()-started>a.wall_budget_s:raise WallBudget()
   return pb.evaluate(z,jac)
  try:
   fit=least_squares(objective,x,jac=(lambda z:objective(z,True)) if effective_jac=='chain' else '2-point',bounds=(-35,35),max_nfev=a.max_nfev,ftol=1e-6,xtol=1e-6,gtol=1e-5,tr_solver='lsmr',verbose=1)
   fit_success=bool(fit.success);fit_message=fit.message;fit_nfev=fit.nfev
  except WallBudget:
   fit_success=False;fit_message='wall_budget_exhausted';fit_nfev=pb.calls

  elapsed=time.perf_counter()-started;u=pb.best[1].reshape(pb.blocks,12);ds,_=rollout(pb.m,pb.initial,u,pb.bs,False)
  files=dependencies(a.scene)|{pathlib.Path(f).resolve() for f in [a.source,a.seed,__file__,pathlib.Path(__file__).with_name('whole_body_shooting_derivatives.py')]}
  if a.warm_start:files.add(pathlib.Path(a.warm_start).resolve())
  report={'schema':'whole-body-shooting-v1','scope':'privileged initialized full-model offline trajectory; not live controller or B1','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'scene':str(pathlib.Path(a.scene).resolve()),'mujoco_version':mujoco.__version__,'input_hashes':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)},'integration_state_spec':pb.state_spec,'initial_integration_state':pb.initial_state.tolist(),'initialization':'source q/dq unchanged except unit quaternion normalization; omitted integration memory uses model defaults, not original full simulator state','times_are_absolute':True,'timestep_s':float(pb.m.opt.timestep),'period_s':pb.period,'initial_phase':pb.phase,'duty':pb.duty,'leg_offsets':[0,.46,.46,0],'command_vx':pb.vcmd,'constraints':{'torque_limit_nm':35,'joint_speed_limit_radps':30,'normal_force_limit_n':180,'roll_pitch_limit_rad':float(np.pi/12),'min_base_height_m':.28},'terminal_reference':{'qpos':pb.refs[-1].qpos.tolist(),'qvel':pb.refs[-1].qvel.tolist()},'controls':np.repeat(u,pb.bs,axis=0).tolist(),'rows':[{'time':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'normal_forces':forces(pb.m,d,pb.gids).tolist(),'torque':d.qfrc_actuator[pb.va].tolist()} for d in ds],'solver':{'method':'scipy least_squares bounded full MuJoCo shooting','jacobian':effective_jac,'requested_jacobian':a.jac,'wall_budget_s':a.wall_budget_s,'block_steps':pb.bs,'force_penalty_scale':a.force_scale,'success':fit_success,'message':fit_message,'nfev':fit_nfev,'calls':pb.calls,'jac_calls':pb.jcalls,'elapsed_s':elapsed,'cost':pb.best[0],'history':pb.history,'derivative_checks':checks}}
  with open(a.out,'x') as f:json.dump(report,f,indent=2);f.write('\n')
  print(json.dumps({'out':a.out,'solver':{k:v for k,v in report['solver'].items() if k!='history'},'peak_force':max(max(r['normal_forces']) for r in report['rows']),'end_omega':report['rows'][-1]['qvel'][3:6]},indent=2))
if __name__=='__main__':main()
