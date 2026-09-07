"""Periodic full-body TVLQR replay challenger; privileged model, not live B1."""
import argparse,copy,fcntl,hashlib,json,pathlib,subprocess,time
import numpy as np,mujoco
from whole_body_shooting_derivatives import checked_transition_fd
def forces(model, data, gids):
 out=np.zeros(len(gids))
 for i in range(data.ncon):
  contact=data.contact[i];force=np.zeros(6);mujoco.mj_contactForce(model,data,i,force)
  for leg,gid in enumerate(gids):
   if gid in (contact.geom1,contact.geom2):out[leg]+=force[0]
 return out
def resolve_inputs(result, source):
 """Resolve portable result paths against its directory, never process cwd."""
 def resolve(name):
  path=pathlib.Path(name)
  return (path if path.is_absolute() else source.parent/path).resolve()
 hashes={}
 for name,expected in result['input_hashes'].items():
  path=resolve(name)
  if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise ValueError('input hash mismatch '+str(path))
  hashes[str(path)]=expected
 scene=resolve(result['scene'])
 if str(scene) not in hashes:raise ValueError('scene is not hash bound')
 return scene,hashes
def state_error(model, qref, vref, qpos, qvel):
 """MuJoCo local tangent: integratePos(qref, error_q, 1) gives qpos."""
 error=np.empty(2*model.nv)
 mujoco.mj_differentiatePos(model,error[:model.nv],1.,qref,qpos)
 error[model.nv:]=qvel-vref
 return error
def periodic_lqr(As, Bs, Q, R, max_iterations=200, tolerance=1e-10):
 """Solve periodic DRE; never silently use a truncated fixed-point iteration."""
 if not len(As) or len(As)!=len(Bs):raise ValueError('nonempty matching transitions required')
 P=Q.copy()
 def sweep(terminal):
  P=terminal.copy();gains=[]
  for A,B in zip(As[::-1],Bs[::-1]):
   K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
   P=Q+A.T@P@(A-B@K);P=(P+P.T)/2;gains.append(K)
  return P,gains[::-1]
 for iteration in range(max_iterations):
  updated,_=sweep(P)
  residual=float(np.linalg.norm(updated-P)/max(np.linalg.norm(updated),1.))
  if not np.isfinite(residual):raise ValueError('nonfinite Riccati iteration')
  P=updated
  if residual<tolerance:
   checked,gains=sweep(P)
   residual=float(np.linalg.norm(checked-P)/max(np.linalg.norm(checked),1.))
   if residual<tolerance:return gains,iteration+1,residual
 raise ValueError('periodic Riccati iteration did not converge')

def translated_reference(data, displacement):
 q=data.qpos.copy();q[0]+=displacement
 return {'qpos':q.tolist(),'qvel':data.qvel.tolist()}
def main():
 import verify_whole_body_shooting as verifier
 p=argparse.ArgumentParser();p.add_argument('trajectory');p.add_argument('--out',required=True);p.add_argument('--periods',type=int,default=5);p.add_argument('--initial-vy',type=float,default=0);p.add_argument('--initial-roll',type=float,default=0);p.add_argument('--r-weight',type=float,default=.01);a=p.parse_args()
 source=pathlib.Path(a.trajectory).resolve();r=json.loads(source.read_text());scene,hashes=resolve_inputs(r,source);m=mujoco.MjModel.from_xml_path(str(scene));u=np.array(r['controls']);N=len(u);nx=2*m.nv
 if abs(N*m.opt.timestep-r['period_s'])>1e-10 or m.na or m.nv!=18 or m.nu!=12 or a.periods<1:raise ValueError('one-cycle na0 nominal required')
 if a.r_weight<=0 or not np.isfinite(a.r_weight):raise ValueError('positive finite effort weight required')
 if u.shape!=(N,m.nu) or not np.all(np.isfinite(u)):raise ValueError('invalid nominal controls')
 if not np.all(np.isfinite([a.initial_vy,a.initial_roll])):raise ValueError('nonfinite perturbation')
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  nominal_certificate=verifier.verify_result(source)
  if not all(nominal_certificate['checks'][key] for key in ('state_reproduction','time_reproduction','force_reproduction','torque_reproduction')):raise ValueError('nominal replay does not reproduce saved trajectory')
  initial=mujoco.MjData(m);mujoco.mj_setState(m,initial,np.array(r['initial_integration_state']),r['integration_state_spec']);d=copy.copy(initial);As=[];Bs=[];nom=[];derivative_checks=[];started=time.perf_counter()
  for k in range(N):
   d.ctrl[:]=u[k];nom.append(copy.copy(d));A,B,check=checked_transition_fd(m,copy.copy(d));derivative_checks.append(check);As.append(A);Bs.append(B);mujoco.mj_step(m,d)
  Q=np.diag(1/np.r_[[.04,.025,.025],[.08]*3,[.35]*12,[.3,.3,.4],[.6]*3,[5.]*12]**2);R=np.eye(m.nu)*a.r_weight
  Ks,iterations,riccati_residual=periodic_lqr(As,Bs,Q,R)
  wrapq=initial.qpos.copy();wrapq[0]+=r['command_vx']*r['period_s']
  wrap_error=state_error(m,wrapq,initial.qvel,d.qpos,d.qvel)
  F=np.eye(nx)
  for A,B,K in zip(As,Bs,Ks):F=(A-B@K)@F
  radius=float(max(abs(np.linalg.eigvals(F))));setup=time.perf_counter()-started
  d=copy.copy(initial);d.qvel[1]+=a.initial_vy
  v=np.zeros(m.nv);v[3]=a.initial_roll;mujoco.mj_integratePos(m,d.qpos,v,1.)
  saved_initial=np.empty(mujoco.mj_stateSize(m,r['integration_state_spec']));mujoco.mj_getState(m,d,saved_initial,r['integration_state_spec']);terminal=translated_reference(d,r['command_vx']*r['period_s']*a.periods);nominal_terminal=translated_reference(initial,r['command_vx']*r['period_s']*a.periods)
  gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')];va=m.jnt_dofadr[m.actuator_trnid[:,0]];rows=[];controls=[];latencies=[];tracking=[];saturation=0
  for step in range(N*a.periods):
   k=step%N;cycle=step//N;qref=nom[k].qpos.copy();qref[0]+=cycle*r['command_vx']*r['period_s'];begin=time.perf_counter_ns();error=state_error(m,qref,nom[k].qvel,d.qpos,d.qvel);requested=u[k]-Ks[k]@error;cmd=np.clip(requested,-35,35);latencies.append((time.perf_counter_ns()-begin)/1000);saturation+=int(np.any(abs(requested)>35));tracking.append(float(np.linalg.norm(error)));d.ctrl[:]=cmd;mujoco.mj_step(m,d);obs=copy.copy(d);mujoco.mj_forward(m,obs);controls.append(cmd.tolist());rows.append({'time':float(obs.time),'qpos':obs.qpos.tolist(),'qvel':obs.qvel.tolist(),'normal_forces':forces(m,obs,gids).tolist(),'torque':obs.qfrc_actuator[va].tolist()})
  out={k:v for k,v in r.items() if k not in ('rows','controls','solver','terminal_reference','initial_integration_state','source_sha','curation')};out.update({'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'scope':'periodic whole-body state-feedback initialized replay; privileged model, not live controller or B1','initial_integration_state':saved_initial.tolist(),'terminal_reference':terminal,'controls':controls,'rows':rows,'feedback':{'method':'periodic time-varying discrete LQR with applied35Nm clipping','nominal_certificate':nominal_certificate,'transition_derivative_checks':derivative_checks,'nominal_terminal_reference':nominal_terminal,'nominal_terminal_tangent_error':state_error(m,np.array(nominal_terminal['qpos']),np.array(nominal_terminal['qvel']),d.qpos,d.qvel).tolist(),'v1_terminal_reference_scope':'perturbed initial translated as frozen V1 requires; nominal recovery is separately reported','r_weight':a.r_weight,'riccati_iterations':iterations,'riccati_relative_residual':riccati_residual,'nominal_wrap_tangent_error':wrap_error.tolist(),'periods':a.periods,'periodic_orbit_certified':False,'spectral_radius_scope':'repeated nominal local linear maps; nonzero wrap defect prevents a periodic-orbit stability claim','linear_period_map_spectral_radius':radius,'setup_s':setup,'feedback_latency_us':dict(zip(['p50','p95','max'],[float(np.percentile(latencies,50)),float(np.percentile(latencies,95)),max(latencies)])),'saturated_samples':saturation,'max_state_tangent_error_norm':max(tracking),'initial_vy_perturbation_mps':a.initial_vy,'initial_roll_perturbation_rad':a.initial_roll,'gains':[K.tolist() for K in Ks]}})
  masks=[sum(1<<leg for leg,value in enumerate(row['normal_forces']) if value>10.) for row in rows]
  out['feedback']['collision_force_contact_summary']={'leg_bit_order':['FR','FL','RR','RL'],'threshold_n':10.,'samples_by_mask':{str(mask):masks.count(mask) for mask in sorted(set(masks))},'provenance':'poststep collision force; neither planned contact nor running acceptance'}
  verifier_path=pathlib.Path(verifier.__file__).resolve()
  out['scene']=str(scene);out['input_hashes']=hashes
  helper_path=pathlib.Path(__file__).with_name('whole_body_shooting_derivatives.py').resolve()
  out['input_hashes'][str(helper_path)]=hashlib.sha256(helper_path.read_bytes()).hexdigest()
  out['input_hashes'][str(verifier_path)]=hashlib.sha256(verifier_path.read_bytes()).hexdigest()
  out['input_hashes'].update({str(source):hashlib.sha256(source.read_bytes()).hexdigest(),str(pathlib.Path(__file__).resolve()):hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()})
  with open(a.out,'x') as f:json.dump(out,f,indent=2,allow_nan=False);f.write('\n')
  print(json.dumps({'out':a.out,'feedback':{k:v for k,v in out['feedback'].items() if k!='gains'},'peak_force':max(max(r['normal_forces']) for r in rows),'end_omega':rows[-1]['qvel'][3:6]},indent=2))
if __name__=='__main__':main()
