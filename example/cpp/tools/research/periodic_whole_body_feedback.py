"""Periodic full-body TVLQR replay challenger; privileged model, not live B1."""
import argparse,copy,fcntl,hashlib,json,pathlib,subprocess,time
import numpy as np,mujoco
from whole_body_cycle import forces
def main():
 p=argparse.ArgumentParser();p.add_argument('trajectory');p.add_argument('--out',required=True);p.add_argument('--periods',type=int,default=5);p.add_argument('--initial-vy',type=float,default=0);p.add_argument('--initial-roll',type=float,default=0);p.add_argument('--r-weight',type=float,default=.01);a=p.parse_args()
 source=pathlib.Path(a.trajectory).resolve();r=json.loads(source.read_text());m=mujoco.MjModel.from_xml_path(r['scene']);u=np.array(r['controls']);N=len(u);nx=2*m.nv
 if abs(N*m.opt.timestep-r['period_s'])>1e-10 or m.na or a.periods<1:raise ValueError('one-cycle na0 nominal required')
 if a.r_weight<=0 or not np.isfinite(a.r_weight):raise ValueError('positive finite effort weight required')
 for name,h in r['input_hashes'].items():
  if hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest()!=h:raise ValueError('input hash mismatch '+name)
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  initial=mujoco.MjData(m);mujoco.mj_setState(m,initial,np.array(r['initial_integration_state']),r['integration_state_spec']);d=copy.copy(initial);As=[];Bs=[];nom=[];started=time.perf_counter()
  for k in range(N):
   d.ctrl[:]=u[k];nom.append(copy.copy(d));A=np.empty((nx,nx));B=np.empty((nx,m.nu));mujoco.mjd_transitionFD(m,copy.copy(d),1e-6,True,A,B,None,None);As.append(A);Bs.append(B);mujoco.mj_step(m,d)
  Q=np.diag(1/np.r_[[.04,.025,.025],[.08]*3,[.35]*12,[.3,.3,.4],[.6]*3,[5.]*12]**2);R=np.eye(m.nu)*a.r_weight;P=Q.copy();Ks=None
  for iteration in range(200):
   end=P.copy();gains=[]
   for A,B in zip(As[::-1],Bs[::-1]):
    K=np.linalg.solve(R+B.T@P@B,B.T@P@A);P=Q+A.T@P@(A-B@K);P=(P+P.T)/2;gains.append(K)
   Ks=gains[::-1]
   if np.linalg.norm(P-end)/max(np.linalg.norm(P),1)<1e-10:break
  F=np.eye(nx)
  for A,B,K in zip(As,Bs,Ks):F=(A-B@K)@F
  radius=float(max(abs(np.linalg.eigvals(F))));setup=time.perf_counter()-started
  d=copy.copy(initial);d.qvel[1]+=a.initial_vy
  v=np.zeros(m.nv);v[3]=a.initial_roll;mujoco.mj_integratePos(m,d.qpos,v,1.)
  saved_initial=np.empty(mujoco.mj_stateSize(m,r['integration_state_spec']));mujoco.mj_getState(m,d,saved_initial,r['integration_state_spec']);terminalq=d.qpos.copy();terminalq[0]+=r['command_vx']*r['period_s']*a.periods;terminalv=d.qvel.copy()
  gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')];va=m.jnt_dofadr[m.actuator_trnid[:,0]];rows=[];controls=[];latencies=[];tracking=[];saturation=0
  for step in range(N*a.periods):
   k=step%N;cycle=step//N;qref=nom[k].qpos.copy();qref[0]+=cycle*r['command_vx']*r['period_s'];begin=time.perf_counter_ns();error=np.zeros(nx);mujoco.mj_differentiatePos(m,error[:m.nv],1,qref,d.qpos);error[m.nv:]=d.qvel-nom[k].qvel;requested=u[k]-Ks[k]@error;cmd=np.clip(requested,-35,35);latencies.append((time.perf_counter_ns()-begin)/1000);saturation+=int(np.any(abs(requested)>35));tracking.append(float(np.linalg.norm(error)));d.ctrl[:]=cmd;mujoco.mj_step(m,d);obs=copy.copy(d);mujoco.mj_forward(m,obs);controls.append(cmd.tolist());rows.append({'time':float(obs.time),'qpos':obs.qpos.tolist(),'qvel':obs.qvel.tolist(),'normal_forces':forces(m,obs,gids).tolist(),'torque':obs.qfrc_actuator[va].tolist()})
  out={k:v for k,v in r.items() if k not in ('rows','controls','solver','terminal_reference','initial_integration_state','source_sha')};out.update({'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'scope':'periodic whole-body state-feedback initialized replay; privileged model, not live controller or B1','initial_integration_state':saved_initial.tolist(),'terminal_reference':{'qpos':terminalq.tolist(),'qvel':terminalv.tolist()},'controls':controls,'rows':rows,'feedback':{'method':'periodic time-varying discrete LQR with applied35Nm clipping','r_weight':a.r_weight,'riccati_iterations':iteration+1,'linear_period_map_spectral_radius':radius,'setup_s':setup,'feedback_latency_us':dict(zip(['p50','p95','max'],[float(np.percentile(latencies,50)),float(np.percentile(latencies,95)),max(latencies)])),'saturated_samples':saturation,'max_state_tangent_error_norm':max(tracking),'initial_vy_perturbation_mps':a.initial_vy,'initial_roll_perturbation_rad':a.initial_roll,'gains':[K.tolist() for K in Ks]}})
  out['input_hashes'].update({str(source):hashlib.sha256(source.read_bytes()).hexdigest(),str(pathlib.Path(__file__).resolve()):hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()})
  with open(a.out,'x') as f:json.dump(out,f,indent=2);f.write('\n')
  print(json.dumps({'out':a.out,'feedback':{k:v for k,v in out['feedback'].items() if k!='gains'},'peak_force':max(max(r['normal_forces']) for r in rows),'end_omega':rows[-1]['qvel'][3:6]},indent=2))
if __name__=='__main__':main()
