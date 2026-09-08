"""Matched periodic-feedback continuation of baseline and coupled32ms controls.
Privileged counterfactual diagnostic only. Failure is policy-specific, never a
proof that no recovery or B1 traversal is possible.
"""
import argparse,copy,fcntl,hashlib,json,pathlib,subprocess
import mujoco,numpy as np
import whole_body_horizon_probe as hp
from probe_coupled_mujoco_horizon import ActualModelHorizon
def main():
 p=argparse.ArgumentParser();p.add_argument('result');p.add_argument('--packet',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 r=hp.read(a.result);manifest,refs,nominal=hp.packet(a.packet)
 if r['candidate_version']!=1 or len(refs)!=70:raise ValueError('requires phase-zero candidate1 and70-step reference')
 if r['solver']['controls'] is None:raise ValueError('missing coupled witness')
 for f,h in r['hashes'].items():
  if hp.sha(f)!=h:raise ValueError('source hash mismatch '+f)
 m=mujoco.MjModel.from_xml_path(r['scene']);spec=mujoco.mjtState.mjSTATE_INTEGRATION
 if hp.sha(r['scene'])!=hp.sha(manifest['model']['scene']):raise ValueError('scene identity mismatch')
 initial=mujoco.MjData(m);mujoco.mj_setState(m,initial,np.array(r['initial_integration_state']),spec)
 if abs(initial.time-refs[0]['time'])>1e-9:raise ValueError('reference clock mismatch')
 baseline=np.asarray(r['baseline_controls']);pb=ActualModelHorizon(m,initial,baseline,6,r['terminal_vy_bound'],True)
 results={}
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  for name,head in [('baseline',baseline),('coupled',np.asarray(r['solver']['controls']))]:
   d=copy.copy(initial);rows=[];first=None
   for tick in range(156):
    ref=dict(refs[tick%70]);ref['qpos']=np.array(ref['qpos']).copy();ref['qpos'][0]+=(tick//70)*nominal['command_vx']*nominal['period_s']
    tau=head[tick] if tick<16 else hp.control(m,ref,d)
    pre=copy.copy(d);pre.ctrl[:]=tau;mujoco.mj_forward(m,pre)
    force=hp.step(m,d,tau,pb.gids);post=copy.copy(d);mujoco.mj_forward(m,post)
    checks=np.r_[pb.observe(pre)[0],pb.observe(post)[0],35-abs(tau)]
    if not np.all(np.isfinite(np.r_[d.qpos,d.qvel,checks])):raise ValueError('nonfinite state/check')
    violation=float(np.max(np.maximum(-checks,0)))
    if violation>0 and first is None:first={'tick':tick,'time':float(d.time),'force_peak':float(max(force)),'scaled_constraint_violation':violation,'phase':'head' if tick<16 else 'continuation'}
    if abs(d.time-initial.time-(tick+1)*.002)>1e-10:raise ValueError('clock drift')
    e=hp.error(m,ref,d)
    rows.append({'tick':tick,'time':float(d.time),'tau':tau.tolist(),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'forces':force.tolist(),'state_error_norm':float(np.linalg.norm(e)),'scaled_constraint_violation':violation})
    if d.qpos[2]<.20:break
   results[name]={'first_constraint_violation':first,'completed_steps':len(rows),'rows':rows}
 out={'schema':'coupled-continuation-audit-v1','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'scope':'matched privileged periodic K feedback; no force projection, runtime authority or B1 claim','policy':'same70-step nominal tau-K*error,35Nm clip, absolute phase maintained;16head+140continuation steps','input_hashes':{str(pathlib.Path(f).resolve()):hp.sha(f) for f in [a.result,pathlib.Path(a.packet)/'manifest.json',pathlib.Path(a.packet)/'trajectory.packet',__file__,pathlib.Path(__file__).with_name('whole_body_horizon_probe.py'),pathlib.Path(__file__).with_name('probe_coupled_mujoco_horizon.py')]},'results':results}
 hp.write(a.out,out)
 print(json.dumps({n:{'first_violation':v['first_constraint_violation'],'steps':v['completed_steps'],'terminal_velocity':v['rows'][-1]['qvel'][:6]} for n,v in results.items()},indent=2))
if __name__=='__main__':main()
