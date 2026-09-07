"""Independent saved-row control-law algebra only; no mj_step/forward or acceptance."""
import argparse,hashlib,json,pathlib
import mujoco
import numpy as np
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def resolve(p,base):
 p=pathlib.Path(p)
 return (p if p.is_absolute() else base/p).resolve()
def audit(filename):
 p=pathlib.Path(filename).resolve();r=json.loads(p.read_text());f=r['feedback'];candidates=[]
 for name,expected in r['input_hashes'].items():
  if name.endswith('result.json'):
   q=resolve(name,p.parent)
   if digest(q)==f['nominal_certificate']['result_sha256']:
    assert digest(q)==expected
    candidates.append(q)
 assert len(candidates)==1,'ambiguous nominal'
 n=json.loads(candidates[0].read_text());scene=resolve(r['scene'],p.parent)
 scene_hash=r['input_hashes'].get(str(scene))
 if scene_hash is None:
  scene_hash=next(v for k,v in r['input_hashes'].items() if resolve(k,p.parent)==scene)
 assert digest(scene)==scene_hash
 m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m);mujoco.mj_setState(m,d,np.array(r['initial_integration_state']),r['integration_state_spec']);ni=mujoco.MjData(m);mujoco.mj_setState(m,ni,np.array(n['initial_integration_state']),n['integration_state_spec'])
 K=np.array(f['gains']);u=np.array(n['controls']);N=len(u);errs=[];norms=[];saturated=0
 assert len(r['controls'])==len(r['rows'])==N*f['periods']
 for step,cmd in enumerate(r['controls']):
  k=step%N;cycle=step//N
  qactual=d.qpos if step==0 else np.array(r['rows'][step-1]['qpos']);vactual=d.qvel if step==0 else np.array(r['rows'][step-1]['qvel'])
  qref=ni.qpos.copy() if k==0 else np.array(n['rows'][k-1]['qpos']);vref=ni.qvel.copy() if k==0 else np.array(n['rows'][k-1]['qvel']);qref[0]+=cycle*r['command_vx']*r['period_s']
  error=np.empty(2*m.nv);mujoco.mj_differentiatePos(m,error[:m.nv],1.,qref,qactual);error[m.nv:]=vactual-vref
  requested=u[k]-K[k]@error;saturated+=int(np.any(abs(requested)>35));errs.append(float(np.max(abs(np.array(cmd)-np.clip(requested,-35,35)))));norms.append(float(np.linalg.norm(error)))
 qtarget=ni.qpos.copy();qtarget[0]+=r['command_vx']*r['period_s']*f['periods'];target=f['nominal_terminal_reference'];terminal=np.empty(2*m.nv);mujoco.mj_differentiatePos(m,terminal[:m.nv],1.,qtarget,np.array(r['rows'][-1]['qpos']));terminal[m.nv:]=np.array(r['rows'][-1]['qvel'])-ni.qvel
 wrapq=ni.qpos.copy();wrapq[0]+=r['command_vx']*r['period_s'];wrap=np.empty(2*m.nv);mujoco.mj_differentiatePos(m,wrap[:m.nv],1.,wrapq,np.array(n['rows'][-1]['qpos']));wrap[m.nv:]=np.array(n['rows'][-1]['qvel'])-ni.qvel
 return {'input_result':str(p),'input_result_sha256':digest(p),'nominal':str(candidates[0]),'nominal_sha256':digest(candidates[0]),'scene':str(scene),'scene_sha256':digest(scene),'source_sha':r['source_sha'],'samples':len(errs),'max_control_reconstruction_error_nm':max(errs),'saturated_samples_reconstructed':saturated,'saturated_samples_recorded':f['saturated_samples'],'max_tracking_norm_difference':abs(max(norms)-f['max_state_tangent_error_norm']),'nominal_terminal_q_difference':float(np.max(abs(qtarget-np.array(target['qpos'])))),'nominal_terminal_v_difference':float(np.max(abs(ni.qvel-np.array(target['qvel'])))),'nominal_terminal_error_difference':float(np.max(abs(terminal-np.array(f['nominal_terminal_tangent_error'])))),'nominal_wrap_error_difference':float(np.max(abs(wrap-np.array(f['nominal_wrap_tangent_error'])))),'wrap_q_max':float(max(abs(wrap[:m.nv]))),'wrap_v_max':float(max(abs(wrap[m.nv:]))) }
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('results',nargs='+');p.add_argument('--out',required=True);a=p.parse_args()
 report={'schema':'saved-feedback-control-law-algebra-v1','scope':'algebra on saved states and gains; no plant replay, derivative or B1 certificate','mujoco_version':mujoco.__version__,'script_sha256':digest(pathlib.Path(__file__)),'results':[audit(r) for r in a.results],'limitations':['local qpos tangent and qvel coordinates omit solver warmstart-state propagation','nonzero periodic wrap defect and absent frame transport prevent Floquet stability claim']}
 with open(a.out,'x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps(report,indent=2))
