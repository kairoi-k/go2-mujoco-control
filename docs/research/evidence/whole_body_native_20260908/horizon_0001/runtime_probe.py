"""Prepare hash-bound research input and independently replay native horizon candidates.
Synthetic delay replays do not assert real computation finished in that delay.
No state-error tolerance or feedback-tube certificate is defined here.
"""
import argparse,copy,fcntl,hashlib,json,pathlib
import mujoco
import numpy as np

def sha(path): return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
def read(path): return json.loads(pathlib.Path(path).read_text())
def write(path,value):
    with pathlib.Path(path).open('x') as f: json.dump(value,f,indent=2,allow_nan=False)
def packet(directory):
    directory=pathlib.Path(directory);manifest=read(directory/'manifest.json');receipt=read(directory/'receipt.json')
    if sha(directory/'manifest.json')!=receipt['manifest_sha256'] or sha(directory/'trajectory.packet')!=receipt['packet_sha256']:
        raise ValueError('packet receipt hash mismatch')
    for name,digest in manifest['all_bound_input_hashes'].items():
        if sha(name)!=digest: raise ValueError('bound input mismatch '+name)
    lines=(directory/'trajectory.packet').read_text().splitlines()
    if lines[:2]!=['whole-body-trajectory-v1-research','manifest_sha256 '+receipt['manifest_sha256']]: raise ValueError('packet identity mismatch')
    refs=[]
    for line in lines:
        if not line.startswith('sample '): continue
        fields=line.split();k=int(fields[1]);time=int(fields[2])*1e-9;v=np.asarray(fields[3:],dtype=float)
        if k!=len(refs) or len(v)!=481 or not np.isfinite(v).all(): raise ValueError('sample shape/order')
        refs.append({'time':time,'qpos':v[:19],'qvel':v[19:37],'tau':v[37:49],'K':v[49:].reshape(12,36)})
    if len(refs)!=manifest['coverage']['samples'] or len(refs)<10: raise ValueError('missing source coverage')
    nominal=read(manifest['source_hashes']['nominal']['path'])
    if nominal['integration_state_spec']!=int(mujoco.mjtState.mjSTATE_INTEGRATION): raise ValueError('full integration state required')
    return manifest,refs,nominal
def prepare(directory,out):
    m,refs,nominal=packet(directory);out=pathlib.Path(out)
    values=lambda a:' '.join(format(float(v),'.17g') for v in np.asarray(a).ravel())
    lines=['whole-body-horizon-probe-input-v1',m['model']['scene'],f"{len(nominal['initial_integration_state'])} 10",values(nominal['initial_integration_state'])]
    for r in refs[:10]: lines.append(values(np.r_[r['time'],r['qpos'],r['qvel'],r['tau'],r['K'].ravel()]))
    with (out/'input.txt').open('x') as f:f.write('\n'.join(lines+['end'])+'\n')
    write(out/'input_manifest.json',{'packet_directory':str(pathlib.Path(directory).resolve()),'manifest_sha256':sha(pathlib.Path(directory)/'manifest.json'),'packet_sha256':sha(pathlib.Path(directory)/'trajectory.packet'),'input_sha256':sha(out/'input.txt'),'scope':'initialized privileged flat research; no production or observed-terrain authority'})
def temporal_admission(plan,absolute_time,dt=.002):
    if not plan['complete'] or plan['completed_steps']!=10 or len(plan['stages'])!=10: return False,'missing_coverage'
    start=plan['stages'][0]['time'];end=start+10*dt
    if absolute_time<start-1e-12: return False,'before_coverage'
    if absolute_time>=end-1e-12: return False,'expired'
    k=round((absolute_time-start)/dt)
    if k<0 or k>=10 or abs(plan['stages'][k]['time']-absolute_time)>1e-9: return False,'missing_sample'
    return True,'temporal_coverage_only'
def error(model,reference,data):
    e=np.empty(36);mujoco.mj_differentiatePos(model,e[:18],1.,np.asarray(reference['qpos']),data.qpos);e[18:]=data.qvel-np.asarray(reference['qvel']);return e
def control(model,reference,data):
    return np.clip(np.asarray(reference['tau'])-np.asarray(reference['K']).reshape(12,36)@error(model,reference,data),-35,35)
def forces(model,data,gids):
    result=np.zeros(4)
    for i,c in enumerate(data.contact):
        force=np.empty(6);mujoco.mj_contactForce(model,data,i,force)
        for leg,gid in enumerate(gids):
            if gid in (c.geom1,c.geom2):result[leg]+=force[0]
    return result
def step(model,data,tau,gids):
    pre=copy.copy(data);pre.ctrl[:]=tau;mujoco.mj_forward(model,pre)
    data.ctrl[:]=tau;mujoco.mj_step(model,data)
    post=copy.copy(data);mujoco.mj_forward(model,post)
    for sample in (pre,post):
        if any(sample.warning[int(w)].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL)):
            raise ValueError('numerical reset/warning in physical replay')
    return np.r_[forces(model,pre,gids),forces(model,post,gids)]
def replay(directory,out):
    manifest,refs,nominal=packet(directory);out=pathlib.Path(out);plans=read(out/'plans.json')['plans'];model=mujoco.MjModel.from_xml_path(manifest['model']['scene']);spec=mujoco.mjtState.mjSTATE_INTEGRATION
    gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')];results=[];checks=[]
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for plan in plans:
            d=mujoco.MjData(model);mujoco.mj_setState(model,d,np.array(plan['initial_integration_state']),spec);max_f=max_x=0.
            for stage in plan['stages']:
                f=step(model,d,np.array(stage['tau']),gids);state=np.empty(mujoco.mj_stateSize(model,spec));mujoco.mj_getState(model,d,state,spec)
                max_f=max(max_f,float(np.max(abs(f-np.array(stage['forces'])))));max_x=max(max_x,float(np.max(abs(state-np.array(stage['next_integration_state'])))))
            checks.append({'plan':plan['name'],'complete':plan['complete'],'steps':plan['completed_steps'],'force_replay_delta_n':max_f,'integration_replay_delta':max_x})
            for delay_ms in (0,2,6,20):
                delay_steps=delay_ms//2;d=mujoco.MjData(model);mujoco.mj_setState(model,d,np.array(nominal['initial_integration_state']),spec);d.qvel[1]+=.05
                adoption_time=float(d.time+delay_ms*.001);admitted,reason=temporal_admission(plan,adoption_time);adoption_error=None;rows=[]
                for k in range(10):
                    proposed=admitted and k>=delay_steps
                    reference=plan['stages'][k] if proposed else refs[k]
                    if proposed and k==delay_steps:
                        e=error(model,reference,d);adoption_error={'position_tangent_max':float(max(abs(e[:18]))),'velocity_max':float(max(abs(e[18:]))),'base_position_norm_m':float(np.linalg.norm(e[:3])),'base_velocity_norm_mps':float(np.linalg.norm(e[18:21])),'vector':e.tolist()}
                    tau=control(model,reference,d);before=float(d.time);f=step(model,d,tau,gids)
                    rows.append({'step':k,'time':before,'source':'candidate_feedback' if proposed else 'existing_feedback_prefix','tau':tau.tolist(),'forces':f.tolist(),'force_violation':bool(max(f)>180),'torque_violation':bool(max(abs(tau))>35),'qpos_after':d.qpos.tolist(),'qvel_after':d.qvel.tolist()})
                results.append({'plan':plan['name'],'injected_delay_ms':delay_ms,'actual_initial_vy_delta':.05,'temporal_admitted':admitted,'temporal_reason':reason,'zero_delay_offline_oracle':delay_ms==0,'measured_producer_ready_by_injected_delay':plan['producer_wall_ms_including_output']<=delay_ms,'measured_deadline_and_coverage_both_pass':admitted and plan['producer_wall_ms_including_output']<=delay_ms,'state_admission_certified':False,'adoption_error':adoption_error,'max_force_n':max(max(r['forces']) for r in rows),'force_violating_steps':[r['step'] for r in rows if r['force_violation']],'candidate_force_violating_steps':[r['step'] for r in rows if r['force_violation'] and r['source']=='candidate_feedback'],'rows':rows})
    report={'scope':'20ms initialized flat candidate/replay probe; no feedback tube, live authority, 2ms deadline guarantee or B1 acceptance','independent_candidate_replays':checks,'delay_replays':results,'plans_sha256':sha(out/'plans.json'),'input_manifest_sha256':sha(out/'input_manifest.json')}
    write(out/'replay_report.json',report)
    summary={'candidate_replays':checks,'plans':[{'name':p['name'],'complete':p['complete'],'failure':p['failure'],'producer_wall_ms_including_output':p['producer_wall_ms_including_output'],'max_force_n':max((max(s['forces']) for s in p['stages']),default=None)} for p in plans],'delay_replays':[{k:v for k,v in r.items() if k not in ('rows','adoption_error')}|{'adoption_error':None if r['adoption_error'] is None else {k:v for k,v in r['adoption_error'].items() if k!='vector'}} for r in results]}
    write(out/'summary.json',summary);print(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('prepare','replay'));p.add_argument('packet_dir');p.add_argument('out_dir');a=p.parse_args()
    (prepare if a.mode=='prepare' else replay)(a.packet_dir,a.out_dir)
