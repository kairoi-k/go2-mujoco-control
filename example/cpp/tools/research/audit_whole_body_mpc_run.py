"""Independent actual executed contacts, physical replay and failed-tail diagnosis."""
import argparse,copy,fcntl,hashlib,json,pathlib,subprocess
import mujoco,numpy as np
from whole_body_mpc_native import WholeBodyMPC
from verify_whole_body_mpc_oracle import verify_rows
def audit(path,out):
    path=pathlib.Path(path);r=json.loads(path.read_text());out=pathlib.Path(out)
    if out.exists():raise ValueError('output already exists')
    scene=r['source']['scene']
    if hashlib.sha256(pathlib.Path(scene).read_bytes()).hexdigest()!=r['source']['sha256'][scene]:raise ValueError('scene changed')
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        m=mujoco.MjModel.from_xml_path(scene);verification=verify_rows(m,r['initial_integration_state'],r['executed_rows'])
        d=mujoco.MjData(m);mujoco.mj_setState(m,d,np.array(r['initial_integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
        gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
        step_id=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'phase2_step_5cm');contacts=[]
        for row in r['executed_rows']:
            d.ctrl[:]=row['control'];mujoco.mj_step(m,d);observed=copy.copy(d);mujoco.mj_forward(m,observed)
            for i,c in enumerate(observed.contact):
                pair=(c.geom1,c.geom2)
                if step_id not in pair:continue
                legs=[leg for leg,g in enumerate(gids) if g in pair]
                f=np.zeros(6);mujoco.mj_contactForce(m,observed,i,f)
                if not legs or f[0]<=1.:continue
                for leg in legs:contacts.append({'time':float(d.time),'leg':('FR','FL','RR','RL')[leg],'normal_force_n':float(f[0]),'foot_center':observed.geom_xpos[gids[leg]].tolist(),'normal_world':c.frame[:3].tolist(),'top_contact':bool(abs(c.frame[2])>.9)})
        first_by_leg={}
        for c in contacts:first_by_leg.setdefault(c['leg'],c)
        failed=None
        if r['chunks'] and 'candidate_controls' not in r['chunks'][-1]:
            c=r['chunks'][-1];pb=WholeBodyMPC(r['source']['library'],scene,c['initial_integration_state'],c['baseline_controls'],c['body_refs'],c['foot_refs'],5,True,top_support_only=r['config'].get('top_support_only',False))
            replay=pb.replay(c['baseline_controls']);negative=np.flatnonzero(replay['g']<0);issues=[]
            for idx in negative:
                k=int(idx)//96;post=int(idx)//48%2
                issues.append({'step':k,'pre_post':post,'component':int(idx)%48,'g':float(replay['g'][idx]),'time':c['start_time_s']+(k+post)*.002,'forces':replay['states'][k]['forces']})
            failed={'chunk':c['index'],'solver':c['solver'],'baseline_cost':replay['cost'],'baseline_min_g':float(min(replay['g'])),'baseline_violations':issues};pb.close()
        lat=np.array([c['solver']['elapsed_s'] for c in r['chunks']]);cl=np.array([c['chunk_latency_ms'] for c in r['chunks'] if 'chunk_latency_ms' in c])
        result={'schema':'whole-body-mpc-run-audit-v1','scope':'known-scene initialized diagnostic only; no B1 acceptance','run_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'run_source_sha':r['source']['git_head'],'audit_source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'physical_verification':verification,'first_step_contact_by_leg':first_by_leg,'step_contacts':contacts,'failed_horizon':failed,'solver_s_p50_p95_max':np.percentile(lat,[50,95,100]).tolist(),'total_chunk_ms_p50_p95_max':np.percentile(cl,[50,95,100]).tolist() if len(cl) else None}
        with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
        print(json.dumps({k:v for k,v in result.items() if k!='step_contacts'},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('out');a=p.parse_args();audit(a.run,a.out)
