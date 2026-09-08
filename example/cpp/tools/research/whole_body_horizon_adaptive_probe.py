"""One explicitly adaptive replay at ceil(measured prefix-candidate runtime / 2ms).
This does not extend its registered commitment or define a computation bound.
"""
import argparse,math,pathlib,fcntl
import mujoco,numpy as np
import whole_body_horizon_probe as probe
def run(packet_dir,out):
    out=pathlib.Path(out);manifest,refs,nominal=probe.packet(packet_dir)
    plans=probe.read(out/'plans.json');plan=next(p for p in plans['plans'] if p['name']=='actual_vy_prefix6ms')
    delay_ms=2*math.ceil(plan['producer_wall_ms_including_output']/2);delay_steps=delay_ms//2
    model=mujoco.MjModel.from_xml_path(manifest['model']['scene']);data=mujoco.MjData(model)
    mujoco.mj_setState(model,data,np.asarray(nominal['initial_integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION);data.qvel[1]+=.05
    gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
    admitted,reason=probe.temporal_admission(plan,float(data.time+delay_ms*.001));rows=[];adoption_error=None;prefix_control_deltas=[]
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for k in range(10):
            proposed=admitted and k>=delay_steps;ref=plan['stages'][k] if proposed else refs[k]
            if proposed and k==delay_steps: adoption_error=probe.error(model,ref,data).tolist()
            tau=probe.control(model,ref,data)
            if k<delay_steps: prefix_control_deltas.append(float(max(abs(tau-np.asarray(plan['stages'][k]['tau'])))))
            time=float(data.time);f=probe.step(model,data,tau,gids)
            rows.append({'step':k,'time':time,'source':'candidate_feedback' if proposed else 'existing_feedback_prefix','tau':tau.tolist(),'forces':f.tolist(),'qpos_after':data.qpos.tolist(),'qvel_after':data.qvel.tolist()})
    report={'scope':'single adaptive follow-up, no rerun or sweep; finite initialized replay only','selection_rule':'2 * ceil(original measured producer_wall_ms_including_output / 2)','plan':plan['name'],'registered_commitment_ms':plan['committed_prefix_steps']*2,'measured_producer_wall_ms':plan['producer_wall_ms_including_output'],'adaptive_delay_ms':delay_ms,'remaining_coverage_ms':max(0,20-delay_ms),'temporal_admitted':admitted,'reason':reason,'measured_ready':plan['producer_wall_ms_including_output']<=delay_ms,'waiting_exceeds_registered_commitment':delay_steps>plan['committed_prefix_steps'],'waiting_controls_vs_predicted_max_nm':max(prefix_control_deltas,default=0),'adoption_state_error':adoption_error,'state_error_tolerance_defined':False,'max_force_n':max(max(r['forces']) for r in rows),'violating_steps':[r['step'] for r in rows if max(r['forces'])>180 or max(abs(np.asarray(r['tau'])))>35],'plans_sha256':probe.sha(out/'plans.json'),'rows':rows}
    probe.write(out/'adaptive_replay.json',report)
    print({k:v for k,v in report.items() if k not in ('rows','adoption_state_error')});print('adoption_state_error_max',None if adoption_error is None else max(abs(np.asarray(adoption_error))))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('packet_dir');p.add_argument('out_dir');a=p.parse_args();run(a.packet_dir,a.out_dir)
