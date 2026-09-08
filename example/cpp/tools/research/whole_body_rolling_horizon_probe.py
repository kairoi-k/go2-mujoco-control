"""Hash-bound input and independent continuous replay for one fixed12ms schedule.
Periodic nominal reuse is authorized for this initialized offline diagnostic only.
"""
import argparse,copy,fcntl,pathlib
import mujoco,numpy as np
import whole_body_horizon_probe as probe

def prepare(packet_dir,out):
    manifest,refs,nominal=probe.packet(packet_dir);out=pathlib.Path(out)
    if len(refs)!=70: raise ValueError('requires the 70step periodic research reference')
    fmt=lambda a:' '.join(format(float(x),'.17g') for x in np.asarray(a).ravel())
    lines=['whole-body-rolling-horizon-input-v1',manifest['model']['scene'],f"{len(nominal['initial_integration_state'])} 70 {nominal['command_vx']:.17g} {nominal['period_s']:.17g}",fmt(nominal['initial_integration_state'])]
    for r in refs: lines.append(fmt(np.r_[r['qpos'],r['qvel'],r['tau'],r['K'].ravel()]))
    with (out/'input.txt').open('x') as f:f.write('\n'.join(lines+['end'])+'\n')
    probe.write(out/'input_manifest.json',{'packet_dir':str(pathlib.Path(packet_dir).resolve()),'manifest_sha256':probe.sha(pathlib.Path(packet_dir)/'manifest.json'),'packet_sha256':probe.sha(pathlib.Path(packet_dir)/'trajectory.packet'),'input_sha256':probe.sha(out/'input.txt'),'offline_periodic_reference_reuse_authorized':True,'update_steps':6,'prefix_steps':6,'tail_steps':10,'periods':20,'initial_vy_delta':.05,'late_policy':'discard; stop if remaining old coverage cannot supply full 12ms commitment'})

def can_commit(end_tick,observation_tick): return end_tick>=observation_tick+6
def audit_schedule(run):
    """Validate epoch/coverage/late records independently of physics replay."""
    candidates=run['candidates'];by_tick={};by_version={0:{'start_tick':0,'end_tick':6}}
    for index,c in enumerate(candidates):
        obs=c['observation_tick']
        if c['version']!=index+1 or obs%6 or obs in by_tick:raise ValueError('candidate identity/calendar')
        if c['start_tick']!=obs+6 or c['end_tick']!=obs+16:raise ValueError('candidate coverage')
        if not np.isfinite(c['generation_ms']) or c['generation_ms']<0 or c['late']!=(c['generation_ms']>12):raise ValueError('invalid measured lateness')
        if c['valid'] and (len(c['stages'])!=16 or [s['tick'] for s in c['stages']]!=list(range(obs,obs+16))):raise ValueError('candidate sample coverage')
        by_tick[obs]=c;by_version[c['version']]=c
    rows=run['rows'];active=0;adopted=set();last=max(len(rows)-1,max(by_tick,default=-1))
    for tick in range(last+1):
        if tick%6==0:
            prior=by_tick.get(tick-6)
            if prior and prior['valid'] and not prior['late']:
                if prior['source_version']!=active:raise ValueError('superseded source version')
                active=prior['version'];adopted.add(active)
            current=by_tick.get(tick)
            if current and (current['source_version']!=active or not can_commit(by_version[active]['end_tick'],tick)):
                raise ValueError('observation/committed prefix mismatch')
        if tick<len(rows):
            row=rows[tick]
            if row['tick']!=tick or row['active_version']!=active:raise ValueError('actual epoch/calendar mismatch')
            if not by_version[active]['start_tick']<=tick<by_version[active]['end_tick']:raise ValueError('actual law expired')
    if any(c['adopted']!=(c['version'] in adopted) for c in candidates):raise ValueError('adoption record mismatch')
    return {'adopted_versions':sorted(adopted),'rows_checked':len(rows),'candidates_checked':len(candidates)}

def verify(packet_dir,out):
    manifest,refs,nominal=probe.packet(packet_dir);out=pathlib.Path(out);run=probe.read(out/'run.json');audit_schedule(run);model=mujoco.MjModel.from_xml_path(manifest['model']['scene']);spec=mujoco.mjtState.mjSTATE_INTEGRATION
    gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
    nominal_at=lambda tick:dict(refs[tick%70],qpos=np.asarray(refs[tick%70]['qpos'])+np.r_[(tick//70)*nominal['command_vx']*nominal['period_s'],np.zeros(18)])
    candidates={c['version']:c for c in run['candidates']}
    def law(version,tick):
        if version==0:
            if not 0<=tick<6:raise ValueError('bootstrap extrapolation')
            return nominal_at(tick)
        c=candidates[version]
        if not c['start_tick']<=tick<c['end_tick']:raise ValueError('published law extrapolation')
        return next(s for s in c['stages'] if s['tick']==tick)
    d=mujoco.MjData(model);mujoco.mj_setState(model,d,np.asarray(run['initial_integration_state']),spec)
    states={0:np.asarray(run['initial_integration_state'])};max_state=max_force=max_law=0.;violations=[];observations=[];active=0;schedule_checks=[]
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for row in run['rows']:
            tick=row['tick']
            if tick and tick%6==0:
                previous=next((c for c in run['candidates'] if c['observation_tick']==tick-6),None)
                if previous and previous['valid'] and not previous['late'] and previous['source_version']==active: active=previous['version']
            if row['active_version']!=active:raise ValueError('publish ordering/version mismatch')
            requested=probe.control(model,law(active,tick),d);max_law=max(max_law,float(max(abs(requested-np.asarray(row['tau'])))))
            f=probe.step(model,d,np.asarray(row['tau']),gids);x=np.empty(len(run['initial_integration_state']));mujoco.mj_getState(model,d,x,spec)
            states[tick+1]=x.copy();max_state=max(max_state,float(max(abs(x-np.asarray(row['next_integration_state'])))));max_force=max(max_force,float(max(abs(f-np.asarray(row['forces'])))))
            if max(f)>180 or max(abs(np.asarray(row['tau'])))>35:violations.append(tick)
            observations.append({'tick':tick,'time':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'forces':f.tolist()})
        for c in run['candidates']:
            source=c['source_version'];end=6 if source==0 else candidates[source]['end_tick'];obs=c['observation_tick']
            if not can_commit(end,obs):raise ValueError('candidate generated without complete committed prefix')
            if c['late']!=(c['generation_ms']>12) or (c['adopted'] and c['late']):raise ValueError('late candidate admitted')
            if obs not in states or not np.array_equal(states[obs],np.asarray(c['initial_integration_state'])):raise ValueError('candidate observation differs from continuous actual integration state')
            p=mujoco.MjData(model);mujoco.mj_setState(model,p,np.asarray(c['initial_integration_state']),spec);prefix_error=force_error=qv_error=0.
            stages=c['stages'] if c['valid'] or c['failure'].startswith('committed_prefix_force_violation:') else c['stages'][:-1]
            for j,s in enumerate(stages):
                qv_error=max(qv_error,float(max(abs(p.qpos-np.asarray(s['qpos'])))),float(max(abs(p.qvel-np.asarray(s['qvel'])))))
                if j<6:prefix_error=max(prefix_error,float(max(abs(probe.control(model,law(source,s['tick']),p)-np.asarray(s['tau'])))))
                f=probe.step(model,p,np.asarray(s['tau']),gids);force_error=max(force_error,float(max(abs(f-np.asarray(s['forces'])))))
            schedule_checks.append({'version':c['version'],'valid':c['valid'],'late':c['late'],'adopted':c['adopted'],'committed_prefix_law_error_nm':prefix_error,'candidate_force_replay_delta_n':force_error,'candidate_qv_replay_delta':qv_error})
    latency=np.asarray([c['generation_ms'] for c in run['candidates']]);report={'scope':'one initialized offline continuous run; synchronous compute with measured-late schedule, no liveRT or B1 certificate','target_steps':1400,'completed_steps':run['completed_steps'],'failure':run['failure'],'failure_tick':run['failure_tick'],'actual_integration_replay_delta':max_state,'actual_force_replay_delta_n':max_force,'actual_published_law_error_nm':max_law,'actual_constraint_violating_ticks':violations,'max_actual_force_n':max((max(row['forces']) for row in run['rows']),default=0),'candidate_count':len(candidates),'late_candidates':[c['version'] for c in run['candidates'] if c['late']],'adopted_candidates':sum(c['adopted'] for c in run['candidates']),'generation_ms':{'min':float(min(latency)),'median':float(np.median(latency)),'max':float(max(latency))},'candidate_checks':schedule_checks,'observations':observations,'run_sha256':probe.sha(out/'run.json')}
    probe.write(out/'verification.json',report);print({k:v for k,v in report.items() if k not in ('candidate_checks','observations')})
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('prepare','verify'));p.add_argument('packet_dir');p.add_argument('out_dir');a=p.parse_args();(prepare if a.mode=='prepare' else verify)(a.packet_dir,a.out_dir)
