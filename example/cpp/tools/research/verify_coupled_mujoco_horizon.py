"""Independent saved-control replay and declared-constraint verification."""
import argparse,copy,fcntl,hashlib,json,pathlib
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from whole_body_horizon_probe import step
def main():
    p=argparse.ArgumentParser();p.add_argument('result');p.add_argument('--out',required=True);a=p.parse_args()
    path=pathlib.Path(a.result);r=json.loads(path.read_text())
    if r['schema']!='coupled-full-model-horizon-v1':raise ValueError('wrong schema')
    for f,h in r['hashes'].items():
        if hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()!=h:raise ValueError('hash mismatch '+f)
    if r['solver']['controls'] is None:raise ValueError('no returned trajectory to verify')
    controls=np.array(r['solver']['controls']);baseline=np.array(r['baseline_controls'])
    if controls.shape!=(16,12) or not np.all(np.isfinite(controls)):raise ValueError('invalid control coverage')
    if not np.array_equal(controls[:6],baseline[:6]):raise ValueError('fixed prefix changed')
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        m=mujoco.MjModel.from_xml_path(r['scene']);d=mujoco.MjData(m)
        mujoco.mj_setState(m,d,np.array(r['initial_integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
        initialtime=d.time;ids=m.actuator_trnid[:,0];va=m.jnt_dofadr[ids];qa=m.jnt_qposadr[ids]
        gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
        maxstate=maxforce=maxclock=0.;forcepeak=speedpeak=rollpitchpeak=nonfootpeak=0.;minheight=float('inf');jointviolation=0.;torqueerror=0.
        if len(r['rows'])!=16:raise ValueError('missing row coverage')
        for k,(tau,saved) in enumerate(zip(controls,r['rows'])):
            pre=copy.copy(d);pre.ctrl[:]=tau;mujoco.mj_forward(m,pre)
            f=step(m,d,tau,gids)
            obs=copy.copy(d);mujoco.mj_forward(m,obs)
            maxstate=max(maxstate,float(max(abs(d.qpos-saved['qpos']))),float(max(abs(d.qvel-saved['qvel']))))
            maxforce=max(maxforce,float(max(abs(f-np.r_[saved['pre_forces'],saved['post_forces']]))))
            maxclock=max(maxclock,abs(d.time-saved['time']),abs(d.time-initialtime-(k+1)*m.opt.timestep))
            forcepeak=max(forcepeak,float(max(f)))
            if min(f)<0:raise ValueError('unilateral violation')
            for q in (pre,obs):
                if not np.all(np.isfinite(np.r_[q.qpos,q.qvel,q.qacc])):raise ValueError('nonfinite state')
                pose=Rotation.from_quat(q.qpos[[4,5,6,3]]).as_euler('xyz')
                rollpitchpeak=max(rollpitchpeak,float(max(abs(pose[:2]))));speedpeak=max(speedpeak,float(max(abs(q.qvel[va]))));minheight=min(minheight,float(q.qpos[2]))
                jointviolation=max(jointviolation,float(max(m.jnt_range[ids,0]-q.qpos[qa])),float(max(q.qpos[qa]-m.jnt_range[ids,1])))
                torqueerror=max(torqueerror,float(max(abs(q.qfrc_actuator[va]-tau))))
                bad=0.
                for i in range(q.ncon):
                    c=q.contact[i];cf=np.empty(6);mujoco.mj_contactForce(m,q,i,cf);pair=[c.geom1,c.geom2]
                    foot=[g for g in pair if g in gids]
                    allowed=len(foot)==1 and m.body_rootid[m.geom_bodyid[pair[0]]]!=m.body_rootid[m.geom_bodyid[pair[1]]]
                    if not allowed:bad+=np.linalg.norm(cf[:3])
                nonfootpeak=max(nonfootpeak,float(bad))
        checks={'state_reproduction':maxstate<=1e-9,'force_reproduction':maxforce<=1e-8,'time_reproduction':maxclock<=1e-10,'motor_composition':torqueerror<=1e-9,'force_bound':forcepeak<=180.,'torque_bound':float(np.max(abs(controls)))<=35.,'joint_speed':speedpeak<=30.,'joint_range':jointviolation<=0.,'height':minheight>=.28,'posture':rollpitchpeak<=np.pi/12,'no_nonfoot_contact':nonfootpeak<=1e-6,'terminal_vy':abs(d.qvel[1])<=r['terminal_vy_bound'],'terminal_omega':max(abs(d.qvel[3:6]))<=.3}
        out={'scope':'independent32ms declared sampled constraints and replay; not swept coverage, fullcycle or B1','checks':{k:bool(v) for k,v in checks.items()},'passed':all(checks.values()),'state_residual':maxstate,'force_residual':maxforce,'clock_residual':maxclock,'motor_residual':torqueerror,'force_peak':forcepeak,'torque_peak':float(np.max(abs(controls))),'terminal_velocity':d.qvel[:6].tolist(),'result_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'verifier_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()}
        with open(a.out,'x') as f:json.dump(out,f,indent=2);f.write('\n')
        print(json.dumps(out,indent=2))
        if not out['passed']:raise SystemExit(1)
if __name__=='__main__':main()
