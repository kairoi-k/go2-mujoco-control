"""Independent sequential physical replay, preserving historical residual limits.
The caller supplies saved executed rows, never reinitializes between commands.
No traversal acceptance follows just from these physical checks.
"""
import copy
import mujoco
import numpy as np
from verify_whole_body_shooting import _contact_metrics,_rotation_metrics
def verify_rows(model,initial,rows):
    if not rows:raise ValueError('missing executed trajectory')
    spec=mujoco.mjtState.mjSTATE_INTEGRATION;d=mujoco.MjData(model)
    mujoco.mj_setState(model,d,np.asarray(initial),spec);t0=d.time
    gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
    root=int(model.jnt_bodyid[0]);robot={b for b in range(model.nbody) if model.body_rootid[b]==root}
    mass=np.empty((model.nv,model.nv));state=np.empty(mujoco.mj_stateSize(model,spec))
    maxima={n:0. for n in ('state_delta','force_delta_n','clock_delta_s','dynamics_absolute','dynamics_scaled','dynamics_normwise','friction_violation_n','unilateral_violation_n','nonfoot_force_n','normal_force_n','torque_nm','motor_force_delta_nm','roll_rad','pitch_rad','joint_speed_radps','joint_position_violation_rad')}
    minheight=float('inf');masks={};force_history=[]
    joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    for k,row in enumerate(rows):
        u=np.asarray(row['control']);saved=np.asarray(row['integration_state'])
        if u.shape!=(12,) or saved.shape!=state.shape or not np.isfinite(np.r_[u,saved]).all():raise ValueError('row coverage/nonfinite')
        maxima['torque_nm']=max(maxima['torque_nm'],float(max(abs(u))))
        pre=copy.copy(d);pre.ctrl[:]=u;mujoco.mj_forward(model,pre)
        d.ctrl[:]=u;mujoco.mj_step(model,d);post=copy.copy(d);mujoco.mj_forward(model,post)
        fs=[]
        for observed in (pre,post):
            if any(observed.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL)):raise ValueError('numerical warning')
            f,friction,unilateral,count,nonfoot=_contact_metrics(model,observed,gids,robot);fs.extend(f)
            mujoco.mj_fullM(model,mass,observed.qM);external=observed.qfrc_applied.copy()
            for b in range(1,model.nbody):
                if np.any(observed.xfrc_applied[b]):mujoco.mj_applyFT(model,observed,observed.xfrc_applied[b,:3],observed.xfrc_applied[b,3:],observed.xipos[b],b,external)
            terms=[mass@observed.qacc,observed.qfrc_bias,-observed.qfrc_passive,-observed.qfrc_actuator,-external,-observed.qfrc_constraint]
            residual=sum(terms);scale=1+sum(abs(t) for t in terms)
            roll,pitch=_rotation_metrics(observed.qpos[3:7]);q=observed.qpos[qa]
            values={'dynamics_absolute':max(abs(residual)),'dynamics_scaled':max(abs(residual)/scale),'dynamics_normwise':max(abs(residual))/(1+sum(max(abs(t)) for t in terms)),'friction_violation_n':friction,'unilateral_violation_n':unilateral,'nonfoot_force_n':nonfoot,'normal_force_n':max(f),'motor_force_delta_nm':max(abs(observed.actuator_force-u)),'roll_rad':roll,'pitch_rad':pitch,'joint_speed_radps':max(abs(observed.qvel[va])),'joint_position_violation_rad':max(0.,max(model.jnt_range[joints,0]-q),max(q-model.jnt_range[joints,1]))}
            if not np.isfinite(list(values.values())).all():raise ValueError('nonfinite physical metrics')
            for name,v in values.items():maxima[name]=max(maxima[name],float(v))
            minheight=min(minheight,float(observed.qpos[2]))
        mujoco.mj_getState(model,d,state,spec)
        maxima['state_delta']=max(maxima['state_delta'],float(max(abs(state-saved))))
        maxima['force_delta_n']=max(maxima['force_delta_n'],float(max(abs(np.asarray(fs)-np.asarray(row['forces'])))))
        maxima['clock_delta_s']=max(maxima['clock_delta_s'],abs(d.time-t0-(k+1)*model.opt.timestep),abs(d.time-row['time']))
        mask=sum(1<<l for l,f in enumerate(fs[4:]) if f>1.);masks[str(mask)]=masks.get(str(mask),0)+1
        force_history.append(fs[4:])
    checks={'state_replay':maxima['state_delta']<=1e-9,'force_replay':maxima['force_delta_n']<=1e-7,'historical_clock':maxima['clock_delta_s']<=1e-12,'historical_absolute_dynamics':maxima['dynamics_absolute']<=1e-7,'friction':maxima['friction_violation_n']<=1e-6,'unilateral':maxima['unilateral_violation_n']<=1e-6,'normal_force':maxima['normal_force_n']<=180,'torque':maxima['torque_nm']<=35,'motor_transmission':maxima['motor_force_delta_nm']<=1e-9,'joint_speed':maxima['joint_speed_radps']<=30,'joint_position':maxima['joint_position_violation_rad']<=0,'attitude':max(maxima['roll_rad'],maxima['pitch_rad'])<=np.pi/12,'height':minheight>=.28,'nonfoot':maxima['nonfoot_force_n']<=1e-6}
    return {'schema':'whole-body-mpc-sequential-replay-v1','scope':'physical replay only; no B1 acceptance','steps':len(rows),'maxima':maxima,'minimum_base_height_m':minheight,'checks':checks,'all_checks_pass':all(checks.values()),'measured_post_contact_mask_counts_1n':masks,'final_qpos':d.qpos.tolist(),'final_qvel':d.qvel.tolist()}
