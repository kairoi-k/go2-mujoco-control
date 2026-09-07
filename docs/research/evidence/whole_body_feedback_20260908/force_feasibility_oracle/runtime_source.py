"""Single exact-state torque feasibility oracle; not a controller or gain update."""
import argparse,copy,fcntl,hashlib,json,pathlib,time
import mujoco
import numpy as np
from scipy.optimize import minimize


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result');parser.add_argument('--out',required=True)
    args=parser.parse_args();source=pathlib.Path(args.result).resolve();r=json.loads(source.read_text())
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        scene=pathlib.Path(r['scene']);m=mujoco.MjModel.from_xml_path(str(scene))
        d=mujoco.MjData(m);mujoco.mj_setState(m,d,np.asarray(r['initial_integration_state']),r['integration_state_spec'])
        k=7
        for cmd in r['controls'][:k]:d.ctrl[:]=cmd;mujoco.mj_step(m,d)
        start=np.asarray(r['controls'][k]);gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
        qa=m.jnt_qposadr[m.actuator_trnid[:,0]];va=m.jnt_dofadr[m.actuator_trnid[:,0]];jids=m.actuator_trnid[:,0]
        def forces(data):
            out=np.zeros(4)
            for i,c in enumerate(data.contact):
                f=np.zeros(6);mujoco.mj_contactForce(m,data,i,f)
                for leg,g in enumerate(gids):
                    if g in (c.geom1,c.geom2):out[leg]+=f[0]
            return out
        def physical(data):
            w,x,y,z=data.qpos[3:7]
            roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1))
            return {'qpos':data.qpos.tolist(),'qvel':data.qvel.tolist(),'height_m':float(data.qpos[2]),
                'roll_pitch_rad':[float(roll),float(pitch)],'joint_speed_max_radps':float(max(abs(data.qvel[va]))),
                'joint_position_limits_pass':bool(np.all(data.qpos[qa]>=m.jnt_range[jids,0]) and np.all(data.qpos[qa]<=m.jnt_range[jids,1])),
                'force_n':forces(data).tolist()}
        def independent(u):
            pre=copy.copy(d);pre.ctrl[:]=u;obs=copy.copy(pre);mujoco.mj_forward(m,obs)
            mujoco.mj_step(m,pre);post=copy.copy(pre);mujoco.mj_forward(m,post)
            return np.r_[forces(obs),forces(post)],{'pre':physical(obs),'post':physical(post)}
        started=time.perf_counter();calls=0;best=None
        class Budget(Exception):pass
        def objective(u):return .5*float((u-start)@(u-start))
        def evaluate(u):
            nonlocal calls,best
            if time.perf_counter()-started>30:raise Budget()
            values,_=independent(u);calls+=1
            if np.all(values<=180.) and np.all(abs(u)<=35):
                candidate=(objective(u),u.copy())
                if best is None or candidate[0]<best[0]:best=candidate
            return 180.-values
        def jac(u,eps=1e-4):
            columns=[]
            for i in range(12):
                e=np.zeros(12);e[i]=eps
                columns.append((evaluate(u+e)-evaluate(u-e))/(2*eps))
            return np.column_stack(columns)
        def objective_jac(u):
            eps=1e-4;g=np.empty(12)
            for i in range(12):
                e=np.zeros(12);e[i]=eps;g[i]=(objective(u+e)-objective(u-e))/(2*eps)
            return g
        initial_force,initial_physical=independent(start)
        J1,J2=jac(start,1e-4),jac(start,1e-5)
        jac_check={'epsilons':[1e-4,1e-5],'relative_error':float(np.linalg.norm(J1-J2)/max(np.linalg.norm(J2),1e-12)),
            'max_abs':float(np.max(abs(J1-J2)))}
        try:
            fit=minimize(objective,start,method='SLSQP',jac=objective_jac,bounds=[(-35,35)]*12,
                constraints=[{'type':'ineq','fun':evaluate,'jac':jac}],options={'maxiter':50,'ftol':1e-9,'disp':False})
            # A successful numerical exit alone does not establish exact feasibility.
            final_force,_=independent(fit.x)
            if np.all(final_force<=180.) and np.all(abs(fit.x)<=35):
                candidate=(objective(fit.x),fit.x.copy())
                if best is None or candidate[0]<best[0]:best=candidate
            status={'success':bool(fit.success),'message':str(fit.message),'iterations':int(fit.nit),'last_candidate_force_n':final_force.tolist()}
        except Budget:
            status={'success':False,'message':'wall_budget_exhausted'}
        selected=None
        if best is not None:
            command=best[1];f,details=independent(command)
            selected={'command_nm':command.tolist(),'delta_from_lqr_nm':(command-start).tolist(),'squared_distance':2*best[0],
                'all_pre_post_force_le180':bool(np.all(f<=180.)),'torque_limit_pass':bool(np.all(abs(command)<=35)),
                'fresh_replay':details}
        hashes={str(source):hashlib.sha256(source.read_bytes()).hexdigest(),str(pathlib.Path(__file__).resolve()):hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()}
        for name,expected in r['input_hashes'].items():
            p=pathlib.Path(name)
            if p.suffix.lower() in ('.xml','.obj','.stl','.png'):
                if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise ValueError('asset hash mismatch')
                hashes[str(p)]=expected
        state=np.empty(mujoco.mj_stateSize(m,r['integration_state_spec']));mujoco.mj_getState(m,d,state,r['integration_state_spec'])
        output={'schema':'single-step-force-feasibility-oracle-v1','scope':'one exact predecessor integration state only; no feedback rerun or robustness claim; failed search is unknown, not proof of infeasibility',
            'step':k,'time':float(d.time),'initial_integration_state':state.tolist(),'integration_state_spec':r['integration_state_spec'],
            'input_hashes':hashes,'mujoco_version':mujoco.__version__,'method':'SLSQP closest LQR command; explicit pre-forward and post-step-forward forces <=180; torque bounds +/-35; independent centered finite differences epsilon1e-4; unchanged model',
            'initial_command_nm':start.tolist(),'initial_physical':initial_physical,'constraint_jacobian_check':jac_check,
            'solver':status,'calls':calls,'elapsed_s':time.perf_counter()-started,'selected':selected,
            'verdict':'one_step_feasible_witness' if selected and selected['all_pre_post_force_le180'] and selected['torque_limit_pass'] else 'optimization_unknown'}
        with pathlib.Path(args.out).open('x') as stream:json.dump(output,stream,indent=2,allow_nan=False);stream.write('\n')
        print(json.dumps(output))


if __name__=='__main__':main()
