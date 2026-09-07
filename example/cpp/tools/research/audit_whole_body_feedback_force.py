"""One-step saved-state force attribution; no policy, gain or model edits."""
import argparse
import copy
import fcntl
import hashlib
import json
import pathlib
import mujoco
import numpy as np

LEGS = ('FR', 'FL', 'RR', 'RL')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('perturbed', 'reference', 'nominal', 'out'):
        parser.add_argument('--'+name, required=True)
    args = parser.parse_args()
    paths = {name: pathlib.Path(getattr(args,name)).resolve() for name in ('perturbed','reference','nominal')}
    reports = {name: json.loads(path.read_text()) for name,path in paths.items()}
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        scene = pathlib.Path(reports['perturbed']['scene'])
        if not scene.is_absolute(): scene = paths['perturbed'].parent/scene
        scene = scene.resolve()
        model = mujoco.MjModel.from_xml_path(str(scene))
        gids = [mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in LEGS]
        def observation(raw):
            d = copy.copy(raw)
            mujoco.mj_forward(model,d)
            force = np.zeros(4)
            contacts = []
            for i,c in enumerate(d.contact):
                f = np.zeros(6)
                mujoco.mj_contactForce(model,d,i,f)
                normal = np.asarray(c.frame[:3])
                item = {'id':i,'geom1':int(c.geom1),'geom2':int(c.geom2),
                    'position_world':c.pos.tolist(),'normal_geom1_to_geom2_world':normal.tolist(),
                    'distance_m':float(c.dist),'force_contact_frame':f.tolist(),'feet':[]}
                for leg,gid in enumerate(gids):
                    if gid in (c.geom1,c.geom2):
                        force[leg] += f[0]
                        jp,jr = np.zeros((3,model.nv)),np.zeros((3,model.nv))
                        mujoco.mj_jac(model,d,jp,jr,c.pos,int(model.geom_bodyid[gid]))
                        other = c.geom1 if gid==c.geom2 else c.geom2
                        op,orr = np.zeros_like(jp),np.zeros_like(jr)
                        mujoco.mj_jac(model,d,op,orr,c.pos,int(model.geom_bodyid[other]))
                        outward = normal if gid==c.geom2 else -normal
                        item['feet'].append({'leg':LEGS[leg],'normal_separation_speed_mps':float(outward@((jp-op)@d.qvel))})
                contacts.append(item)
            feet = []
            for leg,gid in enumerate(gids):
                jp,jr = np.zeros((3,model.nv)),np.zeros((3,model.nv))
                mujoco.mj_jacGeom(model,d,jp,jr,gid)
                feet.append({'leg':LEGS[leg],'normal_force_n':float(force[leg]),
                    'center_position_world':d.geom_xpos[gid].tolist(),
                    'center_velocity_world':(jp@d.qvel).tolist()})
            return {'time':float(d.time),'normal_forces_n':force.tolist(),'feet':feet,'contacts':contacts,
                'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist()}
        saved = reports['perturbed']['rows']
        violating = [i for i,row in enumerate(saved) if max(row['normal_forces'])>180.]
        if not violating: raise ValueError('no >180N row')
        first = violating[0]
        maximum = max(range(len(saved)),key=lambda i:max(saved[i]['normal_forces']))
        wanted = sorted({first,maximum})
        def replay(report):
            d = mujoco.MjData(model)
            mujoco.mj_setState(model,d,np.asarray(report['initial_integration_state']),report['integration_state_spec'])
            witnesses = {}
            errors = {'qpos':0.,'qvel':0.,'force_n':0.,'time_s':0.}
            for k,(cmd,row) in enumerate(zip(report['controls'],report['rows'])):
                if k in wanted: witnesses[k] = copy.copy(d)
                d.ctrl[:] = cmd
                mujoco.mj_step(model,d)
                obs = observation(d)
                for key,actual,expected in [('qpos',d.qpos,row['qpos']),('qvel',d.qvel,row['qvel']),
                    ('force_n',obs['normal_forces_n'],row['normal_forces']),('time_s',d.time,row['time'])]:
                    errors[key] = max(errors[key],float(np.max(abs(np.asarray(actual)-expected))))
            if max(errors.values())>1e-8: raise ValueError('saved replay mismatch '+str(errors))
            return witnesses,errors
        disturbed,dist_errors = replay(reports['perturbed'])
        reference,ref_errors = replay(reports['reference'])
        N = len(reports['nominal']['controls'])
        rows = []
        for k in wanted:
            nominal_cmd = np.asarray(reports['nominal']['controls'][k%N])
            actual_cmd = np.asarray(reports['perturbed']['controls'][k])
            reference_cmd = np.asarray(reports['reference']['controls'][k])
            tests = []
            for label,state,command in [('perturbed_actual',disturbed[k],actual_cmd),
                ('perturbed_nominal_torque',disturbed[k],nominal_cmd),
                ('reference_actual',reference[k],reference_cmd),
                ('reference_nominal_torque',reference[k],nominal_cmd),
                ('perturbed_reference_command',disturbed[k],reference_cmd)]:
                data = copy.copy(state)
                data.ctrl[:] = command
                pre = observation(data)
                mujoco.mj_step(model,data)
                tests.append({'case':label,'pre_command_observation':pre,'post_step_observation':observation(data)})
            rows.append({'step':k,'phase_step':k%N,'first_violation':k==first,'maximum':k==maximum,
                'correction_actual_minus_nominal_nm':(actual_cmd-nominal_cmd).tolist(),
                'correction_reference_minus_nominal_nm':(reference_cmd-nominal_cmd).tolist(),
                'pre_state_difference_qvel':(disturbed[k].qvel-reference[k].qvel).tolist(),
                'cases':tests})
        hashes = {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in list(paths.values())+[pathlib.Path(__file__).resolve()]}
        # Bind and verify every available model asset already bound by the source.
        for key,value in reports['perturbed']['input_hashes'].items():
            path = pathlib.Path(key)
            if not path.is_absolute(): path = paths['perturbed'].parent/path
            if path.suffix.lower() in ('.xml','.obj','.stl','.png'):
                if hashlib.sha256(path.read_bytes()).hexdigest()!=value: raise ValueError('model asset hash mismatch')
                hashes[str(path.resolve())] = value
        result = {'schema':'whole-body-feedback-force-attribution-v1',
            'scope':'one-step counterfactual from exact saved predecessor integration states; no state reset, policy change, model change or stability claim',
            'force_observation':'post-step mj_forward identical to feedback report; positive normal_separation_speed is separating from opposing geometry',
            'input_hashes':hashes,'mujoco_version':mujoco.__version__,
            'first_violation_step':first,'maximum_step':maximum,'violating_sample_count':len(violating),
            'replay_errors':{'perturbed':dist_errors,'reference':ref_errors},'witnesses':rows}
        with pathlib.Path(args.out).open('x') as stream:
            json.dump(result,stream,indent=2,allow_nan=False)
            stream.write('\n')
        print(json.dumps({'out':args.out,'replay_errors':result['replay_errors'],
            'witnesses':[{'step':r['step'],'forces':{c['case']:c['post_step_observation']['normal_forces_n'] for c in r['cases']},
             'delta_tau':r['correction_actual_minus_nominal_nm']} for r in rows]}))


if __name__=='__main__':
    main()
