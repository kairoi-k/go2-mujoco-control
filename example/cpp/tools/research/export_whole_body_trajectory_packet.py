"""Export a hash-bound research WholeBodyTrajectoryV1; never motor authority.

The ASCII .packet is readable with C++ operator>> and requires its SHA-bound
JSON manifest. Format: magic; manifest_sha256; dimensions N nq nv nu nx;
coverage start_ns end_ns dt_ns; authority 1 0 0 (research, production, observed);
typed metadata records (identity, sources/source, model_files/model_file, observation, command_authority,
period, limits/limit, ordering/actuator, certificates/certificate/failflag,
derivatives/derivative); then N lines `sample k t_ns qpos[nq] qvel[nv] tau[nu] K[nu*nx]` (row-major);
last `terminal t_ns qpos[nq] qvel[nv]`, then `end` and EOF. Commands are ZOH
on [t_k,t_k+dt); no interpolation, rebase, looping or extrapolation is granted.
"""
import argparse
import hashlib
import json
import pathlib
import re
import mujoco
import numpy as np


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def finite_tree(value):
    if isinstance(value, dict):
        for item in value.values(): finite_tree(item)
    elif isinstance(value, list):
        for item in value: finite_tree(item)
    elif isinstance(value, float) and not np.isfinite(value):
        raise ValueError('nonfinite input')


def load(path):
    result=json.loads(pathlib.Path(path).read_text())
    finite_tree(result)
    return result


def resolve(source, name):
    path=pathlib.Path(name)
    return (path if path.is_absolute() else source.parent/path).resolve()


def verify_inputs(result, source):
    bound={}
    for name, expected in result['input_hashes'].items():
        path=resolve(source,name)
        if not re.fullmatch('[0-9a-f]{64}',expected) or sha(path)!=expected:
            raise ValueError('input hash mismatch '+str(path))
        if path in bound and bound[path]!=expected: raise ValueError('conflicting hash aliases')
        bound[path]=expected
    return bound


def certificate(path, result_path):
    cert=load(path)
    if cert['schema']!='whole-body-shooting-certificate-v1' or cert['result_sha256']!=sha(result_path):
        raise ValueError('certificate/result identity mismatch')
    result=load(result_path)
    checks=cert['checks']
    required=set(('state_reproduction time_reproduction force_reproduction torque_reproduction '
                  'dynamics_balance friction_cone unilateral torque_bound normal_force_bound '
                  'joint_speed_bound joint_position_bound attitude_bound base_height_bound '
                  'no_nonfoot_contact_force full_period terminal_body_position terminal_orientation '
                  'terminal_base_velocity terminal_body_omega terminal_joint_position terminal_joint_velocity').split())
    if set(checks)!=required: raise ValueError('certificate check coverage mismatch')
    if (not checks or any(type(v) is not bool for v in checks.values())
            or type(cert['diagnostic_success']) is not bool
            or cert['diagnostic_success'] != all(checks.values())):
        raise ValueError('certificate verdict/checks mismatch')
    for key in ('source_sha','mujoco_version'):
        if cert.get(key)!=result.get(key): raise ValueError('certificate source identity mismatch '+key)
    if cert.get('steps')!=len(result['rows']): raise ValueError('certificate step coverage mismatch')
    for key in ('state_reproduction','time_reproduction','force_reproduction','torque_reproduction'):
        if cert['checks'].get(key) is not True: raise ValueError('unvalidated saved nominal/replay')
    return {'path':str(path),'sha256':sha(path),'result_sha256':cert['result_sha256'],
        'diagnostic_success':cert['diagnostic_success'],'checks':cert['checks'],
        'failflags':[key for key,value in cert['checks'].items() if not value],
        'maxima':cert['maxima'],'scope':cert['scope']}


def numeric_array(value,shape,name):
    array=np.asarray(value,dtype=float)
    if array.shape!=shape or not np.all(np.isfinite(array)): raise ValueError('invalid '+name)
    return array


def validate_observation(model,initial,nominal,tokens):
    # Reconstruct the recorded estimated state using the source actuator order.
    # The producer normalized its quaternion; no other state adjustment is valid.
    if len(tokens)<46 or tokens[0]!='joint-shadow-snapshot-v2':
        raise ValueError('source observation schema mismatch')
    header=numeric_array(tokens[3:9],(6,),'source header')
    expected=np.asarray([initial.time,nominal['initial_phase'],nominal['period_s'],
                         nominal['duty'],nominal['command_vx']])
    if np.max(abs(header[:5]-expected))>1e-9:
        raise ValueError('source observation calendar/command mismatch')
    values=numeric_array(tokens[9:46],(37,),'source observation state')
    norm=np.linalg.norm(values[3:7])
    if norm<1e-12 or abs(norm-1)>1e-6: raise ValueError('source observation quaternion invalid')
    observed=mujoco.MjData(model)
    observed.qpos[:3]=values[:3];observed.qpos[3:7]=values[3:7]/norm
    observed.qvel[:6]=values[7:13]
    joints=model.actuator_trnid[:,0]
    observed.qpos[model.jnt_qposadr[joints]]=values[13:25]
    observed.qvel[model.jnt_dofadr[joints]]=values[25:37]
    if (np.max(abs(observed.qpos-initial.qpos))>1e-12
            or np.max(abs(observed.qvel-initial.qvel))>1e-12):
        raise ValueError('source observation initial q/v mismatch')
    return {'qpos_qvel_verified':True,'tolerance':1e-12,
            'method':'source snapshot actuator-order reconstruction; quaternion normalization only'}

def validate_feedback_law(model,nominal,feedback,qrefs,vrefs,gains):
    rows=feedback['rows'];controls=numeric_array(feedback['controls'],(len(rows),model.nu),'feedback controls')
    N=len(qrefs); info=feedback['feedback']
    if feedback.get('times_are_absolute') is not True: raise ValueError('feedback absolute time required')
    if nominal['constraints']['torque_limit_nm']!=35: raise ValueError('unsupported feedback clipping bound')
    if type(info['periods']) is not int: raise ValueError('feedback period count invalid')
    if info.get('initial_vy_perturbation_mps')!=0 or info.get('initial_roll_perturbation_rad')!=0:
        raise ValueError('gain evidence must be nominal unperturbed replay')
    if len(rows)!=N*info['periods'] or info['periods']<1: raise ValueError('feedback coverage mismatch')
    if nominal['initial_integration_state']!=feedback['initial_integration_state']:
        raise ValueError('nominal/closed-loop initial state mismatch')
    d=mujoco.MjData(model)
    mujoco.mj_setState(model,d,np.asarray(nominal['initial_integration_state']),nominal['integration_state_spec'])
    previous_q,previous_v=d.qpos.copy(),d.qvel.copy();max_error=0.
    for step,row in enumerate(rows):
        k,cycle=step%N,step//N
        qref=qrefs[k].copy();qref[0]+=cycle*nominal['command_vx']*nominal['period_s']
        error=np.empty(2*model.nv)
        mujoco.mj_differentiatePos(model,error[:model.nv],1.,qref,previous_q)
        error[model.nv:]=previous_v-vrefs[k]
        expected=np.clip(np.asarray(nominal['controls'][k])-gains[k]@error,-35.,35.)
        max_error=max(max_error,float(np.max(abs(expected-controls[step]))))
        previous_q=numeric_array(row['qpos'],(model.nq,),'feedback qpos')
        previous_v=numeric_array(row['qvel'],(model.nv,),'feedback qvel')
        if abs(row['time']-(d.time+(step+1)*model.opt.timestep))>1e-9:
            raise ValueError('feedback absolute-time mismatch')
        if step<N:
            if np.max(abs(previous_q-np.asarray(nominal['rows'][step]['qpos'])))>1e-9 or np.max(abs(previous_v-np.asarray(nominal['rows'][step]['qvel'])))>1e-9:
                raise ValueError('nominal first-cycle closed-loop mismatch')
    if max_error>1e-8: raise ValueError('recorded feedback law does not match nominal/K')
    return {'samples_checked':len(rows),'max_command_error_nm':max_error,'tolerance_nm':1e-8,
            'method':'state-log algebraic replay of clip(tau_nom-K*error); no new physics simulation'}


def build_packet(nominal_path,feedback_path,nominal_certificate,feedback_certificate):
    nominal_path=pathlib.Path(nominal_path).resolve();feedback_path=pathlib.Path(feedback_path).resolve()
    nominal,feedback=load(nominal_path),load(feedback_path)
    if nominal.get('schema')!='whole-body-shooting-v1' or feedback.get('schema')!='whole-body-shooting-v1' or 'feedback' in nominal:
        raise ValueError('require a nominal shooting result and distinct feedback evidence')
    nb,fb=verify_inputs(nominal,nominal_path),verify_inputs(feedback,feedback_path)
    if fb.get(nominal_path)!=sha(nominal_path): raise ValueError('feedback does not bind this nominal')
    for path in nb.keys() & fb.keys():
        if nb[path]!=fb[path]: raise ValueError('conflicting jointly bound input hashes')
    for result in (nominal,feedback):
        if not re.fullmatch('[0-9a-f]{40}',result['source_sha']): raise ValueError('invalid source SHA')
    scene=resolve(nominal_path,nominal['scene'])
    if scene not in nb or scene!=resolve(feedback_path,feedback['scene']): raise ValueError('model mismatch')
    for name in ('initial_phase','period_s','duty','command_vx','timestep_s','integration_state_spec','constraints'):
        if nominal[name]!=feedback[name]: raise ValueError('nominal/feedback field mismatch '+name)
    model=mujoco.MjModel.from_xml_path(str(scene))
    if (model.nq,model.nv,model.nu,model.na)!=(19,18,12,0): raise ValueError('research packet supports Go2 nq19 nv18 nu12 na0 only')
    if nominal['mujoco_version']!=mujoco.__version__ or feedback['mujoco_version']!=mujoco.__version__:
        raise ValueError('MuJoCo version mismatch')
    N=len(nominal['controls']);dt=float(model.opt.timestep)
    if N<1 or abs(dt-nominal['timestep_s'])>1e-12 or abs(N*dt-nominal['period_s'])>1e-10 or len(nominal['rows'])!=N:
        raise ValueError('nominal period coverage mismatch')
    if nominal.get('times_are_absolute') is not True: raise ValueError('absolute time required')
    controls=numeric_array(nominal['controls'],(N,12),'nominal tau')
    gains=numeric_array(feedback['feedback']['gains'],(N,12,36),'K')
    if np.max(abs(controls))>nominal['constraints']['torque_limit_nm']: raise ValueError('nominal torque exceeds bound')
    if nominal['integration_state_spec']!=int(mujoco.mjtState.mjSTATE_INTEGRATION):
        raise ValueError('complete integration state required')
    initial=mujoco.MjData(model)
    state=numeric_array(nominal['initial_integration_state'],(mujoco.mj_stateSize(model,nominal['integration_state_spec']),),'integration state')
    mujoco.mj_setState(model,initial,state,nominal['integration_state_spec'])
    qs=[initial.qpos.copy()];vs=[initial.qvel.copy()]
    for k,row in enumerate(nominal['rows']):
        q=numeric_array(row['qpos'],(19,),'nominal qpos');v=numeric_array(row['qvel'],(18,),'nominal qvel')
        if abs(np.linalg.norm(q[3:7])-1)>1e-9: raise ValueError('nonunit quaternion')
        if abs(row['time']-(initial.time+(k+1)*dt))>1e-9: raise ValueError('nominal absolute-time mismatch')
        qs.append(q);vs.append(v)
    qrefs,vrefs=np.asarray(qs[:-1]),np.asarray(vs[:-1])
    law=validate_feedback_law(model,nominal,feedback,qrefs,vrefs,gains)
    diagnostics=feedback['feedback']['transition_derivative_checks']
    if len(diagnostics)!=N: raise ValueError('missing local derivative coverage')
    summaries=[]
    for k,check in enumerate(diagnostics):
        selected=check.get('selected_epsilon')
        trials=[t for t in check.get('trials',[]) if t['epsilon']==selected]
        if not trials or selected<=0 or abs(check['time']-(initial.time+k*dt))>1e-9: raise ValueError('unresolved/time-mismatched derivatives')
        trial=trials[-1]
        if trial.get('finite') is not True or not trial.get('matrix_convergence') or not all(x['pass'] for x in trial['matrix_convergence']):
            raise ValueError('derivative convergence missing')
        if len(trial.get('directional_probes',[]))<3 or not all(p['pass'] and p['topology_same'] for p in trial['directional_probes']) or trial.get('axis_topology_crossings')!=[]:
            raise ValueError('derivative directional/topology check missing')
        summaries.append({'step':k,'time':check['time'],'epsilon':selected,
            'max_probe_relative_error':max(p['relative_error'] for p in trial['directional_probes'])})
    certs={'nominal':certificate(pathlib.Path(nominal_certificate).resolve(),nominal_path),
           'feedback':certificate(pathlib.Path(feedback_certificate).resolve(),feedback_path)}
    # Recursive model source closure uses the proven research XML resolver.
    import whole_body_cycle
    dependencies=whole_body_cycle.dependencies
    assets=dependencies(scene)
    if any(path not in nb or fb.get(path)!=nb[path] for path in assets): raise ValueError('recursive model closure is not jointly hash bound')
    embedded=feedback['feedback'].get('nominal_certificate')
    if embedded!=load(nominal_certificate): raise ValueError('feedback embedded nominal certificate mismatch')
    snapshots=[p for p in nb if p.name=='initial_source.txt']
    if len(snapshots)!=1: raise ValueError('ambiguous source observation')
    snapshot=snapshots[0];tokens=snapshot.read_text().split()
    observation_check=validate_observation(model,initial,nominal,tokens)
    ordering=[]
    for actuator in range(12):
        joint=int(model.actuator_trnid[actuator,0])
        ordering.append({'actuator_index':actuator,'actuator_name':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_ACTUATOR,actuator),
            'joint_id':joint,'joint_name':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,joint),
            'qpos_address':int(model.jnt_qposadr[joint]),'qvel_address':int(model.jnt_dofadr[joint]),
            'gear':model.actuator_gear[actuator].tolist()})
    start_ns=round(initial.time*1e9);dt_ns=round(dt*1e9);end_ns=start_ns+N*dt_ns
    if (start_ns<0 or dt_ns<=0 or end_ns>2**63-1
            or abs(start_ns/1e9-initial.time)>1e-12
            or abs(dt_ns/1e9-dt)>1e-12):
        raise ValueError('absolute nanosecond coverage not exactly representable')
    manifest={'schema':'WholeBodyTrajectoryV1.research-manifest.1','packet_format':'whole-body-trajectory-v1-research',
        'research_only':True,'production_ready':False,'execution_ready':False,'centroidal_certificate':None,
        'terrain':{'provenance':'privileged_flat_scene_xml','observed':False,'coverage':'flat research scene only','map_epoch':None},
        'observation':{'path':str(snapshot),'sha256':nb[snapshot],'schema':tokens[0],'absolute_time_ns':start_ns,
            'initial_state_check':observation_check,'raw_header_tokens':tokens[:9],'production_PlanningIdentity':None,'source_state_tick_verified':False},
        'command_authority':{'runtime_valid':False,'phase1_command_epoch':None,'phase1_remains_authority':True,
            'logged_command_vx_mps':nominal['command_vx'],'new_velocity_authority':False,'motor_write_authority':False},
        'coverage':{'start_ns_inclusive':start_ns,'end_ns_exclusive':end_ns,'dt_ns':dt_ns,'samples':N,
            'automatic_loop':False,'rebase_authorized':False,'extrapolation':False,'sample_policy':'zero_order_hold_pre_step'},
        'reference_origin':{'absolute_qpos0':qs[0].tolist(),'qvel0':vs[0].tolist(),'period_s':nominal['period_s'],
            'phase':nominal['initial_phase'],'duty':nominal['duty'],'nominal_wrap_error':feedback['feedback']['nominal_wrap_tangent_error'],
            'periodic_orbit_certified':False},
        'state_convention':{'qpos':'MuJoCo qpos: xyz world, quaternion wxyz world_from_body, model joint order',
            'qvel':'MuJoCo generalized velocity; free translation world, free angular body, model joint order',
            'error':'mujoco.mj_differentiatePos(qref,qactual,dt=1), followed by qvel_actual-qvel_ref',
            'K_order':'row-major actuator by [q tangent(nv),qvel(nv)]','law':'tau_nominal-K*error before external limits',
            'nominal_tau_source':'nominal.controls; feedback.controls are validation evidence only'},
        'model':{'scene':str(scene),'mujoco_version':mujoco.__version__,'nq':19,'nv':18,'nu':12,'na':0,
            'recursive_hashes':{str(p):nb[p] for p in sorted(assets)},'actuator_joint_order':ordering},
        'limits':nominal['constraints'],'local_derivatives':{'scope':'nominal local diagnostics only, not finite-radius stability','checks':summaries},
        'original_diagnostic_certificates':certs,'logged_feedback_law_check':law,
        'source_hashes':{'nominal':{'path':str(nominal_path),'sha256':sha(nominal_path),'source_sha':nominal['source_sha']},
            'gain_evidence':{'path':str(feedback_path),'sha256':sha(feedback_path),'source_sha':feedback['source_sha']},
            'exporter':{'path':str(pathlib.Path(__file__).resolve()),'sha256':sha(__file__)},
            'model_dependency_resolver':{'path':str(pathlib.Path(whole_body_cycle.__file__).resolve()),'sha256':sha(whole_body_cycle.__file__)}},
        'all_bound_input_hashes':{str(p):v for p,v in {**nb,**fb}.items()}}
    manifest['model']['identity_sha256']=hashlib.sha256(json.dumps(manifest['model']['recursive_hashes'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
    manifest['model']['identity_hash_encoding']='SHA256 of UTF8 sorted compact JSON recursive_hashes map'
    finite_tree(manifest)
    return manifest,qrefs,vrefs,controls,gains,np.asarray(qs[-1]),np.asarray(vs[-1])


def write_packet(output_dir,bundle):
    manifest,qrefs,vrefs,controls,gains,terminal_q,terminal_v=bundle
    output_dir=pathlib.Path(output_dir)
    output_dir.mkdir(parents=True,exist_ok=False)
    metadata=json.dumps(manifest,sort_keys=True,separators=(',',':'),allow_nan=False).encode()+b'\n'
    manifest_path=output_dir/'manifest.json';manifest_path.write_bytes(metadata)
    digest=hashlib.sha256(metadata).hexdigest();coverage=manifest['coverage'];N=len(qrefs)
    lines=['whole-body-trajectory-v1-research','manifest_sha256 '+digest,f'dimensions {N} 19 18 12 36',
        f"coverage {coverage['start_ns_inclusive']} {coverage['end_ns_exclusive']} {coverage['dt_ns']}",'authority 1 0 0']
    lines.append(f"identity {manifest['model']['identity_sha256']} {manifest['observation']['sha256']} 0 {coverage['start_ns_inclusive']} 0 0")
    lines.append('sources 2')
    for name in ('nominal','gain_evidence'):
        source=manifest['source_hashes'][name]
        lines.append(f"source {name} {source['source_sha']} {source['sha256']}")
    assets=manifest['model']['recursive_hashes']
    lines.append(f'model_files {len(assets)}')
    for path,asset_digest in sorted(assets.items()): lines.append('model_file '+json.dumps(path)+' '+asset_digest)
    lines.append(f"observation {manifest['observation']['sha256']} {coverage['start_ns_inclusive']}")
    lines.append(f"command_authority 0 0 {manifest['command_authority']['logged_command_vx_mps']:.17g}")
    origin=manifest['reference_origin']
    lines.append(f"period {origin['period_s']:.17g} {origin['phase']:.17g} {origin['duty']:.17g} 0")
    lines.append(f"limits {len(manifest['limits'])}")
    for name,value in sorted(manifest['limits'].items()): lines.append(f'limit {name} {value:.17g}')
    order=manifest['model']['actuator_joint_order']
    lines.append(f'ordering {len(order)}')
    for item in order:
        lines.append(f"actuator {item['actuator_index']} {json.dumps(item['actuator_name'])} {json.dumps(item['joint_name'])} {item['qpos_address']} {item['qvel_address']}")
    certs=manifest['original_diagnostic_certificates']
    lines.append(f'certificates {len(certs)}')
    for name,cert in sorted(certs.items()):
        lines.append(f"certificate {name} {cert['sha256']} {cert['result_sha256']} {int(cert['diagnostic_success'])} {len(cert['failflags'])}")
        for flag in cert['failflags']: lines.append('failflag '+flag)
    checks=manifest['local_derivatives']['checks']
    lines.append(f'derivatives {len(checks)}')
    for check in checks:
        lines.append(f"derivative {check['step']} {check['epsilon']:.17g} {check['max_probe_relative_error']:.17g}")
    def numbers(values):return ' '.join(format(float(v),'.17g') for v in values)
    for k in range(N):
        values=np.r_[qrefs[k],vrefs[k],controls[k],gains[k].ravel()]
        lines.append(f"sample {k} {coverage['start_ns_inclusive']+k*coverage['dt_ns']} "+numbers(values))
    lines.append(f"terminal {coverage['end_ns_exclusive']} "+numbers(np.r_[terminal_q,terminal_v]))
    lines.append('end');packet=output_dir/'trajectory.packet';packet.write_text('\n'.join(lines)+'\n',encoding='ascii',newline='\n')
    receipt={'schema':'whole-body-research-export-receipt-v1','manifest_sha256':digest,'packet_sha256':sha(packet),
        'packet_bytes':packet.stat().st_size,'samples':N,'production_ready':False}
    (output_dir/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('nominal','feedback','nominal-certificate','feedback-certificate','out-dir'):
        parser.add_argument('--'+name,required=True)
    args=parser.parse_args()
    bundle=build_packet(args.nominal,args.feedback,args.nominal_certificate,args.feedback_certificate)
    print(json.dumps(write_packet(args.out_dir,bundle)))


if __name__=='__main__':main()
