import pathlib,json,hashlib,copy
import mujoco,numpy as np
out=pathlib.Path(__file__).resolve().parent
run_dir=out.parent
read=lambda p:json.loads(pathlib.Path(p).read_text())
sha=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
run=read(run_dir/'run.json');source=read(run_dir/'manifest.json');inputs=read(run_dir/'input_manifest.json')
assert source['source_sha']=='54d750eb4b1730db9041abd384d126dbe1f7a028'
for name,h in source['files'].items():assert sha(run_dir/name)==h,(name,'hash mismatch')
packet_dir=pathlib.Path(inputs['packet_dir']);packet=read(packet_dir/'manifest.json')
assert sha(packet_dir/'manifest.json')==inputs['manifest_sha256']
assert sha(packet_dir/'trajectory.packet')==inputs['packet_sha256']
nompath=pathlib.Path(packet['source_hashes']['nominal']['path']);nom=read(nompath)
assert sha(nompath)==packet['source_hashes']['nominal']['sha256']
model=mujoco.MjModel.from_xml_path(packet['model']['scene']);data=mujoco.MjData(model)
spec=int(mujoco.mjtState.mjSTATE_INTEGRATION)
initial=np.array(run['initial_integration_state']);mujoco.mj_setState(model,data,initial,spec)
assert run['completed_steps']==run['target_steps']==len(run['rows'])==1400
q0=data.qpos.copy();v0=data.qvel.copy();initial_time=float(data.time)
rows=[]
for i,row in enumerate(run['rows']):
 assert row['tick']==i and len(row['forces'])==8
 mujoco.mj_setState(model,data,np.array(row['next_integration_state']),spec)
 rows.append({'time':float(data.time),'qpos':data.qpos.tolist(),'qvel':data.qvel.tolist(),'normal_forces':row['forces'][4:],'torque':row['tau']})
reference_q=q0.copy();reference_q[0]+=nom['command_vx']*len(rows)*model.opt.timestep
nominal_q=np.array(packet['reference_origin']['absolute_qpos0']);nominal_q[0]+=nom['command_vx']*len(rows)*model.opt.timestep
nominal_v=np.array(packet['reference_origin']['qvel0']);tangent=np.empty(model.nv)
mujoco.mj_differentiatePos(model,tangent,1.,nominal_q,data.qpos)
recovery={'scope':'terminal error against unperturbed nominal initial translated by command_vx*duration; separate from frozen V1 perturbed-initial reference','body_position_m':float(np.linalg.norm(data.qpos[:3]-nominal_q[:3])),'orientation_rad':float(np.linalg.norm(tangent[3:6])),'base_velocity_mps':float(np.linalg.norm(data.qvel[:3]-nominal_v[:3])),'body_omega_radps':float(np.linalg.norm(data.qvel[3:6]-nominal_v[3:6])),'joint_position_rad':float(np.max(abs(tangent[6:]))),'joint_velocity_radps':float(np.max(abs(data.qvel[6:]-nominal_v[6:]))),'initial_vy_delta':inputs['initial_vy_delta'],'terminal_vy_actual':float(data.qvel[1]),'terminal_vy_nominal':float(nominal_v[1])}
hashes=dict(packet['model']['recursive_hashes'])
for p in [run_dir/'run.json',run_dir/'manifest.json',run_dir/'input_manifest.json',packet_dir/'manifest.json',packet_dir/'trajectory.packet',nompath,pathlib.Path(__file__),out/'verify_whole_body_shooting.py',out/'analyze_shooting_contact_cycles.py']+[run_dir/x for x in source['sources']]:hashes[str(p)]=sha(p)
for path,h in hashes.items():assert sha(path)==h
result={k:copy.deepcopy(nom[k]) for k in ['schema','mujoco_version','times_are_absolute','timestep_s','period_s','initial_phase','duty','leg_offsets','command_vx','constraints']}
result.update(scope='converted actual initialized rolling-horizon run; original perturbed integration state and actual controls preserved; not liveRT/B1',source_sha=source['source_sha'],scene=packet['model']['scene'],input_hashes=hashes,integration_state_spec=spec,initial_integration_state=run['initial_integration_state'],controls=[r['tau'] for r in run['rows']],rows=rows,terminal_reference={'qpos':reference_q.tolist(),'qvel':v0.tolist()},conversion={'source_run_sha256':sha(run_dir/'run.json'),'method':'mj_setState only to decode original fullstate; no stepping or interpolation; normal_forces use original forward-poststep four entries','terminal_scope':'perturbed initial translated by command_vx*20 periods','initial_time':initial_time},nominal_recovery=recovery)
(out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
(out/'nominal_recovery.json').write_text(json.dumps(recovery,indent=2)+'\n')
(out/'source_hashes.json').write_text(json.dumps({'source_sha':source['source_sha'],'files':hashes,'result_sha256':sha(out/'result.json')},indent=2)+'\n')
print('Converted',len(rows),'rows, unchanged source',source['source_sha'])
