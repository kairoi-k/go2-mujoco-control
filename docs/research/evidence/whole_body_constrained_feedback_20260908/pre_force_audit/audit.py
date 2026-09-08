import copy,fcntl,hashlib,json,pathlib
import mujoco,numpy as np
root=pathlib.Path.cwd();out=pathlib.Path(__file__).resolve().parent;reports=[]
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 for n in (1,2,3):
  path=root/f'example/cpp/experiments/_runs/whole_body_constrained_feedback_20260908_{n:04d}/result.json';r=json.loads(path.read_text());m=mujoco.MjModel.from_xml_path(r['scene']);d=mujoco.MjData(m);mujoco.mj_setState(m,d,np.array(r['initial_integration_state']),r['integration_state_spec']);gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name) for name in ('FR','FL','RR','RL')]
  def forces(data):
   result=np.zeros(4)
   for i in range(data.ncon):
    c=data.contact[i];f=np.zeros(6);mujoco.mj_contactForce(m,data,i,f)
    for leg,gid in enumerate(gids):
     if gid in (c.geom1,c.geom2):result[leg]+=f[0]
   return result
  err=0.;maximum_pre=0.;maximum_post=0.;state_error=0.;violations=[]
  assert len(r['controls'])==len(r['feedback']['constraint_checks'])==len(r['rows'])
  for k,(u,check,row) in enumerate(zip(r['controls'],r['feedback']['constraint_checks'],r['rows'])):
   d.ctrl[:]=u;pre=copy.copy(d);mujoco.mj_forward(m,pre);fp=forces(pre)
   mujoco.mj_step(m,d);post=copy.copy(d);mujoco.mj_forward(m,post);fq=forces(post)
   err=max(err,float(np.max(abs(fp-check['pre_force_n']))),float(np.max(abs(fq-check['post_force_n']))));maximum_pre=max(maximum_pre,float(max(fp)));maximum_post=max(maximum_post,float(max(fq)));state_error=max(state_error,float(np.max(abs(d.qpos-row['qpos']))),float(np.max(abs(d.qvel-row['qvel']))))
   if max(max(fp),max(fq))>180.:violations.append(k)
  reports.append({'attempt':n,'steps':len(r['controls']),'metadata_force_error_n':err,'state_error':state_error,'pre_force_max_n':maximum_pre,'post_force_max_n':maximum_post,'strict180n_violation_steps':violations,'result_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'source_sha':r['source_sha']})
report={'schema':'independent-pre-post-force-audit-v1','script_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'mujoco_version':mujoco.__version__,'scope':'independent full-state saved-control pre/post force replay; not live/B1','reports':reports}
with (out/'report.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps(report,indent=2))
