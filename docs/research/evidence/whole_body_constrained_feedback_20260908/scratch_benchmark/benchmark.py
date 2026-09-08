"""Locked same-state copy/restore helper equivalence and five-pair timing."""
import fcntl,hashlib,importlib.util,json,pathlib,time
import mujoco,numpy as np
base=pathlib.Path(__file__).resolve().parent
root=pathlib.Path.cwd()
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
old_path=base/'old_helper.py';new_path=root/'example/cpp/tools/research/constrained_whole_body_feedback.py'
old=load('old_helper',old_path);new=load('new_helper',new_path)
source=root/'example/cpp/experiments/_runs/whole_body_feedback_20260908_0003/result.json';r=json.loads(source.read_text())
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 model=mujoco.MjModel.from_xml_path(r['scene']);data=mujoco.MjData(model);spec=r['integration_state_spec'];mujoco.mj_setState(model,data,np.array(r['initial_integration_state']),spec)
 for cmd in r['controls'][:7]:data.ctrl[:]=cmd;mujoco.mj_step(model,data)
 initial=np.empty(mujoco.mj_stateSize(model,spec));mujoco.mj_getState(model,data,initial,spec)
 desired=np.array(r['controls'][7]);gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
 results={}
 for kind,target in [('failed_predecessor',desired),('feasible_same_state',None)]:
  if target is None:target=chosen
  entries=[]
  for repeat in range(5):
   pair={}
   for label,helper in [('old',old),('new',new)]:
    start=time.perf_counter();control,metadata=helper.constrained_tracking(model,data,target,gids);elapsed=time.perf_counter()-start
    pair[label]={'elapsed_s':elapsed,'control':control.tolist(),'metadata':metadata}
    after=np.empty_like(initial);mujoco.mj_getState(model,data,after,spec);np.testing.assert_array_equal(initial,after)
   chosen=np.array(pair['old']['control'])
   pair['max_control_delta']=float(np.max(abs(np.array(pair['old']['control'])-np.array(pair['new']['control']))))
   pair['max_force_delta']=float(np.max(abs(np.r_[pair['old']['metadata']['pre_force_n'],pair['old']['metadata']['post_force_n']]-np.r_[pair['new']['metadata']['pre_force_n'],pair['new']['metadata']['post_force_n']])))
   pair['calls_identical']=pair['old']['metadata']['calls']==pair['new']['metadata']['calls'];pair['status_identical']=pair['old']['metadata']['status']==pair['new']['metadata']['status'];entries.append(pair)
  results[kind]={'pairs':entries,'timing_s':{label:dict(zip(['min','median','max'],[float(min(values)),float(np.median(values)),float(max(values))])) for label in ('old','new') for values in [[p[label]['elapsed_s'] for p in entries]]}}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
report={'scope':'same-state execution optimization only; five paired observations are not realtime certification','original_source_sha':'714bf3','inputs':{str(p):sha(p) for p in [source,old_path,new_path,pathlib.Path(__file__)]},'initial_integration_state':initial.tolist(),'desired':desired.tolist(),'results':results}
with open(base/'report.json','x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:{'timing_s':v['timing_s'],'max_control_delta':max(p['max_control_delta'] for p in v['pairs']),'max_force_delta':max(p['max_force_delta'] for p in v['pairs']),'calls_identical':all(p['calls_identical'] for p in v['pairs']),'status_identical':all(p['status_identical'] for p in v['pairs'])} for k,v in results.items()},indent=2))
