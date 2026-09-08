import fcntl,json,mujoco,numpy as np
results=[]
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 for x in [0.,.002,.01,.025]:
  for surface in ['plane','cells']:
   terrain='<geom type="plane" size="1 1 .1"/>' if surface=='plane' else '<geom type="box" pos="-.025 0 -.15" size=".025 .1 .15"/><geom type="box" pos=".025 0 -.15" size=".025 .1 .15"/>'
   xml=f'<mujoco><option cone="elliptic" impratio="100"/><worldbody>{terrain}<body pos="{x} 0 .021"><freejoint/><geom name="foot" type="sphere" size=".022" mass="1" margin=".001" priority="1" condim="6" friction=".8 .02 .01"/></body></worldbody></mujoco>'
   m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);forces=[]
   for k in range(d.ncon):
    cf=np.zeros(6);mujoco.mj_contactForce(m,d,k,cf);forces.append(cf[:3].tolist())
   results.append({'x':x,'surface':surface,'ncon':d.ncon,'qacc':d.qacc[:3].tolist(),'forces':forces})
print(json.dumps({'mujoco':mujoco.__version__,'scope':'independent one-sphere cell-seam counterexample, not full Go2 traversal','results':results},indent=2))
