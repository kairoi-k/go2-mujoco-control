#!/usr/bin/env python3
"""Frozen actual-state aerial landing-target QP, not a trajectory certificate."""
import argparse, pathlib, json, re, hashlib
import numpy as np
import mujoco
from scipy.optimize import linprog, minimize, Bounds, LinearConstraint
p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--boundaries',required=True);p.add_argument('--scene',required=True);p.add_argument('--out',required=True);a=p.parse_args()
files=[pathlib.Path(a.source),pathlib.Path(a.boundaries),pathlib.Path(a.scene),pathlib.Path(__file__)]
t=files[0].read_text().split();off=9
pos=np.array(t[off:off+3],float);off+=3;quat=np.array(t[off:off+4],float);off+=4
vel=np.array(t[off:off+3],float);off+=3;omega=np.array(t[off:off+3],float);off+=3
q=np.array(t[off:off+12],float);off+=12;dq=np.array(t[off:off+12],float)
m=mujoco.MjModel.from_xml_path(a.scene);d=mujoco.MjData(m)
assert m.nv==18 and m.nu==12 and np.max(abs(m.actuator_gear[:,0]-1))<1e-12
ids=m.actuator_trnid[:,0];qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
d.qpos[:3]=pos;d.qpos[3:7]=quat/np.linalg.norm(quat);d.qpos[qa]=q;d.qvel[:3]=vel;d.qvel[3:6]=omega;d.qvel[va]=dq
mujoco.mj_forward(m,d)
assert d.ncon==0, "oracle requires contact-free robot model"
d.qacc[:]=0;mujoco.mj_inverse(m,d);model_bias=d.qfrc_inverse.copy()
M=np.zeros((18,18));mujoco.mj_fullM(m,M,d.qM);S=np.zeros((18,12));S[va,np.arange(12)]=1
A=np.linalg.solve(M,S);b=np.linalg.solve(M,-model_bias)
lines={int(re.search(r'leg=(\d+)',l)[1]):l for l in files[1].read_text().splitlines() if l.startswith('SwingAudit')}
C=[];c=[];old=[];max_match=0
for leg,name in enumerate(['FR','FL','RR','RL']):
 l=lines[leg];T=float(re.search(r'remaining_s=(\S+)',l)[1])
 v=lambda key,end:np.array([float(x) for x in l.split(key+'=')[1].split(' '+end+'=')[0].split()])
 p0=v('p0','v0');v0=v('v0','p1');p1=v('p1','hermite_a0');old.extend(p1)
 gid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name);assert gid>=0
 J=np.zeros((3,18));R=np.zeros((3,18));mujoco.mj_jacGeom(m,d,J,R,gid)
 max_match=max(max_match,float(np.max(abs(d.geom_xpos[gid]-p0))),float(np.max(abs(J@d.qvel-v0))))
 jac=[]
 for sign in (-1,1):
  dd=mujoco.MjData(m);dd.qpos[:]=d.qpos;dd.qvel[:]=d.qvel;mujoco.mj_integratePos(m,dd.qpos,d.qvel,sign*1e-6);mujoco.mj_forward(m,dd)
  jj=np.zeros((3,18));mujoco.mj_jacGeom(m,dd,jj,R,gid);jac.append(jj)
 bias=(jac[1]-jac[0])@d.qvel/(2e-6)
 # Cubic boundary identity, terminal v=0; all legs share the SAME qacc/tau.
 C.extend((T*T/6)*(J@A));c.extend(p0+(T*T/6)*(J@b+bias)+2*T*v0/3)
assert max_match<1e-6
C=np.array(C);c=np.array(c);old=np.array(old);z=np.array([2,5,8,11]);xy=np.array([0,1,3,4,6,7,9,10])
E=C[z];f=old[z]-c[z]
# Diagnostic local planar target box only, NOT a terrain coverage certificate.
G=np.vstack([C[xy],-C[xy]]);h=np.r_[old[xy]+.05-c[xy],-(old[xy]-.05-c[xy])]
lp=linprog(np.zeros(12),A_ub=G,b_ub=h,A_eq=E,b_eq=f,bounds=[(-35,35)]*12,method='highs')
report={'scope':'same-state aerial joint body/foot target oracle; not horizon or terrain certification','mujoco_version':mujoco.__version__,'source_model_match':max_match,'lp_status':int(lp.status),'lp_message':lp.message,'hashes':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}}
if lp.success:
 D=np.vstack([C/.03,A[3:6]/40,np.eye(12)*.01/35]);e=np.r_[(old-c)/.03,-b[3:6]/40,np.zeros(12)]
 H=D.T@D;g=-D.T@e
 result=minimize(lambda x:.5*x@H@x+g@x,lp.x,jac=lambda x:H@x+g,bounds=Bounds(-35,35),constraints=[LinearConstraint(E,f,f),LinearConstraint(G,-np.inf,h)],method='SLSQP',options={'ftol':1e-12,'maxiter':500})
 tau=result.x;acc=A@tau+b;target=C@tau+c
 residual=max(float(np.max(abs(E@tau-f))),float(np.max(np.maximum(G@tau-h,0))),float(np.max(np.maximum(abs(tau)-35,0))))
 assert result.success and residual<1e-7
 report.update({'qp_success':bool(result.success),'constraint_residual':residual,'full_dynamics_residual':float(np.max(abs(M@acc+model_bias-S@tau))),'torque_peak_nm':float(np.max(abs(tau))),'body_acceleration':acc[3:6].tolist(),'targets':target.reshape(4,3).tolist(),'old_targets':old.reshape(4,3).tolist(),'torque':tau.tolist()})
with pathlib.Path(a.out).open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items() if k!='hashes'},indent=2))
