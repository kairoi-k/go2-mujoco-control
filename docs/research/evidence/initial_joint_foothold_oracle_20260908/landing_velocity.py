#!/usr/bin/env python3
"""Conditional aerial boundary oracle with normal landing velocity as a decision."""
import contextlib,io,runpy,pathlib,json,re,hashlib,os
import numpy as np
from scipy.optimize import linprog,minimize,LinearConstraint,Bounds
# Same arguments as oracle.py; --out writes its auxiliary fixed-v baseline.
with contextlib.redirect_stdout(io.StringIO()):
 v=runpy.run_path(str(pathlib.Path(__file__).with_name('oracle.py')),run_name='__main__')
C,c,old,A,b=[v[k] for k in ('C','c','old','A','b')]
phase_clearance=os.environ.get('ORACLE_PHASE_CLEARANCE')=='1'
if phase_clearance:
 period=float(v['t'][5]);duty=float(v['t'][6]);full_swing=period*(1-duty)
 c=c.copy()
 for leg,l in v['lines'].items():
  T=float(re.search(r'remaining_s=(\S+)',l)[1]);height=.03*(T/full_swing)**4
  c[3*leg+2]-=32*height/6 # T虏/6 times initial residual bump acceleration

D=np.zeros((12,4))
for leg,l in v['lines'].items():D[3*leg+2,leg]=float(re.search(r'remaining_s=(\S+)',l)[1])/3
# Cubic p1=p0+T虏*a0/6+2*T*v0/3+T*v1/3, v1 tangential=0.
P=np.c_[C,D];z=v['z'];xy=v['xy'];E=np.vstack([P[z],np.c_[A[3:6],np.zeros((3,4))]])
f=np.r_[old[z]-c[z],-b[3:6]]
G=np.vstack([P[xy],-P[xy]]);h=np.r_[old[xy]+.05-c[xy],-(old[xy]-.05-c[xy])]
# -2m/s is an explicit exploratory box, not a validated safe impact limit.
lower=np.r_[np.full(12,-35.),np.full(4,-2.)];upper=np.r_[np.full(12,35.),np.zeros(4)]
lp=linprog(np.zeros(16),A_ub=G,b_ub=h,A_eq=E,b_eq=f,bounds=list(zip(lower,upper)),method='highs')
out={'scope':'conditional instantaneous target oracle; impact and full trajectory unverified','phase_preserving_clearance':phase_clearance,'lp_status':int(lp.status),'lp_message':lp.message,'normal_terminal_velocity_bounds_mps':[-2,0],'script_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()}
if lp.success:
 W=np.vstack([P/.03,np.c_[np.zeros((4,12)),np.eye(4)],np.c_[np.eye(12)*.01/35,np.zeros((12,4))]])
 rhs=np.r_[(old-c)/.03,np.zeros(16)];H=W.T@W;g=-W.T@rhs
 r=minimize(lambda x:.5*x@H@x+g@x,lp.x,jac=lambda x:H@x+g,method='SLSQP',bounds=Bounds(lower,upper),constraints=[LinearConstraint(E,f,f),LinearConstraint(G,-np.inf,h)],options={'ftol':1e-12,'maxiter':500})
 x=r.x;res=max(float(np.max(abs(E@x-f))),float(np.max(np.maximum(G@x-h,0))),float(np.max(np.maximum(lower-x,0))),float(np.max(np.maximum(x-upper,0))))
 assert r.success and res<1e-7
 out.update({'qp_success':bool(r.success),'constraint_residual':res,'torque_peak_nm':float(np.max(abs(x[:12]))),'torque':x[:12].tolist(),'body_angular_acceleration':(A[3:6]@x[:12]+b[3:6]).tolist(),'terminal_velocity_normal_mps':x[12:].tolist(),'targets':(P@x+c).reshape(4,3).tolist()})
print(json.dumps(out,indent=2))
