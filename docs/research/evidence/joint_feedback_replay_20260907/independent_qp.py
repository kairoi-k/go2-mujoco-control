#!/usr/bin/env python3
"""Independent SciPy/HiGHS feasibility and SLSQP QP oracle on exported matrices."""
import argparse,hashlib,json,pathlib
import numpy as np, scipy, scipy.linalg as la
from scipy.optimize import linprog, minimize, LinearConstraint
p=argparse.ArgumentParser();p.add_argument('qp');p.add_argument('--out',required=True);a=p.parse_args()
path=pathlib.Path(a.qp);tokens=iter(path.read_text().split());q={}
while True:
 try:name=next(tokens)
 except StopIteration:break
 rows,cols=int(next(tokens)),int(next(tokens));q[name]=np.array([float(next(tokens)) for _ in range(rows*cols)]).reshape(rows,cols)
H,g,A,b,E,d=[q[k] for k in ['H','g','Aineq','bineq','Aeq','beq']];g=g.ravel();b=b.ravel();d=d.ravel()
# Equality elimination independently via SciPy SVD, then Hessian whitening.
x0=la.lstsq(E,d)[0];Z=la.null_space(E);Hr=Z.T@H@Z;gr=Z.T@(H@x0+g)
L=la.cholesky(Hr,lower=True);B=la.solve_triangular(L.T,np.eye(L.shape[0]),lower=False)
u=la.cho_solve((L,True),-gr);center=x0+Z@u;T=Z@B
C=A@T;rhs=b-A@center;scale=np.maximum(la.norm(C,axis=1),1e-12);C=C/scale[:,None];rhs=rhs/scale
lp=linprog(np.zeros(T.shape[1]),A_ub=C,b_ub=rhs,bounds=[(None,None)]*T.shape[1],method='highs')
def residual(x):return {'equality_inf':float(np.max(np.abs(E@x-d))),'inequality_max':float(max(0,np.max(A@x-b))),'objective':float(.5*x@H@x+g@x)}
report={'scipy_version':scipy.__version__,'matrix_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'linear_feasibility_success':bool(lp.success),'linear_status':lp.message,'reduced_h_condition':float(np.linalg.cond(Hr)),'original_iterate':residual(q['iterate'].ravel()) if q['iterate'].size else None}
if lp.success:
 witness=center+T@lp.x;report['independent_feasible_witness']=residual(witness);report['feasible_seed']=witness.tolist()
 yscale=max(1.0,la.norm(lp.x));
 sol=minimize(lambda y:.5*y@y,lp.x/yscale,jac=lambda y:y,constraints=LinearConstraint(C,-np.inf,rhs/yscale),method='SLSQP',options={'ftol':1e-14,'maxiter':2000})
 x=center+T@(sol.x*yscale);report.update(qp_success=bool(sol.success),qp_message=sol.message,qp_iterations=int(sol.nit),qp_residual=residual(x),solution=x.tolist())
pathlib.Path(a.out).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['solution','feasible_seed']},indent=2))
