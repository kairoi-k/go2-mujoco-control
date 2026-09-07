#!/usr/bin/env python3
"""Replay oracle inputs, compare fixed-target torque and exact body-hold LP."""
import runpy,sys,io,contextlib,json,pathlib
import numpy as np
from scipy.optimize import linprog
# Same CLI as oracle.py; --out is a NEW auxiliary oracle output file.
with contextlib.redirect_stdout(io.StringIO()):
 v=runpy.run_path(str(pathlib.Path(__file__).with_name('oracle.py')),run_name='__main__')
C,c,old,A,b=[v[k] for k in ('C','c','old','A','b')]
tau=np.linalg.solve(C,old-c)
r=linprog(np.zeros(12),A_ub=v['G'],b_ub=v['h'],A_eq=np.vstack([v['E'],A[3:6]]),b_eq=np.r_[old[v['z']]-c[v['z']],-b[3:6]],bounds=[(-35,35)]*12,method='highs')
out={'fixed_original_targets_torque_peak_nm':float(np.max(abs(tau))),'difference_from_cpp_nm':abs(float(np.max(abs(tau)))-312.0440488517426),'zero_body_angular_acceleration_lp_status':int(r.status),'zero_body_angular_acceleration_message':r.message}
# Report disagreement rather than hiding it.
if r.success:
 out['zero_body_hold_residual']=float(np.max(abs(A[3:6]@r.x+b[3:6])))
 out['zero_body_hold_targets']=(C@r.x+c).reshape(4,3).tolist()
print(json.dumps(out,indent=2))
