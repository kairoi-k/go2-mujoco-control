#!/usr/bin/env python3
"""Fixed-matrix numerical task compatibility, never a traversal verdict."""
import argparse, json, hashlib
from pathlib import Path
import numpy as np
import scipy.linalg as la
p=argparse.ArgumentParser();p.add_argument('packet',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
if a.out.exists():raise SystemExit('refusing overwrite')
def read(path):
 it=iter(path.read_text().split());q={}
 for k in it:
  r,c=int(next(it)),int(next(it));q[k]=np.array([float(next(it)) for _ in range(r*c)]).reshape(r,c)
 return q
q=read(a.packet/'first_qp.txt');removed=read(a.packet/'ablation_qp.txt');soft=read(a.packet/'orientation_soft_qp.txt')
expected=q['H'].copy()
for j in range(6,18):expected[j,j]-=2*q['w_posture'][0,0]
assert np.array_equal(expected,removed['H'])
for k in q:
 if k!='H':assert np.array_equal(q[k],removed[k]),k
assert np.allclose(q['Aeq'][9:12,3:6],np.eye(3)*np.sqrt(40),atol=1e-12)
keep=[j for j in range(q['Aeq'].shape[0]) if not 9<=j<12]
for k in q:
 expected=q[k][keep] if k in ('Aeq','beq') else q[k]
 assert np.array_equal(expected,soft[k]),k
E=q['Aeq'];d=q['beq'].ravel();legs=[l for l in range(4) if not q['contact'][l,0]]
S=np.vstack([np.pad(q[f'J{l}'],((0,0),(0,E.shape[1]-18))) for l in legs])
target=np.concatenate([(q[f'task{l}']-q[f'bias{l}']).ravel() for l in legs]);ranks=[]
for cutoff in (1e-8,1e-10,1e-12):
 Z=la.null_space(E,rcond=cutoff);x0=la.lstsq(E,d,cond=cutoff)[0]
 y=la.lstsq(S@Z,target-S@x0,cond=cutoff)[0];x=x0+Z@y
 ranks.append(dict(cutoff=cutoff,singular_values=la.svdvals(S@Z).tolist(),
  swing_residual_norm=float(la.norm(S@x-target)),
  equality_residual=float(np.max(np.abs(E@x-d))),
  inequality_signed_max=float(np.max(q['Aineq']@x-q['bineq'].ravel()))))
results={}
for label in ('baseline','ablation','orientation_soft'):
 report=json.loads((a.packet/f'{label}_oracle.json').read_text());x=np.array(report['solution'])
 results[label]=dict(angular_acceleration=x[3:6].tolist(),max_abs_qdd=float(np.max(abs(x[:18]))),
  foot_task_errors=[float(la.norm(q[f'J{l}']@x[:18]+(q[f'bias{l}']-q[f'task{l}']).ravel())) for l in range(4)],
  oracle_success=report['qp_success'],residual=report['qp_residual'])
out=dict(b1_claim=False,scope='conditional numerical task coupling; not exact symbolic proof or physical rollout',
 matrix_sha256=hashlib.sha256((a.packet/'first_qp.txt').read_bytes()).hexdigest(),
 variants_verified=True,rank_sensitivity=ranks,results=results)
a.out.write_text(json.dumps(out,indent=2)+'\n')
print('saved',a.out)
