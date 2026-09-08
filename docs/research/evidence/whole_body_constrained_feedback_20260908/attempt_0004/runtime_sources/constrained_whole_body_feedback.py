"""Offline one-step constrained torque tracking; no runtime recovery authority."""
import time
import mujoco
import numpy as np
from scipy.optimize import minimize
class ConstrainedFeedbackFailure(RuntimeError):
 def __init__(self, diagnostic):
  self.diagnostic=diagnostic
  super().__init__(diagnostic['status'])
def constrained_tracking(model, data, desired, foot_geom_ids=None, max_iterations=50, wall_budget_s=30.):
 """Minimize Euclidean torque correction under fixed 35Nm/180N limits.
 Forces are independently evaluated before integration and after one unchanged
 model step. Failure is a diagnostic, never proof of physical infeasibility.
 Scratch data restores complete integration state for every evaluation;
 no model options or input state changes. Plugin models are unsupported.
 """
 started=time.perf_counter();calls=0;derivative_calls=0;best=None
 desired=np.asarray(desired,dtype=float)
 if desired.shape!=(model.nu,) or not np.all(np.isfinite(desired)):
  raise ValueError('finite desired torque of model.nu entries required')
 if model.nplugin:raise ValueError('plugin models unsupported by integration-state scratch restoration')
 saved_state=np.empty(mujoco.mj_stateSize(model,mujoco.mjtState.mjSTATE_INTEGRATION))
 mujoco.mj_getState(model,data,saved_state,mujoco.mjtState.mjSTATE_INTEGRATION)
 if not np.all(np.isfinite(saved_state)):raise ValueError('finite complete integration state required')
 if max_iterations<1 or wall_budget_s<=0 or not np.isfinite(wall_budget_s):
  raise ValueError('positive iteration and wall budgets required')
 gids=list(foot_geom_ids) if foot_geom_ids is not None else [mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
 if not gids or len(set(gids))!=len(gids) or min(gids)<0 or max(gids)>=model.ngeom:
  raise ValueError('unique existing foot geometry ids required')
 pre=mujoco.MjData(model);post=mujoco.MjData(model)
 def diagnostic(status,**extra):
  return dict(status=status,scope='offline constrained tracking; not infeasibility or B1 proof',torque_limit_nm=35.,normal_force_limit_n=180.,foot_geom_ids=gids,calls=calls,derivative_calls=derivative_calls,elapsed_s=time.perf_counter()-started,**extra)
 def check_budget():
  if time.perf_counter()-started>wall_budget_s:raise ConstrainedFeedbackFailure(diagnostic('wall_budget_exhausted'))
 def summed(d):
  values=np.zeros(len(gids))
  for i in range(d.ncon):
   c=d.contact[i];force=np.empty(6);mujoco.mj_contactForce(model,d,i,force)
   for j,gid in enumerate(gids):
    if gid in (c.geom1,c.geom2):values[j]+=force[0]
  return values
 def evaluate(tau,origin="constraint_evaluation"):
  nonlocal calls,best
  check_budget();calls+=1
  mujoco.mj_setState(model,pre,saved_state,mujoco.mjtState.mjSTATE_INTEGRATION);pre.ctrl[:]=tau
  mujoco.mj_forward(model,pre)
  mujoco.mj_setState(model,post,saved_state,mujoco.mjtState.mjSTATE_INTEGRATION);post.ctrl[:]=tau
  mujoco.mj_step(model,post);mujoco.mj_forward(model,post)
  values=np.r_[summed(pre),summed(post)]
  if not np.all(np.isfinite(values)):raise ConstrainedFeedbackFailure(diagnostic('nonfinite_force'))
  if np.all(abs(tau)<=35.) and np.all(values<=180.):
   cost=float(.5*np.dot(tau-desired,tau-desired))
   if best is None or cost<best[0]:best=(cost,np.array(tau).copy(),origin)
  return values
 def feasible(tau,forces):return bool(np.all(abs(tau)<=35.) and np.all(forces<=180.))
 def constraints(tau):return (180.-evaluate(tau))/180.
 def jacobian(tau):
  nonlocal derivative_calls
  derivative_calls+=1;previous=None
  for eps in (1e-3,1e-4,1e-5,1e-6):
   columns=[]
   for i in range(model.nu):
    shift=np.zeros(model.nu);shift[i]=eps
    columns.append((evaluate(tau-shift,'derivative_probe')-evaluate(tau+shift,'derivative_probe'))/(360.*eps))
   matrix=np.column_stack(columns)
   if previous is not None:
    errors=np.linalg.norm(matrix-previous,axis=0)
    scales=np.linalg.norm(matrix,axis=0)
    if np.all(errors<=1e-6+.01*scales):return matrix
   previous=matrix
  raise ConstrainedFeedbackFailure(diagnostic('force_derivative_unresolved'))
 projected=np.clip(desired,-35.,35.);initial_forces=evaluate(projected,'box_projection')
 if feasible(projected,initial_forces):
  fresh=evaluate(projected)
  if feasible(projected,fresh):return projected,diagnostic('box_optimum_feasible',pre_force_n=fresh[:len(gids)].tolist(),post_force_n=fresh[len(gids):].tolist(),iterations=0,correction_norm_nm=float(np.linalg.norm(projected-desired)))
 result=minimize(lambda tau:.5*np.dot(tau-desired,tau-desired),projected,jac=lambda tau:tau-desired,
  bounds=[(-35.,35.)]*model.nu,constraints=[{'type':'ineq','fun':constraints,'jac':jacobian}],method='SLSQP',options={'maxiter':max_iterations,'ftol':1e-10,'disp':False})
 final=evaluate(result.x,'solver_final')
 info=diagnostic('constrained_solution' if result.success and feasible(result.x,final) else 'solver_or_feasibility_failure',solver_success=bool(result.success),solver_message=str(result.message),iterations=int(result.nit),pre_force_n=final[:len(gids)].tolist(),post_force_n=final[len(gids):].tolist(),control_nm=result.x.tolist(),correction_norm_nm=float(np.linalg.norm(result.x-desired)),initial_pre_force_n=initial_forces[:len(gids)].tolist(),initial_post_force_n=initial_forces[len(gids):].tolist())
 if result.success and feasible(result.x,final):return result.x.copy(),info
 if best is not None:
  _,chosen,origin=best
  checked=evaluate(chosen,'fresh_witness_verification')
  if feasible(chosen,checked):
   info.update(status='strictly_feasible_evaluated_witness',optimality_claim=False,chosen_origin=origin,solver_final_control_nm=result.x.tolist(),solver_final_pre_force_n=final[:len(gids)].tolist(),solver_final_post_force_n=final[len(gids):].tolist(),control_nm=chosen.tolist(),pre_force_n=checked[:len(gids)].tolist(),post_force_n=checked[len(gids):].tolist(),correction_norm_nm=float(np.linalg.norm(chosen-desired)),calls=calls,elapsed_s=time.perf_counter()-started)
   return chosen.copy(),info
 raise ConstrainedFeedbackFailure(info)
