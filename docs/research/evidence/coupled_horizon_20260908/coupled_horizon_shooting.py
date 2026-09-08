"""Bounded joint optimization of all uncommitted controls in a shooting horizon.
The evaluator owns dynamics and constraints; this module creates no second
robot model. Solver failure never proves global physical infeasibility.
"""
import time
import numpy as np
from scipy.optimize import minimize
class HorizonInputError(ValueError):
    pass
class HorizonNumericalError(RuntimeError):
    pass
def solve(evaluate, controls, lower, upper, fixed_prefix_steps=0,
          max_iterations=50, wall_budget_s=30., constraint_tolerance=1e-9, finite_difference_step=1e-8):
    """evaluate(full_controls) -> (scalar cost, inequalities >= 0).
    All free stages are one optimization vector. The fixed prefix is excluded
    from it and copied exactly on every evaluation. Finite difference probes
    can be feasible witnesses but have no optimality claim. The evaluator must
    check full temporal/model/terrain coverage and reject unknown inputs.
    constraint_tolerance is explicitly a numerical tolerance, never a changed
    physical limit; production authority is never provided by this result.
    """
    started = time.perf_counter()
    initial = np.asarray(controls, dtype=float)
    if initial.ndim != 2 or not initial.size or not np.all(np.isfinite(initial)):
        raise HorizonInputError('finite nonempty two-dimensional controls required')
    if (isinstance(fixed_prefix_steps, bool) or not isinstance(fixed_prefix_steps, int)
            or not 0 <= fixed_prefix_steps <= len(initial)):
        raise HorizonInputError('invalid fixed prefix')
    lo, hi = np.broadcast_to(lower, initial.shape).copy(), np.broadcast_to(upper, initial.shape).copy()
    if (not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi))
            or np.any(lo >= hi) or np.any(initial < lo) or np.any(initial > hi)):
        raise HorizonInputError('invalid bounds or initial controls outside bounds')
    if (max_iterations < 1 or not np.isfinite(wall_budget_s) or wall_budget_s <= 0
            or not np.isfinite(constraint_tolerance) or constraint_tolerance < 0
            or not np.isfinite(finite_difference_step) or finite_difference_step <= 0):
        raise HorizonInputError('invalid budgets or tolerance')
    prefix = initial[:fixed_prefix_steps].copy()
    free_shape = initial[fixed_prefix_steps:].shape
    cache_x = cache_value = best = None
    constraint_count = None
    evaluations = 0
    numerical_detail = None
    class Budget(Exception):
        pass
    def full(x):
        return np.concatenate((prefix, np.asarray(x).reshape(free_shape)), axis=0)
    def checked(x, fresh=False):
        nonlocal cache_x, cache_value, best, constraint_count, evaluations
        if time.perf_counter()-started > wall_budget_s:
            raise Budget()
        if not fresh and cache_x is not None and np.array_equal(x, cache_x):
            return cache_value
        u = full(x)
        value, constraints = evaluate(u.copy())
        g = np.asarray(constraints, dtype=float)
        if (not np.isscalar(value) or not np.isfinite(value) or g.ndim != 1
                or not np.all(np.isfinite(g))):
            raise HorizonNumericalError('nonfinite or malformed evaluation')
        if constraint_count is None:
            constraint_count = len(g)
        if len(g) != constraint_count:
            raise HorizonNumericalError('constraint coverage changed across evaluations')
        evaluations += 1
        result = (float(value), g.copy())
        if (np.all(u >= lo) and np.all(u <= hi)
                and np.all(g >= -constraint_tolerance)
                and (best is None or value < best[0])):
            best = (float(value), np.asarray(x).copy())
        cache_x, cache_value = np.asarray(x).copy(), result
        return result
    x0 = initial[fixed_prefix_steps:].ravel()
    status, solver_success, iterations = 'no_feasible_witness', False, 0
    try:
        checked(x0)
        if not x0.size:
            status = 'fixed_trajectory_checked'
        else:
            result = minimize(lambda x: checked(x)[0], x0, method='SLSQP',
                bounds=list(zip(lo[fixed_prefix_steps:].ravel(), hi[fixed_prefix_steps:].ravel())),
                constraints=[{'type': 'ineq', 'fun': lambda x: checked(x)[1]}],
                options={'maxiter': max_iterations, 'ftol': 1e-10, 'disp': False, 'eps': finite_difference_step})
            solver_success = bool(result.success)
            iterations = int(result.nit)
            status = str(result.message)
            checked(result.x, fresh=True)
        if best is not None:
            witness = best[1].copy()
            cost, g = checked(witness, fresh=True)
            if np.all(g >= -constraint_tolerance):
                return {'status': 'feasible_evaluated_witness', 'solver_status': status,
                    'solver_success': solver_success, 'global_optimality': False,
                    'global_infeasibility': False, 'can_actuate': False,
                    'controls': full(witness), 'cost': cost,
                    'constraint_violation': float(np.max(np.maximum(-g, 0), initial=0)),
                    'constraint_tolerance': constraint_tolerance,
                    'fixed_prefix_steps': fixed_prefix_steps, 'evaluations': evaluations,
                    'iterations': iterations, 'elapsed_s': time.perf_counter()-started}
    except Budget:
        status = 'wall_budget_exhausted'
    except HorizonNumericalError as error:
        status, numerical_detail = 'numerical_failure', str(error)
    # No stale feasible iterate is returned after budget/numerical failure.
    return {'status': status, 'controls': None, 'can_actuate': False,
        'solver_success': solver_success, 'global_infeasibility': False,
        'global_optimality': False, 'numerical_detail': numerical_detail,
        'evaluations': evaluations, 'iterations': iterations,
        'elapsed_s': time.perf_counter()-started}
