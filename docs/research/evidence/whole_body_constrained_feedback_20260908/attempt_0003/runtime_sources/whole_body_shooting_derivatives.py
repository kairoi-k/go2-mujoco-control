"""MuJoCo shooting rollout and local tangent-state control sensitivities.
The state coordinates are (configuration tangent, qvel, act). MuJoCo's
mjd_transitionFD linearizes this state, not solver warmstart memory. We retain
warmstart in the nominal rollout and freeze it locally during each derivative
call; poorly converged contact solves can therefore spoil chained derivatives.
Validate them against whole-rollout perturbations for the scene being optimized.
No model options, solver parameters, or integrator settings are changed.
"""
import copy
import numbers
import mujoco
import numpy as np

def _copy_data(model, data):
    # 3.3.6 exposes complete native MjData copying through __copy__, but does
    # not export mj_copyData as a module-level Python function.
    if hasattr(mujoco, "mj_copyData"):
        result = mujoco.MjData(model)
        mujoco.mj_copyData(result, model, data)
        return result
    return copy.copy(data)

class UnresolvedDerivativeError(RuntimeError):
    """Local derivative validation failed; diagnostic contains all trials."""
    def __init__(self, diagnostic):
        self.diagnostic = diagnostic
        super().__init__('local transition derivative unresolved')


def checked_transition_fd(model, data):
    """Return A, B, local diagnostics without changing model/data.

    Checks centered FD at 1e-6..1e-9 against the adjacent larger epsilon,
    three deterministic manifold directional differences, and contact topology.
    This certifies neither finite-radius smoothness nor omitted warmstart-state
    propagation. Activation models are explicitly unsupported in checked mode.
    """
    if model.na:
        raise ValueError('checked transition currently requires na == 0')
    nv, nu = model.nv, model.nu
    nx = 2*nv
    initial = _copy_data(model, data)
    center = _copy_data(model, initial)
    mujoco.mj_step(model, center)
    def topology(d):
        return (int(d.nefc), tuple((int(c.geom1), int(c.geom2), int(c.efc_address)) for c in d.contact))
    nominal_topology = topology(center)
    def perturb(z):
        d = _copy_data(model, initial)
        mujoco.mj_integratePos(model, d.qpos, z[:nv], 1.)
        d.qvel[:] += z[nv:nx]
        d.ctrl[:] += z[nx:]
        mujoco.mj_step(model, d)
        return d
    def delta(d):
        q = np.empty(nv)
        mujoco.mj_differentiatePos(model, q, 1., center.qpos, d.qpos)
        return np.r_[q, d.qvel-center.qvel]
    def agreement(a, b, rtol, atol):
        error = float(np.linalg.norm(a-b))
        scale = float(np.linalg.norm(b))
        return {'relative_error': error/max(scale, 1e-12),
                'max_abs': float(np.max(abs(a-b))),
                'pass': bool(error <= atol + rtol*scale)}
    rng = np.random.default_rng(1301)
    directions = rng.normal(size=(3, nx+nu))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    diagnostic = {'scope': 'local derivative diagnostic only; no finite-radius stability claim',
        'time': float(data.time), 'trials': [], 'selected_epsilon': None,
        'matrix_rtol': .005, 'matrix_atol': 1e-5,
        'direction_rtol': .01, 'direction_atol': 1e-6}
    previous = None
    for eps in (1e-6, 1e-7, 1e-8, 1e-9):
        A, B = np.empty((nx,nx)), np.empty((nx,nu))
        mujoco.mjd_transitionFD(model, _copy_data(model, initial), eps, True, A, B, None, None)
        matrix = np.hstack([A,B])
        trial = {'epsilon': eps, 'finite': bool(np.all(np.isfinite(matrix)))}
        diagnostic['trials'].append(trial)
        if not trial['finite']:
            previous = None
            continue
        if previous is None:
            previous = matrix
            continue
        # Require each column to converge: a large unrelated column must not
        # hide a bad small control/state column in a whole-matrix norm.
        convergence = [agreement(matrix[:,i], previous[:,i], .005, 1e-5) for i in range(nx+nu)]
        trial['matrix_convergence'] = convergence
        previous = matrix
        if not all(item['pass'] for item in convergence):
            continue
        probes = []
        for direction in directions:
            plus, minus = perturb(eps*direction), perturb(-eps*direction)
            measured = (delta(plus)-delta(minus))/(2*eps)
            probe = agreement(matrix@direction, measured, .01, 1e-6)
            probe['topology_same'] = topology(plus) == topology(minus) == nominal_topology
            probes.append(probe)
        trial['directional_probes'] = probes
        if not all(p['pass'] and p['topology_same'] for p in probes):
            continue
        crossed = []
        for col in range(nx+nu):
            z = np.zeros(nx+nu)
            z[col] = eps
            if topology(perturb(z)) != nominal_topology or topology(perturb(-z)) != nominal_topology:
                crossed.append(col)
        trial['axis_topology_crossings'] = crossed
        if crossed:
            continue
        diagnostic['selected_epsilon'] = eps
        return A, B, diagnostic
    raise UnresolvedDerivativeError(diagnostic)


def rollout(model, initial_data, controls, block_steps=1, compute_jacobian=False,
            transition_mode="native", derivative_diagnostics=None):
    """Return geometry-ready poststep copies and optional control derivatives.
    controls has shape (blocks, model.nu), each held for block_steps timesteps.
    Sensitivities have shape (blocks*block_steps, 2*nv+na, blocks*nu), with
    qpos derivatives expressed in mj_integratePos tangent coordinates. The
    input model/data/controls remain unchanged. Actuator activation states are
    included, including models with na > 0. Returned copies have mj_forward
    evaluated for geometry, while the next step uses the untouched step data.
    """
    if not isinstance(model, mujoco.MjModel) or not isinstance(initial_data, mujoco.MjData):
        raise TypeError("model and initial_data must be MuJoCo model/data")
    if isinstance(block_steps, bool) or not isinstance(block_steps, numbers.Integral) or block_steps < 1:
        raise ValueError("block_steps must be a positive integer")
    if not isinstance(compute_jacobian, (bool, np.bool_)):
        raise ValueError("compute_jacobian must be boolean")
    if transition_mode not in ("native", "checked"):
        raise ValueError("transition_mode must be native or checked")
    u = np.asarray(controls, dtype=float)
    if u.ndim != 2 or u.shape[1] != model.nu or u.shape[0] < 1:
        raise ValueError("controls must have shape (positive blocks, model.nu)")
    if not np.all(np.isfinite(u)):
        raise ValueError("controls must be finite")
    dimensions = {"qpos": model.nq, "qvel": model.nv, "act": model.na,
                  "ctrl": model.nu, "qacc_warmstart": model.nv}
    for name, size in dimensions.items():
        value = np.asarray(getattr(initial_data, name))
        if value.shape != (size,) or not np.all(np.isfinite(value)):
            raise ValueError("initial_data has invalid " + name)
    for name in ("qfrc_applied", "xfrc_applied", "mocap_pos", "mocap_quat", "userdata"):
        if not np.all(np.isfinite(getattr(initial_data, name))):
            raise ValueError("initial_data has nonfinite " + name)
    if not np.isfinite(initial_data.time):
        raise ValueError("initial_data.time must be finite")
    blocks = u.shape[0]
    steps = blocks * int(block_steps)
    nx = 2 * model.nv + model.na
    ntheta = blocks * model.nu
    derivatives = np.empty((steps, nx, ntheta)) if compute_jacobian else None
    sensitivity = np.zeros((nx, ntheta)) if compute_jacobian else None
    data = _copy_data(model, initial_data)
    rows = []
    for k in range(steps):
        block = k // block_steps
        data.ctrl[:] = u[block]
        if compute_jacobian:
            linearization_data = _copy_data(model, data)
            A = np.empty((nx, nx))
            B = np.empty((nx, model.nu))
            if transition_mode == "checked":
                A, B, diagnostic = checked_transition_fd(model, linearization_data)
                if derivative_diagnostics is not None:
                    derivative_diagnostics.append(diagnostic)
            else:
                mujoco.mjd_transitionFD(model, linearization_data, 1e-6, True, A, B, None, None)
            if not np.all(np.isfinite(A)) or not np.all(np.isfinite(B)):
                raise FloatingPointError("nonfinite transition derivative")
            sensitivity = A @ sensitivity
            sensitivity[:, block * model.nu:(block + 1) * model.nu] += B
            derivatives[k] = sensitivity
        mujoco.mj_step(model, data)
        if not all(np.all(np.isfinite(getattr(data, name))) for name in ("qpos", "qvel", "act")):
            raise FloatingPointError("nonfinite rollout state")
        observed = _copy_data(model, data)
        mujoco.mj_forward(model, observed)
        rows.append(observed)
    return rows, derivatives
