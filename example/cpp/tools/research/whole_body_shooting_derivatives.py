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

def rollout(model, initial_data, controls, block_steps=1, compute_jacobian=False):
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
