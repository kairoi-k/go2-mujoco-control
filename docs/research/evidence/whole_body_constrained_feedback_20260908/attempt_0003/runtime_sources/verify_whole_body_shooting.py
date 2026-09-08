#!/usr/bin/env python3
"""Independent saved-control MuJoCo diagnostic certificate, never B1 acceptance."""
import argparse
import copy
import fcntl
import hashlib
import json
import math
import pathlib
import re
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
LIMITS = {"torque_limit_nm": 35., "joint_speed_limit_radps": 30.,
          "normal_force_limit_n": 180., "roll_pitch_limit_rad": 15 * math.pi / 180,
          "min_base_height_m": .28}
TOLERANCES = {"state": 1e-9, "time": 1e-12, "force": 1e-7,
              "dynamics": 1e-7, "friction": 1e-6}

def _array(value, shape, name):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.all(np.isfinite(a)):
        raise ValueError(name + " has malformed dimensions or nonfinite values")
    return a

def _scalar(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + " must be finite numeric")
    return float(value)

def _resolve(value, base):
    p = pathlib.Path(value)
    return (p if p.is_absolute() else base / p).resolve()

def _check_dependencies(scene, hashes):
    """Check every XML include and file-valued asset against bound hashes."""
    visited = set()
    main_dir = scene.parent
    dirs = {}
    assets = []
    def visit(path):
        if path in visited:
            return
        visited.add(path)
        if path not in hashes:
            raise ValueError("unhashed MJCF dependency: " + str(path))
        root = ET.parse(path).getroot()
        for element in root.iter():
            if element.tag == "compiler":
                dirs.update(element.attrib)
            name = element.get("file")
            if not name:
                continue
            raw = pathlib.Path(name)
            if element.tag == "include":
                choices = [raw] if raw.is_absolute() else [path.parent / raw, main_dir / raw]
                existing = {p.resolve() for p in choices if p.is_file()}
                if not existing or not existing.issubset(hashes):
                    raise ValueError("missing or unhashed include: " + name)
                for included in existing:
                    visit(included)
            else:
                assets.append((path, element.tag, raw))
    visit(scene)
    for path, tag, raw in assets:
        key = "meshdir" if tag == "mesh" else "texturedir" if tag == "texture" else "assetdir"
        asset_dir = dirs.get(key, dirs.get("assetdir", ""))
        choices = [raw] if raw.is_absolute() else [main_dir / asset_dir / raw, path.parent / asset_dir / raw]
        existing = {p.resolve() for p in choices if p.is_file()}
        if not existing or not existing.issubset(hashes):
            raise ValueError("missing or unhashed asset: " + str(raw))
    return len(visited)


def _rotation_metrics(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(float(np.clip(2 * (w*y - z*x), -1., 1.)))
    return abs(roll), abs(pitch)

def _contact_metrics(model, data, foot_ids, robot_bodies):
    normals = np.zeros(4)
    friction = unilateral = nonfoot_force = 0.
    nonfoot_count = 0
    for i in range(data.ncon):
        c = data.contact[i]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        if not np.all(np.isfinite(force)):
            raise ValueError("nonfinite contact force")
        unilateral = max(unilateral, -float(force[0]))
        # Anisotropic tangential coefficients are checked in their elliptical
        # cone; equal coefficients reduce exactly to the circular Coulomb cone.
        if c.dim >= 3:
            tangential = np.abs(force[1:3])
            mu = np.asarray(c.friction[:2])
            if np.any(mu <= 0):
                excess = float(np.max(tangential[mu <= 0], initial=0.))
                positive = mu > 0
                radius = float(np.linalg.norm(tangential[positive] / mu[positive]))
                friction = max(friction, excess, radius - force[0])
            else:
                friction = max(friction, float(np.linalg.norm(tangential / mu) - force[0]))
        ids = (int(c.geom1), int(c.geom2))
        foot_matches = [leg for leg, gid in enumerate(foot_ids) if gid in ids]
        for leg in foot_matches:
            normals[leg] += force[0]
        allowed = len(foot_matches) == 1
        if allowed:
            foot = foot_ids[foot_matches[0]]
            other = ids[1] if ids[0] == foot else ids[0]
            allowed = int(model.geom_bodyid[other]) not in robot_bodies
        magnitude = float(np.linalg.norm(force[:3]))
        if not allowed and magnitude > 1e-6:
            nonfoot_count += 1
            nonfoot_force = max(nonfoot_force, magnitude)
    return normals, max(0., friction), max(0., unilateral), nonfoot_count, nonfoot_force

def verify_result(result_path):
    path = pathlib.Path(result_path).resolve()
    r = json.loads(path.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError("nonfinite JSON " + x)))
    if r["schema"] != "whole-body-shooting-v1" or r["times_are_absolute"] is not True:
        raise ValueError("unsupported schema or nonabsolute clock")
    if not re.fullmatch(r"[0-9a-f]{40}", r["source_sha"]):
        raise ValueError("source_sha must bind a full commit")
    if r["mujoco_version"] != mujoco.__version__:
        raise ValueError("MuJoCo version mismatch")
    if r["integration_state_spec"] != int(mujoco.mjtState.mjSTATE_INTEGRATION):
        raise ValueError("full integration state required")
    hashes = {}
    for name, digest in r["input_hashes"].items():
        p = _resolve(name, path.parent)
        if not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("malformed input hash")
        if hashlib.sha256(p.read_bytes()).hexdigest() != digest:
            raise ValueError("input hash mismatch: " + str(p))
        hashes[p] = digest
    scene = _resolve(r["scene"], path.parent)
    xml_count = _check_dependencies(scene, hashes)
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nq < 7 or model.nv < 6 or model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("a free robot base at qpos0 is required")
    timestep = _scalar(r["timestep_s"], "timestep_s")
    period = _scalar(r["period_s"], "period_s")
    phase = _scalar(r["initial_phase"], "initial_phase")
    duty = _scalar(r["duty"], "duty")
    _scalar(r["command_vx"], "command_vx")
    if abs(timestep - model.opt.timestep) > 1e-15 or timestep <= 0 or period <= 0 or not 0 <= phase < 1 or not 0 < duty < 1:
        raise ValueError("invalid timestep, period, phase or duty")
    offsets = _array(r["leg_offsets"], (4,), "leg_offsets")
    if np.any(offsets < 0) or np.any(offsets >= 1):
        raise ValueError("leg offsets outside [0,1)")
    for name, value in LIMITS.items():
        if abs(_scalar(r["constraints"][name], name) - value) > 1e-12:
            raise ValueError("unregistered constraint: " + name)
    controls = np.asarray(r["controls"], dtype=float)
    if controls.ndim != 2 or controls.shape[1] != model.nu or controls.shape[0] == 0 or not np.all(np.isfinite(controls)):
        raise ValueError("invalid controls")
    steps = len(controls)
    if len(r["rows"]) != steps or steps * timestep + 1e-12 < period:
        raise ValueError("missing row or complete-period coverage")
    reference_q = _array(r["terminal_reference"]["qpos"], (model.nq,), "terminal qpos")
    reference_v = _array(r["terminal_reference"]["qvel"], (model.nv,), "terminal qvel")
    data = mujoco.MjData(model)
    state = _array(r["initial_integration_state"], (mujoco.mj_stateSize(model, r["integration_state_spec"]),), "initial integration state")
    mujoco.mj_setState(model, data, state, r["integration_state_spec"])
    initial_time = float(data.time)
    cycles = steps * timestep / period
    if abs(cycles - round(cycles)) > 1e-9:
        raise ValueError("terminal horizon must end at the same gait phase")
    bound_reference_q = data.qpos.copy()
    bound_reference_q[0] += r["command_vx"] * steps * timestep
    if np.max(abs(reference_q - bound_reference_q)) > 1e-12 or np.max(abs(reference_v - data.qvel)) > 1e-12:
        raise ValueError("terminal reference must be initial state translated by commanded world-X displacement")
    for j in range(model.njnt):
        kind = model.jnt_type[j]
        adr = model.jnt_qposadr[j] + (3 if kind == mujoco.mjtJoint.mjJNT_FREE else 0)
        if kind in (mujoco.mjtJoint.mjJNT_FREE, mujoco.mjtJoint.mjJNT_BALL):
            if abs(np.linalg.norm(data.qpos[adr:adr+4]) - 1) > 1e-9 or abs(np.linalg.norm(reference_q[adr:adr+4]) - 1) > 1e-9:
                raise ValueError("nonunit state quaternion")
    foot_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("FR", "FL", "RR", "RL")]
    if min(foot_ids) < 0:
        raise ValueError("missing named foot geometry")
    root_body = int(model.jnt_bodyid[0])
    robot_bodies = {root_body}
    for b in range(root_body + 1, model.nbody):
        if int(model.body_parentid[b]) in robot_bodies:
            robot_bodies.add(b)
    maxes = {key: 0. for key in ("state_error", "time_error", "force_error", "row_torque_error", "applied_torque_error",
        "dynamics_residual", "normal_force_n", "friction_cone_violation_n", "unilateral_violation_n", "torque_nm",
        "joint_speed_radps", "joint_position_violation_rad", "roll_rad", "pitch_rad", "nonfoot_force_n")}
    minimum_height = math.inf
    foot_min = np.full(4, np.inf)
    foot_max = np.full(4, -np.inf)
    episodes = []
    mask_mismatches = nonfoot_count = 0
    contact_by_leg = np.zeros(4, dtype=int)
    mass = np.empty((model.nv, model.nv))
    for k, (control, row) in enumerate(zip(controls, r["rows"])):
        saved_q = _array(row["qpos"], (model.nq,), "row qpos")
        saved_v = _array(row["qvel"], (model.nv,), "row qvel")
        saved_f = _array(row["normal_forces"], (4,), "row normal_forces")
        saved_tau = _array(row["torque"], (model.nu,), "row torque")
        saved_time = _scalar(row["time"], "row time")
        data.ctrl[:] = control
        mujoco.mj_step(model, data)
        observed = copy.copy(data)
        mujoco.mj_forward(model, observed)
        if not all(np.all(np.isfinite(getattr(observed, n))) for n in ("qpos", "qvel", "qacc", "actuator_force", "qfrc_constraint")):
            raise ValueError("nonfinite plant state")
        force, friction, unilateral, count, nonfoot = _contact_metrics(model, observed, foot_ids, robot_bodies)
        mujoco.mj_fullM(model, mass, observed.qM)
        external = observed.qfrc_applied.copy()
        for body in range(1, model.nbody):
            if np.any(observed.xfrc_applied[body]):
                mujoco.mj_applyFT(model, observed, observed.xfrc_applied[body, :3], observed.xfrc_applied[body, 3:], observed.xipos[body], body, external)
        residual = mass @ observed.qacc + observed.qfrc_bias - observed.qfrc_passive - observed.qfrc_actuator - external - observed.qfrc_constraint
        roll, pitch = _rotation_metrics(observed.qpos[3:7])
        joint_violation = 0.
        for j in range(model.njnt):
            if model.jnt_limited[j] and model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                position = observed.qpos[model.jnt_qposadr[j]]
                joint_violation = max(joint_violation, model.jnt_range[j, 0] - position, position - model.jnt_range[j, 1])
        current = {"state_error": max(float(np.max(abs(observed.qpos - saved_q))), float(np.max(abs(observed.qvel - saved_v)))),
            "time_error": max(abs(observed.time - saved_time), abs(observed.time - initial_time - (k+1)*timestep)),
            "force_error": float(np.max(abs(force - saved_f))), "row_torque_error": float(np.max(abs(control - saved_tau), initial=0.)),
            "applied_torque_error": float(np.max(abs(observed.actuator_force - saved_tau), initial=0.)),
            "dynamics_residual": float(np.max(abs(residual), initial=0.)), "normal_force_n": float(np.max(force)),
            "friction_cone_violation_n": friction, "unilateral_violation_n": unilateral,
            "torque_nm": max(float(np.max(abs(control), initial=0.)), float(np.max(abs(observed.actuator_force), initial=0.))),
            "joint_speed_radps": float(np.max(abs(observed.qvel[6:]), initial=0.)), "joint_position_violation_rad": joint_violation,
            "roll_rad": roll, "pitch_rad": pitch, "nonfoot_force_n": nonfoot}
        for name, value in current.items():
            maxes[name] = max(maxes[name], value)
        minimum_height = min(minimum_height, float(observed.qpos[2]))
        heights = observed.geom_xpos[foot_ids, 2]
        foot_min = np.minimum(foot_min, heights)
        foot_max = np.maximum(foot_max, heights)
        actual = sum(1 << i for i, value in enumerate(force) if value > 1e-6)
        planned = sum(1 << i for i, offset in enumerate(offsets) if (phase + (k+1)*timestep/period + offset) % 1 < duty)
        mask_mismatches += actual != planned
        contact_by_leg += force > 1e-6
        nonfoot_count += count
        if episodes and episodes[-1]["actual_mask"] == actual and episodes[-1]["planned_mask"] == planned:
            episodes[-1]["last_time"] = float(observed.time)
            episodes[-1]["samples"] += 1
        else:
            episodes.append({"first_time": float(observed.time), "last_time": float(observed.time), "samples": 1,
                             "actual_mask": actual, "planned_mask": planned})
    tangent = np.empty(model.nv)
    mujoco.mj_differentiatePos(model, tangent, 1., reference_q, data.qpos)
    terminal = {"body_position_m": float(np.linalg.norm(data.qpos[:3] - reference_q[:3])),
                "orientation_rad": float(np.linalg.norm(tangent[3:6])),
                "base_velocity_mps": float(np.linalg.norm(data.qvel[:3] - reference_v[:3])),
                "body_omega_radps": float(np.linalg.norm(data.qvel[3:6] - reference_v[3:6])),
                "joint_position_rad": float(np.max(abs(tangent[6:]), initial=0.)),
                "joint_velocity_radps": float(np.max(abs(data.qvel[6:] - reference_v[6:]), initial=0.))}
    checks = {"state_reproduction": maxes["state_error"] < 1e-9, "time_reproduction": maxes["time_error"] < 1e-12,
              "force_reproduction": maxes["force_error"] < 1e-7, "torque_reproduction": max(maxes["row_torque_error"], maxes["applied_torque_error"]) < 1e-9,
              "dynamics_balance": maxes["dynamics_residual"] < 1e-7,
              "friction_cone": maxes["friction_cone_violation_n"] <= 1e-6, "unilateral": maxes["unilateral_violation_n"] <= 1e-6,
              "torque_bound": maxes["torque_nm"] <= LIMITS["torque_limit_nm"],
              "normal_force_bound": maxes["normal_force_n"] <= LIMITS["normal_force_limit_n"],
              "joint_speed_bound": maxes["joint_speed_radps"] <= LIMITS["joint_speed_limit_radps"],
              "joint_position_bound": maxes["joint_position_violation_rad"] <= 1e-9,
              "attitude_bound": max(maxes["roll_rad"], maxes["pitch_rad"]) <= LIMITS["roll_pitch_limit_rad"],
              "base_height_bound": minimum_height >= LIMITS["min_base_height_m"], "no_nonfoot_contact_force": nonfoot_count == 0,
              "full_period": steps*timestep+1e-12 >= period,
              "terminal_body_position": terminal["body_position_m"] < .02, "terminal_orientation": terminal["orientation_rad"] < .05,
              "terminal_base_velocity": terminal["base_velocity_mps"] < .15, "terminal_body_omega": terminal["body_omega_radps"] < .3,
              "terminal_joint_position": terminal["joint_position_rad"] < .15, "terminal_joint_velocity": terminal["joint_velocity_radps"] < 3.}
    return {"schema": "whole-body-shooting-certificate-v1", "scope": "checked saved-state simulation diagnostic; not B1 or closed-loop acceptance",
            "diagnostic_success": bool(all(checks.values())), "checks": {k: bool(v) for k, v in checks.items()},
            "maxima": maxes, "minimum_base_height_m": minimum_height, "foot_center_height_min_m": foot_min.tolist(),
            "foot_center_height_max_m": foot_max.tolist(), "terminal_errors": terminal, "steps": steps,
            "periods_covered": steps*timestep/period, "nonfoot_contact_samples": nonfoot_count,
            "contact_mask_mismatch_samples": int(mask_mismatches), "actual_contact_samples_per_leg": contact_by_leg.tolist(),
            "contact_force_presence_threshold_n": 1e-6, "contact_episodes": episodes, "source_sha": r["source_sha"],
            "mujoco_version": mujoco.__version__, "xml_dependencies_checked": xml_count, "input_hash_count": len(hashes),
            "result_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "numerical_tolerances": TOLERANCES,
            "external_cartesian_force_handling": "mj_applyFT at body COM, world force/torque",
            "contact_provenance": "poststep collision-force truth; planned mask is reported separately"}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    # Exclusive create protects retained reports, including failures.
    with pathlib.Path(args.out).open("x") as stream:
        try:
            with open("/tmp/go2_mujoco_experiment.lock", "a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                report = verify_result(args.result)
        except Exception as exc:
            report = {"schema": "whole-body-shooting-certificate-v1", "diagnostic_success": False,
                      "verification_error": str(exc), "scope": "failed-closed verification; not B1"}
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("contact_episodes",)}, allow_nan=False))
    return 0 if report.get("diagnostic_success") else 1

if __name__ == "__main__":
    raise SystemExit(main())
