"""Privileged known-terrain whole-body MPC oracle probe.
This is a bounded research harness, not a controller or a B1 acceptance test.
It rolls a 70-step (140 ms) full-state horizon, commits and executes five
2 ms steps, and uses three torque knots for a joint body/foot/torque solve.
The source packet is periodic at absolute phase; controls use q/v/tau/K at
pre-step time while body and foot costs use the post-step reference (tick+1).
The compiled world-step box is a known-scene oracle. Body elevation uses a
spatial smoothstep at its compiled x edges. Foot elevation is phase-fixed: stance uses the
current touchdown target and swing interpolates the previous/next touchdown
terrain heights with smoothstep. It does not add contact policy or retiming.
"""
from __future__ import annotations
import argparse
import copy
import fcntl
import hashlib
import json
import math
import pathlib
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from contextlib import contextmanager
import mujoco
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from coupled_horizon_shooting import solve
from whole_body_horizon_probe import control as nominal_feedback
from whole_body_horizon_probe import packet, step
from whole_body_mpc_native import WholeBodyMPC
from verify_whole_body_mpc_oracle import verify_rows
DT = 0.002
HORIZON_STEPS = 70
COMMIT_STEPS = 5
KNOT_STEPS = np.array([0, 35, 69], dtype=int)
LEGS = ("FR", "FL", "RR", "RL")
TERRAIN_TRANSITION_M = 0.08
TORQUE_LIMIT = 35.0

def _json(value):
    if isinstance(value, pathlib.Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json(v) for v in value]
    return value

def write_json(path, value):
    path = pathlib.Path(path)
    path.write_text(json.dumps(_json(value), indent=2, allow_nan=False) + "\n")

def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()

def _dependencies(scene):
    """Hash the XML include and asset closure used by the known scene."""
    found = set()
    def visit(path):
        path = pathlib.Path(path).resolve()
        if path in found:
            return
        found.add(path)
        tree = ET.parse(path)
        root = tree.getroot()
        compiler = root.find("compiler")
        meshdir = path.parent / (compiler.get("meshdir", "") if compiler is not None else "")
        texturedir = path.parent / (compiler.get("texturedir", "") if compiler is not None else "")
        for node in root.iter():
            filename = node.get("file")
            if not filename:
                continue
            if node.tag == "include":
                visit(path.parent / filename)
            else:
                base = meshdir if node.tag == "mesh" else texturedir if node.tag == "texture" else path.parent
                candidate = (base / filename).resolve()
                if not candidate.is_file():
                    raise ValueError("missing model dependency " + str(candidate))
                found.add(candidate)
    visit(scene)
    return sorted(found)

@contextmanager
def experiment_lock():
    with open("/tmp/go2_mujoco_experiment.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise RuntimeError("experiment lock is busy") from exc
        yield

def state_array(model, data):
    out = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(model, data, out, mujoco.mjtState.mjSTATE_INTEGRATION)
    if not np.isfinite(out).all():
        raise RuntimeError("nonfinite integration state")
    return out

def data_from_state(model, state):
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, np.asarray(state, dtype=float), mujoco.mjtState.mjSTATE_INTEGRATION)
    return data

def smoothstep01(value):
    x = np.clip(float(value), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)

def scene_step_box(model):
    """Read the axis-aligned world step from the compiled scene geometry."""
    boxes = []
    preferred = []
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] != 0 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        boxes.append(geom_id)
        if "step" in name.lower() or "plateau" in name.lower():
            preferred.append(geom_id)
    choices = preferred if preferred else boxes
    if len(choices) != 1:
        raise ValueError("scene must expose one unambiguous world step box")
    geom_id = choices[0]
    pos = np.asarray(model.geom_pos[geom_id], dtype=float)
    size = np.asarray(model.geom_size[geom_id], dtype=float)
    quat = np.asarray(model.geom_quat[geom_id], dtype=float)
    if not np.isfinite(np.r_[pos, size, quat]).all() or np.any(size <= 0.0):
        raise ValueError("invalid world step box geometry")
    if np.max(np.abs(quat - np.array([1.0, 0.0, 0.0, 0.0]))) > 1e-12:
        raise ValueError("rotated world step box is unsupported")
    return float(pos[0] - size[0]), float(pos[0] + size[0]), float(pos[2] + size[2]), geom_id

class PeriodicTemplate:
    """Validated q/v/tau/K packet plus deterministic known-terrain references."""
    def __init__(self, model, refs, nominal):
        if len(refs) != HORIZON_STEPS:
            raise ValueError("validated packet must contain exactly 70 samples")
        self.model = model
        self.refs = refs
        self.nominal = nominal
        self.dt = float(nominal["timestep_s"])
        self.period = float(nominal["period_s"])
        self.phase = float(nominal["initial_phase"])
        self.duty = float(nominal["duty"])
        self.vx = float(nominal["command_vx"])
        self.offsets = np.asarray(nominal.get("leg_offsets", [0.0, 0.46, 0.46, 0.0]), dtype=float)
        self.box_x0, self.box_x1, self.box_top, self.box_geom_id = scene_step_box(model)
        self.transition_m = min(TERRAIN_TRANSITION_M, 0.25 * (self.box_x1 - self.box_x0))
        if not self.transition_m > 0.0:
            raise ValueError("world step box has no usable width")
        if abs(self.dt - DT) > 1e-12 or abs(self.period - HORIZON_STEPS * DT) > 1e-12:
            raise ValueError("packet calendar is not 2 ms / 70 step")
        if self.offsets.shape != (4,) or not np.isfinite(self.offsets).all():
            raise ValueError("invalid leg phase offsets")
        if not 0.0 < self.duty < 1.0:
            raise ValueError("invalid duty")
        if model.nq != 19 or model.nv != 18 or model.nu != 12 or model.na != 0:
            raise ValueError("requires Go2 nq19 nv18 nu12 na0")
        if abs(model.opt.timestep - DT) > 1e-12:
            raise ValueError("scene timestep is not 2 ms")
        self.t0 = float(refs[0]["time"])
        if abs(self.t0 - float(nominal["initial_integration_state"][0])) > 1e-10:
            raise ValueError("packet/initial absolute time mismatch")
        self.gids = np.asarray(
            [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in LEGS], dtype=int
        )
        if np.any(self.gids < 0):
            raise ValueError("foot geometry missing")
        self.flat_feet = np.empty((HORIZON_STEPS, 4, 3), dtype=float)
        probe = mujoco.MjData(model)
        for k, ref in enumerate(refs):
            probe.qpos[:] = ref["qpos"]
            probe.qvel[:] = ref["qvel"]
            mujoco.mj_forward(model, probe)
            self.flat_feet[k] = probe.geom_xpos[self.gids].copy()
        self.step_length = self.vx * self.period
        self._check_source_state(nominal)
    def _check_source_state(self, nominal):
        source = np.asarray(nominal["initial_integration_state"], dtype=float)
        if source.ndim != 1 or not np.isfinite(source).all():
            raise ValueError("invalid exact initial integration state")
        expected = mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_INTEGRATION)
        if len(source) != expected:
            raise ValueError("initial integration state size mismatch")
        if np.max(np.abs(source[1:20] - self.refs[0]["qpos"])) > 1e-12:
            raise ValueError("initial packet position changed")
        if np.max(np.abs(source[20:38] - self.refs[0]["qvel"])) > 1e-12:
            raise ValueError("initial packet velocity changed")
        for ref in self.refs:
            if ref["qpos"].shape != (19,) or ref["qvel"].shape != (18,):
                raise ValueError("packet q/v shape")
            if ref["tau"].shape != (12,) or ref["K"].shape != (12, 36):
                raise ValueError("packet tau/K shape")
            if not np.isfinite(np.r_[ref["qpos"], ref["qvel"], ref["tau"], ref["K"].ravel()]).all():
                raise ValueError("packet q/v/tau/K nonfinite")
    def ref(self, tick):
        """Pre-step source row at the exact absolute periodic tick."""
        index = int(tick) % HORIZON_STEPS
        cycles = int(tick) // HORIZON_STEPS
        source = self.refs[index]
        qpos = source["qpos"].copy()
        qpos[0] += cycles * self.step_length
        return {
            "time": self.t0 + float(tick) * DT,
            "qpos": qpos,
            "qvel": source["qvel"].copy(),
            "tau": source["tau"].copy(),
            "K": source["K"].copy(),
        }
    def foot_position(self, tick, leg):
        index = int(tick) % HORIZON_STEPS
        cycles = int(tick) // HORIZON_STEPS
        xyz = self.flat_feet[index, leg].copy()
        xyz[0] += cycles * self.step_length
        return xyz
    def terrain_weight(self, x):
        # A known-scene reference only: 8 cm spatial smoothstep at both edges.
        entry = smoothstep01((float(x) - (self.box_x0 - self.transition_m)) / (2.0 * self.transition_m))
        exit_ = smoothstep01(((self.box_x1 + self.transition_m) - float(x)) / (2.0 * self.transition_m))
        return entry * exit_
    def touchdown_elevation(self, leg, touchdown_number):
        # The touchdown phase is leg_phase=0. Sample the fixed periodic template
        # at that phase and add only the corresponding absolute cycle distance.
        base_phase = float(touchdown_number) - float(self.offsets[leg])
        delta = base_phase - self.phase
        cycles = math.floor(delta + 1e-12)
        fraction = delta - cycles
        extra_cycle, row = divmod(int(np.rint(fraction * HORIZON_STEPS)), HORIZON_STEPS)
        cycles += extra_cycle
        x = float(self.flat_feet[row, leg, 0] + cycles * self.step_length)
        return self.box_top if self.box_x0 <= x <= self.box_x1 else 0.0
    def foot_elevation(self, global_post_tick, leg):
        # Fixed phase avoids an instantaneous terrain(x) jump in flight. On a
        # stance, hold the current touchdown target; during swing, interpolate
        # previous and next touchdown heights with C1 smoothstep.
        base_phase = self.phase + float(global_post_tick) / HORIZON_STEPS
        leg_phase_unwrapped = base_phase + float(self.offsets[leg])
        touchdown_number = math.floor(leg_phase_unwrapped + 1e-12)
        leg_phase = leg_phase_unwrapped - touchdown_number
        if leg_phase < self.duty:
            return self.touchdown_elevation(leg, touchdown_number)
        swing_fraction = (leg_phase - self.duty) / (1.0 - self.duty)
        blend = smoothstep01(swing_fraction)
        previous = self.touchdown_elevation(leg, touchdown_number)
        following = self.touchdown_elevation(leg, touchdown_number + 1)
        return (1.0 - blend) * previous + blend * following
    def references(self, start_tick):
        body = np.empty((HORIZON_STEPS, 13), dtype=float)
        feet = np.empty((HORIZON_STEPS, 12), dtype=float)
        times = np.empty(HORIZON_STEPS, dtype=float)
        for k in range(HORIZON_STEPS):
            post_tick = int(start_tick) + k + 1
            ref = self.ref(post_tick)
            base_x = float(ref["qpos"][0])
            body[k, :7] = ref["qpos"][:7]
            body[k, 7:13] = ref["qvel"][:6]
            body[k, 2] += self.box_top * self.terrain_weight(base_x)
            # Velocity is the derivative of the added body-height reference.
            before_x = float(self.ref(post_tick-1)["qpos"][0])
            after_x = float(self.ref(post_tick+1)["qpos"][0])
            body[k, 9] += self.box_top * (self.terrain_weight(after_x)-self.terrain_weight(before_x))/(2*DT)
            for leg in range(4):
                xyz = self.foot_position(post_tick, leg)
                xyz[2] += self.foot_elevation(post_tick, leg)
                feet[k, 3 * leg : 3 * leg + 3] = xyz
            times[k] = ref["time"]
        if not np.isfinite(np.r_[body.ravel(), feet.ravel(), times]).all():
            raise ValueError("nonfinite terrain reference")
        return body, feet, times
    def nominal_baseline(self, current, start_tick, committed_prefix=None):
        """Generate K feedback at each pre-step state after the exact prefix."""
        predicted = data_from_state(self.model, state_array(self.model, current))
        controls = np.empty((HORIZON_STEPS, 12), dtype=float)
        prefix = None if committed_prefix is None else np.asarray(committed_prefix, dtype=float)
        if prefix is not None and prefix.shape != (COMMIT_STEPS, 12):
            raise ValueError("committed prefix shape")
        for k in range(HORIZON_STEPS):
            ref = self.ref(int(start_tick) + k)
            tau = prefix[k].copy() if prefix is not None and k < COMMIT_STEPS else np.asarray(nominal_feedback(self.model, ref, predicted), dtype=float)
            if tau.shape != (12,) or not np.isfinite(tau).all() or np.max(np.abs(tau)) > TORQUE_LIMIT:
                raise ValueError("nominal feedback torque invalid")
            controls[k] = tau
            step(self.model, predicted, tau, self.gids)
        return controls

def expand_knots(knots, baseline):
    knots = np.asarray(knots, dtype=float).reshape(3, 12)
    if not np.isfinite(knots).all():
        raise ValueError("nonfinite knot vector")
    full = np.empty_like(baseline)
    grid = np.arange(HORIZON_STEPS, dtype=float)
    for actuator in range(12):
        full[:, actuator] = baseline[:, actuator] + np.interp(grid, KNOT_STEPS, knots[:, actuator])
    # Exact published prefix; expanded torque limits are separate inequalities.
    full[:COMMIT_STEPS] = baseline[:COMMIT_STEPS]
    return full

def source_identity(packet_dir, scene, library, nominal_path):
    packet_dir = pathlib.Path(packet_dir).resolve()
    scene = pathlib.Path(scene).resolve()
    library = pathlib.Path(library).resolve()
    files = {
        packet_dir / "manifest.json",
        packet_dir / "receipt.json",
        packet_dir / "trajectory.packet",
        nominal_path.resolve(),
        HERE / "whole_body_mpc_oracle_probe.py",
        HERE / "whole_body_mpc_native.py",
        HERE / "whole_body_mpc_native.cpp",
        HERE / "whole_body_horizon_probe.py",
        HERE / "coupled_horizon_shooting.py",
    }
    if (HERE / "verify_whole_body_mpc_oracle.py").is_file():
        files.add(HERE / "verify_whole_body_mpc_oracle.py")
    files.update(_dependencies(scene))
    if library.is_file():
        files.add(library)
    missing = [str(path) for path in files if not pathlib.Path(path).is_file()]
    if missing:
        raise ValueError("missing source identity file: " + ", ".join(sorted(missing)))
    hashes = {str(path): sha(path) for path in sorted(files)}
    try:
        git_head = subprocess.check_output(["git", "-C", str(scene.parents[2]), "rev-parse", "HEAD"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        git_head = "unavailable"
    return {
        "git_head": git_head,
        "mujoco_version": mujoco.__version__,
        "scene": str(scene),
        "library": str(library),
        "packet": str(packet_dir),
        "nominal": str(nominal_path.resolve()),
        "sha256": hashes,
    }

def run(args):
    out = pathlib.Path(args.out).resolve()
    if out.exists():
        raise ValueError("output directory exists")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.mkdir()
    report = {
        "schema": "whole-body-mpc-oracle-probe-v1",
        "scope": "privileged known-scene full-state research oracle; not runtime authority or B1 acceptance",
        "status": "started",
        "config": {
            "horizon_steps": HORIZON_STEPS,
            "commit_steps": COMMIT_STEPS,
            "timestep_s": DT,
            "knot_steps": KNOT_STEPS.tolist(),
            "max_chunks": args.max_chunks,
            "solver_wall_budget_s": args.solver_wall_budget_s,
            "solver_max_iterations": args.solver_max_iterations,
            "solver_fd_step": args.solver_fd_step,
        },
        "chunks": [],
        "executed_rows": [],
    }
    try:
        with experiment_lock():
            manifest, refs, nominal = packet(args.packet)
            scene = pathlib.Path(args.scene).resolve()
            model = mujoco.MjModel.from_xml_path(str(scene))
            template = PeriodicTemplate(model, refs, nominal)
            initial = np.asarray(nominal["initial_integration_state"], dtype=float)
            current = data_from_state(model, initial)
            if np.max(np.abs(state_array(model, current) - initial)) > 1e-12:
                raise ValueError("exact initial integration state did not round-trip")
            ids = source_identity(args.packet, scene, pathlib.Path(args.library), pathlib.Path(manifest["source_hashes"]["nominal"]["path"]))
            report["source"] = ids
            report["initial_integration_state"] = initial.tolist()
            report["initial_time_s"] = float(current.time)
            report["terrain_reference_policy"] = {
                "box_x_m": [template.box_x0, template.box_x1],
                "box_top_m": template.box_top,
                "box_geom_id": template.box_geom_id,
                "spatial_transition_m": template.transition_m,
                "touchdown_height": "exact known box top at fixed-phase foothold X, no spatial ramp in support surface",
                "body": "base z adds smoothstep terrain elevation at base x",
                "feet": "stance holds current fixed-phase touchdown elevation; swing C1-interpolates previous/next touchdown elevations",
                "contact_policy": "none added; MuJoCo scene contacts remain authoritative in the oracle",
            }
            input_record = {
                "schema": "whole-body-mpc-oracle-input-v1",
                "manifest_sha256": sha(pathlib.Path(args.packet) / "manifest.json"),
                "packet_sha256": sha(pathlib.Path(args.packet) / "trajectory.packet"),
                "initial_integration_state": initial.tolist(),
                "period_s": template.period,
                "initial_phase": template.phase,
                "duty": template.duty,
                "command_vx_mps": template.vx,
                "refs_qpos": [r["qpos"].tolist() for r in refs],
                "refs_qvel": [r["qvel"].tolist() for r in refs],
                "refs_tau": [r["tau"].tolist() for r in refs],
                "refs_K": [r["K"].tolist() for r in refs],
            }
            write_json(out / "input.json", input_record)
            previous_candidate = None
            start_tick = 0
            for chunk_index in range(args.max_chunks):
                chunk_started = time.perf_counter()
                state_before = state_array(model, current)
                previous_prefix = None
                if previous_candidate is not None:
                    if previous_candidate.shape != (HORIZON_STEPS, 12):
                        raise ValueError("previous candidate shape")
                    previous_prefix = previous_candidate[COMMIT_STEPS : 2 * COMMIT_STEPS]
                baseline = template.nominal_baseline(current, start_tick, previous_prefix)
                if not np.isfinite(baseline).all() or np.max(np.abs(baseline)) > TORQUE_LIMIT:
                    raise ValueError("baseline torque constraint")
                body_refs, foot_refs, ref_times = template.references(start_tick)
                if abs(ref_times[0] - (current.time + DT)) > 1e-9:
                    raise ValueError("reference post-step absolute phase mismatch")
                mpc = None
                chunk = None
                evaluation_latencies = []
                evaluation_min_g = []
                try:
                    mpc = WholeBodyMPC(
                        args.library,
                        scene,
                        state_before,
                        baseline,
                        body_refs,
                        foot_refs,
                        COMMIT_STEPS,
                        privileged=True,
                    )
                    if mpc.gsize != HORIZON_STEPS * 96:
                        raise ValueError("unexpected native constraint size")
                    def evaluate_knots(knots):
                        candidate_controls = expand_knots(knots, baseline)
                        t_eval = time.perf_counter()
                        cost, g = mpc.evaluate(candidate_controls)
                        evaluation_latencies.append((time.perf_counter() - t_eval) * 1000.0)
                        evaluation_min_g.append(float(np.min(g)))
                        return cost, np.r_[g, (TORQUE_LIMIT-candidate_controls).ravel()/TORQUE_LIMIT, (TORQUE_LIMIT+candidate_controls).ravel()/TORQUE_LIMIT]
                    initial_knots = np.zeros((3,12))
                    solver = solve(
                        evaluate_knots,
                        initial_knots,
                        -TORQUE_LIMIT,
                        TORQUE_LIMIT,
                        fixed_prefix_steps=0,
                        max_iterations=args.solver_max_iterations,
                        wall_budget_s=args.solver_wall_budget_s,
                        constraint_tolerance=0.0,
                        finite_difference_step=args.solver_fd_step,
                    )
                    chunk = {
                        "index": chunk_index,
                        "start_tick": start_tick,
                        "start_time_s": float(current.time),
                        "initial_integration_state": state_before.tolist(),
                        "baseline_controls": baseline.tolist(),
                        "body_refs": body_refs.tolist(),
                        "foot_refs": foot_refs.tolist(),
                        "reference_times_s": ref_times.tolist(),
                        "solver": {k: v for k, v in solver.items() if k != "controls"},
                        "evaluation_latency_ms": evaluation_latencies,
                        "evaluation_min_constraint": evaluation_min_g,
                    }
                    candidate = solver.get("controls")
                    if candidate is None:
                        chunk["failure"] = "no_feasible_three_knot_witness"
                        report["chunks"].append(chunk)
                        report["status"] = "failed"
                        report["failure"] = chunk["failure"]
                        write_json(out / "run.json", report)
                        return report
                    chunk["correction_knots"] = np.asarray(candidate).tolist()
                    candidate = expand_knots(candidate, baseline)
                    if candidate.shape != (HORIZON_STEPS, 12) or not np.isfinite(candidate).all():
                        raise ValueError("candidate shape/nonfinite")
                    if not np.array_equal(candidate[:COMMIT_STEPS], baseline[:COMMIT_STEPS]):
                        raise ValueError("candidate committed prefix changed")
                    if np.max(np.abs(candidate)) > TORQUE_LIMIT:
                        raise ValueError("candidate torque limit")
                    native_started = time.perf_counter()
                    native_cost, native_g = mpc.evaluate(candidate)
                    native_latency_ms = (time.perf_counter() - native_started) * 1000.0
                    replay_started = time.perf_counter()
                    replay = mpc.replay(candidate)
                    replay_latency_ms = (time.perf_counter() - replay_started) * 1000.0
                    cost_delta = abs(float(native_cost) - float(replay["cost"]))
                    g_delta = float(np.max(np.abs(native_g - np.asarray(replay["g"], dtype=float))))
                    if cost_delta > 1e-10 or g_delta > 1e-9:
                        raise ValueError("native/Python replay mismatch")
                    if float(np.min(native_g)) < 0 or float(np.min(replay["g"])) < 0:
                        raise ValueError("native constraint violation")
                    replay_report = None
                    if verify_rows is not None:
                        replay_report = verify_rows(model, state_before, replay["states"])
                        report["pending_verification"] = {"chunk": chunk, "verification": replay_report}
                        gate_names = tuple(k for k in replay_report["checks"] if k not in ("historical_absolute_dynamics", "historical_clock"))
                        if (any(not replay_report["checks"].get(name, False) for name in gate_names)
                            or replay_report["maxima"]["dynamics_normwise"] > 1e-8
                            or replay_report["maxima"]["clock_delta_s"] > 1e-10):
                            raise ValueError("independent replay diagnostic V2 failure")
                    chunk.update({
                        "candidate_controls": candidate.tolist(),
                        "native_cost": float(native_cost),
                        "native_min_constraint": float(np.min(native_g)),
                        "native_latency_ms": native_latency_ms,
                        "python_replay_cost": float(replay["cost"]),
                        "python_replay_cost_delta": cost_delta,
                        "python_replay_constraint_delta": g_delta,
                        "python_replay_latency_ms": replay_latency_ms,
                        "python_replay_states": replay["states"],
                        "independent_replay_verification": replay_report,
                    })
                    # Apply only after native/Python equivalence and constraints.
                    executed = []
                    for local_step, tau in enumerate(candidate[:COMMIT_STEPS]):
                        tau = np.asarray(tau, dtype=float)
                        pre_time = float(current.time)
                        force_vector = step(model, current, tau, template.gids)
                        saved_state = state_array(model, current)
                        predicted_row = replay["states"][local_step]
                        if (np.max(abs(saved_state-np.asarray(predicted_row["integration_state"]))) > 1e-9
                            or np.max(abs(force_vector-np.asarray(predicted_row["forces"]))) > 1e-7):
                            raise ValueError("executed command diverged from admitted candidate")
                        if abs(float(current.time) - (pre_time + DT)) > 1e-10:
                            raise ValueError("execution clock did not advance by 2 ms")
                        if abs(float(current.time) - (template.t0 + (start_tick + local_step + 1) * DT)) > 1e-9:
                            raise ValueError("execution absolute phase drift")
                        executed.append({
                            "tick": start_tick + local_step,
                            "time": float(current.time),
                            "pre_time": pre_time,
                            "control": tau.tolist(),
                            "integration_state": saved_state.tolist(),
                            "qpos": current.qpos.tolist(),
                            "qvel": current.qvel.tolist(),
                            "forces": force_vector.tolist(),
                        })
                    chunk["executed_rows"] = executed
                    chunk["chunk_latency_ms"] = (time.perf_counter() - chunk_started) * 1000.0
                    report["chunks"].append(chunk)
                    report["executed_rows"].extend(executed)
                    report.pop("pending_verification", None)
                    previous_candidate = candidate.copy()
                    start_tick += COMMIT_STEPS
                    write_json(out / "run.json", report)
                except Exception as exc:
                    if chunk is not None and all(existing is not chunk for existing in report["chunks"]):
                        chunk["failure"] = str(exc)
                        report["chunks"].append(chunk)
                        write_json(out / "run.json", report)
                    raise
                finally:
                    if mpc is not None:
                        mpc.close()
            if report["executed_rows"]:
                report["sequential_verification"] = verify_rows(model, initial, report["executed_rows"])
            report["status"] = "completed"
            report["completed_chunks"] = len(report["chunks"])
            report["completed_steps"] = len(report["executed_rows"])
            write_json(out / "run.json", report)
            return report
    except Exception as exc:
        report["status"] = "failed"
        report["failure"] = str(exc)
        write_json(out / "run.json", report)
        raise

def main(argv=None):
    repo = HERE.parents[3]
    default_packet = repo / "docs/research/evidence/whole_body_native_20260908/trajectory_packet_0002"
    default_scene = repo / "unitree_robots/go2/b1_v3_running_step_5cm.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, help="built whole_body_mpc_native shared library")
    parser.add_argument("--packet", type=pathlib.Path, default=default_packet)
    parser.add_argument("--scene", type=pathlib.Path, default=default_scene)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    parser.add_argument("--max-chunks", type=int, default=2)
    parser.add_argument("--solver-wall-budget-s", type=float, default=5.0)
    parser.add_argument("--solver-max-iterations", type=int, default=20)
    parser.add_argument("--solver-fd-step", type=float, default=1e-4)
    args = parser.parse_args(argv)
    if args.max_chunks < 1 or args.max_chunks > 200:
        parser.error("--max-chunks must be in 1..200")
    result = run(args)
    print(json.dumps({
        "status": result.get("status"),
        "failure": result.get("failure"),
        "completed_chunks": result.get("completed_chunks", len(result.get("executed_rows", []))//COMMIT_STEPS),
        "completed_steps": result.get("completed_steps", len(result.get("executed_rows", []))),
        "out": str(args.out.resolve()),
    }, indent=2))

if __name__ == "__main__":
    main()
