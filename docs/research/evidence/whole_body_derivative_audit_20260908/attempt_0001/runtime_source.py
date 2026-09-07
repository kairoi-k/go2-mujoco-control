"""Locked, bounded directional audit; unchanged model, no optimization."""
import argparse
import copy
import fcntl
import hashlib
import json
import pathlib
import time
import mujoco
import numpy as np
from whole_body_cycle import CycleProblem, dependencies, forces
from whole_body_shooting_derivatives import rollout


def stats(a, b):
    diff = np.asarray(a) - np.asarray(b)
    return {'relative_error': float(np.linalg.norm(diff) / max(np.linalg.norm(b), 1e-12)),
            'max_abs': float(np.max(np.abs(diff))), 'reference_norm': float(np.linalg.norm(b))}


def trace(pb, x):
    d = copy.copy(pb.initial)
    raw = []
    for k in range(pb.steps):
        d.ctrl[:] = x.reshape(pb.blocks, 12)[k // pb.bs]
        mujoco.mj_step(pb.m, d)
        raw.append(copy.copy(d))
    return raw


def tangent(m, a, b, dt):
    q = np.empty(m.nv)
    mujoco.mj_differentiatePos(m, q, dt, a.qpos, b.qpos)
    return np.r_[q, (b.qvel-a.qvel)/dt]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'seed', 'scene', 'warm-start', 'out'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--force-target-n', type=float, default=170.)
    args = parser.parse_args()
    out = pathlib.Path(args.out)
    if out.exists():
        raise FileExistsError(out)
    with open('/tmp/go2_mujoco_experiment.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        started = time.perf_counter()
        pb = CycleProblem(args.source, args.seed, args.scene, force_target=args.force_target_n)
        warm = json.loads(pathlib.Path(args.warm_start).read_text())
        ctrl = np.asarray(warm['controls'], float)
        if ctrl.shape != (pb.steps, 12) or not np.all(np.isfinite(ctrl)):
            raise ValueError('warm start must contain one finite control per physical step')
        x = np.clip(ctrl.reshape(pb.blocks, pb.bs, 12).mean(axis=1).ravel(), -34.999999, 34.999999)
        J = pb.evaluate(x, True)
        observations, S = rollout(pb.m, pb.initial, x.reshape(pb.blocks, 12), pb.bs, True)
        nominal = trace(pb, x)
        categories = [('position', 18), ('velocity', 18), ('feet', 12), ('force', 4),
                      ('speed_limit', 12), ('joint_lower', 12), ('joint_upper', 12)]
        segments = []
        offset = 0
        for k in range(pb.steps):
            for name, width in categories + ([('terminal_position', 18), ('terminal_velocity', 18)] if k == pb.steps-1 else []):
                segments.append((k, name, offset, offset+width))
                offset += width
        segments.append((-1, 'control_regularization', offset, len(J)))
        repeated_forward = []
        for k, d in enumerate(observations):
            again = copy.copy(d)
            mujoco.mj_forward(pb.m, again)
            repeated_forward.append({'step': k, 'time': float(d.time),
                'force_difference_max_n': float(np.max(abs(forces(pb.m, again, pb.gids)-forces(pb.m, d, pb.gids)))),
                'qacc_difference_max': float(np.max(abs(again.qacc-d.qacc))),
                'warmstart_difference_max': float(np.max(abs(again.qacc_warmstart-d.qacc_warmstart)))})
        checks = []
        rng = np.random.default_rng(1301)
        for direction in range(3):
            v = rng.normal(size=x.size)
            v /= np.linalg.norm(v)
            chained = J @ v
            for eps in (1e-4, 1e-5, 1e-6):
                fd = (pb.evaluate(x+eps*v)-pb.evaluate(x-eps*v))/(2*eps)
                plus, minus = trace(pb, x+eps*v), trace(pb, x-eps*v)
                residuals = [{'step': k, 'category': name, **stats(chained[a:b], fd[a:b])}
                             for k, name, a, b in segments]
                states = []
                for k in range(pb.steps):
                    # Express both perturbed positions relative to the nominal tangent origin.
                    measured = (tangent(pb.m, nominal[k], plus[k], 1.)-tangent(pb.m, nominal[k], minus[k], 1.))/(2*eps)
                    predicted = S[k] @ v
                    # Same perturbed pre-step q/v/u, with warmstart replaced by nominal memory.
                    fp = copy.copy(pb.initial if k == 0 else plus[k-1])
                    fm = copy.copy(pb.initial if k == 0 else minus[k-1])
                    memory = pb.initial.qacc_warmstart if k == 0 else nominal[k-1].qacc_warmstart
                    fp.qacc_warmstart[:] = memory
                    fm.qacc_warmstart[:] = memory
                    fp.ctrl[:] = (x+eps*v).reshape(pb.blocks, 12)[k//pb.bs]
                    fm.ctrl[:] = (x-eps*v).reshape(pb.blocks, 12)[k//pb.bs]
                    mujoco.mj_step(pb.m, fp)
                    mujoco.mj_step(pb.m, fm)
                    frozen = (tangent(pb.m, nominal[k], fp, 1.)-tangent(pb.m, nominal[k], fm, 1.))/(2*eps)
                    states.append({'step': k, 'time': float(nominal[k].time),
                        'position': stats(predicted[:18], measured[:18]),
                        'velocity': stats(predicted[18:], measured[18:]),
                        'combined': stats(predicted, measured),
                        'warmstart_frozen_vs_actual': stats(frozen, measured),
                        'warmstart_directional_norm': float(np.linalg.norm((plus[k].qacc_warmstart-minus[k].qacc_warmstart)/(2*eps)))})
                bad = [r for r in states if r['combined']['relative_error'] > .03 and r['combined']['max_abs'] > 1e-6]
                checks.append({'direction': direction, 'epsilon': eps, 'overall': stats(chained, fd),
                    'first_state_divergence': bad[0]['step'] if bad else None,
                    'worst_state_step': max(states, key=lambda r:r['combined']['max_abs'])['step'],
                    'worst_residual': max(residuals, key=lambda r:r['max_abs']),
                    'residual_segments': residuals, 'states': states})
        files = dependencies(args.scene) | {pathlib.Path(f).resolve() for f in
            (args.source, args.seed, args.warm_start, __file__, pathlib.Path(__file__).with_name('whole_body_cycle.py'),
             pathlib.Path(__file__).with_name('whole_body_shooting_derivatives.py'))}
        report = {'schema': 'whole-body-derivative-audit-v1', 'mujoco_version': mujoco.__version__,
            'methods': 'Three deterministic unit Gaussian directions seed1301; centered whole-rollout FD. State FD uses nominal tangent origin. Frozen-memory probe replaces only pre-step qacc_warmstart with nominal values, preserving perturbed q/v/u. Repeated-forward probe repeats observation evaluation. No model or solver-option changes. Diagnostics do not certify smoothness or feedback.',
            'divergence_definition': 'combined relative error >0.03 and max absolute error >1e-6; diagnostic threshold only',
            'force_target_n': args.force_target_n, 'block_steps': pb.bs,
            'input_hashes': {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)},
            'checks': checks, 'repeated_forward': repeated_forward, 'elapsed_s': time.perf_counter()-started}
        with out.open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps({'out': str(out), 'elapsed_s': report['elapsed_s'],
            'checks': [{k:c[k] for k in ('direction','epsilon','overall','first_state_divergence','worst_state_step','worst_residual')} for c in checks]}))


if __name__ == '__main__':
    main()
