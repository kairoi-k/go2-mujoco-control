#!/usr/bin/env python3
"""Independent saved-control contact-cycle diagnostics; no acceptance changes."""
import argparse
import copy
import fcntl
import hashlib
import json
import pathlib
import subprocess
import mujoco
import numpy as np
from verify_whole_body_shooting import _check_dependencies

def episodes(rows, key, timestep):
    output = []
    for row in rows:
        value = row[key]
        if output and output[-1]['value'] == value:
            output[-1]['last_sample_s'] = row['time']
            output[-1]['samples'] += 1
            output[-1]['duration_s'] += timestep
            output[-1]['body_x_last_m'] = row['body_x_m']
        else:
            output.append({'value': value, 'first_sample_s': row['time'], 'last_sample_s': row['time'],
                           'samples': 1, 'duration_s': timestep, 'body_x_first_m': row['body_x_m'], 'body_x_last_m': row['body_x_m']})
    for episode in output:
        episode['body_advance_between_samples_m'] = episode['body_x_last_m'] - episode['body_x_first_m']
    return output

def complete_cycle_diagnostics(rows, initial_time, phase, period, timestep):
    """Report fully covered phase-zero cycles; samples represent 2ms cells.
    Assignment uses sample midpoints and reports the resulting quantization.
    This is a diagnostic, not the B1 terrain interaction analyzer.
    """
    result = []
    end = initial_time + len(rows)*timestep
    for index in range(int(np.ceil(phase+len(rows)*timestep/period))+1):
        start = initial_time + (index-phase)*period
        stop = start + period
        if start < initial_time-1e-10 or stop > end+1e-10:
            continue
        selected = [row for row in rows if start <= row['time']-timestep/2 < stop]
        diagonal = {}
        for mask, name in ((9,'FR_RL'),(6,'FL_RR')):
            diagonal[name] = max((e['duration_s'] for e in episodes(selected,'mask_10',timestep) if e['value']==mask),default=0.)
        aerial = max((e['duration_s'] for e in episodes(selected,'aerial_below_10n',timestep) if e['value']),default=0.)
        result.append({'phase_cycle_index':index,'start_s':start,'end_s':stop,'samples':len(selected),
            'max_contiguous_diagonal_s':diagonal,'max_contiguous_total_grf_below_10n_s':aerial,
            'running_contact_diagnostic':bool(all(v>=.010-1e-12 for v in diagonal.values()) and aerial>=.004-1e-12)})
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    output = pathlib.Path(args.out)
    if output.exists():
        raise FileExistsError(output)
    path = pathlib.Path(args.result).resolve()
    r = json.loads(path.read_text())
    if r['schema'] != 'whole-body-shooting-v1' or r['mujoco_version'] != mujoco.__version__:
        raise ValueError('unsupported result/model version')
    hashes = {(path.parent / k).resolve(): v for k, v in r['input_hashes'].items()}
    scene = (path.parent / r['scene']).resolve()
    _check_dependencies(scene, hashes)
    # Analysis code may have advanced; replay physics dependencies must match.
    model_hashes = {str(p): digest for p, digest in hashes.items() if p.suffix.lower() in ('.xml', '.obj', '.stl', '.msh', '.png', '.jpg', '.jpeg')}
    for name, digest in model_hashes.items():
        if hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('model input hash changed: ' + name)
    with open('/tmp/go2_mujoco_experiment.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        m = mujoco.MjModel.from_xml_path(str(scene))
        d = mujoco.MjData(m)
        mujoco.mj_setState(m, d, np.array(r['initial_integration_state']), r['integration_state_spec'])
        initial_x = float(d.qpos[0])
        initial_time = float(d.time)
        dt = float(m.opt.timestep)
        feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ('FR', 'FL', 'RR', 'RL')]
        floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'phase2_floor')
        if min(feet) < 0 or floor < 0 or m.geom_type[floor] != mujoco.mjtGeom.mjGEOM_PLANE:
            raise ValueError('flat diagnostic requires named plane and feet')
        root = int(m.jnt_bodyid[0]); robot = {root}
        for b in range(root + 1, m.nbody):
            if int(m.body_parentid[b]) in robot:
                robot.add(b)
        rows = []
        mass = np.empty((m.nv, m.nv))
        reproduction = 0.
        for k, (control, saved) in enumerate(zip(r['controls'], r['rows'])):
            d.ctrl[:] = control
            mujoco.mj_step(m, d)
            o = copy.copy(d)
            mujoco.mj_forward(m, o)
            reproduction = max(reproduction, float(np.max(abs(o.qpos - saved['qpos']))), float(np.max(abs(o.qvel - saved['qvel']))))
            normal = np.zeros(4); grf = np.zeros(3); contact_count = 0
            for i in range(o.ncon):
                c = o.contact[i]; force = np.zeros(6)
                mujoco.mj_contactForce(m, o, i, force)
                for leg, foot in enumerate(feet):
                    if foot in (c.geom1, c.geom2):
                        normal[leg] += force[0]
                first = int(m.geom_bodyid[c.geom1]) in robot
                second = int(m.geom_bodyid[c.geom2]) in robot
                if first != second:
                    # MuJoCo contact force acts on geom2; contact axes are rows.
                    grf += (1 if second else -1) * np.asarray(c.frame).reshape(3, 3).T @ force[:3]
                    contact_count += 1
            mujoco.mj_fullM(m, mass, o.qM)
            external = o.qfrc_applied.copy()
            for body in range(1, m.nbody):
                if np.any(o.xfrc_applied[body]):
                    mujoco.mj_applyFT(m, o, o.xfrc_applied[body, :3], o.xfrc_applied[body, 3:], o.xipos[body], body, external)
            components = [mass @ o.qacc, o.qfrc_bias, o.qfrc_passive, o.qfrc_actuator, external, o.qfrc_constraint]
            residual = components[0] + components[1] - sum(components[2:])
            scale = max(1., max(float(np.max(abs(v))) for v in components))
            floor_normal = np.asarray(o.geom_xmat[floor]).reshape(3, 3)[:, 2]
            clearances = [(float(np.dot(o.geom_xpos[foot] - o.geom_xpos[floor], floor_normal)) - float(m.geom_size[foot, 0])) for foot in feet]
            niter = np.asarray(o.solver_niter).astype(int)
            row = {'time': float(o.time), 'body_x_m': float(o.qpos[0]), 'body_z_m': float(o.qpos[2]),
                   'normal_forces_n': normal.tolist(), 'net_robot_contact_grf_world_n': grf.tolist(),
                   'total_grf_norm_n': float(np.linalg.norm(grf)), 'aerial_below_10n': bool(np.linalg.norm(grf) < 10.),
                   'foot_sphere_surface_plane_clearance_m': clearances,
                   'dynamics_residual_qcoord': residual.tolist(), 'dynamics_residual_max': float(np.max(abs(residual))),
                   'dynamics_component_max': scale, 'dynamics_residual_relative': float(np.max(abs(residual))) / scale,
                   'dynamics_base_force_residual_n': float(np.max(abs(residual[:3]))),
                   'dynamics_base_moment_residual_nm': float(np.max(abs(residual[3:6]))),
                   'dynamics_joint_residual_nm': float(np.max(abs(residual[6:]))),
                   'solver_niter': niter.tolist(), 'robot_environment_contact_count': contact_count}
            for threshold, label in ((1e-6, '1e6'), (5., '5'), (10., '10')):
                row['mask_' + label] = sum(1 << leg for leg in range(4) if normal[leg] > threshold)
            rows.append(row)
        if len(rows) != len(r['controls']) or len(rows) != len(r['rows']) or reproduction >= 1e-9:
            raise ValueError('incomplete or nonreproduced trajectory')
        gait = {label: episodes(rows, 'mask_' + label, dt) for label in ('1e6', '5', '10')}
        aerial = [e for e in episodes(rows, 'aerial_below_10n', dt) if e['value']]
        summary = {}
        for label in gait:
            summary[label] = {'FR_RL_diagonal_s': sum(e['duration_s'] for e in gait[label] if e['value'] == 9),
                              'FL_RR_diagonal_s': sum(e['duration_s'] for e in gait[label] if e['value'] == 6),
                              'all_four_s': sum(e['duration_s'] for e in gait[label] if e['value'] == 15),
                              'no_foot_s': sum(e['duration_s'] for e in gait[label] if e['value'] == 0),
                              'other_masks_s': sum(e['duration_s'] for e in gait[label] if e['value'] not in (0, 6, 9, 15))}
        report = {'schema': 'shooting-contact-cycle-diagnostic-v1', 'scope': 'collision-truth running-contact diagnostic; no B1 or acceptance-version change',
                  'trajectory_source_sha': r['source_sha'], 'analysis_source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                  'analysis_script_sha256': hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                  'result_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'model_input_hashes': model_hashes,
                  'recorded_input_hashes': r['input_hashes'], 'state_reproduction_max': reproduction,
                  'timestep_s': dt, 'steps': len(rows), 'duration_s': len(rows)*dt,
                  'body_advancement_m': float(d.qpos[0]) - initial_x, 'contact_duration_summary': summary, 'contact_episodes': gait, 'complete_phase_cycles': complete_cycle_diagnostics(rows,initial_time,r['initial_phase'],r['period_s'],dt),
                  'aerial_grf_below_10n_episodes': aerial, 'aerial_at_least_4ms': any(e['duration_s'] >= .004 - 1e-12 for e in aerial),
                  'foot_clearance_min_m': np.min([row['foot_sphere_surface_plane_clearance_m'] for row in rows], axis=0).tolist(),
                  'foot_clearance_max_m': np.max([row['foot_sphere_surface_plane_clearance_m'] for row in rows], axis=0).tolist(),
                  'dynamics_residual_max': max(row['dynamics_residual_max'] for row in rows),
                  'dynamics_residual_relative_max': max(row['dynamics_residual_relative'] for row in rows),
                  'dynamics_residual_worst_row': max(rows, key=lambda row: row['dynamics_residual_max']),
                  'solver_options': {'solver': int(m.opt.solver), 'tolerance': float(m.opt.tolerance), 'iterations': int(m.opt.iterations),
                                     'ls_tolerance': float(m.opt.ls_tolerance), 'ls_iterations': int(m.opt.ls_iterations), 'integrator': int(m.opt.integrator)},
                  'sample_duration_convention': '2ms per poststep sample; contact transition times are quantized to2ms, not interpolated',
                  'rows': rows}
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: report[key] for key in ('body_advancement_m', 'contact_duration_summary', 'aerial_at_least_4ms', 'foot_clearance_max_m', 'dynamics_residual_max', 'dynamics_residual_relative_max', 'solver_options')}, indent=2))

if __name__ == '__main__':
    main()
