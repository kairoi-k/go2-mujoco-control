#!/usr/bin/env python3
"""Read-only, deterministic audit of the bounded joint_execution 0005 window."""
import argparse, csv, hashlib, json, math, re
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]
DEFAULT_RUN = ROOT / 'example/cpp/experiments/_runs/joint_execution_flat_20260908_0005'
DEFAULT_OUT = Path('/tmp/joint_execution_0005_divergence.json')
LO, HI = 21.0, 21.43
LEGS = ('FR', 'FL', 'RR', 'RL')
def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()
def num(s):
    try:
        x = float(s)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None
def vec(s):
    a = [num(x) for x in s.split(',')]
    return a if len(a) == 3 and all(x is not None for x in a) else None
def normdiff(a, b):
    if a is None or b is None:
        return None
    return math.sqrt(sum((x-y)*(x-y) for x, y in zip(a, b)))
def read_csv(path):
    with path.open(newline='') as f:
        rd = csv.DictReader(f)
        return rd.fieldnames or [], list(rd)
def select_time(rows, key):
    out = []
    for r in rows:
        t = num(r.get(key))
        if t is not None and LO <= t <= HI:
            out.append((t, r))
    return out
def unique_by_time(items):
    d = {}
    for t, r in items:
        d.setdefault(round(t, 9), (t, r))
    return [d[k] for k in sorted(d)]
def parse_kv_line(line):
    return {k: v.strip() for k, v in re.findall(r'([A-Za-z][A-Za-z0-9_]*)=([^ ]+)', line)}
def parse_tracking(path):
    pat = re.compile(
        r'JointExecutionTracking state=([^ ]+) version=(\d+) leg=(\d+) '
        r'planned_contact=(\d+) measured_contact=(\d+) ref_p=([^ ]+) ref_v=([^ ]+) '
        r'ref_a=([^ ]+) actual_p=([^ ]+) actual_v=([^ ]+) task_a=([^ ]+) '
        r'solved_a=([^ ]+) solved_force=([^ ]+)')
    records = []
    executions = []
    failures = []
    summaries = []
    with path.open(errors='replace') as f:
        for line in f:
            m = pat.search(line)
            if m:
                g = m.groups()
                records.append({
                    'state_s': num(g[0]), 'version': int(g[1]), 'leg': int(g[2]),
                    'planned_contact': int(g[3]), 'measured_contact': int(g[4]),
                    'ref_p_m': vec(g[5]), 'ref_v_mps': vec(g[6]), 'ref_a_mps2': vec(g[7]),
                    'actual_p_m': vec(g[8]), 'actual_v_mps': vec(g[9]),
                    'task_a_mps2': vec(g[10]), 'solved_a_mps2': vec(g[11]),
                    'solved_force_n': vec(g[12]),
                    'task_solved_accel_error_mps2': normdiff(vec(g[10]), vec(g[11])),
                })
                continue
            if line.startswith('JointExecution state='):
                kv = parse_kv_line(line)
                if num(kv.get('state')) is not None:
                    executions.append({k: (num(v) if k in {'state','elapsed_us','com_error_m','foot_error_m','force_residual_N','moment_residual_Nm','joint_residual_Nm','priority_residual'} else (int(v) if k in {'applied','execution_version','proposal','count','qp_iterations','certificate','motor_envelope'} else v)) for k,v in kv.items()})
            elif line.startswith('JointExecutionQpFailure '):
                failures.append(parse_kv_line(line))
            elif line.startswith('JointExecutionTrackingSummary '):
                summaries.append(parse_kv_line(line))
    return records, executions, failures, summaries
def contact_runs(rows, leg):
    key = f'{leg}_foot_contact_grf_world_z_N'
    xs = [(num(r.get('time_s')), num(r.get(key))) for r in rows]
    xs = [(t, v) for t, v in xs if t is not None and v is not None]
    runs = []
    i = 0
    while i < len(xs):
        if xs[i][1] <= 0.0:
            i += 1; continue
        j = i
        while j + 1 < len(xs) and xs[j+1][1] > 0.0:
            j += 1
        runs.append({
            'start_s': xs[i][0], 'end_s': xs[j][0],
            'sample_count': j-i+1, 'start_grf_z_N': xs[i][1],
            'max_grf_z_N': max(v for _, v in xs[i:j+1]),
        })
        i = j + 1
    return runs
def first_cross(items, pred):
    for t, r in items:
        x = pred(r)
        if x:
            return {'state_s': t, **x}
    return None
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', type=Path, default=DEFAULT_RUN)
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    run = args.run_dir
    data_p, gt_p, log_p, manifest_p = [run / x for x in ('data.csv','contact_ground_truth.csv','controller.log','run_manifest.json')]
    evidence_dir = Path(__file__).resolve().parent / 'attempt_0005'
    tracking_p = evidence_dir / 'tracking_excerpt.txt'
    for p in (data_p, gt_p, log_p, manifest_p, tracking_p):
        if not p.is_file():
            raise SystemExit(f'missing input: {p}')
    data_header, data_rows = read_csv(data_p)
    gt_header, gt_rows = read_csv(gt_p)
    data_sel = unique_by_time(select_time(data_rows, 'state_tick_s'))
    gt_sel = select_time(gt_rows, 'time_s')
    tr, ex, qp_fail, tr_summary = parse_tracking(tracking_p)
    tr_window = [x for x in tr if x['state_s'] is not None and LO <= x['state_s'] <= HI]
    ex_window = [x for x in ex if x.get('state') is not None and LO <= x['state'] <= HI]
    with manifest_p.open() as f: manifest = json.load(f)
    qcols = [x for x in data_header if x.endswith('_dq_state')]
    def dq_max(r):
        vals = [(abs(value), k, value) for k in qcols if (value := num(r.get(k))) is not None]
        return max(vals) if vals else None
    def attitude(r):
        roll, pitch = num(r.get('imu_roll_rad')), num(r.get('imu_pitch_rad'))
        return {'roll_rad': roll, 'pitch_rad': pitch, 'max_abs_rad': max(abs(x) for x in (roll,pitch) if x is not None) if any(x is not None for x in (roll,pitch)) else None}
    def row_at(t):
        for tt, r in data_sel:
            if abs(tt-t) < 1e-8: return r
        return None
    first_roll_02 = first_cross(data_sel, lambda r: {'value_rad': abs(num(r.get('imu_roll_rad')))} if num(r.get('imu_roll_rad')) is not None and abs(num(r.get('imu_roll_rad'))) > .2 else None)
    first_pitch_02 = first_cross(data_sel, lambda r: {'value_rad': abs(num(r.get('imu_pitch_rad')))} if num(r.get('imu_pitch_rad')) is not None and abs(num(r.get('imu_pitch_rad'))) > .2 else None)
    first_dq_10 = first_cross(data_sel, lambda r: ({'joint': dq_max(r)[1], 'value_radps': dq_max(r)[2]} if dq_max(r) and dq_max(r)[0] > 10.0 else None))
    gt_events = {leg: contact_runs([r for _,r in gt_sel], leg) for leg in LEGS}
    # Two consecutive >0 samples is used only to suppress the isolated 21.010 transient.
    sustained = {leg: [x for x in runs if x['sample_count'] >= 2] for leg, runs in gt_events.items()}
    # Controller tracking groups by exact logged state and leg.
    by_state = {}
    for r in tr_window:
        by_state.setdefault(f"{r['state_s']:.6f}", {})[str(r['leg'])] = r
    checkpoints = {}
    for target in (21.022, 21.060, 21.066):
        near = min(tr_window, key=lambda r: abs(r['state_s']-target), default=None)
        if near is None: continue
        st = str(near['state_s'])
        checkpoints[f"{target:.3f}"] = {'tracking': by_state.get(f"{near['state_s']:.6f}", {})}
    r022 = by_state.get(f"{21.022:.6f}", {})
    truth_check = {}
    for t in (21.020, 21.022, 21.024):
        rr = next((r for tt,r in gt_sel if abs(tt-t)<1e-8), None)
        if rr is not None:
            truth_check[str(t)] = {'grf_z_N': {leg:num(rr.get(f'{leg}_foot_contact_grf_world_z_N')) for leg in LEGS}, 'total_grf_z_N': num(rr.get('total_contact_grf_world_z_N'))}
    # Keep the reference/sensor distinction explicit: solved_force is a WBC model output, GT is plant contact force.
    support_force_compare = {}
    for leg, idx in [('FR','0'), ('RL','3')]:
        sr = r022.get(idx)
        if sr:
            support_force_compare[leg] = {
                'tracking_state_s': sr['state_s'],
                'wbc_solved_force_z_N': sr['solved_force_n'][2] if sr['solved_force_n'] else None,
                'wbc_solved_accel_z_mps2': sr['solved_a_mps2'][2] if sr['solved_a_mps2'] else None,
                'ground_truth_grf_z_N_by_time': {f"{t:.3f}": truth_check.get(str(t),{}).get('grf_z_N',{}).get(leg) for t in (21.020,21.022,21.024)},
            }
            c = support_force_compare[leg]
            c['difference_at_21.022_N'] = (c['wbc_solved_force_z_N'] - c['ground_truth_grf_z_N_by_time']['21.022']) if c['wbc_solved_force_z_N'] is not None and c['ground_truth_grf_z_N_by_time']['21.022'] is not None else None
    hashes = {p.name: sha256(p) for p in (data_p, gt_p, log_p, manifest_p, tracking_p)}
    source = manifest.get('repository', {})
    out = {
        'audit': 'joint_execution_flat_0005_divergence',
        'read_only': True,
        'window_s': [LO, HI],
        'run': {'path': str(run), 'run_id': manifest.get('run_id'), 'runtime_manifest_repository': source, 'effective_argv': manifest.get('effective_argv')},
        'provenance': {'input_sha256': hashes, 'analyzer_script_sha256': sha256(Path(__file__)), 'manifest_artifacts': manifest.get('artifacts', {}), 'manifest_analyzers': manifest.get('analyzers', {})},
        'input_shape': {'data_columns': len(data_header), 'data_window_rows_with_duplicates': len(select_time(data_rows,'state_tick_s')), 'data_window_unique_state_ticks': len(data_sel), 'ground_truth_columns': len(gt_header), 'ground_truth_window_rows': len(gt_sel), 'tracking_rows': len(tr_window)},
        'timeline': {'first_data_state_s': data_sel[0][0] if data_sel else None, 'last_data_state_s': data_sel[-1][0] if data_sel else None, 'first_tracking_state_s': min((x['state_s'] for x in tr_window), default=None), 'terminal_stop': next((x for x in ex_window if x.get('reason') == 'wbc_solver_failed' or str(x.get('stop_requested')) == '1'), None)},
        'ground_truth_contact': {'criterion': 'contact iff per-leg contact_ground_truth.csv *_foot_contact_grf_world_z_N > 0 N; sustained_onset uses at least 2 consecutive 2 ms samples', 'runs_by_leg': gt_events, 'sustained_runs_by_leg': sustained, 'early_swing_sustained_onsets': {'FL': next((x for x in sustained['FL'] if x['start_s'] >= 21.02), None), 'RR': next((x for x in sustained['RR'] if x['start_s'] >= 21.02), None)}, 'collision_fields': {k: sorted({num(r.get(k)) for _,r in gt_sel}) for k in ('reactive_obstacle_contact_count','phase2_terrain_nonfoot_contact_count') if k in gt_header}},
        'tracking_checkpoints': checkpoints,
        'support_force_model_vs_plant': {'meaning': 'wbc_solved_force is the controller model/reference output; ground_truth *_contact_grf is plant contact force and is not substituted into the former', 'truth_rows': truth_check, 'support_legs_at_21.022': support_force_compare, 'alignment_window_s': [21.020, 21.022, 21.024]},
        'controller_diagnostics': {'joint_execution_rows': ex_window, 'qp_failures': [x for x in qp_fail if num(x.get('state')) is not None and LO <= num(x['state']) <= HI], 'tracking_summaries': [x for x in tr_summary if num(x.get('state')) is not None and LO <= num(x['state']) <= HI]},
        'raw_state_anomaly_descriptors': {'joint_dq_columns': qcols, 'first_max_abs_joint_dq_gt_10_radps': first_dq_10, 'first_abs_imu_roll_gt_0.2_rad': first_roll_02, 'first_abs_imu_pitch_gt_0.2_rad': first_pitch_02, 'thresholds_are_descriptive_only': True},
        'interpretation': {'early_contact': 'GT sustains FL contact from about 21.054 s while tracking reports leg 1 planned swing/measured contact at 21.060 s; GT sustains RR contact from about 21.066 s while tracking reports leg 2 planned aerial/measured contact at 21.066 s.', 'legacy_planner_vs_actual_command': {'legacy_or_planner_diagnostics': ['terrain_plan_*','terrain_shadow_*','JointShadow','JointSelectedFoot','terrain_execution_*'], 'actual_joint_command_evidence': ['JointExecution applied/count/version','JointExecutionTracking ref/actual/task/solved fields','per-joint *_q_target/*_dq_target/*_tau_ff and *_q_state/*_dq_state','JointExecutionQpFailure stop'], 'authority_statement': 'planner/shadow records are diagnostic; JointExecution applied=1 is the sampled command path'}, 'limits': ['No causal attribution is made from these synchronized logs alone.', 'No B1 acceptance claim is made.', 'The GT base_qvel_world_z_mps field is blank in sampled rows, so vertical base velocity is unavailable there.']}
    }
    if out['provenance']['analyzer_script_sha256'] is None: raise SystemExit('script hash failed')
    if out_path := args.out:
        if out_path.exists() and not args.force:
            raise SystemExit(f'output exists; pass --force: {out_path}')
        out_path.write_text(json.dumps(out, indent=2, sort_keys=True) + '\n')
        print(out_path)
if __name__ == '__main__': main()
