#!/usr/bin/env python3
"""Read-only replay of the registered F02 flat/step experiment; no simulation."""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
PACKET = Path(__file__).resolve().parent
REPO = PACKET.parents[3]
SHA = '7a8ffc6b9269490d7e46c3adfbd2e13ed8609dc6'
RUNS = {k: f'wbc_bias_{k}_7a8ffc6_20260907_0001' for k in ('flat', 'step')}
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    result = {'schema': 'wbc-swing-bias-closed-loop-v1', 'runtime_sha': SHA,
              'b1_candidate': False, 'runs': {}}
    binding = json.loads((PACKET/'pre_run_binding.json').read_text())
    for label, name in RUNS.items():
        rel = Path('example/cpp/experiments/_runs')/name
        run = REPO/rel
        manifest = json.loads((run/'run_manifest.json').read_text())
        assert manifest['repository']['git_commit'] == SHA
        assert str(manifest['repository']['git_dirty']).lower() == 'false'
        for key, path in [('controller_sha256','example/cpp/build/real_trot_go2'),
                          ('simulator_sha256','simulate/build/unitree_mujoco')]:
            assert manifest['artifacts'][key] == binding['binary_sha256'][path]
        item = {'run_id': name, 'repository': manifest['repository'],
                'statuses': manifest['statuses'],
                'raw_sha256': {str(p.relative_to(REPO)): digest(p)
                               for p in sorted(run.iterdir()) if p.is_file()},
                'analyses': {}}
        calls = [('cycles','audit_running_cycle_truth.py',['--start','17','--end','24'])] if label == 'flat' else [
            ('v3','analyze_b1_dynamic_v3.py',['--scene','unitree_robots/go2/b1_v3_running_step_5cm.xml']),
            ('approach','audit_b1_approach_contact.py',[])]
        for key, script, extra in calls:
            dest = args.out/(label+'_'+key+'.json')
            cmd = [sys.executable,'example/cpp/tools/analysis/'+script,str(rel),*extra,'--out',str(dest)]
            proc = subprocess.run(cmd,cwd=REPO,capture_output=True,text=True)
            (args.out/(label+'_'+key+'.log')).write_text(proc.stdout+proc.stderr)
            data = json.loads(dest.read_text())
            expected_exit = 1 if key == 'v3' and data['status']=='NOT_CERTIFIED' else 0
            assert proc.returncode == expected_exit
            item['analyses'][key] = data
        with (run/'data.csv').open() as f:
            rows = list(csv.DictReader(f))
        active = [r for r in rows if float(r['motion_stage']) in (2,3) and float(r['velocity_command_active']) == 1]
        offset = float(active[0]['state_tick_s'])-float(active[0]['telemetry_gait_time_s'])
        item['max_clock_drift_s'] = max(abs(float(r['state_tick_s'])-float(r['telemetry_gait_time_s'])-offset) for r in active)
        item['wbc_attempts'] = {'rows': len(active)}
        for k in ['wbc_full_id_ok','wbc_full_id_attempt_qp_converged','wbc_full_id_attempt_qp_recovery_used']:
            item['wbc_attempts'][k] = dict(collections.Counter(r[k] for r in active))
        for k in ['wbc_full_id_attempt_eq_residual','wbc_full_id_attempt_max_tau_violation_nm']:
            values = sorted(float(r[k]) for r in active)
            assert all(math.isfinite(v) for v in values)
            def quantile(q):
                i = (len(values)-1)*q
                lo = int(i)
                return values[lo]+(values[min(lo+1,len(values)-1)]-values[lo])*(i-lo)
            item['wbc_attempts'][k] = {'p50': quantile(.5), 'p95':quantile(.95),'max':values[-1]}
        if label == 'step':
            item['pre_first_front_contact'] = {}
            for leg in ['FR','FL']:
                t = item['analyses']['approach']['first_riser_contact'][leg]['time_s']
                window = [r for r in rows if t-.2 <= float(r['state_tick_s']) <= t]
                fields = ['terrain_plan_failure','terrain_execution_plan_usable','terrain_execution_applied_mask',f'terrain_exec_{leg}_in_flight']
                item['pre_first_front_contact'][leg] = {'interval_s':[t-.2,t], 'rows':len(window),
                    'counts':{k:dict(collections.Counter(r[k] for r in window)) for k in fields}}
        result['runs'][label] = item
    (args.out/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.out/'results.json'),'b1_candidate':False}))
if __name__ == '__main__':
    main()
