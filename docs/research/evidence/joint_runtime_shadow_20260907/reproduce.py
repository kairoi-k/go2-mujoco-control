#!/usr/bin/env python3
"""Verify exact-source bindings and audit joint shadow logs without changing raw runs."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
PACKET=Path(__file__).resolve().parent
REPO=PACKET.parents[3]
def main():
    predecessor=REPO/'docs/research/evidence/joint_articulated_20260907/reproduce.py'
    spec=importlib.util.spec_from_file_location('joint_contact_replay',predecessor)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.PACKET=PACKET;module.__file__=__file__
    module.main()
    # CLI deliberately reuses the verified predecessor's exact argument set.
    output=Path(sys.argv[sys.argv.index('--out')+1])/'result.json'
    result=json.loads(output.read_text())
    run=Path(sys.argv[1]);run=run if run.is_absolute() else REPO/run
    log=run/'controller.log'
    rows=[]
    for line in log.read_text().splitlines():
        if not line.startswith('JointShadow id='):continue
        record=dict(token.split('=',1) for token in line.split()[1:] if '=' in token)
        required={'id','state','phase','period','clock','epoch','phase_residual_ns','events',
                  'combinations','feasible','failure','detail','elapsed_us','anchor_source','command_authority'}
        record['complete']=required.issubset(record)
        if record['complete']:
            for key in ('state','phase','period','elapsed_us'):
                if not math.isfinite(float(record[key])):raise ValueError('nonfinite shadow '+key)
            if record['command_authority']!='0':raise ValueError('shadow scope changed')
        rows.append(record)
    if not rows:raise ValueError('joint shadow capture coverage missing')
    valid=[r for r in rows if r['complete']]
    elapsed=sorted(float(r['elapsed_us']) for r in valid)
    def percentile(q):
        if not elapsed:return None
        x=q*(len(elapsed)-1);a=int(x);b=min(a+1,len(elapsed)-1)
        return elapsed[a]+(elapsed[b]-elapsed[a])*(x-a)
    details={}
    for r in rows:details[r.get('detail','missing')]=details.get(r.get('detail','missing'),0)+1
    shadow={'scope':'conditional reduced-model proposal; no command or contact-evolution certificate',
            'captures':len(rows),'complete_records':len(valid),
            'reduced_feasible':sum(r.get('feasible')=='1' for r in valid),
            'failure_details':details,'elapsed_us':{'p50':percentile(.5),'p95':percentile(.95),'max':max(elapsed) if elapsed else None},
            'controller_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'records':rows}
    attempts=sorted(float(r['elapsed_us']) for r in valid if int(r['combinations'])>0)
    def ap(q):
        if not attempts:return None
        x=q*(len(attempts)-1);a=int(x);b=min(a+1,len(attempts)-1)
        return attempts[a]+(attempts[b]-attempts[a])*(x-a)
    shadow['solver_attempt_captures']=len(attempts)
    shadow['solver_attempt_pipeline_elapsed_us']={'p50':ap(.5),'p95':ap(.95),'max':max(attempts) if attempts else None}
    result['joint_shadow']=shadow
    result['scripts']['joint_contact_replay']={'path':str(predecessor.relative_to(REPO)),
            'sha256':hashlib.sha256(predecessor.read_bytes()).hexdigest()}
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
if __name__=='__main__':main()
