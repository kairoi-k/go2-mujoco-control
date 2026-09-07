#!/usr/bin/env python3
"""Read-only registered contact-point model probe replay using prior audit core."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
PACKET=Path(__file__).resolve().parent
REPO=PACKET.parents[3]
PREDECESSOR=REPO/'docs/research/evidence/wbc_certificate_20260907/reproduce.py'
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--runtime-sha',required=True)
    parser.add_argument('--point-model',required=True,type=int,choices=(0,1))
    parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    spec=importlib.util.spec_from_file_location('registered_wbc_replay_core',PREDECESSOR)
    core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
    core.REPO=REPO;core.PACKET=PACKET;core.RUNTIME_SHA=args.runtime_sha;core.__file__=__file__
    old_rel=core.rel
    def rel(path):
        try:return old_rel(path)
        except ValueError:return str(path.resolve())
    core.rel=rel
    run=(REPO/args.run).resolve() if not args.run.is_absolute() else args.run.resolve()
    run_rel=run.relative_to(REPO)
    args.out.mkdir(parents=True,exist_ok=False)
    cert=core.run_analyzer(core.CERTIFICATE_SCRIPT,run_rel,args.out/'certificate.json',[],None)
    if cert['returncode']!=(0 if cert['json'].get('status')=='PASS' else 1):
        raise RuntimeError('certificate analyzer exit/status conflict')
    cycles=core.run_analyzer(core.CYCLE_SCRIPT,run_rel,args.out/'cycles.json',['--start','18','--end','24'],0)
    result=core.build_result(run_rel,run,run/'run_manifest.json',core.load_json(run/'run_manifest.json'),
        PACKET/'pre_run_binding.json',core.load_json(PACKET/'pre_run_binding.json'),
        cert['json'],cycles['json'],cert,cycles,18,24)
    with (run/'data.csv').open(newline='') as stream:
        reader=csv.DictReader(stream)
        if 'wbc_full_force_application_jacobian_used' not in reader.fieldnames:
            raise RuntimeError('missing force application provenance')
        checked=0;mismatch=0
        for row in reader:
            if None in row:raise RuntimeError('CSV column count mismatch')
            if row['wbc_full_cert_attempt_checked']=='1':
                checked+=1
                mismatch+=row['wbc_full_force_application_jacobian_used']!=str(args.point_model)
    if not checked or mismatch:raise RuntimeError('runtime point-model adoption mismatch')
    result['force_point_adoption']={'expected':args.point_model,'checked_rows':checked,'mismatched_rows':mismatch,
        'joint_planner_adoption':False}
    result['scripts']['imported_predecessor_reproduce']={'path':str(PREDECESSOR.relative_to(REPO)),
        'sha256':hashlib.sha256(PREDECESSOR.read_bytes()).hexdigest()}
    output=args.out/'result.json'
    with output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(output)
if __name__=='__main__':main()
