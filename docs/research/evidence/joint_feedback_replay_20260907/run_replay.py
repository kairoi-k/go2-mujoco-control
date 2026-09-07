#!/usr/bin/env python3
"""Exclusive, exact-source short feedback diagnostic; no traversal claim."""
import argparse, datetime, fcntl, hashlib, json, pathlib, subprocess
p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--snapshot',required=True);p.add_argument('--snapshot-runtime-sha',required=True)
a=p.parse_args();root=pathlib.Path(__file__).resolve().parents[4]
def git(*args):return subprocess.check_output(['git',*args],cwd=root,text=True).strip()
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
if not a.name.replace('_','').replace('-','').isalnum():raise SystemExit('invalid name')
if git('status','--porcelain'):raise SystemExit('clean source required')
snapshot=pathlib.Path(a.snapshot).resolve();binary=root/'example/cpp/build/replay_joint_shadow_snapshot';scene=root/'unitree_robots/go2/phase2_flat.xml'
run=root/'example/cpp/experiments/_runs'/a.name
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 run.mkdir(exist_ok=False)
 files=[snapshot,binary,scene,root/'simulate/mujoco/lib/libmujoco.so.3.3.6']
 files += [root/s for s in git('ls-files','example/cpp','unitree_robots/go2').splitlines() if (root/s).is_file()]
 binding={str(f):sha(f) for f in files}
 cmd=[str(binary),str(snapshot),'--closed-loop',str(scene),str(run/'feedback.csv')]
 manifest={'schema':'joint-feedback-replay-v1','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'runtime_sha':git('rev-parse','HEAD'),'source_clean':True,'snapshot_runtime_sha':a.snapshot_runtime_sha,'kind':'counterfactual_torque_only_recorded_state','b1_claim':False,'command':cmd,'hashes':binding}
 (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 with (run/'stdout.txt').open('x') as out:
  result=subprocess.run(cmd,cwd=root,stdout=out,stderr=subprocess.STDOUT)
 changed=[str(f) for f in files if sha(f)!=binding[str(f)]]
 (run/'result.json').write_text(json.dumps({'returncode':result.returncode,'input_files_changed':changed,'output_hashes':{f.name:sha(f) for f in run.iterdir() if f.is_file()}},indent=2)+'\n')
 print(run);print('returncode',result.returncode,'files_changed',len(changed))
 raise SystemExit(result.returncode if not changed else 3)
