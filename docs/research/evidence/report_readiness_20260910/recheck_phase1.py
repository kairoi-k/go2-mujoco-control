"""Read-only historical settling re-audit. Extract selected files from phase1.tar.gz first."""
import argparse,concurrent.futures,hashlib,json,subprocess,sys
from pathlib import Path
def main():
 p=argparse.ArgumentParser();p.add_argument('--raw-root',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 repo=Path(__file__).resolve().parents[4];a.out.mkdir(parents=True,exist_ok=False)
 runs=sorted(a.raw_root.rglob('data.csv'))
 if len(runs)!=15: raise ValueError(f'Expected 15 retained runs, got {len(runs)}')
 def one(csv):
  run=csv.parent; scenario=run.name.split('_20260825_')[0];dest=a.out/(run.name+'.json')
  profile=repo/'example/cpp/configs'/('phase1_velocity_'+scenario+'.csv')
  cmd=[sys.executable,str(repo/'example/cpp/tools/analysis/audit_phase1_settling.py'),str(run),'--profile',str(profile),'--json-out',str(dest)]
  proc=subprocess.run(cmd,capture_output=True,text=True)
  (a.out/(run.name+'.log')).write_text(proc.stdout+proc.stderr)
  if proc.returncode not in (0,1) or not dest.exists():raise RuntimeError(proc.stderr)
  data=json.loads(dest.read_text());old=json.loads((run/'phase1_quantitative_analysis.json').read_text())
  result={'run':run.name,'scenario':scenario,'audit_status':data['audit_status'],'audit_exit':proc.returncode,'audit':data,'legacy':json.loads(json.dumps(old),parse_constant=lambda value: 'NONFINITE:'+value),'raw_sha256':{x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in run.iterdir() if x.is_file()},'profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest()}
  print(run.name,data['audit_status'],flush=True);return result
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool: results=list(pool.map(one,runs))
 (a.out/'summary.json').write_text(json.dumps({'schema':'report-phase1-reaudit-v1','results':results},indent=2,allow_nan=False)+'\n')
if __name__=='__main__':main()
