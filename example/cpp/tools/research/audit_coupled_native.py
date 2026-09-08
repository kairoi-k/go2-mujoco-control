"""Matched native/Python residual and latency check; no optimization/actuation."""
import argparse,fcntl,hashlib,json,pathlib,subprocess,time
import mujoco,numpy as np
from coupled_horizon_native import NativeHorizon
from coupled_horizon_shooting import HorizonInputError
from probe_coupled_mujoco_horizon import ActualModelHorizon
def main():
 p=argparse.ArgumentParser();p.add_argument('result');p.add_argument('--library',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 r=json.loads(pathlib.Path(a.result).read_text());hashfile=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
 for f,h in r['hashes'].items():
  if hashfile(f)!=h:raise ValueError('input hash mismatch '+f)
 baseline=np.asarray(r['baseline_controls']);witness=np.asarray(r['solver']['controls']);rng=np.random.default_rng(20260908)
 cases=[baseline,witness]
 for _ in range(8):
  u=witness.copy();u[6:]+=rng.uniform(-.01,.01,(10,12));cases.append(u)
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  model=mujoco.MjModel.from_xml_path(r['scene']);d=mujoco.MjData(model);mujoco.mj_setState(model,d,np.asarray(r['initial_integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
  py=ActualModelHorizon(model,d,baseline,6,r['terminal_vy_bound'],True);native=NativeHorizon(a.library,r['scene'],r['initial_integration_state'],baseline,r['terminal_vy_bound'],True)
  comparisons=[]
  for u in cases:
   pc,pg=py.evaluate(u);nc,ng=native.evaluate(u)
   comparisons.append({'cost_delta':abs(pc-nc),'constraint_delta':float(np.max(abs(pg-ng))),'same_feasibility':bool(np.all(pg>=0)==np.all(ng>=0))})
  first=native.evaluate(witness);again=native.evaluate(witness);deterministic=first[0]==again[0] and np.array_equal(first[1],again[1])
  errors={}
  for name,u in [('prefix_conflict',baseline.copy()),('nonfinite',baseline.copy()),('wrong_shape',baseline[:10])]:
   if name=='prefix_conflict':u[0,0]+=.001
   if name=='nonfinite':u[7,0]=np.nan
   try:native.evaluate(u);errors[name]=False
   except HorizonInputError:errors[name]=True
  try:NativeHorizon(a.library,r['scene'],r['initial_integration_state'],baseline,.02,False);errors['unknown_coverage']=False
  except HorizonInputError:errors['unknown_coverage']=True
  timing={}
  for name,evaluator in [('python',py.evaluate),('native',native.evaluate)]:
   evaluator(witness);times=[]
   for _ in range(30):
    start=time.perf_counter();evaluator(witness);times.append(1000*(time.perf_counter()-start))
   timing[name]={'ms':times,'p50':float(np.percentile(times,50)),'p95':float(np.percentile(times,95)),'max':max(times)}
  native.close()
  try:native.evaluate(witness);errors['closed']=False
  except HorizonInputError:errors['closed']=True
 passed=all(c['cost_delta']<=1e-12 and c['constraint_delta']<=1e-9 and c['same_feasibility'] for c in comparisons) and deterministic and all(errors.values())
 files=[a.result,a.library,__file__,pathlib.Path(__file__).with_name('coupled_horizon_native.py'),pathlib.Path(__file__).with_name('coupled_horizon_native.cpp'),pathlib.Path(__file__).with_name('probe_coupled_mujoco_horizon.py')]
 out={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'scope':'same complete MuJoCo integration,10control cases; latency is evaluator only, no solver or B1 claim','hashes':{str(pathlib.Path(f).resolve()):hashfile(f) for f in files},'mujoco':mujoco.__version__,'passed':bool(passed),'comparisons':comparisons,'deterministic':bool(deterministic),'fail_closed':errors,'timing':timing}
 with open(a.out,'x') as f:json.dump(out,f,indent=2,allow_nan=False)
 print(json.dumps({k:v for k,v in out.items() if k not in ('hashes','comparisons','timing')},indent=2));print(json.dumps({n:{k:v for k,v in t.items() if k!='ms'} for n,t in timing.items()},indent=2))
 if not passed:raise SystemExit(1)
if __name__=='__main__':main()
