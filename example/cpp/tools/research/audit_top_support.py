"""Check top-support semantics against retained side-wall collision witness."""
import argparse,fcntl,json,pathlib,hashlib,subprocess
import numpy as np
from whole_body_mpc_native import WholeBodyMPC
def main():
 p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--run',required=True);p.add_argument('--out',required=True);a=p.parse_args();r=json.loads(pathlib.Path(a.run).read_text());results=[]
 with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  for index in (0,25):
   c=r['chunks'][index];u=np.array(c['candidate_controls']);args=(a.library,r['source']['scene'],c['initial_integration_state'],c['baseline_controls'],c['body_refs'],c['foot_refs'],5,True)
   legacy=WholeBodyMPC(*args);top=WholeBodyMPC(*args,top_support_only=True)
   lc,lg=legacy.evaluate(u);tc,tg=top.evaluate(u);py=top.replay(u)
   assert np.max(abs(tg-py['g']))<=1e-9 and abs(tc-py['cost'])<=1e-10 and lc==tc
   assert min(lg)>=0
   assert (min(tg)>=0) if index==0 else (min(tg)<-1.)
   results.append({'chunk':index,'legacy_min_g':float(min(lg)),'top_support_min_g':float(min(tg)),'native_python_g_delta':float(max(abs(tg-py['g']))),'cost_delta':abs(tc-py['cost'])});legacy.close();top.close()
 report={'schema':'top-support-audit-v1','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'run_sha256':hashlib.sha256(pathlib.Path(a.run).read_bytes()).hexdigest(),'library_sha256':hashlib.sha256(pathlib.Path(a.library).read_bytes()).hexdigest(),'cases':results,'scope':'counterexample and flat-control fixture; not traversal acceptance'}
 with open(a.out,'x') as f:json.dump(report,f,indent=2)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
