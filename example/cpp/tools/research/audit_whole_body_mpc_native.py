"""Independent full-cycle native/Python equivalence on the actual 5cm scene."""
import argparse,copy,fcntl,hashlib,json,pathlib,subprocess,time
import mujoco,numpy as np
from whole_body_mpc_native import WholeBodyMPC
from whole_body_horizon_probe import packet
from coupled_horizon_shooting import HorizonInputError
from whole_body_cycle import dependencies
def main():
    p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--packet',required=True);p.add_argument('--scene',required=True);p.add_argument('--out',required=True);a=p.parse_args()
    out=pathlib.Path(a.out)
    if out.exists():raise ValueError('output exists')
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        manifest,refs,nominal=packet(a.packet);m=mujoco.MjModel.from_xml_path(a.scene);d=mujoco.MjData(m)
        initial=np.array(nominal['initial_integration_state']);u=np.array([r['tau'] for r in refs]);body=[];feet=[]
        gids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
        for r in refs:
            d.qpos[:]=r['qpos'];d.qvel[:]=r['qvel'];mujoco.mj_forward(m,d)
            body.append(np.r_[d.qpos[:7],d.qvel[:6]]);feet.append(d.geom_xpos[gids].ravel().copy())
        cases=[];checks=[]
        def expect_reject(name,fn):
            try:fn()
            except HorizonInputError:checks.append({'name':name,'pass':True});return
            raise AssertionError(name+' accepted')
        for n in (1,len(u)):
            prefix=min(5,n);pb=WholeBodyMPC(a.library,a.scene,initial,u[:n],body[:n],feet[:n],prefix,True)
            for perturbation in (0.,.001):
                v=u[:n].copy();v[prefix:]+=perturbation*np.sin(np.arange((n-prefix)*12).reshape(n-prefix,12))
                t=time.perf_counter();cost,g=pb.evaluate(v);latency=(time.perf_counter()-t)*1000
                replay=pb.replay(v);cd=abs(cost-replay['cost']);gd=float(max(abs(g-replay['g'])))
                cost2,g2=pb.evaluate(v)
                assert cd<=1e-10 and gd<=1e-9 and cost==cost2 and np.array_equal(g,g2)
                cases.append({'steps':n,'perturbation':perturbation,'cost':cost,'cost_delta':cd,'constraint_delta':gd,'strict_feasible':bool(min(g)>=0 and max(abs(v.ravel()))<=35),'min_constraint':float(min(g)),'native_ms':latency})
            changed=u[:n].copy();changed[0,0]+=1e-8
            expect_reject('prefix_'+str(n),lambda:pb.evaluate(changed))
            expect_reject('shape_'+str(n),lambda:pb.evaluate(u[:n,:11]))
            bad=u[:n].copy();bad[-1,-1]=np.nan
            expect_reject('nonfinite_'+str(n),lambda:pb.evaluate(bad))
            pb.close();expect_reject('closed_'+str(n),lambda:pb.evaluate(u[:n]))
        expect_reject('unknown_terrain',lambda:WholeBodyMPC(a.library,a.scene,initial,u,body,feet,5))
        badbody=np.array(body);badbody[0,3:7]=0
        expect_reject('zero_quaternion',lambda:WholeBodyMPC(a.library,a.scene,initial,u,badbody,feet,5,True))
        files=dependencies(a.scene)|{pathlib.Path(a.library).resolve(),pathlib.Path(a.packet).resolve()/'manifest.json',pathlib.Path(a.packet).resolve()/'trajectory.packet'}
        files|={pathlib.Path(__file__).resolve(),*[pathlib.Path(__file__).with_name(n).resolve() for n in ('whole_body_mpc_native.cpp','whole_body_mpc_native.py','probe_coupled_mujoco_horizon.py','whole_body_horizon_probe.py')]}
        result={'schema':'whole-body-mpc-native-audit-v1','scope':'known 5cm scene at privileged recorded initial state, no traversal or B1 claim','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'hashes':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)},'mujoco':mujoco.__version__,'cases':cases,'fail_closed':checks,'pass':True}
        with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
        print(json.dumps({k:v for k,v in result.items() if k!='hashes'},indent=2))
if __name__=='__main__':main()
