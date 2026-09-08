"""Privileged fixed-prefix, genuinely coupled 32ms MuJoCo horizon diagnostic.
All free torques are optimized together. No new dynamics model, runtime command
producer or B1 claim. Existing rolling candidate is initialization/comparator.
"""
import argparse,copy,fcntl,hashlib,json,lzma,pathlib,subprocess,time
import mujoco,numpy as np
from coupled_horizon_shooting import solve,HorizonInputError,HorizonNumericalError
from whole_body_cycle import dependencies
class ActualModelHorizon:
    def __init__(self,model,initial,baseline,prefix,terminal_vy,privileged=False):
        if not privileged:raise HorizonInputError('observed terrain horizon coverage unavailable')
        if model.nq!=19 or model.nv!=18 or model.nu!=12 or model.na or model.nplugin:
            raise HorizonInputError('requires existing direct-torque Go2 model')
        if abs(model.opt.timestep-.002)>1e-15:raise HorizonInputError('unchanged2ms model required')
        if not np.isfinite(terminal_vy) or terminal_vy<=0:raise HorizonInputError('finite positive terminal bound required')
        self.m=model;self.initial=copy.copy(initial);self.baseline=np.asarray(baseline)
        self.prefix=prefix;self.terminal_vy=terminal_vy
        self.va=model.jnt_dofadr[model.actuator_trnid[:,0]]
        self.qa=model.jnt_qposadr[model.actuator_trnid[:,0]]
        self.gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ('FR','FL','RR','RL')]
        if min(self.gids)<0:raise HorizonInputError('foot geometry missing')
    def contact(self,d):
        forces=np.zeros(4);nonfoot=0.
        for i in range(d.ncon):
            c=d.contact[i];f=np.empty(6);mujoco.mj_contactForce(self.m,d,i,f)
            legs=[l for l,g in enumerate(self.gids) if g in (c.geom1,c.geom2)]
            for l in legs:forces[l]+=f[0]
            allowed=len(legs)==1
            if allowed:
                foot=self.gids[legs[0]];other=c.geom2 if c.geom1==foot else c.geom1
                allowed=self.m.body_rootid[self.m.geom_bodyid[other]]!=self.m.body_rootid[self.m.geom_bodyid[foot]]
            if not allowed:nonfoot+=float(np.linalg.norm(f[:3]))
        return forces,nonfoot
    def observe(self,d):
        m=self.m;f,nonfoot=self.contact(d)
        w,x,y,z=d.qpos[3:7]
        roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1))
        ids=m.actuator_trnid[:,0];q=d.qpos[self.qa]
        g=np.r_[(180-f)/180,f/180,(30-abs(d.qvel[self.va]))/30,
                 q-m.jnt_range[ids,0],m.jnt_range[ids,1]-q,
                 d.qpos[2]-.28,np.pi/12-abs(roll),np.pi/12-abs(pitch),1e-6-nonfoot]
        return g,f,nonfoot
    def evaluate(self,u,record=False):
        m=self.m;d=copy.copy(self.initial);constraints=[];rows=[]
        if not np.array_equal(u[:self.prefix],self.baseline[:self.prefix]):
            raise HorizonInputError('fixed prefix altered')
        for k,tau in enumerate(u):
            pre=copy.copy(d);pre.ctrl[:]=tau;mujoco.mj_forward(m,pre)
            g,fpre,npre=self.observe(pre);constraints.append(g)
            d.ctrl[:]=tau;mujoco.mj_step(m,d);post=copy.copy(d);mujoco.mj_forward(m,post)
            for state in (pre,post):
                if any(state.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL)):
                    raise HorizonNumericalError('MuJoCo numerical reset/warning')
                if not np.all(np.isfinite(np.r_[state.qpos,state.qvel,state.qacc])):
                    raise HorizonNumericalError('nonfinite state')
            if abs(d.time-self.initial.time-(k+1)*m.opt.timestep)>1e-10:
                raise HorizonNumericalError('absolute clock mismatch')
            g,fpost,npost=self.observe(post);constraints.append(g)
            if record:rows.append({'time':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'pre_forces':fpre.tolist(),'post_forces':fpost.tolist(),'nonfoot_force':max(npre,npost),'control':tau.tolist()})
        constraints.append(np.r_[self.terminal_vy-abs(d.qvel[1]),.3-abs(d.qvel[3:6])])
        # Cost-to-go depends on ALL preceding free controls through actual dynamics.
        cost=.5*float(np.sum(((u-self.baseline)/35)**2))+.5*(d.qvel[1]/.02)**2
        g=np.concatenate(constraints)
        return (cost,g,rows) if record else (cost,g)
def main():
    p=argparse.ArgumentParser();p.add_argument('--rolling-result',required=True)
    p.add_argument('--scene',required=True);p.add_argument('--out',required=True)
    p.add_argument('--control-knots',type=int,default=0)
    p.add_argument('--candidate',type=int,default=1);p.add_argument('--terminal-vy',type=float,default=.02)
    p.add_argument('--wall-budget-s',type=float,default=60);p.add_argument('--max-iterations',type=int,default=30)
    p.add_argument('--privileged-scene-oracle',action='store_true');a=p.parse_args()
    source=pathlib.Path(a.rolling_result);raw=source.read_bytes();run=json.loads(lzma.decompress(raw) if source.suffix=='.xz' else raw)
    c=next(c for c in run['candidates'] if c['version']==a.candidate)
    if not c['valid'] or len(c['stages'])!=16:raise HorizonInputError('complete fixed12ms +20ms candidate required')
    with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        model=mujoco.MjModel.from_xml_path(a.scene);initial=mujoco.MjData(model)
        mujoco.mj_setState(model,initial,np.asarray(c['initial_integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
        controls=np.array([s['tau'] for s in c['stages']]);pb=ActualModelHorizon(model,initial,controls,6,a.terminal_vy,a.privileged_scene_oracle)
        basecost,baseg,baserows=pb.evaluate(controls,True)
        options=dict(max_iterations=a.max_iterations,wall_budget_s=a.wall_budget_s,
                     constraint_tolerance=0.,finite_difference_step=1e-4)
        if a.control_knots:
            if not 1<=a.control_knots<=10:raise HorizonInputError('control knots must be1..10')
            knots=np.linspace(0,1,a.control_knots);stages=np.linspace(0,1,10)
            weights=np.array([np.interp(stages,knots,np.eye(a.control_knots)[j]) for j in range(a.control_knots)]).T
            def expanded(parameters):
                u=controls.copy();u[6:]+=weights@parameters;return u
            def parameter_evaluation(parameters):
                u=expanded(parameters);cost,g=pb.evaluate(u)
                return cost,np.r_[g,(35-u).ravel()/35,(35+u).ravel()/35]
            result=solve(parameter_evaluation,np.zeros((a.control_knots,12)),-35,35,**options)
            if result['controls'] is not None:
                parameters=result['controls'];result['parameters']=parameters.tolist();result['controls']=expanded(parameters)
            result['parameterization']='linear correction knots over existing initial control sequence'
            result['control_knots']=a.control_knots
        else:
            result=solve(pb.evaluate,controls,-35,35,fixed_prefix_steps=6,**options)
        rows=None
        if result['controls'] is not None:
            _,g,rows=pb.evaluate(result['controls'],True)
            result['fresh_constraint_violation']=float(np.max(np.maximum(-g,0)))
            result['controls']=result['controls'].tolist()
        files=dependencies(a.scene)|{source.resolve(),pathlib.Path(__file__).resolve(),pathlib.Path(__file__).with_name('coupled_horizon_shooting.py').resolve(),pathlib.Path(__file__).with_name('whole_body_cycle.py').resolve()}
        report={'schema':'coupled-full-model-horizon-v1','scope':'privileged initialized fixedprefix horizon; no runtime/B1 authority','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'hashes':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)},'scene':str(pathlib.Path(a.scene).resolve()),'mujoco_version':mujoco.__version__,'candidate_version':a.candidate,'prefix_steps':6,'tail_steps':10,'terminal_vy_bound':a.terminal_vy,'initial_integration_state':c['initial_integration_state'],'baseline_controls':controls.tolist(),'baseline_cost':basecost,'baseline_constraint_violation':float(np.max(np.maximum(-baseg,0))),'baseline_terminal_velocity':baserows[-1]['qvel'][:6],'solver':result,'rows':rows}
        with open(a.out,'x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
        print(json.dumps({'out':a.out,'baseline_terminal_velocity':report['baseline_terminal_velocity'],'solver':{k:v for k,v in result.items() if k!='controls'},'terminal_velocity':rows[-1]['qvel'][:6] if rows else None},indent=2))
if __name__=='__main__':main()
