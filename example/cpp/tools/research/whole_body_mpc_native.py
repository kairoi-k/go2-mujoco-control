"""Full-cycle research evaluator and independent Python dynamics replay.
Known-scene/full-state privilege is explicit; this has no runtime authority.
"""
import copy
import ctypes as C
import pathlib
import mujoco
import numpy as np
from coupled_horizon_shooting import HorizonInputError,HorizonNumericalError
from probe_coupled_mujoco_horizon import ActualModelHorizon
P=C.POINTER(C.c_double)
class WholeBodyMPC:
    def __init__(self,library,scene,initial,baseline,body_refs,foot_refs,prefix,privileged=False):
        self.handle=None
        if not privileged:raise HorizonInputError('observed terrain coverage unavailable')
        self.baseline=np.ascontiguousarray(baseline,dtype=np.float64)
        if self.baseline.ndim!=2 or self.baseline.shape[1]!=12:raise HorizonInputError('control shape')
        self.steps=len(self.baseline);self.prefix=prefix
        if not 1<=self.steps<=200 or not isinstance(prefix,int) or not 0<=prefix<=self.steps:raise HorizonInputError('horizon coverage')
        self.body=np.array(body_refs,dtype=np.float64,order='C',copy=True)
        self.feet=np.array(foot_refs,dtype=np.float64,order='C',copy=True)
        self.state=np.ascontiguousarray(initial,dtype=np.float64)
        if self.body.shape!=(self.steps,13) or self.feet.shape!=(self.steps,12) or self.state.ndim!=1:raise HorizonInputError('reference coverage')
        if not np.isfinite(np.r_[self.state,self.baseline.ravel(),self.body.ravel(),self.feet.ravel()]).all():raise HorizonInputError('nonfinite input')
        norms=np.linalg.norm(self.body[:,3:7],axis=1)
        if np.any(norms<=1e-12):raise HorizonInputError('zero quaternion')
        self.body[:,3:7]/=norms[:,None]
        self.model=mujoco.MjModel.from_xml_path(str(scene));self.initial=mujoco.MjData(self.model)
        if len(self.state)!=mujoco.mj_stateSize(self.model,mujoco.mjtState.mjSTATE_INTEGRATION):raise HorizonInputError('integration state coverage')
        mujoco.mj_setState(self.model,self.initial,self.state,mujoco.mjtState.mjSTATE_INTEGRATION)
        self.oracle=ActualModelHorizon(self.model,self.initial,self.baseline,prefix,.02,privileged=True)
        self.lib=C.CDLL(str(pathlib.Path(library).resolve()))
        self.lib.wm_create.argtypes=[C.c_char_p,P,C.c_int,C.c_int,C.c_int,P,P,P,C.c_char_p,C.c_int];self.lib.wm_create.restype=C.c_void_p
        self.lib.wm_destroy.argtypes=[C.c_void_p];self.lib.wm_destroy.restype=None
        self.lib.wm_gsize.argtypes=[C.c_void_p];self.lib.wm_gsize.restype=C.c_int
        self.lib.wm_eval.argtypes=[C.c_void_p,P,P,P,C.c_int,C.c_char_p,C.c_int];self.lib.wm_eval.restype=C.c_int
        e=C.create_string_buffer(2048)
        self.handle=self.lib.wm_create(str(pathlib.Path(scene).resolve()).encode(),self.state.ctypes.data_as(P),len(self.state),self.steps,prefix,self.baseline.ctypes.data_as(P),self.body.ctypes.data_as(P),self.feet.ctypes.data_as(P),e,len(e))
        if not self.handle:raise HorizonInputError(e.value.decode())
        self.gsize=self.lib.wm_gsize(self.handle)
        if self.gsize!=self.steps*96:self.close();raise HorizonInputError('native output coverage')
    def close(self):
        if self.handle:self.lib.wm_destroy(self.handle);self.handle=None
    def __del__(self):self.close()
    def controls(self,u):
        u=np.ascontiguousarray(u,dtype=np.float64)
        if u.shape!=self.baseline.shape or not np.isfinite(u).all():raise HorizonInputError('control coverage')
        if not np.array_equal(u[:self.prefix],self.baseline[:self.prefix]):raise HorizonInputError('committed prefix changed')
        return u
    def evaluate(self,u):
        if not self.handle:raise HorizonInputError('closed evaluator')
        u=self.controls(u);cost=C.c_double();g=np.empty(self.gsize);e=C.create_string_buffer(2048)
        status=self.lib.wm_eval(self.handle,u.ctypes.data_as(P),C.byref(cost),g.ctypes.data_as(P),self.gsize,e,len(e))
        if status==1:raise HorizonInputError(e.value.decode())
        if status:raise HorizonNumericalError(e.value.decode())
        if not np.isfinite(cost.value) or not np.isfinite(g).all():raise HorizonNumericalError('nonfinite native output')
        return cost.value,g
    def replay(self,u):
        u=self.controls(u);m=self.model;d=copy.copy(self.initial);gs=[];rows=[];cost=0.
        for k,tau in enumerate(u):
            pre=copy.copy(d);pre.ctrl[:]=tau;mujoco.mj_forward(m,pre)
            g,fp,npf=self.oracle.observe(pre);gs.append(g)
            d.ctrl[:]=tau;mujoco.mj_step(m,d);post=copy.copy(d);mujoco.mj_forward(m,post)
            for sample in (pre,post):
                if not np.isfinite(np.r_[sample.qpos,sample.qvel,sample.qacc]).all() or any(sample.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC,mujoco.mjtWarning.mjWARN_BADCTRL)):raise HorizonNumericalError('independent replay numerical failure')
            if abs(d.time-self.initial.time-(k+1)*m.opt.timestep)>1e-10:raise HorizonNumericalError('absolute clock mismatch')
            g,ff,nff=self.oracle.observe(post);gs.append(g)
            b=self.body[k];angle=np.empty(3);mujoco.mju_subQuat(angle,post.qpos[3:7],b[3:7])
            e=np.r_[(post.qpos[:3]-b[:3])/.025,angle/.10,(post.qvel[:3]-b[7:10])/.30,(post.qvel[3:6]-b[10:13])/.60,(post.geom_xpos[self.oracle.gids].ravel()-self.feet[k])/.025]
            cost+=float(e@e)+.01*float(np.sum(((tau-self.baseline[k])/35)**2))
            state=np.empty(len(self.state));mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_INTEGRATION)
            rows.append({'time':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'integration_state':state.tolist(),'control':tau.tolist(),'forces':np.r_[fp,ff].tolist(),'nonfoot_force':max(npf,nff)})
        return {'cost':.5*cost/self.steps,'g':np.concatenate(gs),'states':rows}
