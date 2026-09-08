"""Research C ABI adapter; native evaluation uses the unchanged MuJoCo model."""
import ctypes as C
import pathlib
import mujoco,numpy as np
from coupled_horizon_shooting import HorizonInputError,HorizonNumericalError
P=C.POINTER(C.c_double)
class NativeHorizon:
 def __init__(self,library,scene,initial,baseline,terminal_vy,privileged=False):
  self.handle=None
  if not privileged:raise HorizonInputError('observed terrain horizon coverage unavailable')
  self.lib=C.CDLL(str(pathlib.Path(library).resolve()));self.handle=None
  self.lib.gh_create.argtypes=[C.c_char_p,P,C.c_int,P,C.c_double,C.c_char_p,C.c_int];self.lib.gh_create.restype=C.c_void_p
  self.lib.gh_destroy.argtypes=[C.c_void_p];self.lib.gh_destroy.restype=None
  self.lib.gh_gsize.argtypes=[C.c_void_p];self.lib.gh_gsize.restype=C.c_int
  self.lib.gh_eval.argtypes=[C.c_void_p,P,P,P,C.c_int,C.c_char_p,C.c_int];self.lib.gh_eval.restype=C.c_int
  state=np.ascontiguousarray(initial,dtype=np.float64);base=np.ascontiguousarray(baseline,dtype=np.float64)
  if state.ndim!=1 or base.shape!=(16,12) or not np.all(np.isfinite(np.r_[state,base.ravel()])):raise HorizonInputError('invalid native initial state/control coverage')
  error=C.create_string_buffer(2048)
  self.handle=self.lib.gh_create(str(pathlib.Path(scene).resolve()).encode(),state.ctypes.data_as(P),len(state),base.ctypes.data_as(P),terminal_vy,error,len(error))
  if not self.handle:raise HorizonInputError(error.value.decode())
  self.gsize=self.lib.gh_gsize(self.handle)
  if self.gsize!=1540:self.close();raise HorizonInputError('native constraint coverage mismatch')
 def close(self):
  if self.handle:self.lib.gh_destroy(self.handle);self.handle=None
 def __del__(self):self.close()
 def evaluate(self,controls):
  if not self.handle:raise HorizonInputError('closed native evaluator')
  u=np.ascontiguousarray(controls,dtype=np.float64)
  if u.shape!=(16,12) or not np.all(np.isfinite(u)):raise HorizonInputError('native control coverage')
  cost=C.c_double();g=np.empty(self.gsize);error=C.create_string_buffer(2048)
  status=self.lib.gh_eval(self.handle,u.ctypes.data_as(P),C.byref(cost),g.ctypes.data_as(P),self.gsize,error,len(error))
  if status==1:raise HorizonInputError(error.value.decode())
  if status:raise HorizonNumericalError(error.value.decode())
  if not np.isfinite(cost.value) or not np.all(np.isfinite(g)):raise HorizonNumericalError('nonfinite native output')
  return cost.value,g
