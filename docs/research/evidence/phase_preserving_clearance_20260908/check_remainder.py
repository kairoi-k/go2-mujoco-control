#!/usr/bin/env python3
"""Independent scalar Hermite remainder identity, not robot feasibility."""
import json,math
peak=0.
for phase in (.05,.5,.9):
 for i in range(101):
  u=i/100;s=phase+(1-phase)*u
  B=lambda x:16*x*x*(1-x)**2
  Bd=lambda x:32*x-96*x*x+64*x*x*x
  residual=B(s)-(2*u**3-3*u*u+1)*B(phase)-(u**3-2*u*u+u)*(1-phase)*Bd(phase)
  peak=max(peak,abs(residual-(1-phase)**4*B(u)))
assert peak<1e-12
print(json.dumps({'max_remainder_identity_error':peak,'phases':[.05,.5,.9],'samples_per_phase':101,'scope':'scalar curve identity only'},indent=2))
