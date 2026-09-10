"""Render evidence plots and a labeled data animation, not a robot simulation."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio.v2 as imageio
root=Path('/tmp/go2-report-20260910');out=root/'delivery';out.mkdir(exist_ok=True)
names=['steps_20260825_083650','accel_1_to_3_20260825_083849','brake_3_to_0_20260825_083947','ramp_20260825_084038','varying_20260825_084151']
fig,axes=plt.subplots(5,1,figsize=(12,13),layout='constrained')
for ax,name in zip(axes,names):
 d=np.load(root/(name+'.npz'));k=slice(None,None,10)
 for key,color in [('requested','#777777'),('shaped','#d18a00'),('measured','#176ab0')]:ax.plot(d['time'][k],d[key][k],label=key,color=color,lw=1.2)
 ax.set_title(name+' | '+('settling FAIL' if name.startswith(('steps','varying')) else 'settling check PASS'),loc='left',fontsize=11)
 ax.set_ylabel('Speed (m/s)');ax.set_xlabel('Command-relative time (s)');ax.grid(alpha=.2);ax.legend(loc='upper left',ncol=3,fontsize=8)
fig.suptitle('Historical online velocity tests: raw telemetry re-audit, 2026-09-10\nLinear-interpolated commands; no new physics runs',fontsize=15)
fig.savefig(out/'velocity_profiles.png',dpi=150);plt.close(fig)
d=np.load(root/(names[0]+'.npz'));fig,ax=plt.subplots(figsize=(10,5),dpi=100)
for key,color in [('requested','#777777'),('shaped','#d18a00'),('measured','#176ab0')]:ax.plot(d['time'][::10],d[key][::10],label=key,color=color,lw=1)
ax.set(xlabel='Command-relative time (s)',ylabel='Speed (m/s)',title='Recorded velocity telemetry | NOT a robot video\nSteps profile: settling re-audit FAIL');ax.legend();ax.grid(alpha=.2);line=ax.axvline(0,color='red');fig.tight_layout()
with imageio.get_writer(out/'velocity_trace.mp4',fps=15,codec='libx264',macro_block_size=2) as w:
 for t in np.linspace(0,d['time'][-1],300):
  line.set_xdata([t,t]);fig.canvas.draw();w.append_data(np.asarray(fig.canvas.buffer_rgba())[:,:,:3])
plt.close(fig)
