"""Geometry-derived C1 surface-height envelope for a known box swing reference.
This is a soft objective generator, never a dynamic feasibility certificate.
The sphere's XY footprint is conservatively enclosed by its radius square.
"""
import numpy as np
def smooth(s):
    s=float(np.clip(s,0,1));return s*s*(3-2*s)
def surface_envelope(progress,points,box_xy,top,radius,start_height,end_height):
    s=np.asarray(progress,dtype=float);xy=np.asarray(points,dtype=float);box=np.asarray(box_xy,dtype=float)
    if s.ndim!=1 or len(s)<2 or xy.shape!=(len(s),2) or box.shape!=(2,2):raise ValueError('swing geometry coverage')
    if not np.isfinite(np.r_[s,xy.ravel(),box.ravel(),top,radius,start_height,end_height]).all():raise ValueError('nonfinite swing geometry')
    if s[0]!=0 or s[-1]!=1 or np.any(np.diff(s)<=0) or np.any(box[1]<=box[0]) or radius<=0 or min(top,start_height,end_height)<0:raise ValueError('invalid swing geometry')
    lower=box[0]-radius;upper=box[1]+radius;intervals=[]
    for k,(a,b) in enumerate(zip(xy[:-1],xy[1:])):
        lo,hi=0.,1.;delta=b-a
        for axis in range(2):
            if abs(delta[axis])<1e-15:
                if not lower[axis]<=a[axis]<=upper[axis]:lo,hi=1.,0.;break
            else:
                t0,t1=sorted(((lower[axis]-a[axis])/delta[axis],(upper[axis]-a[axis])/delta[axis]))
                lo=max(lo,t0);hi=min(hi,t1)
        if lo<=hi:intervals.append((float(s[k]+lo*(s[k+1]-s[k])),float(s[k]+hi*(s[k+1]-s[k]))))
    if not intervals:return {'entry':None,'exit':None,'peak':max(start_height,end_height),'start':start_height,'end':end_height}
    entry=min(a for a,b in intervals);exit_=max(b for a,b in intervals);peak=max(top,start_height,end_height)
    if entry<=1e-12 and start_height<peak-1e-12:raise ValueError('swing starts in expanded obstacle footprint below support height')
    if exit_>=1-1e-12 and end_height<peak-1e-12:raise ValueError('swing ends in expanded obstacle footprint below support height')
    return {'entry':entry,'exit':exit_,'peak':peak,'start':start_height,'end':end_height}
def height_at(envelope,s):
    if not np.isfinite(s) or not 0<=s<=1:raise ValueError('swing progress coverage')
    a,b=envelope['entry'],envelope['exit'];h0,h1,peak=envelope['start'],envelope['end'],envelope['peak']
    if a is None:return h0+(h1-h0)*smooth(s)
    if s<a:return h0+(peak-h0)*smooth(s/a)
    if s>b:return peak+(h1-peak)*smooth((s-b)/(1-b))
    return peak
