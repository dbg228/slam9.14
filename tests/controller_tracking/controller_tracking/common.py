"""Shared SI/SE(2), gap-bounded interpolation and segment error math.

Migrated from the retired diagnostic package; used by online and offline tools.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False).encode()).hexdigest()


def transform(points, pose):
    a = np.asarray(points, dtype=float).copy()
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x, y = a[:, 0].copy(), a[:, 1].copy()
    a[:, 0], a[:, 1] = pose[0]+c*x-s*y, pose[1]+s*x+c*y
    a[:, 2] = wrap(a[:, 2]+pose[2])
    return a


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def interpolate(rows, times, max_gap):
    """Rows [time,x,y,yaw]; no extrapolation, invalid rows split support."""
    a = np.asarray(rows, float).reshape(-1, 4)
    out = np.full((len(times), 3), np.nan)
    if not len(a): return out
    if np.any(np.diff(a[:, 0])<=0): raise ValueError('Nonmonotonic or duplicate timestamps')
    for n, t in enumerate(times):
        j = np.searchsorted(a[:, 0], t)
        if j<len(a) and abs(a[j, 0]-t)<1e-7:
            out[n] = a[j, 1:]; continue
        if j==0 or j==len(a): continue
        lo, hi = a[j-1], a[j]
        if hi[0]-lo[0]>max_gap or not np.all(np.isfinite([lo,hi])): continue
        f = (t-lo[0])/(hi[0]-lo[0])
        out[n, :2] = lo[1:3]+f*(hi[1:3]-lo[1:3])
        out[n, 2] = wrap(lo[3]+f*wrap(hi[3]-lo[3]))
    return out


def project(points, path):
    """Signed segment projection, tangent heading interpolated along each segment."""
    p = np.asarray(path['points']); ss=np.asarray(path['local']['s'])
    a=p[:-1, :2]; delta=np.diff(p[:, :2], axis=0); length2=np.sum(delta**2, axis=1)
    result=[]
    for x, y, yaw in points:
        if not np.all(np.isfinite([x,y,yaw])): result.append([np.nan]*4); continue
        f=np.clip(np.sum((np.array([x,y])-a)*delta, axis=1)/length2, 0, 1)
        q=a+f[:, None]*delta; dist=np.sum((q-[x,y])**2, axis=1); j=np.argmin(dist)
        heading=p[j,2]+f[j]*wrap(p[j+1,2]-p[j,2])
        lateral=-(x-q[j,0])*math.sin(heading)+(y-q[j,1])*math.cos(heading)
        result.append([ss[j]+f[j]*(ss[j+1]-ss[j]), lateral, wrap(yaw-heading), math.sqrt(dist[j])])
    return np.asarray(result)


def stats(values):
    a=np.asarray(values,float); a=a[np.isfinite(a)]
    if not len(a): return dict(n=0, rmse=None, p95_abs=None, max_abs=None, mean=None, std=None)
    return dict(n=len(a), rmse=float(np.sqrt(np.mean(a*a))), p95_abs=float(np.percentile(abs(a),95)),
                max_abs=float(max(abs(a))), mean=float(np.mean(a)), std=float(np.std(a)))
