"""The single implementation of 3D mount conversion, frame freezing and velocity fitting."""
import math
import numpy as np
from .common import interpolate, wrap


def rotation(q):
    q=np.asarray(q,float)
    if q.shape!=(4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q)<1e-9: raise ValueError('invalid_quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def base_in_level(xyz,quat,mount,level_quat,max_tilt_deg=10):
    if mount.get('measured') is not True: raise ValueError('unmeasured_rigid_body_mount')
    r=rotation(quat); level=rotation(level_quat)
    xyz=level@(np.asarray(xyz,float)+r@np.asarray(mount['translation_m'],float))
    r=level@r@rotation(mount['quaternion_xyzw'])
    if not np.all(np.isfinite(xyz)) or math.degrees(math.acos(float(np.clip(r[2,2],-1,1))))>max_tilt_deg:
        raise ValueError('invalid_pose_or_ground_tilt')
    return np.array([xyz[0],xyz[1],math.atan2(r[1,0],r[0,0])]),float(xyz[2])


def to_test(p,anchor):
    c,s=math.cos(anchor[2]),math.sin(anchor[2]); dx,dy=p[0]-anchor[0],p[1]-anchor[1]
    return np.array([c*dx+s*dy,-s*dx+c*dy,wrap(p[2]-anchor[2])])


def freeze_anchor(rows,position_limit=.01,yaw_limit=.02):
    a=np.asarray(rows,float)
    if len(a)<20 or a[-1,0]-a[0,0]<.5: raise ValueError('need_at_least_0.5s_stationary_valid_data')
    angles=np.unwrap(a[:,3]); anchor=[float(np.mean(a[:,1])),float(np.mean(a[:,2])),float(wrap(np.mean(angles)))]
    if max(np.linalg.norm(a[:,1:3]-anchor[:2],axis=1))>position_limit or max(abs(wrap(a[:,3]-anchor[2])))>yaw_limit:
        raise ValueError('not_stationary_while_freezing')
    return anchor


class VelocityFit:
    """Causal trailing linear fit; pose and body speed share midpoint measurement time.

    No extrapolation. A bad timestamp or large gap discards the entire window.
    Velocity is an estimate, not a motor/VRPN velocity measurement.
    """
    def __init__(self,window=.10,gap=.05):
        if window<=0 or gap<=0:raise ValueError('invalid_window_or_gap')
        self.window=window;self.gap=gap;self.rows=[];self.resets=0

    def reset(self):
        self.rows=[];self.resets+=1

    def add(self,t,p):
        if not np.all(np.isfinite([t,*p])):self.reset();raise ValueError('nonfinite_sample')
        if self.rows and not 0<t-self.rows[-1][0]<=self.gap:
            self.reset();raise ValueError('duplicate_backward_or_gap')
        self.rows.append([t,*p])
        while len(self.rows)>2 and self.rows[1][0]<=t-self.window:self.rows.pop(0)
        a=np.asarray(self.rows)
        if len(a)<5 or a[-1,0]-a[0,0]<self.window*.9:return None
        center=(a[0,0]+a[-1,0])/2
        # Uniform grid avoids overweighting receive-time bursts.
        grid=np.linspace(a[0,0],a[-1,0],max(5,int(self.window*100)+1))
        poses=interpolate(a,grid,self.gap)
        if not np.all(np.isfinite(poses)):return None
        tt=grid-center; coeff=tt@poses[:,:2]/(tt@tt)
        wz=float(tt@np.unwrap(poses[:,2])/(tt@tt))
        pose=interpolate(a,[center],self.gap)[0]
        c,s=math.cos(pose[2]),math.sin(pose[2])
        return center,pose,np.array([c*coeff[0]+s*coeff[1],-s*coeff[0]+c*coeff[1],wz]),t-center
