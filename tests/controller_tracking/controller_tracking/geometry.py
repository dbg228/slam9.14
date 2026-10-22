"""Arc-length paths with C2 position continuity and continuous curvature."""
import math
import numpy as np
from .common import digest, transform


def geometry(cfg, kind=None):
    g = cfg['geometry']; kind = kind or g['kind']; ds = float(g['ds'])
    values = [float(v) for k, v in g.items() if k != 'kind']
    if not all(math.isfinite(v) for v in values) or ds <= 0 or ds > .1:
        raise ValueError('Invalid geometry or sampling distance')
    if min(g['body_length'], g['body_width']) <= 0 or min(g['lead'],g['tail'],g['stop_margin'],g['sweep_margin']) < 0:
        raise ValueError('Invalid footprint or margins')
    if kind == 'line':
        total = float(g['length'])
        if total <= 0: raise ValueError('Line length must be positive')
        s = np.unique(np.r_[np.arange(0, total, ds), total])
        points = np.c_[s, s*0, s*0]; k = s*0; phases = ['line']*len(s)
        bounds = []
    elif kind in ('left_arc', 'right_arc'):
        radius, angle, ramp = float(g['radius']), math.radians(g['angle_deg']), float(g['transition_length'])
        if radius <= 0 or not 0 < angle < math.pi or ramp <= 0 or radius*angle < ramp:
            raise ValueError('Require 0<angle<180deg and radius*angle>=transition_length>0')
        central = radius*angle-ramp; lead,tail=g['lead'],g['tail']
        bounds = [lead, lead+ramp, lead+ramp+central, lead+2*ramp+central]
        total=bounds[-1]+tail
        if total/ds>50000: raise ValueError('Excessive path length')
        # Integrate analytic k(s) at high resolution; sample by s, not by a geometric parameter.
        dense=np.unique(np.r_[np.linspace(0,total,max(10001,int(total/ds)*20)),bounds,total])
        def curvature(ss):
            z=ss-lead; value=np.zeros_like(z)
            entry=(z>=0)&(z<ramp); mid=(z>=ramp)&(z<=ramp+central)
            exit=(z>ramp+central)&(z<=2*ramp+central)
            value[entry]=.5*(1-np.cos(np.pi*z[entry]/ramp))/radius
            value[mid]=1/radius
            value[exit]=.5*(1+np.cos(np.pi*(z[exit]-ramp-central)/ramp))/radius
            return value*(1 if kind=='left_arc' else -1)
        kd=curvature(dense); dd=np.diff(dense)
        yaw=np.r_[0,np.cumsum((kd[:-1]+kd[1:])*dd/2)]
        x=np.r_[0,np.cumsum((np.cos(yaw[:-1])+np.cos(yaw[1:]))*dd/2)]
        y=np.r_[0,np.cumsum((np.sin(yaw[:-1])+np.sin(yaw[1:]))*dd/2)]
        s=np.unique(np.r_[np.arange(0,total,ds),bounds,total])
        points=np.c_[np.interp(s,dense,x),np.interp(s,dense,y),np.interp(s,dense,yaw)]
        k=curvature(s)
        phases=np.array(['lead','entry','arc','exit','tail'])[np.searchsorted(bounds,s,side='right')].tolist()
    else: raise ValueError('Only line, left_arc, right_arc are supported')
    local=dict(points=points.tolist(),s=s.tolist(),curvature=k.tolist(),phase=phases,
               boundaries_s=bounds,total_length=total)
    result=dict(schema=2,kind=kind,frame=cfg['frame'],reference_point=cfg['base_frame'],
                points=local['points'],local=local,config_sha256=digest(cfg))
    result['points_sha256']=digest(result['points'])
    return result


def swept(path,cfg):
    g=cfg['geometry']; maximum=max(abs(np.asarray(path['local']['curvature'])))
    pad=g['sweep_margin']+g['ds']*(1+maximum*math.hypot(g['body_length'],g['body_width'])/2)
    corners=np.array([[x,y,0] for x in (-g['body_length']/2-pad,g['body_length']/2+pad)
                      for y in (-g['body_width']/2-pad,g['body_width']/2+pad)])
    poses=list(path['points']); end=np.array(poses[-1]).copy()
    end[:2]+=g['stop_margin']*np.array([math.cos(end[2]),math.sin(end[2])]); poses.append(end)
    return np.array([transform(corners,p)[:,:2] for p in poses])


def site_check(path,cfg):
    xy=swept(path,cfg).reshape(-1,2); result={}
    for name in ('stable_mocap_bounds','clear_floor_bounds'):
        b=cfg['site'].get(name)
        valid=bool(b and len(b)==4 and all(math.isfinite(float(x)) for x in b) and b[0]<b[1] and b[2]<b[3])
        result[name]=bool(valid and np.all((xy[:,0]>=b[0])&(xy[:,0]<=b[1])&(xy[:,1]>=b[2])&(xy[:,1]<=b[3])))
    for name in ('marker_visibility_verified','stopping_margin_verified','physical_stop_verified'):
        result[name]=cfg['site'].get(name) is True
    return result


def endpoint_diagnostic(path,production):
    p=production['controller_server']['ros__parameters']['FollowPath']
    pts=np.asarray(path['points']); d=np.linalg.norm(pts[:,:2]-pts[-1,:2],axis=1)
    curved=abs(np.asarray(path['local']['curvature']))>1e-5
    ds=np.diff(path['local']['s']); weights=np.r_[ds,0]
    out={'basis':'distance to full endpoint; MPPI processed/pruned endpoint can differ',
         'constant_speed_guaranteed':False,'goal_checker':production['controller_server']['ros__parameters']['goal_checker']}
    for name in ('PathFollowCritic','GoalCritic'):
        threshold=p[name]['threshold_to_consider']; denominator=float(weights[curved].sum())
        out[name]={'threshold_m':threshold,'curved_length_within_threshold_fraction':
                  float(weights[curved&(d<threshold)].sum()/denominator) if denominator else None}
    return out
