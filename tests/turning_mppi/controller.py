"""A tracking proposal and execution-aware, sampled predictive controller.

This is a research prototype of guided MPPI, not a claim of a new MPPI
importance-sampling derivation. All output selection uses the same rollout,
including the weighted mean: averaging collision-free controls can collide.
Units are metres, seconds and radians. No ROS or hardware command publisher.
"""
from dataclasses import asdict, dataclass
import math
import os
import numpy as np


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class Path:
    def __init__(self, points):
        p = np.asarray(points, dtype=float)
        if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or not np.isfinite(p).all():
            raise ValueError('Expected at least three finite x,y,yaw path points')
        keep = np.r_[True, np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1) > 1e-7]
        self.points = p[keep]
        self.s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1))]
        if len(self.s) < 3:
            raise ValueError('Path needs three distinct points')
        self.yaw = np.unwrap(self.points[:, 2])
        self.k = np.gradient(self.yaw, self.s)
        self.dk = np.gradient(self.k, self.s)
        self.length = float(self.s[-1])

    def at(self, s):
        return tuple(np.interp(s, self.s, a) for a in
                     (self.points[:, 0], self.points[:, 1], self.yaw, self.k, self.dk))

    def project(self, x, y, hint=None):
        # Ordered local search avoids jumping between nearby arms of a route.
        lo = 0 if hint is None else max(0, np.searchsorted(self.s, hint - .5) - 1)
        hi = len(self.s)-1 if hint is None else min(len(self.s)-1, np.searchsorted(self.s, hint + 1.5))
        a = self.points[lo:hi, :2]
        d = self.points[lo+1:hi+1, :2] - a
        f = np.clip(np.sum((np.array([x, y])-a)*d, axis=1)/np.sum(d*d, axis=1), 0, 1)
        q = a + f[:, None]*d
        j = int(np.argmin(np.sum((q-[x, y])**2, axis=1)))
        idx = lo+j
        s = self.s[idx]+f[j]*(self.s[idx+1]-self.s[idx])
        yaw = float(np.interp(s, self.s, self.yaw))
        delta = np.array([x,y])-q[j]
        return float(s), float(-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]), yaw


@dataclass(frozen=True)
class Config:
    dt: float = .05
    steps: int = 50
    samples: int = 1000
    speed: float = .20
    vmax: float = .20
    wmax: float = .45
    accel_v: float = .45
    accel_w: float = .80
    tau_v: float = .12
    tau_w: float = .25
    delay: float = .10
    temperature: float = .20
    noise_v: float = .05
    noise_w: float = .12
    lateral_gain: float = 1.5
    heading_gain: float = 2.0
    half_length: float = .48  # production footprint plus 0.03m padding
    half_width: float = .385
    clearance: float = .04


def clearance(x, y, yaw, obstacles, cfg):
    """Exact signed circle-to-oriented-rectangle distance (not point robot)."""
    out = np.full(np.shape(x), np.inf)
    c, s = np.cos(yaw), np.sin(yaw)
    for ox, oy, radius in obstacles:
        dx, dy = ox-x, oy-y
        bx, by = c*dx+s*dy, -s*dx+c*dy
        qx, qy = np.abs(bx)-cfg.half_length, np.abs(by)-cfg.half_width
        dist = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - radius
        out = np.minimum(out, dist)
    return out


def smooth_step(previous, command, cfg, dt=None):
    dt = cfg.dt if dt is None else dt
    u = np.clip(command, [0, -cfg.wmax], [cfg.vmax, cfg.wmax])
    return previous + np.clip(u-previous, -np.array([cfg.accel_v, cfg.accel_w])*dt,
                              np.array([cfg.accel_v, cfg.accel_w])*dt)


class GuidedMPPI:
    """Frenet tracker proposes controls; MPPI chooses them before actuation.

    mode='full' includes smoother, delay and first order actuator dynamics.
    mode='ideal' is an ablation with the same guidance/cost and ideal dynamics.
    mode='tracker' disables sampling, retaining the same feasibility filter.
    """
    def __init__(self, path, cfg=Config(), seed=0, mode='full'):
        if mode not in ('full', 'ideal', 'tracker'):
            raise ValueError(mode)
        if (not np.isfinite(list(asdict(cfg).values())).all() or
            not isinstance(cfg.steps,int) or not isinstance(cfg.samples,int) or
            cfg.steps < 1 or cfg.samples < 5 or
            min(cfg.dt,cfg.tau_v,cfg.tau_w,cfg.temperature,cfg.speed,cfg.vmax,cfg.wmax,
                cfg.accel_v,cfg.accel_w,cfg.half_length,cfg.half_width) <= 0 or
            min(cfg.noise_v,cfg.noise_w,cfg.delay,cfg.clearance) < 0 or cfg.speed > cfg.vmax):
            raise ValueError('Invalid prediction/sampling/dynamics configuration')
        self.path, self.cfg, self.mode = path, cfg, mode
        self.rng = np.random.default_rng(seed)
        self.s = 0.
        self.residual = np.zeros((cfg.steps, 2))
        self.last_diagnostics = {}

    def rollout(self, state, smooth, pending, residuals, obstacles, guide_only=False):
        if os.environ.get('TURNING_MPPI_BACKEND','cpp') == 'python':
            return self._rollout_python(state,smooth,pending,residuals,obstacles,guide_only)
        try:
            from .acceleration.fast_rollout import rollout
        except ImportError:
            from acceleration.fast_rollout import rollout
        return rollout(self,state,smooth,pending,residuals,obstacles,guide_only)

    def _rollout_python(self, state, smooth, pending, residuals, obstacles, guide_only=False):
        c = self.cfg
        count = len(residuals)
        x, y, yaw, v, w = [np.full(count, float(z)) for z in state]
        sv, sw = np.full(count, smooth[0]), np.full(count, smooth[1])
        progress = np.full(count, self.s)
        total = np.zeros(count)
        feasible = np.ones(count, dtype=bool)
        min_clearance = np.full(count, np.inf)
        commands = np.empty((count, c.steps, 2))
        delayed = []
        ideal = self.mode == 'ideal'
        delay_steps = 0 if ideal else len(pending)
        for t in range(c.steps):
            rx, ry, tangent, k, dk = self.path.at(progress)
            e = -np.sin(tangent)*(x-rx)+np.cos(tangent)*(y-ry)
            eh = wrap(yaw-tangent)
            # Derivative feedforward is the inverse of the assumed yaw lag.
            # Preview covers known queued command latency; no future plant truth.
            preview = progress + delay_steps*c.dt*np.maximum(v, .05)
            _, _, _, kp, dkp = self.path.at(preview)
            target_v = np.minimum(c.speed, np.sqrt(2*.30*np.maximum(self.path.length-progress, 0)))
            target_v = np.minimum(target_v, c.wmax/np.maximum(abs(kp), 1e-5))
            target_w = target_v*kp-c.lateral_gain*e-c.heading_gain*eh
            if not ideal:
                target_w += c.tau_w*target_v**2*dkp
            uv = np.clip(target_v+residuals[:,t,0], 0, c.vmax)
            uw = np.clip(target_w+residuals[:,t,1], -c.wmax, c.wmax)
            if guide_only:  # explicit zero-command stopping trajectory
                uv[:] = 0
                uw[:] = 0
            commands[:,t] = np.c_[uv,uw]
            if ideal:
                v, w = uv, uw
            else:
                sv += np.clip(uv-sv, -c.accel_v*c.dt, c.accel_v*c.dt)
                sw += np.clip(uw-sw, -c.accel_w*c.dt, c.accel_w*c.dt)
                delayed.append((sv.copy(), sw.copy()))
                if t < delay_steps:
                    dv, dw = pending[t]
                else:
                    dv, dw = delayed[t-delay_steps]
                v += (1-np.exp(-c.dt/c.tau_v))*(dv-v)
                w += (1-np.exp(-c.dt/c.tau_w))*(dw-w)
            mid = yaw+.5*w*c.dt
            x += v*np.cos(mid)*c.dt
            y += v*np.sin(mid)*c.dt
            yaw += w*c.dt
            progress = np.clip(progress+v*np.cos(eh)/np.maximum(.3, 1-k*e)*c.dt, 0, self.path.length)
            # Soft error and hard footprint constraints are evaluated together.
            dist = clearance(x,y,yaw,obstacles,c)
            min_clearance = np.minimum(min_clearance,dist)
            feasible &= dist >= c.clearance
            curve_weight = 1+2*np.minimum(abs(k),2)
            total += c.dt*(140*curve_weight*e**2+10*eh**2+12*(v-target_v)**2
                            +.15*residuals[:,t,0]**2+.05*residuals[:,t,1]**2)
            if len(obstacles):
                total += c.dt*8*np.maximum(.28-dist,0)**2
        total += 35*e**2+3*eh**2-(8 if len(obstacles) else 2)*(progress-self.s)
        return total, feasible, commands, min_clearance

    def command(self, state, smooth=(0.,0.), pending=(), obstacles=()):
        for name,value,shape in [('state',state,(5,)),('smooth',smooth,(2,))]:
            a=np.asarray(value,dtype=float)
            if a.shape != shape or not np.isfinite(a).all():
                raise ValueError('Invalid '+name)
        for name,value,cols in [('pending',pending,2),('obstacles',obstacles,3)]:
            a=np.asarray(value,dtype=float)
            if a.size and (a.ndim != 2 or a.shape[1] != cols or not np.isfinite(a).all()):
                raise ValueError('Invalid '+name)
        if len(obstacles) and (np.asarray(obstacles)[:,2]<0).any():
            raise ValueError('Negative obstacle radius')
        self.s, _, _ = self.path.project(state[0],state[1],self.s)
        c = self.cfg
        if self.mode == 'tracker':
            proposals = np.zeros((1,c.steps,2))
        else:
            noise = self.rng.normal(size=(c.samples,c.steps,2))*[c.noise_v,c.noise_w]
            # Temporally correlated residuals help find coherent detours.
            for j in range(1,c.steps):
                noise[:,j] = .80*noise[:,j-1]+.60*noise[:,j]
            proposals = self.residual[None]+noise
            proposals[0] = 0  # deterministic tracker candidate
            proposals[1] = self.residual
            # Both turn directions and a braking proposal keep diversity.
            if len(obstacles):
                # Coherent alternatives avoid relying on random noise to
                # sustain a left/right detour over the complete horizon.
                proposals[2:4] = 0
                proposals[2,:,1] = .40
                proposals[3,:,1] = -.40
            else:
                proposals[2,:,1] += .22
                proposals[3,:,1] -= .22
            proposals[4,:,0] = -c.speed
        costs, safe, commands, distances = self.rollout(state,smooth,pending,proposals,obstacles)
        # Test braking explicitly using the same dynamics and footprint.
        brake = np.zeros((1,c.steps,2)); brake[:,:,0] = -c.speed
        bc, bs, bu, bd = self.rollout(state,smooth,pending,brake,obstacles,guide_only=True)
        if not safe.any():
            self.residual[:] = 0
            self.last_diagnostics = {'feasible':bool(bs[0]),'status':'brake' if bs[0] else 'no_feasible_rollout',
                                     'min_predicted_clearance':float(bd[0])}
            return np.array([0.,0.])
        best = int(np.argmin(np.where(safe,costs,np.inf)))
        weights = np.zeros(len(costs))
        weights[safe] = np.exp(-(costs[safe]-costs[best])/c.temperature)
        weights /= weights.sum()
        mean = np.einsum('k,kni->ni',weights,proposals)
        mc, ms, mu, md = self.rollout(state,smooth,pending,mean[None],obstacles)
        use_mean = bool(ms[0] and mc[0] <= costs[best]+.05)
        chosen = mean if use_mean else proposals[best]
        command = mu[0,0] if use_mean else commands[best,0]
        self.residual[:-1] = chosen[1:]
        self.residual[-1] = 0
        self.last_diagnostics = {'feasible':True,'status':'mean' if use_mean else 'sample',
                                 'feasible_samples':int(safe.sum()),
                                 'effective_samples':float(1/np.sum(weights**2)),
                                 'min_predicted_clearance':float(md[0] if use_mean else distances[best])}
        return command.copy()
