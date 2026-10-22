"""ROS independent experiment state machine; no localization feedback here."""
from collections import deque
from dataclasses import asdict, dataclass
import math

import numpy as np
import yaml

from ndt_match_evaluation_core import rigid_body_to_base_pose, yaw, wrap


def load_mount(path):
    with open(path, encoding='utf8') as stream:
        document = yaml.safe_load(stream)
    mount = document['rigid_body_to_base_link']
    if mount.get('measured') is not True:
        raise ValueError('Explicit measured Tracker0->base_link mount required')
    t, q = mount['translation_m'], mount['quaternion_xyzw']
    if len(t) != 3 or len(q) != 4 or not all(math.isfinite(float(v)) for v in [*t, *q]):
        raise ValueError('Invalid mount values')
    if math.hypot(*q) < 1e-9:
        raise ValueError('Invalid mount quaternion')
    return {'translation_m': t, 'quaternion_xyzw': q}


def base_pose(pose, mount):
    return rigid_body_to_base_pose(pose, mount['translation_m'], mount['quaternion_xyzw'])


def pose_dict(stamp, pose):
    return dict(stamp_ns=int(stamp.sec)*10**9+int(stamp.nanosec), valid=True,
                x=pose.position.x, y=pose.position.y, z=pose.position.z,
                qx=pose.orientation.x, qy=pose.orientation.y,
                qz=pose.orientation.z, qw=pose.orientation.w)


@dataclass(frozen=True)
class MotionConfig:
    speed_mps: float = .15
    direction: int = 1
    distance_m: float = 2.0
    settle_s: float = 5.0
    post_stop_s: float = 5.0
    stale_s: float = .20
    acquire_timeout_s: float = 15.0
    max_motion_s: float = 25.0
    max_jump_m: float = .06
    max_cross_track_m: float = .25
    max_yaw_deg: float = 15.0
    no_progress_s: float = 3.0
    min_progress_m: float = .03
    max_static_radius_m: float = .005

    def validate(self):
        if any(not math.isfinite(v) or v <= 0 for k,v in asdict(self).items() if k!='direction'):
            raise ValueError('All motion settings must be finite and positive')
        if type(self.direction) is not int or self.direction not in (-1,1):
            raise ValueError('Direction must be integer +1 or -1')
        if self.speed_mps != .15 or self.distance_m != 2.0 or self.post_stop_s != 5.0:
            raise ValueError('Protocol requires 0.15 m/s, 2.000 m and 5 s after stop')


class MotionGate:
    """Distance is displacement of the compensated base origin, not arc length.

    A/B/C use this exact class. Only this controller's distance stop reads
    mocap in A/B; it never uses a localization estimate to steer or stop.
    Times used for liveness/limits are monotonic, not wall clock timestamps.
    """
    def __init__(self, started, config=None):
        self.config = config or MotionConfig()
        self.config.validate()
        self.created = started
        self.state = 'ACQUIRE'
        self.reason = ''
        self.latest = self.start = None
        self.last_receive = self.settle_started = self.move_started = self.stop_started = None
        self.distance = self.path_length = 0.0
        self.start_samples = deque()
        self.events = []
        self.last_stamp = 0
        self.duplicates = 0

    def stop(self, now, reason):
        if self.state in ('DONE', 'ABORTED', 'POST_STOP'):
            return
        self.reason = reason
        self.state = 'POST_STOP'
        self.stop_started = now
        self.events.append(dict(event='stop_command', steady_s=now, reason=reason,
                                displacement_m=self.distance, path_length_m=self.path_length,
                                mocap_stamp_ns=self.last_stamp))

    def ingest(self, pose, now):
        if self.state in ('DONE', 'ABORTED', 'POST_STOP'):
            return
        try:
            values = [float(pose[k]) for k in ('x','y','z','qx','qy','qz','qw')]
            valid = pose.get('valid', True) and all(math.isfinite(v) for v in values)
            valid = valid and math.hypot(*values[3:]) > 1e-9 and int(pose['stamp_ns']) > 0
        except (ValueError, KeyError, TypeError):
            valid = False
        if not valid:
            self.stop(now, 'invalid_mocap')
            return
        if pose['stamp_ns'] <= self.last_stamp:
            self.duplicates += 1
            # A duplicate is not a new measurement: do not refresh liveness.
            return
        if self.latest:
            jump = math.hypot(pose['x']-self.latest['x'],pose['y']-self.latest['y'])
            if jump > self.config.max_jump_m:
                self.stop(now, 'mocap_pose_jump')
                return
            if self.state == 'MOVING':
                self.path_length += jump
        self.latest = dict(pose)
        self.last_stamp = int(pose['stamp_ns'])
        self.last_receive = now
        if self.state == 'ACQUIRE':
            self.state = 'SETTLE'
            self.settle_started = now
        if self.state == 'SETTLE':
            self.start_samples.append((now, dict(pose)))
            while self.start_samples and now-self.start_samples[0][0] > self.config.settle_s:
                self.start_samples.popleft()
        if self.start:
            dx,dy = pose['x']-self.start['x'],pose['y']-self.start['y']
            self.distance = math.hypot(dx,dy)
            angle = yaw(self.start)
            along = math.cos(angle)*dx+math.sin(angle)*dy
            if self.state=='MOVING' and self.config.direction*along < -.03:
                self.stop(now,'wrong_motion_direction')
                return
            # Reach target first: zero output on this same mocap callback.
            if self.state == 'MOVING' and self.distance >= self.config.distance_m:
                self.stop(now, 'target_reached')
                return
            cross = -math.sin(angle)*dx+math.cos(angle)*dy
            if abs(cross) > self.config.max_cross_track_m:
                self.stop(now, 'cross_track_limit')
            elif abs(wrap(yaw(pose)-angle)) > math.radians(self.config.max_yaw_deg):
                self.stop(now, 'heading_limit')

    def tick(self, now):
        if self.state in ('DONE','ABORTED'):
            return 0.
        if self.state == 'POST_STOP':
            if now-self.stop_started >= self.config.post_stop_s:
                self.state = 'DONE' if self.reason == 'target_reached' else 'ABORTED'
            return 0.
        if self.last_receive is None:
            if now-self.created > self.config.acquire_timeout_s:
                self.stop(now,'mocap_acquire_timeout')
            return 0.
        if now-self.last_receive > self.config.stale_s:
            self.stop(now,'mocap_stale')
            return 0.
        if self.state == 'SETTLE' and now-self.settle_started >= self.config.settle_s:
            samples = [p for _,p in self.start_samples]
            if len(samples) < 30:
                self.stop(now,'insufficient_start_samples')
                return 0.
            xy=np.array([[p['x'],p['y']] for p in samples])
            center=xy.mean(axis=0)
            if np.max(np.linalg.norm(xy-center,axis=1)) > self.config.max_static_radius_m:
                self.stop(now,'start_not_static')
                return 0.
            self.start=dict(samples[-1],x=float(center[0]),y=float(center[1]))
            a=math.atan2(sum(math.sin(yaw(p)) for p in samples),sum(math.cos(yaw(p)) for p in samples))
            self.start.update(qx=0.,qy=0.,qz=math.sin(a/2),qw=math.cos(a/2))
            self.move_started=now
            self.state='MOVING'
            self.events.append(dict(event='motion_start',steady_s=now,start_base_pose=self.start,
                                    direction=self.config.direction))
        if self.state == 'MOVING':
            elapsed=now-self.move_started
            if elapsed > self.config.max_motion_s:
                self.stop(now,'motion_timeout')
            elif elapsed > self.config.no_progress_s and self.distance < self.config.min_progress_m:
                self.stop(now,'no_progress')
            return self.config.direction*self.config.speed_mps if self.state == 'MOVING' else 0.
        return 0.


def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],wrap(a[2]+b[2]))


def inverse(a):
    c,s=math.cos(a[2]),math.sin(a[2])
    return (-c*a[0]-s*a[1],s*a[0]-c*a[1],-a[2])


def planar(p):
    return (p['x'],p['y'],yaw(p))
