"""Immutable session configuration and a minimal, reviewable Nav2 wiring overlay."""
import copy
import hashlib
from pathlib import Path
import yaml
from .common import digest, write_json


def load(path):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    for key in ('production_nav', 'mocap_config'):
        cfg[key] = str((path.parent / cfg[key]).resolve())
    return cfg


def nav_overlay(production, frame):
    result = {k: copy.deepcopy(production[k]) for k in
              ('controller_server', 'velocity_smoother', 'local_costmap')}
    result['controller_server']['ros__parameters']['odom_topic'] = '/tracking/state'
    result['velocity_smoother']['ros__parameters']['odom_topic'] = '/tracking/state'
    result['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = frame
    # Measurement-only delegation. All FollowPath algorithm parameters stay identical.
    result['controller_server']['ros__parameters']['FollowPath']['plugin'] = 'controller_tracking::MeasuredController'
    return {'tracking': result}


def differences(a, b, prefix=''):
    out = []
    for k in sorted(set(a) | set(b)):
        key = prefix + k
        if isinstance(a.get(k), dict) and isinstance(b.get(k), dict):
            out.extend(differences(a[k], b[k], key + '.'))
        elif a.get(k) != b.get(k):
            out.append(dict(parameter=key, production=a.get(k), experiment=b.get(k)))
    return out


def snapshot(cfg, output):
    output = Path(output)
    nav = yaml.safe_load(Path(cfg['production_nav']).read_text())
    mount = yaml.safe_load(Path(cfg['mocap_config']).read_text())
    overlay = nav_overlay(nav, cfg['frame'])
    if nav['controller_server']['ros__parameters']['FollowPath']['vx_max'] != cfg['speed_requirement']['upper_bound_mps']:
        raise ValueError('Speed requirement differs from production MPPI; report/review before creating session')
    (output/'config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    (output/'nav2.yaml').write_text(yaml.safe_dump(overlay, sort_keys=False))
    (output/'mocap.yaml').write_text(yaml.safe_dump(mount, sort_keys=False))
    chosen = {k: nav[k] for k in overlay['tracking']}
    write_json(output/'parameter_diff.json', differences(chosen, overlay['tracking']))
    write_json(output/'identity.json', dict(config_sha256=digest(cfg),
               production_sha256=hashlib.sha256(Path(cfg['production_nav']).read_bytes()).hexdigest(),
               mocap_sha256=digest(mount), nav2_sha256=digest(overlay), algorithm='original MPPI via measurement delegate',
               camera_latency_verified=cfg['timing']['camera_latency_verified']))
    return mount


def check_session(path):
    path = Path(path)
    cfg = yaml.safe_load((path/'config.yaml').read_text())
    identity = __import__('json').loads((path/'identity.json').read_text())
    if identity['config_sha256'] != digest(cfg):
        raise ValueError('Frozen session config changed; create a new session')
    mount = yaml.safe_load((path/'mocap.yaml').read_text())
    if digest(mount) != identity['mocap_sha256']:
        raise ValueError('Frozen mount changed')
    frame = __import__('json').loads((path/'frame.json').read_text())
    if frame['config_sha256'] != digest(cfg):
        raise ValueError('Frame belongs to another configuration')
    if digest(yaml.safe_load((path/'nav2.yaml').read_text())) != identity['nav2_sha256']:
        raise ValueError('Frozen Nav2 overlay changed')
    if digest(frame) != identity['frame_sha256']:
        raise ValueError('Frozen frame transform changed')
    return cfg, mount, frame
