"""Shared SE(3), immutable run inputs and native-reference point audit."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import shlex
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from controller_tracking.common import digest, stats
from controller_tracking.state import rotation
from controller_tracking.world_capture import mean_rotation, static_metrics
from controller_tracking.world_calibration import sha, save, load


def matrix(xyz, quaternion):
    h = np.eye(4); h[:3, :3] = rotation(quaternion); h[:3, 3] = np.asarray(xyz, float)
    if not np.isfinite(h).all(): raise ValueError('nonfinite_transform')
    return h


def inverse(h):
    h = np.asarray(h, float); out = np.eye(4)
    out[:3, :3] = h[:3, :3].T; out[:3, 3] = -out[:3, :3] @ h[:3, 3]
    return out


def record(h, parent, child):
    h = np.asarray(h)
    return dict(parent=parent, child=child, equation=f'p_{parent} = T_{parent}_{child} * p_{child}',
                matrix=h.tolist(), translation_m=h[:3, 3].tolist(),
                quaternion_xyzw=Rotation.from_matrix(h[:3, :3]).as_quat().tolist(), scale=1.)


def native_base(native_pose, config, extrinsics):
    # Native FAST-LIO state is F->IMU. Its point clouds already include T_IMU_LiDAR.
    return np.asarray(native_pose) @ inverse(np.asarray(extrinsics['T_B_I']['matrix']))


def mocap_base(tracker_pose, config, extrinsics):
    definition = config['M_from_mocap_world']
    return matrix(definition['translation_m'], definition['quaternion_xyzw']) @ np.asarray(tracker_pose) @ np.asarray(extrinsics['T_Tracker_B']['matrix'])


def static_window(pairs, cfg):
    limits = cfg['initialization']
    if len(pairs) < limits['minimum_pairs']: raise ValueError('insufficient_pairs')
    times = np.array([p['stamp_ns'] for p in pairs], dtype=np.int64)
    if (times[-1]-times[0])*1e-9 < limits['duration_s']: raise ValueError('need_three_seconds_of_measurement_time')
    if np.any(np.diff(times)<=0) or np.max(np.diff(times))*1e-9 > limits['maximum_pair_gap_s']:
        raise ValueError('nonmonotonic_or_long_gap')
    spread = {}
    for key in ('T_M_B', 'T_F_B'):
        poses = [np.asarray(p[key]) for p in pairs]
        spread[key] = static_metrics([p[:3, 3] for p in poses], [p[:3, :3] for p in poses])
        if spread[key]['maximum_radius_m'] > limits['maximum_static_radius_m'] or spread[key]['maximum_orientation_deviation_deg'] > limits['maximum_static_rotation_deg']:
            raise ValueError('robot_or_source_not_static:'+key)
    return spread


def estimate(pairs, cfg):
    spread=static_window(pairs,cfg)
    times=np.array([p['stamp_ns'] for p in pairs],dtype=np.int64)
    transforms = [np.asarray(p['T_M_B']) @ inverse(np.asarray(p['T_F_B'])) for p in pairs]
    h = np.eye(4); h[:3, :3] = mean_rotation([t[:3, :3] for t in transforms])
    h[:3, 3] = np.mean([t[:3, 3] for t in transforms], axis=0)
    residuals = [h @ np.asarray(p['T_F_B'])-np.asarray(p['T_M_B']) for p in pairs]
    return dict(T_mocap_native=record(h, 'mocap_world', 'native'), T_native_mocap=record(inverse(h), 'native', 'mocap_world'),
        method='mean_of_synchronous_static_T_M_B_times_inverse_T_F_B',
        pairs=len(pairs), duration_s=float((times[-1]-times[0])*1e-9), static_spread=spread,
        position_residual_m=stats([np.linalg.norm(p[:3, 3]) for p in residuals]),
        ndt_used=False, scale_fitted=False, frozen=True, transform_refit=False)


def initial_map_odom(calibration, mocap_native):
    """A new native world is odom; the original map calibration never changes."""
    return np.asarray(calibration['T_map_mocap']['matrix']) @ np.asarray(mocap_native)


def validation_errors(pairs, calibration, map_odom):
    """Held-out points are evaluated with fixed transforms; no fit or re-alignment."""
    mm=np.asarray(calibration['T_map_mocap']['matrix']); errors=[]
    for pair in pairs:
        truth=mm @ np.asarray(pair['T_M_B']); estimated=np.asarray(map_odom) @ np.asarray(pair['T_F_B'])
        delta=estimated[:3,3]-truth[:3,3]
        angle=Rotation.from_matrix(truth[:3,:3].T @ estimated[:3,:3]).magnitude()
        errors.append(dict(stamp_ns=pair['stamp_ns'],XYZ_error_m=delta.tolist(),distance_3D_m=float(np.linalg.norm(delta)),
            distance_XY_m=float(np.linalg.norm(delta[:2])),orientation_error_deg=math.degrees(angle)))
    return dict(fit_performed=False,transform_updated=False,reference_point='base_link',samples=errors,
        distance_3D_m=stats([e['distance_3D_m'] for e in errors]),distance_XY_m=stats([e['distance_XY_m'] for e in errors]),
        orientation_error_deg=stats([e['orientation_error_deg'] for e in errors]))


def import_projector(cfg):
    path = Path(cfg['project_root'])/'make_2d_map_from_pcd.py'
    spec = importlib.util.spec_from_file_location('existing_pcd_projection', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def prepare(run, site):
    run = Path(run).resolve(); cfg = load(site); root = Path(cfg['project_root'])
    if run.exists(): raise ValueError('Use a new run directory; never overwrite a map/session')
    if cfg.get('schema')!=2 or cfg.get('map_basis')!='native_fastlio_mapping_world':raise ValueError('Use the native-map schema 2 configuration')
    if cfg['map_frame'] != 'map': raise ValueError('The native mapping world is called map')
    if any(cfg[k]!=v for k,v in [('native_world_frame','camera_init'),('native_body_frame','body'),('base_frame','base_link')]):
        raise ValueError('Frame names must match the audited native FAST-LIO source and this launch profile')
    if cfg['M_from_mocap_world']['axes_confirmed'] is not True:
        raise ValueError('Confirm right-handed Z-up metre axes before preparing an anchor')
    definition=cfg['M_from_mocap_world']
    if not np.allclose(matrix(definition['translation_m'],definition['quaternion_xyzw']),np.eye(4),atol=1e-12):
        raise ValueError('This profile uses the confirmed mocap_world directly; no hidden axis/origin change')
    joint = ET.parse(cfg['urdf']).getroot().find(f"joint[@name='{cfg['native_body_joint']}']")
    if joint is None or joint.find('parent').get('link') != 'base_link' or joint.find('child').get('link') != 'imu_link':
        raise ValueError('Expected confirmed base_link->imu_link mounting, not a guessed coincident reference')
    origin = joint.find('origin'); xyz = list(map(float, origin.get('xyz').split()))
    quat = Rotation.from_euler('xyz', list(map(float, origin.get('rpy').split()))).as_quat()
    mount = load(cfg['mocap_config']); tracker = mount['rigid_body_to_base_link']
    if tracker.get('measured') is not True or cfg['body_mount_confirmed'] is not True:
        raise ValueError('Missing measured body extrinsics')
    ext = dict(T_B_I=record(matrix(xyz, quat), 'B', 'I'),
               T_Tracker_B=record(matrix(tracker['translation_m'], tracker['quaternion_xyzw']), 'Tracker', 'B'),
               sources=dict(urdf=cfg['urdf'], urdf_sha256=sha(cfg['urdf']), mocap=cfg['mocap_config'], mocap_sha256=sha(cfg['mocap_config'])),
               native_body_is_imu=True, lidar_to_imu_applied_again=False)
    native = load(cfg['fastlio_config']); params = native['/**']['ros__parameters']
    original = copy.deepcopy(params)
    params['map_file_path'] = str(run/'raw/native_F.pcd')
    params['pcd_save'].update(pcd_save_en=True, interval=-1)
    params['publish'].update(map_en=True, scan_publish_en=True)
    # Existing algorithm, preprocessing, covariances, extrinsic estimation and speeds unchanged.
    algo_before = {k:v for k,v in original.items() if k not in ('map_file_path','pcd_save','publish')}
    algo_after = {k:v for k,v in params.items() if k not in ('map_file_path','pcd_save','publish')}
    assert digest(algo_before) == digest(algo_after)
    (run/'raw').mkdir(parents=True); (run/'grid').mkdir(); (run/'initialization').mkdir()
    cfg['run_id'] = run.name; cfg['data_kind'] = cfg.get('data_kind', 'real_mapping_pending_collection')
    save(run/'config.yaml', cfg, True); save(run/'extrinsics.yaml', ext, True)
    save(run/'mocap.yaml', mount, True); save(run/'fastlio.yaml', native, True)
    (run/'nav2.yaml').write_bytes((root/'robot_data/config/navigation.yaml').read_bytes())
    source_paths = [cfg[k] for k in ('fastlio_config','fastlio_source','fastlio_binary','mocap_config','urdf')]
    source_paths += [str(root/p) for p in ('make_2d_map_from_pcd.py','pcd_to_2d_map.sh','robot_data/config/navigation.yaml','tf_to_odom_bridge.py')]
    frozen = dict(inputs={name:sha(run/name) for name in ('config.yaml','extrinsics.yaml','mocap.yaml','fastlio.yaml','nav2.yaml')},
                  sources={str(Path(p).resolve()):sha(p) for p in source_paths}, algorithm_parameters_unchanged=True)
    save(run/'inputs.lock.json', frozen)
    products = dict(native_pcd=str(run/'raw/native_F.pcd'),
        map_yaml=str(run/'grid/map.yaml'), map_image=str(run/'grid/map.pgm'),
        transform=str(run/'calibration.yaml'), initialization=str(run/'initialization/raw.jsonl'))
    save(run/'products.json', {k:dict(path=v,exists=Path(v).exists()) for k,v in products.items()})
    save(run/'state.json', dict(stage='prepared_no_anchor_no_maps', run_id=run.name, products=products,
                               motion_commands_published=False, ndt_used_for_anchor=False))
    return products


def update_products(run):
    """Mutable index only; immutable measurement and transform files stay exclusive."""
    run = Path(run)
    products = load(run/'products.json')
    for entry in products.values():
        p = Path(entry['path']); entry['exists'] = p.is_file()
        if p.is_file(): entry.update(bytes=p.stat().st_size, sha256=sha(p))
    (run/'products.json').write_text(json.dumps(products, indent=2)+'\n')
    state=load(run/'state.json')
    stage='prepared_no_anchor_no_maps'
    if products['initialization']['exists']:stage='initialization_capture_started'
    if products['transform']['exists']:stage='anchor_frozen'
    if products['native_pcd']['exists']:stage='native_pcd_written_check_receipt'
    if products['map_yaml']['exists'] and products['map_image']['exists']:stage='grid_written_check_projection_receipt'
    state.update(stage=stage,acquisition_failed=(run/'initialization/failure.json').exists())
    (run/'state.json').write_text(json.dumps(state,indent=2)+'\n')
    return products


def checked(run, require_anchor=False):
    run = Path(run).resolve(); lock = load(run/'inputs.lock.json')
    for name, expected in lock['inputs'].items():
        if sha(run/name) != expected: raise ValueError('Frozen input changed:'+name)
    for name, expected in lock['sources'].items():
        if sha(name) != expected: raise ValueError('Original source/config changed:'+name)
    cfg = load(run/'config.yaml'); ext = load(run/'extrinsics.yaml')
    if cfg.get('schema')!=2 or cfg.get('map_basis')!='native_fastlio_mapping_world':raise ValueError('Retired map=M run; prepare a new native-map run')
    if require_anchor:
        anchor = load(run/'calibration.yaml')
        if sha(run/'calibration.yaml') != load(run/'calibration.lock.json')['sha256'] or anchor['inputs_sha256'] != sha(run/'inputs.lock.json'):
            raise ValueError('Anchor changed or belongs to another native mapping run')
        for field,name in [('raw_sha256','initialization/raw.jsonl'),('pairs_sha256','initialization/pairs.json')]:
            if field in anchor and sha(run/name)!=anchor[field]:raise ValueError('Initialization evidence changed:'+name)
    return cfg, ext


def checked_session(run, session):
    run=Path(run).resolve();session=Path(session).resolve();checked(run,True)
    if session.parent!=run/'localizations':raise ValueError('Restart session is outside this map run')
    value=load(session/'initialization.yaml')
    if value['calibration_sha256']!=sha(run/'calibration.yaml') or sha(session/'initialization.yaml')!=load(session/'initialization.lock.json')['sha256']:
        raise ValueError('Restart initialization changed or belongs to another map calibration')
    for field,name in [('raw_sha256','raw.jsonl'),('pairs_sha256','pairs.json')]:
        if sha(session/name)!=value[field]:raise ValueError('Restart evidence changed:'+name)
    return value


def projection_args(cfg, input_pcd, prefix):
    """Reuse exact existing slicing/projection/denoising switches, without shell eval."""
    text = (Path(cfg['project_root'])/'pcd_to_2d_map.sh').read_text()
    command = text.split('./make_2d_map_from_pcd.py',1)[1].split('\n\necho',1)[0].replace('\\\n',' ')
    tokens = shlex.split(command); replacements = {'--pcd':str(input_pcd), '--output-prefix':str(prefix)}
    args = []; i = 0
    while i < len(tokens):
        token = tokens[i]
        if token in replacements: args += [token,replacements[token]]; i += 2
        else: args.append(token); i += 1
    args += ['--debug-dir',str(Path(prefix).parent/'debug'),'--enable-debug-output','false','--no-backup']
    return args
