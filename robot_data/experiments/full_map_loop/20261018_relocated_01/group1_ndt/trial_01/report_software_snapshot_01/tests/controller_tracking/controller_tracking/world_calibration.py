"""Offline, scale-free map<-mocap_world registration. Never publishes ROS data.

Protocol: the SAME base_link origin at three training placements and one
held-out placement. The fourth placement is not read by solve().
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .common import digest, stats
from .state import rotation

TRAIN = ('calib_01', 'calib_02', 'calib_03')
HOLDOUT = 'test_04'
GROUPS = (*TRAIN, HOLDOUT)


def load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value, yaml_format=False):
    """Exclusive creation: raw captures, frozen inputs and results are immutable."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = (yaml.safe_dump(value, allow_unicode=True, sort_keys=False) if yaml_format
            else json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    with path.open('x', encoding='utf-8') as stream:
        stream.write(text)


def xyz(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('Need three measured finite XYZ coordinates in metres')
    return result


def pose_xyz(pose, offset=None):
    p = xyz(pose['xyz_m'])
    r = rotation(pose['quaternion_xyzw'])
    return p if offset is None else p + r @ xyz(offset)


def geometry_check(points, minimum_baseline, minimum_altitude):
    """Three points determine SE(3) only when the triangle is nondegenerate."""
    p = np.asarray(points, float)
    if p.shape != (3, 3) or not np.isfinite(p).all():
        raise ValueError('Exactly three finite 3-D training points required')
    edges = np.array([np.linalg.norm(p[1]-p[0]), np.linalg.norm(p[2]-p[0]),
                      np.linalg.norm(p[2]-p[1])])
    altitude = float(np.linalg.norm(np.cross(p[1]-p[0], p[2]-p[0])) / max(edges.max(), 1e-15))
    if edges.min() < minimum_baseline or altitude < minimum_altitude:
        raise ValueError('Training placements are coincident/collinear or too close; use a triangle')
    return dict(edge_lengths_m=edges.tolist(), minimum_altitude_m=altitude,
                centered_singular_values_m=np.linalg.svd(p-p.mean(axis=0), compute_uv=False).tolist())


def fit_rigid(source, target, minimum_baseline=.10, minimum_altitude=.05):
    """Kabsch proper rotation: fixed scale=1, det(R)=+1; equal point weights."""
    source, target = np.asarray(source, float), np.asarray(target, float)
    checks = dict(mocap=geometry_check(source, minimum_baseline, minimum_altitude),
                  map=geometry_check(target, minimum_baseline, minimum_altitude))
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, singular, vt = np.linalg.svd((source-a).T @ (target-b))
    correction = np.eye(3)
    correction[2, 2] = 1. if np.linalg.det(vt.T @ u.T) > 0 else -1.
    r = vt.T @ correction @ u.T
    t = b-r @ a
    checks['cross_covariance_singular_values'] = singular.tolist()
    checks['edge_length_difference_map_minus_mocap_m'] = (
        np.asarray(checks['map']['edge_lengths_m']) - checks['mocap']['edge_lengths_m']).tolist()
    return r, t, checks


def transform_record(r, t, parent, child):
    h = np.eye(4); h[:3, :3] = r; h[:3, 3] = t
    return dict(equation=f'p_{parent} = R * p_{child} + t', parent_frame=parent,
                child_frame=child, rotation_matrix=r.tolist(), translation_m=t.tolist(),
                homogeneous_matrix=h.tolist(), scale=1.0)


def error_report(source, target, r, t, labels):
    source, target = np.asarray(source, float), np.asarray(target, float)
    prediction = source @ r.T + t
    errors = prediction-target
    rows = [dict(group=label, mocap_xyz_m=a.tolist(), map_xyz_m=b.tolist(),
                 predicted_map_xyz_m=p.tolist(), error_xyz_m=e.tolist(),
                 error_3d_m=float(np.linalg.norm(e)), error_xy_m=float(np.linalg.norm(e[:2])))
            for label, a, b, p, e in zip(labels, source, target, prediction, errors)]
    return dict(error_sign='transformed_mocap_minus_map', points=rows,
                x_m=stats(errors[:, 0]), y_m=stats(errors[:, 1]), z_m=stats(errors[:, 2]),
                distance_3d_m=stats(np.linalg.norm(errors, axis=1)),
                distance_xy_m=stats(np.linalg.norm(errors[:, :2], axis=1)))


def init_session(session, mocap_config, map_source):
    session = Path(session)
    if session.exists():
        raise ValueError('Use a new empty session directory; existing experiments are retained')
    original = load(mocap_config)
    session.mkdir(parents=True)
    # Body mounting stays a separate file, never optimized with world alignment.
    save(session/'body_extrinsic.yaml', dict(
        direction='T_Tracker0_base_link; p_Tracker0 = R * p_base_link + t',
        source_path=str(Path(mocap_config).resolve()), source_sha256=sha(mocap_config),
        rigid_body_to_base_link=original['rigid_body_to_base_link']), True)
    cfg = dict(schema_version=1, protocol='three_training_placements_one_holdout',
        data_kind='real_measurement', unit='metre', reference_point='base_link_origin',
        mocap_frame='mocap_world', map_frame='map', calibration_id=None,
        map_reference_id=None, map_reference_files=[], physical_reference_confirmed=False,
        map_source=map_source, map_method='NDT/FAST-LIO estimate' if map_source == 'ndt' else None,
        mocap=original['mocap'], normalizer=original['normalizer'],
        topics=dict(ndt='/static_localization/ndt_match_diagnostic', command='/reference_velocity',
                    map_ready='/static_localization/ready'),
        ndt_pose_field='final_map_to_base_at_cloud', ndt_base_frame='base_link',
        camera_latency_s=None, clock_offset_s=0., clock_offset_verified=False,
        limits=dict(seconds=4., warmup_s=2., minimum_mocap_samples=30, minimum_pairs=3,
                    minimum_raw_normalized_coverage=.90,
                    maximum_mocap_gap_s=.15, maximum_ndt_gap_s=2., maximum_pair_skew_s=.05,
                    maximum_source_age_s=.50, maximum_cloud_odom_skew_s=.05,
                    maximum_mocap_radius_m=.01, maximum_map_radius_m=.05,
                    maximum_orientation_deviation_deg=2., minimum_baseline_m=.10,
                    minimum_triangle_altitude_m=.05, holdout_minimum_separation_m=.10,
                    holdout_minimum_heading_difference_deg=10., maximum_command_abs=1e-4),
        acceptance=dict(validation_3d_max_m=None, validation_xy_max_m=None))
    save(session/'session.yaml', cfg, True)
    for group in GROUPS:
        save(session/'map_inputs'/f'{group}.yaml', dict(group=group, frame='map',
            reference_point='base_link_origin', unit='metre', xyz_m=[None, None, None],
            method=None, independently_measured=None), True)
    return dict(session=str(session.resolve()), next='Fill session.yaml; then freeze-inputs')


def validate_inputs(cfg, mount):
    if cfg.get('protocol') != 'three_training_placements_one_holdout':
        raise ValueError('Expected 4 placements: 3 training points and 1 held-out point')
    if (cfg.get('unit'), cfg.get('reference_point'), cfg.get('mocap_frame'), cfg.get('map_frame')) != (
            'metre', 'base_link_origin', 'mocap_world', 'map'):
        raise ValueError('Reference point, units or world frames do not match')
    if not cfg.get('calibration_id') or not cfg.get('map_reference_id') or cfg.get('physical_reference_confirmed') is not True:
        raise ValueError('Fill camera calibration_id, map_reference_id and confirm the SAME base_link origin')
    if cfg.get('map_source') not in ('manual', 'ndt') or not cfg.get('map_method'):
        raise ValueError('Declare map source and measurement/estimation method')
    if cfg['ndt_pose_field'] not in ('final_map_to_base_at_cloud', 'raw_ndt_map_to_base_at_cloud') or cfg['ndt_base_frame'] != 'base_link':
        raise ValueError('Only explicitly configured base_link poses at cloud time are supported')
    if cfg['mocap']['world_frame'] != 'mocap_world' or cfg['normalizer']['output_frame'] != 'mocap_world':
        raise ValueError('Mocap stream must be in mocap_world')
    scale = cfg['mocap']['position_scale_to_m']
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Raw mocap position scale must be finite and positive')
    if mount.get('measured') is not True:
        raise ValueError('Tracker0->base_link extrinsic must be measured and confirmed')
    xyz(mount['translation_m']); rotation(mount['quaternion_xyzw'])
    for name, value in cfg['limits'].items():
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid positive quality limit: '+name)
    if not 3 <= cfg['limits']['seconds'] <= 5:
        raise ValueError('Capture window must be between 3 and 5 seconds')
    if cfg['limits']['minimum_raw_normalized_coverage'] > 1:
        raise ValueError('Raw/normalized coverage must be in (0,1]')
    if not math.isfinite(cfg['clock_offset_s']):
        raise ValueError('Invalid explicit clock offset')
    for value in cfg['acceptance'].values():
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise ValueError('Acceptance thresholds must be positive or null (unspecified)')


def freeze_inputs(session):
    s = Path(session); cfg = load(s/'session.yaml'); body = load(s/'body_extrinsic.yaml')
    validate_inputs(cfg, body['rigid_body_to_base_link'])
    files = {str(Path(p).resolve()): sha(p) for p in cfg.get('map_reference_files', [])}
    result = dict(config=cfg, body_extrinsic=body, session_sha256=sha(s/'session.yaml'),
                  body_sha256=sha(s/'body_extrinsic.yaml'), map_file_hashes=files)
    save(s/'frozen_inputs.json', result)
    return dict(frozen=True, sha256=sha(s/'frozen_inputs.json'))


def frozen_inputs(session):
    s = Path(session); doc = load(s/'frozen_inputs.json')
    if sha(s/'session.yaml') != doc['session_sha256'] or sha(s/'body_extrinsic.yaml') != doc['body_sha256']:
        raise ValueError('Frozen session or body extrinsic changed; create a new session')
    for path, expected in doc.get('map_file_hashes', {}).items():
        if sha(path) != expected: raise ValueError('Map reference file changed during calibration')
    validate_inputs(doc['config'], doc['body_extrinsic']['rigid_body_to_base_link'])
    return doc


def enter_map(session, group, input_path):
    s = Path(session); cfg = frozen_inputs(s)['config']
    if cfg['map_source'] != 'manual':
        raise ValueError('NDT source is collected per placement; no manual substitution')
    if (s/'calibration.yaml').exists() and group in TRAIN:
        raise ValueError('Training map coordinates are already frozen')
    if (s/'validation.json').exists():
        raise ValueError('Validation is closed; keep the result unchanged')
    doc = load(input_path)
    if doc.get('group') != group or doc.get('frame') != 'map' or doc.get('unit') != 'metre' or doc.get('reference_point') != 'base_link_origin':
        raise ValueError('Map coordinate label/frame/reference point/unit mismatch')
    xyz(doc['xyz_m'])
    if not doc.get('method') or not isinstance(doc.get('independently_measured'), bool):
        raise ValueError('Declare map measurement method and whether it is independent')
    save(s/group/'map_coordinate.json', doc)
    return doc


def get_group(session, group):
    s = Path(session); cfg = frozen_inputs(s)['config']; summary = load(s/group/'summary.json')
    if summary['group'] != group or summary['inputs_sha256'] != sha(s/'frozen_inputs.json'):
        raise ValueError('Capture label or frozen configuration mismatch')
    if summary['raw_sha256'] != sha(s/group/'raw.jsonl') or not summary['quality_passed']:
        raise ValueError(f'{group}: raw data changed or capture quality failed; retain it and use a new session')
    result = dict(summary)
    if cfg['map_source'] == 'manual':
        manual = load(s/group/'map_coordinate.json')
        result['map_xyz_m'] = xyz(manual['xyz_m']).tolist()
        result['map_provenance'] = manual
    return result


def group_digest(session, group):
    s = Path(session)/group
    files = ('summary.json', 'raw.jsonl', 'map_coordinate.json')
    return {name: sha(s/name) for name in files if (s/name).exists()}


def write_report(path, title, result):
    kind = result['data_kind']
    lines = [f'# {title}', '',
             '**合成软件测试，非实车结果。**' if kind != 'real_measurement' else '数据类型：实车静止测量。', '',
             '误差符号：固定变换后的动捕坐标 − 地图坐标；单位：米。', '',
             '|位置|ΔX|ΔY|ΔZ|三维距离|XY距离|', '|---|---:|---:|---:|---:|---:|']
    for p in result['errors']['points']:
        lines.append('|'+p['group']+'|'+ '|'.join(f'{v:.8f}' for v in [*p['error_xyz_m'], p['error_3d_m'], p['error_xy_m']])+'|')
    lines += ['', '|指标|RMSE m|最大绝对值 m|', '|---|---:|---:|']
    for key in ('x_m', 'y_m', 'z_m', 'distance_3d_m', 'distance_xy_m'):
        a = result['errors'][key]; lines.append(f"|{key}|{a['rmse']:.8f}|{a['max_abs']:.8f}|")
    if 'protocol_passed' in result:
        lines += ['', '独立布点协议：'+('通过' if result['protocol_passed'] else '未通过，参见 validation.json 的 diversity'),
                  '预先指定的数值验收：'+('未指定门槛，只报告误差' if result['numeric_acceptance_passed'] is None else
                                        ('通过' if result['numeric_acceptance_passed'] else '未通过')),
                  '标定文件 SHA256：'+result['calibration_sha256']]
    lines += ['', result['interpretation'], '',
              '未估计尺度；未逐点重新对齐；未广播 TF；验证点从未参与拟合。', '',
              '仅三个标定位置无法可靠自动识别并剔除异常点；全部保留，报告三角形边长差。',
              '单个验证位置只能检查该位置的一致性，不能证明全场或动态精度。',
              '动捕时间是接收时刻，曝光延迟未独立标定；静止采集不能认证动态同步。']
    if 'origins' in result:
        lines += ['', '动捕原点在地图中（m）：'+str(result['origins']['mocap_origin_in_map_m']),
                  '地图原点在动捕中（m）：'+str(result['origins']['map_origin_in_mocap_m'])]
    with Path(path).open('x', encoding='utf-8') as stream:
        stream.write('\n'.join(lines)+'\n')


def interpretation(cfg):
    if cfg['map_source'] == 'ndt':
        return ('地图参照来自 NDT/FAST-LIO：残差保留定位链、动捕、安装外参及时间配对的综合影响；'
                '不能将其全部归因于世界外参，也不能作为独立地图真值精度。全局固定偏差可能被标定吸收，'
                '独立验证保留其未被固定变换解释的变化；后续方法比较必须沿用本变换，不分别重标定。')
    return '地图参照为逐位置录入；是否独立实测见各位置来源记录，未独立测量时仍不是绝对精度认证。'


def solve(session):
    s = Path(session); cfg = frozen_inputs(s)['config']
    if (s/'calibration.yaml').exists() or (s/HOLDOUT).exists():
        raise ValueError('Solve exactly once BEFORE collecting test_04; no refitting after holdout exposure')
    # Deliberately read only the three predeclared training placements.
    rows = [get_group(s, g) for g in TRAIN]
    source = [xyz(row['mocap_xyz_m']) for row in rows]
    target = [xyz(row['map_xyz_m']) for row in rows]
    limits = cfg['limits']
    r, t, checks = fit_rigid(source, target, limits['minimum_baseline_m'], limits['minimum_triangle_altitude_m'])
    inv_r, inv_t = r.T, -r.T @ t
    result = dict(schema_version=1, protocol=cfg['protocol'], data_kind=cfg['data_kind'],
        source_frame='mocap_world', target_frame='map', reference_point='base_link_origin',
        calibration_id=cfg['calibration_id'], map_reference_id=cfg['map_reference_id'],
        inputs_sha256=sha(s/'frozen_inputs.json'), training_groups=list(TRAIN), training_point_count=3,
        holdout_group=HOLDOUT, holdout_used=False, transform_refit=False,
        map_from_mocap=transform_record(r, t, 'map', 'mocap_world'),
        mocap_from_map=transform_record(inv_r, inv_t, 'mocap_world', 'map'),
        origins=dict(mocap_origin_in_map_m=t.tolist(), map_origin_in_mocap_m=inv_t.tolist()),
        checks=checks, errors=error_report(source, target, r, t, TRAIN),
        training_hashes={g: group_digest(s, g) for g in TRAIN},
        body_extrinsic_file='body_extrinsic.yaml', body_extrinsic_sha256=sha(s/'body_extrinsic.yaml'),
        map_provenance={g: row['map_provenance'] for g, row in zip(TRAIN, rows)},
        interpretation=interpretation(cfg), tf_published=False, absolute_accuracy_certified=False)
    save(s/'calibration.yaml', result, True)
    save(s/'calibration.lock.json', dict(sha256=sha(s/'calibration.yaml')))
    write_report(s/'calibration_report.md', '三位置三维无尺度标定结果', result)
    return result


def check_calibration(session):
    s = Path(session); frozen_inputs(s)
    lock = load(s/'calibration.lock.json')
    if sha(s/'calibration.yaml') != lock['sha256']:
        raise ValueError('Frozen calibration changed')
    result = load(s/'calibration.yaml')
    if result['inputs_sha256'] != sha(s/'frozen_inputs.json'):
        raise ValueError('Frozen calibration input mismatch')
    for g in TRAIN:
        if result['training_hashes'][g] != group_digest(s, g):
            raise ValueError('Training observations changed after solve')
    return result


def verify(session):
    s = Path(session); cfg = frozen_inputs(s)['config']; calibration = check_calibration(s)
    test = get_group(s, HOLDOUT)
    if test.get('calibration_sha256_at_capture') != sha(s/'calibration.yaml'):
        raise ValueError('Held-out observation must have been captured AFTER this transform was frozen')
    training = [get_group(s, g) for g in TRAIN]
    distances = [float(np.linalg.norm(xyz(test['mocap_xyz_m'])-xyz(row['mocap_xyz_m']))) for row in training]
    # Orientation is used to validate experimental diversity, never to fit R.
    test_r = np.asarray(test['mocap_base_rotation_matrix'])
    angles = [math.degrees(math.acos(float(np.clip((np.trace(test_r @ np.asarray(row['mocap_base_rotation_matrix']).T)-1)/2, -1, 1)))) for row in training]
    diversity = dict(distances_to_training_m=distances, orientation_difference_deg=angles,
        passed=bool(min(distances) >= cfg['limits']['holdout_minimum_separation_m'] and
                    min(angles) >= cfg['limits']['holdout_minimum_heading_difference_deg']))
    forward = calibration['map_from_mocap']; r = np.asarray(forward['rotation_matrix']); t = np.asarray(forward['translation_m'])
    errors = error_report([test['mocap_xyz_m']], [test['map_xyz_m']], r, t, [HOLDOUT])
    acceptance = cfg['acceptance']
    specified = all(v is not None for v in acceptance.values())
    passed = (errors['distance_3d_m']['max_abs'] <= acceptance['validation_3d_max_m'] and
              errors['distance_xy_m']['max_abs'] <= acceptance['validation_xy_max_m']) if specified else None
    result = dict(data_kind=cfg['data_kind'], errors=errors, transform_refit=False,
        calibration_sha256=sha(s/'calibration.yaml'), holdout_hashes=group_digest(s, HOLDOUT),
        diversity=diversity, acceptance=acceptance, numeric_acceptance_passed=passed,
        protocol_passed=diversity['passed'], absolute_accuracy_certified=False,
        map_provenance=test['map_provenance'], origins=calibration['origins'],
        interpretation=interpretation(cfg),
        status=('completed' if diversity['passed'] else 'retained_protocol_failed_placement_not_independent'))
    save(s/'validation.json', result)
    write_report(s/'validation_report.md', '第四位置独立验证结果', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    init = sub.add_parser('init'); init.add_argument('--session', required=True)
    init.add_argument('--mocap-config', required=True); init.add_argument('--map-source', choices=['ndt', 'manual'], default='ndt')
    for name in ('freeze-inputs', 'check-inputs', 'solve', 'verify'):
        p = sub.add_parser(name); p.add_argument('--session', required=True)
    enter = sub.add_parser('set-map'); enter.add_argument('--session', required=True)
    enter.add_argument('--group', choices=GROUPS, required=True); enter.add_argument('--file', required=True)
    capture = sub.add_parser('capture'); capture.add_argument('--session', required=True)
    capture.add_argument('--group', choices=GROUPS, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'init': result = init_session(args.session, args.mocap_config, args.map_source)
        elif args.command == 'freeze-inputs': result = freeze_inputs(args.session)
        elif args.command == 'check-inputs':
            cfg = load(Path(args.session)/'session.yaml'); body = load(Path(args.session)/'body_extrinsic.yaml')
            validate_inputs(cfg, body['rigid_body_to_base_link']); result = dict(inputs_valid=True)
        elif args.command == 'set-map': result = enter_map(args.session, args.group, args.file)
        elif args.command == 'solve': result = solve(args.session)
        elif args.command == 'verify': result = verify(args.session)
        else:
            from .world_capture import capture_static
            result = capture_static(args.session, args.group)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        if args.command == 'capture' and not result['quality_passed']: return 2
        if args.command == 'verify' and (not result['protocol_passed'] or result['numeric_acceptance_passed'] is False): return 2
        return 0
    except (ValueError, FileNotFoundError, FileExistsError, KeyError) as error:
        parser.exit(2, f'{error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
