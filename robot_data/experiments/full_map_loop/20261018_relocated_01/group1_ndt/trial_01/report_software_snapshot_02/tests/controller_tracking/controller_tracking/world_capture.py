"""Passive four-placement acquisition; ROS imports are local to capture_static.

The raw journal includes raw VRPN, validity, normalized Tracker0 poses, NDT
diagnostics and observed chassis commands. This module creates no publishers.
"""
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np

from .state import rotation
from .world_calibration import (GROUPS, HOLDOUT, TRAIN, check_calibration, frozen_inputs,
                                pose_xyz, save, sha, xyz)


def clean(value):
    """Unknown NaN diagnostic fields become null, never zero."""
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, bytes): return list(value)
    return value


def stamp_ns(stamp):
    return int(stamp.sec)*10**9 + int(stamp.nanosec)


def read_pose(message, stamp, receive_ns, frame):
    p, q = message.position, message.orientation
    result = dict(stamp_ns=int(stamp), receive_ros_ns=int(receive_ns), frame=frame,
                  xyz_m=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w])
    xyz(result['xyz_m']); rotation(result['quaternion_xyzw'])
    return result


def mean_rotation(rotations):
    u, _, vt = np.linalg.svd(np.mean(rotations, axis=0))
    correction = np.eye(3); correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def static_metrics(positions, rotations):
    positions = np.asarray(positions)
    center = positions.mean(axis=0); r = mean_rotation(rotations)
    angles = [math.degrees(math.acos(float(np.clip((np.trace(a @ r.T)-1)/2, -1, 1)))) for a in rotations]
    return dict(mean_xyz_m=center.tolist(), mean_rotation_matrix=r.tolist(),
                std_xyz_m=positions.std(axis=0).tolist(),
                maximum_radius_m=float(np.linalg.norm(positions-center, axis=1).max()),
                maximum_orientation_deviation_deg=float(max(angles)))


def check_raw_normalized(raw, normalized, scale):
    """Confirm actual runtime units/frame/stamps, not just the YAML assertion."""
    by_stamp = {p['stamp_ns']: p for p in raw}
    matched = 0; mismatches = []
    for pose in normalized:
        original = by_stamp.get(pose['stamp_ns'])
        if original is None: continue
        matched += 1
        try:
            if (original['frame'] != pose['frame'] or
                not np.allclose(xyz(original['xyz_m'])*scale, xyz(pose['xyz_m']), atol=1e-7, rtol=1e-8) or
                not np.allclose(rotation(original['quaternion_xyzw']), rotation(pose['quaternion_xyzw']), atol=1e-7)):
                mismatches.append(pose['stamp_ns'])
        except ValueError:
            mismatches.append(pose['stamp_ns'])
    return dict(matched_samples=matched, normalized_samples=len(normalized),
                coverage=matched/len(normalized) if normalized else 0.,
                scale_to_m=scale, mismatched_stamp_ns=mismatches,
                note='xyz_m in raw internal records retains native units until this explicit scale check')


def summarize(cfg, mount, mocap, ndt, commands, invalid_events=0):
    """All samples retained; no outlier trimming, interpolation or frame refit."""
    limits = cfg['limits']; errors = []; rejected = Counter(); pairs = []
    valid = []
    last = 0
    for sample in mocap:
        try:
            if sample['stamp_ns'] <= last: raise ValueError('duplicate_or_backward_mocap')
            if sample['frame'] != cfg['mocap_frame']: raise ValueError('unexpected_mocap_frame')
            age = (sample['receive_ros_ns']-sample['stamp_ns'])*1e-9
            if not 0 <= age <= limits['maximum_source_age_s']: raise ValueError('stale_mocap')
            p = pose_xyz(sample, mount['translation_m'])
            r = rotation(sample['quaternion_xyzw']) @ rotation(mount['quaternion_xyzw'])
            valid.append(dict(stamp_ns=sample['stamp_ns'], position=p, rotation=r))
            last = sample['stamp_ns']
        except (ValueError, KeyError, TypeError) as error:
            rejected[str(error)] += 1
    if len(valid) < limits['minimum_mocap_samples']: errors.append('insufficient_mocap_samples')
    if invalid_events: errors.append('mocap_invalid_or_unpaired_events_during_window')
    mocap_stats = None; map_stats = None
    if valid:
        stamps = np.array([s['stamp_ns'] for s in valid], dtype=np.int64)
        span = float((stamps[-1]-stamps[0])*1e-9)
        maximum_gap = float(np.diff(stamps).max()*1e-9) if len(stamps)>1 else None
        if span < limits['seconds']*.90: errors.append('insufficient_mocap_time_coverage')
        if maximum_gap is not None and maximum_gap > limits['maximum_mocap_gap_s']: errors.append('long_mocap_gap')
        mocap_stats = static_metrics([s['position'] for s in valid], [s['rotation'] for s in valid])
        if mocap_stats['maximum_radius_m'] > limits['maximum_mocap_radius_m']: errors.append('mocap_not_static')
        if mocap_stats['maximum_orientation_deviation_deg'] > limits['maximum_orientation_deviation_deg']: errors.append('mocap_orientation_not_static')
        seen_ids = set(); last_ndt = 0
        for record in ndt:
            try:
                stamp = record['stamp_ns']; identity = record['match_id']
                if not record.get('map_ready', False) or record['stage']=='waiting_for_initialization':
                    errors.append('ndt_not_ready_during_capture')
                    raise ValueError('ndt_not_initialized_or_no_accepted_map_registration')
                if identity in seen_ids: raise ValueError('duplicate_ndt_match_id')
                seen_ids.add(identity)
                if stamp <= last_ndt: raise ValueError('duplicate_or_backward_cloud_stamp')
                if not record['valid']: raise ValueError('ndt_pose_at_cloud_invalid')
                if record['frame'] != cfg['map_frame']: raise ValueError('unexpected_ndt_frame')
                if not record['sync_valid'] or abs(record['cloud_odom_skew_s']) > limits['maximum_cloud_odom_skew_s']:
                    raise ValueError('ndt_cloud_odom_not_synchronized')
                age = (record['receive_ros_ns']-stamp)*1e-9
                if not 0 <= age <= limits['maximum_source_age_s']: raise ValueError('stale_ndt_measurement')
                target = stamp + round(cfg['clock_offset_s']*1e9)
                j = int(np.searchsorted(stamps, target))
                candidates = [k for k in (j-1, j) if 0 <= k < len(valid)]
                k = min(candidates, key=lambda k: abs(int(stamps[k])-target))
                skew = abs(int(stamps[k])-target)*1e-9
                if skew > limits['maximum_pair_skew_s']: raise ValueError('no_nearby_valid_mocap')
                # Match source time, never the NDT publication header time.
                mapped = xyz(record['pose']['xyz_m']); mapped_r = rotation(record['pose']['quaternion_xyzw'])
                pairs.append(dict(ndt_stamp_ns=stamp, mocap_stamp_ns=int(stamps[k]), skew_s=skew,
                                  match_id=identity, map_xyz_m=mapped.tolist(),
                                  mocap_xyz_m=valid[k]['position'].tolist(),
                                  map_rotation=mapped_r.tolist(), mocap_rotation=valid[k]['rotation'].tolist(),
                                  accepted=record['accepted'], stage=record['stage']))
                last_ndt = stamp
            except (ValueError, KeyError, TypeError) as error:
                rejected[str(error)] += 1
        if cfg['map_source'] == 'ndt':
            if len(pairs) < limits['minimum_pairs']: errors.append('insufficient_ndt_mocap_pairs')
            if pairs:
                paired_stamps = [p['ndt_stamp_ns'] for p in pairs]
                if len(pairs)>1 and max(np.diff(paired_stamps))*1e-9 > limits['maximum_ndt_gap_s']: errors.append('long_ndt_gap')
                if (paired_stamps[-1]-paired_stamps[0])*1e-9 < limits['seconds']*.50: errors.append('insufficient_ndt_time_coverage')
                map_stats = static_metrics([p['map_xyz_m'] for p in pairs], [np.asarray(p['map_rotation']) for p in pairs])
                if map_stats['maximum_radius_m'] > limits['maximum_map_radius_m']: errors.append('map_not_static')
                if map_stats['maximum_orientation_deviation_deg'] > limits['maximum_orientation_deviation_deg']: errors.append('map_orientation_not_static')
    else:
        span = 0.; maximum_gap = None
    if commands and not np.isfinite(commands).all():
        errors.append('nonfinite_final_command')
        maximum_command = None
    else:
        maximum_command = float(np.max(np.abs(commands))) if commands else None
    if maximum_command is None: errors.append('no_observed_final_command')
    elif maximum_command > limits['maximum_command_abs']: errors.append('nonzero_final_command')
    # NDT mode uses the same paired instants on both sides, not different means.
    mocap_mean = np.mean([p['mocap_xyz_m'] for p in pairs], axis=0).tolist() if pairs and cfg['map_source']=='ndt' else (mocap_stats['mean_xyz_m'] if mocap_stats else None)
    provenance = dict(source=cfg['map_source'], method=cfg['map_method'], independently_measured=False,
                      ndt_pose_field=cfg['ndt_pose_field'], topic=cfg['topics']['ndt'],
                      stamp_meaning='input cloud measurement time, not publication time') if cfg['map_source']=='ndt' else dict(source='manual_pending')
    return dict(quality_passed=not errors, failures=errors, rejection_counts=dict(rejected),
        valid_mocap_samples=len(valid), mocap_span_s=span, maximum_mocap_gap_s=maximum_gap,
        matched_pairs=pairs, pair_count=len(pairs), maximum_command_abs=maximum_command,
        mocap_xyz_m=mocap_mean, map_xyz_m=map_stats['mean_xyz_m'] if map_stats else None,
        mocap_base_rotation_matrix=mocap_stats['mean_rotation_matrix'] if mocap_stats else None,
        mocap_static=mocap_stats, map_static=map_stats, map_provenance=provenance,
        raw_marker_positions_available=False,
        mocap_point_method='Tracker0 world pose * separately measured Tracker0->base_link; not individual marker coordinates',
        camera_native_visibility_available=False, interpolation_used=False,
        clock_offset_s=cfg['clock_offset_s'], clock_offset_verified=cfg['clock_offset_verified'],
        camera_latency_s=cfg['camera_latency_s'])


def capture_static(session, group):
    s = Path(session); inputs = frozen_inputs(s); cfg = inputs['config']
    mount = inputs['body_extrinsic']['rigid_body_to_base_link']
    evaluation=group in ('initial','endpoint') and cfg.get('evaluation_only') is True
    fixed=None
    if evaluation:
        fixed=Path(cfg['fixed_calibration_session']).resolve()
        calibration=check_calibration(fixed)
        if calibration['calibration_id']!=cfg['calibration_id']:
            raise ValueError('Evaluation camera calibration differs from the frozen registration')
    elif group not in GROUPS: raise ValueError('Unknown placement label')
    if group == HOLDOUT:
        check_calibration(s)
    elif (s/'calibration.yaml').exists():
        raise ValueError('Calibration already frozen; no new training capture allowed')
    if group in TRAIN:
        for previous in TRAIN[:TRAIN.index(group)]:
            if not (s/previous/'summary.json').exists(): raise ValueError('Capture training placements in order')
    destination = s/group
    if destination.exists() and (destination/'raw.jsonl').exists():
        raise ValueError('Placement capture already exists; retain failures, do not overwrite')
    preroll=cfg.get('retain_mocap_warmup',False)
    phase=cfg.get('ndt_capture_phase') if cfg['map_source']=='ndt' else None
    if phase:
        period=float(phase['period_s']);lead=float(phase['lead_s']);wait=float(phase['maximum_wait_s'])
        if not all(math.isfinite(v) and v>0 for v in (period,lead,wait)) or lead>=period:
            raise ValueError('Invalid NDT capture phase timing')
    # Imported only for passive acquisition; offline solving needs no ROS.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Bool
    from rosidl_runtime_py.convert import message_to_ordereddict
    from .feedback import PairedMocap
    if cfg['map_source'] == 'ndt':
        from static_map_localization.msg import NdtMatchDiagnostic
    destination.mkdir(parents=True, exist_ok=True)
    journal = (destination/'raw.jsonl').open('x', encoding='utf-8')
    mocap = []; raw_poses = []; ndt = []; commands = []; invalid = Counter(); observed = Counter(); failure = None
    map_ready = [False]
    ready_topic = cfg['topics'].get('map_ready', '/static_localization/ready')
    # Keep the context alive through Ctrl+C so raw data and failure summaries
    # are flushed before shutdown, instead of failing during graph inspection.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('fixed_world_calibration_capture')
    started = time.monotonic(); begin = started+cfg['limits']['warmup_s']; end = begin+cfg['limits']['seconds']
    # Optional static-only protocol: retain the real warmup history for delayed
    # NDT stamps, and choose a window by source cadence, never by residuals.
    phase_info=None
    hard_end=end+.20
    if phase:
        begin=end=float('inf')
        hard_end=started+cfg['limits']['warmup_s']+wait+period+cfg['limits']['seconds']+.20

    def in_window(): return begin <= time.monotonic() <= end

    def log(topic, msg):
        if in_window(): observed[topic] += 1
        if (in_window() or preroll) and topic == cfg['normalizer']['input_topic']:
            try:
                raw_poses.append(read_pose(msg.pose, stamp_ns(msg.header.stamp), node.get_clock().now().nanoseconds, msg.header.frame_id))
            except ValueError:
                pass  # The original malformed message is still journaled below.
        journal.write(json.dumps(clean(dict(receive_ros_ns=node.get_clock().now().nanoseconds,
            in_capture_window=in_window(), topic=topic, message=message_to_ordereddict(msg))),
            ensure_ascii=False, allow_nan=False)+'\n')

    def good(msg):
        if in_window() or preroll:
            try:
                mocap.append(read_pose(msg.pose, stamp_ns(msg.header.stamp), node.get_clock().now().nanoseconds, msg.header.frame_id))
            except ValueError as error: invalid[str(error)] += 1

    def bad(reason):
        if in_window(): invalid[reason] += 1

    pair_cfg = dict(topics=dict(mocap=cfg['normalizer']['output_topic'], validity=cfg['normalizer']['diagnostics_topic']),
                    timing=dict(max_age_s=cfg['limits']['maximum_source_age_s'], validity_wait_s=.15))
    paired = PairedMocap(node, pair_cfg, good, bad)
    # Journal is independent from validity association: rejected raw input survives.
    for topic, message_type in ((cfg['normalizer']['input_topic'], PoseStamped),
                                (cfg['normalizer']['output_topic'], PoseStamped),
                                (cfg['normalizer']['diagnostics_topic'], DiagnosticArray)):
        node.create_subscription(message_type, topic, lambda msg, topic=topic: log(topic, msg), qos_profile_sensor_data)

    def command(msg):
        log(cfg['topics']['command'], msg)
        if in_window(): commands.append([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])

    node.create_subscription(Twist, cfg['topics']['command'], command, qos_profile_sensor_data)

    def ndt_callback(msg):
        nonlocal begin,end,phase_info
        now=time.monotonic()
        if phase and phase_info is None and now-started>=cfg['limits']['warmup_s'] and map_ready[0]:
            begin=now+period-lead;end=begin+cfg['limits']['seconds']
            phase_info=dict(trigger_match_id=int(msg.match_id),trigger_cloud_stamp_ns=stamp_ns(msg.cloud_stamp),
                period_s=period,lead_s=lead,begin_after_start_s=begin-started,
                selection='fixed cadence only; no residual or fitness selection')
        log(cfg['topics']['ndt'], msg)
        if not in_window(): return
        field = cfg['ndt_pose_field']; stamp = stamp_ns(msg.cloud_stamp)
        try:
            pose = read_pose(getattr(msg, field), stamp, node.get_clock().now().nanoseconds, msg.header.frame_id)
            ndt.append(dict(stamp_ns=stamp, receive_ros_ns=pose['receive_ros_ns'], frame=msg.header.frame_id,
                valid=bool(getattr(msg, field+'_valid')), sync_valid=msg.cloud_to_odom_sync_valid,
                cloud_odom_skew_s=msg.cloud_to_odom_skew_sec, pose=pose,
                match_id=int(msg.match_id), accepted=msg.accepted, stage=msg.stage, map_ready=map_ready[0]))
        except ValueError as error: invalid['invalid_ndt_pose:'+str(error)] += 1

    if cfg['map_source'] == 'ndt':
        def ready_callback(msg):
            log(ready_topic, msg); map_ready[0] = msg.data
        node.create_subscription(Bool, ready_topic, ready_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(NdtMatchDiagnostic, cfg['topics']['ndt'], ndt_callback, 50)
    node.get_logger().info(f'{group}: stationary capture only; no goals, commands, parameters or TF published')
    try:
        while rclpy.ok() and time.monotonic() < min(end+.20,hard_end):
            rclpy.spin_once(node, timeout_sec=.01)
    except (KeyboardInterrupt, Exception) as error:
        failure = type(error).__name__+': '+str(error)
    finally:
        try:
            graph = {topic: [dict(node_name=p.node_name, node_namespace=p.node_namespace,
                                  topic_type=p.topic_type) for p in node.get_publishers_info_by_topic(topic)]
                     for topic in [cfg['normalizer'][k] for k in ('input_topic', 'output_topic', 'diagnostics_topic')]+
                     [cfg['topics']['command']]+([cfg['topics']['ndt'], ready_topic] if cfg['map_source']=='ndt' else [])}
        except Exception as error:
            graph = {}; failure = failure or 'graph_inspection_failed:'+str(error)
        journal.flush(); journal.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    result = summarize(cfg, mount, mocap, ndt, commands, sum(invalid.values()))
    result.update(retained_mocap_warmup=preroll,ndt_capture_phase=phase_info,
        source_age_limit_s=cfg['limits']['maximum_source_age_s'],
        source_age_note='Measured timestamps are unchanged. Warmup reference history is used only for static pairing; this is not a claim of real-time control freshness.')
    if phase and phase_info is None:
        result['quality_passed']=False;result['failures'].append('no_ready_ndt_phase_before_timeout')
    units = check_raw_normalized(raw_poses, mocap, cfg['mocap']['position_scale_to_m'])
    result['raw_normalized_check'] = units; result['publisher_graph'] = graph
    result['map_ready_at_end'] = map_ready[0] if cfg['map_source']=='ndt' else None
    if cfg['map_source']=='ndt' and not map_ready[0]:
        result['quality_passed'] = False; result['failures'].append('map_not_ready_at_capture_end')
    if units['coverage'] < cfg['limits']['minimum_raw_normalized_coverage'] or units['mismatched_stamp_ns']:
        result['quality_passed'] = False; result['failures'].append('raw_normalized_units_or_coverage_mismatch')
    if any(len(publishers) != 1 for publishers in graph.values()):
        result['quality_passed'] = False; result['failures'].append('missing_or_multiple_source_publishers')
    result['observed_messages_in_window'] = dict(observed)
    for topic in (cfg['normalizer']['input_topic'], cfg['normalizer']['output_topic'], cfg['normalizer']['diagnostics_topic']):
        if not observed[topic]:
            result['quality_passed'] = False; result['failures'].append('missing_raw_or_normalized_stream:'+topic)
    result.update(group=group, inputs_sha256=sha(s/'frozen_inputs.json'), raw_sha256=sha(destination/'raw.jsonl'),
                  data_kind=cfg['data_kind'], invalid_events=dict(invalid), no_motion_published=True,
                  calibration_sha256_at_capture=sha(fixed/'calibration.yaml') if evaluation else (sha(s/'calibration.yaml') if group==HOLDOUT else None),
                  evaluation_only=evaluation,transform_refitted=False)
    if failure:
        result['quality_passed'] = False; result['failures'].append(failure)
    save(destination/'summary.json', result)
    return result
