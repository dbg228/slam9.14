#!/usr/bin/env python3
"""Four synthetic placements in an explicitly isolated ROS domain, no hardware."""
import argparse
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller_tracking import world_calibration as wc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', required=True, type=int)
    parser.add_argument('--output', required=True)
    parser.add_argument('--mocap-config', required=True)
    args = parser.parse_args()
    if not 100 <= args.domain <= 232 or args.domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        parser.error('Choose an isolated domain 100..232 different from the current domain')
    if Path(args.output).exists(): parser.error('Use a new artifact directory')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from std_msgs.msg import Bool
    from static_map_localization.msg import NdtMatchDiagnostic
    rclpy.init(); node = Node('fixed_world_software_fixture')
    try:
        until = time.monotonic()+2.
        while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.1)
        others = [name for name in node.get_node_names() if name != node.get_name()]
        if others: raise RuntimeError('Isolated ROS domain is occupied: '+str(others))
        s = Path(args.output).resolve()
        wc.init_session(s, args.mocap_config, 'ndt')
        cfg = wc.load(s/'session.yaml')
        cfg.update(data_kind='software_fixture', calibration_id='software_fixture_not_camera_calibration',
                   map_reference_id='software_fixture_not_real_map', physical_reference_confirmed=True)
        # Use the real collector's discovery warmup. Half a second can start
        # sampling before latched readiness/validity endpoints are discovered.
        cfg['limits'].update(seconds=3., warmup_s=2.)
        import yaml
        (s/'session.yaml').write_text(yaml.safe_dump(cfg))
        wc.freeze_inputs(s)
        raw = node.create_publisher(PoseStamped, cfg['normalizer']['input_topic'], 20)
        normalized = node.create_publisher(PoseStamped, cfg['normalizer']['output_topic'], 20)
        validity = node.create_publisher(DiagnosticArray, cfg['normalizer']['diagnostics_topic'], 20)
        ndt_pub = node.create_publisher(NdtMatchDiagnostic, cfg['topics']['ndt'], 20)
        command = node.create_publisher(Twist, cfg['topics']['command'], 20)
        ready = node.create_publisher(Bool, cfg['topics']['map_ready'],
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        ready.publish(Bool(data=True))
        state = dict(point=[0., 0., 0.], yaw=0., test=False, tick=0)
        offset = [.03, -.04, .12]
        def publish():
            stamp = node.get_clock().now().to_msg()
            point = state['point']; yaw = state['yaw']
            pose = PoseStamped(); pose.header.stamp = stamp; pose.header.frame_id = 'mocap_world'
            pose.pose.position.x = point[0]; pose.pose.position.y = point[1]; pose.pose.position.z = point[2]+.18
            pose.pose.orientation.z = math.sin(yaw/2); pose.pose.orientation.w = math.cos(yaw/2)
            normalized.publish(pose)
            pose.pose.position.x *= 1000; pose.pose.position.y *= 1000; pose.pose.position.z *= 1000
            raw.publish(pose)
            diagnostic = DiagnosticArray(); diagnostic.header = pose.header
            status = DiagnosticStatus(); status.name = 'mocap_tracker0_normalizer'
            status.level = DiagnosticStatus.OK; status.values = [KeyValue(key='valid', value='true')]
            diagnostic.status = [status]; validity.publish(diagnostic); command.publish(Twist())
            state['tick'] += 1
            if state['tick'] % 20: return
            msg = NdtMatchDiagnostic(); msg.header.stamp = stamp; msg.header.frame_id = 'map'; msg.cloud_stamp = stamp
            msg.match_id = state['tick']; msg.accepted = True; msg.stage = 'accepted'
            msg.cloud_to_odom_sync_valid = True; msg.cloud_to_odom_skew_sec = 0.
            msg.final_map_to_base_at_cloud_valid = True; msg.raw_ndt_map_to_base_at_cloud_valid = True
            p = msg.final_map_to_base_at_cloud
            c, sn = math.cos(.6), math.sin(.6)
            p.position.x = c*point[0]-sn*point[1]+2.
            p.position.y = sn*point[0]+c*point[1]-3.
            p.position.z = point[2]+.4
            if state['test']:
                p.position.x -= offset[0]; p.position.y -= offset[1]; p.position.z -= offset[2]
            p.orientation.z = math.sin((yaw+.6)/2); p.orientation.w = math.cos((yaw+.6)/2)
            msg.raw_ndt_map_to_base_at_cloud = p; ndt_pub.publish(msg)
        node.create_timer(.01, publish)
        tool = Path(__file__).with_name('world_calibration.py')
        points = [[0., 0., 0.], [1., 0., 0.], [.2, .8, 0.], [.6, .5, 0.]]
        for i, (group, point) in enumerate(zip(wc.GROUPS, points)):
            state.update(point=point, yaw=i*.35, test=group==wc.HOLDOUT)
            with (s/f'{group}_capture.log').open('x') as log:
                proc = subprocess.Popen([sys.executable, str(tool), 'capture', '--session', str(s), '--group', group],
                                        stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic()+20.
                    while proc.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.005)
                    if proc.poll() is None: proc.terminate(); proc.wait(timeout=5)
                    if proc.returncode != 0: raise RuntimeError(f'{group} capture failed: see {log.name}')
                finally:
                    if proc.poll() is None: proc.kill(); proc.wait()
            if group == wc.TRAIN[-1]: wc.solve(s)
        before = wc.sha(s/'calibration.yaml'); result = wc.verify(s)
        assert abs(result['errors']['distance_3d_m']['rmse']-.13) < 1e-8
        assert abs(result['errors']['distance_xy_m']['rmse']-.05) < 1e-8
        assert result['protocol_passed'] and wc.sha(s/'calibration.yaml') == before
        interrupted = s/'interrupted_capture'
        wc.init_session(interrupted, args.mocap_config, 'ndt')
        (interrupted/'session.yaml').write_text(yaml.safe_dump(cfg))
        wc.freeze_inputs(interrupted)
        with (interrupted/'capture.log').open('x') as log:
            proc = subprocess.Popen([sys.executable, str(tool), 'capture', '--session', str(interrupted),
                                     '--group', wc.TRAIN[0]], stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+2.
                while proc.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(node, timeout_sec=.005)
                if proc.poll() is not None: raise RuntimeError('Interrupt fixture exited before interruption')
                proc.send_signal(signal.SIGINT)
                deadline = time.monotonic()+10.
                while proc.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(node, timeout_sec=.005)
                if proc.poll() is None: raise RuntimeError('Interrupted capture did not finish')
                failed = wc.load(interrupted/wc.TRAIN[0]/'summary.json')
                assert proc.returncode==2 and not failed['quality_passed']
                assert any('KeyboardInterrupt' in reason for reason in failed['failures'])
                assert (interrupted/wc.TRAIN[0]/'raw.jsonl').stat().st_size>0
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
        wc.save(s/'software_checks.json', dict(data_kind='software_fixture', domain=args.domain,
            four_capture_commands_passed=True, injected_holdout_error_xyz_m=offset,
            recovered_holdout_3d_m=result['errors']['distance_3d_m']['rmse'],
            calibration_unchanged=True, interrupted_capture_retained=True, real_robot_tested=False))
        print('Synthetic four-placement capture/solve/holdout passed: '+str(s))
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
