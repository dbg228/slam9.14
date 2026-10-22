"""Group 3: explicitly requested static checkpoints, held outside mocap coverage."""
from collections import deque
import json
import time
import numpy as np
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .core import checked, checked_session, record, save, sha
from .checkpoints import correction_report, next_checkpoint, service_name
from .online import NativeBridge, tf_message


class CheckpointBridge(NativeBridge):
    def __init__(self,run,capture_dir=None,resume_state=None,maximum_native_age_s=None):
        self.active_checkpoint=None
        self.completed=[]
        self.last_checkpoint=None
        self.version=0
        self.nav_goals={}
        super().__init__(run,'localization',capture_dir,resume_state=resume_state,maximum_native_age_s=maximum_native_age_s)
        self.restart=self.capture
        if self.resume_state:
            self.version=int(self.resume_state['map_odom_version'])
            self.completed=list(self.resume_state['completed_checkpoints'])
            self.last_checkpoint=self.resume_state['last_checkpoint']
            self.publish_initial_transforms()
            self.publish_status('frozen_transform_resumed_without_native_restart_or_refit')
        for action in ('follow_path','navigate_to_pose','navigate_through_poses','follow_waypoints','spin','backup','drive_on_heading','assisted_teleop'):
            topic=f'/{action}/_action/status'
            from rclpy.qos import QoSProfile,DurabilityPolicy
            self.create_subscription(GoalStatusArray,topic,
                lambda msg,topic=topic:self.nav_goals.update({topic:[g.status for g in msg.status_list]}),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.checkpoint_services=[self.create_service(Trigger,service_name(self.restart,name),
            lambda request,response,name=name:self.request_checkpoint(name,response)) for name in ('middle','end')]
        if self.resume_state:
            self.idle_probes={}
            self.create_timer(.5,self.confirm_unseen_actions_idle)

    def confirm_unseen_actions_idle(self):
        # A new listener may never receive an empty action status. Ask those
        # servers to cancel all goals; only an explicit empty success confirms
        # idle. Existing statuses and active actions are never inferred away.
        from action_msgs.srv import CancelGoal
        for action in ('follow_path','navigate_to_pose','navigate_through_poses','follow_waypoints','spin','backup','drive_on_heading','assisted_teleop'):
            topic=f'/{action}/_action/status'
            if topic in self.nav_goals or topic in self.idle_probes or not self.get_publishers_info_by_topic(topic):continue
            client=self.create_client(CancelGoal,f'/{action}/_action/cancel_goal')
            if not client.service_is_ready():self.destroy_client(client);continue
            future=client.call_async(CancelGoal.Request());self.idle_probes[topic]=(client,future)
            def completed(future,topic=topic,client=client):
                try:
                    result=future.result()
                    if result.return_code==0 and not result.goals_canceling:
                        self.nav_goals.setdefault(topic,[])
                        self.event('empty_cancel_all_confirmed_idle:'+topic)
                finally:
                    self.idle_probes.pop(topic,None);self.destroy_client(client)
            future.add_done_callback(completed)

    def collecting(self):
        return self.h is None or self.active_checkpoint is not None

    def localization_metadata(self):
        return dict(initialization_only=False,continuous_mocap_correction=False,
                    checkpoint_corrections=True,checkpoint_order=['middle','end'],
                    map_odom_transport='dynamic_only')

    def publish_initial_transforms(self):
        # map->odom never appears on /tf_static in this mode.
        stamp=self.get_clock().now().to_msg()
        self.static.sendTransform(tf_message(np.eye(4),stamp,'odom','camera_init'))

    def publish_base(self,h,stamp):
        self.dynamic.sendTransform(tf_message(self.h,stamp,'map','odom'))
        super().publish_base(h,stamp)

    def publish_status(self,reason):
        self.status.publish(String(data=json.dumps(dict(
            run_id=self.run.name,run_path=str(self.run),
            capture_dir=str(getattr(self,'restart',self.capture)),
            active_capture_dir=str(self.capture),mode='checkpoints',context='localization',
            map_basis=self.cfg['map_basis'],frozen=self.h is not None,
            fault=self.fault,reason=reason,pairs=len(self.pairs),native_stamp_ns=self.last_native,
            source_mode='mocap_initial_then_lio_with_static_checkpoints',
            motion_commands_published=False,ndt_used=False,
            checkpoint_corrections=True,active_checkpoint=self.active_checkpoint,
            completed_checkpoints=self.completed,last_checkpoint=self.last_checkpoint,
            map_odom_version=self.version,
            maximum_native_age_s=self.maximum_native_age_s,
            T_map_odom=record(self.h,'map','odom') if self.h is not None else None))))

    def request_checkpoint(self,name,response):
        try:
            checked_session(self.run,self.restart)
            if self.fault or self.h is None:raise ValueError('Localization is not healthy and initialized')
            if self.active_checkpoint:raise ValueError('A checkpoint capture is already active')
            if not self.navigation_idle():raise ValueError('Cancel active navigation/behavior goals before correction')
            if name!=next_checkpoint(self.completed):raise ValueError('Expected checkpoint '+str(next_checkpoint(self.completed)))
            limits=self.cfg['initialization']
            now=time.monotonic()
            if not self.graph_verified or (self.get_clock().now().nanoseconds-self.last_native)*1e-9>self.maximum_native_age_s:
                raise ValueError('Need fresh native localization and verified TF owners')
            if self.last_command is None or now-self.command_at>limits['maximum_command_age_s'] or not np.isfinite(self.last_command).all() or np.max(abs(self.last_command))>limits['maximum_command_abs']:
                raise ValueError('Stop and cancel navigation, then wait for a fresh zero chassis command')
            target=self.restart/'checkpoints'/f'{name}_{time.time_ns()}'
            target.mkdir(parents=True,exist_ok=False)
            journal=(target/'raw.jsonl').open('x',buffering=1)
            self.capture=target;self.journal=journal;self.start=now
            self.pairs=[];self.pending=deque();self.mc.clear();self.raw.clear();self.normalized.clear()
            self.active_checkpoint=name;self.publish_status('checkpoint_capture_started')
            response.success=True
            response.message=json.dumps(dict(run_path=str(self.run),session=str(self.restart),checkpoint=name,capture_dir=str(target)))
        except (ValueError,OSError) as exc:
            response.success=False;response.message=str(exc)
        return response

    def navigation_idle(self):
        for action in ('follow_path','navigate_to_pose','navigate_through_poses','follow_waypoints','spin','backup','drive_on_heading','assisted_teleop'):
            topic=f'/{action}/_action/status'
            if self.get_publishers_info_by_topic(topic) and topic not in self.nav_goals:return False
        return not any(state in (1,2,3) for values in self.nav_goals.values() for state in values)

    def capture_timeout(self):
        if self.active_checkpoint:
            self.event('checkpoint_timeout_previous_transform_retained');self.journal.close()
            save(self.capture/'failure.json',dict(checkpoint=self.active_checkpoint,
                reason='no_valid_static_window_before_timeout',previous_transform_retained=True))
            self.last_checkpoint=dict(checkpoint=self.active_checkpoint,outcome='failed',capture_dir=str(self.capture))
            self.active_checkpoint=None;self.publish_status('checkpoint_failed_previous_transform_retained')
        else:super().capture_timeout()

    def freeze(self,result,units):
        if not self.active_checkpoint:
            super().freeze(result,units)
            return
        if not self.navigation_idle():
            self.invalid('navigation_goal_active_during_checkpoint');return
        checked(self.run,True);checked_session(self.run,self.restart)
        try:
            candidate,report=correction_report(self.pairs,self.cfg,self.calibration,self.h)
        except ValueError as exc:
            if str(exc)=='checkpoint_reference_not_upright':
                self.invalid(str(exc));return
            raise
        self.event('checkpoint_static_window_complete');self.journal.close()
        save(self.capture/'pairs.json',self.pairs)
        report.update(checkpoint=self.active_checkpoint,run_path=str(self.run),
            session=str(self.restart),camera_world_id=self.cfg.get('camera_world_id'),
            calibration_sha256=sha(self.run/'calibration.yaml'),
            initialization_sha256=sha(self.restart/'initialization.yaml'),
            raw_sha256=sha(self.capture/'raw.jsonl'),pairs_sha256=sha(self.capture/'pairs.json'),
            raw_normalized_check=units,clock_offset_verified=self.cfg['clock_offset_verified'],
            camera_latency_s=self.cfg['camera_latency_s'],map_odom_version=self.version+1,
            motion_commands_published=False,ndt_used=False)
        # A complete durable report is required before the active correction changes.
        save(self.capture/'summary.json',report)
        save(self.capture/'summary.lock.json',dict(sha256=sha(self.capture/'summary.json')))
        self.h=candidate;self.version+=1;self.completed.append(self.active_checkpoint)
        self.last_checkpoint=dict(checkpoint=self.active_checkpoint,outcome='applied',capture_dir=str(self.capture))
        self.active_checkpoint=None;self.publish_status('checkpoint_applied')
