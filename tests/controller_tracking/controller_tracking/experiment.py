"""Explicit user-invoked experiment operations. No startup path sends a motion goal."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import shutil
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as RosPath, Odometry
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python
from lifecycle_msgs.srv import GetState
from .common import digest, write_json, wrap
from .configuration import load, snapshot, check_session
from .geometry import geometry, site_check
from .feedback import PairedMocap, converted, ns, set_stamp, xy_yaw
from .state import freeze_anchor


def spin(node,seconds):
    end=time.monotonic()+seconds
    while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=min(.02,max(0,end-time.monotonic())))


def wait(node,future,seconds=5):
    end=time.monotonic()+seconds
    while not future.done() and rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    if not future.done():raise RuntimeError('service_or_action_timeout')
    return future.result()


def service(node,typ,name,req):
    client=node.create_client(typ,name)
    try:
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('missing_service:'+name)
        return wait(node,client.call_async(req))
    finally:node.destroy_client(client)


def freeze(config,output):
    cfg=load(config);cal=cfg['calibration']
    if not cal['id'] or not cal['level_verified'] or cal['world_level_rotation_xyzw'] is None:
        raise ValueError('Fill measured camera calibration id and verified ground rotation before freeze')
    paths={k:geometry(cfg,k) for k in ('line','left_arc','right_arc')}
    # Require only the configured route to fit; other routes remain previews and cannot execute unless checked.
    if not all(site_check(paths[cfg['geometry']['kind']],cfg).values()):
        raise ValueError('Selected path sweep/parking/visibility/site confirmation incomplete; use preview first')
    mocap=yaml.safe_load(Path(cfg['mocap_config']).read_text())
    if cfg['timing']['source']!=mocap['mocap']['timestamp_source'] or mocap['normalizer']['output_frame']!='mocap_world':
        raise ValueError('Mocap clock/frame configuration mismatch')
    node=Node('tracking_freeze');rows=[];errors=[]
    def bad(reason):rows.clear();errors.append(reason)
    def good(msg):
        try:
            t=ns(msg.header.stamp)*1e-9;p=converted(msg,cfg,mocap)
            if rows and not 0<t-rows[-1][0]<=cfg['timing']['max_gap_s']:rows.clear()
            rows.append([t,*p])
            while len(rows)>2 and rows[1][0]<t-1.:rows.pop(0)
        except ValueError as e:bad(str(e))
    paired=PairedMocap(node,cfg,good,bad)
    anchor=None;deadline=time.monotonic()+15
    try:
        while time.monotonic()<deadline:
            spin(node,.02)
            try:anchor=freeze_anchor(rows)
            except ValueError:continue
            break
        if anchor is None:raise RuntimeError('No valid stationary mocap window: '+str(errors[-5:]))
        output=Path(output);output.mkdir(parents=True,exist_ok=False)
        snapshot(cfg,output)
        write_json(output/'frame.json',dict(frame=cfg['frame'],reference_point=cfg['base_frame'],
            anchor_level_xyyaw=anchor,world_level_rotation_xyzw=cal['world_level_rotation_xyzw'],
            calibration_id=cal['id'],config_sha256=digest(cfg),frozen=True,
            first_source_s=rows[0][0],last_source_s=rows[-1][0],samples=len(rows),
            method='one stationary mocap window; level rotation measured separately; no map or FAST-LIO'))
        identity=json.loads((output/'identity.json').read_text())
        identity['frame_sha256']=digest(json.loads((output/'frame.json').read_text()))
        write_json(output/'identity.json',identity)
        for kind,path in paths.items():
            path['frame_sha256']=digest(json.loads((output/'frame.json').read_text()))
            write_json(output/(kind+'.json'),path)
        return dict(session=str(output.resolve()),anchor=anchor,source='mocap_only')
    finally:node.destroy_node()


class Inspector(Node):
    def __init__(self,cfg):
        super().__init__('experiment_client_'+str(os.getpid()),namespace='tracking');self.cfg=cfg;self.status=None;self.gate=None;self.state=None;self.mode=None
        self.last_state_received=0;self.last_status_received=0;self.final=[];self.received_plan=None
        self.create_subscription(String,'/tracking/state_status',self.on_status,50)
        self.create_subscription(String,'/tracking/gate_status',self.on_gate,50)
        self.create_subscription(Odometry,'/tracking/state',self.on_state,50)
        self.create_subscription(String,'/teleop/mode',self.on_mode,10)
        self.create_subscription(Twist,'/reference_velocity',self.on_final,50)
        self.create_subscription(RosPath,'/tracking/received_plan',self.on_plan,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_status(self,m):self.status=json.loads(m.data);self.last_status_received=time.monotonic()
    def on_gate(self,m):self.gate=json.loads(m.data)
    def on_state(self,m):self.state=m;self.last_state_received=time.monotonic()
    def on_mode(self,m):self.mode=m.data
    def on_final(self,m):
        self.final.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z])
        self.final=self.final[-1000:]
    def on_plan(self,m):self.received_plan=m

    def inspect(self,parameters=True):
        # DDS graph discovery is asynchronous; do not turn a transient empty graph
        # into an apparent missing-control-chain failure (or weaken any checks).
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            spin(self,.1)
            ready=all(self.get_publishers_info_by_topic(t) for t in
                ('/nav/cmd_vel','/tracking/state','/tracking/tf','/tracking/controller_cmd','/tracking/smoothed_cmd'))
            if ready and self.status and self.state:break
        errors=[];nodes=self.get_node_names_and_namespaces();graph={}
        for topic in ('/nav/cmd_vel','/tracking/state','/tracking/tf','/tracking/controller_cmd','/tracking/smoothed_cmd'):
            graph[topic]=[p.node_namespace.rstrip('/')+'/'+p.node_name for p in self.get_publishers_info_by_topic(topic)]
        expected={'/nav/cmd_vel':'/tracking/command_gate','/tracking/state':'/tracking/mocap_feedback',
                  '/tracking/tf':'/tracking/mocap_feedback','/tracking/controller_cmd':'/tracking/controller_server',
                  '/tracking/smoothed_cmd':'/tracking/velocity_smoother'}
        for topic,publisher in expected.items():
            if graph[topic]!=[publisher]:errors.append('publisher mismatch '+topic+': '+str(graph[topic]))
        subscriptions={}
        for node,namespace in nodes:
            if namespace.startswith('/tracking'):
                key=namespace.rstrip('/')+'/'+node
                subscriptions[key]=self.get_subscriber_names_and_types_by_node(node,namespace)
                forbidden=[t for t,_ in subscriptions[key] if t in ('/odom','/Odometry','/tf','/tf_lio_raw','/static_localization/pose')]
                if forbidden:errors.append(key+' retains original localization subscriptions: '+str(forbidden))
        if not self.status or not self.status.get('valid') or time.monotonic()-self.last_status_received>self.cfg['timing']['max_age_s']:
            errors.append('mocap feedback not valid/fresh')
        if not self.state or time.monotonic()-self.last_state_received>self.cfg['timing']['max_age_s']:
            errors.append('no fresh coherent state')
        runtime={};lifecycle={}
        if parameters:
            for name in ('controller_server','velocity_smoother','local_costmap/local_costmap'):
                full='/tracking/'+name
                try:
                    listed=service(self,ListParameters,full+'/list_parameters',ListParameters.Request(depth=100)).result.names
                    values=service(self,GetParameters,full+'/get_parameters',GetParameters.Request(names=listed)).values
                    runtime[full]={n:parameter_value_to_python(v) for n,v in zip(listed,values)}
                except RuntimeError as e:errors.append(str(e))
            for name in ('controller_server','velocity_smoother'):
                try:
                    state=service(self,GetState,'/tracking/'+name+'/get_state',GetState.Request()).current_state
                    lifecycle[name]=state.label
                    if state.id!=3:errors.append(name+' is not active')
                except RuntimeError as e:errors.append(str(e))
            if runtime.get('/tracking/controller_server',{}).get('odom_topic')!='/tracking/state':errors.append('controller speed is not mocap')
            if runtime.get('/tracking/velocity_smoother',{}).get('feedback')!='OPEN_LOOP':errors.append('unexpected smoother feedback mode')
        return dict(ok=not errors,errors=errors,publishers=graph,subscriptions=subscriptions,
                    runtime_parameters=runtime,lifecycle=lifecycle,state_status=self.status,gate_status=self.gate,mode=self.mode,
                    timestamp_note='pose/speed source time on state; MPPI input_speed stamp is invocation time')


def path_message(path,node):
    msg=RosPath();set_stamp(msg.header,node.get_clock().now().nanoseconds,path['frame'])
    for point in path['points']:
        pose=PoseStamped();pose.header=msg.header;xy_yaw(pose.pose,point);msg.poses.append(pose)
    return msg


def path_points(msg):
    return [[p.pose.position.x,p.pose.position.y,math.atan2(2*p.pose.orientation.w*p.pose.orientation.z,
            1-2*p.pose.orientation.z**2)] for p in msg.poses]


def recorder_alive(run):
    manifest=json.loads((Path(run)/'recording.json').read_text())
    try:
        cmd=Path('/proc',str(manifest['pid']),'cmdline').read_bytes().replace(b'\0',b' ').decode()
        return 'ros2 bag record' in cmd and manifest['bag'] in cmd
    except FileNotFoundError:return False


def record(session,run,kind,seconds=None):
    session=Path(session).resolve();cfg,_,_=check_session(session)
    path=json.loads((session/(kind+'.json')).read_text())
    run=Path(run).resolve();run.mkdir(parents=True,exist_ok=False)
    shutil.copytree(session,run/'session',ignore=shutil.ignore_patterns('runs'))
    write_json(run/'path.json',path)
    node=Inspector(cfg);process=None;log=None
    event=node.create_publisher(String,'/tracking/events',50)
    try:
        audit=node.inspect();write_json(run/'data_check.json',audit)
        # Passive recording is allowed without valid feedback; preserve failure evidence.
        topics=list(set(cfg['topics'].values())|{'/tracking/tf','/tracking/tf_static','/tf_static',
            '/tracking/input_pose','/tracking/input_speed','/tracking/received_plan','/tracking/compute_timing',
            '/tracking/reference_path','/tracking/events','/tracking/gate_status','/rosout','/parameter_events'})
        share=Path(__file__).resolve().parents[1]/'config'
        if not share.exists():
            from ament_index_python.packages import get_package_share_directory
            share=Path(get_package_share_directory('controller_tracking'))/'config'
        argv=['ros2','bag','record','-o',str(run/'bag'),'--storage','sqlite3',
              '--storage-config-file',str(share/'sqlite_recording.yaml'),'--qos-profile-overrides-path',str(share/'bag_qos.yaml'),*sorted(topics)]
        log=(run/'recorder.log').open('w');process=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        write_json(run/'recording.json',dict(pid=process.pid,argv=argv,bag=str(run/'bag'),
            session=str(session),path_kind=kind,source='software_fixture' if os.environ.get('TRACKING_SOFTWARE_TEST_DOMAIN') else 'real_bag',started_ros_ns=node.get_clock().now().nanoseconds))
        deadline=time.monotonic()+seconds if seconds else math.inf
        while process.poll() is None and time.monotonic()<deadline:
            spin(node,.1)
            event.publish(String(data=json.dumps(dict(event='recorder_heartbeat',ros_ns=node.get_clock().now().nanoseconds,pid=process.pid))))
            # One heartbeat per 0.5 s, no SQLite reads during recording.
            spin(node,.4)
    except KeyboardInterrupt:pass
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
        if process:write_json(run/'recording_result.json',dict(exit_code=process.returncode,stopped_ros_ns=node.get_clock().now().nanoseconds))
        if log:log.close()
        node.destroy_node()


def flatten(obj,prefix=''):
    out={}
    for k,v in obj.items():
        if isinstance(v,dict):out.update(flatten(v,prefix+k+'.'))
        else:out[prefix+k]=v
    return out


def execute(run):
    run=Path(run).resolve();cfg,_,_=check_session(run/'session')
    if (run/'execution.json').exists():raise ValueError('Round already attempted; preserve it and use a new round directory')
    path=json.loads((run/'path.json').read_text());node=Inspector(cfg)
    if digest(path['points'])!=path['points_sha256'] or path['config_sha256']!=digest(cfg):
        node.destroy_node();raise ValueError('Frozen path or configuration integrity mismatch')
    event=node.create_publisher(String,'/tracking/events',100)
    lease=node.create_publisher(Empty,'/tracking/execution_lease',10)
    reference=node.create_publisher(RosPath,'/tracking/reference_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    action=ActionClient(node,FollowPath,'/tracking/follow_path');goal=None;armed=False;events=[]
    result=dict(source='software_fixture' if os.environ.get('TRACKING_SOFTWARE_TEST_DOMAIN') else 'real_manual_execution',success=False,reason='preflight_failed',path_sha256=path['points_sha256'])
    def emit(name,**kw):
        e=dict(event=name,ros_ns=node.get_clock().now().nanoseconds,**kw);events.append(e)
        with (run/'events.jsonl').open('a') as f:f.write(json.dumps(e)+'\n')
        event.publish(String(data=json.dumps(e)));return e
    try:
        audit=node.inspect();write_json(run/'preflight.json',audit)
        if not audit['ok']:raise RuntimeError('; '.join(audit['errors']))
        expected=yaml.safe_load((run/'session/nav2.yaml').read_text())['tracking']
        for name,values in expected.items():
            params=values['local_costmap']['ros__parameters'] if name=='local_costmap' else values['ros__parameters']
            full='/tracking/local_costmap/local_costmap' if name=='local_costmap' else '/tracking/'+name
            actual=audit['runtime_parameters'].get(full,{})
            # Undeclared configuration keys are reported, not silently treated as active settings.
            differences=[k for k,v in flatten(params).items() if k in actual and actual[k]!=v]
            if differences:raise RuntimeError('runtime parameters changed: '+str(differences))
        if not all(site_check(path,cfg).values()):raise RuntimeError('route sweep outside confirmed site')
        if not recorder_alive(run):raise RuntimeError('no recorder for this round')
        # Recorder must have discovered key endpoints before the single FollowPath goal.
        for topic in ('/tracking/reference_path','/tracking/received_plan','/tracking/compute_timing','/tracking/state'):
            if not any('rosbag' in s.node_name for s in node.get_subscriptions_info_by_topic(topic)):
                raise RuntimeError('recorder has not discovered '+topic)
        pose=node.state.pose.pose;xy=[pose.position.x,pose.position.y]
        yaw=math.atan2(2*pose.orientation.w*pose.orientation.z,1-2*pose.orientation.z**2)
        if np.linalg.norm(np.array(xy)-path['points'][0][:2])>cfg['execution']['start_position_tolerance_m'] or abs(wrap(yaw-path['points'][0][2]))>cfg['execution']['start_yaw_tolerance_rad']:
            raise RuntimeError('manual_reset_required; no automatic approach segment')
        if node.mode!='EXTERNAL':raise RuntimeError('manual_switch_to_EXTERNAL_required')
        if not action.wait_for_server(timeout_sec=3):raise RuntimeError('FollowPath server unavailable')
        arm=service(node,SetBool,'/tracking/arm',SetBool.Request(data=True))
        if not arm.success:raise RuntimeError(arm.message)
        armed=True;timer=node.create_timer(.05,lambda:lease.publish(Empty()))
        msg=path_message(path,node);reference.publish(msg)
        request=FollowPath.Goal();request.path=msg;request.controller_id='FollowPath';request.goal_checker_id='goal_checker'
        from rclpy.serialization import serialize_message
        import hashlib
        result['serialized_goal_sha256']=hashlib.sha256(serialize_message(request)).hexdigest()
        node.received_plan=None
        emit('goal_sent',path_sha256=path['points_sha256'])
        result['start_ros_ns']=events[-1]['ros_ns']
        goal=wait(node,action.send_goal_async(request))
        if not goal.accepted:raise RuntimeError('goal_rejected')
        result['goal_uuid']=[int(x) for x in goal.goal_id.uuid];emit('goal_accepted',uuid=result['goal_uuid'])
        future=goal.get_result_async();deadline=time.monotonic()+cfg['execution']['timeout_s'];checked=False
        while not future.done():
            spin(node,.02)
            if time.monotonic()>deadline:raise RuntimeError('experiment_timeout')
            if not recorder_alive(run):raise RuntimeError('recorder_died')
            if not node.status or not node.status['valid'] or time.monotonic()-node.last_status_received>cfg['timing']['max_age_s']:
                raise RuntimeError('mocap_invalid_or_stale')
            if node.gate and node.gate['reason']!='armed':raise RuntimeError('gate:'+node.gate['reason'])
            if node.received_plan is not None and ns(node.received_plan.header.stamp)==ns(msg.header.stamp) and not checked:
                if node.received_plan.header.frame_id!=path['frame']:raise RuntimeError('MPPI_received_path_frame_mismatch')
                actual=np.array(path_points(node.received_plan))
                if actual.shape!=np.array(path['points']).shape or not np.allclose(actual,np.array(path['points']),atol=1e-9):
                    raise RuntimeError('MPPI_received_path_mismatch')
                write_json(run/'received_path_check.json',dict(verified=True,points=len(actual),basis='delegate echoes only after original MPPI setPlan returns'))
                write_json(run/'received_path.json',dict(frame=node.received_plan.header.frame_id,request_stamp_ns=ns(msg.header.stamp),points=actual.tolist()))
                checked=True;emit('plugin_path_verified')
        reply=future.result();result['action_status']=reply.status;result['success']=reply.status==4
        result['reason']='succeeded' if result['success'] else 'action_status_'+str(reply.status)
        if not checked:result['success']=False;result['reason']='missing_plugin_path_confirmation'
        emit('goal_result',status=reply.status)
    except (RuntimeError,ValueError,KeyboardInterrupt) as e:
        result['reason']=str(e) or 'operator_interrupt';emit('failure',reason=result['reason'])
    finally:
        # Block the command relay first; cancellation can take a service round trip.
        if armed:
            try:service(node,SetBool,'/tracking/arm',SetBool.Request(data=False))
            except RuntimeError as e:emit('disarm_unconfirmed',reason=str(e))
        if goal and goal.accepted and result['reason']!='succeeded':
            try:
                cancel=wait(node,goal.cancel_goal_async());emit('cancel_response',return_code=cancel.return_code)
            except RuntimeError as e:emit('cancel_unconfirmed',reason=str(e))
        result['end_ros_ns']=node.get_clock().now().nanoseconds
        spin(node,1.2)
        recent=[row[1:] for row in node.final if time.monotonic()-row[0]<.5]
        result['stop_check']=dict(samples=len(recent),zero=bool(recent) and bool(np.max(abs(np.asarray(recent)))<1e-6))
        if not result['stop_check']['zero']:
            result['success']=False;result['reason']+='; final_zero_command_unconfirmed'
        result['duration_s']=(result['end_ros_ns']-result.get('start_ros_ns',result['end_ros_ns']))*1e-9
        write_json(run/'execution.json',result);node.destroy_node()
    return result
