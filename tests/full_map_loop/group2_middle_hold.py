#!/usr/bin/env python3
"""One brief hold via original Nav2 parameter services, then original validation."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import os
import signal

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    data=parser.parse_args().trial.resolve()/'data'
    meta=json.loads((data/'manifest.json').read_text());run=Path(meta['run'])
    rclpy.init();node=rclpy.create_node('group2_middle_hold')
    get=node.create_client(GetParameters,'/velocity_smoother/get_parameters')
    setp=node.create_client(SetParametersAtomically,'/velocity_smoother/set_parameters_atomically')
    cancel=node.create_client(Trigger,'/inspection_route/cancel')
    report={'pose_transform_changed':False,'nav_goal_replaced':False};held=False
    def call(client,req):
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('missing:'+client.srv_name)
        f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=3)
        if not f.done() or f.exception():raise RuntimeError('failed:'+client.srv_name)
        return f.result()
    def limits(values):
        result=call(setp,SetParametersAtomically.Request(parameters=[Parameter(k,value=values[k]).to_parameter_msg() for k in ('max_velocity','min_velocity')])).result
        if not result.successful:raise RuntimeError(result.reason)
    def save(): (data/'middle_static_capture.json').write_text(json.dumps(report,indent=2)+'\n')
    try:
        values=call(get,GetParameters.Request(names=['max_velocity','min_velocity'])).values
        original={k:list(v.double_array_value) for k,v in zip(('max_velocity','min_velocity'),values)}
        if any(len(v)!=3 for v in original.values()):raise RuntimeError('invalid_original_limits')
        report['original_limits']=original
        deadline=time.monotonic()+700
        while not (data/'middle_pause_request.json').exists():
            if time.monotonic()>deadline:report['outcome']='middle_missing';save();return
            if (data/'navigation_terminal.json').exists():report['outcome']='middle_missing';save();return
            rclpy.spin_once(node,timeout_sec=.05)
        request=json.loads((data/'middle_pause_request.json').read_text())
        if (node.get_clock().now().nanoseconds-request['pose_ns'])*1e-9>.25:raise RuntimeError('stale_middle_request')
        report.update(hold_ros_ns=node.get_clock().now().nanoseconds,request=request)
        limits({'max_velocity':[0.,0.,0.],'min_velocity':[0.,0.,0.]});held=True;save()
        settle=time.monotonic()+1
        while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.05)
        before=set((run/'validation').glob('*'))
        child=subprocess.Popen([str(Path(__file__).resolve().parents[2]/'mapping_modes.sh'),'anchored','validate','--run',str(run),'--context','localization','--session',meta['localization_session']],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        deadline=time.monotonic()+20
        while child.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        timed_out=child.poll() is None
        if timed_out:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        captures=sorted(set((run/'validation').glob('*'))-before)
        report.update(exit_code=child.returncode,captures=[str(p.resolve()) for p in captures])
        docs=[json.loads((p/'summary.json').read_text()) for p in captures if (p/'summary.json').exists()]
        unavailable='static_validation_timeout' if timed_out else 'static_validation_failed' if child.returncode!=0 or not docs else 'near_inverted_reference' if docs[0]['orientation_error_deg']['rmse']>90 else None
        limits(original);held=False
        report.update(outcome='middle_static_unavailable_limits_restored' if unavailable else 'static_measurement_saved_limits_restored',reason=unavailable,resume_ros_ns=node.get_clock().now().nanoseconds);save()
    except BaseException as e:
        report.update(outcome='failed',reason=str(e))
        if held:
            try:report['cancel_response']=call(cancel,Trigger.Request()).message
            except Exception as ce:report['cancel_error']=str(ce)
            report['zero_limits_retained_until_inspection']=True
        save();raise
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
