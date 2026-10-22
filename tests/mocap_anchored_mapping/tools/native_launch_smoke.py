#!/usr/bin/env python3
"""Real binary startup in a sensor-free isolated ROS domain; no real map claim."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from mocap_anchored_mapping.core import checked,load,prepare,save,sha


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 180<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=220:raise ValueError('Requires isolated localhost test domain 180..220')
    mode=sys.argv[1];out=Path(sys.argv[2]).resolve();out.mkdir(parents=True,exist_ok=False)
    if mode=='mapping':
        pkg=Path(__file__).resolve().parents[1];site=load(pkg/'config/site.yaml');site['data_kind']='software_fixture_no_sensor_input'
        save(out/'site.yaml',site,True);run=out/'software_fixture';prepare(run,out/'site.yaml')
    else:run=Path(sys.argv[3]).resolve()
    cfg,_=checked(run,mode!='mapping');source_root=Path(cfg['fastlio_source']).resolve().parents[1]
    old={str(p):p.read_bytes() for sub in ['Log','PCD'] for p in (source_root/sub).rglob('*') if p.is_file()}
    old_pcd=sha(run/'raw/native_F.pcd') if mode!='mapping' else None
    rclpy.init(args=[]);node=rclpy.create_node('isolated_native_launch_probe');maps=[]
    node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log=(out/'launch.log').open('x');p=subprocess.Popen(['ros2','launch','mocap_anchored_mapping',f'anchored_{mode}.launch.py',f'run:={run}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        expected={'anchored_fastlio','initial_anchor'} if mode=='mapping' else {'anchored_fastlio','native_base_bridge','anchored_map_server','anchored_map_lifecycle','tf_to_odom_bridge'}
        end=time.monotonic()+12;names=set()
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05);names={n for n,_ in node.get_node_names_and_namespaces()}
            if expected.issubset(names) and (mode=='mapping' or maps):break
            if p.poll() is not None:raise RuntimeError('Launch exited early; see launch.log')
        assert expected.issubset(names),str(names)
        assert not any('ndt' in n.lower() or n=='static_map_localizer' for n in names)
        assert not node.get_publishers_info_by_topic('/livox/lidar') and not node.get_publishers_info_by_topic('/livox/imu')
        client=node.create_client(GetParameters,'/anchored_fastlio/get_parameters');assert client.wait_for_service(timeout_sec=3)
        f=client.call_async(GetParameters.Request(names=['map_file_path','pcd_save.pcd_save_en']));rclpy.spin_until_future_complete(node,f,timeout_sec=3)
        values=f.result().values;assert values[0].string_value==str(run/'raw/native_F.pcd') and values[1].bool_value==(mode=='mapping')
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        assert p.poll() is None
        for name,payload in old.items():assert Path(name).read_bytes().startswith(payload),'Original native auxiliary file changed:'+name
        private_logs=list((run/'native_aux/Log').glob('mat_pre.txt')) if mode=='mapping' else list((run/'localizations').glob('*/native_aux/Log/mat_pre.txt'))
        assert private_logs
        if mode=='mapping':assert not (run/'calibration.yaml').exists() and not (run/'raw/native_F.pcd').exists()
        else:
            assert maps and maps[-1].header.frame_id=='map' and maps[-1].info.width>0
            assert sha(run/'raw/native_F.pcd')==old_pcd
            with (out/'navigation_rejection.log').open('x') as stream:
                nav=subprocess.run(['ros2','launch','mocap_anchored_mapping','anchored_navigation.launch.py',f'run:={run}'],stdout=stream,stderr=subprocess.STDOUT,timeout=10)
            assert nav.returncode!=0
            assert 'frozen mocap initialization' in (out/'navigation_rejection.log').read_text()
        save(out/'result.json',dict(passed=True,schema=2,mode=mode,domain=os.environ['ROS_DOMAIN_ID'],
            data_kind='software_fixture_no_sensor_input',nodes=sorted(names),NDT_nodes_present=False,
            PCD_native_and_unchanged=True,private_native_logs_verified=True,no_false_initialization_without_sources=True,
            native_grid_published=(mode=='localization'),uninitialized_navigation_rejected=(mode=='localization'),run=str(run)))
        print(json.dumps(dict(passed=True,mode=mode,directory=str(out))))
    finally:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=4)
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
