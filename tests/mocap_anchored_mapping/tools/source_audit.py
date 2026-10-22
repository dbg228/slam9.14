#!/usr/bin/env python3
"""Read-only source inspection; never calculates or publishes an anchor/TF."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from controller_tracking.feedback import ns
from controller_tracking.world_capture import read_pose,check_raw_normalized
from mocap_anchored_mapping.core import load


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--site',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--native-topic',default='/anchored_mapping/native_odom');args=parser.parse_args()
    cfg=load(args.site);mount=load(cfg['mocap_config']);normal=mount['normalizer']
    rclpy.init(args=[]);node=rclpy.create_node('read_only_mapping_source_audit');frames=Counter();ages=[];raw=[];mc=[];edges=Counter()
    def mocap(msg,store):
        try:store.append(read_pose(msg.pose,ns(msg.header.stamp),node.get_clock().now().nanoseconds,msg.header.frame_id))
        except ValueError:frames['invalid_mocap']+=1
    def native(msg):
        frames[f'{msg.header.frame_id}->{msg.child_frame_id}']+=1
        ages.append((node.get_clock().now().nanoseconds-ns(msg.header.stamp))*1e-9)
    node.create_subscription(PoseStamped,normal['input_topic'],lambda m:mocap(m,raw),qos_profile_sensor_data)
    node.create_subscription(PoseStamped,normal['output_topic'],lambda m:mocap(m,mc),qos_profile_sensor_data)
    node.create_subscription(Odometry,args.native_topic,native,qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf_lio_raw',lambda m:edges.update((t.header.frame_id+'->'+t.child_frame_id) for t in m.transforms),50)
    until=time.monotonic()+3.2
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    result=dict(read_only=True,anchor_calculated=False,native_topic_inspected=args.native_topic,
        native_publishers=[dict(node=p.node_name,namespace=p.node_namespace,type=p.topic_type) for p in node.get_publishers_info_by_topic(args.native_topic)],
        native_header_frames=dict(frames),native_header_age_s_range=[min(ages),max(ages)] if ages else None,
        private_old_native_TF_edges=dict(edges),raw_mocap_samples=len(raw),normalized_samples=len(mc),
        mocap_frames=sorted(set(p['frame'] for p in raw+mc)),raw_normalized=check_raw_normalized(raw,mc,mount['mocap']['position_scale_to_m']),
        timestamp_source=mount['mocap']['timestamp_source'],camera_latency_s=cfg['camera_latency_s'],
        note='Publisher/header inventory is measured above. This project publishes NDT odometry separately on /static_localization/odometry; never substitute that map-corrected pose for the native source. Static availability is not exposure synchronization or real mapping validation.')
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
