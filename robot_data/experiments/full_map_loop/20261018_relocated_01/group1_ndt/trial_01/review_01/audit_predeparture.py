"""Read-only audit of fixed, time-selected windows in the closed original bag."""
import sys,json,copy,bisect
from pathlib import Path
from collections import Counter
import numpy as np
ROOT=Path('/home/zkam/test_nav');sys.path.insert(0,str(ROOT/'tests/full_map_loop'))
from arrival_capture import messages,command_values,capture,wc
from ndt_endpoint_report import score_pairs
p=Path(__file__).resolve().parent.parent;cal=p/'registration_retry_01';bag=p/'formal_bag_01'
inputs=wc.frozen_inputs(cal);cfg=inputs['config'];h=wc.check_calibration(cal)['map_from_mocap']['homogeneous_matrix']
interval=wc.load(p/'endpoint_arrival_capture_02/motion_interval.json');stop=interval['zero_command_ns'];start=interval['start_ns']
ready=sorted((r,bool(m.data)) for r,m in messages(bag,cfg['topics']['map_ready']));ready_stamps=[x[0] for x in ready]
def audit(label,begin,end):
 local=copy.deepcopy(cfg);local['static_history_pairing']=True;local['limits']['seconds']=(end-begin)*1e-9
 raw=[];norm=[];status={};invalid=Counter()
 for topic,key in [(cfg['normalizer']['input_topic'],'raw'),(cfg['normalizer']['output_topic'],'norm'),(cfg['normalizer']['diagnostics_topic'],'status')]:
  for received,m in messages(bag,topic,begin-2*10**9,end+2*10**9):
   stamp=capture.stamp_ns(m.header.stamp)
   if not begin<=stamp<=end:continue
   if key=='status':
    relevant=[s for s in m.status if s.name=='mocap_tracker0_normalizer']
    valid=bool(relevant) and all(s.level in (0,b'\x00') and {v.key:v.value for v in s.values}.get('valid')=='true' for s in relevant)
    status[stamp]=valid
    if not valid:invalid['invalid_normalizer']+=1
   else:(raw if key=='raw' else norm).append(capture.read_pose(m.pose,stamp,received,m.header.frame_id))
 norm.sort(key=lambda r:r['stamp_ns']);valid=[]
 for r in norm:
  if status.get(r['stamp_ns']) is True:valid.append(r)
  else:invalid['missing_or_invalid_status']+=1
 ndt=[]
 for received,m in messages(bag,cfg['topics']['ndt'],begin,end+30*10**9):
  stamp=capture.stamp_ns(m.cloud_stamp)
  if not begin<=stamp<=end:continue
  index=bisect.bisect_right(ready_stamps,received)-1
  pose=capture.read_pose(getattr(m,cfg['ndt_pose_field']),stamp,received,m.header.frame_id)
  ndt.append(dict(stamp_ns=stamp,receive_ros_ns=received,frame=m.header.frame_id,valid=bool(getattr(m,cfg['ndt_pose_field']+'_valid')),sync_valid=m.cloud_to_odom_sync_valid,cloud_odom_skew_s=m.cloud_to_odom_skew_sec,pose=pose,match_id=int(m.match_id),accepted=bool(m.accepted),stage=m.stage,map_ready=ready[index][1] if index>=0 else False))
 commands=[command_values(m) for _,m in messages(bag,cfg['topics']['command'],begin,end)]
 result=capture.summarize(local,inputs['body_extrinsic']['rigid_body_to_base_link'],valid,ndt,commands,sum(invalid.values()),evaluation_only=True)
 units=capture.check_raw_normalized(raw,valid,cfg['mocap']['position_scale_to_m'])
 if units['coverage']<cfg['limits']['minimum_raw_normalized_coverage'] or units['mismatched_stamp_ns']:result['quality_passed']=False;result['failures'].append('raw_units_or_coverage')
 result.update(label=label,source_window_ns=[begin,end],raw_normalized_check=units,invalid_events=dict(invalid),transform_refitted=False,source='closed formal_bag_01; original source times; supplemental audit, not replacement trial')
 if result['matched_pairs']:
  result['score']=score_pairs(result['matched_pairs'],h)
  e=np.array([[r['ex_m'],r['ey_m']] for r in result['score']['rows']]);result['mean_point_error_mm']=float(np.linalg.norm(e.mean(axis=0))*1000)
 return result
# Declare all intervals before reading/scoring. No residual-based selection.
windows=[('predeparture_10s_excluding_first_moving_command',start-10*10**9,start-1)]
results=[]
for label,begin,end in windows:
 r=audit(label,begin,end);results.append(r)
 print(label,json.dumps(dict(quality=r['quality_passed'],failures=r['failures'],pairs=r['pair_count'],rmse_mm=r.get('score',{}).get('xy_m',{}).get('rmse',0)*1000,mean_point_mm=r.get('mean_point_error_mm'),raw_coverage=r['raw_normalized_check']['coverage'])),flush=True)
wc.save(p/'review_01/predeparture_window_audit.json',dict(policy='All intervals above declared before scoring; no change to calibration or original trial, no hardware interaction',results=results))
