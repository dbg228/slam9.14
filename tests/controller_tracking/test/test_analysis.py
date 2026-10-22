import copy
import json
from pathlib import Path
import numpy as np
from controller_tracking.analysis import analyze_records,compare
from controller_tracking.configuration import load
from controller_tracking.geometry import geometry

ROOT=Path(__file__).resolve().parents[3]


def make_pose(t,x,y):
    sec=int(t);return dict(header=dict(frame_id='tracking_world',stamp=dict(sec=sec,nanosec=round((t-sec)*1e9))),
        pose=dict(position=dict(x=x,y=y,z=0.),orientation=dict(x=0.,y=0.,z=0.,w=1.)))


def test_missing_reference_and_failed_run_are_retained(tmp_path):
    cfg=load(ROOT/'tests/controller_tracking/config/experiment.yaml');path=geometry(cfg,'line')
    records=[dict(topic='/tracking/events',receive_s=10.,msg=dict(data='{}')),
             dict(topic='/tracking/events',receive_s=11.,msg=dict(data='{}'))]
    failed=dict(success=False,reason='mocap_stale',start_ros_ns=10*10**9,end_ros_ns=11*10**9)
    summary=analyze_records(records,path,cfg,failed,tmp_path/'analysis',source='software_fixture')
    assert summary['reference_data_coverage']==0
    assert summary['lateral_m']['rmse'] is None
    assert summary['compute_ms']['rmse'] is None
    assert summary['wheel']['physical_rate_error'] is None
    assert not list(tmp_path.rglob('*.csv'))
    result=compare([tmp_path],tmp_path/'comparison')
    assert result['total']==1 and result['success_rate']==0


def test_gap_and_constant_offset_metrics(tmp_path):
    cfg=load(ROOT/'tests/controller_tracking/config/experiment.yaml');path=geometry(cfg,'line');records=[]
    for t in np.arange(10,11.001,.01):
        if 10.30<t<10.70:continue
        pose=make_pose(float(t),float(t-10),.025)
        records.append(dict(topic=cfg['topics']['base_pose'],receive_s=float(t),msg=pose))
    execution=dict(success=True,reason='software_success',start_ros_ns=10*10**9,end_ros_ns=11*10**9)
    summary=analyze_records(records,path,cfg,execution,tmp_path/'analysis',source='software_fixture')
    assert .5<summary['reference_data_coverage']<.7
    assert abs(summary['lateral_m']['rmse']-.025)<1e-12
    assert summary['sampling']['base_pose']['long_gap_count']==1
    assert summary['speed_mps']['rmse'] is None


def test_byte_diagnostic_status_matches_integer_stamp():
    from controller_tracking.feedback import PairedMocap
    from geometry_msgs.msg import PoseStamped
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    class Clock:
        def now(self):return type('T',(),dict(nanoseconds=10**9))()
    class Node:
        def create_subscription(self,*a):pass
        def create_timer(self,*a):pass
        def get_clock(self):return Clock()
    valid=[];bad=[];cfg={'timing':dict(max_age_s=.15,validity_wait_s=.04),'topics':dict(mocap='m',validity='v')}
    paired=PairedMocap(Node(),cfg,valid.append,bad.append)
    pose=PoseStamped();pose.header.stamp.sec=1
    diagnostic=DiagnosticArray();diagnostic.header=pose.header
    diagnostic.status=[DiagnosticStatus(name='mocap_tracker0_normalizer',level=DiagnosticStatus.OK,values=[KeyValue(key='valid',value='true')])]
    paired.pose(pose);paired.diagnostic(diagnostic)
    assert valid==[pose] and bad==[]
    paired.pose(pose)
    assert bad==['duplicate_or_backward_pose']


def test_batch_cannot_silently_skip_failed_attempts(tmp_path):
    import pytest
    from controller_tracking.analysis import batch_runs
    batch=tmp_path/'batch';batch.mkdir()
    for number in (1,2):
        round=tmp_path/f'round_{number}';round.mkdir()
        (round/'recording.json').write_text(json.dumps(dict(session=str(batch),path_kind='line')))
        (round/'recording_result.json').write_text('{}')
        (round/'analysis').mkdir()
    (tmp_path/'round_1/analysis/summary.json').write_text('{}')
    with pytest.raises(ValueError,match='every failed/successful'):
        batch_runs(batch,'line')
    (tmp_path/'round_2/analysis/summary.json').write_text('{}')
    assert batch_runs(batch,'line')==[tmp_path/'round_1',tmp_path/'round_2']
