"""Geometry and file export checks with explicitly synthetic trajectories."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

SPEC=importlib.util.spec_from_file_location('tracking_report',Path(__file__).with_name('tracking_report.py'))
report=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(report)


def test_segment_projection_preserves_bias_and_ignores_duplicate_vertices():
    rows,length=report.project([[.5,.2,.1],[2.2,-.3,-.2]],[[0,0,0],[0,0,0],[1,0,0],[2,0,0]])
    assert length==2
    assert rows[0]['path_distance_m']==pytest.approx(.2)
    assert rows[0]['lateral_m']==pytest.approx(.2)
    assert rows[1]['path_distance_m']==pytest.approx(math.hypot(.2,.3))
    assert rows[1]['lateral_m']==pytest.approx(-.3)
    assert rows[1]['reference_s_m']==2


def test_heading_wrap_and_invalid_reference():
    rows,_=report.project([[.5,0,math.radians(-179)]],[[0,0,math.radians(179)],[1,0,math.radians(-179)]])
    assert rows[0]['heading_error_deg']==pytest.approx(1)
    with pytest.raises(ValueError,match='nonzero'):
        report.project([[0,0,0]],[[1,1,0],[1,1,0]])


def test_export_includes_trajectory_errors_csv_and_real_gaps(tmp_path):
    make=lambda t,x,y:dict(stamp_ns=t,x_m=x,y_m=y,yaw_rad=0.)
    data=dict(reference=[make(0,0,0),make(0,2,0)],inspection=None,
        actual=[make(10**9,0,.1),make(11*10**8,1,.1),make(3*10**9,2,.1)],
        execution_index=0,recorded_executions=1)
    output=tmp_path/'software_fixture_report'
    result=report.export(data,output)
    assert result['tracking_rmse_m']==pytest.approx(.1)
    import json
    summary=json.loads((output/'summary.json').read_text())
    assert not summary['transform_fitted'] and len(summary['gaps'])==1
    assert len((output/'tracking_samples.csv').read_text().splitlines())==4
    for name in ('trajectory_tracking','tracking_errors','overall_tracking_error'):
        for ext in ('png','svg','pdf'):assert (output/f'{name}.{ext}').stat().st_size>1000


def test_sqlite_paths_do_not_merge_separate_executions(tmp_path):
    serialization=pytest.importorskip('rclpy.serialization')
    nav=pytest.importorskip('nav_msgs.msg');geometry=pytest.importorskip('geometry_msgs.msg')
    import sqlite3
    def path(stamp,offset):
        msg=nav.Path();msg.header.frame_id='map';msg.header.stamp.sec=stamp
        for i in range(2):
            p=geometry.PoseStamped();p.header.frame_id='map';p.header.stamp.sec=stamp+i
            p.pose.position.x=float(i);p.pose.position.y=float(offset);p.pose.orientation.w=1.
            msg.poses.append(p)
        return serialization.serialize_message(msg)
    with sqlite3.connect(tmp_path/'fixture.db3') as db:
        db.execute('CREATE TABLE topics (id INTEGER,name TEXT)');db.execute('CREATE TABLE messages (topic_id INTEGER,timestamp INTEGER,data BLOB)')
        db.executemany('INSERT INTO topics VALUES (?,?)',[(1,'/inspection_route/reference_path'),(2,'/inspection_route/actual_path')])
        db.executemany('INSERT INTO messages VALUES (?,?,?)',[(1,10,path(1,0)),(2,20,path(2,.1)),(1,30,path(3,0)),(2,40,path(4,.2))])
    with pytest.raises(ValueError,match='executions found'):report.read_paths([tmp_path])
    first=report.read_paths([tmp_path],0);second=report.read_paths([tmp_path],1)
    assert first['actual'][0]['y_m']==pytest.approx(.1)
    assert second['actual'][0]['y_m']==pytest.approx(.2)
