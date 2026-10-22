"""Native-frame calibration, restart direction, held-out errors and preservation."""
from pathlib import Path
import shlex
import subprocess
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from mocap_anchored_mapping.core import (checked,estimate,inverse,load,matrix,mocap_base,native_base,prepare,
    projection_args,record,save,sha,initial_map_odom,validation_errors,static_window)
from mocap_anchored_mapping.products import inspect,project,read_pcd,native_cloud

PKG=Path(__file__).resolve().parents[1];SITE=PKG/'config/site.yaml'


def transform(xyz,rpy):return matrix(xyz,Rotation.from_euler('xyz',rpy).as_quat())


def wall_pcd(path):
    # Software fixture only: four walls in the native coordinate frame.
    points=[]
    for axis,vals,sides in [(0,np.arange(0,3,.02),[0.,2.]),(1,np.arange(0,2,.02),[0.,3.])]:
        for side in sides:
            for v in vals:
                for z in np.arange(0,.82,.02):points.append([v,side,z,17.] if axis==0 else [side,v,z,17.])
    data=np.asarray(points,dtype='<f4');n=len(data)
    header=f'# SOFTWARE FIXTURE\nVERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
    path.write_bytes(header.encode()+data.tobytes());return data


def fixture_calibration(run,h):
    save(run/'calibration.yaml',dict(T_mocap_map=record(h,'mocap_world','map'),T_map_mocap=record(inverse(h),'map','mocap_world'),
        inputs_sha256=sha(run/'inputs.lock.json'),map_origin_in_mocap_m=h[:3,3].tolist(),data_kind='software_fixture'),True)
    save(run/'calibration.lock.json',dict(sha256=sha(run/'calibration.yaml')))
    save(run/'raw/native_receipt.json',dict(run_id=run.name,frame='map=native_F0',transform_count=0,sha256=sha(run/'raw/native_F.pcd'),calibration_sha256=sha(run/'calibration.yaml')))


def pairs(h,b,n=32):return [dict(stamp_ns=10**12+i*100000000,T_M_B=(h@b).tolist(),T_F_B=b.tolist()) for i in range(n)]


def test_body_references_full_se3():
    h=transform([2,-1,.6],[.2,-.12,3.13]);b=transform([.5,.7,-.4],[-.1,.07,-3.12])
    bi=transform([.37,.28,.15],[.1,-.2,.82]);tb=transform([.02,-.01,-.18],[.04,.03,-.02])
    cfg={'M_from_mocap_world':dict(translation_m=[0,0,0],quaternion_xyzw=[0,0,0,1])};ext=dict(T_B_I=record(bi,'B','I'),T_Tracker_B=record(tb,'T','B'))
    np.testing.assert_allclose(native_base(b@bi,cfg,ext),b,atol=1e-12)
    np.testing.assert_allclose(mocap_base(h@b@inverse(tb),cfg,ext),h@b,atol=1e-12)
    np.testing.assert_allclose(h@inverse(h),np.eye(4),atol=1e-12)


def test_calibration_direction_and_origin():
    cfg=load(SITE);h=transform([1,2,-.4],[.07,.12,np.pi-.001]);b=transform([.3,.7,.1],[.02,-.03,.5])
    s=estimate(pairs(h,b),cfg)
    np.testing.assert_allclose(s['T_mocap_native']['matrix'],h,atol=1e-12)
    np.testing.assert_allclose(s['T_native_mocap']['matrix'],inverse(h),atol=1e-12)
    assert not s['ndt_used'] and s['frozen']


def test_angle_wrap_mean():
    cfg=load(SITE);p=pairs(np.eye(4),np.eye(4))
    for i,row in enumerate(p):row['T_M_B']=transform([0,0,0],[0,0,np.pi+(-1 if i%2 else 1)*.001]).tolist()
    assert estimate(p,cfg)['T_mocap_native']['matrix'][0][0]<-.999


def test_restart_and_heldout_no_refit():
    h=transform([1,-2,.8],[.04,-.03,.4]);g=transform([-.5,.6,.04],[.03,.02,-.8]);b=transform([.4,-.2,.1],[.01,-.01,.2])
    c=dict(T_map_mocap=record(inverse(h),'map','mocap_world'))
    np.testing.assert_allclose(initial_map_odom(c,h@g),g,atol=1e-12)
    # Independent mocap point has a known 3-4-5 cm bias. It must remain in the error.
    bias=transform([.03,-.04,0],[0,0,0]);p=pairs(h@bias@g,b)
    value=validation_errors(p,c,g)
    assert not value['fit_performed'] and not value['transform_updated']
    assert abs(value['distance_3D_m']['rmse']-.05)<1e-12
    np.testing.assert_allclose(value['samples'][0]['XYZ_error_m'],[-.03,.04,0],atol=1e-12)


@pytest.mark.parametrize('kind',['gap','backward','moving','few','short'])
def test_bad_windows_rejected(kind):
    cfg=load(SITE);p=pairs(np.eye(4),np.eye(4))
    if kind=='gap':
        for row in p[15:]:row['stamp_ns']+=10**9
    elif kind=='backward':p[15]['stamp_ns']=p[14]['stamp_ns']
    elif kind=='moving':p[15]['T_M_B'][0][3]=.1
    elif kind=='few':p=p[:10]
    else:
        for i,row in enumerate(p):row['stamp_ns']=10**12+i*10000000
    with pytest.raises(ValueError):static_window(p,cfg)


def test_prepare_immutable_and_algorithm_unchanged(tmp_path):
    run=tmp_path/'fixture';prepare(run,SITE);cfg,_=checked(run)
    old=load(cfg['fastlio_config'])['/**']['ros__parameters'];new=load(run/'fastlio.yaml')['/**']['ros__parameters']
    assert [k for k in old if old[k]!=new[k]]==['map_file_path']
    assert not (run/'projection.yaml').exists() and not (run/'pcd').exists()
    with pytest.raises(ValueError):prepare(run,SITE)
    (run/'extrinsics.yaml').write_text('modified')
    with pytest.raises(ValueError,match='Frozen input'):checked(run)


def test_old_schema_rejected(tmp_path):
    import yaml
    cfg=load(SITE);cfg['schema']=1;site=tmp_path/'site.yaml';site.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='schema 2'):prepare(tmp_path/'run',site)


def test_native_pcd_unchanged_and_original_projection(tmp_path):
    run=tmp_path/'fixture';prepare(run,SITE);cfg,_=checked(run)
    before=wall_pcd(run/'raw/native_F.pcd');fixture_calibration(run,transform([5,-3,.8],[.04,-.03,.35]))
    digest=sha(run/'raw/native_F.pcd');_,data=read_pcd(run/'raw/native_F.pcd',cfg)
    np.testing.assert_array_equal(data['intensity'],before[:,3])
    assert inspect(run)['frame']=='map=native_F0'
    args=projection_args(cfg,run/'raw/native_F.pcd',run/'grid/map')
    assert args[args.index('--z-min')+1]=='0.05' and args[args.index('--z-max')+1]=='0.35'
    assert '--height-layers=-0.15:0.05,0.05:0.35,0.35:0.60' in args
    result=project(run);assert result['PCD_XYZ_transform_count']==0 and not result['slice_heights_changed_for_mocap']
    assert sha(run/'raw/native_F.pcd')==digest and result['T_map_F0']==np.eye(4).tolist()
    with pytest.raises(ValueError):project(run)
    (run/'raw/native_F.pcd').write_bytes(b'changed')
    with pytest.raises(ValueError):native_cloud(run)


@pytest.mark.parametrize('mode',['mapping','localization'])
def test_native_auxiliary_isolation(tmp_path,mode):
    from mocap_anchored_mapping.isolation import native_prefix
    native=tmp_path/'native';(native/'src').mkdir(parents=True);(native/'Log').mkdir();(native/'PCD').mkdir()
    (native/'Log/mat_pre.txt').write_text('old log');(native/'PCD/scans.pcd').write_text('old map')
    run=tmp_path/'run';run.mkdir();prefix=native_prefix(run,mode,dict(fastlio_source=str(native/'src/laserMapping.cpp')))
    command='from pathlib import Path;import sys;Path(sys.argv[1]).write_text("new log");Path(sys.argv[2]).write_text("new map")'
    subprocess.run([*shlex.split(prefix),sys.executable,'-c',command,str(native/'Log/mat_pre.txt'),str(native/'PCD/scans.pcd')],check=True)
    assert (native/'Log/mat_pre.txt').read_text()=='old log' and (native/'PCD/scans.pcd').read_text()=='old map'
    assert len(list(run.rglob('mat_pre.txt')))==1 and len(list(run.rglob('scans.pcd')))==1


def test_synthetic_map_domain_guard(monkeypatch):
    from mocap_anchored_mapping.cli import require_fixture_domain
    monkeypatch.delenv('ROS_DOMAIN_ID',raising=False);monkeypatch.delenv('ROS_LOCALHOST_ONLY',raising=False)
    with pytest.raises(ValueError):require_fixture_domain(dict(data_kind='software_fixture'))
    monkeypatch.setenv('ROS_DOMAIN_ID','208');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1');require_fixture_domain(dict(data_kind='software_fixture'))
