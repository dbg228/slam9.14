"""Mount consistency and actual bridge geometry; never starts a ROS node."""
import ast
import math
from pathlib import Path
import re
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest


ROOT=Path(__file__).resolve().parents[3]
MOUNT={'sensor_x':.37,'sensor_y':.28,'sensor_z':.15,
       'sensor_yaw':math.radians(47.15)}


@pytest.mark.parametrize('name',['lio_tf_to_base_tf.py','map_to_odom_bridge.py'])
def test_python_defaults(name):
    values={}
    for call in ast.walk(ast.parse((ROOT/name).read_text())):
        if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
                and call.func.attr=='declare_parameter' and len(call.args)>=2
                and isinstance(call.args[0],ast.Constant) and call.args[0].value in MOUNT):
            values[call.args[0].value]=ast.literal_eval(call.args[1])
    assert values==pytest.approx(MOUNT)


@pytest.mark.parametrize('name',['localization.sh','fastlio_baseline.sh','fastlio_mocap_baseline.sh'])
def test_launch_overrides(name):
    text=(ROOT/name).read_text()
    for key,value in MOUNT.items():
        matches=re.findall(r'-p '+key+r':=([0-9.+-]+)',text)
        assert len(matches)==1
        assert float(matches[0])==pytest.approx(value)


def test_ndt_cpp_defaults():
    text=(ROOT/'src/static_map_localization/src/static_map_localizer_node.cpp').read_text()
    for key,value in MOUNT.items():
        matches=re.findall(r'declare_parameter<double>\("'+key+r'", ([0-9.+-]+)\)',text)
        assert len(matches)==1
        assert float(matches[0])==pytest.approx(value)


@pytest.mark.parametrize('path',[
    ROOT/'robot_data/urdf/robot.urdf',
    Path('/home/zkam/OmniBot_ws/src/omnibot_control/urdf/robot.urdf.xacro'),
])
def test_description_matches_bridges(path):
    if not path.exists():pytest.skip('Optional external hardware workspace is absent')
    joint=ET.parse(path).getroot().find("joint[@name='imu_joint']")
    assert joint.find('parent').get('link')=='base_link'
    origin=joint.find('origin')
    assert [float(v) for v in origin.get('xyz').split()]==pytest.approx([.37,.28,.15])
    assert [float(v) for v in origin.get('rpy').split()]==pytest.approx([0.,0.,math.radians(47.15)])


def sensor_pose(base_x,base_y,base_yaw):
    c,s=math.cos(base_yaw),math.sin(base_yaw)
    return (base_x+c*.37-s*.28,base_y+s*.37+c*.28,base_yaw+MOUNT['sensor_yaw'])


@pytest.mark.parametrize('angle',[0.,-math.radians(45.),3.,-3.])
def test_local_bridge_recovers_base_and_preserves_measurement_stamp(angle):
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from lio_tf_to_base_tf import LioTfToBaseTf,yaw_from_quat
    x,y,a=sensor_pose(1.2,-.8,angle)
    raw=TransformStamped()
    raw.header.frame_id='camera_init';raw.child_frame_id='body'
    raw.header.stamp.sec=123;raw.header.stamp.nanosec=456
    raw.transform.translation.x=x;raw.transform.translation.y=y;raw.transform.translation.z=.45
    raw.transform.rotation.z=math.sin(a/2);raw.transform.rotation.w=math.cos(a/2)
    result=[]
    fake=SimpleNamespace(**MOUNT,raw_parent_frame='camera_init',raw_child_frame='body',
                         odom_frame='odom',base_frame='base_link',count=0,
                         tf_broadcaster=SimpleNamespace(sendTransform=result.append),
                         get_logger=lambda:SimpleNamespace(info=lambda _:None))
    LioTfToBaseTf.on_tf(fake,TFMessage(transforms=[raw]))
    assert len(result)==1
    out=result[0]
    assert out.header.stamp==raw.header.stamp
    assert (out.header.frame_id,out.child_frame_id)==('odom','base_link')
    t=out.transform.translation
    assert (t.x,t.y,t.z)==pytest.approx((1.2,-.8,.30))
    assert yaw_from_quat(out.transform.rotation)==pytest.approx(angle)


def test_global_bridge_applies_mount_inverse_before_map_to_odom():
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from map_to_odom_bridge import MapToOdomBridge
    # Known map->odom=(1,-2,.4), odom->base=(.2,-.3,-.8).
    c,s=math.cos(.4),math.sin(.4)
    bx,by,ba=1+c*.2-s*(-.3),-2+s*.2+c*(-.3),-.4
    x,y,a=sensor_pose(bx,by,ba)
    global_pose=Odometry()
    global_pose.pose.pose.position.x=x;global_pose.pose.pose.position.y=y
    global_pose.pose.pose.orientation.z=math.sin(a/2);global_pose.pose.pose.orientation.w=math.cos(a/2)
    odom_base=TransformStamped()
    odom_base.transform.translation.x=.2;odom_base.transform.translation.y=-.3
    odom_base.transform.rotation.z=math.sin(-.8/2);odom_base.transform.rotation.w=math.cos(-.8/2)
    result=[]
    fake=SimpleNamespace(**MOUNT,latest_global_odom=global_pose,odom_frame='odom',base_frame='base_link',
                         lookup_timeout=.05,max_global_xy_norm=100.,
                         tf_buffer=SimpleNamespace(lookup_transform=lambda *a,**kw:odom_base),
                         publish_transform=lambda *v:result.append(v))
    MapToOdomBridge.publish_map_to_odom(fake)
    assert len(result)==1
    assert result[0]==pytest.approx((1.,-2.,.4))
