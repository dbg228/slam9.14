"""DDS endpoint identity discovery must neither accept unknown owners nor false-fail early."""
from types import SimpleNamespace
import pytest

pytest.importorskip('rclpy')
from mocap_anchored_mapping import online


def owner(name, namespace='/'):
    return SimpleNamespace(node_name=name, node_namespace=namespace)


def bridge(native=None, dynamic=None, static=None):
    topics={'/anchored_localization/native_odom':native if native is not None else [owner('anchored_fastlio')],
            '/tf':dynamic if dynamic is not None else [owner('native_base_bridge'),owner('robot_state_publisher')],
            '/tf_static':static if static is not None else [owner('native_base_bridge'),owner('robot_state_publisher')]}
    state=SimpleNamespace(fault=None,done=False,graph_verified=False,start=100.,phase='localization',prefix='/anchored_localization')
    state.get_publishers_info_by_topic=topics.__getitem__
    state.fail=lambda reason:setattr(state,'fault',reason)
    return state,topics


def test_unknown_discovery_defers_then_requires_resolved_owner(monkeypatch):
    state,topics=bridge(native=[owner('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')])
    monkeypatch.setattr(online.time,'monotonic',lambda:101.)
    online.NativeBridge.audit_publishers(state)
    assert state.fault is None and not state.graph_verified
    topics['/anchored_localization/native_odom']=[owner('anchored_fastlio')]
    online.NativeBridge.audit_publishers(state)
    assert state.graph_verified and state.fault is None


def test_unknown_cannot_survive_discovery_window(monkeypatch):
    state,_=bridge(dynamic=[owner('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')])
    monkeypatch.setattr(online.time,'monotonic',lambda:103.1)
    online.NativeBridge.audit_publishers(state)
    assert state.fault=='publisher_identity_unresolved_after_discovery' and not state.graph_verified


@pytest.mark.parametrize('native,dynamic,reason',[
    ([owner('foreign_lio')],None,'competing_or_wrong_native_publisher'),
    ([owner('anchored_fastlio'),owner('anchored_fastlio')],None,'competing_or_wrong_native_publisher'),
    (None,[owner('old_localizer')],'foreign_global_TF_publisher'),
    (None,[owner('native_base_bridge'),owner('native_base_bridge')],'duplicate_TF_publisher_name')])
def test_real_competitors_rejected_even_during_discovery(monkeypatch,native,dynamic,reason):
    state,_=bridge(native=native,dynamic=dynamic)
    monkeypatch.setattr(online.time,'monotonic',lambda:101.)
    online.NativeBridge.audit_publishers(state)
    assert state.fault.startswith(reason) and not state.graph_verified


def test_no_pairs_processed_before_graph_verified(monkeypatch):
    state,_=bridge()
    state.h=None;state.live_ok=True;state.cfg={'initialization':{'timeout_s':45}}
    state.collecting=lambda:True
    monkeypatch.setattr(online.time,'monotonic',lambda:101.)
    # No command/pair fields: touching them before verification would fail this test.
    online.NativeBridge.tick(state)
