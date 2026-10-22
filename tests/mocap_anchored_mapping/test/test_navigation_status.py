from pathlib import Path
from mocap_anchored_mapping.launching import navigation_status_ready,ordered_navigation_parameters


def test_retained_stale_status_requires_a_new_fresh_update():
    run=Path('/saved/map')
    state=dict(run_path=str(run),frozen=True,fault=None,native_stamp_ns=1_000_000_000)
    assert not navigation_status_ready(state,run,2_000_000_000,.25)
    state['native_stamp_ns']=1_900_000_000
    assert navigation_status_ready(state,run,2_000_000_000,.25)


def test_fresh_status_does_not_allow_wrong_map_fault_or_correction():
    run=Path('/saved/map')
    state=dict(run_path=str(run),frozen=True,fault=None,native_stamp_ns=1_900_000_000)
    for changed in [dict(run_path='/other/map'),dict(frozen=False),dict(fault='bad_pose'),
                    dict(active_checkpoint='middle'),dict(native_stamp_ns=2_100_000_000)]:
        assert not navigation_status_ready({**state,**changed},run,2_000_000_000,.25)


def test_ordered_checker_layer_preserves_original_navigation_settings():
    source={'controller_server':{'ros__parameters':{
        'goal_checker_plugins':['goal_checker'],
        'goal_checker':{'plugin':'nav2_controller::SimpleGoalChecker','xy_goal_tolerance':.2},
        'InspectionStrict':{'plugin':'nav2_mppi_controller::MPPIController','vx_max':.2}}}}
    out=ordered_navigation_parameters(source)
    original=source['controller_server']['ros__parameters'];changed=out['controller_server']['ros__parameters']
    assert original['goal_checker_plugins']==['goal_checker']
    assert original['goal_checker']==changed['goal_checker']
    assert original['InspectionStrict']==changed['InspectionStrict']
    assert changed['inspection_route_goal_checker']['xy_goal_tolerance']==.2
    assert changed['inspection_route_goal_checker']['plugin']=='inspection_geometry_tools::OrderedRouteGoalChecker'
    assert ordered_navigation_parameters(out)==out
