"""Semantic checks for paper analysis: timing, integration and replicate units."""
import json

import numpy as np
import pytest

from . import paper_metrics as metrics


def test_integrals_omit_unobserved_initial_state_and_use_trapezoids():
    t = np.array([.05, .10, .15])
    e = np.array([1., 2., 3.])
    value, duration, n = metrics.integrate_observed(t, e, np.ones(3, bool), .05)
    assert value == pytest.approx(.2)
    assert duration == pytest.approx(.1)
    assert n == 2
    itae, _, _ = metrics.integrate_observed(t, t*e, np.ones(3, bool), .05)
    assert itae == pytest.approx(.0225)


def test_integrals_never_bridge_phase_segments_or_missing_sample_gaps():
    t = np.arange(1, 8)*.05
    mask = np.array([True, True, False, True, False, True, True])
    value, duration, n = metrics.integrate_observed(t, np.ones(7), mask, .05)
    assert value == pytest.approx(.1)
    assert duration == pytest.approx(.1)
    assert n == 2
    value, duration, n = metrics.integrate_observed(np.array([.05,.10,.30,.35]), np.ones(4), np.ones(4,bool), .05)
    assert value == pytest.approx(.1)
    assert duration == pytest.approx(.1)
    assert n == 2


def test_empty_phase_integral_is_undefined_not_zero():
    value, duration, n = metrics.integrate_observed(np.array([.05,.10]), np.ones(2), np.zeros(2,bool), .05)
    assert value is None and duration == 0 and n == 0
    assert metrics.moments([])["rmse"] is None
    assert metrics.moments([1.])["sd"] is None


def test_distance_weighting_uses_real_displacement_not_time_samples():
    t = np.arange(4)*.05
    xy = np.c_[np.array([0., 0., 1., 3.]), np.zeros(4)]
    e = np.array([0., 1., 2., 2.])
    rms, mae, distance = metrics.distance_moments(e, xy, t, np.ones(4,bool), .05)
    assert distance == pytest.approx(3.)
    assert rms == pytest.approx(np.sqrt(10.5/3))
    assert mae == pytest.approx(5.5/3)
    assert metrics.distance_moments(e, xy*0, t, np.ones(4,bool), .05)[:2] == (None, None)


def test_derivative_midpoint_clock_is_correct_for_unequal_steps():
    t = np.array([0., .04, .10, .15])
    first, first_t, second, second_t = metrics.derivative_series(t*t, t, .05)
    assert np.isnan(first[0]) and np.isnan(second[:2]).all()
    assert first[1:] == pytest.approx(2*first_t[1:])
    assert second[2:] == pytest.approx([2.,2.])
    assert second_t[2:] == pytest.approx(.5*(first_t[1:-1]+first_t[2:]))
    first, _, second, _ = metrics.derivative_series(np.array([0.,1.,2.,3.]), np.array([0.,.05,.2,.25]), .05)
    assert np.isnan(first[2]) and np.isnan(second[2:]).all()


def test_positive_lag_means_response_trails_reference_on_its_own_clock():
    t = np.arange(1, 301)*.05
    command_t = t-.05
    def signal(x):
        return .15*np.sin(1.7*x)+.08*np.cos(3.1*x)+.05*np.sin(5.3*x)
    true_lag = .25
    peak, _ = metrics.lag_curve(t, signal(t), command_t, signal(command_t-true_lag), np.ones(len(t),bool))
    assert peak["lag_s"] == pytest.approx(true_lag)
    assert peak["correlation"] == pytest.approx(1., abs=1e-12)
    # Pretending command values live at the later state timestamps shifts lag.
    wrong, _ = metrics.lag_curve(t, signal(t), t, signal(command_t-true_lag), np.ones(len(t),bool))
    assert wrong["lag_s"] == pytest.approx(true_lag+.05)


def test_planar_acceleration_includes_centripetal_effect_at_constant_body_speed():
    trace=np.zeros((100,14))
    trace[:,0]=np.arange(1,101)*.05
    trace[:,3]=2*trace[:,0]
    trace[:,4]=1.
    planar=metrics.planar_derivatives(trace,.05)
    expected_accel=2*np.sin(.05)/.05
    assert planar["world_accel_norm_mps2"][1:]==pytest.approx(np.full(99,expected_accel))
    expected_jerk=(2*np.sin(.05)/.05)**2
    assert planar["world_jerk_norm_mps3"][2:]==pytest.approx(np.full(98,expected_jerk))
    # Longitudinal acceleration is zero, unlike world acceleration on a turn.
    longitudinal=metrics.derivative_series(trace[:,4],trace[:,0],.05)[0]
    assert np.max(abs(longitudinal[1:]))==0


def test_constant_reference_does_not_produce_a_fake_zero_delay_estimate():
    t = np.arange(1, 101)*.05
    peak, _ = metrics.lag_curve(t, np.ones(100)*.2, t, np.ones(100)*.2, np.ones(100,bool))
    assert peak["lag_s"] is None
    assert not peak["reliable"]
    assert "constant" in peak["status"]


def test_lag_boundary_peak_is_marked_unreliable():
    t = np.arange(1,401)*.05
    def signal(x):
        return .2*np.sin(.9*x)+.1*np.cos(1.1*x)
    peak, _ = metrics.lag_curve(t, signal(t), t, signal(t-1.2), np.ones(len(t),bool))
    assert peak["lag_s"] == pytest.approx(1.)
    assert not peak["reliable"] and "window_boundary" in peak["status"]


def test_seed_blocks_are_not_pseudoreplicated_across_three_scenarios():
    trials=[]
    for scenario in ("left","right","s_bend"):
        for seed in range(3):
            for method, error in (("native",.004+seed*.001),("full",.001+seed*.0001)):
                trials.append(dict(dataset="tracking_final",scenario=scenario,condition="nominal",seed=seed,
                                   method=method,phase="turn",trial_id=f"{scenario}_{seed}_{method}",complete=True,
                                   lateral_rmse_m=error))
    pairs, summary=metrics.paired_tables(trials)
    pooled=next(r for r in summary if r["scenario"]=="fixed_three_scenarios" and r["metric"]=="lateral_rmse_m")
    assert pooled["n_valid_pairs"] == 9
    assert pooled["n_seed_blocks"] == 3
    assert pooled["seed_block_signflip_p"] == pytest.approx(.25)
    assert pooled["signflip_p_holm_all_reported_tests"] == 1.


def test_expected_worker_failure_remains_in_inventory(tmp_path):
    manifest={"arguments":{"scenarios":["left"],"plants":["nominal"],"methods":["native"],"seeds":[0,1]}}
    record=dict(scenario="left",plant="nominal",method="native",seed=1,complete=False,failure="worker_failure: timeout")
    (tmp_path/"summary.json").write_text(json.dumps([record]))
    expected=metrics.collect_expected(tmp_path, manifest)
    assert len(expected)==2
    assert expected["left_nominal_native_1"]["failure"]=="worker_failure: timeout"


def test_model_replay_reproduces_same_synthetic_plant_but_is_not_independent_validation():
    cfg=metrics.Config()
    path=metrics.Path(np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]]))
    plant=metrics.Plant(cfg,tau_w=.38,delay=.15,bias=.008)
    trace=[]
    for j in range(30):
        command=np.array([.2,.2*np.sin(j*.1)])
        plant.step(command)
        row=np.zeros(14)
        row[0],row[1:6],row[6:8]=plant.time,plant.state,command
        trace.append(row)
    trace=np.asarray(trace)
    reproduced,smooth,delayed=metrics.reconstruct(trace,path,cfg,{"true_plant":{"tau_w":.38,"delay":.15,"yaw_bias":.008}})
    assert reproduced==pytest.approx(trace[:,1:6],abs=1e-14)
    assert np.all(delayed[:3]==0)
    assert np.any(smooth[:,0]>0)
