"""Check scientific metric conventions and identifiable synthetic recovery."""
from dataclasses import replace

import numpy as np
import pytest

from .controller import Config, GuidedMPPI
from .paper_supplement import (RolloutObserver, fit_channel, fit_output_error,
                               make_path, output_error_prediction, settling_metrics)


def test_first_residence_and_last_exit_are_distinct_and_can_be_censored():
    times=np.arange(0,4.01,.05)
    lateral=np.where(((times>=.5)&(times<=1.6))|(times>=3.5),0.,.02)
    result=settling_metrics(times,lateral,np.zeros(len(times)))
    assert result["joint_settling_first_window_s"] == pytest.approx(.5)
    assert result["first_window_censored"] is False
    assert result["last_exit_settling_s"] is None
    assert result["last_exit_censored"] is True
    assert result["threshold_exit_count"] == 1


def test_final_residence_requires_full_observed_dwell():
    times=np.arange(0,3.01,.05)
    result=settling_metrics(times,np.where(times>=2.,0.,.02),np.zeros(len(times)))
    assert result["last_exit_settling_s"] == pytest.approx(2.)
    assert not result["last_exit_censored"]
    result=settling_metrics(times,np.full(len(times),.02),np.zeros(len(times)))
    assert result["first_window_censored"] and result["last_exit_censored"]


def test_identifiable_smooth_input_recovers_delay_lag_bias_and_validates():
    tau,delay,bias=.38,3,.008
    decay=np.exp(-.05/tau)
    def trace(seed):
        rng=np.random.default_rng(seed)
        rows=np.zeros((240,9))
        rows[:,0]=np.arange(240)*.05
        rows[:,4]=rng.uniform(-.2,.2,240)
        for k in range(1,len(rows)):
            inp=rows[k-delay,4] if k>=delay else 0.
            rows[k,8]=decay*rows[k-1,8]+(1-decay)*(inp+bias)
        return rows
    fit,prediction=fit_channel(trace(4),trace(8),"w")
    assert fit["delay_steps"] == delay
    assert fit["tau_s"] == pytest.approx(tau,abs=1e-10)
    assert fit["bias"] == pytest.approx(bias,abs=1e-10)
    assert fit["validation_recursive_rmse"] < 1e-12
    assert prediction.shape == (240,)


def test_cost_observation_preserves_commands_and_rng_sequence():
    cfg=replace(Config(),samples=8,steps=12)
    plain=GuidedMPPI(make_path("left"),cfg,seed=14)
    observed=GuidedMPPI(make_path("left"),cfg,seed=14)
    observer=RolloutObserver(observed)
    for x in [0.,.01,.02]:
        state=np.array([x,.1,.12,.1,0.])
        actual=observed.command(state,pending=[(.1,0.),(.1,0.)])
        expected=plain.command(state,pending=[(.1,0.),(.1,0.)])
        assert np.array_equal(actual,expected)
        assert np.array_equal(observed.residual,plain.residual)
        assert observer.calls
        observer.calls.clear()


def test_output_error_fit_recovers_unknown_gain_and_bias_without_noisy_arx():
    u=np.random.default_rng(34).uniform(-.2,.2,250)
    truth=output_error_prediction(u,.37,1.08,.004,2)
    fit=fit_output_error(u,truth,.003)
    assert fit["delay_steps"] == 2
    assert fit["tau_s"] == pytest.approx(.37,abs=1e-7)
    assert fit["gain"] == pytest.approx(1.08,abs=1e-7)
    assert fit["bias"] == pytest.approx(.004,abs=1e-8)
    assert fit["jacobian_rank"] == 3
    assert fit["active_bound_count"] == 0
