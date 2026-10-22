"""Mean curves preserve trial weights and cannot hide missing mocap bins."""
import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('forward_trials_report',Path(__file__).resolve().parents[1]/'tools/forward_trials_report.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def samples(error,times=(.05,.15,.25,.35,.45)):
    return [dict(time_s=t,reference_valid=True,error_xy_m=error,yaw_error_deg=-.1) for t in times]


def test_equal_trial_weights_and_zero_is_a_valid_error():
    result=module.mean_curve([samples(0.),samples(.01,(.05,.25,.45))])[0]
    assert result['support']==2 and result['xy_mean_mm']==pytest.approx(5.)
    assert result['yaw_abs_mean_deg']==pytest.approx(.1)


def test_missing_trial_is_not_zero_filled_or_interpolated():
    result=module.mean_curve([samples(.01),[]])[0]
    assert result['support']==1 and result['xy_mean_mm'] is None


def test_long_gap_or_invalid_reference_excludes_a_trial_bin():
    invalid=samples(.02)
    for s in invalid:s['reference_valid']=False
    result=module.mean_curve([samples(.01),samples(.02,(.05,.10,.45)),invalid])[0]
    assert result['support']==1 and result['xy_mean_mm'] is None
