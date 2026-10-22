"""Basic signal invariants for the explicitly parameterized Welch appendix."""
import numpy as np
import pytest
from .paper_spectrum import spectrum


def test_unit_sinusoid_peak_and_integrated_power():
    t=np.arange(2048)/20
    f,p,power=spectrum(np.sin(2*np.pi*3.125*t))
    assert f[np.argmax(p)]==pytest.approx(3.125)
    assert power['total_power']==pytest.approx(.5,rel=.01)
    assert power['fraction_above_2hz']>.999


@pytest.mark.parametrize('constant',[0.,.2,1.])
def test_constant_signal_has_undefined_power_fraction(constant):
    f,p,power=spectrum(np.full(512,constant))
    assert power['fraction_above_2hz'] is None
    assert power['power_status']=='zero_or_roundoff_power_fraction_undefined'
    assert np.isfinite(p).all() and np.all(p>=0)


def test_no_interpolation_or_dropping_of_invalid_signal():
    with pytest.raises(ValueError):
        spectrum(np.zeros(100))
    x=np.zeros(512);x[100]=np.nan
    with pytest.raises(ValueError):
        spectrum(x)
