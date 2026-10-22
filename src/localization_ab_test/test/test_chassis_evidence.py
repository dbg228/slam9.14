from localization_ab_test.chassis_evidence import hardware_readiness


def test_require_completed_restore_after_latest_reset():
    good='=== Operation Group 1 start ===\nVelocity command gate: OPEN\n=== Operation Group 3 complete ===\n'
    assert hardware_readiness(good)['ready']
    assert not hardware_readiness(good+'=== Operation Group 1 start ===')['ready']
    assert not hardware_readiness(good+'[ERROR] motor fault')['ready']
    assert not hardware_readiness('Velocity command gate: OPEN')['ready']
