from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest
from scipy.linalg import expm

from common_time_model import identity_assumption, integrate, scale_motion, transport


def twist():
    x = np.zeros((4, 4)); x[:3, :3] = [[0, -.4, -.2], [.4, 0, -.3], [.2, .3, 0]]
    x[:3, 3] = [.6, -.8, .2]
    return x


def call(native=None, target=None, source='rgb', contract=None, e=None, d=None):
    return transport((expm(twist()*.2) if d is None else d).tolist(),
                     [100, 300] if native is None else native,
                     [200, 450] if target is None else target,
                     source, 'rgb', (np.eye(4) if e is None else e).tolist(), contract)


def test_constant_twist_shift_duration_and_conjugation():
    e = expm(twist()*.7)
    result = call(e=e)
    np.testing.assert_allclose(result['delta_target'], e@expm(twist()*.25)@np.linalg.inv(e), atol=1e-12)
    assert result['transport_kind'] == 'conditional_model'
    assert result['admission_ready'] is False


def test_acceleration_negative_control():
    # x(t)=t+2t². Native .1->.3: .36; shifted .2->.4 truth: .44.
    d = np.eye(4); d[0, 3] = .36
    result = call(target=[200, 400], d=d)
    assert result['accepted']
    assert abs(result['delta_target'][0][3]-.44) == pytest.approx(.08)


def test_epoch_and_unit_invariance():
    base = call()['delta_target']
    for factor, epoch in [(1, 10**18), (1000000, 0)]:
        result = call(native=[epoch+factor*t for t in [100, 300]],
                      target=[epoch+factor*t for t in [200, 450]])
        np.testing.assert_allclose(result['delta_target'], base, atol=1e-14)


def test_exact_and_cross_clock_identity_are_distinct():
    exact = call(target=[100, 300])
    assert exact['transport_kind'] == 'exact_native'
    assert call(source='thermal')['reason'] == 'clock_contract_required'
    model = call(source='thermal', contract=identity_assumption('thermal', 'rgb'), target=[100, 300])
    assert model['transport_kind'] == 'conditional_model'
    assert not model['sensor_clock_verified']


@pytest.mark.parametrize('key,value', [('offset_ns', 1), ('rate_numerator', 2),
    ('offset_ns', False), ('rate_denominator', 1.), ('physically_verified', True),
    ('source_clock_id', 'other'), ('kind', 'fitted')])
def test_clock_contract_rejects_fits_and_false_proof(key, value):
    c = identity_assumption('thermal', 'rgb'); c[key] = value
    with pytest.raises(ValueError):
        call(source='thermal', contract=c)


@pytest.mark.parametrize('endpoints', [[1., 2], [True, 2], [2, 2], [-1, 2], [3, 2]])
def test_exact_integer_interval(endpoints):
    with pytest.raises(ValueError):
        call(native=endpoints)


def test_horizon_boundary_and_gap():
    assert call(target=[300, 500])['accepted']
    bad = call(target=[300, 501])
    assert not bad['accepted'] and bad['delta_target'] is None
    good = call(target=[100, 300])
    result = integrate([good, bad, good])
    assert result['poses'][2] is None
    assert result['segments'] == [0, 0, None, 1]
    assert result['gauge_roots'][1]['published_gauge_anchor'] is False
    np.testing.assert_allclose(result['poses'][3], good['delta_target'])


@pytest.mark.parametrize('angle', [0., 1e-10, .2, np.pi-1e-6])
def test_translation_rotation_near_zero_and_pi(angle):
    x = np.zeros((4, 4)); x[:3, :3] = [[0, -angle, 0], [angle, 0, 0], [0, 0, 0]]
    x[:3, 3] = [.3, -.7, .1]
    np.testing.assert_allclose(scale_motion(expm(x), .4), expm(x*.4), atol=2e-10)


def test_rigid_roundoff_and_bad_geometry():
    d = expm(twist()); d[0, 0] += 1e-7
    assert call(d=d)['rotation_roundoff_max_abs'] > 0
    d[0, 0] += .1
    with pytest.raises(ValueError): call(d=d)


def test_strict_publisher_still_rejects_thermal_native_clock():
    # Reuse a fully formed strict contract; projection does not relabel its provenance.
    from test_selected_motion_trajectory import contract
    from selected_motion_trajectory import publish
    c = contract()
    c['slots'][0]['selected']['payload']['sensor_clock_id'] = 'other_clock'
    out = publish(c)
    assert not out['decisions'][0]['accepted']
