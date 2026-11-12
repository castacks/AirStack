"""Known image warps test endpoint accuracy independently of the fit residual."""
import numpy as np
import pytest
from radiometric_tracking import refine


def image(dx=0., dy=0., gain=1.):
    y, x = np.mgrid[:80, :96].astype(float)
    x, y = x-dx, y-dy
    values = 15000+1000*np.sin(x*.17)+850*np.cos(y*.23)+700*np.sin(x*.11+y*.19)
    return np.round(gain*values).astype(np.uint16)


@pytest.mark.parametrize('shift', [(1., -1.), (.65, -.45), (-.7, .35)])
@pytest.mark.parametrize('gain', [1., 1.5])
def test_recovers_known_motion_from_biased_prior(shift, gain):
    point = np.array([45., 40.]); truth = point+shift
    result = refine(image(), image(*shift, gain), point, truth+[.45, -.35])
    assert result['status'] == 'OK'
    assert np.linalg.norm(np.array(result['endpoint_uv'])-truth) < .05


def test_no_mutation_and_repeatability():
    a, b = image(), image(1, -1); old_a, old_b = a.copy(), b.copy()
    first = refine(a, b, [45, 40], [46.2, 38.8])
    assert first == refine(a, b, [45, 40], [46.2, 38.8])
    assert np.array_equal(a, old_a) and np.array_equal(b, old_b)


def test_full_range_keeps_sub_u8_texture():
    # All pixels collapse to a single code under a right shift of eight bits.
    a = (30080+np.rint((image().astype(float)-15000)/50)).astype(np.uint16)
    b = np.roll(a, 1, axis=1)
    assert np.unique(a >> 8).size == 1
    result = refine(a, b, [45, 40], [46.3, 39.7])
    assert result['status'] == 'OK'
    assert np.linalg.norm(np.array(result['endpoint_uv'])-[46, 40]) < .05


def test_flat_images_not_valid_tracks():
    flat = np.full((80, 96), 15000, np.uint16)
    assert refine(flat, flat, [45, 40], [45, 40])['status'] == 'LOW_TEXTURE'


def test_aperture_is_explicit():
    stripes = np.tile((15000+1000*np.sin(np.arange(96)*.17)).astype(np.uint16), (80, 1))
    assert refine(stripes, stripes, [45, 40], [45, 40])['status'] == 'APERTURE'


def test_large_wrong_prior_does_not_claim_local_success():
    result = refine(image(), image(5, 0), [45, 40], [45, 40])
    assert result['status'] in ('CORRECTION_LIMIT', 'NOT_CONVERGED')
    assert result['endpoint_uv'] is None


@pytest.mark.parametrize('point', [[1, 1], [np.nan, 40], [np.inf, 40]])
def test_invalid_or_border_points_retained(point):
    result = refine(image(), image(), point, point)
    assert result['status'] in ('BORDER', 'INVALID_COORDINATES')
    assert result['endpoint_uv'] is None


def test_rejects_display_quantized_input():
    with pytest.raises(ValueError, match='uint16'):
        refine(image().astype(np.uint8), image(), [45, 40], [45, 40])


def test_zero_target_not_a_valid_track():
    assert refine(image(), np.zeros((80, 96), np.uint16), [45, 40], [45, 40])['status'] == 'INVALID_TARGET_PATCH'
