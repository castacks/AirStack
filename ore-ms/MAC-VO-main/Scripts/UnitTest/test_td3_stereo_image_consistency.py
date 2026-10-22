import cv2
import numpy as np
import pytest

from Scripts.Setup.td3_stereo_image_consistency import acceptance, match_grid, ncc_peak, summarize


@pytest.mark.parametrize('disparity,vertical', [(12, 0), (-12, 1), (7, -1)])
def test_signed_stereo_and_vertical_translation(disparity, vertical):
    rng = np.random.default_rng(15)
    first = rng.integers(0, 256, (160, 240), dtype=np.uint8)
    second = cv2.warpAffine(first, np.array([[1., 0, -disparity], [0, 1., vertical]]), (240, 160))
    uv = np.array([[x, y] for y in [48, 80, 112] for x in [64, 96, 128, 160]])
    data = match_grid(first, second, uv, horizontal_radius=24)
    keep = acceptance(data)
    assert keep.all()
    np.testing.assert_allclose(data['observed_disparity_px'], disparity)
    np.testing.assert_allclose(data['observed_vertical_flow_px'], vertical)
    np.testing.assert_allclose(data['reciprocal_error_px'], 0)


def test_periodic_ambiguity_is_not_accepted():
    rng = np.random.default_rng(2)
    tile = rng.integers(0, 256, (160, 8), dtype=np.uint8)
    image = np.tile(tile, (1, 30));uv = np.array([[100, 80]])
    data = match_grid(image, image, uv, horizontal_radius=32)
    assert data['forward_gap'][0] < .05
    assert not acceptance(data).any()


def test_constant_image_and_search_boundary_fail_visibly():
    image = np.zeros((160, 240), np.uint8);uv = np.array([[100, 80]])
    data = match_grid(image, image, uv, horizontal_radius=24)
    assert not acceptance(data).any() and np.isnan(data['forward_score']).all()
    rng = np.random.default_rng(4);first = rng.integers(0, 256, image.shape, dtype=np.uint8)
    second = cv2.warpAffine(first, np.array([[1., 0, -24], [0, 1., 0]]), (240, 160))
    data = match_grid(first, second, uv, horizontal_radius=24)
    assert data['forward_boundary'][0] and not acceptance(data).any()
    with pytest.raises(ValueError):
        ncc_peak(first, second, [1, 1])


def test_empty_population_keeps_counts_and_undefined_statistics():
    keys = ['observed_disparity_px', 'model_disparity_px', 'horizontal_error_px',
            'vertical_error_px', 'endpoint_error_px', 'raw_stereo_variance_u',
            'raw_stereo_variance_trace', 'forward_source_std', 'forward_score']
    arrays = {key: np.ones(3) for key in keys}
    summary = summarize(arrays, np.zeros(3, bool))
    assert summary['count'] == 0 and summary['sign_eligible_count'] == 0
    assert summary['sign_agreement'] is None
    assert summary['endpoint_error_px']['p10_p50_p90'] is None
    assert summary['variance_trace_endpoint_error_rho'] is None


def test_low_contrast_ncc_is_stable_under_additive_intensity_offsets():
    rng = np.random.default_rng(91)
    first = rng.integers(192, 209, (160, 240), dtype=np.uint8)
    second = cv2.warpAffine(first, np.array([[1., 0, -12], [0, 1., 0]]), (240, 160))
    second = np.clip(second.astype(int)+rng.integers(-3, 4, second.shape), 0, 255).astype(np.uint8)
    uv = [120, 80]
    peak = ncc_peak(first, second, uv, horizontal_radius=24)
    shifted_second = np.clip(second.astype(np.int16)-100, 0, 255).astype(np.uint8)
    shifted = ncc_peak(first-100, shifted_second, uv, horizontal_radius=24)
    assert peak['endpoint'] == shifted['endpoint'] == [108, 80]
    np.testing.assert_allclose(peak['score'], shifted['score'], atol=1e-5)
    a = first[75:86, 115:126].astype(float)
    b = second[75:86, 103:114].astype(float)
    a -= a.mean();b -= b.mean()
    direct = (a*b).sum()/(np.linalg.norm(a)*np.linalg.norm(b))
    np.testing.assert_allclose(peak['score'], direct, atol=1e-5)
