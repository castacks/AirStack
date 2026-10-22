from pathlib import Path

import numpy as np
import pytest

from Scripts.Setup.td3_reference import PoseReference
from Scripts.Setup.td3_reference_timing import (
    compare_export, endpoint_errors, reference_at_offset, source_timestamps,
)


def reference():
    # Acceleration makes known time shifts observable, with a large Unix epoch.
    times = 1_761_617_620_000_000_000 + np.arange(5, dtype=np.int64) * 1_000_000_000
    poses = np.zeros((5, 7));poses[:, 0] = np.arange(5) ** 2;poses[:, 6] = 1
    return PoseReference(times, poses)


def test_timestamp_precision_holes_and_duplicate_indices(tmp_path):
    path = tmp_path / 'timestamps.txt'
    path.write_text('1761617620.207000001\nNaN\n-1\n1761617620.207000001\n')
    assert source_timestamps(path) == [1761617620207000001, None, None, 1761617620207000001]


def test_reference_offset_precision_and_coverage():
    ref = reference();times = ref.time_ns[[1, 2]]
    np.testing.assert_allclose(reference_at_offset(ref, times, 1_000_000_000)[:, 0, 3], [4, 9])
    assert reference_at_offset(ref, times, 1)[0, 0, 3] > 1
    with pytest.raises(ValueError):
        reference_at_offset(ref, ref.time_ns[[0, 1]], -1)
    with pytest.raises(ValueError):
        reference_at_offset(ref, times, .1)


def test_binding_export_agreement_invalid_rows_and_no_extrapolation():
    ref = reference();times = [int(ref.time_ns[1]), int(ref.time_ns[1] + 500_000_000), None, int(ref.time_ns[-1] + 1)]
    poses = np.zeros((4, 7));poses[:, 6] = 1;poses[:2, 0] = [1, 2.5]
    result = compare_export(ref, times, poses)
    assert result['compared_rows'] == 2 and result['invalid_rows'] == 1 and result['outside_source_support'] == 1
    assert result['translation_m']['max'] == 0
    poses[1, 0] += .2
    assert compare_export(ref, times, poses)['translation_m']['max'] == pytest.approx(.2)
    with pytest.raises(ValueError):
        compare_export(ref, times[:-1], poses)


def test_known_shifted_projection_and_depth_free_residual():
    ref = reference();times = ref.time_ns[[1, 2]]
    camera = reference_at_offset(ref, times, 0)
    relative = np.linalg.inv(camera[1]) @ camera[0]
    points = {'uv0': np.array([[50., 50.], [60., 60.]]), 'depth0': np.array([10., 20.]),
              'K': np.array([[100., 0, 50], [0, 100., 50], [0, 0, 1.]])}
    # Translation is along NED forward, so scale image offsets analytically.
    points['lk_endpoint'] = 50 + (points['uv0'] - 50) * points['depth0'][:, None] / (points['depth0'][:, None] - 3)
    f, s = endpoint_errors(points, relative)
    np.testing.assert_allclose(f, 0, atol=1e-12)
    shifted = reference_at_offset(ref, times, 1_000_000_000)
    fshift, _ = endpoint_errors(points, np.linalg.inv(shifted[1]) @ shifted[0])
    assert fshift[1] > .1
    points['depth0'][:] = np.nan
    missing, sampson = endpoint_errors(points, relative)
    assert np.isnan(missing).all()
    np.testing.assert_allclose(sampson, s, equal_nan=True)
    assert np.isnan(endpoint_errors(points, np.eye(4))[1]).all()
