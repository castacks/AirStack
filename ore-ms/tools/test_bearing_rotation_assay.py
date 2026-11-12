"""Filter, motion convention, minimum support and stopping-rule contracts."""
import numpy as np
from scipy.spatial.transform import Rotation
import bearing_rotation as br
import run_bearing_rotation_assay as assay
from pose_covariance_accuracy import mapping
from test_bearing_rotation import scene, K


def pixels(ray):
    value = ray@K.T
    return value[:, :2]/value[:, 2:]


def test_filter_repeatability_on_synthetic_geometry():
    f, g, *_ = scene()
    code, a = assay.essential_mask(pixels(f), pixels(g), K, K)
    code2, b = assay.essential_mask(pixels(f), pixels(g), K, K)
    assert code == code2 == 'OK'
    np.testing.assert_array_equal(a, b)
    assert a.sum() == len(f)


def test_multiple_essential_solutions_are_unavailable(monkeypatch):
    monkeypatch.setattr(assay.cv2, 'findEssentialMat', lambda *a, **kw: (np.ones((6, 3)), np.ones((8, 1))))
    status, mask = assay.essential_mask(np.zeros((8, 2)), np.zeros((8, 2)), K, K)
    assert status == 'MULTIPLE_ESSENTIAL_SOLUTIONS' and not mask.any()


def test_filter_rejects_changed_intrinsics_and_insufficient_support():
    status, _ = assay.essential_mask(np.zeros((7, 2)), np.zeros((7, 2)), K, K)
    assert status == 'INSUFFICIENT_POINTS'
    status, _ = assay.essential_mask(np.zeros((8, 2)), np.zeros((8, 2)), K, K*2)
    assert status == 'UNSUPPORTED_INTRINSIC_CHANGE'


def test_joint_mask_is_shared_and_preserves_exclusions(monkeypatch):
    masks = iter([np.array([1, 0, 1, 1, 1, 1, 1, 1], bool), np.array([0, 1, 1, 1, 1, 1, 1, 1], bool)])
    monkeypatch.setattr(assay, 'essential_mask', lambda *a: ('OK', next(masks)))
    packet = {'common': np.array([False]+[True]*8), 'source_K': K, 'target_K': K,
              'source_uv': np.zeros((9, 2)), 'original_uv': np.zeros((9, 2)), 'replacement_uv': np.zeros((9, 2))}
    common, _, _ = assay.filter_packet(packet)
    np.testing.assert_array_equal(common, [False, False, False, True, True, True, True, True, True])


def test_mapping_and_horizon():
    r = Rotation.from_rotvec([.01, .02, -.03]).as_matrix()
    frame = np.eye(4); frame[:3, :3] = Rotation.from_rotvec([.3, -.2, .1]).as_matrix()
    native = np.eye(4); native[:3, :3] = assay.Q.T@r@assay.Q
    expected = mapping(native, frame, .8)[:3, :3]
    np.testing.assert_allclose(assay.mapped_rotation(r, frame, [100, 200], [110, 190]), expected, atol=1e-12)
    assert assay.mapped_rotation(r, frame, [100, 200], [300, 400]) is None


def errors(nec, candidate, native):
    return {'original/NEC': nec, 'original/PNEC_original_pixel_covariance': candidate, 'native': native}


def test_empty_primary_support_stops():
    assert assay.primary_decision(errors({}, {}, {}))['decision'].startswith('STOP')


def test_native_accuracy_gate_cannot_be_ignored():
    result = assay.primary_decision(errors({0: 2.}, {0: 1.}, {0: .5}))
    assert result['checks']['improves_NEC_mean'] and not result['checks']['not_worse_native_mean']
    assert result['decision'].startswith('STOP')


def test_coverage_loss_stops_even_with_lower_matched_error():
    result = assay.primary_decision(errors({0: 2., 1: 2.}, {0: .1}, {0: 1., 1: 1.}))
    assert not result['checks']['no_NEC_coverage_loss'] and result['decision'].startswith('STOP')


def test_all_frozen_primary_conditions_are_required():
    result = assay.primary_decision(errors({0: 2., 1: 3.}, {0: .5, 1: 1.}, {0: 1., 1: 1.}))
    assert result['decision'] == 'RAIN_DEVELOPMENT_GATE_PASS'


def test_eight_point_initializer_uses_actual_nullspace():
    f, g, _, r, t, *_ = scene()
    fitted = br.initialize(f[:8], g[:8])
    assert fitted is not None
    assert Rotation.from_matrix(fitted[0].T@r).magnitude() < 1e-6
    assert abs(fitted[1]@(t/np.linalg.norm(t))) > 1-1e-8
