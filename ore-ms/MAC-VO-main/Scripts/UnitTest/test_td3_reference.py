import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from Scripts.Setup.td3_reference import (
    PoseReference, OPTICAL_FROM_NED, camera_reference, matrices,
    select_window, trajectory_metrics,
)


def reference():
    quats = Rotation.from_euler('z', [[0], [90]], degrees=True).as_quat()
    poses = np.concatenate([np.array([[0., 0, 0], [1., 0, 0]]), quats], axis=1)
    return PoseReference(np.array([10**18, 10**18 + 10**9]), poses)


def test_interpolation_and_extrapolation_rejection():
    ref = reference()
    middle = ref.at([10**18 + 500_000_000])[0]
    np.testing.assert_allclose(middle[:3, 3], [.5, 0, 0])
    np.testing.assert_allclose(middle[:3, :3], Rotation.from_euler('z', 45, degrees=True).as_matrix(), atol=1e-12)
    for t in [10**18 - 1, 10**18 + 10**9 + 1]:
        with pytest.raises(ValueError, match='extrapolation'): ref.at([t])


def test_reference_duplicate_times_rejected():
    with pytest.raises(ValueError, match='strictly increasing'):
        PoseReference([0, 0], [[0, 0, 0, 0, 0, 0, 1]] * 2)


def test_camera_reference_lever_arm_and_basis(tmp_path):
    directory = tmp_path / 'sensors/thermal_left_processed_camera_info'
    directory.mkdir(parents=True)
    (directory / 'data.yaml').write_text(yaml.safe_dump({'r': np.eye(3).tolist()}))
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text(yaml.safe_dump({'transform_params': [{
        'from_frame': 'vehicle', 'to_frame': 'thermal_left/optical_frame',
        'translation': [1., 0, 0], 'quaternion': [0., 0, 0, 1.]}]}))
    poses = camera_reference(reference(), [10**18, 10**18 + 10**9], tmp_path, calibration, 'thermal')
    np.testing.assert_allclose(poses[:, :3, 3], [[1, 0, 0], [1, 1, 0]], atol=1e-12)
    np.testing.assert_allclose(OPTICAL_FROM_NED @ [1, 0, 0], [0, 0, 1])
    assert np.linalg.det(OPTICAL_FROM_NED) == pytest.approx(1)


def test_relative_metrics_do_not_fit_scale():
    identity = [0, 0, 0, 1]
    ref = matrices([[0, 0, 0, *identity], [1, 0, 0, *identity]])
    est = matrices([[0, 0, 0, *identity], [2, 0, 0, *identity]])
    exact = trajectory_metrics(ref, ref)
    assert exact['consecutive_translation_rmse_m'] == 0
    result = trajectory_metrics(est, ref)
    assert result['anchored_translation_rmse_m'] == pytest.approx(1 / np.sqrt(2))
    assert result['consecutive_translation_rmse_m'] == pytest.approx(1)


def test_motion_window_selected_from_reference_only():
    ref = PoseReference([0, 10**9, 2 * 10**9], [[0, 0, 0, 0, 0, 0, 1], [1, 0, 0, 0, 0, 0, 1], [3, 0, 0, 0, 0, 0, 1]])
    rows = [{'rgb_left': {'time_ns': t}} for t in ref.time_ns]
    result = select_window(ref, rows, 2, 10**9)
    assert result['manifest_start'] == 1
    assert result['body_reference_displacement_m'] == 2
    with pytest.raises(ValueError, match='No continuous window'):
        select_window(ref, rows, 2, 10)
