"""Synthetic bridge to the inherited nominal frame/time mapping; no labels."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from pose_covariance_accuracy import mapping


@pytest.mark.parametrize('ratio', [.5, 1., 1.7])
def test_optical_rotation_matches_native_frame_and_time_mapping(ratio):
    # Utility.Point maps optical EDN to camera NED by rolling coordinates once.
    q = np.array([[0., 1, 0], [0, 0, 1], [1, 0, 0]])
    optical = Rotation.from_rotvec([.03, -.02, .01]).as_matrix()
    mounting = Rotation.from_rotvec([.2, -.3, .1]).as_matrix()
    frame = np.eye(4); frame[:3, :3] = mounting@q
    frame[:3, 3] = [1.2, -.7, .4]
    expected = Rotation.from_rotvec(ratio*Rotation.from_matrix(
        mounting@optical@mounting.T).as_rotvec()).as_matrix()
    for translation in [[0., 0, 0], [.5, -.1, .2]]:
        native = np.eye(4); native[:3, :3] = q.T@optical@q
        native[:3, 3] = translation
        np.testing.assert_allclose(mapping(native, frame, ratio)[:3, :3], expected, atol=1e-12)


def test_inverse_motion_reverses_rotation():
    r = Rotation.from_rotvec([.1, -.04, .03]).as_matrix()
    t = np.eye(4); t[:3, :3] = r; t[:3, 3] = [.2, .5, -.3]
    mapped = mapping(t, np.eye(4), 1.)
    inverse = mapping(np.linalg.inv(t), np.eye(4), 1.)
    np.testing.assert_allclose(inverse[:3, :3], mapped[:3, :3].T, atol=1e-12)
