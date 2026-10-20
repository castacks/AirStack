import numpy as np
import pytest

from Scripts.Setup.td3_weight_influence import (
    analyze, group_diagnostics, projection, projection_jacobian, rotation_xyzw,
)


def geometry():
    rng = np.random.default_rng(3)
    points = rng.uniform([4, -2, -1], [20, 2, 1], size=(60, 3))
    k = np.array([[200., 0, 160], [0, 200., 120], [0, 0, 1.]])
    return points, k


def test_jacobian_matches_local_camera_pose_finite_difference():
    points, k = geometry()
    jac = projection_jacobian(points, k, .3)
    epsilon = 1e-6
    for axis in range(6):
        values = []
        for sign in [-1, 1]:
            delta = np.zeros(6)
            delta[axis] = sign*epsilon
            angle = np.linalg.norm(delta[3:])
            q = np.r_[delta[3:]/angle*np.sin(angle/2), np.cos(angle/2)] if angle else np.array([0., 0, 0, 1])
            shifted = (points-delta[:3]) @ rotation_xyzw(q)
            values.append(projection(shifted, k, .3))
        np.testing.assert_allclose((values[1]-values[0])/(2*epsilon), jac[:, :, axis], atol=1e-7)


def test_precision_leverage_and_deletion_match_direct_linear_solution():
    points, k = geometry()
    j = projection_jacobian(points, k, .3)
    rng = np.random.default_rng(12)
    r = rng.normal(size=(len(points), 3))
    v = np.broadcast_to([.4, .6, 2.], r.shape).copy()
    mask = np.arange(len(points)) < 20
    report = group_diagnostics(j, r, v, {'subset': mask, 'all': np.ones(len(points), bool)})
    assert report['rank'] == 6
    assert report['leverage_sum'] == pytest.approx(6)
    group = report['groups']['subset']
    np.testing.assert_allclose(group['pixel_precision_share_u_v'], [1/3, 1/3])
    a, b = j/np.sqrt(v)[..., None], r/np.sqrt(v)
    full = np.linalg.lstsq(a.reshape(-1, 6), -b.ravel(), rcond=None)[0]
    removed = np.linalg.lstsq(a[~mask].reshape(-1, 6), -b[~mask].ravel(), rcond=None)[0]
    assert group['linearized_deletion_step_change']['translation_m'] == pytest.approx(np.linalg.norm((removed-full)[:3]))
    assert group['linearized_deletion_step_change']['rotation_deg'] == pytest.approx(np.rad2deg(np.linalg.norm((removed-full)[3:])))
    assert report['groups']['all']['linearized_deletion_step_change'] is None


def test_nonuniform_precision_and_invalid_covariance():
    points, k = geometry()
    j = projection_jacobian(points, k, .3)
    v = np.ones((len(points), 3))
    mask = np.arange(len(points)) < 30
    v[mask, :2] = .25
    report = group_diagnostics(j, np.zeros_like(v), v, {'near': mask})
    np.testing.assert_allclose(report['groups']['near']['pixel_precision_share_u_v'], [.8, .8])
    v[0, 0] = 0
    with pytest.raises(ValueError):
        group_diagnostics(j, np.zeros_like(v), v, {'near': mask})
    points[0, 0] = -1
    with pytest.raises(ValueError):
        projection_jacobian(points, k, .3)


def test_saved_graph_analysis_preserves_inputs_and_partitions_observations():
    points, k = geometry()
    uvdisp = projection(points, k, .3)
    n = len(points)
    graph = {'frames//pose': np.array([[0., 0, 0, 0, 0, 0, 1]]*2),
             'frames//need_interp': np.zeros(2, bool),
             'frames//K': np.stack([k]*2), 'frames//baseline': np.array([[.3], [.3]]),
             'frames//time_ns': np.array([[100], [200]]),
             'edge/match2frame1/mapping': np.zeros(n, int),
             'edge/match2frame2/mapping': np.ones(n, int),
             'edge/match2point/mapping': np.arange(n), 'points//pos_Tw': points,
             'match//pixel2_uv': uvdisp[:, :2], 'match//pixel2_disp': uvdisp[:, 2:],
             'match//pixel2_uv_cov': np.tile([.25, .25, 0], (n, 1)),
             'match//pixel2_disp_cov': np.ones((n, 1))}
    original = {name: value.copy() for name, value in graph.items()}
    report = analyze(graph, [.4, .6, 1], 240, 320)
    for name, value in graph.items():
        np.testing.assert_array_equal(value, original[name])
    for arm in ['learned', 'fixed_uv']:
        groups = report[0][arm]['groups']
        for prefix in ['row_', 'column_', 'forward_depth_']:
            assert sum(g['count'] for name, g in groups.items() if name.startswith(prefix)) == n
        assert report[0][arm]['leverage_sum'] == pytest.approx(6)
    with pytest.raises(ValueError, match='dimensions'):
        analyze(graph, [.4, .6, 1], 20, 20)
