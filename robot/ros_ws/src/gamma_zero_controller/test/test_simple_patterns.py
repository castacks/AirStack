import numpy as np
import pytest

from gamma_zero_controller.trajectory import Trajectory, TrajectoryConfig


@pytest.mark.parametrize('pattern', ['hover', 'forward_backward', 'yaw', 'altitude'])
def test_simple_reference_bounds_derivatives_and_return(pattern):
    tr = Trajectory(TrajectoryConfig(pattern=pattern))
    origin = np.array([.3, -.2, 1.3])
    tr.start(origin, np.pi/2)
    for t in np.linspace(0., 35., 351):
        p, v, phase = tr.reference(t)
        yaw, rate = tr.heading(t)
        local = tr.rotation.T@(p-origin)
        assert np.all(np.abs(local) <= np.array([tr.ax, 0., tr.az])+1e-12)
        assert np.linalg.norm(v) <= .3+1e-12
        assert abs(yaw-tr.yaw) <= np.deg2rad(20)+1e-12
        if t > 1e-4:
            np.testing.assert_allclose(v, (tr.reference(t+1e-5)[0]-tr.reference(t-1e-5)[0])/2e-5, atol=1e-8)
            np.testing.assert_allclose(rate, (tr.heading(t+1e-5)[0]-tr.heading(t-1e-5)[0])/2e-5, atol=1e-8)
    for t in [0., 10., 30., 35.]:
        p, v, _ = tr.reference(t)
        np.testing.assert_allclose(p, origin)
        np.testing.assert_allclose(v, 0., atol=1e-12)
        np.testing.assert_allclose(tr.heading(t), [tr.yaw, 0.])
    if pattern == 'forward_backward':
        np.testing.assert_allclose(tr.reference(15.)[0]-origin, [0., .5, 0.], atol=1e-12)
        np.testing.assert_allclose(tr.reference(25.)[0]-origin, [0., -.5, 0.], atol=1e-12)
    if pattern == 'altitude':
        np.testing.assert_allclose(tr.reference(15.)[0]-origin, [0., 0., .15])
        np.testing.assert_allclose(tr.reference(25.)[0]-origin, [0., 0., -.15])
    if pattern == 'yaw':
        assert tr.heading(15.)[0]-tr.yaw == pytest.approx(np.deg2rad(20))
        assert tr.heading(25.)[0]-tr.yaw == pytest.approx(-np.deg2rad(20))


@pytest.mark.parametrize('kwargs', [
    {'simple_duration_s': 0}, {'simple_duration_s': float('nan')},
    {'forward_distance_m': -1}, {'altitude_distance_m': float('inf')},
    {'yaw_amplitude_deg': 180}, {'pattern': 'forward_backward', 'simple_duration_s': 1},
    {'pattern': 'yaw', 'simple_duration_s': 1, 'max_yaw_rate': .1},
])
def test_invalid_simple_configuration(kwargs):
    with pytest.raises(ValueError):
        Trajectory(TrajectoryConfig(**dict({'pattern': 'hover'}, **kwargs)))


def test_yaw_wrap_and_feedforward_at_matched_heading():
    tr = Trajectory(TrajectoryConfig(pattern='yaw'))
    tr.start([0., 0., 1.3], np.pi-.05)
    yaw, rate = tr.heading(12.5)
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    command, _, _, _ = tr.command(12.5, tr.origin, rotation, [1.]*4)
    assert command[3] == pytest.approx(rate)
    np.testing.assert_allclose(command[:3], 0.)
