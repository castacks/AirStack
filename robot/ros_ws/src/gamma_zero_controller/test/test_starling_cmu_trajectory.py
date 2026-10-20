import numpy as np
import pytest

from gamma_zero_controller.trajectory import Trajectory, TrajectoryConfig
from gamma_zero_controller.reference import figure_eight_reference, outer_loop_command


@pytest.mark.parametrize("pattern,parameters", [
    ("flown", (1., .5, 40., .2, .8)),
    ("large", (2., 1., 24., .3, 1.)),
])
def test_unramped_protocol_matches_evaluator(pattern, parameters):
    trajectory = Trajectory(TrajectoryConfig(pattern=pattern, ramp_s=0))
    origin = np.array([.2, -.1, 1.3])
    trajectory.start(origin, 0.)
    ax, ay, period, az, speed = parameters
    for t in np.arange(10.02, 10+2*period, .37):
        expected, velocity = figure_eight_reference(t-10, ax, ay, period, origin[2], az)
        expected[:2] += origin[:2]
        goal, reference, actual_velocity, phase = trajectory.command(t, origin, np.eye(3), [2., 2., 2., 2.])
        np.testing.assert_allclose(reference, expected, atol=1e-12)
        np.testing.assert_allclose(actual_velocity, velocity, atol=1e-12)
        np.testing.assert_allclose(goal, outer_loop_command(origin, np.eye(3), expected, velocity, "hold", 1., 1.5, speed, 1.), atol=1e-7)
        assert phase == "tracking"


def test_ramp_velocity_is_derivative_and_endpoints_are_stationary():
    trajectory = Trajectory()
    trajectory.start([0., 0., 1.], .7)
    for t in [10., 11., 13., 30., 90., 91., 93.]:
        p, v, phase = trajectory.reference(t)
        before = trajectory.reference(t-1e-5)[0]
        after = trajectory.reference(t+1e-5)[0]
        np.testing.assert_allclose(v, (after-before)/2e-5, atol=1e-8)
    for t in [0., 10., 93., 100.]:
        p, v, phase = trajectory.reference(t)
        np.testing.assert_allclose(p, [0., 0., 1.], atol=1e-12)
        np.testing.assert_allclose(v, 0., atol=1e-12)


def test_origin_heading_rotates_path_and_preserves_body_command():
    a, b = Trajectory(), Trajectory()
    a.start([0., 0., 1.], 0.)
    b.start([3., 4., 2.], np.pi/2)
    ga, pa, va, _ = a.command(20., [0., 0., 1.], np.eye(3), [2.]*4)
    gb, pb, vb, _ = b.command(20., [3., 4., 2.], b.rotation, [2.]*4)
    np.testing.assert_allclose(ga, gb, atol=1e-7)
    np.testing.assert_allclose(pb-b.origin, b.rotation@(pa-a.origin), atol=1e-12)
    np.testing.assert_allclose(vb, b.rotation@va, atol=1e-12)


@pytest.mark.parametrize("config", [
    TrajectoryConfig(pattern="invalid"), TrajectoryConfig(ramp_s=-1),
    TrajectoryConfig(settle_s=float("nan")), TrajectoryConfig(laps=0),
])
def test_invalid_trajectory_config(config):
    with pytest.raises(ValueError):
        Trajectory(config)


def test_golden_samples_generated_on_s2():
    import json
    from pathlib import Path
    rows = json.loads((Path(__file__).parent/'fixtures/trajectory_golden.json').read_text())
    for row in rows:
        tr = Trajectory(TrajectoryConfig(pattern=row['pattern'],ramp_s=row['ramp']))
        tr.start([.2,-.1,1.3],.7)
        p,v,phase = tr.reference(row['t'])
        np.testing.assert_allclose(p,row['p'],atol=1e-12)
        np.testing.assert_allclose(v,row['v'],atol=1e-12)
        assert phase == row['phase']
