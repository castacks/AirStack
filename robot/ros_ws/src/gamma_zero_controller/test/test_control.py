import json
from pathlib import Path
import numpy as np
import pytest
from gamma_zero_controller.control import Controller, Reference
from gamma_zero_controller.trajectory import TrajectoryConfig


class Policy:
    assets = {'max_command_velocity': [1.,1.,1.,1.]}

    def reset_history(self):
        pass

    def act(self, state, goal, previous):
        self.goal = goal
        return np.zeros(4), np.array([5.90562, 0.,0.,0.])


@pytest.fixture
def control():
    c = Controller(Policy(), json.loads((Path(__file__).parents[1]/'config/simulation.json').read_text()))
    c.session.update_state([1.,0.,0.,0.], [0.,0.,-1.3], [0.]*3, [0.]*3, 100., 10., 0)
    c.update_stability(100., 10.)
    c.session.update_state([1.,0.,0.,0.], [0.,0.,-1.3], [0.]*3, [0.]*3, 101.1, 11.1, 0)
    return c


def test_engagement_requires_airborne_hold(control):
    with pytest.raises(ValueError, match='airborne'):
        control.engage(101.1,11.1,False)
    control.engage(101.1,11.1,True)
    control.handback(101.1,11.1)
    assert control.mode == 'conventional' and control.source == 'hold'
    assert control.generation == 2


def test_fence_applies_after_position_correction(control):
    control.engage(101.1,11.1,True)
    s=control.session
    s.state['position_enu'][0] = 3.9
    r=Reference(np.array([4.1,0.,1.3]), np.array([.4,0.,0.]), 0.)
    control.command(r,101.1,11.1)
    assert control.last_record['fence_limited']
    assert abs((s.state['rotation_enu_flu']@control.session.policy.goal[:3])[0]) < 1e-9


@pytest.mark.parametrize('pattern',['flown','large'])
@pytest.mark.parametrize('ramp',[0.,3.])
def test_deployment_command_matches_existing_evaluator(control,pattern,ramp):
    control.engage(101.1,11.1,True)
    control.session.trajectory.config = TrajectoryConfig(pattern=pattern,ramp_s=ramp)
    from gamma_zero_controller.trajectory import Trajectory
    control.session.trajectory = Trajectory(control.session.trajectory.config)
    control.start_trajectory(101.1,11.1)
    t=25.
    trajectory=control.session.trajectory
    p,v,_=trajectory.reference(t)
    control.session.state['position_enu']=p+np.array([.01,.02,.01])
    control.session.source_s=101.1+t
    control.session.received_s=11.1+t
    control.session.last_tick_s=11.1+t
    expected,*_=trajectory.command(t,control.session.state['position_enu'],control.session.state['rotation_enu_flu'],[1.]*4)
    control.command(None,101.1+t,11.1+t)
    np.testing.assert_allclose(control.session.policy.goal,expected,atol=1e-7)


@pytest.mark.parametrize("pattern", ["flown", "large"])
def test_evaluated_terminal_hold_is_two_seconds(control, pattern):
    from gamma_zero_controller.trajectory import Trajectory
    control.session.trajectory = Trajectory(TrajectoryConfig(pattern=pattern))
    assert control.config["completion_hold_s"] == 2.
    control.engage(101.1, 11.1, True)
    control.start_trajectory(101.1, 11.1)
    elapsed = control.session.trajectory.duration_s + 2.02
    control.session.source_s = 101.1 + elapsed
    control.session.received_s = 11.1 + elapsed
    control.session.last_tick_s = 11.1 + elapsed
    with pytest.raises(ValueError, match="completion hold expired"):
        control.command(None, 101.1 + elapsed, 11.1 + elapsed)



def test_yaw_pattern_reaches_policy_with_heading_feedback_and_feedforward(control):
    from gamma_zero_controller.trajectory import Trajectory
    control.session.trajectory = Trajectory(TrajectoryConfig(pattern='yaw', settle_s=0.))
    control.engage(101.1, 11.1, True)
    control.start_trajectory(101.1, 11.1)
    control.session.source_s, control.session.received_s = 103.6, 13.6
    control.session.last_tick_s = 13.6
    control.command(None, 103.6, 13.6)
    target, feedforward = control.session.trajectory.heading(2.5)
    assert control.session.policy.goal[3] == pytest.approx(1.5*(target-np.pi/2)+feedforward)
    assert control.last_record['reference_yaw'] == pytest.approx(target)


@pytest.mark.parametrize('pattern', ['hover', 'forward_backward', 'yaw', 'altitude'])
def test_simple_completion_enters_existing_handback_path(control, pattern):
    from gamma_zero_controller.trajectory import Trajectory
    control.session.trajectory = Trajectory(TrajectoryConfig(pattern=pattern, settle_s=0.))
    control.engage(101.1, 11.1, True)
    control.start_trajectory(101.1, 11.1)
    control.session.source_s, control.session.received_s = 123.2, 33.2
    control.session.last_tick_s = 33.2
    with pytest.raises(ValueError, match='completion hold expired'):
        control.command(None, 123.2, 33.2)


def test_forward_excursion_rejected_when_route_exceeds_fence(control):
    from gamma_zero_controller.trajectory import Trajectory
    control.session.trajectory = Trajectory(TrajectoryConfig(pattern='forward_backward'))
    control.session.state['position_enu'][:2] = control.session.high[:2]-.6
    control.engage(101.1, 11.1, True)
    with pytest.raises(ValueError, match='Planned route'):
        control.start_trajectory(101.1, 11.1)
