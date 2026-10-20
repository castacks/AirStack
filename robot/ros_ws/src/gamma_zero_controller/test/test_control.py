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
