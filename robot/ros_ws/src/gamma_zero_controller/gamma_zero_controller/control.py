from dataclasses import dataclass

import numpy as np

from gamma_zero_controller.ctbr import BVConfig, ctbr_to_airstack
from gamma_zero_controller.session import FlightSession


@dataclass
class Reference:
    position: np.ndarray
    velocity: np.ndarray
    yaw: float
    yaw_rate: float = 0.
    rate_mode: bool = False


class Controller:
    def __init__(self, policy, config):
        self.session = FlightSession(policy, config)
        self.config = config
        self.mode = 'conventional'
        self.source = 'svg'
        self.fault = ''
        self.hold = None
        self.generation = 0
        self.inference_s = 0.
        self.last_record = {}
        self.thrust = BVConfig(hover_normalized=float(config.get('hover_normalized', .15190907567739487)))
        self.stable_since = None


    def update_stability(self, ros_s, monotonic_s):
        self.session.check_state(ros_s, monotonic_s)
        stable = np.linalg.norm(self.session.state['linear_velocity_flu']) < .15
        stable = stable and np.linalg.norm(self.session.state['angular_velocity_flu']) < .2
        self.stable_since = (self.stable_since if self.stable_since is not None else monotonic_s) if stable else None


    def engage(self, ros_s, monotonic_s, airborne_hold):
        if self.mode != 'conventional' or not airborne_hold:
            raise ValueError('Engage requires conventional airborne hold')
        self.update_stability(ros_s, monotonic_s)
        if self.stable_since is None or monotonic_s-self.stable_since < 1.:
            raise ValueError('One second of stable hold required')
        self.session.check_state(ros_s, monotonic_s)
        self.session.policy.reset_history()
        self.session.previous = np.zeros(4)
        self.session.active = True
        self.session.last_tick_s = monotonic_s
        self.mode, self.source, self.fault = 'ctbr', 'svg', ''
        self.hold = None
        self.generation += 1


    def handback(self, ros_s, monotonic_s, reason='operator handback'):
        self.mode, self.source, self.fault = 'conventional', 'hold', reason
        self.generation += 1
        self.session.active = False
        self.session.check_state(ros_s, monotonic_s)
        state = self.session.state
        r = state['rotation_enu_flu']
        self.hold = Reference(state['position_enu'].copy(), np.zeros(3), float(np.arctan2(r[1, 0], r[0, 0])))


    def start_trajectory(self, ros_s, monotonic_s):
        if self.mode != 'ctbr':
            raise ValueError('Engage policy before starting the tested trajectory')
        self.session.active = False
        self.session.start(ros_s, monotonic_s)
        self.source = 'tested_trajectory'


    def command(self, reference, ros_s, monotonic_s):
        self.session.check_state(ros_s, monotonic_s)
        if not 0 <= monotonic_s-self.session.last_tick_s <= self.config['max_tick_gap_s']:
            raise ValueError('Control deadline missed')
        s = self.session.state
        if self.source == 'tested_trajectory':
            elapsed = monotonic_s-self.session.started_s
            if elapsed > self.session.trajectory.duration_s+self.config['completion_hold_s']:
                raise ValueError('Tested trajectory completion hold expired')
            p, v, phase = self.session.trajectory.reference(elapsed)
            reference = Reference(p, v, self.session.trajectory.yaw)
            speed_limit = self.session.trajectory.max_speed
        else:
            phase, speed_limit = 'svg', float(self.config.get('max_speed_mps', .8))
        if reference is None:
            raise ValueError('Missing SVG reference')
        error = reference.position-s['position_enu']
        if not np.isfinite(np.r_[reference.position, reference.velocity, reference.yaw, reference.yaw_rate]).all():
            raise ValueError('Nonfinite reference')
        if np.linalg.norm(error) > self.config['max_tracking_error_m']:
            raise ValueError('Tracking error exceeds runtime limit')
        world = reference.velocity+self.session.trajectory.config.position_gain*error
        world *= min(1., speed_limit/max(np.linalg.norm(world), 1e-12))
        margin = float(self.config.get('fence_margin_m', .15))
        brake = float(self.config.get('brake_accel_mps2', .5))
        if margin <= 0 or brake <= 0:
            raise ValueError('Invalid fence braking configuration')
        low = np.sqrt(2*brake*np.maximum(s['position_enu']-self.session.low-margin, 0))
        high = np.sqrt(2*brake*np.maximum(self.session.high-s['position_enu']-margin, 0))
        bounded = np.clip(world, -low, high)
        yaw = np.arctan2(s['rotation_enu_flu'][1, 0], s['rotation_enu_flu'][0, 0])
        yaw_error = np.arctan2(np.sin(reference.yaw-yaw), np.cos(reference.yaw-yaw))
        yaw_rate = reference.yaw_rate if reference.rate_mode else self.session.trajectory.config.yaw_gain*yaw_error
        yaw_rate = np.clip(yaw_rate, -self.session.trajectory.config.max_yaw_rate, self.session.trajectory.config.max_yaw_rate)
        goal = np.r_[s['rotation_enu_flu'].T@bounded, yaw_rate].astype(np.float32)
        goal = np.clip(goal, -np.asarray(self.session.policy.assets['max_command_velocity']), self.session.policy.assets['max_command_velocity'])
        action, physical = self.session.policy.act(s, goal, self.session.previous)
        mapped = ctbr_to_airstack(physical, self.thrust)
        self.session.previous = action.copy()
        self.session.last_tick_s = monotonic_s
        self.last_record = {'phase': phase, 'position_enu': s['position_enu'].tolist(),
                            'reference_enu': reference.position.tolist(), 'reference_velocity_enu': reference.velocity.tolist(),
                            'goal_velocity': goal.tolist(), 'ctbr_si': physical.tolist(),
                            'saturated': mapped['saturated'], 'fence_limited': bool(np.any(world != bounded))}
        return mapped
