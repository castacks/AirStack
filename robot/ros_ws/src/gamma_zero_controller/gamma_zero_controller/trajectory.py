from dataclasses import dataclass

import numpy as np

from gamma_zero_controller.reference import figure_eight_reference, outer_loop_command


@dataclass(frozen=True)
class TrajectoryConfig:
    pattern: str = "flown"
    settle_s: float = 10.
    laps: int = 2
    ramp_s: float = 3.
    position_gain: float = 1.
    yaw_gain: float = 1.5
    max_yaw_rate: float = 1.


class Trajectory:
    def __init__(self, config=TrajectoryConfig()):
        self.config = config
        patterns = {"flown": (1., .5, 40., .2, .8), "large": (2., 1., 24., .3, 1.)}
        if config.pattern not in patterns:
            raise ValueError("Unknown figure-eight pattern")
        if not np.isfinite([config.settle_s, config.ramp_s, config.position_gain, config.yaw_gain, config.max_yaw_rate]).all():
            raise ValueError("Nonfinite trajectory configuration")
        if config.settle_s < 0 or config.ramp_s < 0 or config.laps < 1 or int(config.laps) != config.laps:
            raise ValueError("Invalid trajectory timing")
        if min(config.position_gain, config.yaw_gain, config.max_yaw_rate) <= 0:
            raise ValueError("Controller gains and yaw limit must be positive")
        self.ax, self.ay, self.period, self.az, self.max_speed = patterns[config.pattern]
        if config.ramp_s > config.laps*self.period:
            raise ValueError("Ramp cannot exceed total lap time")
        self.duration_s = config.settle_s + config.laps*self.period + config.ramp_s
        self.origin = None
        self.yaw = None
        self.rotation = None


    def start(self, position_enu, yaw):
        position = np.asarray(position_enu, dtype=float)
        if position.shape != (3,) or not np.isfinite(np.r_[position, yaw]).all():
            raise ValueError("Invalid trajectory origin")
        self.origin = position.copy()
        self.yaw = float(yaw)
        c, s = np.cos(yaw), np.sin(yaw)
        self.rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


    def reference(self, elapsed_s):
        if self.origin is None or not np.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError("Trajectory not started or invalid elapsed time")
        t = elapsed_s-self.config.settle_s
        travel = self.config.laps*self.period
        ramp = self.config.ramp_s
        if t <= 0:
            return self.origin.copy(), np.zeros(3), "settling"
        if t >= travel+ramp:
            return self.origin.copy(), np.zeros(3), "complete_hold"
        phase, rate = t, 1.
        if ramp > 0:
            if t < ramp:
                u = t/ramp
                phase = ramp*(u**3-.5*u**4)
                rate = 3*u*u-2*u**3
            elif t <= travel:
                phase = t-.5*ramp
            else:
                u = (t-travel)/ramp
                phase = travel-.5*ramp+ramp*(u-u**3+.5*u**4)
                rate = 1.-3*u*u+2*u**3
        position, velocity = figure_eight_reference(phase, self.ax, self.ay, self.period, 0., self.az)
        return self.origin+self.rotation@position, self.rotation@velocity*rate, "tracking"


    def command(self, elapsed_s, position_enu, rotation_enu_flu, limits):
        reference, velocity, phase = self.reference(elapsed_s)
        local_position = self.rotation.T@(np.asarray(position_enu)-self.origin)
        local_rotation = self.rotation.T@np.asarray(rotation_enu_flu)
        raw = outer_loop_command(local_position, local_rotation, self.rotation.T@(reference-self.origin),
                                 self.rotation.T@velocity, "hold", self.config.position_gain,
                                 self.config.yaw_gain, self.max_speed, self.config.max_yaw_rate)
        return np.clip(raw, -np.asarray(limits), limits), reference, velocity, phase
