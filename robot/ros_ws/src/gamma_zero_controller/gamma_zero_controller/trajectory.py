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
    simple_duration_s: float = 20.
    forward_distance_m: float = .5
    altitude_distance_m: float = .15
    yaw_amplitude_deg: float = 20.


class Trajectory:
    def __init__(self, config=TrajectoryConfig()):
        self.config = config
        patterns = {"flown": (1., .5, 40., .2, .8), "large": (2., 1., 24., .3, 1.)}
        simple = config.pattern in ("hover", "forward_backward", "yaw", "altitude")
        if config.pattern not in patterns and not simple:
            raise ValueError("Unknown trajectory pattern")
        if not np.isfinite([config.settle_s, config.ramp_s, config.position_gain, config.yaw_gain, config.max_yaw_rate]).all():
            raise ValueError("Nonfinite trajectory configuration")
        if config.settle_s < 0 or config.ramp_s < 0 or config.laps < 1 or int(config.laps) != config.laps:
            raise ValueError("Invalid trajectory timing")
        if min(config.position_gain, config.yaw_gain, config.max_yaw_rate) <= 0:
            raise ValueError("Controller gains and yaw limit must be positive")
        if not np.isfinite([config.simple_duration_s, config.forward_distance_m, config.altitude_distance_m, config.yaw_amplitude_deg]).all():
            raise ValueError("Nonfinite simple trajectory configuration")
        if min(config.simple_duration_s, config.forward_distance_m, config.altitude_distance_m, config.yaw_amplitude_deg) <= 0 or config.yaw_amplitude_deg > 90:
            raise ValueError("Invalid simple trajectory configuration")
        if simple:
            self.ax = config.forward_distance_m if config.pattern == "forward_backward" else 0.
            self.ay = 0.
            self.az = config.altitude_distance_m if config.pattern == "altitude" else 0.
            self.period, self.max_speed = config.simple_duration_s, .3
            peak_rate = 7.5/config.simple_duration_s
            if peak_rate*max(self.ax, self.az) > self.max_speed or (config.pattern == "yaw" and peak_rate*np.deg2rad(config.yaw_amplitude_deg) > config.max_yaw_rate):
                raise ValueError("Simple trajectory duration is too short for its amplitude and speed limit")
        else:
            self.ax, self.ay, self.period, self.az, self.max_speed = patterns[config.pattern]
        if not simple and config.ramp_s > config.laps*self.period:
            raise ValueError("Ramp cannot exceed total lap time")
        self.duration_s = config.settle_s + (config.simple_duration_s if simple else config.laps*self.period + config.ramp_s)
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


    def excursion(self, elapsed_s):
        t = elapsed_s-self.config.settle_s
        duration = self.config.simple_duration_s
        if t <= 0 or t >= duration:
            return 0., 0.
        leg_s = duration/4
        leg = min(int(t/leg_s), 3)
        u = (t-leg*leg_s)/leg_s
        waypoints = (0., 1., 0., -1., 0.)
        delta = waypoints[leg+1]-waypoints[leg]
        position = waypoints[leg]+delta*(10*u**3-15*u**4+6*u**5)
        velocity = delta*(30*u**2-60*u**3+30*u**4)/leg_s
        return position, velocity


    def heading(self, elapsed_s):
        if self.yaw is None or not np.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError("Trajectory not started or invalid elapsed time")
        if self.config.pattern == "yaw":
            offset, rate = self.excursion(elapsed_s)
            amplitude = np.deg2rad(self.config.yaw_amplitude_deg)
            return self.yaw+amplitude*offset, amplitude*rate
        return self.yaw, 0.


    def reference(self, elapsed_s):
        if self.origin is None or not np.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError("Trajectory not started or invalid elapsed time")
        if self.config.pattern in ("hover", "forward_backward", "yaw", "altitude"):
            offset, rate = self.excursion(elapsed_s)
            axis = np.array([self.ax, 0., self.az])
            phase = "settling" if elapsed_s <= self.config.settle_s else ("complete_hold" if elapsed_s >= self.duration_s else "tracking")
            return self.origin+self.rotation@axis*offset, self.rotation@axis*rate, phase
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
        if self.config.pattern == "yaw":
            yaw, yaw_rate = self.heading(elapsed_s)
            measured_yaw = np.arctan2(rotation_enu_flu[1, 0], rotation_enu_flu[0, 0])
            error = np.arctan2(np.sin(yaw-measured_yaw), np.cos(yaw-measured_yaw))
            raw[3] = np.clip(yaw_rate+self.config.yaw_gain*error, -self.config.max_yaw_rate, self.config.max_yaw_rate)
        return np.clip(raw, -np.asarray(limits), limits), reference, velocity, phase
