from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class BVConfig:
    mass_kg: float = .602
    gravity: float = 9.81
    hover_normalized: float = .15190907567739487
    thrust_limit_n: float = 18.
    rate_limits_flu: tuple = (2.2689280275926285, 2.2689280275926285, 2.6)
    command_timeout_s: float = .1
    state_timeout_s: float = .1
    future_tolerance_s: float = .02


def action_to_ctbr(action, config=BVConfig()):
    action = np.asarray(action, dtype=float)
    if action.shape != (4,) or not np.isfinite(action).all():
        raise ValueError("Expected four finite policy actions")
    hover = config.mass_kg*config.gravity
    physical = action*np.array([.3*hover, 1., 1., 1.]) + np.array([hover, 0., 0., 0.])
    return np.clip(physical, [0., -2.3, -2.3, -2.6], [18., 2.3, 2.3, 2.6])


def ctbr_to_airstack(command, config=BVConfig()):
    command = np.asarray(command, dtype=float)
    if command.shape != (4,) or not np.isfinite(command).all():
        raise ValueError("Expected [collective N, p, q, r rad/s] in body FLU")
    if command[0] < 0:
        raise ValueError("Negative collective is invalid for BV")
    limits = np.asarray(config.rate_limits_flu)
    bounded = np.clip(command, np.r_[0., -limits], np.r_[config.thrust_limit_n, limits])
    normalized = bounded[0] * config.hover_normalized / (config.mass_kg*config.gravity)
    if not 0 <= normalized <= 1:
        raise ValueError("Normalized collective outside [0, 1]")
    return {"rates_flu": bounded[1:].tolist(), "normalized_collective": float(normalized),
            "saturated": bool(np.any(command != bounded))}


def airstack_to_px4(rates_flu, normalized_collective):
    rates = np.asarray(rates_flu, dtype=float)
    if rates.shape != (3,) or not np.isfinite(rates).all() or not np.isfinite(normalized_collective) or not 0 <= normalized_collective <= 1:
        raise ValueError("Invalid AirStack rate/thrust boundary")
    return {"roll": float(rates[0]), "pitch": float(-rates[1]), "yaw": float(-rates[2]),
            "thrust_body": [0., 0., -float(normalized_collective)],
            "offboard": {"position": False, "velocity": False, "acceleration": False, "attitude": False, "body_rate": True}}


def px4_state_to_policy(q_wxyz, position_ned, velocity_ned, rates_frd):
    q = np.asarray(q_wxyz, dtype=float)
    position, velocity, rates = [np.asarray(v, dtype=float) for v in [position_ned, velocity_ned, rates_frd]]
    if q.shape != (4,) or any(v.shape != (3,) for v in [position, velocity, rates]) or not np.isfinite(np.r_[q, position, velocity, rates]).all():
        raise ValueError("Invalid vehicle state")
    if abs(np.linalg.norm(q)-1.) > .01:
        raise ValueError("Invalid attitude quaternion norm")
    w, x, y, z = q/np.linalg.norm(q)
    rotation_ned_frd = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    enu_from_ned = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    flu_from_frd = np.diag([1., -1., -1.])
    rotation_enu_flu = enu_from_ned @ rotation_ned_frd @ flu_from_frd
    return {"position_enu": enu_from_ned @ position,
            "rotation_enu_flu": rotation_enu_flu,
            "linear_velocity_flu": flu_from_frd @ rotation_ned_frd.T @ velocity,
            "angular_velocity_flu": flu_from_frd @ rates,
            "projected_gravity_flu": rotation_enu_flu.T @ np.array([0., 0., -1.])}


@dataclass
class CommandGate:
    config: BVConfig = field(default_factory=BVConfig)
    enabled: bool = False
    command: object = None
    command_stamp_s: float = float("-inf")
    command_received_s: float = float("-inf")
    state_stamp_s: float = float("-inf")
    state_received_s: float = float("-inf")
    fault: str = "disabled"


    def trip(self, reason):
        self.enabled = False
        self.command = None
        self.fault = reason


    def update_command(self, command, source_s, received_s):
        try:
            mapped = ctbr_to_airstack(command, self.config)
            if not np.isfinite([source_s, received_s]).all() or source_s <= self.command_stamp_s or received_s < self.command_received_s:
                raise ValueError("Nonmonotonic or invalid command timestamp")
            self.command, self.command_stamp_s, self.command_received_s = mapped, source_s, received_s
        except ValueError as exc:
            self.trip(str(exc))
            raise


    def update_state(self, q_wxyz, position_ned, velocity_ned, rates_frd, source_s, received_s, pose_frame=1, velocity_frame=1):
        try:
            if pose_frame != 1 or velocity_frame != 1:
                raise ValueError("Expected PX4 NED pose and NED world velocity")
            state = px4_state_to_policy(q_wxyz, position_ned, velocity_ned, rates_frd)
            if not np.isfinite([source_s, received_s]).all() or source_s <= self.state_stamp_s or received_s < self.state_received_s:
                raise ValueError("Nonmonotonic or invalid state timestamp")
            self.state_stamp_s, self.state_received_s = source_s, received_s
            return state
        except ValueError as exc:
            self.trip(str(exc))
            raise


    def check(self, now_ros_s, now_monotonic_s, exclusive=True):
        if self.command is None:
            return "missing command"
        if not exclusive:
            return "conflicting or missing command owner"
        if not np.isfinite([now_ros_s, now_monotonic_s]).all():
            return "invalid clock"
        for name, source, received, timeout in [
                ("command", self.command_stamp_s, self.command_received_s, self.config.command_timeout_s),
                ("state", self.state_stamp_s, self.state_received_s, self.config.state_timeout_s)]:
            source_age = now_ros_s-source
            receipt_age = now_monotonic_s-received
            if not -self.config.future_tolerance_s <= source_age <= timeout or not 0 <= receipt_age <= timeout:
                return name+" stale or clocks unsynchronized"
        return None


    def enable(self, now_ros_s, now_monotonic_s, exclusive=True):
        reason = self.check(now_ros_s, now_monotonic_s, exclusive)
        if reason:
            self.trip(reason)
            return False
        self.enabled, self.fault = True, ""
        return True


    def output(self, now_ros_s, now_monotonic_s, exclusive=True):
        if not self.enabled:
            return None
        reason = self.check(now_ros_s, now_monotonic_s, exclusive)
        if reason:
            self.trip(reason)
            return None
        return self.command
