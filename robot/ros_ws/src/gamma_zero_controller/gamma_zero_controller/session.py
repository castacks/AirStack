import numpy as np

from gamma_zero_controller.ctbr import px4_state_to_policy
from gamma_zero_controller.trajectory import Trajectory, TrajectoryConfig


class FlightSession:
    def __init__(self, policy, config):
        self.policy = policy
        self.config = config
        self.trajectory = Trajectory(TrajectoryConfig(**config.get("trajectory", {})))
        self.low = np.asarray(config["position_min_enu"], dtype=float)
        self.high = np.asarray(config["position_max_enu"], dtype=float)
        self.lever = np.asarray(config["body_origin_to_imu_flu"], dtype=float)
        if any(v.shape != (3,) for v in [self.low, self.high, self.lever]) or not np.isfinite(np.r_[self.low, self.high, self.lever]).all() or np.any(self.low >= self.high):
            raise ValueError("Invalid geofence or calibrated IMU lever arm")
        for key in ["state_timeout_s", "max_tilt_deg", "max_tracking_error_m", "completion_hold_s", "max_tick_gap_s"]:
            if not np.isfinite(config[key]) or config[key] <= 0:
                raise ValueError("Invalid runtime limit: "+key)
        if not 0 < config["state_timeout_s"] <= .1 or not 0 < config["max_tick_gap_s"] <= .1:
            raise ValueError("State/tick timeout must be at most 100 ms")
        self.active = False
        self.fault = "not started"
        self.state = None
        self.source_s = float("-inf")
        self.received_s = float("-inf")
        self.reset_counter = None
        self.previous = np.zeros(4)
        self.started_s = None
        self.last_tick_s = None


    def stop(self, reason):
        self.active = False
        self.fault = reason
        self.previous = np.zeros(4)
        try:
            self.policy.reset_history()
        except (OSError, RuntimeError):
            pass


    def update_state(self, q, position, velocity, rates, source_s, received_s, reset_counter, pose_frame=1, velocity_frame=1):
        try:
            if pose_frame != 1 or velocity_frame != 1:
                raise ValueError("Expected NED pose and world velocity")
            if not np.isfinite([source_s, received_s]).all() or source_s <= self.source_s or received_s < self.received_s:
                raise ValueError("Invalid/nonmonotonic state timestamps")
            state = px4_state_to_policy(q, position, velocity, rates)
            state["linear_velocity_flu"] += np.cross(state["angular_velocity_flu"], self.lever)
            if self.reset_counter is not None and reset_counter != self.reset_counter:
                self.stop("estimator reset; manual restart required")
            self.reset_counter = reset_counter
            self.state, self.source_s, self.received_s = state, source_s, received_s
        except ValueError as exc:
            self.stop(str(exc))
            raise


    def check_state(self, ros_s, monotonic_s):
        if self.state is None or not np.isfinite([ros_s, monotonic_s]).all():
            raise ValueError("Missing state or invalid clock")
        if not -.02 <= ros_s-self.source_s <= self.config["state_timeout_s"] or not 0 <= monotonic_s-self.received_s <= self.config["state_timeout_s"]:
            raise ValueError("Stale state or clocks unsynchronized")
        p = self.state["position_enu"]
        if np.any(p < self.low) or np.any(p > self.high):
            raise ValueError("Vehicle outside configured ENU bounds")
        tilt = np.rad2deg(np.arccos(np.clip(-self.state["projected_gravity_flu"][2], -1., 1.)))
        if tilt > self.config["max_tilt_deg"]:
            raise ValueError("Tilt exceeds runtime limit")


    def start(self, ros_s, monotonic_s):
        if self.active:
            raise ValueError("Session already active")
        self.check_state(ros_s, monotonic_s)
        rotation = self.state["rotation_enu_flu"]
        self.trajectory.start(self.state["position_enu"], np.arctan2(rotation[1, 0], rotation[0, 0]))
        extent = np.abs(self.trajectory.rotation) @ np.array([self.trajectory.ax, .5*self.trajectory.ay, self.trajectory.az])
        margin = self.config["max_tracking_error_m"]
        if np.any(self.trajectory.origin-extent-margin < self.low) or np.any(self.trajectory.origin+extent+margin > self.high):
            raise ValueError("Planned route plus tracking margin exceeds configured bounds")
        self.policy.reset_history()
        self.previous = np.zeros(4)
        self.started_s = self.last_tick_s = monotonic_s
        self.active, self.fault = True, ""


    def step(self, ros_s, monotonic_s):
        if not self.active:
            return None
        try:
            self.check_state(ros_s, monotonic_s)
            if not 0 <= monotonic_s-self.last_tick_s <= self.config["max_tick_gap_s"]:
                raise ValueError("Control deadline missed")
            elapsed = monotonic_s-self.started_s
            if elapsed > self.trajectory.duration_s+self.config["completion_hold_s"]:
                raise ValueError("Completion hold expired; operator handback required")
            goal, reference, velocity, phase = self.trajectory.command(elapsed, self.state["position_enu"],
                self.state["rotation_enu_flu"], self.policy.assets["max_command_velocity"])
            if np.linalg.norm(reference-self.state["position_enu"]) > self.config["max_tracking_error_m"]:
                raise ValueError("Tracking error exceeds runtime limit")
            action, ctbr = self.policy.act(self.state, goal, self.previous)
            self.previous = action.copy()
            self.last_tick_s = monotonic_s
            return {"elapsed_s": elapsed, "source_s": self.source_s, "phase": phase,
                    "position_enu": self.state["position_enu"].tolist(), "reference_enu": reference.tolist(),
                    "reference_velocity_enu": velocity.tolist(), "goal_velocity": goal.tolist(),
                    "raw_action": action.tolist(), "ctbr_si": ctbr.tolist(),
                    "state": {k: v.tolist() for k, v in self.state.items()}}
        except (ValueError, RuntimeError) as exc:
            self.stop(str(exc))
            raise
