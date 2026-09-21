"""Typed RRM output for AirStack's public drone task-action boundary.

This module deliberately describes only task proposals.  It does not import ROS or
provide a PX4, MAVROS, service, publisher, or trajectory-control interface.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, model_validator


class DroneTaskKind(str, Enum):
    TAKEOFF = "TAKEOFF"
    NAVIGATE = "NAVIGATE"
    LAND = "LAND"
    EXPLORE = "EXPLORE"


class DroneOutcomeVerdict(str, Enum):
    """Whether independent observations corroborate one task-server result."""

    VERIFIED = "VERIFIED"
    MISMATCH = "MISMATCH"
    UNCONFIRMED = "UNCONFIRMED"


class MapWaypoint(BaseModel):
    """One robot-local map-frame waypoint; orientation defaults to identity."""

    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float

    @model_validator(mode="after")
    def validate_coordinates(self) -> "MapWaypoint":
        if not all(math.isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("waypoint coordinates must be finite")
        return self


class DroneTaskProposal(BaseModel):
    """One RRM proposal mapped to the exact AirStack task-action goal schema."""

    model_config = ConfigDict(frozen=True)

    task_id: str
    action_id: str
    robot_name: str = "robot_1"
    kind: DroneTaskKind
    target_altitude_m: float | None = None
    velocity_m_s: float | None = None
    frame_id: str | None = None
    waypoints: tuple[MapWaypoint, ...] = ()
    goal_tolerance_m: float | None = None
    min_altitude_agl_m: float | None = None
    max_altitude_agl_m: float | None = None
    min_flight_speed_m_s: float | None = None
    max_flight_speed_m_s: float | None = None
    time_limit_s: float | None = None

    @model_validator(mode="after")
    def validate_proposal(self) -> "DroneTaskProposal":
        for name in ("task_id", "action_id", "robot_name"):
            if not getattr(self, name).strip():
                raise ValueError(f"{name} is required")
        numeric = (self.target_altitude_m, self.velocity_m_s, self.goal_tolerance_m,
                   self.min_altitude_agl_m, self.max_altitude_agl_m,
                   self.min_flight_speed_m_s, self.max_flight_speed_m_s,
                   self.time_limit_s)
        if any(value is not None and not math.isfinite(value) for value in numeric):
            raise ValueError("task parameters must be finite")
        if self.kind is DroneTaskKind.TAKEOFF:
            if self.target_altitude_m is None or self.target_altitude_m <= 0:
                raise ValueError("takeoff target_altitude_m must be positive")
            if self.velocity_m_s is None or self.velocity_m_s <= 0:
                raise ValueError("takeoff velocity_m_s must be positive")
            if (self.waypoints or self.frame_id is not None or self.goal_tolerance_m is not None
                    or any(value is not None for value in (
                        self.min_altitude_agl_m, self.max_altitude_agl_m,
                        self.min_flight_speed_m_s, self.max_flight_speed_m_s,
                        self.time_limit_s))):
                raise ValueError("takeoff proposal contains navigation fields")
        elif self.kind is DroneTaskKind.NAVIGATE:
            if self.frame_id != "map" or not self.waypoints:
                raise ValueError("navigate requires one or more robot-local map waypoints")
            if self.goal_tolerance_m is None or self.goal_tolerance_m <= 0:
                raise ValueError("navigate goal_tolerance_m must be positive")
            if (self.target_altitude_m is not None or self.velocity_m_s is not None
                    or any(value is not None for value in (
                        self.min_altitude_agl_m, self.max_altitude_agl_m,
                        self.min_flight_speed_m_s, self.max_flight_speed_m_s,
                        self.time_limit_s))):
                raise ValueError("navigate proposal contains takeoff/land fields")
        elif self.kind is DroneTaskKind.LAND:
            if self.velocity_m_s is None or self.velocity_m_s < 0:
                raise ValueError("land velocity_m_s must be nonnegative")
            if (self.target_altitude_m is not None or self.waypoints or self.frame_id is not None
                    or self.goal_tolerance_m is not None
                    or any(value is not None for value in (
                        self.min_altitude_agl_m, self.max_altitude_agl_m,
                        self.min_flight_speed_m_s, self.max_flight_speed_m_s,
                        self.time_limit_s))):
                raise ValueError("land proposal contains takeoff/navigation fields")
        else:
            values = (self.min_altitude_agl_m, self.max_altitude_agl_m,
                      self.min_flight_speed_m_s, self.max_flight_speed_m_s,
                      self.time_limit_s)
            if any(value is None for value in values):
                raise ValueError("explore proposal requires altitude, speed, and time limits")
            if (self.min_altitude_agl_m <= 0
                    or self.max_altitude_agl_m < self.min_altitude_agl_m
                    or self.min_flight_speed_m_s <= 0
                    or self.max_flight_speed_m_s < self.min_flight_speed_m_s
                    or self.time_limit_s <= 0):
                raise ValueError("explore proposal limits are invalid")
            if (self.target_altitude_m is not None or self.velocity_m_s is not None
                    or self.waypoints or self.frame_id is not None
                    or self.goal_tolerance_m is not None):
                raise ValueError("explore proposal contains fields from another task kind")
        return self

    @property
    def action_name(self) -> str:
        suffix = "exploration" if self.kind is DroneTaskKind.EXPLORE else self.kind.value.lower()
        return f"/{self.robot_name}/tasks/{suffix}"

    def preview(self) -> dict[str, Any]:
        """The action endpoint and goal fields an execution adapter would send."""
        if self.kind is DroneTaskKind.TAKEOFF:
            goal: dict[str, Any] = {
                "target_altitude_m": self.target_altitude_m,
                "velocity_m_s": self.velocity_m_s,
            }
        elif self.kind is DroneTaskKind.LAND:
            goal = {"velocity_m_s": self.velocity_m_s}
        elif self.kind is DroneTaskKind.EXPLORE:
            goal = {
                "search_bounds": [],
                "min_altitude_agl": self.min_altitude_agl_m,
                "max_altitude_agl": self.max_altitude_agl_m,
                "min_flight_speed": self.min_flight_speed_m_s,
                "max_flight_speed": self.max_flight_speed_m_s,
                "time_limit_sec": self.time_limit_s,
            }
        else:
            goal = {
                "global_plan": {
                    "header": {"frame_id": self.frame_id},
                    "poses": [
                        {"pose": {"position": point.model_dump(),
                                  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
                        for point in self.waypoints
                    ],
                },
                "goal_tolerance_m": self.goal_tolerance_m,
            }
        return {
            "task_id": self.task_id,
            "action_id": self.action_id,
            "action_name": self.action_name,
            "action_type": ("task_msgs/action/ExplorationTask"
                            if self.kind is DroneTaskKind.EXPLORE
                            else f"task_msgs/action/{self.kind.value.title()}Task"),
            "goal": goal,
            "execution_requested": False,
        }


class OdometryEvidence(BaseModel):
    """One independently received map-frame odometry sample.

    This is a transport-neutral record: ROS conversion belongs in the optional runner.
    The received time, rather than a simulator clock alone, is used for freshness.
    """

    model_config = ConfigDict(frozen=True)

    received_monotonic_s: float = Field(ge=0.0)
    source_stamp_ns: int = Field(ge=0)
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    z: float

    @model_validator(mode="after")
    def validate_observation(self) -> "OdometryEvidence":
        if self.frame_id != "map" or self.child_frame_id != "base_link":
            raise ValueError("outcome odometry must be map -> base_link")
        if not all(math.isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("outcome odometry coordinates must be finite")
        return self


class VehicleStateEvidence(BaseModel):
    """One independently received MAVROS vehicle state sample."""

    model_config = ConfigDict(frozen=True)

    received_monotonic_s: float = Field(ge=0.0)
    connected: bool
    armed: bool


class DroneOutcomeVerification(BaseModel):
    """Causal task-result record with a conservative independent-effect verdict."""

    model_config = ConfigDict(frozen=True)

    task_id: str
    action_id: str
    kind: DroneTaskKind
    action_success: bool
    action_message: str
    dispatch_monotonic_s: float
    completion_monotonic_s: float
    verdict: DroneOutcomeVerdict
    reasons: tuple[str, ...]
    pre_odometry: OdometryEvidence | None
    post_odometry: OdometryEvidence | None
    post_vehicle_state: VehicleStateEvidence | None


def verify_drone_outcome(
    proposal: DroneTaskProposal,
    *,
    action_success: bool,
    action_message: str,
    pre_odometry: OdometryEvidence | None,
    post_odometry: OdometryEvidence | None,
    post_vehicle_state: VehicleStateEvidence | None,
    dispatch_monotonic_s: float,
    now_monotonic_s: float,
    max_observation_age_s: float = 1.0,
    takeoff_acceptance_distance_m: float = 0.3,
    takeoff_max_horizontal_displacement_m: float = 0.3,
    landing_max_altitude_m: float = 0.3,
) -> DroneOutcomeVerification:
    """Evaluate a task outcome without granting or changing execution authority.

    ``VERIFIED`` requires a successful task result plus fresh independent evidence. A
    missing/stale sample is never inferred from a result. Navigation additionally
    requires the independently observed endpoint within the proposal's tolerance.
    """
    parameters = (dispatch_monotonic_s, now_monotonic_s, max_observation_age_s,
                  takeoff_acceptance_distance_m, takeoff_max_horizontal_displacement_m,
                  landing_max_altitude_m)
    if not all(math.isfinite(value) for value in parameters):
        raise ValueError("outcome verification parameters must be finite")
    if dispatch_monotonic_s < 0 or now_monotonic_s < dispatch_monotonic_s or max_observation_age_s <= 0:
        raise ValueError("outcome verification time bounds must be positive")
    if (takeoff_acceptance_distance_m <= 0
            or takeoff_max_horizontal_displacement_m <= 0
            or landing_max_altitude_m < 0):
        raise ValueError("outcome verification distance bounds are invalid")

    reasons: list[str] = []
    if not action_success:
        reasons.append("task_result_unsuccessful")
    if pre_odometry is None:
        reasons.append("pre_odometry_missing")
    elif _is_stale(pre_odometry.received_monotonic_s, dispatch_monotonic_s,
                   max_observation_age_s):
        reasons.append("pre_odometry_stale")
    if post_odometry is None:
        reasons.append("post_odometry_missing")
    elif _is_stale(post_odometry.received_monotonic_s, now_monotonic_s, max_observation_age_s):
        reasons.append("post_odometry_stale")
    if post_odometry is not None:
        if post_odometry.received_monotonic_s <= dispatch_monotonic_s:
            reasons.append("post_odometry_predates_dispatch")
        if pre_odometry is not None and post_odometry.source_stamp_ns <= pre_odometry.source_stamp_ns:
            reasons.append("odometry_source_clock_not_advanced")

    if proposal.kind is DroneTaskKind.NAVIGATE:
        if post_odometry is not None:
            target = proposal.waypoints[-1]
            distance = math.dist((post_odometry.x, post_odometry.y, post_odometry.z),
                                 (target.x, target.y, target.z))
            if distance > proposal.goal_tolerance_m:
                reasons.append("navigation_endpoint_mismatch")
    elif proposal.kind is DroneTaskKind.TAKEOFF and post_odometry is not None:
        if post_vehicle_state is None:
            reasons.append("post_vehicle_state_missing")
        elif _is_stale(post_vehicle_state.received_monotonic_s, now_monotonic_s,
                       max_observation_age_s):
            reasons.append("post_vehicle_state_stale")
        elif not post_vehicle_state.connected:
            reasons.append("post_vehicle_disconnected")
        elif not post_vehicle_state.armed:
            reasons.append("post_vehicle_not_armed")
        altitude_error = abs(post_odometry.z - proposal.target_altitude_m)
        if altitude_error > takeoff_acceptance_distance_m:
            reasons.append("takeoff_altitude_mismatch")
        if pre_odometry is not None:
            horizontal_displacement = math.hypot(
                post_odometry.x - pre_odometry.x,
                post_odometry.y - pre_odometry.y,
            )
            if horizontal_displacement > takeoff_max_horizontal_displacement_m:
                reasons.append("takeoff_horizontal_displacement_mismatch")
    elif proposal.kind is DroneTaskKind.LAND:
        if post_vehicle_state is None:
            reasons.append("post_vehicle_state_missing")
        elif _is_stale(post_vehicle_state.received_monotonic_s, now_monotonic_s,
                       max_observation_age_s):
            reasons.append("post_vehicle_state_stale")
        elif not post_vehicle_state.connected:
            reasons.append("post_vehicle_disconnected")
        elif post_vehicle_state.armed:
            reasons.append("post_vehicle_still_armed")
        if post_odometry is not None and post_odometry.z > landing_max_altitude_m:
            reasons.append("landing_altitude_mismatch")

    if not reasons:
        verdict = DroneOutcomeVerdict.VERIFIED
    elif not action_success:
        verdict = DroneOutcomeVerdict.UNCONFIRMED
    elif any(reason.endswith("_mismatch") for reason in reasons):
        verdict = DroneOutcomeVerdict.MISMATCH
    else:
        verdict = DroneOutcomeVerdict.UNCONFIRMED
    return DroneOutcomeVerification(
        task_id=proposal.task_id,
        action_id=proposal.action_id,
        kind=proposal.kind,
        action_success=action_success,
        action_message=action_message,
        dispatch_monotonic_s=dispatch_monotonic_s,
        completion_monotonic_s=now_monotonic_s,
        verdict=verdict,
        reasons=tuple(reasons),
        pre_odometry=pre_odometry,
        post_odometry=post_odometry,
        post_vehicle_state=post_vehicle_state,
    )


def _is_stale(received_monotonic_s: float, now_monotonic_s: float,
              max_observation_age_s: float) -> bool:
    return received_monotonic_s > now_monotonic_s or (
        now_monotonic_s - received_monotonic_s > max_observation_age_s
    )
