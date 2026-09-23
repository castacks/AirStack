"""Ground natural language only into discovered public AirStack task actions.

The compiler is deterministic and auditable. It does not claim that a language model
or a camera image measured a value: each numeric parameter records where it came from,
and ambiguity that would materially change motion produces one targeted question.
"""
from __future__ import annotations

from enum import Enum
import math
import re
from typing import Iterable

from pydantic import BaseModel, ConfigDict, Field, model_validator

from rrm.airstack_drone import (
    DroneTaskKind,
    DroneTaskProposal,
    MapWaypoint,
    SearchBoundPoint,
)


TAKEOFF_ACTION = "task_msgs/action/TakeoffTask"
LAND_ACTION = "task_msgs/action/LandTask"
NAVIGATE_ACTION = "task_msgs/action/NavigateTask"
EXPLORATION_ACTION = "task_msgs/action/ExplorationTask"


class ParameterSource(str, Enum):
    OPERATOR_EXPLICIT = "OPERATOR_EXPLICIT"
    SEMANTIC_INTERPRETATION = "SEMANTIC_INTERPRETATION"
    ENVIRONMENT_OBSERVATION = "ENVIRONMENT_OBSERVATION"
    VEHICLE_ENVELOPE = "VEHICLE_ENVELOPE"
    POLICY_DEFAULT = "POLICY_DEFAULT"


class ParameterGrounding(BaseModel):
    """Auditable reason for one command parameter."""

    model_config = ConfigDict(frozen=True)

    action_id: str
    parameter: str
    value: float | list[list[float]]
    unit: str
    source: ParameterSource
    evidence: str
    confidence: float = Field(ge=0.0, le=1.0)
    requested_value: float | None = None


class CommandEnvironment(BaseModel):
    """Read-only environment facts available when the command is compiled."""

    model_config = ConfigDict(frozen=True)

    active_scene: str | None = None
    current_position: tuple[float, float, float] | None = None
    yaw_rad: float | None = None
    map_fresh: bool = False
    map_point_count: int | None = Field(default=None, ge=0)
    map_bounds_xy: tuple[float, float, float, float] | None = None

    @model_validator(mode="after")
    def validate_finite_environment(self) -> "CommandEnvironment":
        values = tuple(self.current_position or ()) + (() if self.yaw_rad is None else (self.yaw_rad,))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("environment pose and heading must be finite")
        if self.map_bounds_xy is not None:
            min_x, max_x, min_y, max_y = self.map_bounds_xy
            if (not all(math.isfinite(value) for value in self.map_bounds_xy)
                    or min_x >= max_x or min_y >= max_y):
                raise ValueError("map_bounds_xy must be finite ordered bounds")
        return self


class VehicleEnvelope(BaseModel):
    """Versioned conservative task-level envelope; this is not a controller model."""

    model_config = ConfigDict(frozen=True)

    revision: str = "airstack-office-conservative-v1"
    min_takeoff_altitude_m: float = 0.5
    max_takeoff_altitude_m: float = 3.0
    default_takeoff_altitude_m: float = 1.5
    default_takeoff_speed_m_s: float = 0.5
    min_exploration_duration_s: float = 5.0
    max_exploration_duration_s: float = 900.0
    default_exploration_duration_s: float = 30.0
    default_exploration_radius_m: float = 3.0
    min_exploration_altitude_agl_m: float = 1.0
    max_exploration_altitude_agl_m: float = 3.0
    min_exploration_speed_m_s: float = 0.5
    max_exploration_speed_m_s: float = 2.0
    default_goal_tolerance_m: float = 0.5
    default_landing_speed_m_s: float = 0.5

    @model_validator(mode="after")
    def validate_envelope(self) -> "VehicleEnvelope":
        numeric = tuple(
            value for name, value in self.model_dump().items() if name != "revision"
        )
        if (not self.revision.strip() or not all(math.isfinite(value) and value > 0
                                                for value in numeric)):
            raise ValueError("vehicle envelope values must be finite and positive")
        if not (self.min_takeoff_altitude_m <= self.default_takeoff_altitude_m
                <= self.max_takeoff_altitude_m):
            raise ValueError("default takeoff altitude must be inside the envelope")
        if not (self.min_exploration_duration_s <= self.default_exploration_duration_s
                <= self.max_exploration_duration_s):
            raise ValueError("default exploration duration must be inside the envelope")
        if self.min_exploration_altitude_agl_m > self.max_exploration_altitude_agl_m:
            raise ValueError("exploration altitude envelope is inverted")
        if self.min_exploration_speed_m_s > self.max_exploration_speed_m_s:
            raise ValueError("exploration speed envelope is inverted")
        return self


class GroundedCommandPlan(BaseModel):
    """Typed actions plus the evidence and assumptions used to parameterize them."""

    model_config = ConfigDict(frozen=True)

    schema_version: str = "rrm-grounded-command/v1"
    objective: str
    actions: tuple[DroneTaskProposal, ...]
    parameter_grounding: tuple[ParameterGrounding, ...]
    assumptions: tuple[str, ...]
    environment: CommandEnvironment
    vehicle_envelope_revision: str


class CommandClarificationRequired(ValueError):
    """The command cannot be safely grounded without one operator answer."""

    def __init__(self, question: str):
        self.question = question
        super().__init__(f"Clarification required: {question}")


def ground_command(
    objective: str,
    *,
    task_id: str,
    robot_name: str,
    action_servers: Iterable[str],
    airborne: bool,
    environment: CommandEnvironment | None = None,
    envelope: VehicleEnvelope | None = None,
) -> GroundedCommandPlan:
    """Compile one command and retain provenance for every generated task parameter."""
    if not isinstance(objective, str) or not objective.strip():
        raise ValueError("Movement command is required.")
    text = objective.strip().lower()
    available = frozenset(action_servers)
    environment = environment or CommandEnvironment()
    envelope = envelope or VehicleEnvelope()
    proposals: list[DroneTaskProposal] = []
    grounding: list[ParameterGrounding] = []
    assumptions: list[str] = []

    wants_takeoff = bool(re.search(r"\b(take[ -]?off|launch|ascend)\b", text))
    wants_land = bool(re.search(r"\b(land|touch[ -]?down)\b", text))
    wants_explore = bool(re.search(r"\b(explore|survey|roam|map the|move around)\b", text))
    wants_return = bool(re.search(
        r"\b(come back|go back|return(?:\s+(?:home|to (?:the )?start))?)\b", text
    ))
    coordinates = _coordinate_path(text)
    takeoff_altitude, altitude_source, altitude_evidence, requested_altitude = (
        _takeoff_altitude(text, envelope)
    )
    relative_origin = environment.current_position
    if not airborne and relative_origin is not None:
        relative_origin = (
            relative_origin[0], relative_origin[1],
            max(relative_origin[2], takeoff_altitude),
        )
    relative = _relative_destination(text, relative_origin, environment.yaw_rad)
    if relative is not None:
        if coordinates:
            raise CommandClarificationRequired(
                "Should I follow the map coordinates or the relative movement?"
            )
        coordinates = (relative,)

    if (re.search(r"\b(?:fly|go|move)(?:\s+over)?\s+there\b", text)
            and not coordinates and not wants_explore):
        raise CommandClarificationRequired(
            "What map coordinate or relative direction should I use for ‘there’?"
        )
    if wants_return and environment.current_position is None:
        raise CommandClarificationRequired(
            "Can I capture a fresh map-frame start position before returning to it?"
        )
    if not any((wants_takeoff, wants_land, wants_explore, wants_return, coordinates)):
        raise ValueError(
            "Unsupported movement command. Use take off, land, explore/survey, a map "
            "goal, or a measured relative movement."
        )

    needs_airborne = wants_explore or wants_return or bool(coordinates)
    if (wants_takeoff or (needs_airborne and not airborne)) and not airborne:
        _require(available, TAKEOFF_ACTION, "takeoff")
        action_id = f"takeoff-{len(proposals)}"
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=action_id, robot_name=robot_name,
            kind=DroneTaskKind.TAKEOFF, target_altitude_m=takeoff_altitude,
            velocity_m_s=envelope.default_takeoff_speed_m_s,
        ))
        grounding.extend((
            ParameterGrounding(
                action_id=action_id, parameter="target_altitude_m",
                value=takeoff_altitude, unit="m", source=altitude_source,
                evidence=altitude_evidence, confidence=0.95,
                requested_value=requested_altitude,
            ),
            ParameterGrounding(
                action_id=action_id, parameter="velocity_m_s",
                value=envelope.default_takeoff_speed_m_s, unit="m/s",
                source=ParameterSource.VEHICLE_ENVELOPE,
                evidence=f"conservative task envelope {envelope.revision}", confidence=1.0,
            ),
        ))

    if coordinates:
        _require(available, NAVIGATE_ACTION, "navigation")
        action_id = f"navigate-{len(proposals)}"
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=action_id, robot_name=robot_name,
            kind=DroneTaskKind.NAVIGATE, frame_id="map",
            waypoints=tuple(MapWaypoint(x=x, y=y, z=z) for x, y, z in coordinates),
            goal_tolerance_m=envelope.default_goal_tolerance_m,
        ))
        coordinate_source = (ParameterSource.ENVIRONMENT_OBSERVATION
                             if relative is not None else ParameterSource.OPERATOR_EXPLICIT)
        coordinate_evidence = (
            "operator distance applied to the fresh map-frame pose and heading"
            if relative is not None else "map coordinates stated by the operator"
        )
        grounding.extend((
            ParameterGrounding(
                action_id=action_id, parameter="waypoints",
                value=[[x, y, z] for x, y, z in coordinates], unit="m",
                source=coordinate_source, evidence=coordinate_evidence, confidence=1.0,
            ),
            ParameterGrounding(
                action_id=action_id, parameter="goal_tolerance_m",
                value=envelope.default_goal_tolerance_m, unit="m",
                source=ParameterSource.VEHICLE_ENVELOPE,
                evidence=f"conservative task envelope {envelope.revision}", confidence=1.0,
            ),
        ))

    if wants_explore:
        _require(available, EXPLORATION_ACTION, "global exploration planning")
        duration, duration_source, duration_evidence, requested_duration = (
            _exploration_duration(text, envelope, assumptions)
        )
        search_bounds, bounds_source, bounds_evidence = _exploration_bounds(
            environment, envelope, assumptions
        )
        action_id = f"explore-{len(proposals)}"
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=action_id, robot_name=robot_name,
            kind=DroneTaskKind.EXPLORE,
            min_altitude_agl_m=envelope.min_exploration_altitude_agl_m,
            max_altitude_agl_m=envelope.max_exploration_altitude_agl_m,
            min_flight_speed_m_s=envelope.min_exploration_speed_m_s,
            max_flight_speed_m_s=envelope.max_exploration_speed_m_s,
            time_limit_s=duration, search_bounds=search_bounds,
        ))
        grounding.append(ParameterGrounding(
            action_id=action_id, parameter="time_limit_s", value=duration, unit="s",
            source=duration_source, evidence=duration_evidence, confidence=0.95,
            requested_value=requested_duration,
        ))
        if search_bounds:
            grounding.append(ParameterGrounding(
                action_id=action_id, parameter="search_bounds",
                value=[[point.x, point.y] for point in search_bounds], unit="map m",
                source=bounds_source, evidence=bounds_evidence, confidence=0.85,
            ))
        for parameter, value, unit in (
            ("min_altitude_agl_m", envelope.min_exploration_altitude_agl_m, "m AGL"),
            ("max_altitude_agl_m", envelope.max_exploration_altitude_agl_m, "m AGL"),
            ("min_flight_speed_m_s", envelope.min_exploration_speed_m_s, "m/s"),
            ("max_flight_speed_m_s", envelope.max_exploration_speed_m_s, "m/s"),
        ):
            grounding.append(ParameterGrounding(
                action_id=action_id, parameter=parameter, value=value, unit=unit,
                source=ParameterSource.VEHICLE_ENVELOPE,
                evidence=f"conservative task envelope {envelope.revision}", confidence=1.0,
            ))

    if wants_return:
        _require(available, NAVIGATE_ACTION, "return-to-start navigation")
        start = environment.current_position
        assert start is not None
        return_z = max(start[2], takeoff_altitude)
        action_id = f"return-{len(proposals)}"
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=action_id, robot_name=robot_name,
            kind=DroneTaskKind.NAVIGATE, frame_id="map",
            waypoints=(MapWaypoint(x=start[0], y=start[1], z=return_z),),
            goal_tolerance_m=envelope.default_goal_tolerance_m,
        ))
        grounding.extend((
            ParameterGrounding(
                action_id=action_id, parameter="waypoints",
                value=[[start[0], start[1], return_z]], unit="map m",
                source=ParameterSource.ENVIRONMENT_OBSERVATION,
                evidence="fresh command-start map pose; altitude kept at the safe takeoff level",
                confidence=1.0,
            ),
            ParameterGrounding(
                action_id=action_id, parameter="goal_tolerance_m",
                value=envelope.default_goal_tolerance_m, unit="m",
                source=ParameterSource.VEHICLE_ENVELOPE,
                evidence=f"conservative task envelope {envelope.revision}", confidence=1.0,
            ),
        ))

    if wants_land and (airborne or proposals):
        _require(available, LAND_ACTION, "landing")
        action_id = f"land-{len(proposals)}"
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=action_id, robot_name=robot_name,
            kind=DroneTaskKind.LAND, velocity_m_s=envelope.default_landing_speed_m_s,
        ))
        grounding.append(ParameterGrounding(
            action_id=action_id, parameter="velocity_m_s",
            value=envelope.default_landing_speed_m_s, unit="m/s",
            source=ParameterSource.VEHICLE_ENVELOPE,
            evidence=f"conservative task envelope {envelope.revision}", confidence=1.0,
        ))
    if not proposals:
        raise ValueError("Command is already satisfied by the observed airborne state.")
    if any(proposal.kind is DroneTaskKind.TAKEOFF for proposal in proposals):
        _require(available, LAND_ACTION, "safe recovery landing")
    return GroundedCommandPlan(
        objective=objective.strip(), actions=tuple(proposals),
        parameter_grounding=tuple(grounding), assumptions=tuple(assumptions),
        environment=environment, vehicle_envelope_revision=envelope.revision,
    )


def plan_command(objective: str, *, task_id: str, robot_name: str,
                 action_servers: Iterable[str], airborne: bool,
                 current_position: tuple[float, float, float] | None = None,
                 yaw_rad: float | None = None) -> tuple[DroneTaskProposal, ...]:
    """Backward-compatible proposal-only wrapper around :func:`ground_command`."""
    return ground_command(
        objective, task_id=task_id, robot_name=robot_name,
        action_servers=action_servers, airborne=airborne,
        environment=CommandEnvironment(current_position=current_position, yaw_rad=yaw_rad),
    ).actions


def takeoff_recovery_action(proposals: Iterable[DroneTaskProposal]) -> DroneTaskProposal | None:
    """Predeclare the public LandTask used after a verified takeoff mismatch."""
    takeoff = next((item for item in proposals if item.kind is DroneTaskKind.TAKEOFF), None)
    if takeoff is None:
        return None
    return DroneTaskProposal(
        task_id=takeoff.task_id, action_id=f"{takeoff.action_id}-recovery-land",
        robot_name=takeoff.robot_name, kind=DroneTaskKind.LAND, velocity_m_s=0.5,
    )


def _require(available: frozenset[str], action_type: str, label: str) -> None:
    if action_type not in available:
        raise ValueError(f"The active AirStack configuration has no {label} task executor.")


def _takeoff_altitude(
    text: str, envelope: VehicleEnvelope,
) -> tuple[float, ParameterSource, str, float | None]:
    match = re.search(
        r"(?:take[ -]?off|launch|ascend)(?:\s+to)?\s+"
        r"(-?\d+(?:\.\d+)?)\s*(?:m|meter|meters)?\b", text,
    )
    if match:
        requested = float(match.group(1))
        if not envelope.min_takeoff_altitude_m <= requested <= envelope.max_takeoff_altitude_m:
            raise CommandClarificationRequired(
                f"What takeoff altitude between {envelope.min_takeoff_altitude_m:g} and "
                f"{envelope.max_takeoff_altitude_m:g} meters should I use?"
            )
        return requested, ParameterSource.OPERATOR_EXPLICIT, (
            "takeoff altitude stated by the operator"
        ), requested
    if re.search(r"\b(?:low|low-altitude)\b", text):
        value = max(envelope.min_takeoff_altitude_m, 1.0)
        return value, ParameterSource.SEMANTIC_INTERPRETATION, (
            f"‘low’ resolved conservatively within {envelope.revision}"
        ), None
    return envelope.default_takeoff_altitude_m, ParameterSource.VEHICLE_ENVELOPE, (
        f"default takeoff altitude from {envelope.revision}"
    ), None


def _exploration_duration(
    text: str, envelope: VehicleEnvelope, assumptions: list[str],
) -> tuple[float, ParameterSource, str, float | None]:
    quantity = r"(?:a\s+|an\s+)?(\d+(?:\.\d+)?|one|two|three|couple|few|several)"
    matches = re.findall(
        rf"(?:for|during)\s+{quantity}(?:\s+of)?\s*(seconds?|secs?|minutes?|mins?)?",
        text,
    )
    if len(matches) > 1:
        raise CommandClarificationRequired("Which single exploration duration should I use?")
    if matches:
        token, unit = matches[0]
        words = {"one": 1.0, "two": 2.0, "three": 3.0,
                 "couple": 2.0, "few": 3.0, "several": 5.0}
        requested = (float(token) if re.fullmatch(r"\d+(?:\.\d+)?", token)
                     else words[token])
        if unit.startswith(("minute", "min")):
            requested *= 60.0
        explicit = bool(re.fullmatch(r"\d+(?:\.\d+)?", token))
        if explicit and not (envelope.min_exploration_duration_s
                             <= requested <= envelope.max_exploration_duration_s):
            raise CommandClarificationRequired(
                f"What exploration duration between {envelope.min_exploration_duration_s:g} "
                f"and {envelope.max_exploration_duration_s:g} seconds should I use?"
            )
        value = min(max(requested, envelope.min_exploration_duration_s),
                    envelope.max_exploration_duration_s)
        if value != requested:
            assumptions.append(
                f"Interpreted ‘{token}’ as {requested:g} seconds and raised it to the "
                f"supported minimum {value:g} seconds."
            )
            return value, ParameterSource.VEHICLE_ENVELOPE, (
                f"semantic duration {requested:g} s bounded by {envelope.revision}"
            ), requested
        return value, (ParameterSource.OPERATOR_EXPLICIT if explicit
                       else ParameterSource.SEMANTIC_INTERPRETATION), (
            "numeric duration stated by the operator" if explicit
            else f"quantifier ‘{token}’ interpreted as {requested:g} seconds"
        ), requested
    if re.search(r"\bbrief(?:ly)?\b", text):
        return 10.0, ParameterSource.SEMANTIC_INTERPRETATION, (
            "‘briefly’ resolved to the conservative short-duration policy"
        ), None
    if re.search(r"\b(?:a while|some time)\b", text):
        assumptions.append("Interpreted the vague duration as the conservative 30-second policy.")
        return envelope.default_exploration_duration_s, ParameterSource.POLICY_DEFAULT, (
            f"vague duration resolved by {envelope.revision}"
        ), None
    assumptions.append(
        f"No duration was stated; using the {envelope.default_exploration_duration_s:g}-second "
        "conservative exploration policy."
    )
    return envelope.default_exploration_duration_s, ParameterSource.POLICY_DEFAULT, (
        f"default exploration duration from {envelope.revision}"
    ), None


def _exploration_bounds(
    environment: CommandEnvironment, envelope: VehicleEnvelope, assumptions: list[str],
) -> tuple[tuple[SearchBoundPoint, ...], ParameterSource, str]:
    if environment.current_position is None:
        assumptions.append("No fresh start pose was available, so exploration bounds are unbounded.")
        return (), ParameterSource.POLICY_DEFAULT, "no fresh map-frame start pose"
    x, y, _ = environment.current_position
    radius = envelope.default_exploration_radius_m
    source = ParameterSource.POLICY_DEFAULT
    evidence = f"{radius:g} m conservative radius from {envelope.revision}"
    if environment.map_fresh and environment.map_bounds_xy is not None:
        min_x, max_x, min_y, max_y = environment.map_bounds_xy
        clearance = min(x - min_x, max_x - x, y - min_y, max_y - y)
        if clearance >= 1.0:
            observed_limit = max(0.75, 0.8 * clearance)
            if observed_limit < radius:
                radius = observed_limit
                source = ParameterSource.ENVIRONMENT_OBSERVATION
                evidence = (
                    f"fresh VDB XY extent limited the "
                    f"{envelope.default_exploration_radius_m:g} m policy radius to "
                    f"{radius:.3f} m around the start pose"
                )
            else:
                evidence = (
                    f"{radius:g} m conservative radius from {envelope.revision}; "
                    "fresh VDB extent encloses that radius"
                )
        else:
            assumptions.append(
                "The fresh VDB extent did not enclose a one-meter start-pose margin; "
                "the conservative policy radius was retained."
            )
    points = (
        SearchBoundPoint(x=x - radius, y=y - radius),
        SearchBoundPoint(x=x + radius, y=y - radius),
        SearchBoundPoint(x=x + radius, y=y + radius),
        SearchBoundPoint(x=x - radius, y=y + radius),
    )
    return points, source, evidence


def _coordinate_path(text: str) -> tuple[tuple[float, float, float], ...]:
    labelled = re.findall(
        r"\bx\s*=\s*(-?\d+(?:\.\d+)?)\s*[, ]+\s*y\s*=\s*(-?\d+(?:\.\d+)?)"
        r"\s*[, ]+\s*z\s*=\s*(-?\d+(?:\.\d+)?)", text,
    )
    if labelled:
        return tuple(tuple(float(value) for value in match) for match in labelled)
    parenthesized = re.findall(
        r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*"
        r"(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)", text,
    )
    if parenthesized:
        return tuple(tuple(float(value) for value in match) for match in parenthesized)
    plain = re.search(
        r"\b(?:fly|go|move|navigate)\s+to\s+(-?\d+(?:\.\d+)?)\s*[ ,]+\s*"
        r"(-?\d+(?:\.\d+)?)\s*[ ,]+\s*(-?\d+(?:\.\d+)?)\b", text,
    )
    return ((tuple(float(plain.group(index)) for index in range(1, 4))),) if plain else ()


def _relative_destination(text: str, current_position: tuple[float, float, float] | None,
                          yaw_rad: float | None) -> tuple[float, float, float] | None:
    match = re.search(
        r"\b(?:move|fly|go)\s+(forward|back(?:ward)?|left|right|up|down)"
        r"(?:\s+by)?\s+(-?\d+(?:\.\d+)?)\s*(?:m|meter|meters)?\b", text,
    )
    if not match:
        return None
    if current_position is None or yaw_rad is None:
        raise CommandClarificationRequired(
            "Can I capture a fresh position and heading for that relative movement?"
        )
    distance = float(match.group(2))
    if not math.isfinite(distance) or distance <= 0 or distance > 100:
        raise CommandClarificationRequired(
            "What positive relative distance of at most 100 meters should I use?"
        )
    direction = match.group(1)
    x, y, z = current_position
    if direction == "up":
        z += distance
    elif direction == "down":
        z -= distance
    else:
        angle = yaw_rad + {
            "forward": 0.0, "back": math.pi, "backward": math.pi,
            "left": math.pi / 2.0, "right": -math.pi / 2.0,
        }[direction]
        x += math.cos(angle) * distance
        y += math.sin(angle) * distance
    if z < 0.5:
        raise CommandClarificationRequired(
            "What relative movement should I use that keeps altitude at or above 0.5 meters?"
        )
    return x, y, z
