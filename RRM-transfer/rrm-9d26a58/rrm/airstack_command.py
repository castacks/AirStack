"""Deterministic natural-language routing to discovered AirStack task executors.

This is the command-to-task boundary, not a controller. It emits only typed goals for
public AirStack actions that the active robot configuration actually serves.
"""
from __future__ import annotations

import math
import re
from typing import Iterable

from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal, MapWaypoint


TAKEOFF_ACTION = "task_msgs/action/TakeoffTask"
LAND_ACTION = "task_msgs/action/LandTask"
NAVIGATE_ACTION = "task_msgs/action/NavigateTask"
EXPLORATION_ACTION = "task_msgs/action/ExplorationTask"


def plan_command(objective: str, *, task_id: str, robot_name: str,
                 action_servers: Iterable[str], airborne: bool,
                 current_position: tuple[float, float, float] | None = None,
                 yaw_rad: float | None = None) -> tuple[DroneTaskProposal, ...]:
    """Translate a movement command into a bounded sequence of live task actions."""
    if not isinstance(objective, str) or not objective.strip():
        raise ValueError("Movement command is required.")
    text = objective.strip().lower()
    available = frozenset(action_servers)
    proposals: list[DroneTaskProposal] = []

    wants_takeoff = bool(re.search(r"\b(take[ -]?off|launch|ascend)\b", text))
    wants_land = bool(re.search(r"\b(land|touch[ -]?down)\b", text))
    wants_explore = bool(re.search(r"\b(explore|survey|roam|map the|move around)\b", text))
    coordinates = _coordinate_path(text)
    takeoff_altitude = min(max(
        _number_after(text, r"(?:take[ -]?off|launch|ascend)(?:\s+to)?", 1.5), 0.5
    ), 3.0)
    relative_origin = current_position
    if not airborne and current_position is not None:
        relative_origin = (current_position[0], current_position[1],
                           max(current_position[2], takeoff_altitude))
    relative = _relative_destination(text, relative_origin, yaw_rad)
    if relative is not None:
        if coordinates:
            raise ValueError("Use either map coordinates or one relative movement in a command.")
        coordinates = (relative,)

    if not any((wants_takeoff, wants_land, wants_explore, coordinates)):
        raise ValueError(
            "Unsupported movement command. Use take off, land, explore/survey, or "
            "a map goal such as 'fly to x=1.0 y=2.0 z=1.5'."
        )

    needs_airborne = wants_explore or bool(coordinates)
    if (wants_takeoff or (needs_airborne and not airborne)) and not airborne:
        _require(available, TAKEOFF_ACTION, "takeoff")
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=f"takeoff-{len(proposals)}", robot_name=robot_name,
            kind=DroneTaskKind.TAKEOFF, target_altitude_m=takeoff_altitude, velocity_m_s=0.5,
        ))

    if coordinates:
        _require(available, NAVIGATE_ACTION, "navigation")
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=f"navigate-{len(proposals)}", robot_name=robot_name,
            kind=DroneTaskKind.NAVIGATE, frame_id="map",
            waypoints=tuple(MapWaypoint(x=x, y=y, z=z) for x, y, z in coordinates),
            goal_tolerance_m=0.5,
        ))

    if wants_explore:
        _require(available, EXPLORATION_ACTION, "global exploration planning")
        duration = _number_after(text, r"(?:for|during)", 60.0)
        duration = min(max(duration, 5.0), 900.0)
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=f"explore-{len(proposals)}", robot_name=robot_name,
            kind=DroneTaskKind.EXPLORE,
            min_altitude_agl_m=1.0, max_altitude_agl_m=3.0,
            min_flight_speed_m_s=0.5, max_flight_speed_m_s=2.0,
            time_limit_s=duration,
        ))

    if wants_land and (airborne or proposals):
        _require(available, LAND_ACTION, "landing")
        proposals.append(DroneTaskProposal(
            task_id=task_id, action_id=f"land-{len(proposals)}", robot_name=robot_name,
            kind=DroneTaskKind.LAND, velocity_m_s=0.5,
        ))
    if not proposals:
        raise ValueError("Command is already satisfied by the observed airborne state.")
    if any(proposal.kind is DroneTaskKind.TAKEOFF for proposal in proposals):
        _require(available, LAND_ACTION, "safe recovery landing")
    return tuple(proposals)


def takeoff_recovery_action(proposals: Iterable[DroneTaskProposal]) -> DroneTaskProposal | None:
    """Predeclare the public LandTask used after a verified takeoff mismatch."""
    takeoff = next((item for item in proposals if item.kind is DroneTaskKind.TAKEOFF), None)
    if takeoff is None:
        return None
    return DroneTaskProposal(
        task_id=takeoff.task_id,
        action_id=f"{takeoff.action_id}-recovery-land",
        robot_name=takeoff.robot_name,
        kind=DroneTaskKind.LAND,
        velocity_m_s=0.5,
    )


def _require(available: frozenset[str], action_type: str, label: str) -> None:
    if action_type not in available:
        raise ValueError(f"The active AirStack configuration has no {label} task executor.")


def _number_after(text: str, prefix: str, default: float) -> float:
    match = re.search(prefix + r"\s+(-?\d+(?:\.\d+)?)", text)
    return float(match.group(1)) if match else default


def _coordinate_path(text: str) -> tuple[tuple[float, float, float], ...]:
    labelled = re.findall(
        r"\bx\s*=\s*(-?\d+(?:\.\d+)?)\s*[, ]+\s*y\s*=\s*(-?\d+(?:\.\d+)?)"
        r"\s*[, ]+\s*z\s*=\s*(-?\d+(?:\.\d+)?)",
        text,
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
        raise ValueError("Fresh position and heading are required for relative movement.")
    distance = float(match.group(2))
    if not math.isfinite(distance) or distance <= 0 or distance > 100:
        raise ValueError("Relative movement distance must be greater than 0 and at most 100 meters.")
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
        raise ValueError("Relative movement would place the vehicle below 0.5 meters.")
    return x, y, z
