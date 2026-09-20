"""Nominal high-level policies and conflict scenarios for the CBF swarm.

Ported from ~/drone_soccer (drone_soccer/scenarios.py) and adapted for the
ground controller: scenarios receive plain ``positions`` arrays instead of a
MuJoCo state, and ``initial_positions()`` doubles as the per-drone takeoff
target. The CBF filter projects every nominal velocity onto the safe set, so
a scenario's job is purely to generate interesting conflicts.

Scenarios:

- ``hover``: hold the configured hover positions (the original demo).
- ``random_walk``: each drone holds a fixed-speed velocity and bounces off
  the arena walls -- no goals.
- ``random_goals``: each drone seeks a random goal point and picks a new one
  on arrival.
- ``head_on``: two facing groups fly to swapped goals, re-swapping on arrival
  for perpetual head-on conflicts at the center.
- ``antipodal``: drones on a sphere fly to their antipodes, all crossing the
  center at once.
- ``squeeze``: 3-drone CBF demo -- two "holder" drones goal-track posts
  separated by ``gap_factor * safety_radius`` while the third drone flies
  straight through the gap between them; the holders must yield and return.

Any drone listed in the commander's ``teleop_drones`` has its scenario row
replaced by operator input, so e.g. the squeeze intruder can be hand-flown.

Go-to-goal law (every scenario but ``random_walk``): see trajectory.py. A
drone the commander tracks with a reference point (real drones on the
trajectory output) gets an acceleration-limited velocity profile evaluated at
that reference plus its acceleration as feedforward; a velocity-only drone
gets the stateless braking law at its own position. ``nominal_velocity``
therefore takes the commander's ``references``/``applied``/``dt`` and the
acceleration comes back through ``nominal_acceleration``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np

from svg_ground_control.trajectory import (  # noqa: F401 (seek_velocity re-exported)
    DEFAULT_ACCEL, DEFAULT_SETTLE, DEFAULT_VELOCITY_ONLY_SETTLE, ReferenceTracker,
    seek_velocity)

DEFAULT_DT = 0.05   # s, the commander's control period when none is given


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned arena box the swarm is kept inside.

    Attributes:
        low: shape (3,) minimum corner (m); ``low[2]`` is the floor clearance.
        high: shape (3,) maximum corner (m).
    """

    low: np.ndarray
    high: np.ndarray

    @property
    def center(self) -> np.ndarray:
        return 0.5 * (self.low + self.high)

    @property
    def size(self) -> np.ndarray:
        return self.high - self.low


class Scenario(ABC):
    """A takeoff layout plus a nominal go-where policy for the swarm."""

    def __init__(
        self,
        num_drones: int,
        bounds: Bounds,
        nominal_speed: float,
        rng: np.random.Generator,
        safety_radius: float,
        accel: float = DEFAULT_ACCEL,
        settle_s: float = DEFAULT_SETTLE,
        velocity_only_settle_s: float = DEFAULT_VELOCITY_ONLY_SETTLE,
    ) -> None:
        self.num_drones = int(num_drones)
        self.bounds = bounds
        self.nominal_speed = float(nominal_speed)
        self.rng = rng
        self.safety_radius = float(safety_radius)
        # Go-to-goal law (trajectory.py): accel-limited reference profile for
        # drones the commander tracks with a reference point, stateless
        # braking law with the longer settle for velocity-only drones.
        self.tracker = ReferenceTracker(self.num_drones, accel, settle_s)
        self.velocity_only_settle = float(velocity_only_settle_s)
        self._acceleration = np.zeros((self.num_drones, 3))

    @abstractmethod
    def initial_positions(self) -> np.ndarray:
        """Return shape (N, 3) takeoff targets (non-overlapping, in bounds)."""
        ...

    @abstractmethod
    def nominal_velocity(self, positions: np.ndarray, references=None,
                         applied=None, dt: float = DEFAULT_DT) -> np.ndarray:
        """Return shape (N, 3) nominal (pre-CBF) velocities for this state.

        Args:
            positions: (N, 3) measured drone positions.
            references: (N, 3) the commander's per-drone reference points;
                a row of NaN (or ``None``) means that drone is velocity-only
                and gets the stateless law at its position.
            applied: (N, 3) velocity each reference point moved with since
                the last call (published, post-CBF), for re-attachment.
            dt: control period (s).
        The matching feedforward is in ``nominal_acceleration`` afterwards.
        """
        ...

    def seek(self, positions: np.ndarray, goals: np.ndarray, speeds,
             references=None, applied=None, dt: float = DEFAULT_DT) -> np.ndarray:
        """Go-to-goal velocities for all drones (see ``nominal_velocity``)."""
        positions = np.asarray(positions, dtype=float).reshape(self.num_drones, 3)
        goals = np.asarray(goals, dtype=float).reshape(self.num_drones, 3)
        speeds = np.broadcast_to(np.asarray(speeds, dtype=float).reshape(-1),
                                 (self.num_drones,))
        v = seek_velocity(positions, goals, speeds, self.tracker.accel,
                          self.velocity_only_settle)
        self._acceleration[:] = 0.0
        if references is None:
            return v
        refs = np.asarray(references, dtype=float).reshape(self.num_drones, 3)
        tracked = np.isfinite(refs).all(axis=1)
        if tracked.any():
            # The tracker steps every row; rows without a reference are
            # evaluated at the drone itself and simply not used.
            safe_refs = np.where(tracked[:, None], refs, positions)
            vt, at = self.tracker.step(safe_refs, goals, speeds, dt, applied)
            v[tracked] = vt[tracked]
            self._acceleration[tracked] = at[tracked]
        return v

    @property
    def nominal_acceleration(self) -> np.ndarray:
        """(N, 3) acceleration feedforward matching the last nominal_velocity."""
        return self._acceleration.copy()

    def reset_tracking(self) -> None:
        """Forget reference velocities (mission start / hold hand-overs)."""
        self.tracker.reset()
        self._acceleration[:] = 0.0

    @property
    def goals(self) -> Optional[np.ndarray]:
        """Current per-drone goal points (N, 3) for debugging, or None."""
        return None

    @property
    def cbf_exempt_indices(self) -> list:
        """Drone indices whose commands bypass the CBF while the scenario
        runs (deliberate obstacles — everyone else dodges them)."""
        return []

    def _random_positions(self, min_separation: float) -> np.ndarray:
        """Rejection-sample N takeoff positions ``min_separation`` apart."""
        margin = 0.1 * self.bounds.size
        low = self.bounds.low + margin
        high = self.bounds.high - margin
        positions = np.zeros((self.num_drones, 3))
        for index in range(self.num_drones):
            for _attempt in range(4000):
                candidate = self.rng.uniform(low, high)
                if index == 0 or np.all(
                    np.linalg.norm(positions[:index] - candidate, axis=-1)
                    >= min_separation
                ):
                    positions[index] = candidate
                    break
            else:
                positions[index] = candidate
        return positions


class HoverScenario(Scenario):
    """Hold fixed hover positions (the original N-1 hover + teleop demo)."""

    def __init__(self, *args, hover_positions: np.ndarray, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._targets = np.array(hover_positions, dtype=float).reshape(-1, 3)
        if self._targets.shape[0] != self.num_drones:
            raise ValueError(
                f'hover scenario needs {self.num_drones} hover positions, '
                f'got {self._targets.shape[0]}')

    def initial_positions(self) -> np.ndarray:
        return self._targets.copy()

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        return self.seek(positions, self._targets, self.nominal_speed,
                         references, applied, dt)

    @property
    def goals(self) -> Optional[np.ndarray]:
        return self._targets


class RandomWalkScenario(Scenario):
    """Fixed-speed drift with billiard reflection off the arena walls."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        directions = self.rng.normal(size=(self.num_drones, 3))
        directions /= np.linalg.norm(directions, axis=-1, keepdims=True)
        self._velocities = directions * self.nominal_speed

    def initial_positions(self) -> np.ndarray:
        return self._random_positions(min_separation=2.4 * self.safety_radius)

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        self._acceleration[:] = 0.0
        # Reflect any drone that has reached a wall and is still heading out.
        beyond_low = (positions <= self.bounds.low) & (self._velocities < 0.0)
        beyond_high = (positions >= self.bounds.high) & (self._velocities > 0.0)
        self._velocities[beyond_low | beyond_high] *= -1.0
        return self._velocities.copy()


class RandomGoalsScenario(Scenario):
    """Each drone seeks a random goal and resamples a new one on arrival."""

    _ARRIVAL_RADIUS_M = 0.3

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._positions = self._random_positions(
            min_separation=2.4 * self.safety_radius)
        self._goals = self._sample_goals()

    def _sample_goals(self) -> np.ndarray:
        margin = 0.1 * self.bounds.size
        return self.rng.uniform(
            self.bounds.low + margin,
            self.bounds.high - margin,
            size=(self.num_drones, 3),
        )

    def initial_positions(self) -> np.ndarray:
        return self._positions.copy()

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        reached = (
            np.linalg.norm(positions - self._goals, axis=-1)
            < self._ARRIVAL_RADIUS_M
        )
        if np.any(reached):
            fresh = self._sample_goals()
            self._goals[reached] = fresh[reached]
        return self.seek(positions, self._goals, self.nominal_speed,
                         references, applied, dt)

    @property
    def goals(self) -> Optional[np.ndarray]:
        return self._goals


class HeadOnScenario(Scenario):
    """Two facing rows swap sides, re-swapping on arrival for repeated conflict.

    Drones split into a left group (negative x) and a right group (positive
    x), each stacked along y and z. A small per-drone y offset breaks the
    perfectly-symmetric head-on that would otherwise deadlock the CBF.
    """

    _ARRIVAL_RADIUS_M = 0.4

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        left, right = self._build_rows()
        self._positions = np.concatenate([left, right], axis=0)[: self.num_drones]
        # Each drone's goal is its own slot mirrored across x (the far side).
        self._goals = self._positions.copy()
        self._goals[:, 0] *= -1.0

    def _build_rows(self) -> tuple[np.ndarray, np.ndarray]:
        """Lay each group on a y-z grid against its x wall."""
        left_count = (self.num_drones + 1) // 2
        right_count = self.num_drones // 2
        group_size = max(left_count, right_count, 1)

        columns = int(np.ceil(np.sqrt(group_size)))
        layers = int(np.ceil(group_size / columns))
        y_values = np.linspace(
            self.bounds.low[1] + 0.3, self.bounds.high[1] - 0.3, columns
        )
        if layers == 1:
            z_values = np.array([self.bounds.center[2]])
        else:
            z_values = np.linspace(
                self.bounds.low[2] + 0.5, self.bounds.high[2] - 0.5, layers
            )

        x_left = self.bounds.low[0] + 0.4
        x_right = self.bounds.high[0] - 0.4
        symmetry_break = 0.25

        def grid(count: int, x_wall: float, y_shift: float) -> np.ndarray:
            slots = []
            for k in range(count):
                layer, column = divmod(k, columns)
                slots.append([x_wall, y_values[column] + y_shift, z_values[layer]])
            return np.array(slots)

        left = grid(left_count, x_left, -symmetry_break)
        right = grid(right_count, x_right, symmetry_break)
        return left, right

    def initial_positions(self) -> np.ndarray:
        return self._positions.copy()

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        reached = (
            np.linalg.norm(positions - self._goals, axis=-1)
            < self._ARRIVAL_RADIUS_M
        )
        # Flip x target on arrival so the groups cross back and forth forever.
        self._goals[reached, 0] *= -1.0
        return self.seek(positions, self._goals, self.nominal_speed,
                         references, applied, dt)

    @property
    def goals(self) -> Optional[np.ndarray]:
        return self._goals


class AntipodalScenario(Scenario):
    """Drones on a sphere fly to their antipodes, all crossing the center."""

    _ARRIVAL_RADIUS_M = 0.4

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._center = self.bounds.center
        self._radius = 0.45 * float(np.min(self.bounds.size))
        self._positions = self._fibonacci_sphere()
        self._goals = 2.0 * self._center - self._positions

    def _fibonacci_sphere(self) -> np.ndarray:
        indices = np.arange(self.num_drones) + 0.5
        z = 1.0 - 2.0 * indices / self.num_drones
        radius_xy = np.sqrt(np.maximum(0.0, 1.0 - z * z))
        golden_angle = np.pi * (3.0 - np.sqrt(5.0))
        theta = golden_angle * indices
        unit = np.stack(
            [radius_xy * np.cos(theta), radius_xy * np.sin(theta), z], axis=-1)
        return self._center + self._radius * unit

    def initial_positions(self) -> np.ndarray:
        return self._positions.copy()

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        reached = (
            np.linalg.norm(positions - self._goals, axis=-1)
            < self._ARRIVAL_RADIUS_M
        )
        if np.any(reached):
            self._goals[reached] = 2.0 * self._center - self._goals[reached]
        return self.seek(positions, self._goals, self.nominal_speed,
                         references, applied, dt)

    @property
    def goals(self) -> Optional[np.ndarray]:
        return self._goals


class SqueezeScenario(Scenario):
    """3-drone CBF showcase: an intruder squeezes between two holders.

    Drones 0 and 1 ("holders") goal-track two explicitly configured posts.
    Drone 2 (the "intruder") shuttles back and forth between two explicitly
    configured waypoints; place the segment so it passes between the posts.

    The expected behavior: as the intruder approaches, the holders' filtered
    velocities push them apart (their nominal keeps pulling them back to
    their posts), the intruder passes through, and the holders settle back
    onto their posts. For the gap to be impassable without yielding, the
    intruder's path must come closer than ``2 * safety_radius`` to a post.

    The intruder is CBF-EXEMPT by default (``intruder_cbf_exempt``): it is
    the deliberate obstacle, and filtering it makes the filter push it
    *backwards* as it approaches the gap (the pair-constraint gradient
    points away from the holders), so it stalls or retreats instead of
    squeezing through. Exempt, it presses on and the holders alone yield.

    To hand-fly the intruder instead, list it in ``teleop_drones`` -- its
    scenario row is then ignored in favor of operator input.
    """

    def __init__(self, *args, holder_positions: np.ndarray,
                 intruder_waypoints: np.ndarray,
                 intruder_cbf_exempt: bool = True, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._intruder_cbf_exempt = bool(intruder_cbf_exempt)
        if self.num_drones != 3:
            raise ValueError(
                f'squeeze scenario requires exactly 3 drones, got {self.num_drones}')
        self._holder_posts = np.asarray(
            holder_positions, dtype=float).reshape(2, 3)
        self._intruder_ends = np.asarray(
            intruder_waypoints, dtype=float).reshape(2, 3)
        post_gap = float(np.linalg.norm(
            self._holder_posts[0] - self._holder_posts[1]))
        if post_gap < 2.0 * self.safety_radius:
            raise ValueError(
                f'holder posts are {post_gap:.2f} m apart, inside their own '
                f'2r keep-out ({2 * self.safety_radius:.2f} m) — the holders '
                'could never both reach their posts')
        self._intruder_goal_index = 1  # start by flying toward waypoint B
        self._arrival_radius = 0.3

    @property
    def holder_posts(self) -> np.ndarray:
        return self._holder_posts.copy()

    @property
    def intruder_waypoints(self) -> np.ndarray:
        return self._intruder_ends.copy()

    @property
    def cbf_exempt_indices(self) -> list:
        return [2] if self._intruder_cbf_exempt else []

    def initial_positions(self) -> np.ndarray:
        return np.vstack([self._holder_posts, self._intruder_ends[0]])

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        intruder_goal = self._intruder_ends[self._intruder_goal_index]
        if np.linalg.norm(positions[2] - intruder_goal) < self._arrival_radius:
            self._intruder_goal_index = 1 - self._intruder_goal_index
            intruder_goal = self._intruder_ends[self._intruder_goal_index]
        goals = np.vstack([self._holder_posts, intruder_goal])
        return self.seek(positions, goals, self.nominal_speed,
                         references, applied, dt)

    @property
    def goals(self) -> Optional[np.ndarray]:
        return np.vstack(
            [self._holder_posts, self._intruder_ends[self._intruder_goal_index]])


class GoalScenario(Scenario):
    """Each drone seeks a per-drone goal that can be retargeted live.

    Goals default to the configured takeoff layout (``initial_goals``, from
    ``hover_positions``); the commander updates them from
    ``/svg/{name}/goal_command`` while flying, and per-drone speed from
    ``/svg/{name}/speed_command`` (defaults to ``nominal_speed``). All
    goal-tracking drones are CBF-filtered, so commanding two of them at each
    other just makes them dodge. This backs the single- and multi-drone
    tracking tests.
    """

    # The speed setting is a cruise cap: the drone reaches it only if the
    # goal is farther than stopping_distance(speed, accel, settle) — see
    # trajectory.py — and brakes at goal_accel_mps2 from there.

    def __init__(self, *args, initial_goals: np.ndarray, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        # np.array (not asarray): set_goal writes in place and must never
        # alias the caller's array.
        self._goals = np.array(
            initial_goals, dtype=float).reshape(self.num_drones, 3)
        self._speeds = np.full(self.num_drones, self.nominal_speed)

    def set_goal(self, index: int, point: np.ndarray) -> None:
        self._goals[index] = np.asarray(point, dtype=float)

    def set_speed(self, index: int, speed: float) -> None:
        self._speeds[index] = max(0.0, float(speed))

    def set_all_speeds(self, speed: float) -> None:
        """New default for every drone (a live ``scenario_speed_mps`` change)."""
        self.nominal_speed = max(0.0, float(speed))
        self._speeds[:] = self.nominal_speed

    @property
    def speeds(self) -> np.ndarray:
        return self._speeds.copy()

    def initial_positions(self) -> np.ndarray:
        return self._goals.copy()

    def nominal_velocity(self, positions, references=None, applied=None,
                         dt: float = DEFAULT_DT) -> np.ndarray:
        return self.seek(positions, self._goals, self._speeds,
                         references, applied, dt)

    @property
    def goals(self) -> Optional[np.ndarray]:
        return self._goals


_SCENARIOS = {
    'hover': HoverScenario,
    'goal': GoalScenario,
    'random_walk': RandomWalkScenario,
    'random_goals': RandomGoalsScenario,
    'head_on': HeadOnScenario,
    'antipodal': AntipodalScenario,
    'squeeze': SqueezeScenario,
}


def make_scenario(
    name: str,
    num_drones: int,
    nominal_speed: float,
    bounds: Bounds,
    safety_radius: float,
    seed: int = 7,
    **kwargs,
) -> Scenario:
    """Construct a scenario by name.

    Args:
        name: one of ``hover``, ``random_walk``, ``random_goals``, ``head_on``,
            ``antipodal``, ``squeeze``.
        num_drones: number of drones (scenario rows match drone_names order).
        nominal_speed: nominal flight speed (m/s).
        bounds: arena box.
        safety_radius: CBF safety radius r (m), used for spacing decisions.
        seed: RNG seed for the randomized scenarios.
        **kwargs: scenario-specific options (``hover_positions`` for hover,
            ``initial_goals`` for goal, ``holder_positions`` /
            ``intruder_waypoints`` for squeeze) and the go-to-goal law's
            ``accel`` (m/s^2), ``settle_s`` and ``velocity_only_settle_s``
            (s) — see trajectory.py.

    Raises:
        ValueError: if ``name`` is unknown.
    """
    if name not in _SCENARIOS:
        raise ValueError(
            f'unknown scenario {name!r}; choose from {sorted(_SCENARIOS)}')
    return _SCENARIOS[name](
        num_drones, bounds, nominal_speed, np.random.default_rng(seed),
        safety_radius, **kwargs)
