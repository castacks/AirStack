"""Mock action policy. Replaced by GR00T N1.7 via LeRobot at Phase 2."""

from __future__ import annotations

from .schema import AbstractAction, Trajectory, Verb, WorldState

# ---------------------------------------------------------------------------
# Action policy  (stands in for GR00T N1.7 via the embodiment adapter)
# ---------------------------------------------------------------------------

class MockPolicy:
    """Emits trajectory chunks. Real subtasks take several cycles, not one.

    GR00T emits a 16-step action chunk per inference (~64 ms on an L40). One
    reasoner decision therefore spans many policy inferences — the inner loop.
    """

    CYCLES = {
        Verb.LOCATE: 1, Verb.GRASP: 3, Verb.PLACE: 3,
        Verb.RELEASE: 1, Verb.OPEN: 2, Verb.CLOSE: 2, Verb.INSPECT: 1,
    }

    def __init__(self) -> None:
        self._elapsed: dict[str, int] = {}

    def step(self, action: AbstractAction, ws: WorldState) -> Trajectory:
        n = self._elapsed.get(action.id, 0) + 1
        self._elapsed[action.id] = n
        budget = self.CYCLES.get(action.verb, 1)
        # Placeholder kinematics. The real adapter renders the action as a language
        # instruction ("pick up the red cup"), pairs it with raw camera frames and
        # proprioception — NOT WorldState — and asks GR00T for joint targets.
        bx, by, bz = ws.robot.base_pose
        return Trajectory(
            action_id=action.id,
            waypoints=[(bx + 0.1 * n, by - 0.2 * n, bz + 0.05 * n)],
            max_velocity=0.4,
            terminal=n >= budget,
        )

    def reset(self, action_id: str) -> None:
        self._elapsed.pop(action_id, None)


