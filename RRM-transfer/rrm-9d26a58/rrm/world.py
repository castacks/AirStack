"""Mock world backend. Replaced by Isaac Sim via ROS 2 at Phase 1.

Scaffolding, deliberately: see docs/benchmarks.md §6. The durable artifacts are
the WorldBackend protocol, the task definitions and the metrics."""

from __future__ import annotations

import logging

from .schema import (
    AbstractAction, Relation, RobotState, Trajectory, Verb, WorldObject, WorldState,
)

log = logging.getLogger("rrm")

# ---------------------------------------------------------------------------
# Mock world  (stands in for Isaac Sim)
# ---------------------------------------------------------------------------

class MockWorld:
    """Ground truth. Mutates on execute(); the loop only sees observe()."""

    def __init__(self, fail_grasp_once: bool = False, human: bool = False,
                 fail_grasp_always: bool = False) -> None:
        objects = [
            WorldObject(id="obj_table", cls="table", pose=(0.6, 0.0, 0.4),
                        properties={"graspable": False, "supports": True}),
            WorldObject(id="obj_cup", cls="cup", pose=(0.4, -0.2, 0.0),
                        properties={"graspable": True, "color": "red"}),
            WorldObject(id="obj_floor", cls="floor", pose=(0.0, 0.0, 0.0),
                        properties={"graspable": False, "supports": True}),
        ]
        if human:
            objects.append(WorldObject(id="obj_person", cls="person", pose=(0.75, 0.1, 0.0)))

        self.state = WorldState(
            t=0,
            objects=objects,
            relations=[Relation(subject="obj_cup", predicate="on", obj="obj_floor")],
            robot=RobotState(),
        )
        self._fail_grasp = fail_grasp_once or fail_grasp_always
        self._fail_forever = fail_grasp_always
        self._attempts: dict[str, int] = {}

    def observe(self) -> WorldState:
        return self.state.model_copy(deep=True)

    def begin_dispatch(self, action: AbstractAction) -> None:
        """Mark a new outer-loop attempt at this action.

        Failure injection is per *attempt*, not per chunk. A policy that retries
        within one dispatch and succeeds has genuinely recovered, and the outer loop
        should never hear about it — only failures the policy cannot fix itself are
        worth replanning over.
        """
        key = f"{action.verb.value}:{','.join(action.targets)}"
        self._attempts[key] = self._attempts.get(key, 0) + 1
        self._attempt_key = key

    def _attempt_no(self, action: AbstractAction) -> int:
        key = f"{action.verb.value}:{','.join(action.targets)}"
        return self._attempts.get(key, 1)

    def apply(self, action: AbstractAction, traj: Trajectory) -> None:
        """Advance physics by one trajectory chunk.

        Physical consequences land only on the terminal chunk — mid-motion the world
        is in transit and the action's effects are legitimately not yet satisfied.
        That is exactly why the inner loop polls rather than assuming success.
        """
        self.state.t += 1
        if not traj.terminal:
            return

        s = self.state

        if action.verb is Verb.LOCATE:
            return  # pose already known in the mock

        if action.verb is Verb.GRASP:
            target = action.targets[0]
            if self._fail_grasp and (self._fail_forever or self._attempt_no(action) == 1):
                log.info("        [world] grasp slipped — cup displaced, not held")
                o = s.get(target)
                if o and o.pose:
                    o.pose = (o.pose[0] - 0.02, o.pose[1] + 0.02, o.pose[2])
                return
            s.relations = [r for r in s.relations if r.subject != target]
            s.robot.holding = target
            s.robot.gripper = "holding"
            return

        if action.verb is Verb.PLACE:
            obj, surface = action.targets[0], action.targets[1]
            s.relations = [r for r in s.relations if r.subject != obj]
            s.relations.append(Relation(subject=obj, predicate="on", obj=surface))
            s.robot.holding = None
            s.robot.gripper = "open"
            surf = s.get(surface)
            o = s.get(obj)
            if surf and surf.pose and o:
                o.pose = (surf.pose[0], surf.pose[1], surf.pose[2] + 0.05)
            return

        if action.verb is Verb.RELEASE:
            s.robot.holding = None
            s.robot.gripper = "open"
            return

        if action.verb in (Verb.OPEN, Verb.CLOSE):
            o = s.get(action.targets[0])
            if o:
                o.properties["open"] = action.verb is Verb.OPEN
            return


