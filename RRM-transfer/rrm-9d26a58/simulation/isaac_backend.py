"""Isaac Sim backend — the work that does NOT transfer from the mock.

The loop, schema, verb table, planner, divergence detection and metrics all move
across unchanged. This file is everything that does not.

    python3 simulation/isaac_backend.py     # self-test, no Isaac required
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rrm import (  # noqa: E402
    AbstractAction, Relation, Trajectory, WorldObject, WorldState,
)

# ---------------------------------------------------------------------------
# Relation inference — the largest hidden cost of the Isaac migration.
#
# Isaac Sim gives poses. It does NOT give `on(cup, table)`. Every semantic
# relation the world model holds must be COMPUTED from geometry, and the quality
# of that computation sets a ceiling on world-state accuracy (benchmarks.md §3).
# Written here as pure geometry so it is testable before Isaac exists.
# ---------------------------------------------------------------------------

CONTACT_TOL_M = 0.03        # vertical slack when deciding "resting on"
CONTAINMENT_MARGIN = 0.0    # xy inset required to count as "inside"


def _extent(obj: WorldObject) -> tuple[float, float, float]:
    """Half-extents in metres. Isaac supplies these from the prim's bounding box."""
    e = obj.properties.get("extent")
    return tuple(e) if e else (0.05, 0.05, 0.05)  # type: ignore[return-value]


def _xy_within(a: WorldObject, b: WorldObject, margin: float = 0.0) -> bool:
    assert a.pose and b.pose
    (ax, ay, _), (bx, by, _) = a.pose, b.pose
    bex, bey, _ = _extent(b)
    return abs(ax - bx) <= bex - margin and abs(ay - by) <= bey - margin


def infer_relations(objects: list[WorldObject]) -> list[Relation]:
    """Derive semantic relations from poses and extents.

    Handles `on` and `in`. Deliberately conservative: a relation that cannot be
    established geometrically is simply absent rather than guessed, because a
    confidently wrong relation corrupts divergence detection, while a missing one
    only costs a replan.
    """
    out: list[Relation] = []
    posed = [o for o in objects if o.pose is not None]

    for a in posed:
        for b in posed:
            if a.id == b.id:
                continue

            aex, aey, aez = _extent(a)
            bex, bey, bez = _extent(b)
            az, bz = a.pose[2], b.pose[2]      # type: ignore[index]

            # `in`: a's centre inside b's volume, and b is a container.
            if b.properties.get("container") and _xy_within(a, b, CONTAINMENT_MARGIN) \
                    and abs(az - bz) <= bez:
                out.append(Relation(subject=a.id, predicate="in", obj=b.id,
                                    confidence=0.9))
                continue

            # `on`: a rests on b's upper surface, within b's footprint.
            if b.properties.get("supports") and _xy_within(a, b):
                expected_z = bz + bez + aez
                if abs(az - expected_z) <= CONTACT_TOL_M:
                    out.append(Relation(subject=a.id, predicate="on", obj=b.id,
                                        confidence=0.95))

    return _resolve_conflicts(out)


def _resolve_conflicts(relations: list[Relation]) -> list[Relation]:
    """An object rests on at most one surface. Keep the highest-confidence claim.

    Stacked geometry routinely satisfies `on` against several surfaces at once
    (a cup on a tray on a table). Emitting all of them would make `¬on(x, *)` —
    GR00T's grasp effect — unsatisfiable.
    """
    best: dict[tuple[str, str], Relation] = {}
    for r in relations:
        key = (r.subject, r.predicate)
        if key not in best or r.confidence > best[key].confidence:
            best[key] = r
    return list(best.values())


# ---------------------------------------------------------------------------
# WorldBackend implementation
# ---------------------------------------------------------------------------

class IsaacWorldBackend:
    """Implements the WorldBackend protocol against Isaac Sim over ROS 2.

    TODO(deploy) in dependency order:

    1. USD scene: Franka Panda (LIBERO_PANDA embodiment, see architecture §8.0),
       table, cup, floor. Objects need `supports`/`container`/`graspable`
       annotations and bounding-box extents — infer_relations depends on them.
    2. Deterministic episode reset. Isaac physics is not reproducible by default;
       benchmarks.md §5 requires seeded episodes. This gates the whole benchmark.
    3. observe(): read prim poses -> WorldObject, run infer_relations, read
       /joint_states -> RobotState. Provenance is SIM_GROUND_TRUTH in Ph.3-5.
    4. apply(): publish JointTrajectory, step the sim, return once the chunk has
       been executed. See the time-semantics note below.
    5. Predicate grounding: `reachable` needs real IK against the Panda's
       workspace, not a radius. `graspable` comes from scene annotation.
       `gripper_empty` from the gripper joint or a contact sensor.
    """

    def __init__(self, ros_node: object, stage: object) -> None:
        self._node = ros_node
        self._stage = stage
        raise NotImplementedError("deployment item 3")

    def observe(self) -> WorldState:
        """Sensors -> semantic belief. The backend owns perception (architecture §9).

        Must return a copy: callers retain snapshots across an action to compute
        divergence, and a shared mutable reference silently makes `before` equal
        `after`, which reports zero divergence for every action.
        """
        raise NotImplementedError("deployment item 3")

    def apply(self, action: AbstractAction, traj: Trajectory) -> None:
        """Execute one chunk.

        TIME SEMANTICS — the one place the mock lies. MockWorld.apply() is
        synchronous and instantaneous; `t` is an integer counter. Isaac runs
        continuously and motion takes wall-clock time. Two options:

          (a) block until the controller reports the trajectory complete, keeping
              the loop synchronous and the existing code correct; or
          (b) return immediately and let the inner loop poll, which is closer to
              real robots but means dispatch() cycles no longer correspond to
              policy inferences and CYCLE_BUDGET changes meaning.

        Start with (a). It preserves loop semantics exactly and (b) can be
        introduced later as a measured change rather than an unexamined default.
        """
        raise NotImplementedError("deployment item 4")

    def begin_dispatch(self, action: AbstractAction) -> None:
        """New outer-loop attempt. Isaac needs no bookkeeping here; the mock used
        it only for failure injection."""
        return None


# ---------------------------------------------------------------------------
# Embodiment adapter + policy
# ---------------------------------------------------------------------------

VERB_PHRASING = {
    "GRASP": "pick up the {desc}",
    "PLACE": "put the {desc} on the {target}",
    "OPEN": "open the {desc}",
    "CLOSE": "close the {desc}",
    "LOCATE": "look at the {desc}",
}


class EmbodimentAdapter:
    """AbstractAction -> what GR00T actually consumes.

    Renders a language instruction AND passes the resolved target pose through the
    numeric state channel. Passing the string alone discards the grounding the world
    model just computed — with two cups in the scene "pick up the red cup" is
    ambiguous where the ObjectID was not (architecture §2.1).
    """

    def render(self, action: AbstractAction, ws: WorldState) -> tuple[str, list[float]]:
        objs = [ws.get(t) for t in action.targets]
        if any(o is None for o in objs):
            raise ValueError(f"{action} references an object absent from WorldState")

        def describe(o: WorldObject) -> str:
            colour = o.properties.get("color")
            return f"{colour} {o.cls}" if colour else o.cls

        template = VERB_PHRASING.get(action.verb.value, "{desc}")
        text = template.format(
            desc=describe(objs[0]),                      # type: ignore[arg-type]
            target=describe(objs[1]) if len(objs) > 1 else "",  # type: ignore[arg-type]
        )
        target_pose = list(objs[0].pose or (0.0, 0.0, 0.0))   # type: ignore[union-attr]
        return text, target_pose


class GR00TPolicy:
    """Implements the ActionPolicy protocol.

    TODO(deploy):
    6. Load nvidia/GR00T-N1.7-3B with the LIBERO_PANDA embodiment tag.
    7. Build the observation dict its modality_config expects: camera frames from
       /camera/rgb (NOT WorldState), proprioception from /joint_states, the
       instruction and target pose from EmbodimentAdapter.
    8. Convert the 16-step action chunk to JointTrajectory. Note GR00T reads
       LeRobot v2.1 while the framework records v3.0 (architecture §8.0).

    Numeric safety (Safety #2) must run on the output before it is published, and
    the current implementation checks scalar waypoints against a placeholder range.
    Replace with real Panda joint limits from the URDF, velocities derived from
    trajectory timing, and swept-volume collision — the last is genuinely hard and
    is the item most likely to be underestimated.
    """

    def __init__(self, checkpoint: str, embodiment_tag: str = "LIBERO_PANDA") -> None:
        self._checkpoint = checkpoint
        self._tag = embodiment_tag
        raise NotImplementedError("deployment item 6")

    def step(self, action: AbstractAction, ws: WorldState) -> Trajectory:
        raise NotImplementedError("deployment item 7")

    def reset(self, action_id: str) -> None:
        raise NotImplementedError("deployment item 7")


# ---------------------------------------------------------------------------
# Self-test for the one part that needs no Isaac
# ---------------------------------------------------------------------------

def _self_test() -> int:
    table = WorldObject(id="table", cls="table", pose=(0.6, 0.0, 0.40),
                        properties={"supports": True, "extent": (0.4, 0.3, 0.02)})
    floor = WorldObject(id="floor", cls="floor", pose=(0.0, 0.0, 0.0),
                        properties={"supports": True, "extent": (5.0, 5.0, 0.01)})
    cabinet = WorldObject(id="cabinet", cls="cabinet", pose=(1.2, 0.0, 0.5),
                          properties={"container": True, "extent": (0.3, 0.3, 0.4)})
    cup = WorldObject(id="cup", cls="cup", pose=(0.6, 0.0, 0.47),
                      properties={"graspable": True, "extent": (0.04, 0.04, 0.05)})
    bottle = WorldObject(id="bottle", cls="bottle", pose=(1.2, 0.0, 0.5),
                         properties={"graspable": True, "extent": (0.04, 0.04, 0.10)})

    rels = infer_relations([table, floor, cabinet, cup, bottle])
    got = {(r.subject, r.predicate, r.obj) for r in rels}

    expected = {("cup", "on", "table"), ("bottle", "in", "cabinet")}
    missing = expected - got
    for r in sorted(got):
        print(f"  inferred: {r[0]} {r[1]} {r[2]}")

    # The cup sits within the floor's footprint too; conflict resolution must keep
    # only the table, or ¬on(cup, *) becomes unsatisfiable after a grasp.
    if ("cup", "on", "floor") in got:
        print("FAIL: cup resolved onto both table and floor")
        return 1
    if missing:
        print(f"FAIL: missing {missing}")
        return 1
    print("infer_relations: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
