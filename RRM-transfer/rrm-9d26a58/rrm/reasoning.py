"""Reasoner backends. See docs/architecture.md §6."""

from __future__ import annotations

from .schema import (
    AbstractAction, Divergence, ObjectID, Predicate, SELF, TaskGraph, Verb,
    WILDCARD, WorldState,
)
from .verbs import VERB_TABLE, holds, preconditions_of

# ---------------------------------------------------------------------------
# Reasoner backends  (docs/architecture.md §6)
# ---------------------------------------------------------------------------

class ReasonerBackend:
    name = "base"

    def plan(self, mission: str, ws: WorldState) -> TaskGraph:
        raise NotImplementedError

    def replan(self, mission: str, ws: WorldState, graph: TaskGraph,
               div: Divergence) -> TaskGraph:
        raise NotImplementedError


def _unify(template: Predicate, goal: Predicate) -> list[ObjectID] | None:
    """Match a verb's effect template against a goal, returning bound targets.

    Returns None if the template cannot produce this goal.
    """
    if template.name != goal.name or template.negated != goal.negated:
        return None

    bound: dict[int, ObjectID] = {}
    for tok, val in ((template.subject, goal.subject), (template.obj, goal.obj)):
        if isinstance(tok, str) and tok.startswith("$") and tok[1:].isdigit():
            bound[int(tok[1:])] = str(val)
        elif tok == SELF:
            if val != SELF:
                return None
        elif tok == WILDCARD or tok is None:
            continue
        elif tok != val:
            return None

    if not bound:
        return []
    arity = max(bound) + 1
    if any(i not in bound for i in range(arity)):
        return None      # goal did not bind every target the verb needs
    return [bound[i] for i in range(arity)]


class ScriptedOracle(ReasonerBackend):
    """Deterministic backward-chaining planner over VERB_TABLE. No model.

    This is the reference ceiling for everything except learned reasoning: it plans
    perfectly within the verb vocabulary but cannot resolve ambiguity, infer intent
    from natural language, or handle a goal outside the table. When RRM-1 trails this,
    the gap is the reasoner; when this fails, the bug is elsewhere.
    """

    name = "scripted_oracle"
    MAX_DEPTH = 6

    def __init__(self, goal: Predicate) -> None:
        # The oracle is handed the goal predicate because it cannot parse language.
        # A real reasoner derives this from mission text — which is precisely the
        # capability T5 (ambiguous missions) is designed to measure.
        self.goal = goal

    def _chain(self, goal: Predicate, ws: WorldState,
               seen: set[str], depth: int) -> list[AbstractAction] | None:
        if holds(goal, ws):
            return []                      # already satisfied — plan nothing
        key = str(goal)
        if depth > self.MAX_DEPTH or key in seen:
            return None
        seen = seen | {key}

        for verb, spec in VERB_TABLE.items():
            for template in spec.expected_effects:
                targets = _unify(template, goal)
                if targets is None or len(targets) != spec.arity:
                    continue
                action = AbstractAction(id="", verb=verb, targets=targets,
                                        rationale=f"achieves {goal}")
                steps: list[AbstractAction] = []
                for pre in preconditions_of(action):
                    sub = self._chain(pre, ws, seen, depth + 1)
                    if sub is None:
                        break
                    steps.extend(s for s in sub if s not in steps)
                else:
                    return steps + [action]
        return None

    def _graph(self, mission: str, goal: Predicate, ws: WorldState,
               version: int = 0) -> TaskGraph:
        nodes = self._chain(goal, ws, set(), 0)
        if nodes is None:
            raise ValueError(f"oracle cannot achieve {goal} from the current state")
        for i, n in enumerate(nodes):
            n.id = f"a{i}"
        return TaskGraph(mission_id="m0", mission_text=mission, nodes=nodes,
                         version=version)

    def plan(self, mission: str, ws: WorldState) -> TaskGraph:
        return self._graph(mission, self.goal, ws)

    def replan(self, mission: str, ws: WorldState, graph: TaskGraph,
               div: Divergence) -> TaskGraph:
        """Re-derive a plan from the observed state toward the same goal.

        Backward chaining from current state is the whole recovery mechanism: work
        already accomplished is skipped because those predicates now hold, and work
        undone by a failure reappears because they do not.
        """
        return self._graph(mission, self.goal, ws, graph.version + 1)


