"""Mission-authorized automatic action → verified-outcome → replan coordinator.

The coordinator is transport and embodiment neutral.  It does not execute a vehicle
action: an adapter supplies independently verified outcomes.  One explicit mission
authorization covers a bounded, declared action scope; subsequent fresh observations
and replans are automatic until verification fails or the scope expires.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Protocol

from rrm.live_replan import LiveInferenceProvider, LiveReplanCycle


class LiveSnapshotter(Protocol):
    def capture(self) -> tuple[dict, bytes, dict]: ...


@dataclass(frozen=True)
class MissionAuthorization:
    task_id: str
    task_revision: str
    allowed_verbs: frozenset[str]
    allowed_targets: frozenset[str]
    max_actions: int

    def __post_init__(self) -> None:
        if (not self.task_id or not self.task_revision or not self.allowed_verbs or
                not self.allowed_targets or self.max_actions <= 0):
            raise ValueError("Mission authorization requires a nonempty bounded action scope.")


class ContinuousReplanMission:
    """Automate replanning after verified outcomes under one bounded authorization."""

    def __init__(self, cycle: LiveReplanCycle, context, provider: LiveInferenceProvider,
                 snapshotter: LiveSnapshotter, authorization: MissionAuthorization):
        if (cycle.state["task_id"] != authorization.task_id or
                cycle.state["task_revision"] != authorization.task_revision or
                context.task.task_id != authorization.task_id or
                context.task.revision != authorization.task_revision):
            raise ValueError("Mission authorization must match the immutable cycle task.")
        self.cycle, self.context, self.provider = cycle, context, provider
        self.snapshotter, self.authorization = snapshotter, authorization
        self.authorized = False
        self.actions_completed = 0

    def propose_initial_action(self) -> dict:
        if self.authorized or self.actions_completed or self.cycle.phase.value != "AWAITING_OBSERVATION":
            raise ValueError("Initial mission proposal is unavailable in the current state.")
        return self._capture_and_propose()

    def approve_mission(self) -> dict:
        """Authorize the already-presented first action and bounded later replans."""
        action = self._require_ready_action()
        self._validate_scope(action)
        self.cycle.mark_reviewed(action["id"])
        self.authorized = True
        return {"action": action, "execution_authorized": True,
                "automatic_replan_after_verified_outcome": True}

    def record_verified_outcome_and_replan(self, action_id: str, *, verified: bool, detail: str) -> dict:
        """After an adapter verifies one action, automatically capture and replan."""
        if not self.authorized:
            raise ValueError("Mission approval is required before accepting an action outcome.")
        current = self._require_outcome_action()
        if action_id != current:
            raise ValueError("Outcome belongs to a different action.")
        self.cycle.record_outcome(action_id, verified=verified, detail=detail)
        if not verified:
            return {"state": self.cycle.phase.value, "next_action": None,
                    "reason": "action_outcome_not_verified", "execution_authorized": False}
        self.actions_completed += 1
        if self.actions_completed >= self.authorization.max_actions:
            self.cycle.state.update({"state": "HALTED", "halt_reason": "mission_action_budget_exhausted"})
            self.cycle._persist()
            return {"state": "HALTED", "next_action": None,
                    "reason": "mission_action_budget_exhausted", "execution_authorized": False}
        result = self._capture_and_propose()
        action = result.get("next_action", {}).get("action")
        if action is None:
            return {**result, "execution_authorized": False}
        self._validate_scope(action)
        self.cycle.mark_reviewed(action["id"])
        return {**result, "action": action, "execution_authorized": True,
                "automatic_replan_after_verified_outcome": True}

    def _capture_and_propose(self) -> dict:
        metadata, image, scene_state = self.snapshotter.capture()
        self.cycle.record_observation(metadata, image, scene_state)
        return self.cycle.request_next_action(self.context, self.provider)

    def _require_ready_action(self) -> dict:
        if self.cycle.phase.value != "REVIEW_REQUIRED":
            raise ValueError("A fresh accepted proposal is required.")
        return self._current_action()

    def _require_outcome_action(self) -> str:
        if self.cycle.phase.value != "OUTCOME_REQUIRED":
            raise ValueError("The active action is not awaiting an outcome.")
        return self.cycle.state["active_action_id"]

    def _current_action(self) -> dict:
        step = self.cycle.root / "steps" / f"{self.cycle.state['active_step_index']:04d}" / "provider-response.json"
        candidate = json.loads(step.read_text())["candidate"]
        return candidate["plan"]["actions"][0]["action"]

    def _validate_scope(self, action: dict) -> None:
        if action.get("verb") not in self.authorization.allowed_verbs:
            raise ValueError("Proposed action verb is outside the authorized mission scope.")
        targets = action.get("targets")
        if not isinstance(targets, list) or not targets or not set(targets) <= self.authorization.allowed_targets:
            raise ValueError("Proposed action targets are outside the authorized mission scope.")
