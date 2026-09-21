"""Explicitly invoked, bounded live-mission coordinator for simulator integration.

This module composes fresh entity-verifier context, learned C05 proposals,
deterministic embodiment compilation, and independently verified outcomes.  It has no
ROS, Docker, subprocess, OSMO, or model-runtime dependency; a separately started
runtime must inject those boundaries.  Starting a Cosmos worker therefore never gives
the worker an execution path.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time
from typing import Callable, Protocol

from rrm.airstack_drone import DroneOutcomeVerdict, DroneOutcomeVerification, DroneTaskProposal
from rrm.continuous_replan import MissionAuthorization
from rrm.cosmos_reason2 import CosmosReasoningInput
from rrm.drone_decision import DroneDecision, DroneDecisionStatus
from rrm.dynamic_feasibility import (
    DynamicFeasibilityEvaluator,
    FailClosedFeasibilityEvaluator,
    SingleUseAdmission,
)
from rrm.live_replan import LiveInferenceProvider, LiveReplanCycle
from rrm.state_contracts import FactKey
from rrm.contracts import Truth
from rrm.task_contracts import PlanProposal


@dataclass(frozen=True)
class VerifiedLiveStep:
    """One camera capture and separate verifier-produced C02 context."""

    metadata: dict
    image: bytes
    scene_state: dict
    context: CosmosReasoningInput

    def __post_init__(self) -> None:
        checksum = hashlib.sha256(self.image).hexdigest()
        if self.metadata.get("sha256") != checksum:
            raise ValueError("Verified live step image does not match observation metadata.")
        if self.scene_state.get("observation_sha256") != checksum:
            raise ValueError("Entity verifier record is not bound to this camera image.")


class LiveEntityContextSource(Protocol):
    """Captures a frame and produces independent entity-verifier C02 evidence."""

    def capture(self) -> VerifiedLiveStep: ...


class VerifiedProposalDispatcher(Protocol):
    """Separately invoked public-action adapter; returns independent outcome evidence."""

    def dispatch(self, proposal: DroneTaskProposal, evidence_dir: Path) -> DroneOutcomeVerification: ...


Compiler = Callable[[PlanProposal, CosmosReasoningInput], DroneDecision]


class AuthorizedLiveMission:
    """Run only an explicitly approved, scope-bounded sequence of single C05 actions.

    A restart is fail-closed: this object refuses an already-started cycle because the
    in-memory mission approval and active adapter state cannot be reconstructed safely.
    The caller must create a new authorization and fresh cycle after reconciliation.
    """

    def __init__(self, cycle: LiveReplanCycle, authorization: MissionAuthorization,
                 source: LiveEntityContextSource, provider: LiveInferenceProvider,
                 compiler: Compiler, dispatcher: VerifiedProposalDispatcher,
                 feasibility: DynamicFeasibilityEvaluator | None = None,
                 *, stop_generation: int = 0):
        if (cycle.state["task_id"] != authorization.task_id
                or cycle.state["task_revision"] != authorization.task_revision):
            raise ValueError("Mission authorization must match the immutable live cycle task.")
        if cycle.phase.value != "AWAITING_OBSERVATION":
            raise ValueError("Existing live cycles require reconciliation before a mission can resume.")
        self.cycle = cycle
        self.authorization = authorization
        self.source = source
        self.provider = provider
        self.compiler = compiler
        self.dispatcher = dispatcher
        self.feasibility = feasibility or FailClosedFeasibilityEvaluator()
        if stop_generation < 0:
            raise ValueError("Stop generation must be nonnegative.")
        self.stop_generation = stop_generation
        self.admission = SingleUseAdmission()
        self.authorized = False
        self.actions_completed = 0
        self._context: CosmosReasoningInput | None = None

    def propose_initial_action(self) -> dict:
        if self.authorized or self.actions_completed:
            raise ValueError("Initial mission proposal is unavailable after authorization or execution.")
        return self._capture_and_propose()

    def approve_mission(self) -> dict:
        """Approve the already presented first action and its declared bounded scope."""
        action = self._current_action()
        self._validate_scope_and_evidence(action)
        self.cycle.mark_reviewed(action["id"])
        self.authorized = True
        return {"state": self.cycle.phase.value, "action": action,
                "mission_authorized": True, "execution_dispatch": False}

    def dispatch_and_replan(self) -> dict:
        """Dispatch one approved action, then automatically re-observe/replan if verified.

        This method is the only place that invokes the injected dispatcher.  It never
        grants authority itself; a caller must first call ``approve_mission`` and the
        injected adapter is responsible for its own C06/C07/C08 checks.
        """
        if not self.authorized or self.cycle.phase.value != "OUTCOME_REQUIRED":
            raise ValueError("A mission-approved action awaiting outcome is required.")
        action = self._current_action()
        self._validate_scope_and_evidence(action)
        step_dir = self._step_dir()
        try:
            proposal = self._compile_current_action(action)
        except Exception as error:
            _write_json(step_dir / "dispatch-refusal.json", {
                "action_id": action["id"], "reason": "compiler_exception:" + type(error).__name__,
                "execution_dispatch": False,
            })
            return self._halt(action["id"], "compiler_exception:" + type(error).__name__)
        _write_json(step_dir / "compiled-proposal.json", proposal.model_dump(mode="json"))
        observation = json.loads((step_dir / "observation.json").read_text(encoding="utf-8"))
        scene_state = json.loads((step_dir / "scene-state.json").read_text(encoding="utf-8"))
        try:
            feasibility = self.feasibility.evaluate(
                action, proposal, self._context, observation, scene_state,
                stop_generation=self.stop_generation,
            )
            _write_json(step_dir / "feasibility.json", feasibility.model_dump(mode="json"))
            admission = self.admission.consume(
                feasibility, proposal, self._context, observation, scene_state,
                stop_generation=self.stop_generation,
                now_monotonic_s=time.monotonic(),
            )
            _write_json(step_dir / "admission.json", {
                **admission, "execution_requested": True,
            })
            _write_json(step_dir / "dispatch-intent.json", {
                "schema_version": "rrm-bounded-dispatch-intent/v1",
                "task_id": proposal.task_id,
                "action_id": proposal.action_id,
                "proposal_sha256": admission["proposal_sha256"],
                "feasibility_result_id": admission["result_id"],
                "stop_generation": self.stop_generation,
                "action_index": self.actions_completed,
                "maximum_actions": self.authorization.max_actions,
                "execution_dispatch": True,
            })
        except Exception as error:
            _write_json(step_dir / "dispatch-refusal.json", {
                "action_id": action["id"],
                "reason": "feasibility_not_admitted:" + type(error).__name__,
                "detail": str(error),
                "execution_dispatch": False,
            })
            return self._halt(action["id"], "feasibility_not_admitted:" + type(error).__name__)
        try:
            outcome = self.dispatcher.dispatch(proposal, step_dir)
        except Exception as error:
            return self._halt(action["id"], "dispatcher_exception:" + type(error).__name__)
        if not isinstance(outcome, DroneOutcomeVerification):
            return self._halt(action["id"], "dispatcher_invalid_outcome")
        _write_json(step_dir / "dispatch-outcome.json", outcome.model_dump(mode="json"))
        verified = self._valid_verified_outcome(outcome, proposal)
        result = self.cycle.record_outcome(
            action["id"], verified=verified,
            detail=("independent_adapter_outcome_verified" if verified
                    else f"independent_adapter_outcome_{outcome.verdict.value.lower()}"),
        )
        if not verified:
            return {**result, "reason": "action_outcome_not_verified", "next_action": None,
                    "action_dispatched": True, "execution_dispatch": False}
        self.actions_completed += 1
        if self.actions_completed >= self.authorization.max_actions:
            self.cycle.state.update({"state": "HALTED", "halt_reason": "mission_action_budget_exhausted"})
            self.cycle._persist()
            return {"state": "HALTED", "reason": "mission_action_budget_exhausted",
                    "next_action": None, "action_dispatched": True,
                    "execution_dispatch": False}
        result = self._capture_and_propose()
        if result.get("next_action") is None:
            return {**result, "execution_dispatch": False}
        next_action = self._current_action()
        self._validate_scope_and_evidence(next_action)
        # One explicit mission authorization covers only this bounded, revalidated scope.
        self.cycle.mark_reviewed(next_action["id"])
        return {"state": self.cycle.phase.value, "next_action": next_action,
                "automatic_replan_after_verified_outcome": True, "execution_dispatch": False}

    def _capture_and_propose(self) -> dict:
        if self.cycle.phase.value != "AWAITING_OBSERVATION":
            raise ValueError("A verified prior outcome is required before a fresh live capture.")
        step = self.source.capture()
        if (step.context.task.task_id != self.authorization.task_id
                or step.context.task.revision != self.authorization.task_revision):
            raise ValueError("Entity-verifier context does not match the authorized task.")
        self.cycle.record_observation(step.metadata, step.image, step.scene_state)
        self._context = step.context
        return self.cycle.request_next_action(step.context, self.provider)

    def _current_action(self) -> dict:
        if self._context is None or self.cycle.phase.value not in {"REVIEW_REQUIRED", "OUTCOME_REQUIRED"}:
            raise ValueError("A fresh accepted C05 action is required.")
        candidate = json.loads((self._step_dir() / "provider-response.json").read_text(encoding="utf-8"))["candidate"]
        plan = candidate.get("plan") or {}
        actions = plan.get("actions") or []
        active = self.cycle.state.get("active_action_id")
        for item in actions:
            action = item.get("action", {})
            if action.get("id") == active:
                return action
        raise ValueError("The active C05 action is missing from persisted provider evidence.")

    def _validate_scope_and_evidence(self, action: dict) -> None:
        if action.get("verb") not in self.authorization.allowed_verbs:
            raise ValueError("C05 action verb is outside the mission authorization.")
        targets = action.get("targets")
        if not isinstance(targets, list):
            raise ValueError("C05 action targets are outside the mission authorization.")
        if action.get("verb") == "TAKEOFF":
            if targets:
                raise ValueError("TAKEOFF must not carry semantic targets.")
            if self._context is None:
                raise ValueError("Current C02 verifier context is unavailable.")
            return
        if not targets or not set(targets) <= self.authorization.allowed_targets:
            raise ValueError("C05 action targets are outside the mission authorization.")
        if self._context is None:
            raise ValueError("Current C02 verifier context is unavailable.")
        verified = set(json.loads((self._step_dir() / "scene-state.json").read_text())["verified_entities"])
        for target in targets:
            if target not in verified:
                raise ValueError("C05 target is absent from the current entity-verifier record.")
            if self._context.snapshot.resolve(FactKey(subject=target, predicate="localized"),
                                              now_monotonic_s=self._context.now_monotonic_s) is not Truth.TRUE:
                raise ValueError("C05 target lacks fresh localized TRUE evidence.")

    def _compile_current_action(self, action: dict) -> DroneTaskProposal:
        if self._context is None:
            raise ValueError("Current C02 verifier context is unavailable.")
        candidate = json.loads((self._step_dir() / "provider-response.json").read_text(encoding="utf-8"))["candidate"]
        full_plan = PlanProposal.model_validate(candidate["plan"])
        selected = next(item for item in full_plan.actions if item.action.id == action["id"])
        decision = self.compiler(full_plan.model_copy(update={"actions": (selected,)}), self._context)
        if decision.status is not DroneDecisionStatus.READY or decision.proposal is None:
            raise ValueError("C05 action could not be compiled into a fresh drone proposal.")
        if decision.proposal.action_id != action["id"]:
            raise ValueError("Compiled proposal action identity does not match C05 action.")
        return decision.proposal

    def _valid_verified_outcome(self, outcome: DroneOutcomeVerification,
                                proposal: DroneTaskProposal) -> bool:
        return (outcome.task_id == proposal.task_id and outcome.action_id == proposal.action_id
                and outcome.kind is proposal.kind and outcome.verdict is DroneOutcomeVerdict.VERIFIED)

    def _halt(self, action_id: str, detail: str) -> dict:
        result = self.cycle.record_outcome(action_id, verified=False, detail=detail)
        return {**result, "reason": detail, "next_action": None, "execution_dispatch": False}

    def _step_dir(self) -> Path:
        return self.cycle.root / "steps" / f"{self.cycle.state['active_step_index']:04d}"


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
