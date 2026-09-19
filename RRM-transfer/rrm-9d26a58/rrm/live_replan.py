"""Durable, shadow-only orchestration for live action → observe → replan cycles.

This module contains no ROS, model runtime, simulator, or dispatch dependency.  A
provider may keep Cosmos warm in OSMO or call a hosted API, but it receives only a
fresh, immutable evidence bundle and returns a proposal candidate.  The coordinator
never executes that candidate: even a multi-action proposal exposes only its first
action for explicit per-step review.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
import hashlib
import json
from pathlib import Path
from typing import Protocol
import uuid

from rrm.cosmos_reason2 import CosmosCandidateStatus, CosmosReasoningCandidate, CosmosReasoningInput
from rrm.live_observation import validate_live_observation


class CycleState(str, Enum):
    AWAITING_OBSERVATION = "AWAITING_OBSERVATION"
    AWAITING_PROVIDER = "AWAITING_PROVIDER"
    REVIEW_REQUIRED = "REVIEW_REQUIRED"
    OUTCOME_REQUIRED = "OUTCOME_REQUIRED"
    HALTED = "HALTED"


@dataclass(frozen=True)
class LiveCycleRequest:
    """One provider request, cryptographically bound to exactly one live capture."""

    cycle_id: str
    step_index: int
    context: CosmosReasoningInput
    observation: dict
    image_path: Path
    scene_state: dict
    prior_outcome: dict | None
    prior_plan_action_ids: tuple[str, ...]


@dataclass(frozen=True)
class LiveCycleResponse:
    """A provider's candidate for one observation, not an execution authorization."""

    cycle_id: str
    step_index: int
    observation_sha256: str
    candidate: CosmosReasoningCandidate


class LiveInferenceProvider(Protocol):
    """Implemented later by a warm OSMO Cosmos worker or a credentialed VLM adapter."""

    def propose(self, request: LiveCycleRequest) -> LiveCycleResponse: ...


class LiveReplanCycle:
    """Persist one shadow-only mission cycle with mandatory per-step fresh evidence."""

    def __init__(self, root: Path, *, task_id: str, task_revision: str,
                 expected_camera_frame: str):
        if not task_id or not task_revision or not expected_camera_frame:
            raise ValueError("Cycle task identity and expected camera frame are required.")
        self.root = root.resolve()
        self.manifest_path = self.root / "cycle.json"
        if self.manifest_path.exists():
            self.state = _read_json(self.manifest_path)
            if (self.state.get("task_id") != task_id or self.state.get("task_revision") != task_revision or
                    self.state.get("expected_camera_frame") != expected_camera_frame):
                raise ValueError("Cycle directory belongs to a different immutable task.")
        else:
            self.root.mkdir(parents=True, exist_ok=False)
            self.state = {
                "schema_version": "rrm-live-replan-cycle/v1",
                "cycle_id": uuid.uuid4().hex,
                "task_id": task_id,
                "task_revision": task_revision,
                "expected_camera_frame": expected_camera_frame,
                "state": CycleState.AWAITING_OBSERVATION.value,
                "next_step_index": 0,
                "execution_dispatch": False,
            }
            self._persist()

    @property
    def cycle_id(self) -> str:
        return self.state["cycle_id"]

    @property
    def phase(self) -> CycleState:
        return CycleState(self.state["state"])

    def record_observation(self, metadata: dict, image: bytes, scene_state: dict) -> dict:
        """Bind a fresh camera/state capture before each provider invocation.

        `scene_state` is deliberately a separate, future live entity-verifier output.
        A camera image alone is not treated as proof that a target exists or is safe.
        """
        if self.phase is not CycleState.AWAITING_OBSERVATION:
            raise ValueError("A verified outcome is required before the next live observation.")
        observation = validate_live_observation(
            metadata, expected_camera_frame=self.state["expected_camera_frame"]
        )
        if hashlib.sha256(image).hexdigest() != observation["sha256"]:
            raise ValueError("Live cycle image does not match its observation checksum.")
        _validate_scene_state(scene_state, observation)
        step_index = self.state["next_step_index"]
        directory = self._step_dir(step_index)
        directory.mkdir(parents=True, exist_ok=False)
        (directory / "input.png").write_bytes(image)
        _write_json(directory / "observation.json", observation)
        _write_json(directory / "scene-state.json", scene_state)
        self.state.update({
            "state": CycleState.AWAITING_PROVIDER.value,
            "active_step_index": step_index,
            "active_observation_sha256": observation["sha256"],
            "active_scene_state_sha256": _sha256(directory / "scene-state.json"),
        })
        self._persist()
        return observation

    def request_next_action(self, context: CosmosReasoningInput,
                            provider: LiveInferenceProvider) -> dict:
        """Ask one provider for a response bound to the active live observation."""
        if self.phase is not CycleState.AWAITING_PROVIDER:
            raise ValueError("A fresh live observation is required before replanning.")
        if context.task.task_id != self.state["task_id"] or context.task.revision != self.state["task_revision"]:
            raise ValueError("Provider context does not match this immutable cycle task.")
        directory = self._step_dir(self.state["active_step_index"])
        observation = _read_json(directory / "observation.json")
        request = LiveCycleRequest(
            cycle_id=self.cycle_id,
            step_index=self.state["active_step_index"],
            context=context,
            observation=observation,
            image_path=directory / "input.png",
            scene_state=_read_json(directory / "scene-state.json"),
            prior_outcome=self._prior_outcome(),
            prior_plan_action_ids=tuple(self.state.get("prior_plan_action_ids", ())),
        )
        response = provider.propose(request)
        self._validate_response(response, request)
        _write_json(directory / "provider-request.json", {
            "cycle_id": request.cycle_id, "step_index": request.step_index,
            "observation_sha256": observation["sha256"],
            "prior_plan_action_ids": request.prior_plan_action_ids,
            "prior_outcome": request.prior_outcome,
            "execution_dispatch": False,
        })
        _write_json(directory / "provider-response.json", {
            "cycle_id": response.cycle_id, "step_index": response.step_index,
            "observation_sha256": response.observation_sha256,
            "candidate": response.candidate.model_dump(mode="json"),
            "execution_dispatch": False,
        })
        if response.candidate.status is not CosmosCandidateStatus.ACCEPTED:
            self.state.update({"state": CycleState.HALTED.value,
                               "halt_reason": "provider_candidate_not_accepted"})
            self._persist()
            return {"state": self.phase.value, "next_action": None,
                    "reason": "provider_candidate_not_accepted"}
        actions = response.candidate.plan.actions
        # The cycle intentionally preserves the complete proposal for reasoning, but
        # presents exactly one action for review. The rest can never auto-dispatch.
        next_action = actions[0]
        self.state.update({
            "state": CycleState.REVIEW_REQUIRED.value,
            "active_action_id": next_action.action.id,
            "prior_plan_action_ids": [item.action.id for item in actions],
            "remaining_action_ids": [item.action.id for item in actions[1:]],
        })
        self._persist()
        return {"state": self.phase.value, "next_action": next_action.model_dump(mode="json"),
                "remaining_action_ids": self.state["remaining_action_ids"],
                "execution_dispatch": False}

    def mark_reviewed(self, action_id: str) -> dict:
        """Record review only; dispatch belongs to a later, separately approved adapter."""
        if self.phase is not CycleState.REVIEW_REQUIRED or action_id != self.state.get("active_action_id"):
            raise ValueError("Only the current next action can be marked reviewed.")
        directory = self._step_dir(self.state["active_step_index"])
        _write_json(directory / "review.json", {
            "cycle_id": self.cycle_id, "step_index": self.state["active_step_index"],
            "action_id": action_id, "reviewed_at": _now(), "execution_dispatch": False,
        })
        self.state["state"] = CycleState.OUTCOME_REQUIRED.value
        self._persist()
        return {"state": self.phase.value, "execution_dispatch": False}

    def record_outcome(self, action_id: str, *, verified: bool, detail: str) -> dict:
        """Require an independently recorded outcome before another replan cycle."""
        if self.phase is not CycleState.OUTCOME_REQUIRED or action_id != self.state.get("active_action_id"):
            raise ValueError("Only the reviewed action can receive an outcome.")
        if not isinstance(detail, str) or not detail.strip() or len(detail) > 500:
            raise ValueError("Outcome detail is required and must be concise.")
        directory = self._step_dir(self.state["active_step_index"])
        _write_json(directory / "outcome.json", {
            "cycle_id": self.cycle_id, "step_index": self.state["active_step_index"],
            "action_id": action_id, "verified": verified, "detail": detail.strip(),
            "recorded_at": _now(), "execution_dispatch": False,
        })
        if not verified:
            self.state.update({"state": CycleState.HALTED.value,
                               "halt_reason": "action_outcome_not_verified"})
        else:
            self.state.update({"state": CycleState.AWAITING_OBSERVATION.value,
                               "next_step_index": self.state["active_step_index"] + 1})
        self._persist()
        return {"state": self.phase.value, "execution_dispatch": False}

    def _validate_response(self, response: LiveCycleResponse, request: LiveCycleRequest) -> None:
        if not isinstance(response, LiveCycleResponse):
            raise ValueError("Live provider returned an invalid response object.")
        if (response.cycle_id != self.cycle_id or response.step_index != request.step_index or
                response.observation_sha256 != request.observation["sha256"]):
            raise ValueError("Live provider response is not bound to the active observation.")
        if response.candidate.task_id != self.state["task_id"]:
            raise ValueError("Live provider candidate belongs to a different task.")

    def _prior_outcome(self) -> dict | None:
        step_index = self.state.get("active_step_index", 0) - 1
        path = self._step_dir(step_index) / "outcome.json"
        return _read_json(path) if step_index >= 0 and path.is_file() else None

    def _step_dir(self, step_index: int) -> Path:
        return self.root / "steps" / f"{step_index:04d}"

    def _persist(self) -> None:
        _write_json(self.manifest_path, self.state)


def _validate_scene_state(scene_state: dict, observation: dict) -> None:
    required = {"source_stamp_ns", "verified_entities", "provenance"}
    if not isinstance(scene_state, dict) or not required.issubset(scene_state):
        raise ValueError("Live scene state must identify verified entities and provenance.")
    if type(scene_state["source_stamp_ns"]) is not int or scene_state["source_stamp_ns"] < observation["source_stamp_ns"]:
        raise ValueError("Live scene state is older than the camera observation.")
    entities = scene_state["verified_entities"]
    if not isinstance(entities, list) or not entities or not all(isinstance(item, str) and item for item in entities):
        raise ValueError("Live scene state must contain verified entity IDs.")
    if not isinstance(scene_state["provenance"], str) or not scene_state["provenance"].strip():
        raise ValueError("Live scene state provenance is required.")


def _write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _read_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("Cycle evidence must be a JSON object.")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()
