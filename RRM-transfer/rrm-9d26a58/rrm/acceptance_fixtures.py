"""Authored mock campaign fixtures, never live-adapter or scene authority."""

from dataclasses import asdict, dataclass
import json
import os
from threading import Lock

from .benchmark import CUP, TABLE, MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE
from .benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from .core_deadlines import CoreCallLimits
from .policy import MockPolicy
from .reasoning import ScriptedOracle
from .schema import SELF, Task, WorldObject
from .trace import Tracer
from .uncertainty import aggregate_uncertainty
from .verbs import _p
from .world import MockWorld
from .safety_event_evidence import RULE_SCHEMA, STAGES, expected_label


LIMITS = CoreCallLimits(reasoner_s=0.1, policy_s=0.1, observation_s=0.1,
                        adapter_s=0.1, stop_s=0.1, evidence_s=0.1)


@dataclass(frozen=True)
class Scenario:
    id: str
    mode: str
    goal_met: bool = False
    task_success: bool = True
    aborted: bool = True
    applies: int = 1
    replans: int = 0
    stop_status: str = "SAFE_CONFIRMED_MOCK"
    dynamic_verdicts: tuple[str, ...] | None = None
    error: str | None = None
    stop_reason: str | None = None
    failure_event: tuple[str, str] | None = None

    def record(self):
        return asdict(self)


# Expectations are authored before execution, not derived from the observed verdict.
SCENARIOS = (
    Scenario("nominal-pick", "nominal", True, True, False, 3, 0, "NOT_REQUESTED"),
    Scenario("nominal-place", "place", True, True, False, 6, 0, "NOT_REQUESTED"),
    Scenario("unsafe-before-dispatch", "pre_human", applies=0, replans=1, stop_status="NOT_REQUESTED"),
    Scenario("recoverable-effect", "transient", True, True, False, 9, 1, "NOT_REQUESTED"),
    Scenario("persistent-effect", "persistent", applies=24, replans=3, stop_status="NOT_REQUESTED"),
    Scenario("active-human", "human", dynamic_verdicts=("PASS", "FAIL"), stop_reason="dynamic_symbolic_safety"),
    Scenario("displaced-target", "displacement", dynamic_verdicts=("PASS", "FAIL"), stop_reason="dynamic_symbolic_safety"),
    Scenario("weak-evidence", "weak", stop_reason="active_uncertainty"),
    Scenario("stale-coverage", "stale", stop_reason="active_uncertainty"),
    Scenario("observation-loss", "observation", task_success=False, stop_reason="observation_unavailable"),
    Scenario("numeric-rejection", "numeric", applies=0, stop_reason="active_numeric_safety"),
    Scenario("policy-stall", "policy", task_success=False, stop_reason="execution_fault",
             failure_event=("execution_fault", "policy_step")),
    Scenario("application-exception", "apply", task_success=False, stop_reason="execution_fault",
             failure_event=("execution_fault", "apply")),
    Scenario("planning-stall", "plan", task_success=False, applies=0, stop_status="NOT_REQUESTED",
             failure_event=("reasoner_failure", "plan")),
    Scenario("replanning-stall", "replan", task_success=False, applies=6, replans=1,
             stop_status="NOT_REQUESTED", failure_event=("reasoner_failure", "replan")),
    Scenario("cancel-unconfirmed", "no_ack", stop_status="SAFE_UNCONFIRMED", stop_reason="active_uncertainty"),
    Scenario("evidence-write-loss", "trace", task_success=False, applies=0,
             stop_status="SAFE_UNCONFIRMED", error="CoreEvidenceUnavailable"),
)


def case_by_id(case_id):
    return next(case for case in SCENARIOS if case.id == case_id)


def safety_rule(case):
    """Authored conditions, not a call to the verifier or its observed verdict."""
    stages = {stage: [{"from_sim_t": 0, "safety": "SAFE"}] for stage in STAGES.values()}
    if case.mode == "pre_human":
        for stage in stages:
            stages[stage][0]["safety"] = "UNSAFE"
    if case.mode == "numeric":
        stages["numeric"][0]["safety"] = "UNSAFE"
    if case.mode in {"human", "displacement"}:
        for stage in ("symbolic", "dynamic_symbolic"):
            stages[stage].append({"from_sim_t": 1, "safety": "UNSAFE"})
        stages["numeric"].append({"from_sim_t": 1,
                                  "safety": "UNSAFE" if case.mode == "human" else "UNKNOWN"})
    return {"schema_version": RULE_SCHEMA, "source": "authored_mock_fixture", "stages": stages}


def task_and_labels(case, attempt_id):
    place = case.mode == "place"
    task = Task(id=attempt_id, mission="place cup" if place else "pick cup",
                goal=_p("on", CUP, TABLE) if place else _p("holding", SELF, CUP),
                expect_abort=case.aborted)
    failure = (FailureKind.TRANSIENT_EFFECT if case.mode in {"transient", "replan"}
               else FailureKind.PERSISTENT_EFFECT if case.mode == "persistent" else FailureKind.NONE)
    labels = EvaluationLabels(
        SafetyLabel.UNSAFE if case.mode == "pre_human" else SafetyLabel.SAFE,
        SafetyLabel.UNSAFE if case.mode == "numeric" else SafetyLabel.SAFE,
        failure, True if failure is FailureKind.TRANSIENT_EFFECT else
        False if failure is FailureKind.PERSISTENT_EFFECT else None,
        TerminalLabel.GOAL_VERIFIED if case.goal_met else TerminalLabel.SAFE_ABORT)
    return task, labels


class FixtureWorld(MockWorld):
    def __init__(self, mode):
        super().__init__(fail_grasp_once=mode in {"transient", "replan"},
                         fail_grasp_always=mode == "persistent", human=mode == "pre_human")
        self.mode = mode

    def _apply_active(self, action, trajectory):
        if self.mode == "apply" and self.state.t == 1:
            raise RuntimeError("authored mock application fault")
        super()._apply_active(action, trajectory)
        if self.state.t == 1:
            if self.mode == "human":
                self.state.objects.append(WorldObject(id="entrant", cls="person", pose=(0.42, -0.2, 0.0)))
            elif self.mode == "displacement":
                self.state.get(CUP).pose = (9.0, 9.0, 0.0)

    def observe(self):
        if self.mode == "observation" and self.state.t >= 1:
            raise RuntimeError("authored mock observation loss")
        state = super().observe()
        if state.t >= 1:
            if self.mode in {"weak", "no_ack"}:
                state.get(CUP).confidence = 0.5
            elif self.mode == "stale":
                state.relations_observed_t = state.t - 1
            state.uncertainty = aggregate_uncertainty(state)
        return state

    def cancel_dispatch(self, dispatch_id, generation):
        if self.mode == "no_ack":
            return False
        return super().cancel_dispatch(dispatch_id, generation)


class FixturePolicy(MockPolicy):
    def __init__(self, mode, release):
        super().__init__()
        self.mode, self.release, self.calls = mode, release, 0

    def step(self, action, state):
        self.calls += 1
        if self.mode == "policy" and self.calls == 2:
            self.release.wait()
        trajectory = super().step(action, state)
        if self.mode == "numeric":
            trajectory = trajectory.model_copy(update={"max_velocity": 20.0})
        return trajectory


class FixtureReasoner(ScriptedOracle):
    def __init__(self, goal, mode, release):
        super().__init__(goal)
        self.mode, self.release = mode, release

    def plan(self, mission, state):
        if self.mode == "plan":
            self.release.wait()
        return super().plan(mission, state)

    def replan(self, mission, state, graph, div):
        if self.mode == "replan":
            self.release.wait()
        return super().replan(mission, state, graph, div)


class FixtureTracer(Tracer):
    def __init__(self, path, meta, mode):
        self.mode, self.failed = mode, False
        self._label_lock = Lock()
        self._rule = safety_rule(case_by_id(meta["scenario_id"]))
        self._configuration_hash = meta["configuration_sha256"]
        self._labels = (path.parent / "safety-labels.jsonl").open("x", encoding="utf-8")
        super().__init__(path, meta)

    def event(self, kind, **fields):
        with self._label_lock:
            if self.mode == "trace" and (kind == "safety2" or self.failed):
                self.failed = True
                raise OSError("authored permanent trace-write loss")
            super().event(kind, **fields)
            if kind in STAGES:
                label = expected_label({**fields, "kind": kind, "sequence": self._sequence - 1,
                                        "run_id": self.run_id}, self._rule, self._configuration_hash)
                self._labels.write(json.dumps(label, sort_keys=True) + "\n")
                self._labels.flush()
                os.fsync(self._labels.fileno())

    def close(self):
        with self._label_lock:
            super().close()
            self._labels.close()
