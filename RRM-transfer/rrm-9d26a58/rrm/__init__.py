"""RRM-1 — Robotics Reasoning Model reference architecture.

A modular embodied-reasoning stack: perception feeds a semantic world model, a
reasoner plans over symbols, a deterministic verifier gates every action, and a
VLA policy converts verbs into motion.

See docs/architecture.md for the design and docs/benchmarks.md for what is measured.
"""

from .schema import (
    SELF,
    WILDCARD,
    AbstractAction,
    ActionPolicy,
    Divergence,
    ObjectID,
    Predicate,
    Provenance,
    Relation,
    RobotState,
    RunMetrics,
    SafetyVerdict,
    Task,
    TaskGraph,
    Termination,
    Trajectory,
    Verb,
    VerbSpec,
    Violation,
    WorldBackend,
    WorldObject,
    WorldState,
)
from .verbs import (
    VERB_TABLE,
    bind,
    expected_effects_of,
    holds,
    preconditions_of,
)
from .safety import NumericSafetyVerifier, SafetyVerifier
from .reasoning import ReasonerBackend, ScriptedOracle
from .policy import MockPolicy
from .world import MockWorld
from .trace import Tracer
from .loop import CYCLE_BUDGET, REPLAN_BUDGET, THETA_DIV, check_divergence, dispatch, run
from .benchmark import TASKS, run_suite

__all__ = [
    "SELF", "WILDCARD", "AbstractAction", "ActionPolicy", "Divergence", "ObjectID",
    "Predicate", "Provenance", "Relation", "RobotState", "RunMetrics", "SafetyVerdict",
    "Task", "TaskGraph", "Termination", "Trajectory", "Verb", "VerbSpec", "Violation",
    "WorldBackend", "WorldObject", "WorldState",
    "VERB_TABLE", "bind", "expected_effects_of", "holds", "preconditions_of",
    "NumericSafetyVerifier", "SafetyVerifier",
    "ReasonerBackend", "ScriptedOracle", "MockPolicy", "MockWorld", "Tracer",
    "CYCLE_BUDGET", "REPLAN_BUDGET", "THETA_DIV", "check_divergence", "dispatch", "run",
    "TASKS", "run_suite",
]
