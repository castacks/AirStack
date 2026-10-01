"""RRM-1 — Robotics Reasoning Model reference architecture.

A modular embodied-reasoning stack: perception feeds a semantic world model, a
reasoner plans over symbols, a deterministic verifier gates every action, and a
VLA policy converts verbs into motion.

See docs/architecture.md for the design and docs/benchmarks.md for what is measured.
"""

from .contracts import ApprovalDecision, ApprovalProvider, ApprovalScope, Truth
from .schema import (
    SELF,
    WILDCARD,
    AbstractAction,
    AxisLimit,
    ActionPolicy,
    Divergence,
    ObjectID,
    NumericLimitProfile,
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
    predicate_truth,
    preconditions_of,
)
from .safety import NumericSafetyVerifier, SafetyVerifier
from .reasoning import ReasonerBackend, ScriptedOracle
from .policy import MockPolicy
from .world import MockWorld
from .trace import Tracer
from .core_admission import CoreAdmission, SafetyDecisionProvider
from .loop import (
    CYCLE_BUDGET, REPLAN_BUDGET, THETA_DIV, THETA_UNC,
    check_divergence, dispatch, run,
)
from .benchmark import TASKS, run_suite

__all__ = [
    "SELF", "WILDCARD", "Truth", "ApprovalDecision", "ApprovalProvider",
    "ApprovalScope", "AbstractAction", "ActionPolicy", "Divergence", "ObjectID",
    "AxisLimit", "NumericLimitProfile",
    "Predicate", "Provenance", "Relation", "RobotState", "RunMetrics", "SafetyVerdict",
    "Task", "TaskGraph", "Termination", "Trajectory", "Verb", "VerbSpec", "Violation",
    "WorldBackend", "WorldObject", "WorldState",
    "VERB_TABLE", "bind", "expected_effects_of", "holds", "predicate_truth",
    "preconditions_of",
    "NumericSafetyVerifier", "SafetyVerifier", "CoreAdmission", "SafetyDecisionProvider",
    "ReasonerBackend", "ScriptedOracle", "MockPolicy", "MockWorld", "Tracer",
    "CYCLE_BUDGET", "REPLAN_BUDGET", "THETA_DIV", "THETA_UNC",
    "check_divergence", "dispatch", "run",
    "TASKS", "run_suite",
]
