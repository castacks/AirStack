"""Scene-independent adjudication labels for the core RRM benchmark."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


LABEL_SCHEMA = "rrm-core-benchmark-label/v1"


class SafetyLabel(str, Enum):
    SAFE = "SAFE"
    UNSAFE = "UNSAFE"


class FailureKind(str, Enum):
    NONE = "NONE"
    TRANSIENT_EFFECT = "TRANSIENT_EFFECT"
    PERSISTENT_EFFECT = "PERSISTENT_EFFECT"


class TerminalLabel(str, Enum):
    GOAL_VERIFIED = "GOAL_VERIFIED"
    SAFE_ABORT = "SAFE_ABORT"


@dataclass(frozen=True)
class EvaluationLabels:
    """Frozen semantic labels; intentionally contains no scene or geometry fields."""

    symbolic_safety: SafetyLabel
    numeric_safety: SafetyLabel
    failure_kind: FailureKind
    failure_recoverable: bool | None
    expected_terminal: TerminalLabel

    def __post_init__(self) -> None:
        if not isinstance(self.symbolic_safety, SafetyLabel) \
                or not isinstance(self.numeric_safety, SafetyLabel):
            raise ValueError("safety labels must use the SafetyLabel vocabulary")
        if not isinstance(self.failure_kind, FailureKind):
            raise ValueError("failure_kind must use the FailureKind vocabulary")
        if not isinstance(self.expected_terminal, TerminalLabel):
            raise ValueError("expected_terminal must use the TerminalLabel vocabulary")
        if self.failure_recoverable is not None \
                and type(self.failure_recoverable) is not bool:
            raise ValueError("failure_recoverable must be boolean or null")
        if self.failure_kind is FailureKind.NONE and self.failure_recoverable is not None:
            raise ValueError("a scenario without an injected failure cannot label recoverability")
        if self.failure_kind is FailureKind.TRANSIENT_EFFECT \
                and self.failure_recoverable is not True:
            raise ValueError("a transient effect failure must be labelled recoverable")
        if self.failure_kind is FailureKind.PERSISTENT_EFFECT \
                and self.failure_recoverable is not False:
            raise ValueError("a persistent effect failure must be labelled non-recoverable")
        if self.expected_terminal is TerminalLabel.GOAL_VERIFIED \
                and self.symbolic_safety is SafetyLabel.UNSAFE:
            raise ValueError("a symbolically unsafe scenario cannot expect a verified goal")
        if self.failure_kind is FailureKind.PERSISTENT_EFFECT \
                and self.expected_terminal is not TerminalLabel.SAFE_ABORT:
            raise ValueError("a persistent effect failure must expect a safe abort")

    def as_record(self) -> dict[str, Any]:
        return {
            "schema_version": LABEL_SCHEMA,
            "symbolic_safety": self.symbolic_safety.value,
            "numeric_safety": self.numeric_safety.value,
            "failure_kind": self.failure_kind.value,
            "failure_recoverable": self.failure_recoverable,
            "expected_terminal": self.expected_terminal.value,
        }

    @classmethod
    def from_record(cls, value: Any) -> "EvaluationLabels":
        if not isinstance(value, dict):
            raise ValueError("evaluation labels must be an object")
        required = {
            "schema_version", "symbolic_safety", "numeric_safety",
            "failure_kind", "failure_recoverable", "expected_terminal",
        }
        if set(value) != required:
            raise ValueError("evaluation labels have missing or unknown fields")
        if value["schema_version"] != LABEL_SCHEMA:
            raise ValueError("unsupported evaluation-label schema")
        recoverable = value["failure_recoverable"]
        if recoverable is not None and type(recoverable) is not bool:
            raise ValueError("failure_recoverable must be boolean or null")
        try:
            return cls(
                symbolic_safety=SafetyLabel(value["symbolic_safety"]),
                numeric_safety=SafetyLabel(value["numeric_safety"]),
                failure_kind=FailureKind(value["failure_kind"]),
                failure_recoverable=recoverable,
                expected_terminal=TerminalLabel(value["expected_terminal"]),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid evaluation labels: {exc}") from exc
