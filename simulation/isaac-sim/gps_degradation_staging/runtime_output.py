"""Latest after-step GPS degradation result shared with the Pegasus GPS sensor."""
from __future__ import annotations

from typing import Optional

from .state_machine import DegradationOutput


_latest_output: Optional[DegradationOutput] = None


def set_latest_output(output: DegradationOutput) -> None:
    global _latest_output
    _latest_output = output


def get_latest_output() -> Optional[DegradationOutput]:
    return _latest_output
