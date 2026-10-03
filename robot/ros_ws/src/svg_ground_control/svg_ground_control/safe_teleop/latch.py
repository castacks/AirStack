"""Hold some axes at their last value until a button is pressed again."""

from __future__ import annotations

from .pad import PadState

FREEZE_BUTTON = 4       # left bumper
FREEZE_AXES = (0, 1)    # left stick only; everything else stays live


class ButtonToggle:
    """A bool that a button press flips. Edge-triggered.

    Use this where a stick drives a RATE. Holding the stick's last position
    would keep ramping the value forever, so the lock has to zero the stick's
    contribution instead -- which is a plain on/off, not a held snapshot.
    """

    def __init__(self, button: int = FREEZE_BUTTON, engaged: bool = False):
        self.button = button
        self.engaged = engaged
        self._was_down = False

    def update(self, live: PadState) -> bool:
        down = live.button(self.button)
        if down and not self._was_down:
            self.engaged = not self.engaged
        self._was_down = down
        return self.engaged


class FreezeLatch:
    """Edge-triggered hold on a subset of axes.

    Only a press flips the state, so holding the button down does not flap it
    once per frame. The pad keeps being read while held -- that is how the
    release press gets noticed.
    """

    def __init__(self, button: int = FREEZE_BUTTON, axes=FREEZE_AXES):
        self.button = button
        self.axes = tuple(axes)
        self.held: PadState | None = None
        self._was_down = False

    def update(self, live: PadState) -> PadState:
        """Take this frame's reading, return the values that should be used."""
        down = live.button(self.button)
        if down and not self._was_down:
            self.held = None if self.held is not None else live
        self._was_down = down
        if self.held is None:
            return live
        axes = list(live.axes)
        raw = list(live.raw_axes)
        for i in self.frozen_axes:
            axes[i] = self.held.axes[i]
            raw[i] = self.held.raw_axes[i]
        return PadState(tuple(axes), tuple(raw), live.buttons, live.connected)

    @property
    def frozen(self) -> bool:
        return self.held is not None

    @property
    def frozen_axes(self) -> set:
        """Which axis numbers are currently being held."""
        if self.held is None:
            return set()
        return {i for i in self.axes if i < len(self.held.axes)}
