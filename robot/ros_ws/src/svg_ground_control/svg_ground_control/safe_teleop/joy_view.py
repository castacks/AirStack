"""Live table of a /joy topic, in the same layout as view.py.

view.py reads /dev/input directly, so it tells you what the pad is doing. This
one subscribes to sensor_msgs/Joy, so it tells you what actually reached ROS —
the whole chain from the pad through joy_node to this container.

    ros2 run svg_ground_control joy_topic_view
    ros2 run svg_ground_control joy_topic_view --ros-args -p topic:=/joy

Run it next to view.py to see whether joy_node is changing the values. It is,
in two ways:

* It negates every axis. A stick pushed right reads positive on the device and
  negative on /joy. This is why teleop_node's forward_sign, left_sign and
  climb_sign all default to -1.0 — they undo it.
* It applies its own deadzone, so a stick barely off center reads exactly
  0.000 here while view.py still shows the raw counts.

So the direction words and trigger percentages below follow the /joy
convention, not view.py's device convention. The two tools will disagree on
sign for the same physical stick position, and that is correct.
"""

from __future__ import annotations

import curses
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .latch import FREEZE_BUTTON, FreezeLatch
from .pad import (AXIS_FULL_SCALE, XBOX_AXIS_NAMES, XBOX_BUTTON_NAMES,
                  XBOX_TRIGGER_AXES, PadState)
from .view import BAR_WIDTH

FRAME_MS = 16           # ~60 redraws a second, same as view.py
RATE_WINDOW = 50        # messages averaged for the displayed rate
STALE_AFTER_S = 1.0     # no message for this long and the table is called stale
CENTERED = 0.02         # deflection under this counts as centered

# Which way each Xbox axis reads when it goes positive ON /joy. These are the
# opposite of view.py's, because joy_node negates every axis.
AXIS_POSITIVE = {0: "left", 1: "up", 3: "left", 4: "up", 6: "left", 7: "up"}
AXIS_NEGATIVE = {0: "right", 1: "down", 3: "right", 4: "down",
                 6: "right", 7: "down"}


def trigger_fraction(value: float) -> float:
    """A /joy trigger reading as 0.0 released .. 1.0 fully squeezed.

    Not pad.trigger_fraction: on /joy a trigger rests at +1.0 and runs to -1.0,
    the negation of what the device reports.
    """
    return max(0.0, min(1.0, (1.0 - value) / 2.0))


def axis_state(labels, number: int, value: float) -> str:
    """Where one axis is sitting, in words."""
    if labels.is_trigger(number):
        return f"{trigger_fraction(value):.0%} squeezed"
    if abs(value) < CENTERED:
        return "centered"
    if number not in AXIS_POSITIVE or not labels.xbox_layout:
        return "positive" if value > 0 else "negative"
    return AXIS_POSITIVE[number] if value > 0 else AXIS_NEGATIVE[number]


def bar_length(labels, number: int, value: float) -> int:
    """How far to fill the bar. A trigger at +1.0 is released, so it draws empty."""
    magnitude = (trigger_fraction(value) if labels.is_trigger(number)
                 else abs(value))
    return int(magnitude * BAR_WIDTH)


class JoyLabels:
    """Names for /joy channels, when there is no device to ask.

    sensor_msgs/Joy carries no channel map, so the layout is inferred from the
    array lengths. An 8-axis, 11-button pad is the xpad mapping pad.py names.
    Anything else falls back to bare channel numbers.
    """

    def __init__(self, axis_count: int, button_count: int):
        self.xbox_layout = (axis_count == len(XBOX_AXIS_NAMES)
                            and button_count == len(XBOX_BUTTON_NAMES))

    def axis_label(self, index: int) -> str:
        if self.xbox_layout and index < len(XBOX_AXIS_NAMES):
            return XBOX_AXIS_NAMES[index]
        return f"Axis {index}"

    def button_label(self, index: int) -> str:
        if self.xbox_layout and index < len(XBOX_BUTTON_NAMES):
            return XBOX_BUTTON_NAMES[index]
        return f"Button {index}"

    def is_trigger(self, index: int) -> bool:
        return self.xbox_layout and index in XBOX_TRIGGER_AXES


class JoyWatcher(Node):
    """Holds the newest Joy message and how fast they are arriving."""

    def __init__(self):
        super().__init__("joy_topic_view")
        self.topic = self.declare_parameter("topic", "/joy").value
        self.latest: Joy | None = None
        self.count = 0
        self.last_seen = 0.0
        self._arrivals: deque[float] = deque(maxlen=RATE_WINDOW)
        self.create_subscription(Joy, self.topic, self._on_joy, 10)

    def _on_joy(self, message: Joy) -> None:
        # Arrival time, not header.stamp: this is about the delivery chain, and
        # a publisher on sim time would put the table on the wrong clock.
        now = time.monotonic()
        self.latest = message
        self.count += 1
        self.last_seen = now
        self._arrivals.append(now)

    @property
    def age(self) -> float:
        """Seconds since the last message, or -1 before the first one."""
        return -1.0 if self.count == 0 else time.monotonic() - self.last_seen

    @property
    def rate(self) -> float:
        """Messages a second over the recent window."""
        if len(self._arrivals) < 2:
            return 0.0
        span = self._arrivals[-1] - self._arrivals[0]
        return (len(self._arrivals) - 1) / span if span > 0 else 0.0


def as_pad_state(message: Joy) -> PadState:
    """A Joy message as a PadState, so FreezeLatch works on it unchanged.

    raw_axes is reconstructed from the normalized value only because PadState
    carries the field and the latch copies it. Nothing here reads it.
    """
    axes = tuple(float(v) for v in message.axes)
    return PadState(
        axes=axes,
        raw_axes=tuple(int(v * AXIS_FULL_SCALE) for v in axes),
        buttons=tuple(bool(b) for b in message.buttons),
        connected=True,
    )


def render(screen, watcher: JoyWatcher, labels, state, frozen_axes=()) -> None:
    screen.erase()
    height, width = screen.getmaxyx()
    frozen_axes = set(frozen_axes)
    row = 0

    def line(text: str = "") -> None:
        nonlocal row
        if row < height:
            screen.addnstr(row, 0, text, max(0, width - 1))
        row += 1

    lock = labels.button_label(FREEZE_BUTTON) if labels else f"button {FREEZE_BUTTON}"
    if watcher.count == 0:
        line(f"WAITING for the first message on {watcher.topic} — "
             "is joy_node running? Ctrl-C to quit")
    elif frozen_axes:
        names = ", ".join(labels.axis_label(i) for i in sorted(frozen_axes))
        line(f"HELD: {names} — press {lock} to release, Ctrl-C to quit")
    elif watcher.age > STALE_AFTER_S:
        line(f"STALE — nothing on {watcher.topic} for {watcher.age:.1f}s. "
             "joy_node publishes on change, so a still pad is normal.")
    else:
        line(f"Live {watcher.topic} — press {lock} to hold the left stick, "
             "Ctrl-C to quit")

    if watcher.count == 0:
        line()
    else:
        line(f"{watcher.count} messages   {watcher.rate:5.1f} Hz   "
             f"last {watcher.age:.2f}s ago")

    if state is None:
        line()
        line("No values yet.")
        screen.refresh()
        return

    line(f"{'Control':<26} {'Channel':<10} {'-1 to +1':>10}  State")
    line("-" * 72)

    for number, value in enumerate(state.axes):
        mark = "HELD " if number in frozen_axes else ""
        line(f"{labels.axis_label(number):<26} {'axis ' + str(number):<10} "
             f"{value:>+10.3f}  "
             f"{mark + axis_state(labels, number, value):<20} "
             f"{'#' * bar_length(labels, number, value)}")

    line("-" * 72)
    for number, pressed in enumerate(state.buttons):
        line(f"{labels.button_label(number):<26} {'button ' + str(number):<10} "
             f"{'':>10}  {'PRESSED' if pressed else 'released'}")
    screen.refresh()


def run(screen, watcher: JoyWatcher) -> None:
    curses.curs_set(0)
    screen.timeout(FRAME_MS)       # getch doubles as the frame clock
    latch = FreezeLatch()
    labels = None
    state = None
    while rclpy.ok():
        screen.getch()
        rclpy.spin_once(watcher, timeout_sec=0.0)
        if watcher.latest is not None:
            if labels is None:
                labels = JoyLabels(len(watcher.latest.axes),
                                   len(watcher.latest.buttons))
            state = latch.update(as_pad_state(watcher.latest))
        render(screen, watcher, labels, state, latch.frozen_axes)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    watcher = JoyWatcher()

    if not sys.stdout.isatty():
        watcher.destroy_node()
        rclpy.shutdown()
        print("This live table needs an interactive terminal, not captured "
              "command output. For a rate check use: ros2 topic hz /joy",
              file=sys.stderr)
        raise SystemExit(1)

    try:
        curses.wrapper(run, watcher)
    except KeyboardInterrupt:
        pass
    finally:
        watcher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
