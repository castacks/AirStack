"""Live table of what a gamepad is doing, for finding your axis numbers.

    python3 -m svg_ground_control.safe_teleop.view
    python3 -m svg_ground_control.safe_teleop.view /dev/input/js1
"""

from __future__ import annotations

import argparse
import curses
import sys

from .latch import FREEZE_BUTTON, FreezeLatch
from .pad import PadState, list_devices, open_pad, trigger_fraction

FRAME_MS = 16           # ~60 redraws a second
CENTERED_RAW = 500      # raw units either side of rest that count as centered
BAR_WIDTH = 20

# Which way each Xbox axis reads when it goes positive.
AXIS_POSITIVE = {0: "right", 1: "down", 3: "right", 4: "down",
                 6: "right", 7: "down"}
AXIS_NEGATIVE = {0: "left", 1: "up", 3: "left", 4: "up",
                 6: "left", 7: "up"}


def axis_state(pad, number: int, value: float, raw: int) -> str:
    """Where one axis is sitting, in words."""
    if pad.is_trigger(number):
        return f"{trigger_fraction(value):.0%} squeezed"
    if abs(raw) < CENTERED_RAW:
        return "centered"
    if number not in AXIS_POSITIVE or not pad.xbox_layout:
        return "positive" if value > 0 else "negative"
    return AXIS_POSITIVE[number] if value > 0 else AXIS_NEGATIVE[number]


def bar_length(pad, number: int, value: float) -> int:
    """How far to fill the bar. A trigger at -1.0 is released, so it draws empty."""
    magnitude = trigger_fraction(value) if pad.is_trigger(number) else abs(value)
    return int(magnitude * BAR_WIDTH)


def render(screen, pad, state: PadState, frozen_axes=()) -> None:
    screen.erase()
    height, width = screen.getmaxyx()
    frozen_axes = set(frozen_axes)
    row = 0

    def line(text: str = "") -> None:
        nonlocal row
        if row < height:
            screen.addnstr(row, 0, text, max(0, width - 1))
        row += 1

    if frozen_axes:
        names = ", ".join(pad.axis_label(i) for i in sorted(frozen_axes))
        line(f"HELD: {names} — press {pad.button_label(FREEZE_BUTTON)} to "
             "release, Ctrl-C to quit")
    elif state.connected:
        line(f"Live joystick state — press {pad.button_label(FREEZE_BUTTON)} "
             "to hold the left stick, Ctrl-C to quit")
    else:
        line("PAD DISCONNECTED — values below are the last ones seen, frozen")
    line(f"{'Control':<26} {'Channel':<10} {'Raw':>7} {'-1 to +1':>10}  State")
    line("-" * 72)

    for number, value in enumerate(state.axes):
        raw = state.raw_axes[number]
        mark = "HELD " if number in frozen_axes else ""
        line(f"{pad.axis_label(number):<26} {'axis ' + str(number):<10} "
             f"{raw:>7} {value:>+10.3f}  "
             f"{mark + axis_state(pad, number, value, raw):<20} "
             f"{'#' * bar_length(pad, number, value)}")

    line("-" * 72)
    for number, pressed in enumerate(state.buttons):
        line(f"{pad.button_label(number):<26} {'button ' + str(number):<10} "
             f"{'':>7} {'':>10}  {'PRESSED' if pressed else 'released'}")
    screen.refresh()


def run(screen, pad) -> None:
    curses.curs_set(0)
    screen.timeout(FRAME_MS)      # getch doubles as the frame clock
    latch = FreezeLatch()
    while True:
        screen.getch()
        render(screen, pad, latch.update(pad.poll()), latch.frozen_axes)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Show a live table of a gamepad's axes and buttons")
    parser.add_argument("device", nargs="?",
                        help="device path, e.g. /dev/input/js0")
    args = parser.parse_args(argv)

    found = list_devices()
    if len(found) > 1 and not args.device:
        print(f"Found {', '.join(found)}; using {found[0]}. "
              "Pass a device path to choose another one.", file=sys.stderr)

    try:
        pad = open_pad(args.device)
    except (FileNotFoundError, PermissionError, OSError) as error:
        print(f"Cannot open joystick: {error}", file=sys.stderr)
        print("Check with: ls /dev/input/js*", file=sys.stderr)
        raise SystemExit(1)

    if not sys.stdout.isatty():
        pad.close()
        print("This live table needs an interactive terminal, not captured "
              "command output.", file=sys.stderr)
        raise SystemExit(1)

    try:
        curses.wrapper(run, pad)
    except KeyboardInterrupt:
        pass
    finally:
        pad.close()
    print("Done.")


if __name__ == "__main__":
    main()
