"""Turn pad readings into a velocity command. No ROS, no drone connection.

Two different mappings, on purpose:

    right stick  ->  horizontal velocity, directly. Release means stop.
    left stick   ->  the RATE OF CHANGE of a target altitude. Hold it and the
                     target ramps; release and the target stays where it got
                     to, and the drone is flown back to it if it sags.

So altitude holds itself on release, which is what altitude-hold feels like on
a real drone. The vertical velocity actually sent is computed from the gap
between the target and where the drone is, not from the stick.

Left bumper holds the left stick, so a knock cannot move the target at all.

    python3 -m svg_ground_control.safe_teleop.velocity
"""

from __future__ import annotations

import argparse
import curses
import sys
from dataclasses import dataclass

from . import view
from .latch import FREEZE_BUTTON, ButtonToggle
from .pad import list_devices, open_pad

# Axis indices and signs for an Xbox pad as reported on /joy.
FORWARD_AXIS = 4        # right stick up/down
LEFT_AXIS = 3           # right stick left/right
CLIMB_AXIS = 1          # left stick up/down
YAW_AXIS = 0            # left stick left/right
FORWARD_SIGN = 1.0
LEFT_SIGN = -1.0
CLIMB_SIGN = 1.0
YAW_SIGN = 1.0
YAW_RATE_RAD_S = 1.0

# The lock covers the whole left stick: altitude and yaw.
LOCKED_AXES = (0, 1)

DEADZONE = 0.15
MAX_SPEED_MPS = 1.0         # horizontal speed at full stick
CLIMB_RATE_MPS = 0.5        # how fast the TARGET moves at full stick
ALTITUDE_GAIN = 1.0         # target-to-current gap -> vertical velocity
MAX_CLIMB_SPEED_MPS = 0.8   # cap on the vertical velocity actually sent
MIN_ALTITUDE_M = 0.3        # the target is clamped into this band; the CBF
MAX_ALTITUDE_M = 2.5        # knows nothing about the floor or ceiling


def deadzone(value: float, width: float = DEADZONE) -> float:
    """Ignore stick slop, rescaled so full deflection still reaches 1.0."""
    if abs(value) < width:
        return 0.0
    return (abs(value) - width) / (1.0 - width) * (1.0 if value > 0 else -1.0)


@dataclass(frozen=True)
class Command:
    """What to send to the drone this tick, plus what produced it.

    vx / vy / vz are metres per second. target_altitude is the remembered
    height vz is chasing.
    """

    vx: float
    vy: float
    vz: float
    yaw_rate: float
    target_altitude: float
    altitude: float
    held: bool

    @property
    def altitude_error(self) -> float:
        return self.target_altitude - self.altitude


class VelocityMapper:
    """Pad state in, velocity command out.

    ``update`` needs the drone's current altitude, because the vertical
    velocity is a correction toward the remembered target rather than a
    reading off the stick.
    """

    def __init__(self, *, max_speed=MAX_SPEED_MPS, climb_rate=CLIMB_RATE_MPS,
                 altitude_gain=ALTITUDE_GAIN, max_climb_speed=MAX_CLIMB_SPEED_MPS,
                 min_altitude=MIN_ALTITUDE_M, max_altitude=MAX_ALTITUDE_M,
                 deadzone_width=DEADZONE,
                 forward_axis=FORWARD_AXIS, left_axis=LEFT_AXIS,
                 climb_axis=CLIMB_AXIS, yaw_axis=YAW_AXIS,
                 yaw_rate=YAW_RATE_RAD_S, lock_button=FREEZE_BUTTON,
                 forward_sign=FORWARD_SIGN, left_sign=LEFT_SIGN,
                 climb_sign=CLIMB_SIGN, yaw_sign=YAW_SIGN):
        self.max_speed = max_speed
        self.climb_rate = climb_rate
        self.altitude_gain = altitude_gain
        self.max_climb_speed = max_climb_speed
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.deadzone_width = deadzone_width
        self.forward_axis = forward_axis
        self.left_axis = left_axis
        self.climb_axis = climb_axis
        self.yaw_axis = yaw_axis
        self.yaw_rate = yaw_rate
        self.forward_sign = forward_sign
        self.left_sign = left_sign
        self.climb_sign = climb_sign
        self.yaw_sign = yaw_sign
        self.lock = ButtonToggle(lock_button)
        self.target_altitude: float | None = None

    def update(self, state, dt: float, altitude: float) -> Command:
        locked = self.lock.update(state)

        # First tick: adopt where the drone already is, so nothing commands a
        # jump to an altitude that was never set.
        if self.target_altitude is None:
            self.target_altitude = altitude

        # A vanished pad must not leave the last stick deflection latched.
        if not state.connected:
            return self._command(0.0, 0.0, altitude, 0.0)

        forward = deadzone(state.axis(self.forward_axis),
                           self.deadzone_width) * self.forward_sign
        left = deadzone(state.axis(self.left_axis),
                        self.deadzone_width) * self.left_sign
        climb = 0.0 if locked else (
            deadzone(state.axis(self.climb_axis),
                     self.deadzone_width) * self.climb_sign)
        yaw = 0.0 if locked else (
            deadzone(state.axis(self.yaw_axis),
                     self.deadzone_width) * self.yaw_sign)

        # This is the whole difference from a direct mapping: the stick moves
        # the target, it does not set the velocity.
        self.target_altitude = min(self.max_altitude, max(
            self.min_altitude, self.target_altitude + climb * self.climb_rate * dt))

        return self._command(forward * self.max_speed, left * self.max_speed,
                             altitude, yaw * self.yaw_rate)

    def _command(self, vx: float, vy: float, altitude: float,
                 yaw_rate: float = 0.0) -> Command:
        error = self.target_altitude - altitude
        vz = max(-self.max_climb_speed,
                 min(self.max_climb_speed, error * self.altitude_gain))
        # + 0.0 turns -0.0 back into 0.0, which otherwise displays as "-0.00".
        return Command(vx=vx + 0.0, vy=vy + 0.0, vz=vz,
                       yaw_rate=yaw_rate + 0.0,
                       target_altitude=self.target_altitude,
                       altitude=altitude, held=self.lock.engaged)


# ---------------------------------------------------------------------------
# Live preview. Flies a stand-in drone that simply moves at whatever vertical
# velocity is commanded, so the ramp-and-hold behaviour is visible at a desk.
# ---------------------------------------------------------------------------

FRAME_MS = 20
BAR_WIDTH = 24


def _bar(value: float, limit: float) -> str:
    """Centre-zero bar, so sign is visible at a glance."""
    half = BAR_WIDTH // 2
    filled = int(min(1.0, abs(value) / limit) * half) if limit else 0
    if value >= 0:
        return " " * half + "#" * filled + " " * (half - filled)
    return " " * (half - filled) + "#" * filled + " " * half


PAD_COLUMN_WIDTH = 56
COLUMN_GAP = 3


def pad_lines(pad, state, locked: bool) -> list[str]:
    """The raw pad reading, one row per axis, buttons on one line."""
    rows = ["Controller Input Commands", "-" * (PAD_COLUMN_WIDTH - 2)]
    for number, value in enumerate(state.axes):
        # The lock does not freeze the reading, it stops the reading being
        # used -- so this row keeps moving and is labelled IGNORED instead.
        note = " IGNORED" if locked and number in LOCKED_AXES else ""
        rows.append(f"{'axis ' + str(number):<7}{pad.axis_label(number):<25}"
                    f"{value:>+7.3f}  "
                    f"{view.axis_state(pad, number, value, state.raw_axes[number])}"
                    f"{note}")
    pressed = [pad.button_label(i)
               for i, down in enumerate(state.buttons) if down]
    rows.append("")
    rows.append(f"pressed: {', '.join(pressed) if pressed else '(none)'}")
    if not state.connected:
        rows.append("PAD DISCONNECTED — values frozen")
    return rows


def command_lines(mapper: VelocityMapper, command: Command) -> list[str]:
    """What that reading turns into."""
    rows = [
        "Velocity Output Commands",
        "-" * 44,
        f"{'vx':<4} {command.vx:>+7.2f} m/s  {_bar(command.vx, mapper.max_speed)}",
        f"{'vy':<4} {command.vy:>+7.2f} m/s  {_bar(command.vy, mapper.max_speed)}",
        f"{'vz':<4} {command.vz:>+7.2f} m/s  "
        f"{_bar(command.vz, mapper.max_climb_speed)}",
        "-" * 44,
        f"{'target altitude':<17}{command.target_altitude:>7.2f} m",
        f"{'drone altitude':<17}{command.altitude:>7.2f} m",
        f"{'gap':<17}{command.altitude_error:>+7.2f} m",
        f"{'clamped to':<17}{mapper.min_altitude}–{mapper.max_altitude} m",
        "",
    ]
    if command.held:
        rows.append("LEFT STICK LOCKED — the target cannot move.")
        rows.append("Left bumper to release.")
    else:
        rows.append("Release the left stick and the target stops")
        rows.append("where it is: that is the hold.")
    return rows


def render(screen, pad, state, mapper: VelocityMapper, command: Command) -> None:
    screen.erase()
    height, width = screen.getmaxyx()
    row = 0

    def line(text: str = "") -> None:
        nonlocal row
        if row < height:
            screen.addnstr(row, 0, text, max(0, width - 1))
        row += 1

    line("Velocity preview — PREVIEW ONLY, nothing is being flown. Ctrl-C to quit")
    line("Right stick: move.  Left stick: raise/lower the target.  "
         "Left bumper: lock the left stick.")
    line()

    left = pad_lines(pad, state, command.held)
    right = command_lines(mapper, command)
    if width >= PAD_COLUMN_WIDTH + COLUMN_GAP + 46:
        pad_out = " " * (PAD_COLUMN_WIDTH + COLUMN_GAP)
        for i in range(max(len(left), len(right))):
            a = left[i] if i < len(left) else ""
            b = right[i] if i < len(right) else ""
            line(f"{a:<{PAD_COLUMN_WIDTH + COLUMN_GAP}}{b}" if b
                 else a if a else pad_out.rstrip())
    else:
        for text in left + [""] + right:
            line(text)
    screen.refresh()


def run(screen, pad) -> None:
    curses.curs_set(0)
    screen.timeout(FRAME_MS)
    mapper = VelocityMapper()
    dt = FRAME_MS / 1000.0
    altitude = 1.0          # stand-in drone, starts hovering at 1 m
    while True:
        screen.getch()
        state = pad.poll()
        command = mapper.update(state, dt, altitude)
        altitude += command.vz * dt     # perfect tracking, no lag or inertia
        render(screen, pad, state, mapper, command)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Preview the velocity command a gamepad would produce")
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
        print("This preview needs an interactive terminal, not captured "
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
