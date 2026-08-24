#!/usr/bin/env python3
"""Show live button and axis values from a Linux joystick device.

No ROS packages or third-party Python modules are required.

Examples:
    python3 print_joystick.py
    python3 print_joystick.py /dev/input/js1
"""

import argparse
import curses
import fcntl
import glob
import os
import struct
import sys


EVENT = struct.Struct("IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# Mapping reported by Linux for a Microsoft X-Box 360 pad. Other controllers
# still work; their rows are intentionally labelled by their Linux channel.
XBOX_AXIS_NAMES = [
    "Left stick left / right",
    "Left stick up / down",
    "Left trigger",
    "Right stick left / right",
    "Right stick up / down",
    "Right trigger",
    "D-pad left / right",
    "D-pad up / down",
]
XBOX_BUTTON_NAMES = [
    "A", "B", "X", "Y", "Left bumper", "Right bumper", "Back", "Start",
    "Guide", "Left-stick click", "Right-stick click",
]

# Linux joystick API requests. These expose the same channel maps that jstest
# displays, without needing a third-party Python package.
IOC_READ = 2 << 30
JSIOCGAXES = IOC_READ | (1 << 16) | (ord("j") << 8) | 0x11
JSIOCGBUTTONS = IOC_READ | (1 << 16) | (ord("j") << 8) | 0x12
JSIOCGAXMAP = IOC_READ | (64 << 16) | (ord("j") << 8) | 0x32
JSIOCGBTNMAP = IOC_READ | (512 << 16) | (ord("j") << 8) | 0x34

AXIS_CODE_NAMES = {
    0x00: "X", 0x01: "Y", 0x02: "Z", 0x03: "Rx", 0x04: "Ry", 0x05: "Rz",
    0x10: "Hat0X", 0x11: "Hat0Y", 0x12: "Hat1X", 0x13: "Hat1Y",
}
BUTTON_CODE_NAMES = {
    0x100: "Btn0", 0x101: "Btn1", 0x102: "Btn2", 0x103: "Btn3",
    0x130: "BtnA", 0x131: "BtnB", 0x132: "BtnC", 0x133: "BtnX",
    0x134: "BtnY", 0x135: "BtnZ", 0x136: "BtnTL", 0x137: "BtnTR",
    0x138: "BtnTL2", 0x139: "BtnTR2", 0x13A: "BtnSelect", 0x13B: "BtnStart",
    0x13C: "BtnMode", 0x13D: "BtnThumbL", 0x13E: "BtnThumbR",
}


def find_device(requested: str | None) -> str:
    if requested:
        return requested
    devices = sorted(glob.glob("/dev/input/js*"))
    if not devices:
        raise FileNotFoundError("No joystick found under /dev/input/js*")
    if len(devices) > 1:
        print(f"Found {', '.join(devices)}; using {devices[0]}. "
              "Pass a device path to choose another one.", file=sys.stderr)
    return devices[0]


def driver_map(fd: int) -> tuple[list[str], list[str], bool]:
    """Read Linux's own names and channel counts for this joystick."""
    axis_count = bytearray(1)
    button_count = bytearray(1)
    fcntl.ioctl(fd, JSIOCGAXES, axis_count, True)
    fcntl.ioctl(fd, JSIOCGBUTTONS, button_count, True)

    axis_codes = bytearray(64)
    button_codes = bytearray(512)
    fcntl.ioctl(fd, JSIOCGAXMAP, axis_codes, True)
    fcntl.ioctl(fd, JSIOCGBTNMAP, button_codes, True)
    axes = [AXIS_CODE_NAMES.get(code, f"Axis 0x{code:02X}")
            for code in axis_codes[:axis_count[0]]]
    buttons = [BUTTON_CODE_NAMES.get(code, f"Button 0x{code:03X}")
               for code in struct.unpack(f"{button_count[0]}H", button_codes[:button_count[0] * 2])]

    xbox_layout = (axes == ["X", "Y", "Z", "Rx", "Ry", "Rz", "Hat0X", "Hat0Y"]
                   and buttons == ["BtnA", "BtnB", "BtnX", "BtnY", "BtnTL", "BtnTR",
                                   "BtnSelect", "BtnStart", "BtnMode", "BtnThumbL", "BtnThumbR"])
    return axes, buttons, xbox_layout


def axis_state(number: int, value: int, xbox_layout: bool) -> str:
    """Describe the direction of a standard Xbox axis."""
    if abs(value) < 500:
        return "centered"
    if xbox_layout and number in (2, 5):
        return f"{(value + 32767) / 65534:.0%} pressed"
    positive = {
        0: "right", 1: "down", 3: "right", 4: "down", 6: "right", 7: "down",
    }
    negative = {
        0: "left", 1: "up", 3: "left", 4: "up", 6: "left", 7: "up",
    }
    if not xbox_layout or number not in positive:
        return "positive" if value > 0 else "negative"
    return positive[number] if value > 0 else negative[number]


def render(screen: "curses.window", axes: list[int], buttons: list[int],
           axis_names: list[str], button_names: list[str], xbox_layout: bool) -> None:
    """Redraw the current controller state as one table."""
    screen.erase()
    _, width = screen.getmaxyx()
    row = 0

    def line(text: str) -> None:
        nonlocal row
        screen.addnstr(row, 0, text, max(0, width - 1))
        row += 1

    line("Live joystick state — Ctrl-C to quit")
    line(f"{'Control':<26} {'Channel':<10} {'Raw':>7} {'-1 to +1':>10}  State")
    line("-" * 72)
    for number, value in enumerate(axes):
        name = (XBOX_AXIS_NAMES[number] if xbox_layout and number < len(XBOX_AXIS_NAMES)
                else axis_names[number] if number < len(axis_names) else f"Axis {number}")
        line(f"{name:<26} {'axis ' + str(number):<10} {value:>7} "
             f"{value / 32767:>+10.3f}  {axis_state(number, value, xbox_layout)}")
    line("-" * 72)
    for number, value in enumerate(buttons):
        name = (XBOX_BUTTON_NAMES[number] if xbox_layout and number < len(XBOX_BUTTON_NAMES)
                else button_names[number] if number < len(button_names) else f"Button {number}")
        line(f"{name:<26} {'button ' + str(number):<10} {'':>7} {'':>10}  "
             f"{'PRESSED' if value else 'released'}")
    screen.refresh()


def main() -> None:
    parser = argparse.ArgumentParser(description="Show a live Linux joystick table")
    parser.add_argument("device", nargs="?", help="device path, e.g. /dev/input/js0")
    args = parser.parse_args()

    try:
        device = find_device(args.device)
        fd = os.open(device, os.O_RDONLY)
    except (FileNotFoundError, PermissionError, OSError) as error:
        print(f"Cannot open joystick: {error}", file=sys.stderr)
        print("Try: ls /dev/input/js*", file=sys.stderr)
        print("If it exists but is permission denied, run once with sudo or add "
              "your user to the input group.", file=sys.stderr)
        raise SystemExit(1)

    if not sys.stdout.isatty():
        print("This live table needs an interactive terminal, not captured command output.",
              file=sys.stderr)
        raise SystemExit(1)

    axis_names, button_names, xbox_layout = driver_map(fd)

    def run(screen: "curses.window") -> None:
        # This controller identifies itself as an Xbox 360 pad. Eight axes and
        # eleven buttons match the mapping above; longer lists remain generic.
        axes = [0] * len(axis_names)
        buttons = [0] * len(button_names)
        curses.curs_set(0)
        render(screen, axes, buttons, axis_names, button_names, xbox_layout)
        while True:
            data = os.read(fd, EVENT.size)
            if len(data) != EVENT.size:
                continue
            _, value, event_type, number = EVENT.unpack(data)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS:
                if number >= len(axes):
                    axes.extend([0] * (number + 1 - len(axes)))
                axes[number] = value
            elif event_type == JS_EVENT_BUTTON:
                if number >= len(buttons):
                    buttons.extend([0] * (number + 1 - len(buttons)))
                buttons[number] = value
            else:
                continue
            render(screen, axes, buttons, axis_names, button_names, xbox_layout)

    try:
        curses.wrapper(run)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
    print("Done.")


if __name__ == "__main__":
    main()
