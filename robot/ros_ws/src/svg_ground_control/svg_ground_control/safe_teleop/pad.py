"""Read a Linux joystick.

    pad = open_pad()
    state = pad.poll()      # never blocks
    state.axis(1)           # -1.0 .. +1.0
    state.button(0)         # True / False
    state.connected         # False once the pad is unplugged

Axis numbers vary by pad and by driver. Use view.py to find yours.
"""

from __future__ import annotations

import fcntl
import glob
import os
import struct
from dataclasses import dataclass

# struct js_event: __u32 time, __s16 value, __u8 type, __u8 number
EVENT = struct.Struct("IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

AXIS_FULL_SCALE = 32767.0

# So poll() always returns even if events arrive faster than we read them.
MAX_EVENTS_PER_POLL = 256

# Mapping reported by Linux for a Microsoft X-Box 360 pad. Other controllers
# still work; their rows are labelled by their Linux channel.
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

# The analog triggers. They rest at -1.0 and run to +1.0 when squeezed.
XBOX_TRIGGER_AXES = (2, 5)

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

XBOX_AXIS_CODES = ["X", "Y", "Z", "Rx", "Ry", "Rz", "Hat0X", "Hat0Y"]
XBOX_BUTTON_CODES = ["BtnA", "BtnB", "BtnX", "BtnY", "BtnTL", "BtnTR",
                     "BtnSelect", "BtnStart", "BtnMode", "BtnThumbL",
                     "BtnThumbR"]


def normalize(raw: int) -> float:
    """Raw Linux axis value to -1.0 .. +1.0."""
    return max(-1.0, min(1.0, raw / AXIS_FULL_SCALE))


def trigger_fraction(value: float) -> float:
    """A trigger's -1..+1 reading as 0.0 released .. 1.0 fully squeezed."""
    return max(0.0, min(1.0, (value + 1.0) / 2.0))


@dataclass(frozen=True)
class PadState:
    """One snapshot of the controller.

    When connected is False the values are the last ones seen, frozen.
    """

    axes: tuple[float, ...]
    raw_axes: tuple[int, ...]
    buttons: tuple[bool, ...]
    connected: bool

    def axis(self, index: int, default: float = 0.0) -> float:
        """Axis value, or default if this pad has no such axis."""
        if 0 <= index < len(self.axes):
            return self.axes[index]
        return default

    def button(self, index: int, default: bool = False) -> bool:
        """Button state, or default if this pad has no such button."""
        if 0 <= index < len(self.buttons):
            return self.buttons[index]
        return default


class Pad:
    """A joystick device, opened non-blocking."""

    def __init__(self, device: str | None = None):
        self.device = find_device(device)
        self._fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        try:
            self.axis_names, self.button_names, self.xbox_layout = driver_map(self._fd)
        except OSError:
            os.close(self._fd)
            raise
        self._raw = [0] * len(self.axis_names)
        self._buttons = [False] * len(self.button_names)
        self._connected = True

    @property
    def connected(self) -> bool:
        return self._connected

    def axis_label(self, index: int) -> str:
        """Readable name for an axis, falling back to the Linux channel name."""
        if self.xbox_layout and index < len(XBOX_AXIS_NAMES):
            return XBOX_AXIS_NAMES[index]
        if index < len(self.axis_names):
            return self.axis_names[index]
        return f"Axis {index}"

    def button_label(self, index: int) -> str:
        """Readable name for a button, falling back to the Linux channel name."""
        if self.xbox_layout and index < len(XBOX_BUTTON_NAMES):
            return XBOX_BUTTON_NAMES[index]
        if index < len(self.button_names):
            return self.button_names[index]
        return f"Button {index}"

    def is_trigger(self, index: int) -> bool:
        """Whether this axis is an analog trigger, which rests at -1.0."""
        return self.xbox_layout and index in XBOX_TRIGGER_AXES

    def poll(self) -> PadState:
        """Consume pending events and return the current state."""
        if self._connected:
            self._pump()
        return PadState(
            axes=tuple(normalize(v) for v in self._raw),
            raw_axes=tuple(self._raw),
            buttons=tuple(self._buttons),
            connected=self._connected,
        )

    def _pump(self) -> None:
        for _ in range(MAX_EVENTS_PER_POLL):
            try:
                data = os.read(self._fd, EVENT.size)
            except BlockingIOError:
                return
            except OSError:
                self._connected = False     # unplugged
                return
            if len(data) != EVENT.size:
                return
            _, value, event_type, number = EVENT.unpack(data)
            # The driver replays every channel on open with the INIT bit set.
            # That is how the triggers' resting values arrive, so keep them.
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS:
                _grow(self._raw, number, 0)
                self._raw[number] = value
            elif event_type == JS_EVENT_BUTTON:
                _grow(self._buttons, number, False)
                self._buttons[number] = bool(value)

    def close(self) -> None:
        if getattr(self, "_fd", None) is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self) -> "Pad":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _grow(values: list, index: int, fill) -> None:
    if index >= len(values):
        values.extend([fill] * (index + 1 - len(values)))


def find_device(requested: str | None = None) -> str:
    """Path of the joystick to open: the one asked for, or the first found."""
    if requested:
        return requested
    devices = list_devices()
    if not devices:
        raise FileNotFoundError("No joystick found under /dev/input/js*")
    return devices[0]


def list_devices() -> list[str]:
    return sorted(glob.glob("/dev/input/js*"))


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
               for code in struct.unpack(f"{button_count[0]}H",
                                         button_codes[:button_count[0] * 2])]

    xbox_layout = axes == XBOX_AXIS_CODES and buttons == XBOX_BUTTON_CODES
    return axes, buttons, xbox_layout


def open_pad(device: str | None = None) -> Pad:
    """Open a joystick, with a usable message when it exists but is unreadable."""
    try:
        return Pad(device)
    except PermissionError as error:
        raise PermissionError(
            f"{error}. Add yourself to the 'input' group (then log out and "
            "back in), or run once with sudo.") from error
