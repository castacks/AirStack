"""Teleop controller registry: which physical input device flies a drone.

The ``teleop_controller`` parameter (in a config's ``safe_teleop`` block, or
``teleop_controller:=`` on the launch line) names an entry of ``CONTROLLERS``.
Each entry bundles everything that differs between input devices:

* which ROS nodes turn the device into ``sensor_msgs/Joy`` (the *driver*),
* the axis / button numbers and signs on that ``/joy`` stream (the *mapping*).

The teleop node itself (``teleop_node.py``) is device-agnostic: it only ever
sees ``/joy``. So adding a new controller is one new ``ControllerProfile``
here — no launch or node changes — as long as it can be expressed as a Joy
stream plus an axis map. A device that needs a different driver package
(a keyboard, a RC transmitter over serial, a space mouse) gets its own
``DriverNode`` list; the mapping fields then describe its Joy layout.

Currently supported:

    dragonrise_usb  generic DragonRise/SHANWAN "Android gamepad" (hid-generic
                    + ros2 joy_node) — the pad on the bench
    xbox_usb        Microsoft Xbox 360 wired controller (Linux ``xpad`` +
                    ros2 joy_node)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

# ROS 2 ``joy`` node conventions for a Linux xpad Xbox 360 pad:
#   axes    0 LS left/right  1 LS up/down  2 LT  3 RS left/right  4 RS up/down
#           5 RT  6 D-pad left/right  7 D-pad up/down
#   buttons 0 A  1 B  2 X  3 Y  4 LB  5 RB  6 Back  7 Start  8 Guide
#           9 LS click  10 RS click
# joy_node negates every axis relative to the raw device (stick left / up
# read positive), which the signs below already account for: stick up ->
# +forward (+x), stick left -> +left (+y, ENU), stick left on the yaw stick
# -> +yaw rate (counter-clockwise). All +1.0; flip one only if a pad or
# driver build really runs backwards (verified on the bench 2026-09-19 that
# left_sign = -1.0 moved the drone the wrong way).


@dataclass(frozen=True)
class DriverNode:
    """One ROS node to launch so the device shows up as ``sensor_msgs/Joy``."""

    package: str
    executable: str
    name: str
    parameters: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ControllerProfile:
    """Everything the stack needs to know about one kind of input device."""

    name: str
    description: str
    # Nodes launched by ground_control.launch.py before safe_teleop.
    drivers: Tuple[DriverNode, ...]
    # Topic those drivers publish sensor_msgs/Joy on.
    joy_topic: str
    # Axis map on that Joy stream (defaults for the safe_teleop parameters of
    # the same names; a config may still override any of them).
    forward_axis: int
    left_axis: int
    climb_axis: int
    yaw_axis: int
    lock_button: int
    forward_sign: float = 1.0
    left_sign: float = 1.0
    climb_sign: float = 1.0
    yaw_sign: float = 1.0

    def mapping_parameters(self) -> Dict[str, Any]:
        """The axis-map fields as a ROS parameter dict."""
        return {
            'forward_axis': self.forward_axis,
            'left_axis': self.left_axis,
            'climb_axis': self.climb_axis,
            'yaw_axis': self.yaw_axis,
            'lock_button': self.lock_button,
            'forward_sign': self.forward_sign,
            'left_sign': self.left_sign,
            'climb_sign': self.climb_sign,
            'yaw_sign': self.yaw_sign,
        }


XBOX_USB = ControllerProfile(
    name='xbox_usb',
    description='Xbox 360 wired USB controller (Linux xpad + ros2 joy_node)',
    drivers=(
        DriverNode(
            package='joy', executable='joy_node', name='joy_node',
            parameters={
                # First pad found. Override with device_id in the config's
                # joy_node block if several are plugged in.
                'device_id': 0,
                # joy_node's own deadzone is left small; safe_teleop applies
                # the real one (its `deadzone` parameter) with rescaling.
                'deadzone': 0.05,
                # Republish at this rate even while nothing moves, so the
                # teleop node's joy_timeout_s does not trip on a still stick.
                'autorepeat_rate': 20.0,
            }),
    ),
    joy_topic='/joy',
    # Right stick = horizontal velocity, left stick = altitude rate + yaw,
    # left bumper = lock the left stick (see teleop.md "Controls").
    forward_axis=4, left_axis=3, climb_axis=1, yaw_axis=0, lock_button=4,
    forward_sign=1.0, left_sign=1.0, climb_sign=1.0, yaw_sign=1.0,
)

# The very common generic "Android gamepad" (DragonRise / SHANWAN, USB
# 0079:181c and relatives), driven by plain hid-generic + joydev rather than
# xpad. Its DirectInput layout is NOT the Xbox one:
#   axes    0 LS left/right  1 LS up/down  2 RS left/right  3 RS up/down
#           4, 5 analog triggers (rest at FULL SCALE)  6, 7 D-pad
#   buttons 0 A  1 B  2 C  3 X  4 Y  5 Z  6 left bumper  7 right bumper
#           8, 9 triggers  10 Select  11 Start  12 Mode  13/14 stick clicks
# So the right stick is on 2/3 where an Xbox pad has 3/4, and the left bumper
# is button 6, not 4. Flying this pad on the xbox_usb map would read an idle
# trigger as full forward stick — check the layout, do not assume.
# Signs match xbox_usb: both follow the Linux ABS convention (stick up is
# negative, right is positive) and joy_node negates every axis alike.
DRAGONRISE_USB = ControllerProfile(
    name='dragonrise_usb',
    description='Generic DragonRise/SHANWAN USB gamepad, DirectInput layout '
                '(hid-generic + ros2 joy_node)',
    drivers=(
        DriverNode(
            package='joy', executable='joy_node', name='joy_node',
            parameters={
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }),
    ),
    joy_topic='/joy',
    forward_axis=3, left_axis=2, climb_axis=1, yaw_axis=0, lock_button=6,
    forward_sign=1.0, left_sign=1.0, climb_sign=1.0, yaw_sign=1.0,
)

CONTROLLERS: Dict[str, ControllerProfile] = {
    XBOX_USB.name: XBOX_USB,
    DRAGONRISE_USB.name: DRAGONRISE_USB,
}

DEFAULT_CONTROLLER = XBOX_USB.name


def controller_names() -> List[str]:
    return sorted(CONTROLLERS)


def get_controller(name: str) -> ControllerProfile:
    """Look up a profile by its ``teleop_controller`` value.

    Raises ``KeyError`` with the list of supported names, so a typo in a
    config fails loudly at launch instead of flying with a wrong axis map.
    """
    key = (name or '').strip()
    if key not in CONTROLLERS:
        raise KeyError(
            f"unknown teleop_controller '{name}'; supported: "
            f"{', '.join(controller_names())}")
    return CONTROLLERS[key]
