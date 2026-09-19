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

    xbox_usb   Microsoft Xbox 360 wired controller through the Linux ``xpad``
               driver and the standard ROS 2 ``joy`` node.
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
# read positive), which the signs below already account for.


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
    forward_sign=1.0, left_sign=-1.0, climb_sign=1.0, yaw_sign=1.0,
)

CONTROLLERS: Dict[str, ControllerProfile] = {
    XBOX_USB.name: XBOX_USB,
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
