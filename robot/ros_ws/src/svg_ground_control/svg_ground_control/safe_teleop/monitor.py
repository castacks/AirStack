"""Live view of the sticks and what they turn into, side by side.

    ros2 run svg_ground_control teleop_monitor --ros-args -p drone:=drone_3

Shows /joy on the left and the velocity actually published on the right, so a
stick that does nothing can be traced to whichever stage drops it: the axis
never moves, the deadzone eats it, the lock is engaged, or the command is
published but the drone ignores it.

Read-only. It subscribes to /joy, /svg/{drone}/teleop_command and the drone's
odometry, and never publishes. It runs its own VelocityMapper alongside
safe_teleop's purely to show the altitude target, which safe_teleop keeps
internally and does not publish.
"""

from __future__ import annotations

import curses
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .pad import PadState
from .velocity import VelocityMapper, deadzone

FRAME_MS = 16
STALE_S = 0.5           # matches safe_teleop's joy_timeout_s / odometry_timeout_s
BAR = 16


def bar(value: float, width: int = BAR) -> str:
    """A centered bar: left half for negative, right half for positive."""
    half = width // 2
    n = int(min(1.0, abs(value)) * half)
    if value >= 0:
        return " " * half + "#" * n + " " * (half - n)
    return " " * (half - n) + "#" * n + " " * half


class Monitor(Node):
    def __init__(self):
        super().__init__("teleop_monitor")
        self.drone = self.declare_parameter("drone", "drone_1").value
        self.joy: Joy | None = None
        self.cmd: TwistStamped | None = None
        self.alt: float | None = None
        self.t_joy = self.t_cmd = self.t_odom = 0.0
        self.n_joy = self.n_cmd = 0

        self.mapper = VelocityMapper()
        self.last_update = time.monotonic()
        self.target: float | None = None
        self.locked = False

        self.create_subscription(Joy, "/joy", self._joy, 10)
        self.create_subscription(
            TwistStamped, f"/svg/{self.drone}/teleop_command", self._cmd, 10)
        self.create_subscription(
            Odometry, f"/{self.drone}/odometry_conversion/odometry", self._odom, 10)

    def _joy(self, m: Joy) -> None:
        self.joy = m
        self.t_joy = time.monotonic()
        self.n_joy += 1
        if self.alt is None:
            return
        # Mirror safe_teleop's mapping so the altitude target is visible.
        now = time.monotonic()
        dt = max(1e-3, now - self.last_update)
        self.last_update = now
        state = PadState(
            axes=tuple(float(v) for v in m.axes),
            raw_axes=tuple(int(v * 32767) for v in m.axes),
            buttons=tuple(bool(b) for b in m.buttons),
            connected=True,
        )
        c = self.mapper.update(state, dt, self.alt)
        self.target = c.target_altitude
        self.locked = c.held

    def _cmd(self, m: TwistStamped) -> None:
        self.cmd = m
        self.t_cmd = time.monotonic()
        self.n_cmd += 1

    def _odom(self, m: Odometry) -> None:
        self.alt = float(m.pose.pose.position.z)
        self.t_odom = time.monotonic()

    def age(self, t: float) -> float:
        return 1e9 if t == 0.0 else time.monotonic() - t


def render(screen, mon: Monitor) -> None:
    screen.erase()
    h, w = screen.getmaxyx()
    row = 0

    def line(text: str = "") -> None:
        nonlocal row
        if row < h:
            screen.addnstr(row, 0, text, max(0, w - 1))
        row += 1

    m = mon.mapper
    joy_ok = mon.age(mon.t_joy) < STALE_S
    odom_ok = mon.age(mon.t_odom) < STALE_S

    line(f"teleop monitor — {mon.drone}    Ctrl-C to quit")
    line(f"  /joy {'OK ' if joy_ok else 'STALE'} ({mon.n_joy} msgs)   "
         f"teleop_command ({mon.n_cmd} msgs)   "
         f"odometry {'OK' if odom_ok else 'STALE'}")
    line("-" * 74)

    if mon.joy is None:
        line()
        line("  Waiting for /joy. Is joy_node running?")
        screen.refresh()
        return

    axes = mon.joy.axes

    def axis_row(name: str, idx: int, sign: float, extra: str = "") -> None:
        raw = axes[idx] if idx < len(axes) else 0.0
        after = deadzone(raw, m.deadzone_width) * sign
        flag = "" if abs(raw) > m.deadzone_width else "  (inside deadzone)"
        line(f"  {name:<22} axis {idx}  raw {raw:+6.3f}  ->{after:+6.3f} "
             f"[{bar(after)}]{flag}{extra}")

    line("  STICKS")
    axis_row("right stick fwd/back", m.forward_axis, m.forward_sign)
    axis_row("right stick left/rt", m.left_axis, m.left_sign)
    lock_note = "   LOCKED, climb forced to 0" if mon.locked else ""
    axis_row("left stick up/down", m.climb_axis, m.climb_sign, lock_note)

    lock_btn = m.lock.button
    pressed = (mon.joy.buttons[lock_btn]
               if lock_btn < len(mon.joy.buttons) else 0)
    line(f"  lock button {lock_btn:<10} {'DOWN' if pressed else 'up':<5}"
         f"        left stick is {'LOCKED' if mon.locked else 'free'}")
    line()

    line("  ALTITUDE")
    if mon.alt is None:
        line("    no odometry — safe_teleop publishes zero until it arrives")
    else:
        tgt = mon.target
        line(f"    measured {mon.alt:+6.2f} m     target "
             f"{'--' if tgt is None else f'{tgt:+6.2f} m'}     "
             f"error {'--' if tgt is None else f'{tgt - mon.alt:+6.2f} m'}")
        line(f"    clamps [{m.min_altitude:.2f}, {m.max_altitude:.2f}] m   "
             f"climb_rate {m.climb_rate} m/s   gain {m.altitude_gain}")
    line()

    line("  PUBLISHED VELOCITY   /svg/%s/teleop_command" % mon.drone)
    if mon.cmd is None:
        line("    nothing published yet — is safe_teleop running?")
    else:
        v = mon.cmd.twist.linear
        line(f"    vx {v.x:+6.3f}  [{bar(v.x)}]")
        line(f"    vy {v.y:+6.3f}  [{bar(v.y)}]")
        line(f"    vz {v.z:+6.3f}  [{bar(v.z)}]   <- left stick ends up here")
        if not joy_ok or not odom_ok:
            line("    (zeroed: a timeout is active — that is a stop, not a hold)")
    line()
    line("  Sticks do nothing until /swarm_commander/start has been called.")
    screen.refresh()


def run(screen, mon: Monitor) -> None:
    curses.curs_set(0)
    screen.timeout(FRAME_MS)
    while rclpy.ok():
        screen.getch()
        rclpy.spin_once(mon, timeout_sec=0.0)
        render(screen, mon)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    mon = Monitor()
    if not sys.stdout.isatty():
        mon.destroy_node()
        rclpy.shutdown()
        print("This live view needs an interactive terminal.", file=sys.stderr)
        raise SystemExit(1)
    try:
        curses.wrapper(run, mon)
    except KeyboardInterrupt:
        pass
    finally:
        mon.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
