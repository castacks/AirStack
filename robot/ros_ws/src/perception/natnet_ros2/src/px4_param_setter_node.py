#!/usr/bin/env python3

"""
PX4 Parameter Checker Node

Compares the FCU's PX4 parameters against a configured set (see
config/px4_params.yaml) for mocap-only flight (OptiTrack external vision, no GNSS,
no magnetometer). **By default it only checks and flags** — it does not write to the
FCU. The desired values are meant to be set once by a human in QGroundControl (see
docs/robot/…/px4_external_vision.md); this node is a safety net that catches a
mis-configured FCU before flight.

Two safety flags control behaviour:

- ``auto_set`` (default ``false``): when ``true``, the node also *writes* any
  mismatched param via ``param/set`` (ParamSetV2) and verifies the readback — the
  legacy enforce behaviour. When ``false`` (default) the node never writes.
- ``on_mismatch`` (``warn`` | ``halt``, default ``warn``): with ``auto_set=false``,
  what to do when a param disagrees. ``warn`` logs the diffs and lets the stack come
  up; ``halt`` logs fatal and exits non-zero so a ``required`` launch node tears the
  stack down.

For each entry under the ``params.`` prefix the node waits for an FCU connection +
settle, reads the current value via ``get_parameters``, and compares. Type mapping
follows the YAML literal: integers → MAVLink int params, floats → float params —
write ``6.0`` (not ``6``) for float params like EKF2_EV_DELAY so the type matches.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSetV2


class Px4ParamSetterNode(Node):
    """PX4 parameter checker (optionally setter) via the MAVROS param plugin."""

    def __init__(self):
        super().__init__(
            'px4_param_setter',
            automatically_declare_parameters_from_overrides=True,
        )

        self._enabled = self._param_or('enabled', True)
        if not self._enabled:
            self.get_logger().info('PX4 param checker disabled (enabled=false).')
            return

        # Safety flags: check-only by default; opt in to writing with auto_set.
        self._auto_set = bool(self._param_or('auto_set', False))
        self._on_mismatch = str(self._param_or('on_mismatch', 'warn')).lower()
        if self._on_mismatch not in ('warn', 'halt'):
            self.get_logger().warn(
                f"Invalid on_mismatch {self._on_mismatch!r}; falling back to 'warn'."
            )
            self._on_mismatch = 'warn'

        # Seconds after MAVROS connects before the first attempt (initial
        # param-table pull over serial takes a while at 115200 baud).
        self._settle_sec = float(self._param_or('settle_sec', 10.0))
        self._retry_period_sec = float(self._param_or('retry_period_sec', 2.0))
        self._max_attempts = int(self._param_or('max_attempts', 30))

        # Desired FCU params from the params.* prefix; YAML int → PX4 int32,
        # YAML float → PX4 float.
        self._desired = {
            name: p.value
            for name, p in self.get_parameters_by_prefix('params').items()
        }
        self._pending = dict(self._desired)
        self._changed: list[str] = []
        self._skipped: list[str] = []
        # (param_id, current, desired) for params that disagree and were NOT set
        # (auto_set=false). Drives the on_mismatch policy in _finish().
        self._mismatched: list[tuple] = []
        self._attempts = 0
        self._connected_since = None
        self._done = False
        self._inflight = False

        if not self._pending:
            self.get_logger().warn('No params.* entries configured; nothing to do.')
            self._done = True
            return

        self._get_cli = self.create_client(GetParameters, 'param_get_parameters')
        self._set_cli = self.create_client(ParamSetV2, 'param_set')
        self._state_sub = self.create_subscription(
            State, 'mavros_state', self._on_mavros_state, 10
        )
        self._timer = self.create_timer(self._retry_period_sec, self._tick)

        mode = 'auto-set' if self._auto_set else f'check-only (on_mismatch={self._on_mismatch})'
        self.get_logger().info(
            f'PX4 param checker started [{mode}]: {len(self._pending)} params '
            f'({", ".join(sorted(self._pending))}), settle_sec={self._settle_sec}'
        )

    def _param_or(self, name, default):
        """Return a declared-from-overrides parameter value, or the default."""
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            if value is not None:
                return value
        return default

    # --- MAVROS state ------------------------------------------------------

    def _on_mavros_state(self, msg: State):
        if msg.connected and self._connected_since is None:
            self._connected_since = self.get_clock().now()
            self.get_logger().info('FCU connected; waiting for param table to settle.')

    # --- Main retry loop ---------------------------------------------------

    def _tick(self):
        if self._done or self._inflight:
            return
        if self._connected_since is None:
            return
        elapsed = (self.get_clock().now() - self._connected_since).nanoseconds * 1e-9
        if elapsed < self._settle_sec:
            return
        if not self._pending:
            self._finish()
            return
        if self._attempts >= self._max_attempts:
            self.get_logger().error(
                f'Giving up after {self._attempts} attempts; '
                f'unset params: {", ".join(sorted(self._pending))}'
            )
            self._finish()
            return

        self._attempts += 1
        param_id = sorted(self._pending)[0]
        if not self._get_cli.service_is_ready() or not self._set_cli.service_is_ready():
            self.get_logger().info('MAVROS param services not ready yet; retrying.')
            return

        self._inflight = True
        req = GetParameters.Request(names=[param_id])
        future = self._get_cli.call_async(req)
        future.add_done_callback(
            lambda f, pid=param_id: self._on_get_done(pid, f)
        )

    # --- Get → compare → set → verify chain --------------------------------

    def _on_get_done(self, param_id: str, future):
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001 — retry on any transport error
            self.get_logger().warn(f'{param_id}: get_parameters failed ({e}); will retry.')
            self._inflight = False
            return

        current = resp.values[0] if resp.values else None
        if current is not None and self._matches(current, self._desired[param_id]):
            self.get_logger().info(f'{param_id}: already {self._desired[param_id]} — skipping.')
            self._skipped.append(param_id)
            del self._pending[param_id]
            self._inflight = False
            return
        if current is None or current.type == ParameterType.PARAMETER_NOT_SET:
            # Param table likely not pulled yet — retry rather than flag/force-set.
            self.get_logger().info(f'{param_id}: not in MAVROS param table yet; will retry.')
            self._inflight = False
            return

        # Mismatch. Check-only mode (default): record and flag, never write.
        if not self._auto_set:
            self._mismatched.append(
                (param_id, self._value_of(current), self._desired[param_id])
            )
            del self._pending[param_id]
            self._inflight = False
            return

        req = ParamSetV2.Request()
        req.force_set = False
        req.param_id = param_id
        req.value = self._to_parameter_value(self._desired[param_id])
        set_future = self._set_cli.call_async(req)
        set_future.add_done_callback(
            lambda f, pid=param_id, old=self._value_of(current): self._on_set_done(pid, old, f)
        )

    def _on_set_done(self, param_id: str, old_value, future):
        self._inflight = False
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001 — retry on any transport error
            self.get_logger().warn(f'{param_id}: set failed ({e}); will retry.')
            return

        desired = self._desired[param_id]
        if not resp.success or not self._matches(resp.value, desired):
            self.get_logger().warn(
                f'{param_id}: set rejected or readback mismatch '
                f'(wanted {desired}, got {self._value_of(resp.value)}); will retry.'
            )
            return

        self.get_logger().info(f'{param_id}: {old_value} -> {desired}')
        self._changed.append(param_id)
        del self._pending[param_id]

    def _finish(self):
        self._done = True
        self._timer.cancel()
        self.get_logger().info(
            f'PX4 param check finished: {len(self._skipped)} already correct, '
            f'{len(self._changed)} set, {len(self._mismatched)} mismatched, '
            f'{len(self._pending)} unread.'
        )
        if self._changed:
            self.get_logger().warn(
                f'FCU parameters changed ({", ".join(sorted(self._changed))}). '
                'Reboot the flight controller before flying so EKF2 starts clean.'
            )

        # Check-only mismatches: report each, then apply the on_mismatch policy.
        if self._mismatched:
            for pid, current, desired in sorted(self._mismatched):
                self.get_logger().warn(
                    f'{pid}: FCU has {current}, expected {desired} '
                    '(not set — auto_set=false). Fix in QGroundControl.'
                )
            names = ", ".join(sorted(p for p, _, _ in self._mismatched))
            if self._on_mismatch == 'halt':
                self.get_logger().fatal(
                    f'{len(self._mismatched)} PX4 param(s) wrong for external-vision '
                    f'flight ({names}); halting (on_mismatch=halt). Set them in '
                    'QGroundControl or enable auto_set.'
                )
                # SystemExit propagates out of spin(); main()'s finally shuts down
                # rclpy. Non-zero code lets a `required` launch node tear the stack down.
                sys.exit(1)
            self.get_logger().warn(
                f'{len(self._mismatched)} PX4 param(s) differ from the external-vision '
                f'set ({names}); continuing (on_mismatch=warn).'
            )

    # --- Value helpers ------------------------------------------------------

    @staticmethod
    def _to_parameter_value(value) -> ParameterValue:
        pv = ParameterValue()
        if isinstance(value, bool) or isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        else:
            raise TypeError(f'Unsupported PX4 param value type: {type(value)}')
        return pv

    @staticmethod
    def _value_of(pv: ParameterValue):
        if pv.type == ParameterType.PARAMETER_INTEGER:
            return pv.integer_value
        if pv.type == ParameterType.PARAMETER_DOUBLE:
            return pv.double_value
        return None

    @classmethod
    def _matches(cls, pv: ParameterValue, desired) -> bool:
        current = cls._value_of(pv)
        if current is None:
            return False
        # FCU floats are float32 — compare with a tolerance that absorbs the
        # float64 → float32 round trip.
        return math.isclose(float(current), float(desired), rel_tol=1e-5, abs_tol=1e-6)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Px4ParamSetterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
