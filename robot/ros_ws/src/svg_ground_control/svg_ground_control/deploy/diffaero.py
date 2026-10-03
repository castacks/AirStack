"""DiffAero policy flown through the generic policy commander."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Float32MultiArray

from svg_ground_control.deploy.interfaces import AttitudeSetpoint, RobotState
from svg_ground_control.deploy.policy_commander_base import PolicyCommanderBase
from svg_ground_control.diffaero.diffaero_core import DiffAeroObs, DiffAeroPolicy
from svg_ground_control.diffaero.perception_builder import Intrinsics


class DiffAeroObservationBuilder:
    """Packs a RobotState into the DiffAero observation."""

    def build(self, state: RobotState) -> DiffAeroObs:
        return DiffAeroObs(
            position_enu=state.position_enu,
            velocity_enu=state.velocity_enu,
            R_enu=state.rotation_enu,
            goal_enu=state.goal_enu,
            perception_encoded=state.perception,
        )

    def reset(self) -> None:
        pass


class DiffAeroPolicyAdapter:
    """Adapts DiffAeroPolicy to the Policy protocol, returning an attitude setpoint."""

    def __init__(self, policy: DiffAeroPolicy):
        self._policy = policy

    def compute(self, obs: DiffAeroObs) -> AttitudeSetpoint:
        cmd = self._policy.compute(obs)
        return AttitudeSetpoint(cmd.attitude_enu_flu_xyzw, cmd.thrust_norm)

    def reset(self) -> None:
        self._policy.reset()


class DiffAeroCommander(PolicyCommanderBase):

    def __init__(self):
        super().__init__('diffaero_commander')

    def declare_extra_parameters(self) -> None:
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('max_accel', 30.0)
        self.declare_parameter('max_vel', 5.0)
        self.declare_parameter('max_acc_xy', 20.0)
        self.declare_parameter('max_acc_z', 40.0)
        self.declare_parameter('tof_topic_template', '/{name}/perception/tof')
        self.declare_parameter('tof_timeout_s', 0.5)
        self._tof = None
        self._tof_time = None

    def make_observation_builder(self) -> DiffAeroObservationBuilder:
        return DiffAeroObservationBuilder()

    def make_policy(self) -> DiffAeroPolicyAdapter:
        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        if not checkpoint_path:
            raise ValueError('checkpoint_path is required for the DiffAero policy')
        # The ToF topic sends a pre-encoded 9x16 grid, so PerceptionBuilder is
        # bypassed and these intrinsics are placeholders.
        intrinsics = Intrinsics(fx=1.0, fy=1.0, cx=0.5, cy=0.5, H=1, W=1)
        policy = DiffAeroPolicy(
            intrinsics=intrinsics,
            checkpoint_path=checkpoint_path,
            max_accel=float(self.get_parameter('max_accel').value),
            max_vel=float(self.get_parameter('max_vel').value),
            max_acc_xy=float(self.get_parameter('max_acc_xy').value),
            max_acc_z=float(self.get_parameter('max_acc_z').value),
        )
        self._warm_up(policy)
        self.get_logger().info(f'DiffAero policy loaded from {checkpoint_path}')
        return DiffAeroPolicyAdapter(policy)

    def on_setup_complete(self) -> None:
        self.tof_timeout = float(self.get_parameter('tof_timeout_s').value)
        tof_tmpl = str(self.get_parameter('tof_topic_template').value)
        self.create_subscription(
            Float32MultiArray, tof_tmpl.format(name=self.name), self._tof_callback, 10)

    def _warm_up(self, policy: DiffAeroPolicy) -> None:
        # The first compute() pays a one-time JIT/CUDA cost that would otherwise
        # stall the odometry callback at policy handoff and trip the stale-odom hold.
        warm = DiffAeroObs(
            position_enu=np.zeros(3), velocity_enu=np.zeros(3),
            R_enu=np.eye(3), goal_enu=np.zeros(3))
        try:
            for _ in range(3):
                policy.compute(warm)
            policy.reset()
            self.get_logger().info('DiffAero policy warmed up')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'policy warm-up failed: {e}')

    def _tof_callback(self, msg: Float32MultiArray) -> None:
        dims = [d.size for d in msg.layout.dim]
        h, w = dims if len(dims) == 2 else (9, 16)
        self._tof = np.array(msg.data, dtype=np.float32).reshape(h, w)
        self._tof_time = self.get_clock().now()

    def get_perception(self):
        if self._tof_time is None:
            return None
        age = self.get_clock().now() - self._tof_time
        if age < Duration(seconds=self.tof_timeout):
            return self._tof
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DiffAeroCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
