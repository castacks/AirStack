"""Improve the flying drone_soccer policy online with policy-memory updates and hot swaps.

Rounds repeat while the drone flies:

1. Wait until the policy commander announces its active checkpoint on
   ``<commander_prefix>/active_policy`` (it needs ``enable_policy_hot_swap: true``).
2. Buffer the live flight (mocap poses, goals, policy ticks, setpoints) from ``settle_s``
   after that announcement, so the buffer holds only the active policy's flight.
3. Every ``check_period_s``, cut the buffer into 3 s chunks with the offline filters
   (`drone_soccer.policy_memory.online`). Once ``min_kept_chunks`` pass (60 = 30 s),
   run one ``finetune`` update of the active checkpoint in a background thread; samples
   arriving meanwhile are dropped (they belong to the old policy).
4. Save the update under ``<output_root>/<session>/update_<n>/`` and request it on
   ``<commander_prefix>/load_policy``; when the commander announces it, start the next
   round with an empty buffer.

If the commander announces a checkpoint the learner did not request (restart or manual
swap), the learner adopts it, drops any update in flight, and starts over.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict
from enum import Enum
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from drone_soccer.policy_memory.finetune import FinetuneSettings
from drone_soccer.policy_memory.online import (
    BufferStatus,
    LiveFlightBuffer,
    OnlineMemoryConfig,
    OnlineMemoryUpdater,
    UpdateResult,
    buffer_status,
)


class Phase(Enum):
    """What the learner is doing with incoming samples."""

    WAITING_FOR_POLICY = 'waiting for the commander to announce its policy'
    COLLECTING = 'collecting chunks of the active policy'
    UPDATING = 'running an update (samples dropped)'
    SWAPPING = 'waiting for the commander to swap in the update (samples dropped)'


PARAMETER_DEFAULTS = {
    'drone': 'drone_3',
    'ball_name': 'VolleyBall',
    'pose_topic_template': '/{name}/pose',
    'trajectory_setpoint_topic_template': '/{name}/fmu/in/trajectory_setpoint',
    'commander_prefix': '/policy_commander',
    'run_dir': '/root/drone_soccer/logs/motor_dynamics_scratch_20261108',
    'memory_dir': '/root/drone_soccer/logs/motor_dynamics_scratch_20261108/analysis/policy_memory_20261112',
    'barrier_dataset': 'data_20261112',
    'output_root': '/bags/online_memory',
    'min_kept_chunks': 60,
    'top_k': 8,
    'sim_chunks': 2048,
    'alpha': 0.3,
    'real_loss_share': 0.25,
    'epochs': 5,
    'learning_rate': 1e-4,
    'check_period_s': 2.0,
    'settle_s': 0.5,
    'swap_retry_s': 5.0,
    'torch_threads': 4,
    'max_updates': 0,  # 0 = keep updating
}


class OnlineMemoryLearner(Node):
    """Buffers the active policy's flight, updates it, and hands the update to the commander."""

    def __init__(self) -> None:
        super().__init__('online_memory_learner')
        self._declare_parameters()
        get = lambda name: self.get_parameter(name).value  # noqa: E731

        import torch
        torch.set_num_threads(int(get('torch_threads')))  # leave the CPU to the commander

        self.config = OnlineMemoryConfig(
            run_dir=Path(get('run_dir')),
            memory_dir=Path(get('memory_dir')),
            barrier_dataset=str(get('barrier_dataset')),
            min_kept_chunks=int(get('min_kept_chunks')),
            top_k=int(get('top_k')),
            sim_chunks=int(get('sim_chunks')),
            finetune=FinetuneSettings(
                alpha=float(get('alpha')), real_loss_share=float(get('real_loss_share')),
                epochs=int(get('epochs')), learning_rate=float(get('learning_rate'))),
        )
        self.updater = OnlineMemoryUpdater(self.config)
        self.session_dir = Path(get('output_root')) / time.strftime('%Y%m%d_%H%M%S')
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.settle_s = float(get('settle_s'))
        self.max_updates = int(get('max_updates'))
        self.swap_retry_s = float(get('swap_retry_s'))
        self.swap_requested_s = 0.0

        self.buffer = LiveFlightBuffer()
        self.phase = Phase.WAITING_FOR_POLICY
        self.active_policy: str | None = None
        self.requested_policy: str | None = None
        self.collect_after_s = 0.0
        self.update_count = 0
        self._update_thread: threading.Thread | None = None
        self._update_outcome: UpdateResult | Exception | None = None
        self._update_generation = 0  # bumped to orphan an update when the policy changes under it

        self._create_interfaces()
        self._write_session_record()
        self.create_timer(float(get('check_period_s')), self._check)
        self.get_logger().info(
            f'Online memory learner ready: {self.config.min_kept_chunks} chunks per update, '
            f'memory {self.config.memory_dir}, writing to {self.session_dir}')

    # ----- setup ----------------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        """Topics, policy-memory inputs, thresholds, and update settings."""
        for name, value in PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, value)

    def _create_interfaces(self) -> None:
        """Best-effort sample subscriptions (match any publisher), latched policy topics."""
        get = lambda name: str(self.get_parameter(name).value)  # noqa: E731
        samples = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        prefix = get('commander_prefix')
        drone_topic = get('pose_topic_template').format(name=get('drone'))
        ball_topic = get('pose_topic_template').format(name=get('ball_name'))
        setpoint_topic = get('trajectory_setpoint_topic_template').format(name=get('drone'))

        self.create_subscription(PoseStamped, drone_topic, self._drone_pose_cb, samples)
        self.create_subscription(PoseStamped, ball_topic, self._ball_pose_cb, samples)
        self.create_subscription(Float32MultiArray, f'{prefix}/goal', self._goal_cb, samples)
        self.create_subscription(Float32MultiArray, f'{prefix}/action', self._action_cb, samples)
        self.create_subscription(TrajectorySetpoint, setpoint_topic, self._setpoint_cb, samples)
        self.create_subscription(String, f'{prefix}/active_policy', self._active_policy_cb, latched)
        self.load_policy_pub = self.create_publisher(String, f'{prefix}/load_policy', 10)

    def _write_session_record(self) -> None:
        """Settings of this session, beside its update folders."""
        record = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        record['config'] = asdict(self.config)
        (self.session_dir / 'session.json').write_text(json.dumps(record, indent=2, default=str) + '\n')

    # ----- samples --------------------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _collecting(self) -> float | None:
        """Receive time of a sample if it belongs in the buffer, else None."""
        if self.phase is not Phase.COLLECTING:
            return None
        now = self._now_s()
        return now if now >= self.collect_after_s else None

    def _drone_pose_cb(self, msg: PoseStamped) -> None:
        if (now := self._collecting()) is not None:
            self.buffer.add_drone_pose(now, *_pose_tuples(msg))

    def _ball_pose_cb(self, msg: PoseStamped) -> None:
        if (now := self._collecting()) is not None:
            self.buffer.add_ball_pose(now, *_pose_tuples(msg))

    def _goal_cb(self, msg: Float32MultiArray) -> None:
        if (now := self._collecting()) is not None and len(msg.data) >= 2:
            self.buffer.add_goal(now, msg.data[0], msg.data[1])

    def _action_cb(self, _msg: Float32MultiArray) -> None:
        if (now := self._collecting()) is not None:
            self.buffer.add_policy_tick(now)

    def _setpoint_cb(self, msg: TrajectorySetpoint) -> None:
        if (now := self._collecting()) is not None:
            self.buffer.add_setpoint_ned(now, tuple(msg.position))

    # ----- rounds ---------------------------------------------------------------------------

    def _active_policy_cb(self, msg: String) -> None:
        """Start a fresh round whenever the commander announces a policy."""
        expected = msg.data == self.requested_policy
        if not expected and self.phase in (Phase.UPDATING, Phase.SWAPPING):
            self._update_generation += 1  # the running update starts from a policy no longer flying
            self.get_logger().warn(f'Commander switched to {msg.data} on its own; dropping the pending update')
        self.active_policy = msg.data
        self.requested_policy = None
        self._start_collecting()
        self.get_logger().info(f'Active policy {msg.data}; collecting a new buffer')

    def _start_collecting(self) -> None:
        self.buffer.clear()
        self.collect_after_s = self._now_s() + self.settle_s
        self.phase = Phase.COLLECTING

    def _check(self) -> None:
        """Timer: report progress, start an update when the buffer is full, hand finished updates over."""
        if self.phase is Phase.COLLECTING:
            self._check_buffer()
        elif self.phase is Phase.UPDATING and not self._update_thread.is_alive():
            self._finish_update()
        elif self.phase is Phase.SWAPPING and self._now_s() - self.swap_requested_s > self.swap_retry_s:
            self.get_logger().warn('Commander has not announced the update yet (enable_policy_hot_swap?); '
                                   're-sending the load request')
            self._request_swap()

    def _check_buffer(self) -> None:
        status = buffer_status(self.buffer, self.config)
        drops = {name: count for name, count in status.drops.items() if name not in ('candidates', 'kept') and count}
        self.get_logger().info(
            f'{status.kept_chunks}/{self.config.min_kept_chunks} chunks kept of '
            f'{status.drops["candidates"]} (dropped: {drops})', throttle_duration_sec=10.0)
        if status.kept_chunks < self.config.min_kept_chunks:
            return
        if self.max_updates and self.update_count >= self.max_updates:
            self.get_logger().info('max_updates reached; not updating further', once=True)
            return
        self._start_update(status)

    def _start_update(self, status: BufferStatus) -> None:
        self.update_count += 1
        update_dir = self.session_dir / f'update_{self.update_count:03d}'
        policy_path, generation = Path(self.active_policy), self._update_generation
        self.phase = Phase.UPDATING
        self.buffer.clear()
        self.get_logger().info(f'Update {self.update_count}: {status.kept_chunks} chunks, from {policy_path}')

        def run() -> None:
            try:
                outcome = self.updater.update(policy_path, status, update_dir, seed=self.update_count)
            except Exception as error:  # reported by _finish_update; the drone keeps its policy
                outcome = error
            if generation == self._update_generation:
                self._update_outcome = outcome

        self._update_outcome = None
        self._update_thread = threading.Thread(target=run, daemon=True)
        self._update_thread.start()

    def _finish_update(self) -> None:
        outcome = self._update_outcome
        if isinstance(outcome, Exception) or outcome is None:
            self.get_logger().error(f'Update {self.update_count} failed: {outcome!r}; collecting again')
            self._start_collecting()
            return
        real_after = outcome.diagnostics.get('real_after', {})
        self.get_logger().info(
            f'Update {self.update_count} done in {outcome.seconds:.1f} s -> {outcome.policy_path} '
            f'(real rms change {real_after.get("rms_change_from_old_action_m", float("nan")):.3f} m); '
            'requesting hot swap')
        self.requested_policy = str(outcome.policy_path)
        self.phase = Phase.SWAPPING
        self._request_swap()

    def _request_swap(self) -> None:
        msg = String()
        msg.data = self.requested_policy
        self.load_policy_pub.publish(msg)
        self.swap_requested_s = self._now_s()


def _pose_tuples(msg: PoseStamped) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """(position xyz, orientation wxyz) of a pose message."""
    position, orientation = msg.pose.position, msg.pose.orientation
    return (position.x, position.y, position.z), (orientation.w, orientation.x, orientation.y, orientation.z)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OnlineMemoryLearner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
