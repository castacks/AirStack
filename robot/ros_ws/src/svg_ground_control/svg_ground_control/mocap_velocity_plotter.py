"""Interactive tuner for Kalman-filtered mocap velocity."""

from __future__ import annotations

from collections import deque
import threading

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from svg_ground_control.mocap_velocity import create_velocity_estimator


SERIES_STYLES = {
    'kalman': {'color': 'tab:blue', 'linewidth': 2.0},
}


class VelocitySeries:
    """Bounded timestamp/velocity history for one plotted signal."""

    def __init__(self, history_s: float) -> None:
        self.history_s = history_s
        self.times: deque[float] = deque(maxlen=20000)
        self.values: deque[np.ndarray] = deque(maxlen=20000)

    def append(self, stamp_s: float, velocity: np.ndarray) -> None:
        self.times.append(float(stamp_s))
        self.values.append(np.asarray(velocity, dtype=float).reshape(3).copy())
        oldest_allowed = stamp_s - self.history_s
        while self.times and self.times[0] < oldest_allowed:
            self.times.popleft()
            self.values.popleft()

    def clear(self) -> None:
        self.times.clear()
        self.values.clear()

    def snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.times:
            return np.empty(0), np.empty((0, 3))
        return np.asarray(self.times), np.asarray(self.values)


class MocapVelocityPlotter(Node):
    """Run a tunable Kalman estimator and retain plotting history."""

    def __init__(self) -> None:
        super().__init__('mocap_velocity_plotter')
        self.declare_parameter('pose_topic', '/SoccerBall/pose')
        self.declare_parameter('history_s', 15.0)
        self.declare_parameter('refresh_rate_hz', 20.0)
        self.declare_parameter('mocap_qos_best_effort', False)
        self.declare_parameter('velocity_kalman_position_stddev_m', 0.004)
        self.declare_parameter(
            'velocity_kalman_acceleration_stddev_mps2', 0.1)
        self.declare_parameter(
            'velocity_kalman_initial_velocity_stddev_mps', 1.7)
        self.declare_parameter('velocity_filter_max_dt_s', 0.5)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.history_s = float(self.get_parameter('history_s').value)
        self.refresh_rate_hz = float(
            self.get_parameter('refresh_rate_hz').value)
        if self.history_s <= 0.0:
            raise ValueError('history_s must be > 0')
        if self.refresh_rate_hz <= 0.0:
            raise ValueError('refresh_rate_hz must be > 0')

        estimator_settings = {
            'kalman_position_stddev_m': float(
                self.get_parameter(
                    'velocity_kalman_position_stddev_m').value),
            'kalman_acceleration_stddev_mps2': float(
                self.get_parameter(
                    'velocity_kalman_acceleration_stddev_mps2').value),
            'kalman_initial_velocity_stddev_mps': float(
                self.get_parameter(
                    'velocity_kalman_initial_velocity_stddev_mps').value),
            'max_dt_s': float(
                self.get_parameter('velocity_filter_max_dt_s').value),
        }
        self.estimator_settings = estimator_settings
        self.estimators = {
            'kalman': create_velocity_estimator(
                'kalman', **estimator_settings),
        }
        self.series = {
            name: VelocitySeries(self.history_s)
            for name in SERIES_STYLES
        }
        self._lock = threading.Lock()
        self._start_stamp_s: float | None = None
        self._last_pose_stamp_s: float | None = None
        self._latest_position: np.ndarray | None = None
        self._warned_zero_stamp = False

        qos = QoSProfile(depth=100)
        if bool(self.get_parameter('mocap_qos_best_effort').value):
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            qos,
        )
        self.get_logger().info(
            f'Plotting tunable Kalman estimate from pose={self.pose_topic}, '
            f'history={self.history_s:.1f}s')

    def _message_stamp_s(self, sec: int, nanosec: int) -> float:
        stamp_s = float(sec) + float(nanosec) * 1.0e-9
        if stamp_s > 0.0:
            return stamp_s
        if not self._warned_zero_stamp:
            self.get_logger().warning(
                'Input header stamp is zero; using ROS receipt time for plots')
            self._warned_zero_stamp = True
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _reset_history(self, stamp_s: float) -> None:
        for estimator in self.estimators.values():
            estimator.reset()
        for series in self.series.values():
            series.clear()
        self._start_stamp_s = stamp_s

    def _relative_time(self, stamp_s: float) -> float:
        if self._start_stamp_s is None:
            self._start_stamp_s = stamp_s
        return stamp_s - self._start_stamp_s

    def _pose_callback(self, message: PoseStamped) -> None:
        stamp_s = self._message_stamp_s(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
        )
        position = message.pose.position
        position_xyz = np.array([position.x, position.y, position.z])

        with self._lock:
            if (
                self._last_pose_stamp_s is not None
                and stamp_s < self._last_pose_stamp_s - 1.0e-6
            ):
                self._reset_history(stamp_s)
            relative_time = self._relative_time(stamp_s)
            for name, estimator in self.estimators.items():
                velocity = estimator.update(position_xyz, stamp_s)
                display_name = name.replace('_', ' ')
                self.series[display_name].append(relative_time, velocity)
            self._last_pose_stamp_s = stamp_s
            self._latest_position = position_xyz.copy()

    def update_tuning(self, **settings: float) -> None:
        """Apply plot-only filter settings and restart affected estimates."""
        kalman_keys = {
            'kalman_position_stddev_m',
            'kalman_acceleration_stddev_mps2',
            'kalman_initial_velocity_stddev_mps',
        }
        unknown = set(settings) - kalman_keys
        if unknown:
            raise ValueError(f'Unknown tuning settings: {sorted(unknown)}')

        with self._lock:
            self.estimator_settings.update(settings)
            affected = []
            if set(settings) & kalman_keys:
                affected.append('kalman')

            for name in affected:
                estimator = create_velocity_estimator(
                    name, **self.estimator_settings)
                if (
                    self._latest_position is not None
                    and self._last_pose_stamp_s is not None
                ):
                    estimator.update(
                        self._latest_position, self._last_pose_stamp_s)
                self.estimators[name] = estimator
                self.series[name.replace('_', ' ')].clear()

    def snapshots(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        with self._lock:
            return {name: values.snapshot()
                    for name, values in self.series.items()}


def show_interactive_plot(node: MocapVelocityPlotter) -> None:
    """Display a rolling Matplotlib plot until its window is closed."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider

    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
    component_names = ('x', 'y', 'z', 'speed')
    lines = {}
    for component_index, axis in enumerate(axes):
        for series_name, style in SERIES_STYLES.items():
            line, = axis.plot([], [], label=series_name, **style)
            lines[(component_index, series_name)] = line
        axis.axhline(0.0, color='black', linewidth=0.6, alpha=0.35)
        axis.set_ylabel(f'{component_names[component_index]} (m/s)')
        axis.set_ylim(-3.0, 3.0)
        axis.grid(True, alpha=0.25)
    axes[-1].set_xlabel('time (s)')
    axes[0].legend(loc='upper right', ncols=2)
    figure.suptitle(
        f'Kalman mocap velocity tuner: {node.pose_topic}\n'
        'Close this window or press Ctrl-C to stop')
    figure.subplots_adjust(
        left=0.10, right=0.97, bottom=0.30, top=0.91, hspace=0.16)

    initial = node.estimator_settings.copy()
    slider_left = 0.17
    slider_width = 0.66
    slider_height = 0.025
    sliders = {
        'measurement': Slider(
            figure.add_axes([slider_left, 0.200, slider_width, slider_height]),
            'KF position noise (m)', 0.0001, 0.05,
            valinit=initial['kalman_position_stddev_m'], valstep=0.0001,
            valfmt='%.4f',
        ),
        'process': Slider(
            figure.add_axes([slider_left, 0.140, slider_width, slider_height]),
            'KF acceleration noise (m/s²)', 0.1, 50.0,
            valinit=initial['kalman_acceleration_stddev_mps2'], valstep=0.1,
        ),
        'initial_velocity': Slider(
            figure.add_axes([slider_left, 0.080, slider_width, slider_height]),
            'KF initial velocity noise (m/s)', 0.1, 10.0,
            valinit=initial['kalman_initial_velocity_stddev_mps'], valstep=0.1,
        ),
    }
    sliders['measurement'].on_changed(
        lambda value: node.update_tuning(
            kalman_position_stddev_m=float(value)))
    sliders['process'].on_changed(
        lambda value: node.update_tuning(
            kalman_acceleration_stddev_mps2=float(value)))
    sliders['initial_velocity'].on_changed(
        lambda value: node.update_tuning(
            kalman_initial_velocity_stddev_mps=float(value)))

    reset_button = Button(
        figure.add_axes([0.85, 0.015, 0.10, 0.035]), 'Reset sliders')

    def _reset_sliders(_event) -> None:
        for slider in sliders.values():
            slider.reset()

    reset_button.on_clicked(_reset_sliders)
    figure.text(
        0.10, 0.025,
        'Slider changes affect this plot only; mocap_bridge is unchanged.',
        fontsize=9,
    )

    closed = threading.Event()
    figure.canvas.mpl_connect('close_event', lambda _event: closed.set())
    refresh_period_s = 1.0 / node.refresh_rate_hz

    while rclpy.ok() and not closed.is_set():
        snapshots = node.snapshots()
        newest_time = None
        for series_name, (times, velocities) in snapshots.items():
            if times.size:
                newest = float(times[-1])
                newest_time = newest if newest_time is None else max(
                    newest_time, newest)
            for component_index in range(3):
                lines[(component_index, series_name)].set_data(
                    times, velocities[:, component_index])
            speed = (
                np.linalg.norm(velocities, axis=1)
                if velocities.size else np.empty(0)
            )
            lines[(3, series_name)].set_data(times, speed)

        if newest_time is not None:
            left = max(0.0, newest_time - node.history_s)
            right = max(node.history_s, newest_time)
            axes[-1].set_xlim(left, right)
        figure.canvas.draw_idle()
        plt.pause(refresh_period_s)


def _spin(executor: SingleThreadedExecutor) -> None:
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MocapVelocityPlotter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin_thread.start()
    try:
        show_interactive_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
