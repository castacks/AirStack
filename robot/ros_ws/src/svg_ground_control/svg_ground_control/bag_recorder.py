"""Rosbag recorder with a switch: ``ros2 bag record --all-topics`` started and
stopped over a ROS service, so the Foxglove basestation panel (or a shell)
can turn recording on and off mid-run.

    ros2 run svg_ground_control bag_recorder
    ros2 service call /bag_recorder/record std_srvs/srv/SetBool "{data: true}"
    ros2 service call /bag_recorder/record std_srvs/srv/SetBool "{data: false}"
    ros2 topic echo /svg/bag_recorder/status      # JSON: recording, path, size

``ground_control.launch.py`` starts this node next to the commander
(``use_bag_recorder``), and its ``record_bag:=true`` becomes this node's
``record_on_start`` — the whole-run recording it always offered, now with the
panel able to stop and restart it.

One bag per start at ``<bag_dir>/<bag_prefix>_<YYYYmmdd_HHMMSS>`` (mcap).
Everything published on the domain is captured: odometry, commands, mocap
poses, joystick, CBF debug, ``/rosout``; topics that appear later are picked
up too (``--all-topics`` keeps discovering). Stop sends SIGINT so
``ros2 bag record`` closes the file and writes ``metadata.yaml`` — a bag that
is killed hard has no metadata and ``ros2 bag info`` cannot read it — and
escalates to SIGTERM / SIGKILL only if the recorder ignores it.

Status (``status_topic``, ``std_msgs/String`` JSON, ``status_hz``)::

    {"recording": true, "path": ".../svg_20260927_143012", "started_at": ...,
     "duration_s": 12.4, "size_bytes": 5312000, "topic_count": 87,
     "bag_dir": "...", "bag_prefix": "svg", "last_bag": ..., "last_result": ...,
     "error": null}

``recording`` is the truth the panel's switch follows: the recorder process
is polled every tick, so a recorder that dies on its own (disk full, bad
output directory) flips it back to ``false`` with the exit code in ``error``.
"""

import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

# How long a SIGINT'd recorder gets to flush and write metadata.yaml before
# SIGTERM, and SIGTERM before SIGKILL. Closing an mcap of a long run takes a
# moment; killing early is what leaves an unreadable bag.
STOP_GRACE_S = 8.0
KILL_GRACE_S = 2.0


def bag_size_bytes(path: str) -> int:
    """Bytes under the bag directory (mcap chunks are flushed as they fill)."""
    total = 0
    try:
        for root, _dirs, files in os.walk(path):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
    except OSError:
        pass
    return total


class BagRecorderNode(Node):

    def __init__(self, **node_kwargs):
        super().__init__('bag_recorder', **node_kwargs)

        self.declare_parameter('bag_dir', '~/AirStack/robot/ros_ws/bags')
        self.declare_parameter('bag_prefix', 'svg')
        self.declare_parameter('storage', 'mcap')
        self.declare_parameter('include_hidden', False)
        # Extra `ros2 bag record` arguments (e.g. ['--max-cache-size', '0']);
        # empty by default. --all-topics / --storage / --output are ours.
        self.declare_parameter('extra_args', [''])
        self.declare_parameter('record_on_start', False)
        self.declare_parameter('status_topic', '/svg/bag_recorder/status')
        self.declare_parameter('status_hz', 2.0)

        self.proc = None            # subprocess.Popen while recording
        self.path = None            # bag directory of the current recording
        self.started_at = None      # wall-clock epoch seconds
        self.last_bag = None        # directory of the previous recording
        self.last_result = None     # human-readable outcome of the last stop
        self.error = None           # why the last start failed / recorder died

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        hz = max(0.2, float(self.get_parameter('status_hz').value))
        self.status_timer = self.create_timer(1.0 / hz, self.publish_status)

        # One switch (what the panel uses) plus start/stop triggers for the
        # shell, so `ros2 service call ~/start` needs no request body.
        self.create_service(SetBool, '~/record', self.on_record)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)

        self.get_logger().info(
            f"bag recorder ready: bag_dir={self.bag_dir()} prefix="
            f"{self.get_parameter('bag_prefix').value} — switch with "
            f"{self.get_name()}/record (std_srvs/SetBool)")
        if bool(self.get_parameter('record_on_start').value):
            ok, message = self.start()      # start() logs the bag path itself
            if not ok:
                self.get_logger().error(f'record_on_start failed: {message}')

    # ── parameters ──────────────────────────────────────────────────────────
    def bag_dir(self) -> str:
        return os.path.expanduser(str(self.get_parameter('bag_dir').value))

    def new_bag_path(self) -> str:
        prefix = str(self.get_parameter('bag_prefix').value).strip() or 'svg'
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.bag_dir(), f'{prefix}_{stamp}')
        # `ros2 bag record` refuses an existing output directory; two starts
        # inside one second get a suffix instead of a refusal.
        n = 1
        while os.path.exists(path):
            n += 1
            path = os.path.join(self.bag_dir(), f'{prefix}_{stamp}_{n}')
        return path

    def build_command(self, path: str) -> list:
        cmd = ['ros2', 'bag', 'record', '--all-topics',
               '--storage', str(self.get_parameter('storage').value),
               '--output', path]
        if bool(self.get_parameter('include_hidden').value):
            cmd.append('--include-hidden-topics')
        extra = [str(a) for a in self.get_parameter('extra_args').value if str(a)]
        return cmd + extra

    # ── recording ───────────────────────────────────────────────────────────
    def recording(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        """Start a new bag. Returns (ok, message)."""
        if self.recording():
            return True, f'already recording to {self.path}'
        self.reap()
        try:
            os.makedirs(self.bag_dir(), exist_ok=True)
        except OSError as error:
            self.error = f'cannot create bag_dir {self.bag_dir()}: {error}'
            return False, self.error
        path = self.new_bag_path()
        cmd = self.build_command(path)
        try:
            # The recorder's own output goes to our stdout, i.e. the launch
            # screen — the same place the old ExecuteProcess put it. No pipe:
            # an unread pipe would eventually block the recorder.
            self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                         stdout=sys.stdout, stderr=sys.stderr)
        except OSError as error:
            self.proc = None
            self.error = f'cannot start {cmd[0]}: {error}'
            return False, self.error
        self.path = path
        self.started_at = time.time()
        self.error = None
        message = f'recording all topics to {path}'
        self.get_logger().info(message)
        self.publish_status()
        return True, message

    def stop(self):
        """Stop the current bag cleanly. Returns (ok, message)."""
        if self.proc is None:
            return True, 'not recording'
        if self.proc.poll() is not None:
            # Died on its own; reap() records why.
            self.reap()
            return True, self.last_result or 'recorder had already exited'
        self.get_logger().info(f'stopping recording {self.path}')
        try:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    f'recorder ignored SIGINT for {STOP_GRACE_S:.0f} s, terminating')
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=KILL_GRACE_S)
                except subprocess.TimeoutExpired:
                    self.get_logger().error('recorder ignored SIGTERM, killing')
                    self.proc.kill()
                    self.proc.wait()
        except OSError as error:
            self.get_logger().error(f'stopping the recorder failed: {error}')
        self.reap(stopped=True)
        self.publish_status()
        return True, self.last_result

    def reap(self, stopped: bool = False):
        """Record the outcome of a finished recorder process and clear it."""
        if self.proc is None:
            return
        if self.proc.poll() is None:
            return
        code = self.proc.returncode
        size = bag_size_bytes(self.path) if self.path else 0
        duration = time.time() - self.started_at if self.started_at else 0.0
        summary = f'{self.path} ({duration:.0f} s, {size / 1e6:.1f} MB)'
        # SIGINT is the recorder's normal exit: return code 0, or -SIGINT
        # from a signal-terminated wrapper. Anything else is a failure.
        clean = code in (0, -signal.SIGINT)
        if stopped and clean:
            self.last_result = f'stopped: {summary}'
            self.error = None
        elif stopped:
            self.last_result = f'stopped with exit code {code}: {summary}'
            self.error = f'recorder exited with code {code}'
        else:
            self.last_result = f'recorder exited on its own with code {code}: {summary}'
            self.error = self.last_result
            self.get_logger().error(self.last_result)
        self.last_bag = self.path
        self.proc = None
        self.path = None
        self.started_at = None

    # ── services ────────────────────────────────────────────────────────────
    def on_record(self, request, response):
        ok, message = self.start() if request.data else self.stop()
        response.success = ok
        response.message = message
        return response

    def on_start(self, request, response):
        response.success, response.message = self.start()
        return response

    def on_stop(self, request, response):
        response.success, response.message = self.stop()
        return response

    # ── status ──────────────────────────────────────────────────────────────
    def build_status(self) -> dict:
        if self.proc is not None and self.proc.poll() is not None:
            self.reap()
        recording = self.recording()
        now = time.time()
        try:
            topic_count = len(self.get_topic_names_and_types())
        except Exception:      # graph unavailable during shutdown
            topic_count = None
        return {
            'stamp': now,
            'node': self.get_fully_qualified_name(),
            'recording': recording,
            'path': self.path,
            'started_at': self.started_at,
            'duration_s': (now - self.started_at) if recording and self.started_at else None,
            'size_bytes': bag_size_bytes(self.path) if recording and self.path else None,
            'topic_count': topic_count,
            'bag_dir': self.bag_dir(),
            'bag_prefix': str(self.get_parameter('bag_prefix').value),
            'storage': str(self.get_parameter('storage').value),
            'include_hidden': bool(self.get_parameter('include_hidden').value),
            'last_bag': self.last_bag,
            'last_result': self.last_result,
            'error': self.error,
        }

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self.build_status())
        self.status_pub.publish(msg)

    def destroy_node(self):
        # A launch Ctrl-C reaches the recorder too (same process group), so
        # this usually finds it already gone; a plain node shutdown must
        # still close the bag properly.
        if self.recording():
            self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
