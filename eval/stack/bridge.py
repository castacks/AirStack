"""Host-side handle on the in-container bridge (container_bridge.py).

One `docker exec -i` process per stack bring-up. JSON lines out via stdin,
JSON lines in via stdout — see container_bridge.py for the message kinds.
Publishes to the stack's real topics (global_plan, trajectory_override) and
relays EKF odometry; ground truth and scenario control come from
scenario_client.py instead.
"""

from __future__ import annotations

import json
import base64
import zlib

import numpy as np
import os
import subprocess
import threading

from .ros import ros2_env


class BridgeError(RuntimeError):
    pass


class StackBridge:

    def __init__(self, container: str, robot: str = "robot_1", domain_id: int = 1):
        self.container = container
        self.robot = robot
        self.domain_id = domain_id
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Condition()
        self._odom_frame: str | None = None
        self._odom_pos: list | None = None
        self._navigate_ack: bool | None = None
        self._sensors: dict = {}
        self._sensor_revision = 0
        self._ready = False
        self._stderr_tail: list[str] = []

    # ── lifecycle ─────────────────────────────────────────────────────────

    def start(self, timeout: float = 60.0) -> None:
        # eval/ is not mounted in the robot container — copy the script in.
        script = os.path.join(os.path.dirname(__file__), "container_bridge.py")
        subprocess.run(["docker", "cp", script, f"{self.container}:/tmp/container_bridge.py"],
                       check=True, capture_output=True, timeout=30)
        cmd = (f"{ros2_env(self.domain_id)} && "
               f"python3 /tmp/container_bridge.py --robot {self.robot}")
        self._proc = subprocess.Popen(
            ["docker", "exec", "-i", self.container, "bash", "-c", cmd],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )
        threading.Thread(target=self._stdout_loop, daemon=True).start()
        threading.Thread(target=self._stderr_loop, daemon=True).start()
        with self._lock:
            self._lock.wait_for(lambda: self._ready or self._proc.poll() is not None,
                                timeout=timeout)
            if not self._ready:
                raise BridgeError(
                    "container bridge failed to start: "
                    + "".join(self._stderr_tail[-20:]))

    def stop(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None

    @property
    def alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    # ── reader threads ────────────────────────────────────────────────────

    def _stdout_loop(self):
        assert self._proc is not None
        for line in self._proc.stdout:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except (ValueError, TypeError):
                continue
            kind = msg.get("kind")
            with self._lock:
                if kind == "odom":
                    self._odom_frame = msg.get("frame")
                    self._odom_pos = msg.get("pos")
                elif kind == "sensors":
                    self._sensors = msg.get("values") or {}
                    self._sensor_revision += 1
                elif kind == "navigate":
                    self._navigate_ack = bool(msg.get("accepted"))
                elif kind == "ready":
                    self._ready = True
                self._lock.notify_all()

    def _stderr_loop(self):
        assert self._proc is not None
        for line in self._proc.stderr:
            self._stderr_tail.append(line)
            del self._stderr_tail[:-50]

    # ── writes ────────────────────────────────────────────────────────────

    def _send(self, obj: dict) -> None:
        if not self.alive:
            raise BridgeError("container bridge is not running")
        self._proc.stdin.write(json.dumps(obj) + "\n")
        self._proc.stdin.flush()

    def navigate(self, waypoints, goal_tolerance: float, timeout: float = 15.0) -> bool:
        """Send a NavigateTask goal and return whether the planner accepted it."""
        with self._lock:
            self._navigate_ack = None
        self._send({"kind": "navigate",
                    "waypoints": [[float(v) for v in wp] for wp in waypoints],
                    "goal_tolerance": float(goal_tolerance)})
        with self._lock:
            self._lock.wait_for(lambda: self._navigate_ack is not None or not self.alive,
                                timeout=timeout)
            return bool(self._navigate_ack)

    def cancel_navigate(self) -> None:
        if self.alive:
            self._send({"kind": "cancel_navigate"})

    def publish_global_plan(self, waypoints) -> None:
        self._send({"kind": "global_plan",
                    "waypoints": [[float(v) for v in wp] for wp in waypoints]})

    def publish_traj_override(self, waypoints, velocity: float) -> None:
        self._send({"kind": "traj_override",
                    "waypoints": [[float(v) for v in wp] for wp in waypoints],
                    "velocity": float(velocity)})

    def publish_attitude_thrust(self, xyzw_enu_flu, thrust: float) -> None:
        self._send({'kind': 'attitude_thrust',
                    'attitude_xyzw_enu_flu': [float(x) for x in xyzw_enu_flu],
                    'thrust': float(thrust)})

    def publish_setpoint(self, position_enu, velocity_enu, acceleration_enu,
                         yaw_enu: float, yaw_rate_enu: float | None) -> None:
        """Complete ROS-ENU planner reference for MAVROS local setpoint."""
        self._send({'kind': 'setpoint',
                    'position': [float(x) for x in position_enu],
                    'velocity': [float(x) for x in velocity_enu],
                    'acceleration': [float(x) for x in acceleration_enu],
                    'yaw': float(yaw_enu),
                    'yaw_rate': None if yaw_rate_enu is None else float(yaw_rate_enu)})

    def clear_external_command(self) -> None:
        """Stop the bridge's offboard setpoint heartbeat."""
        if self.alive:
            self._send({'kind': 'clear_external_command'})

    def latest_sensor_obs(self, timeout: float = 5.0) -> dict:
        """Request one actual ROS snapshot; absent frames stay absent (never fabricated)."""
        with self._lock:
            revision = self._sensor_revision
        self._send({'kind': 'sensor_snapshot'})
        with self._lock:
            self._lock.wait_for(lambda: self._sensor_revision > revision or not self.alive,
                                timeout=timeout)
            raw = dict(self._sensors)
        out = {}
        for key in ('depth', 'rgb'):
            if key in raw:
                out[key] = _decode_ros_image(raw[key])
        if 'lidar' in raw:
            out['lidar'] = _decode_ros_pointcloud(raw['lidar'])
        for key in ('imu', 'imu_ros'):
            if key in raw:
                out[key] = np.asarray(raw[key], dtype=np.float32)
        return out

    # ── state access ──────────────────────────────────────────────────────

    def latest_odom_pos(self) -> list | None:
        with self._lock:
            return list(self._odom_pos) if self._odom_pos else None


def _decode_ros_image(image: dict) -> np.ndarray:
    """Decode ROS Image with row padding and explicit metric depth units."""
    encoding = image['encoding'].lower()
    formats = {
        '32fc1': (np.float32, 1), '16uc1': (np.uint16, 1),
        'mono8': (np.uint8, 1), 'rgb8': (np.uint8, 3),
        'bgr8': (np.uint8, 3), 'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4),
    }
    if encoding not in formats:
        raise BridgeError(f"unsupported ROS image encoding {encoding!r}")
    dtype, channels = formats[encoding]
    width, height, step = int(image['width']), int(image['height']), int(image['step'])
    raw = zlib.decompress(base64.b64decode(image['data']))
    row_bytes = width * channels * np.dtype(dtype).itemsize
    if len(raw) != height * step or step < row_bytes:
        raise BridgeError('invalid ROS image payload length or row stride')
    arr = np.ndarray((height, step // np.dtype(dtype).itemsize), dtype=dtype,
                     buffer=raw)[:, :width * channels].copy()
    if channels > 1:
        arr = arr.reshape(height, width, channels)
    if encoding == '16uc1':
        return arr.astype(np.float32) / 1000.0
    if encoding == 'bgr8':
        return arr[..., ::-1].copy()
    if encoding == 'bgra8':
        return arr[..., [2, 1, 0]].copy()
    if encoding == 'rgba8':
        return arr[..., :3].copy()
    return arr


def _decode_ros_pointcloud(cloud: dict) -> np.ndarray:
    """Extract finite XYZ from a PointCloud2 without assuming packed field order."""
    fields = {f['name']: f for f in cloud['fields']}
    if not all(k in fields for k in ('x', 'y', 'z')):
        raise BridgeError('PointCloud2 lacks x/y/z fields')
    if any(int(fields[k]['datatype']) != 7 for k in ('x', 'y', 'z')):
        raise BridgeError('PointCloud2 x/y/z must be FLOAT32')
    width, height = int(cloud['width']), int(cloud['height'])
    point_step, row_step = int(cloud['point_step']), int(cloud['row_step'])
    raw = zlib.decompress(base64.b64decode(cloud['data']))
    if len(raw) != height * row_step or point_step < 12 or row_step < width * point_step:
        raise BridgeError('invalid PointCloud2 payload length or stride')
    byte_order = '>' if cloud.get('is_bigendian') else '<'
    out = np.empty((width * height, 3), dtype=np.float32)
    for i, key in enumerate(('x', 'y', 'z')):
        offset = int(fields[key]['offset'])
        if offset < 0 or offset + 4 > point_step:
            raise BridgeError(f'invalid PointCloud2 {key} field offset')
        view = np.ndarray((height, width), dtype=np.dtype(byte_order + 'f4'),
                          buffer=raw, offset=offset, strides=(row_step, point_step))
        out[:, i] = view.reshape(-1)
    return out[np.isfinite(out).all(axis=1)]
