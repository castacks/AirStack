#!/usr/bin/env python3
"""
Adapter node: bridges the SUPER planner TCP sidecar to AirStack's trajectory controller.

The SUPER planner runs in its own Docker container (ROS Humble) and exposes a
newline-delimited JSON TCP server (default port 8768). This node:
  1. Subscribes to AirStack odometry and global_plan
  2. Sends step requests to the sidecar each tick
  3. Receives position+velocity commands
  4. Publishes them as TrajectoryXYZVYaw to the trajectory controller

Start the SUPER container before this node:
  SAFE_SUPER_IMAGE=super_planner:1 bash <path_to>/super.sh
"""

import json
import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw


class SuperPlannerNode(Node):

    def __init__(self):
        super().__init__('super_planner')

        self.declare_parameter('sidecar_host', '127.0.0.1')
        self.declare_parameter('sidecar_port', 8768)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('step_hz', 10.0)
        self.declare_parameter('max_lidar_points', 512)

        host = self.get_parameter('sidecar_host').value
        port = self.get_parameter('sidecar_port').value
        self._max_speed = self.get_parameter('max_speed').value
        step_hz = self.get_parameter('step_hz').value
        self._max_lidar_points = self.get_parameter('max_lidar_points').value

        self._odom: Odometry | None = None
        self._goal: list[float] | None = None
        self._goal_sent = False
        self._cloud: list[float] | None = None
        self._lock = threading.Lock()

        self._sidecar_host = host
        self._sidecar_port = port
        self._sock: socket.socket | None = None
        self._sock_file = None
        self._connect_sidecar()

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Odometry, 'odometry', self._odom_cb, best_effort)
        self.create_subscription(Path, 'global_plan', self._plan_cb, transient)
        self.create_subscription(PointCloud2, 'point_cloud', self._cloud_cb, best_effort)

        self._traj_pub = self.create_publisher(TrajectoryXYZVYaw, 'trajectory_override', 1)

        self.create_timer(1.0 / step_hz, self._step)

        self.get_logger().info(f'SUPER planner adapter ready — sidecar={host}:{port}')

    # ------------------------------------------------------------------
    # Sidecar connection
    # ------------------------------------------------------------------

    def _connect_sidecar(self):
        try:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((self._sidecar_host, self._sidecar_port))
            s.settimeout(0.1)
            self._sock = s
            self._sock_file = s.makefile('rb')
            self._send_raw({'type': 'reset'})
            self.get_logger().info('Connected to SUPER sidecar')
        except Exception as e:
            self._sock = None
            self._sock_file = None
            self.get_logger().warn(f'Sidecar connect failed: {e} — will retry on next step')

    def _send_raw(self, msg: dict) -> dict | None:
        if self._sock is None:
            return None
        try:
            data = (json.dumps(msg, separators=(',', ':')) + '\n').encode()
            self._sock.sendall(data)
            line = self._sock_file.readline()
            if not line:
                raise ConnectionResetError('sidecar closed connection')
            return json.loads(line.decode())
        except Exception as e:
            self.get_logger().warn(f'Sidecar error: {e} — reconnecting')
            self._connect_sidecar()
            return None

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def _plan_cb(self, msg: Path):
        if not msg.poses:
            return
        last = msg.poses[-1].pose.position
        new_goal = [last.x, last.y, last.z]
        with self._lock:
            if new_goal != self._goal:
                self._goal = new_goal
                self._goal_sent = False
        self.get_logger().info(f'New goal received: {new_goal}')

    def _cloud_cb(self, msg: PointCloud2):
        pts = _parse_pointcloud2(msg, self._max_lidar_points)
        with self._lock:
            self._cloud = pts

    # ------------------------------------------------------------------
    # Step timer
    # ------------------------------------------------------------------

    def _step(self):
        with self._lock:
            odom = self._odom
            goal = self._goal
            goal_sent = self._goal_sent
            cloud = self._cloud

        if odom is None or goal is None:
            return

        if self._sock is None:
            self._connect_sidecar()
            return

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular

        resp = self._send_raw({
            'type': 'step',
            'pos':      [p.x, p.y, p.z],
            'quat':     [q.x, q.y, q.z, q.w],
            'lin_vel':  [lv.x, lv.y, lv.z],
            'ang_vel':  [av.x, av.y, av.z],
            'target':   goal,
            'goal_sent': goal_sent,
            'lidar':    cloud,
        })

        if resp is None:
            return

        with self._lock:
            self._goal_sent = True

        cmd = resp.get('cmd')
        if cmd is None:
            return

        self._publish_trajectory(p, cmd)

    # ------------------------------------------------------------------
    # Trajectory conversion
    # ------------------------------------------------------------------

    def _publish_trajectory(self, current_pos, cmd: dict):
        cp = cmd['position']
        cv = cmd['velocity']

        speed = math.sqrt(cv['x'] ** 2 + cv['y'] ** 2 + cv['z'] ** 2)
        speed = max(0.1, min(speed, self._max_speed))

        yaw = math.atan2(
            cp['y'] - current_pos.y,
            cp['x'] - current_pos.x,
        )

        wp = WaypointXYZVYaw()
        wp.position = Point(x=float(cp['x']), y=float(cp['y']), z=float(cp['z']))
        wp.velocity = speed
        wp.yaw = yaw
        wp.acceleration = Vector3(x=0.0, y=0.0, z=0.0)
        wp.jerk = Vector3(x=0.0, y=0.0, z=0.0)

        traj = TrajectoryXYZVYaw()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'map'
        traj.waypoints = [wp]

        self._traj_pub.publish(traj)


def _parse_pointcloud2(msg: PointCloud2, max_points: int) -> list[float]:
    """Extract a flat [x,y,z,...] list from a PointCloud2 message, downsampled to max_points."""
    # Locate x, y, z field offsets
    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset
    if len(offsets) < 3:
        return []

    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
    ps = msg.point_step
    n_total = msg.width * msg.height
    data = bytes(msg.data)

    # Stride-based downsample
    stride = max(1, n_total // max_points)
    result: list[float] = []
    for i in range(0, n_total, stride):
        base = i * ps
        if base + oz + 4 > len(data):
            break
        x, y, z = (
            struct.unpack_from('<f', data, base + ox)[0],
            struct.unpack_from('<f', data, base + oy)[0],
            struct.unpack_from('<f', data, base + oz)[0],
        )
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            result += [x, y, z]
    return result


def main(args=None):
    rclpy.init(args=args)
    node = SuperPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
