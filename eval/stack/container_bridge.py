#!/usr/bin/env python3
"""In-container ROS 2 bridge for the S.A.F.E. eval harness.

Runs INSIDE the robot container (it needs rclpy + airstack_msgs, which the
host does not have) as a single long-lived `docker exec -i` process:

    stdin  ← JSON lines from the host harness
    stdout → JSON lines to the host harness

It only touches the stack's REAL interfaces:
    action     /{robot}/tasks/navigate                               (task_msgs/NavigateTask)
    publishes  /{robot}/global_plan                                (nav_msgs/Path)
    publishes  /{robot}/trajectory_controller/trajectory_override  (airstack_msgs/TrajectoryXYZVYaw)
    subscribes /{robot}/odometry_conversion/odometry               (frame_id + EKF pose passthrough)

(Scenario control and ground truth ride a direct TCP link to the sim — see
eval/stack/scenario_client.py — not this bridge.)

stdin message kinds:
    {"kind": "navigate",        "waypoints": [[x,y,z], ...], "goal_tolerance": 1.0}
    {"kind": "cancel_navigate"}
    {"kind": "global_plan",     "waypoints": [[x,y,z], ...]}
    {"kind": "traj_override",   "waypoints": [[x,y,z], ...], "velocity": 1.0}

stdout message kinds (one JSON object per line):
    {"kind": "odom", "frame": ..., "pos": [...]} — throttled EKF odometry
    {"kind": "navigate", "accepted": bool}       — NavigateTask goal response
    {"kind": "ready"}           — bridge is up and spinning
"""

import argparse
import base64
import zlib
import json
import queue
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
from rclpy.qos import qos_profile_sensor_data

try:
    from mav_msgs.msg import AttitudeThrust
except ImportError:
    AttitudeThrust = None

try:
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
except ImportError:  # workspace not built — global_plan / command paths still work
    TrajectoryXYZVYaw = WaypointXYZVYaw = None

try:
    from task_msgs.action import NavigateTask
    from action_msgs.srv import CancelGoal
except ImportError:
    NavigateTask = CancelGoal = None


def emit(obj: dict) -> None:
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


class SafeEvalBridge(Node):

    def __init__(self, robot: str):
        super().__init__(
            "safe_eval_bridge",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.robot = robot
        self._frame = "map"
        self._last_odom_emit = 0.0
        self._sensor_lock = threading.Lock()
        self._sensors = {}
        self.create_subscription(Image, f"/{robot}/sensors/front_stereo/left/depth_ground_truth",
                                 lambda msg: self._on_image("depth", msg), qos_profile_sensor_data)
        self.create_subscription(Image, f"/{robot}/sensors/front_stereo/left/image_rect",
                                 lambda msg: self._on_image("rgb", msg), qos_profile_sensor_data)
        self.create_subscription(Imu, f"/{robot}/sensors/imu", self._on_imu,
                                 qos_profile_sensor_data)
        self.att_pub = (self.create_publisher(
            AttitudeThrust, f"/{robot}/interface/attitude_thrust_command", 10)
            if AttitudeThrust is not None else None)

        self.plan_pub = self.create_publisher(Path, f"/{robot}/global_plan", 1)
        self.traj_pub = (
            self.create_publisher(TrajectoryXYZVYaw,
                                  f"/{robot}/trajectory_controller/trajectory_override", 1)
            if TrajectoryXYZVYaw is not None else None
        )

        self.create_subscription(Odometry, f"/{robot}/odometry_conversion/odometry",
                                 self._on_odom, 10)

        # The planner only starts following a plan once a NavigateTask goal is
        # active; a bare global_plan publish is stored but never executed.
        self.navigate_client = (
            ActionClient(self, NavigateTask, f"/{robot}/tasks/navigate")
            if NavigateTask is not None else None
        )
        # Cancel-all through the action's own service: a goal left by another
        # client would otherwise make the planner reject every new one.
        self.cancel_client = (
            self.create_client(CancelGoal, f"/{robot}/tasks/navigate/_action/cancel_goal")
            if CancelGoal is not None else None
        )
        self._navigate_handle = None

        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self.create_timer(0.02, self._drain_inbox)
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        emit({"kind": "ready"})

    # ── ROS → host ────────────────────────────────────────────────────────

    def _on_image(self, key: str, msg: Image):
        # Preserve raw pixels, row stride and encoding. Decode exactly on the host.
        packed = base64.b64encode(zlib.compress(bytes(msg.data), level=1)).decode('ascii')
        with self._sensor_lock:
            self._sensors[key] = {
                'width': msg.width, 'height': msg.height, 'step': msg.step,
                'encoding': msg.encoding, 'data': packed,
            }

    def _on_imu(self, msg: Imu):
        # ROS IMU is REP-103 FLU→ENU. SAFE's Pegasus contract is FRD→NED.
        # Convert explicitly so SAFE policy attitude decoding remains valid.
        q = msg.orientation
        q_ros = [q.x, q.y, q.z, q.w]
        def quat_mul(a, b):
            ax, ay, az, aw = a
            bx, by, bz, bw = b
            return [aw*bx + ax*bw + ay*bz - az*by,
                    aw*by - ax*bz + ay*bw + az*bx,
                    aw*bz + ax*by - ay*bx + az*bw,
                    aw*bw - ax*bx - ay*by - az*bz]
        q_safe = quat_mul(quat_mul(
            [0.7071067811865476, 0.7071067811865476, 0, 0], q_ros),
            [1, 0, 0, 0])
        omega = msg.angular_velocity
        with self._sensor_lock:
            self._sensors['imu'] = [float(v) for v in q_safe] + [
                float(omega.x), -float(omega.y), -float(omega.z)]
            self._sensors['imu_ros'] = [float(v) for v in q_ros] + [
                float(omega.x), float(omega.y), float(omega.z)]

    def _send_sensor_snapshot(self):
        with self._sensor_lock:
            payload = dict(self._sensors)
        emit({'kind': 'sensors', 'values': payload})

    def _on_odom(self, msg: Odometry):
        self._frame = msg.header.frame_id or self._frame
        t = self.get_clock().now().nanoseconds * 1e-9
        if t - self._last_odom_emit >= 1.0:
            self._last_odom_emit = t
            p = msg.pose.pose.position
            emit({"kind": "odom", "frame": self._frame,
                  "pos": [round(p.x, 3), round(p.y, 3), round(p.z, 3)]})

    # ── host → ROS ────────────────────────────────────────────────────────

    def _stdin_loop(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                self._inbox.put(json.loads(line))
            except (ValueError, TypeError):
                emit({"kind": "error", "error": f"bad stdin line: {line[:120]}"})

    def _drain_inbox(self):
        while True:
            try:
                msg = self._inbox.get_nowait()
            except queue.Empty:
                return
            try:
                self._handle(msg)
            except Exception as e:
                emit({"kind": "error", "error": str(e)})

    def _handle(self, msg: dict):
        kind = msg.get("kind")
        if kind == "navigate":
            self._send_navigate(msg["waypoints"], float(msg.get("goal_tolerance", 1.0)))
        elif kind == "cancel_navigate":
            self._cancel_navigate()
        elif kind == "global_plan":
            self.plan_pub.publish(self._make_path(msg["waypoints"]))
        elif kind == "sensor_snapshot":
            self._send_sensor_snapshot()
        elif kind == "attitude_thrust":
            if self.att_pub is None:
                raise RuntimeError("mav_msgs not available — build the workspace first")
            cmd = AttitudeThrust()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'map'
            q = msg['attitude_xyzw_enu_flu']
            cmd.attitude.x, cmd.attitude.y = float(q[0]), float(q[1])
            cmd.attitude.z, cmd.attitude.w = float(q[2]), float(q[3])
            cmd.thrust.z = float(msg['thrust'])
            self.att_pub.publish(cmd)
        elif kind == "traj_override":
            if self.traj_pub is None:
                raise RuntimeError("airstack_msgs not available — build the workspace first")
            self.traj_pub.publish(
                self._make_traj(msg["waypoints"], float(msg.get("velocity", 1.0))))
        else:
            raise ValueError(f"unknown kind '{kind}'")

    def _send_navigate(self, waypoints, goal_tolerance: float) -> None:
        if self.navigate_client is None:
            raise RuntimeError("task_msgs not available — build the workspace first")
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            emit({"kind": "navigate", "accepted": False, "error": "navigate action server absent"})
            return
        goal = NavigateTask.Goal()
        goal.global_plan = self._make_path(waypoints)
        goal.goal_tolerance_m = float(goal_tolerance)
        self.navigate_client.send_goal_async(goal).add_done_callback(self._on_navigate_response)

    def _on_navigate_response(self, future) -> None:
        handle = future.result()
        self._navigate_handle = handle if handle.accepted else None
        emit({"kind": "navigate", "accepted": bool(handle.accepted)})

    def _cancel_navigate(self) -> None:
        self._navigate_handle = None
        if self.cancel_client is not None and self.cancel_client.wait_for_service(timeout_sec=2.0):
            self.cancel_client.call_async(CancelGoal.Request())  # zero goal id cancels all

    def _make_path(self, waypoints) -> Path:
        path = Path()
        path.header.frame_id = self._frame
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = float(wp[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _make_traj(self, waypoints, velocity: float):
        traj = TrajectoryXYZVYaw()
        traj.header.frame_id = self._frame
        traj.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            w = WaypointXYZVYaw()
            w.position.x = float(wp[0])
            w.position.y = float(wp[1])
            w.position.z = float(wp[2])
            w.velocity = float(wp[3]) if len(wp) > 3 else velocity
            w.yaw = 0.0
            traj.waypoints.append(w)
        return traj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="robot_1")
    args = parser.parse_args()

    rclpy.init()
    node = SafeEvalBridge(args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
