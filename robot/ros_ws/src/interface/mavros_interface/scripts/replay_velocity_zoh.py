#!/usr/bin/env python3
"""Zero-order-hold replay of hardware TwistStamped velocity commands into PX4.

Does not use Takeoff / Navigate / Land. Commands are held at 20 Hz (50 ms).
Yaw is always 0. Primary publish topic is /{robot}/interface/velocity_command
(geometry_msgs/TwistStamped, frame_id=map).
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from airstack_msgs.srv import RobotCommand
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import ParamSet, ParamSetV2
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_NPZ = Path(__file__).resolve().parent / "velocity_replay.npz"

PX4_PARAMS = {
    "MPC_XY_VEL_P_ACC": 1.8,
    "MPC_Z_VEL_P_ACC": 8.0,
    "MPC_XY_VEL_MAX": 0.6,
    "MPC_XY_P": 0.95,
    "MPC_Z_P": 5.0,
}


def _best_effort():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class VelocityZohReplay(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("velocity_zoh_replay")
        self.args = args
        self.robot = args.robot
        self.t0 = None
        self.pose_t: list[float] = []
        self.pose_xyz: list[list[float]] = []
        self.vel_t: list[float] = []
        self.vel_xyz: list[list[float]] = []
        self.mavros_state = State()

        data = np.load(args.npz)
        self.cmd_t = np.asarray(data["command_times"], dtype=float)
        self.cmd_v = np.asarray(data["velocity_command"], dtype=float)
        self.hw_pose_t = np.asarray(data["pose_times"], dtype=float)
        self.hw_pose = np.asarray(data["pose_position"], dtype=float)

        self.twist_pub = self.create_publisher(
            TwistStamped, f"/{self.robot}/interface/velocity_command", 10
        )
        self.raw_pub = self.create_publisher(
            PositionTarget, f"/{self.robot}/interface/mavros/setpoint_raw/local", 10
        )
        self.create_subscription(
            State, f"/{self.robot}/interface/mavros/state", self._on_state, 10
        )
        self.create_subscription(
            PoseStamped,
            f"/{self.robot}/interface/mavros/local_position/pose",
            self._on_pose,
            _best_effort(),
        )
        self.create_subscription(
            Odometry,
            f"/{self.robot}/odometry_conversion/odometry",
            self._on_odom,
            _best_effort(),
        )
        self.cmd_cli = self.create_client(
            RobotCommand, f"/{self.robot}/interface/robot_command"
        )
        self.param_cli_v2 = self.create_client(
            ParamSetV2, f"/{self.robot}/interface/mavros/param/set"
        )
        self._spin_ok = True
        self._spin_thread = threading.Thread(target=self._spin_bg, daemon=True)
        self._spin_thread.start()
        self.get_logger().info(
            f"loaded {len(self.cmd_t)} velocity samples, "
            f"t=[{self.cmd_t[0]:.3f},{self.cmd_t[-1]:.3f}] s"
        )

    def _spin_bg(self):
        while self._spin_ok and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def _on_state(self, msg: State):
        self.mavros_state = msg

    def _on_pose(self, msg: PoseStamped):
        if self.t0 is None:
            return
        t = time.monotonic() - self.t0
        p = msg.pose.position
        self.pose_t.append(t)
        self.pose_xyz.append([p.x, p.y, p.z])

    def _on_odom(self, msg: Odometry):
        if self.t0 is None:
            return
        t = time.monotonic() - self.t0
        v = msg.twist.twist.linear
        self.vel_t.append(t)
        self.vel_xyz.append([v.x, v.y, v.z])
        if not self.pose_t or t - self.pose_t[-1] > 0.015:
            p = msg.pose.pose.position
            self.pose_t.append(t)
            self.pose_xyz.append([p.x, p.y, p.z])

    def publish_enu(self, vx: float, vy: float, vz: float):
        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = "map"
        if self.args.invert_z:
            vz = -vz
        tw.twist.linear.x = float(vx)
        tw.twist.linear.y = float(vy)
        tw.twist.linear.z = float(vz)
        self.twist_pub.publish(tw)
        if self.args.also_setpoint_raw:
            raw = PositionTarget()
            raw.header = tw.header
            raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            raw.type_mask = (
                PositionTarget.IGNORE_PX
                | PositionTarget.IGNORE_PY
                | PositionTarget.IGNORE_PZ
                | PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW_RATE
            )
            raw.velocity.x = float(vy)
            raw.velocity.y = float(vx)
            raw.velocity.z = float(-vz)
            raw.yaw = 0.0
            self.raw_pub.publish(raw)

    def robot_command(self, command: int, timeout: float = 10.0) -> bool:
        if not self.cmd_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("robot_command service not available")
            return False
        req = RobotCommand.Request()
        req.command = command
        fut = self.cmd_cli.call_async(req)
        deadline = time.time() + timeout
        while time.time() < deadline and not fut.done():
            time.sleep(0.05)
        ok = bool(fut.done() and fut.result() and fut.result().success)
        self.get_logger().info(f"robot_command {command} success={ok}")
        return ok

    def stream_until(self, vx, vy, vz, seconds: float, hz: float = 20.0):
        dt = 1.0 / hz
        n = max(1, int(seconds * hz))
        for _ in range(n):
            self.publish_enu(vx, vy, vz)
            time.sleep(dt)

    def wait_px4(self, timeout: float = 180.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.5)
            if self.mavros_state.connected:
                self.get_logger().info(
                    f"MAVROS connected mode={self.mavros_state.mode} armed={self.mavros_state.armed}"
                )
                return True
            self.get_logger().info("waiting for MAVROS connected ...")
        return False

    def set_px4_params(self) -> None:
        for name, value in PX4_PARAMS.items():
            ok = False
            if self.param_cli_v2.wait_for_service(timeout_sec=3.0):
                req = ParamSetV2.Request()
                req.force_set = True
                req.param_id = name
                req.value = ParameterValue()
                req.value.type = ParameterType.PARAMETER_DOUBLE
                req.value.double_value = float(value)
                fut = self.param_cli_v2.call_async(req)
                deadline = time.time() + 8.0
                while time.time() < deadline and not fut.done():
                    time.sleep(0.02)
                ok = bool(fut.done() and fut.result() and fut.result().success)
            self.get_logger().info(f"PX4 param {name}={value} ok={ok}")

    def enter_offboard(self) -> bool:
        self.get_logger().info("streaming zero velocity before OFFBOARD")
        self.stream_until(0.0, 0.0, 0.0, 2.5)
        if not self.robot_command(RobotCommand.Request.REQUEST_CONTROL):
            self.get_logger().error("REQUEST_CONTROL failed")
            return False
        self.stream_until(0.0, 0.0, 0.0, 0.5)
        if not self.robot_command(RobotCommand.Request.ARM):
            self.get_logger().error("ARM failed")
            return False
        self.stream_until(0.0, 0.0, 0.0, 0.5)
        time.sleep(0.2)
        self.get_logger().info(
            f"after arm: mode={self.mavros_state.mode} armed={self.mavros_state.armed}"
        )
        return self.mavros_state.armed or self.mavros_state.mode == "OFFBOARD"

    def run_full(self):
        if not self.enter_offboard():
            raise SystemExit("PX4 never entered offboard/armed")
        self.t0 = time.monotonic()
        self.get_logger().info("playing full velocity stream (ZOH 20 Hz)")
        t_wall0 = time.monotonic()
        i = 0
        n = len(self.cmd_t)
        t_offset = float(self.cmd_t[0])
        while i < n:
            now = time.monotonic() - t_wall0
            while i + 1 < n and (self.cmd_t[i + 1] - t_offset) <= now:
                i += 1
            vx, vy, vz = self.cmd_v[i]
            self.publish_enu(vx, vy, vz)
            target = self.cmd_t[i] - t_offset
            # hold until next sample (or 50 ms if last)
            next_t = (self.cmd_t[i + 1] - t_offset) if i + 1 < n else target + 0.05
            sleep_s = next_t - (time.monotonic() - t_wall0)
            if sleep_s > 0:
                time.sleep(min(sleep_s, 0.05))
            if i + 1 >= n and sleep_s <= 0:
                break
        self.get_logger().info(f"full replay done, recorded {len(self.pose_t)} poses")

    def run_step(self):
        if not self.enter_offboard():
            raise SystemExit("PX4 never entered offboard/armed")
        self.t0 = time.monotonic()
        self.get_logger().info("step: climb to hover with vz=+0.8")
        deadline = time.time() + 15.0
        while time.time() < deadline:
            z = self.pose_xyz[-1][2] if self.pose_xyz else 0.0
            if z >= 1.0:
                break
            self.publish_enu(0.0, 0.0, 0.8)
            time.sleep(0.05)
        self.get_logger().info(f"hover at z={self.pose_xyz[-1][2] if self.pose_xyz else float('nan'):.3f}")
        self.stream_until(0.0, 0.0, 0.0, 2.0)
        step_t0 = time.monotonic() - self.t0
        self.get_logger().info(f"step v=(0,-0.6,0) at t={step_t0:.3f}s")
        self.stream_until(0.0, -0.6, 0.0, self.args.step_hold_s)
        self.stream_until(0.0, 0.0, 0.0, 1.0)
        self.get_logger().info(f"step done, recorded {len(self.pose_t)} poses, {len(self.vel_t)} vels")
        return step_t0

    def save(self, extra: dict):
        out = Path(self.args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "sim_pose_t": np.asarray(self.pose_t, dtype=float),
            "sim_pose": np.asarray(self.pose_xyz, dtype=float) if self.pose_xyz else np.zeros((0, 3)),
            "sim_vel_t": np.asarray(self.vel_t, dtype=float),
            "sim_vel": np.asarray(self.vel_xyz, dtype=float) if self.vel_xyz else np.zeros((0, 3)),
            "hw_pose_t": self.hw_pose_t,
            "hw_pose": self.hw_pose,
            "cmd_t": self.cmd_t,
            "cmd_v": self.cmd_v,
            "mode": np.array(self.args.mode),
        }
        np.savez(out, **payload)
        meta = {
            "mode": self.args.mode,
            "robot": self.robot,
            "n_sim_pose": len(self.pose_t),
            "n_sim_vel": len(self.vel_t),
            "mavros_mode": self.mavros_state.mode,
            "armed": bool(self.mavros_state.armed),
            **extra,
        }
        meta_path = out.with_suffix(".json")
        meta_path.write_text(json.dumps(meta, indent=2))
        self.get_logger().info(f"wrote {out} and {meta_path}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("step", "full"), required=True)
    p.add_argument("--npz", type=Path, default=DEFAULT_NPZ)
    p.add_argument("--robot", default="robot_1")
    p.add_argument("--output", default="/tmp/velocity_zoh_result.npz")
    p.add_argument("--step-hold-s", type=float, default=3.0)
    p.add_argument(
        "--also-setpoint-raw",
        action="store_true",
        help="also publish FRAME_LOCAL_NED PositionTarget (debug / if interface rebuild missing)",
    )
    p.add_argument(
        "--invert-z",
        action="store_true",
        help="negate vz (diagnostic if +up takeoff dives / sits on the floor)",
    )
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = VelocityZohReplay(args)
    extra = {}
    try:
        if not node.wait_px4():
            raise SystemExit("MAVROS never connected")
        node.set_px4_params()
        if args.mode == "full":
            node.run_full()
        else:
            extra["step_t0_s"] = node.run_step()
        node.save(extra)
    finally:
        node._spin_ok = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
