#!/usr/bin/env python3
"""Rosbag recorder that auto-starts when target tracking is enabled.

Subscribes to bpmp/target_tracking_enable (std_msgs/Bool).
- True  → starts MCAP recording
- False → stops recording
- Node shutdown (container stop) → stops recording

Bag output name is derived from SCENE_CONFIG_PATH env var, with an
auto-incrementing 4-digit run index (since the recorder's -o name has no
timestamp, and ros2 bag record fails silently if the output dir already
exists):
  configs/scene_10/case_0/seeds_1234_5678.yaml → scene_10_seeds_1234_0001, _0002, ...
  configs/scene_2/2-eval.yaml                  → scene_2_2-eval_0001, _0002, ...
Falls back to "target_tracking_0001", ... if SCENE_CONFIG_PATH is not set.
"""
import copy
import os
import re
import signal
import subprocess
from pathlib import Path

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from bag_record_pid.bag_record_node import BagRecorderNode


def _next_indexed_name(base: str, output_dir: str, width: int = 4) -> str:
    """Return f'{base}_NNNN' for the smallest unused NNNN under output_dir.

    The recorder always uses the same scene-derived base name across runs
    (no timestamp), and ros2 bag record fails silently if -o points at a
    directory that already exists, so each run needs a fresh index.
    """
    try:
        existing = set(os.listdir(output_dir))
    except OSError:
        existing = set()
    pattern = re.compile(rf"^{re.escape(base)}_(\d{{{width}}})$")
    used = {int(m.group(1)) for name in existing if (m := pattern.match(name))}
    idx = 1
    while idx in used:
        idx += 1
    return f"{base}_{idx:0{width}d}"


def _bag_base_from_scene_config() -> str:
    """Derive a descriptive bag name base (without run index) from SCENE_CONFIG_PATH."""
    config_path = os.environ.get("SCENE_CONFIG_PATH", "")
    if not config_path:
        return "target_tracking"

    # Extract scene_N from the path (e.g. configs/scene_10/... → scene_10)
    match = re.search(r"(scene_\d+)", config_path)
    scene = match.group(1) if match else None

    # Translate container path to be accessible inside the robot container.
    # Isaac Sim container mounts the whole repo at /isaac-sim/AirStack/..., but
    # robot-desktop only mounts launch_scripts/ itself, at /root/launch_scripts
    # (see robot/docker/docker-compose.yaml robot-desktop volumes).
    robot_path = config_path.replace(
        "/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/", "/root/launch_scripts/"
    )

    # Try to read size_seed from the YAML; the run index (added per recording
    # start, see _next_indexed_name) takes the place of position_seed.
    try:
        with open(robot_path) as f:
            cfg = yaml.safe_load(f) or {}
        patch = cfg.get("patch", {})
        size_seed = patch.get("size_seed")
        if scene and size_seed is not None:
            return f"{scene}_seeds_{size_seed}"
    except Exception:
        pass

    # Fallback: scene + config filename stem (e.g. scene_2_2-eval)
    stem = Path(config_path).stem
    return f"{scene}_{stem}" if scene and stem else (scene or stem or "target_tracking")


class TargetTrackingBagRecorderNode(BagRecorderNode):
    def __init__(self):
        super().__init__()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(Bool, "bpmp/target_tracking_enable", self._on_enable, qos)
        self._bag_base = _bag_base_from_scene_config()
        self._bag_prefix = None
        self.get_logger().info(
            f"Target tracking bag recorder ready — bag base name: '{self._bag_base}'"
        )

    def _on_enable(self, msg: Bool):
        if msg.data:
            self.run()
        else:
            self.interrupt()

    def run(self):
        """Start recording. Uses scene-derived prefix instead of timestamped section name."""
        if self.active:
            return
        self.active = True
        self._bag_prefix = _next_indexed_name(self._bag_base, self.output_dir)

        for section_name, command_dict in self.commands.items():
            cmd = copy.deepcopy(command_dict["prefix"])
            cmd += ["-o", self._bag_prefix]
            if command_dict["suffix"]:
                cmd.extend(command_dict["suffix"])

            self.get_logger().warn(f"Starting recording to '{self._bag_prefix}'")
            self.process[section_name] = {
                "process": subprocess.Popen(cmd),
                "output_filename": self._bag_prefix,
            }
            self.process[section_name]["pid"] = self.process[section_name]["process"].pid
            self.get_logger().warn(
                f"Recording PID {self.process[section_name]['pid']} → {self._bag_prefix}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackingBagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.interrupt()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
