#!/usr/bin/env python3
"""Standalone command-timing recorder for AirStack person-tracking experiments.

Records how long after Isaac Sim starts playing the drone is commanded to take
off and target tracking is first enabled, appending one JSON line per event
to ``records/command_timing_records.jsonl`` (next to the scene loader's
``patch_records.jsonl``).

Why a separate process: the timing must be observed over ROS 2, but doing that
inside the Isaac Sim process hard-crashes its embedded rclpy. This script instead
runs in the **robot-desktop container**, which has a normal, full ROS 2
environment (the same place ``run_sequence.py`` issues the commands from), so it
is completely decoupled from the simulator.

Timing reference ("T+0"):
    The first ``/clock`` message after launch -- i.e. the instant Isaac Sim
    starts playing. The recorder writes both wall-clock and simulation-clock
    delays. Use ``sim_delay_s`` for scene-time milestones; wall time can drift
    when Isaac Sim runs slower or faster than real time.
    ``sequence_delay_s`` is the wall-clock delta since the previous recorded
    event, matching the current sequence runner's relative ``delay_s`` sleeps.

Observed signals (no custom message types required):
    * takeoff  -- ``/<robot>/tasks/takeoff/_action/status``
                  (action_msgs/GoalStatusArray): each new goal to become active
                  (STATUS_ACCEPTED or STATUS_EXECUTING).
    * takeoff_finished -- each takeoff goal to reach a terminal status
                          (STATUS_SUCCEEDED, STATUS_ABORTED, or STATUS_CANCELED).
    * tracking -- ``/<robot>/bpmp/target_tracking_enable``
                  (std_msgs/Bool): the first ``True``.

Usage (inside the robot-desktop container, ROS 2 sourced):
    python3 record_command_timing.py [--robot-name robot_1] [--output-dir DIR]
                                     [--config-name scene_0/2-eval.yaml]
                                     [--exit-when-done]

    docker exec airstack-robot-desktop-1 bash -c \
        "source /root/.bashrc && python3 \
         /root/AirStack/simulation/isaac-sim/launch_scripts/record_command_timing.py \
         --exit-when-done"

Run it before commanding takeoff (e.g. alongside the sequence runner): it blocks
until ``/clock`` appears, then listens. With ``--exit-when-done`` it exits once
tracking is enabled; otherwise it keeps listening until interrupted.
"""

import argparse
import json
import os
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def log(msg: str) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


def short_config_name(path: str) -> str:
    """Render a config path as "<parent_dir>/<filename>" (e.g. scene_0/2-eval.yaml).

    Mirrors the naming used by the scene loader's patch_records.jsonl so timing
    records can be cross-referenced to the run that produced them.
    """
    if not path:
        return None
    return os.path.join(os.path.basename(os.path.dirname(path)), os.path.basename(path))


class CommandTimingRecorder(Node):
    """Anchors T+0 on the first ``/clock`` and records the takeoff/tracking delays.

    A goal is "active" (the command has been issued and is running) once it leaves
    UNKNOWN for ACCEPTED/EXECUTING. action_msgs/GoalStatus has no STATUS_ACTIVE --
    the live states are STATUS_ACCEPTED (1) and STATUS_EXECUTING (2); the rest are
    terminal.
    """

    ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
    TERMINAL_STATUSES = (
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELED,
    )

    def __init__(self, robot_name, output_dir, config_name, scene, exit_when_done):
        super().__init__("command_timing_recorder")
        self.robot_name = robot_name
        self.output_dir = output_dir
        self.config_name = config_name
        self.scene = scene
        self.exit_when_done = exit_when_done
        self.record_path = os.path.join(output_dir, "command_timing_records.jsonl")

        self.takeoff_topic = f"/{robot_name}/tasks/takeoff/_action/status"
        self.tracking_topic = f"/{robot_name}/bpmp/target_tracking_enable"

        self._t0_wall = None
        self._t0_sim = None
        self._latest_sim = None
        self._last_record_wall_delay_s = 0.0
        self._last_record_sim_delay_s = 0.0
        self._takeoff_goal_count = 0
        self._takeoff_attempt_by_goal_id = {}
        self._recorded_takeoff_goal_ids = set()
        self._recorded_finished_goal_ids = set()
        self._tracking_recorded = False
        self.done = False

        # /clock is sim-time; subscribe best-effort so we are compatible whether
        # the bridge publishes it reliable or best-effort.
        clock_qos = QoSProfile(depth=1)
        clock_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        clock_qos.history = QoSHistoryPolicy.KEEP_LAST
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)

        # Match the action-status QoS (reliable, transient-local, keep-last) so a
        # status latched just before we subscribe is still delivered.
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = QoSReliabilityPolicy.RELIABLE
        status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        status_qos.history = QoSHistoryPolicy.KEEP_LAST
        self.create_subscription(
            GoalStatusArray, self.takeoff_topic, self._on_takeoff_status, status_qos
        )
        self.create_subscription(
            Bool, self.tracking_topic, self._on_tracking_enable, 10
        )

        log(f"Waiting for Isaac Sim to start playing (monitoring /clock)...")
        log(f"  takeoff  : {self.takeoff_topic}")
        log(f"  tracking : {self.tracking_topic}")
        log(f"  records  : {self.record_path}")

    # ----------------------------- callbacks --------------------------------
    def _on_clock(self, msg):
        self._latest_sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self._t0_wall is None:
            self._t0_wall = time.monotonic()
            self._t0_sim = self._latest_sim
            log(f"Isaac Sim is playing -- T+0 anchored. Recording command timing.")

    def _on_takeoff_status(self, msg):
        if self._t0_wall is None or self._tracking_recorded:
            return

        for status in msg.status_list:
            goal_id = self._goal_id_hex(status)
            if (
                status.status in self.ACTIVE_STATUSES
                and goal_id not in self._recorded_takeoff_goal_ids
            ):
                self._recorded_takeoff_goal_ids.add(goal_id)
                self._takeoff_goal_count += 1
                self._takeoff_attempt_by_goal_id[goal_id] = self._takeoff_goal_count
                self._write(
                    "takeoff",
                    f"/{self.robot_name}/tasks/takeoff",
                    goal_id=goal_id,
                    takeoff_attempt=self._takeoff_goal_count,
                    action_status=status.status,
                )

            if (
                status.status in self.TERMINAL_STATUSES
                and goal_id not in self._recorded_finished_goal_ids
            ):
                self._recorded_finished_goal_ids.add(goal_id)
                takeoff_attempt = self._takeoff_attempt_by_goal_id.get(goal_id)
                self._write(
                    "takeoff_finished",
                    f"/{self.robot_name}/tasks/takeoff",
                    goal_id=goal_id,
                    takeoff_attempt=takeoff_attempt,
                    action_status=status.status,
                )

    def _on_tracking_enable(self, msg):
        if self._tracking_recorded or self._t0_wall is None or not msg.data:
            return
        self._tracking_recorded = True
        self._write("enable_tracking", self.tracking_topic)
        self.done = True
        log("Tracking enabled; recorder is done.")

    # ------------------------------ output ----------------------------------
    def _goal_id_hex(self, status):
        return bytes(status.goal_info.goal_id.uuid).hex()

    def _write(self, event, target, **extra):
        wall_delay_s = round(time.monotonic() - self._t0_wall, 3)
        sim_delay_s = (
            round(self._latest_sim - self._t0_sim, 3)
            if self._latest_sim is not None and self._t0_sim is not None
            else None
        )
        relative_wall_delay_s = round(
            wall_delay_s - self._last_record_wall_delay_s, 3
        )
        relative_sim_delay_s = (
            round(sim_delay_s - self._last_record_sim_delay_s, 3)
            if sim_delay_s is not None and self._last_record_sim_delay_s is not None
            else None
        )
        record = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "scene": self.scene,
            "config_name": self.config_name,
            "robot_name": self.robot_name,
            "event": event,
            "target": target,
            # Backward-compatible field. Prefer wall_delay_s or sim_delay_s so
            # the intended clock is explicit at the call site.
            "delay_s": wall_delay_s,
            "wall_delay_s": wall_delay_s,
            "sim_delay_s": sim_delay_s,
            "relative_wall_delay_s": relative_wall_delay_s,
            "relative_sim_delay_s": relative_sim_delay_s,
            "sequence_delay_s": relative_wall_delay_s,
        }
        record.update(extra)
        os.makedirs(self.output_dir, exist_ok=True)
        with open(self.record_path, "a") as handle:
            handle.write(json.dumps(record) + "\n")
        self._last_record_wall_delay_s = wall_delay_s
        self._last_record_sim_delay_s = sim_delay_s
        log(
            f"Recorded {event}: wall_delay_s={wall_delay_s}, "
            f"relative_wall_delay_s={relative_wall_delay_s}, "
            f"sim_delay_s={sim_delay_s}, "
            f"relative_sim_delay_s={relative_sim_delay_s} -> {self.record_path}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="AirStack command-timing recorder")
    parser.add_argument(
        "--robot-name",
        default=os.environ.get("ROBOT_NAME", "robot_1"),
        help="Robot namespace (default: $ROBOT_NAME, else robot_1).",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "records"),
        help="Directory for command_timing_records.jsonl (default: ./records "
        "next to this script).",
    )
    parser.add_argument(
        "--config-name",
        default=None,
        help="Config tag for the record. Defaults to '<dir>/<file>' derived from "
        "$SCENE_CONFIG_PATH.",
    )
    parser.add_argument(
        "--exit-when-done",
        action="store_true",
        help="Exit once both takeoff and tracking have been recorded.",
    )
    args = parser.parse_args()

    config_name = args.config_name or short_config_name(
        os.environ.get("SCENE_CONFIG_PATH", "")
    )
    scene = os.environ.get("SCENE_USD_PATH", "")
    scene = os.path.splitext(os.path.basename(scene))[0] if scene else None

    rclpy.init()
    node = CommandTimingRecorder(
        robot_name=args.robot_name,
        output_dir=args.output_dir,
        config_name=config_name,
        scene=scene,
        exit_when_done=args.exit_when_done,
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.exit_when_done and node.done:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy installs its own SIGINT/SIGTERM handlers; a Ctrl-C or kill/timeout
        # surfaces here as ExternalShutdownException. Exit cleanly either way.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    log("Recorder stopped.")


if __name__ == "__main__":
    main()
