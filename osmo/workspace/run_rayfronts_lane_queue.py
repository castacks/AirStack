#!/usr/bin/env python3
"""Run the remaining one-cell shared-RAVEN missions on one held OSMO pod.

The nested Docker daemon exposes host GPU indices, while OSMO's outer task
reports only its two assigned devices renumbered from zero. Consequently the
correct Isaac/offboard indices vary by worker. This wrapper materializes each
committed mission with the worker-audited indices immediately before launch.
Every child still goes through run_held_mission.sh and its <12 h hard bound.
"""

import argparse
from pathlib import Path
import subprocess
import sys

import yaml


LANES = {
    "a": [
        "firesuburbanl1v1", "firesuburbanl3v1",
        "hurricanesuburbanl2v1", "tornadosuburbanl1v1",
        "tornadosuburbanl3v1", "earthquakesuburbanl2v1",
        "fireurbanl1v1", "fireurbanl3v1", "hurricaneurbanl2v1",
        "tornadourbanl1v1", "tornadourbanl3v1", "earthquakeurbanl2v1",
    ],
    "b": [
        "firesuburbanl2v1", "hurricanesuburbanl1v1",
        "hurricanesuburbanl3v1", "tornadosuburbanl2v1",
        "earthquakesuburbanl1v1", "earthquakesuburbanl3v1",
        "fireurbanl2v1", "hurricaneurbanl1v1", "hurricaneurbanl3v1",
        "tornadourbanl2v1", "earthquakeurbanl1v1", "earthquakeurbanl3v1",
    ],
}


def launcher_is_held(pid: int) -> bool:
    stat = Path(f"/proc/{pid}/stat").read_text().split()[2]
    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
    return stat == "T" and b"mission_launcher.sh" in cmd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher-pid", type=int, required=True)
    parser.add_argument("--lane", choices=sorted(LANES), required=True)
    parser.add_argument("--isaac-gpu", required=True)
    parser.add_argument("--offboard-gpu", required=True)
    parser.add_argument(
        "--start-at",
        help="Resume at this scene key instead of replaying earlier lane cells",
    )
    parser.add_argument("--root", type=Path, default=Path("/root/AirStack"))
    args = parser.parse_args()

    if not launcher_is_held(args.launcher_pid):
        raise SystemExit(f"launcher {args.launcher_pid} is not held")

    mission_dir = args.root / "osmo" / "missions"
    scenes = LANES[args.lane]
    if args.start_at:
        if args.start_at not in scenes:
            raise SystemExit(
                f"start scene {args.start_at!r} is not in lane {args.lane}")
        scenes = scenes[scenes.index(args.start_at):]

    for scene in scenes:
        if not launcher_is_held(args.launcher_pid):
            raise SystemExit(f"launcher {args.launcher_pid} lost its hold")
        source = mission_dir / f"raven_{scene}_raven_remaining_2gpu1.yaml"
        mission = yaml.safe_load(source.read_text())
        assert mission["iterations"] == 1
        assert len(mission["environments"]) == 1
        mission["env"]["ISAAC_SIM_ACTIVE_GPU"] = str(args.isaac_gpu)
        mission["env"]["OFFBOARD_COMPUTE_GPU"] = str(args.offboard_gpu)
        assert mission["env"]["ZED_TIME_SLICE_GROUPS"] == "8"
        assert mission["env"]["ZED_TIME_SLICE_BURST"] == "8"
        assert mission["env"]["ZED_HYDRA_TIME_SLICE"] == "true"
        search = next(s["action"] for s in mission["steps"]
                      if s.get("action", {}).get("task") == "semantic_search")
        takeoff = next(s["action"] for s in mission["steps"]
                       if s.get("action", {}).get("task") == "takeoff")
        # At low RTF the takeoff action can be accepted and ascending for
        # several wall minutes before the relay forwards its first feedback.
        # The generic 15 s default then resends a duplicate goal; the task
        # correctly rejects that duplicate as "another task is already active"
        # even though the original later succeeds, and the runner falsely
        # fails an otherwise healthy team takeoff. Wait for the accepted
        # action's real result/feedback instead of manufacturing duplicates.
        takeoff["feedback_timeout_s"] = 300
        assert search["timeout_s"] == 21600
        assert search["goal"]["max_sim_seconds"] == 600.0

        # Takeoff can succeed and then a single MAVROS odometry stream can
        # disappear.  semantic_search_task intentionally derives both pose and
        # its 600-s budget from converted odometry, so starting without a fresh
        # stream produces an invalid robot path and used to hang until the
        # six-hour outer timeout.  Require two advancing reliable samples on
        # every robot immediately before dispatching the team search.
        search_index = next(
            i for i, step in enumerate(mission["steps"])
            if step.get("action", {}).get("task") == "semantic_search")
        mission["steps"].insert(search_index, {
            "run": {
                "container": "airstack-robot-desktop-{n}",
                "timeout_s": 60,
                "cmd": (
                    "python3 - <<'PY'\n"
                    "import time\n"
                    "import rclpy\n"
                    "from nav_msgs.msg import Odometry\n"
                    "from rclpy.qos import QoSProfile, ReliabilityPolicy\n"
                    "rclpy.init()\n"
                    "node = rclpy.create_node('benchmark_odom_freshness')\n"
                    "stamps = []\n"
                    "qos = QoSProfile(depth=10, "
                    "reliability=ReliabilityPolicy.BEST_EFFORT)\n"
                    "def cb(msg):\n"
                    "    stamp = float(msg.header.stamp.sec) + "
                    "float(msg.header.stamp.nanosec) * 1e-9\n"
                    "    if not stamps or stamp > stamps[-1]: stamps.append(stamp)\n"
                    "node.create_subscription(Odometry, "
                    "'/robot_{n}/odometry_conversion/odometry', cb, "
                    "qos)\n"
                    "deadline = time.monotonic() + 30.0\n"
                    "while len(stamps) < 2 and time.monotonic() < deadline:\n"
                    "    rclpy.spin_once(node, timeout_sec=1.0)\n"
                    "node.destroy_node(); rclpy.shutdown()\n"
                    "assert len(stamps) >= 2 and stamps[-1] > stamps[0], stamps\n"
                    "print(f'ODOMETRY_FRESH robot_{n} "
                    "{stamps[0]:.3f}->{stamps[-1]:.3f}')\n"
                    "PY\n"
                ),
            },
        })

        # Action success proves each semantic-search task reached its terminal
        # condition, but it does not prove every robot accumulated the full
        # scored window.  A live failure left one planner orphaned while the
        # other seven returned normally.  Validate the independently dumped
        # per-robot duration before landing, collection, or upload.
        #
        # Do not gate on the raw raven_nav completion_reason here.  The 600 s
        # budget is owned by semantic_search_task, while raven_nav only writes
        # `coverage` itself; consequently a healthy budget-limited run leaves
        # the raw robot JSON reason as `in_progress`.  The preceding action's
        # 8/8 success plus this duration check is the cross-node completion
        # contract.  The compiled team result records mission_reason as
        # `sim_time_budget`.
        search_index = next(
            i for i, step in enumerate(mission["steps"])
            if step.get("action", {}).get("task") == "semantic_search")
        mission["steps"].insert(search_index + 1, {
            "run": {
                "container": "airstack-robot-desktop-{n}",
                "timeout_s": 120,
                "cmd": (
                    "python3 - <<'PY'\n"
                    "import json\n"
                    "p = '/root/.cache/raven_results/robot_{n}.json'\n"
                    "d = json.load(open(p))\n"
                    "duration = float(d.get('mission_duration_s') or 0.0)\n"
                    "assert 598.5 <= duration <= 610.0, (p, duration)\n"
                    "print(f'RAVEN_WINDOW_OK {p} {duration:.2f}s')\n"
                    "PY\n"
                ),
            },
        })

        runtime = Path("/tmp") / f"{source.stem}.lane_{args.lane}.yaml"
        runtime.write_text(yaml.safe_dump(mission, sort_keys=False, width=100))
        print(f"[{scene}] GPU Isaac={args.isaac_gpu} "
              f"offboard={args.offboard_gpu}", flush=True)
        rc = subprocess.call([
            "bash", str(args.root / "osmo/workspace/run_held_mission.sh"),
            str(args.launcher_pid), str(runtime),
        ], cwd=args.root)
        if rc != 0:
            print(f"[{scene}] stopped queue after rc={rc}", file=sys.stderr)
            return rc
        print(f"[{scene}] passed, uploaded and verified", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
