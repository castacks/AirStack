#!/usr/bin/env python3
"""Verify and recover the bidirectional DDS routes used by team planners.

An eight-robot team planner runs on ROS domain 0 while each robot runs on its
own domain.  A DDS Router process in each robot container carries observations
to domain 0 and plans back to that robot.  Under a heavily loaded simulator a
router can remain alive while one side of that route is silent.  Process-only
health checks therefore miss the failure.

This helper has two host-side gates:

``sensors``
    Subscribe on domain 0 to the five inputs the planner needs from every
    robot.  Restart only the main router for robots with missing streams and
    probe again.

``plans``
    Require each robot's MIGHTY bridge to log that it adopted a real
    ``global_plan``.  Restart the main router for missing reverse routes and
    wait for the continuously running planner to republish.

The private ``probe`` and ``restart-main-router`` modes execute inside a
container.  The main and gossip routers are intentionally distinguished by
their exact log filter: only ``--log-filter DDSROUTER`` is eligible for a
restart; ``DDSROUTER|DDSPIPE|FASTDDS`` is the separate gossip path.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


DEFAULT_CONTAINER = "offboard-compute"
CONTAINER_SCRIPT = "/tmp/team_dds_route_guard.py"
SETUP_BASH = "/root/AirStack/robot/ros_ws/install/setup.bash"


def _run(argv: list[str], timeout: float = 60.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        argv,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )


def _docker_python(container: str, args: list[str], timeout: float) -> subprocess.CompletedProcess:
    # OSMO mounts robot/ros_ws into the compute containers, not the osmo/
    # directory that contains this host helper. Copy the same small script to
    # /tmp before invoking either private container-side mode.
    copied = _run(
        ["docker", "cp", str(Path(__file__).resolve()),
         f"{container}:{CONTAINER_SCRIPT}"],
        timeout=15,
    )
    if copied.returncode != 0:
        return copied
    command = "source " + SETUP_BASH + " && exec python3 " + CONTAINER_SCRIPT
    # Arguments are deliberately separate docker-exec argv entries. bash -lc
    # consumes the first argument after the command as $0, so use a fixed
    # sentinel before the Python arguments.
    return _run(
        [
            "docker", "exec", "-e", "ROS_DOMAIN_ID=0", container,
            "bash", "-lc", command + ' "$@"', "route-guard", *args,
        ],
        timeout=timeout,
    )


def _last_json(text: str) -> dict:
    for line in reversed((text or "").splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    raise ValueError("probe produced no JSON result")


def _probe_mode(args: argparse.Namespace) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, NavSatFix

    robots = range(1, args.robots + 1)
    topic_specs = []
    for n in robots:
        robot = f"robot_{n}"
        topic_specs.extend(
            [
                (n, "rgb", Image,
                 f"/{robot}/sensors/front_stereo/left/image_rect"),
                (n, "depth", Image,
                 args.depth_topic_template.format(robot=robot)),
                (n, "camera_info", CameraInfo,
                 f"/{robot}/sensors/front_stereo/left/camera_info"),
                (n, "odometry", Odometry,
                 f"/{robot}/odometry_conversion/odometry"),
                (n, "navsat", NavSatFix,
                 f"/{robot}/interface/mavros/global_position/global"),
            ]
        )

    rclpy.init()
    node = rclpy.create_node("team_dds_sensor_route_probe")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen: set[tuple[int, str]] = set()
    subscriptions = []

    def callback(key: tuple[int, str]):
        def receive(_msg):
            seen.add(key)
        return receive

    for n, label, msg_type, topic in topic_specs:
        subscriptions.append(
            node.create_subscription(msg_type, topic, callback((n, label)), qos)
        )

    deadline = time.monotonic() + args.timeout_s
    expected = {(n, label) for n, label, _, _ in topic_specs}
    while rclpy.ok() and time.monotonic() < deadline and seen != expected:
        rclpy.spin_once(node, timeout_sec=0.25)

    missing = {
        str(n): [label for nn, label, _, _ in topic_specs
                 if nn == n and (nn, label) not in seen]
        for n in robots
    }
    missing = {n: labels for n, labels in missing.items() if labels}
    result = {
        "ok": not missing,
        "seen": len(seen),
        "expected": len(expected),
        "missing": missing,
    }
    print(json.dumps(result, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not missing else 2


def _ddsrouter_processes() -> list[tuple[int, list[str]]]:
    found = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc / "cmdline").read_bytes().split(b"\0")
            argv = [part.decode(errors="replace") for part in argv if part]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if not argv or Path(argv[0]).name != "ddsrouter":
            continue
        if "-c" not in argv or "--log-filter" not in argv:
            continue
        try:
            config = argv[argv.index("-c") + 1]
            log_filter = argv[argv.index("--log-filter") + 1]
        except (IndexError, ValueError):
            continue
        if config.startswith("/tmp/dds_router_") and log_filter == "DDSROUTER":
            found.append((int(proc.name), argv))
    return found


def _restart_main_router_mode(_args: argparse.Namespace) -> int:
    routers = _ddsrouter_processes()
    if not routers:
        print("MAIN_DDS_ROUTER_NOT_FOUND", flush=True)
        return 1

    configs = []
    for pid, argv in routers:
        configs.append(argv[argv.index("-c") + 1])
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + 5.0
    remaining = {pid for pid, _ in routers}
    while remaining and time.monotonic() < deadline:
        remaining = {pid for pid in remaining if Path(f"/proc/{pid}").exists()}
        if remaining:
            time.sleep(0.1)
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    # Every matching process should use the same generated configuration. If
    # a stale duplicate existed, prefer the largest/newest surviving config;
    # the main config is much larger than the gossip config in any case.
    candidates = [Path(p) for p in set(configs) if Path(p).is_file()]
    if not candidates:
        print("MAIN_DDS_ROUTER_CONFIG_MISSING", flush=True)
        return 1
    config = max(candidates, key=lambda p: (p.stat().st_size, p.stat().st_mtime))
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/usr/local/lib:" + env.get("LD_LIBRARY_PATH", "")
    log_path = Path("/tmp/ddsrouter_dds_router.log")
    with log_path.open("ab", buffering=0) as log_file:
        child = subprocess.Popen(
            ["stdbuf", "-oL", "-eL", "ddsrouter", "-c", str(config),
             "--log-filter", "DDSROUTER"],
            stdin=subprocess.DEVNULL,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
            env=env,
        )
    time.sleep(1.0)
    if child.poll() is not None:
        print(f"MAIN_DDS_ROUTER_RESTART_FAILED exit={child.returncode}", flush=True)
        return 1
    print(
        f"MAIN_DDS_ROUTER_RESTARTED old={','.join(str(p) for p, _ in routers)} "
        f"new={child.pid} config={config}",
        flush=True,
    )
    return 0


def _restart_robot(robot: int) -> bool:
    container = f"airstack-robot-desktop-{robot}"
    result = _docker_python(container, ["restart-main-router"], timeout=20)
    print(f"[{container}] {result.stdout.strip()}", flush=True)
    return result.returncode == 0


def _sensors_mode(args: argparse.Namespace) -> int:
    probe_timeout = float(args.probe_timeout_s)
    for attempt in range(1, args.attempts + 1):
        result = _docker_python(
            args.container,
            [
                "probe", "--robots", str(args.robots),
                "--timeout-s", str(probe_timeout),
                "--depth-topic-template", args.depth_topic_template,
            ],
            timeout=probe_timeout + 30,
        )
        print(result.stdout.strip(), flush=True)
        try:
            report = _last_json(result.stdout)
        except ValueError as exc:
            print(f"TEAM_DDS_SENSOR_PROBE_INVALID: {exc}", flush=True)
            report = {"ok": False, "missing": {str(n): ["probe"]
                                                  for n in range(1, args.robots + 1)}}
        if result.returncode == 0 and report.get("ok"):
            print(f"TEAM_DDS_SENSORS_OK attempt={attempt}", flush=True)
            return 0
        missing = sorted(int(n) for n in report.get("missing", {}))
        print(f"TEAM_DDS_SENSORS_MISSING attempt={attempt} robots={missing}", flush=True)
        if attempt == args.attempts:
            break
        if not missing or not all(_restart_robot(n) for n in missing):
            return 1
        time.sleep(args.recovery_wait_s)
    return 1


def _robots_with_plan(robots: int) -> set[int]:
    adopted = set()
    for n in range(1, robots + 1):
        result = _run(
            [
                "docker", "exec", f"airstack-robot-desktop-{n}",
                "bash", "-c",
                "grep -ahq 'follower: adopted global_plan' /root/.ros/log/*.log",
            ],
            timeout=10,
        )
        if result.returncode == 0:
            adopted.add(n)
    return adopted


def _wait_for_plans(robots: int, timeout_s: float, poll_s: float) -> set[int]:
    deadline = time.monotonic() + timeout_s
    adopted: set[int] = set()
    while time.monotonic() < deadline:
        adopted = _robots_with_plan(robots)
        if len(adopted) == robots:
            return adopted
        time.sleep(poll_s)
    return _robots_with_plan(robots)


def _plans_mode(args: argparse.Namespace) -> int:
    expected = set(range(1, args.robots + 1))
    for attempt in range(1, args.attempts + 1):
        adopted = _wait_for_plans(args.robots, args.plan_timeout_s, args.poll_s)
        missing = sorted(expected - adopted)
        if not missing:
            print(f"TEAM_DDS_PLANS_OK attempt={attempt} robots={sorted(adopted)}", flush=True)
            return 0
        print(
            f"TEAM_DDS_PLANS_MISSING attempt={attempt} "
            f"adopted={sorted(adopted)} missing={missing}",
            flush=True,
        )
        if attempt == args.attempts:
            break
        if not all(_restart_robot(n) for n in missing):
            return 1
        time.sleep(args.recovery_wait_s)
    return 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)

    sensors = sub.add_parser("sensors", help="host-side shared sensor gate")
    sensors.add_argument("--robots", type=int, default=8)
    sensors.add_argument("--container", default=DEFAULT_CONTAINER)
    sensors.add_argument("--attempts", type=int, default=3)
    sensors.add_argument("--probe-timeout-s", type=float, default=25.0)
    sensors.add_argument("--recovery-wait-s", type=float, default=15.0)
    sensors.add_argument(
        "--depth-topic-template",
        default="/{robot}/sensors/front_stereo/left/depth_ground_truth",
    )
    sensors.set_defaults(func=_sensors_mode)

    plans = sub.add_parser("plans", help="host-side reverse plan gate")
    plans.add_argument("--robots", type=int, default=8)
    plans.add_argument("--attempts", type=int, default=3)
    plans.add_argument("--plan-timeout-s", type=float, default=45.0)
    plans.add_argument("--recovery-wait-s", type=float, default=15.0)
    plans.add_argument("--poll-s", type=float, default=5.0)
    plans.set_defaults(func=_plans_mode)

    probe = sub.add_parser("probe", help=argparse.SUPPRESS)
    probe.add_argument("--robots", type=int, default=8)
    probe.add_argument("--timeout-s", type=float, default=25.0)
    probe.add_argument(
        "--depth-topic-template",
        default="/{robot}/sensors/front_stereo/left/depth_ground_truth",
    )
    probe.set_defaults(func=_probe_mode)

    restart = sub.add_parser("restart-main-router", help=argparse.SUPPRESS)
    restart.set_defaults(func=_restart_main_router_mode)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if hasattr(args, "robots") and args.robots < 1:
        raise SystemExit("--robots must be >= 1")
    if hasattr(args, "attempts") and args.attempts < 1:
        raise SystemExit("--attempts must be >= 1")
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
