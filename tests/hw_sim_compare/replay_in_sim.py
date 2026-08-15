#!/usr/bin/env python3
"""Replay a hardware waypoint mission through AirStack Takeoff/Navigate/Land."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"
ROBOT_SETUP = "/root/AirStack/robot/ros_ws/install/setup.bash"
DOMAIN = 1
ROBOT = "robot_1"


def _run(cmd: list[str], timeout: int = 30) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def _docker_exec(container: str, inner: str, timeout: int = 30) -> subprocess.CompletedProcess:
    wrapped = (
        f"source {ROS_DISTRO_SETUP} && source {ROBOT_SETUP} && "
        f"export ROS_DOMAIN_ID={DOMAIN} && {inner}"
    )
    return _run(["docker", "exec", container, "bash", "-c", wrapped], timeout=timeout)


def _find_robot() -> str:
    r = _run(["docker", "ps", "--filter", "name=robot", "--format", "{{.Names}}"])
    names = [n for n in r.stdout.splitlines() if n.strip()]
    if not names:
        raise SystemExit("no robot container running — start with: "
                         "VERSION=0.19.0-alpha.5 airstack up isaac-sim robot-desktop")
    return sorted(names)[0]


def _action_ok(stdout: str) -> bool:
    return "success: true" in stdout


def _action_message(stdout: str) -> str:
    for line in stdout.splitlines():
        s = line.strip()
        if s.startswith("message:"):
            return s[len("message:"):].strip().strip("'\"")
    return stdout[-400:]


def wait_px4(container: str, timeout_s: float = 300.0) -> None:
    deadline = time.time() + timeout_s
    connected = False
    while time.time() < deadline:
        if not connected:
            r = _docker_exec(
                container,
                f"timeout 5 ros2 topic echo --once --csv --field connected "
                f"/{ROBOT}/interface/mavros/state",
                timeout=12,
            )
            if any(line.strip() == "True" for line in r.stdout.splitlines()):
                connected = True
                print("MAVROS connected")
            else:
                print("waiting for MAVROS connected=True ...")
                time.sleep(2)
                continue
        r = _docker_exec(
            container,
            f"timeout 5 ros2 topic echo --once /{ROBOT}/interface/mavros/local_position/odom",
            timeout=12,
        )
        if r.returncode == 0 and "pose:" in r.stdout:
            print("PX4 local_position/odom is publishing")
            return
        print("waiting for local_position/odom ...")
        time.sleep(2)
    raise SystemExit("PX4/MAVROS not ready within timeout")


def start_bag(container: str, bag_name: str) -> tuple[int | None, str]:
    container_root = f"/bags/{bag_name}"
    cmd = (
        f"mkdir -p {container_root} && "
        f"nohup ros2 bag record -s mcap -o {container_root}/flight "
        f"/{ROBOT}/odometry_conversion/odometry "
        f"/{ROBOT}/interface/mavros/local_position/odom "
        f"/{ROBOT}/pose "
        f"/{ROBOT}/fmu/velocity_command "
        f"/{ROBOT}/interface/velocity_command "
        f"> {container_root}/record.log 2>&1 & echo $!"
    )
    r = _docker_exec(container, cmd, timeout=20)
    pid_txt = (r.stdout or "").strip().splitlines()
    pid = int(pid_txt[-1]) if pid_txt and pid_txt[-1].isdigit() else None
    if pid is None:
        print(f"warning: bag recorder may have failed: {r.stdout!r} {r.stderr!r}")
    else:
        print(f"recording bag {container_root}/flight pid={pid}")
    time.sleep(2.0)
    return pid, container_root


def stop_bag(container: str, pid: int | None, container_root: str) -> None:
    stop = (
        f"kill -INT {pid} 2>/dev/null || true; "
        f"for i in 1 2 3 4 5 6 7 8 9 10; do "
        f"  kill -0 {pid} 2>/dev/null || break; sleep 1; "
        f"done; "
        f"pkill -INT -f 'ros2 bag record.*{container_root}/flight' 2>/dev/null || true; "
        f"sleep 2"
    )
    _docker_exec(container, stop, timeout=40)
    print("bag recorder stopped")


def takeoff(container: str, altitude: float, velocity: float) -> None:
    timeout = max(30.0, altitude / max(velocity, 0.1) + 20.0)
    goal = f"{{target_altitude_m: {altitude}, velocity_m_s: {velocity}}}"
    print(f"TakeoffTask altitude={altitude}m velocity={velocity}m/s")
    r = _docker_exec(
        container,
        f'ros2 action send_goal --feedback /{ROBOT}/tasks/takeoff '
        f'task_msgs/action/TakeoffTask "{goal}"',
        timeout=int(timeout + 15),
    )
    if not _action_ok(r.stdout):
        raise SystemExit(f"takeoff failed: {_action_message(r.stdout)}\n{r.stderr}")
    print("takeoff succeeded")


def navigate_to(container: str, x: float, y: float, z: float, tol: float) -> None:
    goal = (
        "{global_plan: {header: {frame_id: \"map\"}, poses: ["
        f"{{pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}}}"
        f"]}}, goal_tolerance_m: {tol}}}"
    )
    print(f"NavigateTask -> ({x}, {y}, {z}) tol={tol}m")
    r = _docker_exec(
        container,
        f"ros2 action send_goal --feedback /{ROBOT}/tasks/navigate "
        f"task_msgs/action/NavigateTask '{goal}'",
        timeout=90,
    )
    if not _action_ok(r.stdout):
        raise SystemExit(f"navigate failed: {_action_message(r.stdout)}\n{r.stderr}")
    print(f"reached ({x}, {y}, {z})")


def land(container: str, velocity: float, altitude: float) -> None:
    timeout = max(30.0, altitude / max(velocity, 0.1) + 20.0)
    goal = f"{{velocity_m_s: {velocity}}}"
    print(f"LandTask velocity={velocity}m/s")
    r = _docker_exec(
        container,
        f'ros2 action send_goal --feedback /{ROBOT}/tasks/land '
        f'task_msgs/action/LandTask "{goal}"',
        timeout=int(timeout + 15),
    )
    if not _action_ok(r.stdout):
        raise SystemExit(f"land failed: {_action_message(r.stdout)}\n{r.stderr}")
    print("land succeeded")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--bag-name", default="")
    parser.add_argument("--skip-px4-wait", action="store_true")
    args = parser.parse_args()
    mission = json.loads(args.mission.read_text())
    container = _find_robot()
    print(f"robot container: {container}")

    if not args.skip_px4_wait:
        wait_px4(container)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    bag_name = args.bag_name or f"hw_replay_{stamp}"
    pid, root = start_bag(container, bag_name)
    try:
        takeoff(
            container,
            float(mission["takeoff_altitude_m"]),
            float(mission["takeoff_velocity_m_s"]),
        )
        time.sleep(2.0)
        for i, wp in enumerate(mission["waypoints"], 1):
            print(f"--- waypoint {i}/{len(mission['waypoints'])} ---")
            navigate_to(
                container, wp["x"], wp["y"], wp["z"],
                float(mission.get("goal_tolerance_m", 0.3)),
            )
        land(container, float(mission["land_velocity_m_s"]),
             float(mission["takeoff_altitude_m"]))
    finally:
        if pid is not None:
            stop_bag(container, pid, root)

    host_bag = Path("robot/bags") / bag_name / "flight"
    print(f"sim bag (host): {host_bag.resolve()}")
    print(
        "compare with:\n"
        f"  python tests/hw_sim_compare/compare_bags.py "
        f"--hw {mission['source_bag']} --sim {host_bag} "
        f"--mission {args.mission} -o tests/hw_sim_compare/compare.json"
    )


if __name__ == "__main__":
    main()
