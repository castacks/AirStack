#!/usr/bin/env python3
"""Host orchestrator: discover velocity topic, set PX4 params, run ZOH replay."""

from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path

ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"
ROBOT_SETUP = "/root/AirStack/robot/ros_ws/install/setup.bash"
DOMAIN = 1
ROBOT = "robot_1"
INNER = (
    "/root/AirStack/robot/ros_ws/src/interface/mavros_interface/scripts/replay_velocity_zoh.py"
)
PX4_PARAMS = {
    "MPC_XY_VEL_P_ACC": 1.8,
    "MPC_Z_VEL_P_ACC": 8.0,
    "MPC_XY_VEL_MAX": 0.6,
    "MPC_XY_P": 0.95,
    "MPC_Z_P": 5.0,
}


def _run(cmd, timeout=30):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def _docker_exec(container: str, inner: str, timeout: int = 30):
    wrapped = (
        f"source {ROS_DISTRO_SETUP} && source {ROBOT_SETUP} && "
        f"export ROS_DOMAIN_ID={DOMAIN} && {inner}"
    )
    return _run(["docker", "exec", container, "bash", "-lc", wrapped], timeout=timeout)


def find_robot() -> str:
    r = _run(["docker", "ps", "--filter", "name=robot", "--format", "{{.Names}}"])
    names = [n for n in r.stdout.splitlines() if n.strip()]
    if not names:
        raise SystemExit("no robot container — start with VERSION=0.19.0-alpha.5 airstack up")
    return sorted(names)[0]


def discover_velocity_topics(container: str) -> list[str]:
    r = _docker_exec(container, "ros2 topic list", timeout=20)
    topics = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    hits = [
        t
        for t in topics
        if "velocity" in t.lower() or t.endswith("/setpoint_raw/local") or t.endswith("/cmd_vel")
    ]
    print("velocity-related topics:")
    for t in hits:
        info = _docker_exec(container, f"ros2 topic info {t}", timeout=15)
        print(f"  {t}")
        print("   " + " | ".join(info.stdout.splitlines()[:6]))
    return hits


def set_px4_params(container: str) -> None:
    for name, value in PX4_PARAMS.items():
        inner = (
            f"ros2 service call /{ROBOT}/interface/mavros/param/set "
            f"mavros_msgs/srv/ParamSet "
            f"\"{{param_id: '{name}', value: {{integer: 0, real: {value}}}}}\""
        )
        r = _docker_exec(container, inner, timeout=20)
        ok = "success: true" in r.stdout or "true" in r.stdout
        print(f"param {name}={value} ok={ok}")
        if not ok:
            print(r.stdout[-400:])
            print(r.stderr[-400:])


def wait_px4(container: str, timeout_s: float = 300.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        r = _docker_exec(
            container,
            f"timeout 5 ros2 topic echo --once --csv --field connected "
            f"/{ROBOT}/interface/mavros/state",
            timeout=12,
        )
        if any(line.strip() == "True" for line in r.stdout.splitlines()):
            print("MAVROS connected")
            return
        print("waiting for MAVROS connected=True ...")
        time.sleep(3)
    raise SystemExit("PX4/MAVROS not ready")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("step", "full"), required=True)
    p.add_argument("--output", default="/tmp/velocity_zoh_result.npz")
    p.add_argument("--also-setpoint-raw", action="store_true")
    p.add_argument("--skip-param", action="store_true")
    p.add_argument("--skip-px4-wait", action="store_true")
    args = p.parse_args()

    container = find_robot()
    print(f"robot container: {container}")
    if not args.skip_px4_wait:
        wait_px4(container)
    discover_velocity_topics(container)
    if not args.skip_param:
        set_px4_params(container)

    extra = " --also-setpoint-raw" if args.also_setpoint_raw else ""
    inner = (
        f"python3 {INNER} --mode {args.mode} --output {args.output} "
        f"--robot {ROBOT}{extra}"
    )
    print(f"running: {inner}")
    timeout = 240 if args.mode == "step" else 240
    r = _docker_exec(container, inner, timeout=timeout)
    print(r.stdout)
    if r.returncode != 0:
        print(r.stderr)
        raise SystemExit(r.returncode)
    host_out = Path("tests/hw_sim_compare/results") / Path(args.output).name
    host_out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["docker", "cp", f"{container}:{args.output}", str(host_out)], check=False)
    subprocess.run(
        ["docker", "cp", f"{container}:{Path(args.output).with_suffix('.json')}", str(host_out.with_suffix('.json'))],
        check=False,
    )
    print(f"copied results to {host_out}")


if __name__ == "__main__":
    main()
