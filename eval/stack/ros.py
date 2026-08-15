"""docker-exec helpers for driving the AirStack stack's real ROS 2 interfaces.

Mirrors the patterns proven in tests/conftest.py: every ROS interaction runs
inside a container via `docker exec <c> bash -c "<setup> && <cmd>"`, so the
host needs nothing but the docker CLI.
"""

from __future__ import annotations

import subprocess
import time

ROS_SETUP = "source /opt/ros/jazzy/setup.bash"
WS_SETUP = "source /root/AirStack/robot/ros_ws/install/setup.bash"


def docker_exec(container: str, cmd: str, timeout: float = 30.0,
                check: bool = False) -> subprocess.CompletedProcess:
    result = subprocess.run(
        ["docker", "exec", container, "bash", "-c", cmd],
        capture_output=True, text=True, timeout=timeout,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"docker exec failed ({result.returncode}) in {container}: {cmd}\n"
            f"stdout: {result.stdout[-2000:]}\nstderr: {result.stderr[-2000:]}")
    return result


def ros2_env(domain_id: int) -> str:
    return f"{ROS_SETUP} && {WS_SETUP} && export ROS_DOMAIN_ID={domain_id}"


def ros2_exec(container: str, ros2_cmd: str, domain_id: int = 1,
              timeout: float = 30.0, check: bool = False) -> subprocess.CompletedProcess:
    return docker_exec(container, f"{ros2_env(domain_id)} && {ros2_cmd}",
                       timeout=timeout, check=check)


def find_containers(name_pattern: str) -> list[str]:
    result = subprocess.run(
        ["docker", "ps", "--filter", f"name={name_pattern}", "--format", "{{.Names}}"],
        capture_output=True, text=True, timeout=15,
    )
    return sorted(n for n in result.stdout.split() if n)


def robot_container(index: int = 1) -> str:
    matches = [c for c in find_containers("robot") if c.endswith(f"-{index}")]
    if not matches:
        raise RuntimeError(f"robot container #{index} not running (docker ps shows none)")
    return matches[0]


def action_send_goal(container: str, action: str, action_type: str, goal_yaml: str,
                     domain_id: int = 1, timeout: float = 120.0) -> tuple[bool, str]:
    """ros2 action send_goal with the same success check the system tests use."""
    cmd = (f"timeout {int(timeout)} ros2 action send_goal --feedback "
           f"{action} {action_type} '{goal_yaml}'")
    result = ros2_exec(container, cmd, domain_id=domain_id, timeout=timeout + 15)
    ok = result.returncode == 0 and "success: true" in result.stdout.lower()
    return ok, result.stdout + result.stderr


def topic_echo_once(container: str, topic: str, domain_id: int = 1,
                    timeout: float = 10.0) -> subprocess.CompletedProcess:
    return ros2_exec(container, f"timeout {int(timeout)} ros2 topic echo --once {topic}",
                     domain_id=domain_id, timeout=timeout + 10)


def service_call(container: str, service: str, srv_type: str, request_yaml: str,
                 domain_id: int = 1, timeout: float = 20.0) -> subprocess.CompletedProcess:
    return ros2_exec(container,
                     f"timeout {int(timeout)} ros2 service call {service} {srv_type} '{request_yaml}'",
                     domain_id=domain_id, timeout=timeout + 10)


def wait_until(predicate, timeout: float, poll: float = 2.0, desc: str = "condition"):
    deadline = time.monotonic() + timeout
    last_err = None
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except Exception as e:  # containers may not be exec-able yet
            last_err = e
        time.sleep(poll)
    raise TimeoutError(f"timed out after {timeout:.0f}s waiting for {desc}"
                       + (f" (last error: {last_err})" if last_err else ""))
