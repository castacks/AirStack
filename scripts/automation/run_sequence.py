#!/usr/bin/env python3
"""Automated sequence runner for AirStack.

Waits for Isaac Sim to start playing (detected via /clock topic),
then executes steps from a YAML config at configured time intervals.

Usage:
    python3 run_sequence.py [--config sequence.yaml] [--dry-run]
"""

import argparse
import subprocess
import sys
import threading
import time
import yaml
from datetime import datetime
from pathlib import Path


def log(msg: str) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


def run_cmd(cmd: str, dry_run: bool = False) -> subprocess.CompletedProcess:
    log(f"$ {cmd}")
    if dry_run:
        return subprocess.CompletedProcess(args=cmd, returncode=0, stdout="", stderr="")
    # Source ~/.bashrc so that shell functions like `airstack` are available
    wrapped = f'bash -i -c "{cmd}"'
    result = subprocess.run(wrapped, shell=True, capture_output=True, text=True)
    if result.stdout.strip():
        log(f"  → {result.stdout.strip()}")
    if result.returncode != 0:
        log(f"  ERROR (exit {result.returncode}): {result.stderr.strip()}")
    return result


def wait_for_isaac_sim_playing(container: str, poll_interval_s: float = 2.0) -> None:
    """Block until /clock is published inside the robot container."""
    log("Waiting for Isaac Sim to start playing (monitoring /clock)...")
    while True:
        result = subprocess.run(
            f'docker exec {container} bash -c '
            f'"source /root/.bashrc && timeout 3 ros2 topic echo /clock --once 2>/dev/null"',
            shell=True, capture_output=True, text=True,
        )
        if result.returncode == 0 and result.stdout.strip():
            log("Isaac Sim is playing — starting sequence.")
            return
        time.sleep(poll_interval_s)


def execute_step(step: dict, container: str, dry_run: bool) -> None:
    action = step["action"]
    name = step.get("name", action)
    log(f"[{name}] action={action}")

    if action == "ros2_publish":
        topic = step["topic"]
        msg_type = step["msg_type"]
        msg = step["msg"]
        run_cmd(
            f'docker exec {container} bash -c '
            f'"source /root/.bashrc && ros2 topic pub --once {topic} {msg_type} \'{msg}\'"',
            dry_run,
        )

    elif action == "ros2_action":
        server = step["action_server"]
        atype = step["action_type"]
        goal = step["goal"]
        run_cmd(
            f'docker exec {container} bash -c '
            f'"source /root/.bashrc && ros2 action send_goal {server} {atype} \'{goal}\'"',
            dry_run,
        )

    elif action == "airstack_up":
        args = step.get("args", "")
        run_cmd(f"airstack up {args}".strip(), dry_run)

    elif action == "airstack_down":
        args = step.get("args", "")
        run_cmd(f"airstack down {args}".strip(), dry_run)

    elif action == "parallel":
        threads = [
            threading.Thread(target=execute_step, args=(s, container, dry_run))
            for s in step.get("steps", [])
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    else:
        log(f"  WARNING: unknown action '{action}', skipping.")


def main() -> None:
    parser = argparse.ArgumentParser(description="AirStack automated sequence runner")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).parent / "sequence.yaml"),
        help="Path to sequence YAML config (default: sequence.yaml next to this script)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without executing them",
    )
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    container = cfg.get("robot_container", "airstack-robot-desktop-1")
    isaac_container = cfg.get("isaac_sim_container", "isaac-sim")
    steps = cfg.get("steps", [])

    if args.dry_run:
        log("=== DRY RUN MODE — no commands will be executed ===")

    wait_for_isaac_sim_playing(isaac_container)

    for i, step in enumerate(steps):
        delay = step.get("delay_s", 0)
        name = step.get("name", f"step_{i}")
        log(f"Waiting {delay}s before '{name}'...")
        time.sleep(delay)
        execute_step(step, container, args.dry_run)

    log("Sequence complete.")


if __name__ == "__main__":
    main()
