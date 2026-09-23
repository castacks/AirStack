# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Contract tests for OSMO's robot/Isaac simulation-clock boot ordering."""
from pathlib import Path
import subprocess

import pytest


pytestmark = pytest.mark.unit

ROOT = Path(__file__).parents[2]
ENTRYPOINT = ROOT / "osmo/workspace/entrypoint.sh"


def _epoch_guard_source() -> str:
    source = ENTRYPOINT.read_text()
    start = source.index("reconcile_isaac_clock_epoch() {")
    end = source.index("\n}\n\nif [ \"${OSMO_AIRSTACK_UP", start) + 2
    return source[start:end]


def _run_guard(*, isaac_started: str, robot_started: str) -> str:
    """Run the extracted helper with Docker metadata mocks, never real Docker."""
    shell = r'''
log() { printf 'LOG:%s\n' "$*"; }
date() { printf '%s\n' 100; }
sleep() { :; }
docker() {
  if [ "$1" = inspect ] && [ "$2" = isaac-sim-livestream ] && [ "$#" -eq 2 ]; then
    return 0
  fi
  if [ "$1" = inspect ] && [ "$2" = isaac-sim ]; then
    return 1
  fi
  if [ "$1" = inspect ] && [ "$2" = -f ]; then
    if [ "${!#}" = isaac-sim-livestream ]; then
      printf '%s\n' "$ISAAC_STARTED"
    else
      printf '%s\n' "$ROBOT_STARTED"
    fi
    return 0
  fi
  if [ "$1:$2" = 'ps:--filter' ]; then
    printf 'airstack-robot-desktop-1\n'
    return 0
  fi
  if [ "$1" = restart ]; then
    printf 'RESTART:%s\n' "$2" >&2
    return 0
  fi
  return 1
}
eval "$1"
reconcile_isaac_clock_epoch
'''
    result = subprocess.run(
        ["bash", "-c", shell, "_", _epoch_guard_source()],
        capture_output=True,
        text=True,
        check=True,
        env={"ISAAC_STARTED": isaac_started, "ROBOT_STARTED": robot_started},
    )
    return result.stdout + result.stderr


def test_osmo_boot_restarts_only_robot_when_it_predates_isaac():
    output = _run_guard(
        isaac_started="2026-09-23T10:00:00Z",
        robot_started="2026-09-23T09:00:00Z",
    )
    assert "restarting only airstack-robot-desktop-1" in output
    assert "RESTART:airstack-robot-desktop-1" in output


def test_osmo_boot_leaves_current_robot_epoch_untouched():
    output = _run_guard(
        isaac_started="2026-09-23T09:00:00Z",
        robot_started="2026-09-23T10:00:00Z",
    )
    assert "already matches the Isaac container clock epoch" in output
    assert "RESTART:" not in output
