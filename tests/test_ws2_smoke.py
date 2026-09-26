"""WS2 adapter for the existing AirStack system-test harness."""

import json
import os
import shutil
import subprocess
import uuid
from pathlib import Path

import pytest

import conftest


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "robot/ros_ws/ws2_runtime"
SCENARIO = ROOT / "tools/ws2_bench/scenarios/validate_timeout.yaml"


@pytest.mark.ws2_smoke
@pytest.mark.timeout(1800)
@pytest.mark.skipif(
    os.environ.get("WS2_SMOKE_READY") != "1",
    reason="run through system-tests.yml with -m ws2_smoke",
)
def test_ws2_office_kim_smoke():
    output = RUNTIME / f"ci-smoke-{uuid.uuid4().hex[:12]}"
    stdout = ""
    returncode = -1
    try:
        completed = subprocess.run(
            [
                "python3",
                str(ROOT / "tools/ws2_bench/episode.py"),
                str(SCENARIO),
                "--output",
                str(output),
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=1700,
            check=False,
        )
        stdout = completed.stdout
        returncode = completed.returncode
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or "WS2 episode timed out"
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
    finally:
        subprocess.run(
            [
                "docker", "stop", "--timeout", "3", "ws2-episode-worker",
                "collision-avoidance-airstack", "isaac-sim",
                "airstack-robot-desktop-1",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        artifact = conftest.RUN_DIR / "ws2-smoke"
        artifact.mkdir(parents=True, exist_ok=True)
        (artifact / "episode-stdout.log").write_text(stdout, encoding="utf-8")
        if output.is_dir():
            shutil.copytree(output, artifact / "episode", dirs_exist_ok=True)

    assert returncode == 0, stdout[-4000:]
    result_path = output / "result.json"
    assert result_path.is_file(), "WS2 did not produce result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result.get("outcome") != "infrastructure_error", result
    assert result.get("termination", {}).get("reason") == "simulation_time_budget", result
    assert result.get("planner_command_count", 0) >= 1, result
    assert not result.get("cleanup_errors"), result
