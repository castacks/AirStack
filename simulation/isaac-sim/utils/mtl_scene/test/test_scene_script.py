"""End-to-end check of search_mission_scene.py without Isaac (needs pxr)."""
import subprocess
import sys
from pathlib import Path

import pytest

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


def test_scene_script_authors_a_working_gimbal():
    pytest.importorskip("pxr")
    pytest.importorskip("yaml")
    harness = Path(__file__).with_name("scene_smoke_harness.py")
    r = subprocess.run([sys.executable, str(harness)], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and "SMOKE OK" in r.stdout, r.stdout + r.stderr
