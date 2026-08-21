"""Hermetic contracts for bounded diagnostics and fail-fast readiness."""

import json
from types import SimpleNamespace

import pytest

from harness import diagnostics
from harness.sim import SimulatorHealthError, wait_for_first_message
from system import test_optitrack_e2e

pytestmark = pytest.mark.unit


def test_diagnostic_bundle_is_bounded_and_secret_free(tmp_path, monkeypatch):
    monkeypatch.setattr(diagnostics.session, "run_dir", lambda: tmp_path)
    monkeypatch.setattr(
        diagnostics.session,
        "recent_cmd_outputs",
        lambda: [{"command": "probe", "output": "x" * 50_000}],
    )

    def fake_run(args, **kwargs):
        output = "container-a\n" if args[:2] == ["docker", "ps"] else "y" * 50_000
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    monkeypatch.setattr(diagnostics.subprocess, "run", fake_run)
    path = diagnostics.collect_failure_diagnostics(
        {
            "COMPOSE_PROFILES": "desktop,isaac-sim",
            "DOCKER_REGISTRY_PASSWORD": "must-not-leak",
        },
        "pane died",
        "system/test",
    )
    payload = json.loads(path.read_text())
    assert payload["schema_version"] == 1
    assert "DOCKER_REGISTRY_PASSWORD" not in payload["effective_config"]
    assert path.stat().st_size < 150_000


def test_message_wait_aborts_immediately_on_dead_process():
    with pytest.raises(SimulatorHealthError, match="pane exited"):
        wait_for_first_message(
            "sim",
            "/clock",
            1,
            "/setup.bash",
            timeout=600,
            health_check=lambda: (False, "pane exited"),
            health_grace=0,
        )


def test_optitrack_missing_sdk_is_explicit_infrastructure(monkeypatch):
    missing = SimpleNamespace(returncode=1, stdout="", stderr="")
    healthy = SimpleNamespace(returncode=0, stdout="", stderr="")
    monkeypatch.setattr(test_optitrack_e2e, "ros2_exec", lambda *a, **k: missing)
    monkeypatch.setattr(test_optitrack_e2e, "docker_exec", lambda *a, **k: healthy)
    monkeypatch.setattr(
        test_optitrack_e2e,
        "collect_failure_diagnostics",
        lambda *a, **k: "diagnostics.json",
    )
    with pytest.raises(pytest.fail.Exception, match="licensed NatNet SDK"):
        test_optitrack_e2e._check_optitrack_prerequisites("robot")
