"""Static safety contract for the manually dispatched WS2 smoke workflow."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_ws2_smoke_workflow_contract():
    path = ROOT / ".github/workflows/ws2-smoke.yml"
    workflow = yaml.load(path.read_text(encoding="utf-8"), Loader=yaml.BaseLoader)
    assert set(workflow["on"]) == {"workflow_call", "workflow_dispatch"}
    assert workflow["permissions"] == {"contents": "read"}

    smoke = workflow["jobs"]["smoke"]
    assert smoke["runs-on"] == ["self-hosted", "airstack-ephemeral"]
    assert smoke["concurrency"]["cancel-in-progress"] == "false"
    steps = {step["name"]: step for step in smoke["steps"]}
    assert steps["Stop smoke-owned containers"]["if"] == "always()"
    assert steps["Verify expected smoke result"]["if"] == "always()"
    assert steps["Upload smoke evidence"]["if"] == "always()"
    assert steps["Enforce smoke conclusion"]["if"] == "always()"


def test_existing_dispatch_entry_point_can_call_ws2_smoke():
    path = ROOT / ".github/workflows/system-tests.yml"
    workflow = yaml.load(path.read_text(encoding="utf-8"), Loader=yaml.BaseLoader)
    inputs = workflow["on"]["workflow_dispatch"]["inputs"]
    assert inputs["ws2_smoke"]["type"] == "boolean"
    job = workflow["jobs"]["ws2-smoke"]
    assert job["uses"] == "./.github/workflows/ws2-smoke.yml"
    assert job["secrets"] == "inherit"


def test_ws2_smoke_pins_companion_revisions():
    text = (ROOT / ".github/workflows/ws2-smoke.yml").read_text(encoding="utf-8")
    assert 'workspace_root="$(dirname "$GITHUB_WORKSPACE")"' in text
    assert 'dirname "$(dirname "$GITHUB_WORKSPACE")"' not in text
    assert "checkout cc5ec88" in text
    assert "checkout 395485b" in text
    assert "validate_timeout.yaml" in text
