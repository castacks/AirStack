# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Contract tests for `airstack doctor` (tools/doctor/ — RFC #379 §4).

Doctor's posture is observe-and-report: it edits nothing and, in default
(compose-time) mode, exits non-zero in exactly **two enumerated places**:

1. module dep conflicts that would compose a broken image (RFC #379 §6);
2. control-setpoint / trajectory-group names in any stack's bridge.yaml
   (RFC #380 §2).

Contracts pinned here:

- the compose-time battery is green (exit 0) on this repository;
- each hard-gate path exits 1 against a synthetic sandbox (built in tmp_path
  from tests/fixtures-style synthetic manifests — the developer's checkout is
  never mutated);
- soft findings (broken stack anatomy, invalid manifests) are REPORTED but do
  not gate — exit stays 0;
- the stack-layout check agrees with tests/meta/test_stack_layout_contract.py,
  extended with the split-stack rule: two or more entry points require a
  bridge.yaml;
- the --live safety-floor scan (unit-tested on synthetic graphs, since CI has
  no running stack here) flags unblessed control-setpoint publishers and
  controller impersonators, and stays quiet for the blessed chain.
"""
import importlib.util

import pytest
import yaml

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

REPO = repo_path()

ENV_TEXT = (
    'VERSION="0.19.0"\n'
    'PROJECT_NAME="airstack"\n'
    'PROJECT_DOCKER_REGISTRY="registry.example.com/airstack"\n'
    'DOCKER_IMAGE_BUILD_MODE="dev"\n'
)


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def doctor():
    return _load(REPO / "tools" / "doctor" / "__init__.py", "airstack_doctor")


# ── sandbox builders (tmp_path only; the real checkout is never touched) ────

def make_sandbox(tmp_path):
    root = tmp_path / "airstack"
    root.mkdir()
    (root / ".env").write_text(ENV_TEXT, encoding="utf-8")
    return root


def add_stack(root, name, entries=("stack",), bridge=None, readme=None,
              compat=">=0.19.0 <0.21.0"):
    stack = root / "stacks" / name
    (stack / "launch").mkdir(parents=True)
    (stack / "modules.repos").write_text(
        yaml.safe_dump({"airstack_compat": compat, "repositories": {}}),
        encoding="utf-8",
    )
    for entry in entries:
        (stack / "launch" / f"{entry}.launch.xml").write_text(
            "<launch>\n  <!-- synthetic entry -->\n</launch>\n",
            encoding="utf-8",
        )
    (stack / "docker-compose.yaml").write_text("services: {}\n", encoding="utf-8")
    (stack / "README.md").write_text(
        readme if readme is not None else
        (f"# {name}\n\nSynthetic doctor-contract stack. " + "Purpose text. " * 20),
        encoding="utf-8",
    )
    if bridge is not None:
        (stack / "bridge.yaml").write_text(yaml.safe_dump(bridge), encoding="utf-8")
    return stack


def add_module(root, name, pip=()):
    mdir = root / "modules" / name
    mdir.mkdir(parents=True)
    manifest = {
        "name": name,
        "description": f"Synthetic doctor fixture {name}.",
        "maintainer": "test@example.com",
        "license": "MIT",
        "type": "ros_package",
        "airstack_compat": ">=0.19.0 <0.21.0",
        "targets": ["robot"],
        "deps": {"apt": [], "pip": list(pip)},
        "tests": {"packages": [], "marks": []},
    }
    (mdir / "module.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")
    return mdir


GOOD_BRIDGE = {
    "version": 1,
    "stack": "split_ok",
    "bridge": [
        {"topic": "global_plan", "type": "nav_msgs/msg/Path",
         "direction": "offboard_to_onboard", "qos": "reliable"},
    ],
}

BAD_BRIDGE = {
    "version": 1,
    "stack": "split_bad",
    "bridge": [
        {"topic": "trajectory_controller/trajectory_override",
         "type": "airstack_msgs/msg/TrajectoryXYZVYaw",
         "direction": "offboard_to_onboard", "qos": "reliable"},
    ],
}


# ── green on the repository ──────────────────────────────────────────────────

def test_compose_time_doctor_green_on_repo(doctor, capsys):
    assert doctor.main(["--project-root", str(REPO)]) == 0, (
        "doctor must exit 0 on the repository (only the two hard gates gate):\n"
        + capsys.readouterr().out
    )


def test_repo_hard_gates_individually_clean(doctor):
    assert doctor.check_layer_conflicts(REPO).status == doctor.OK
    gate = doctor.check_bridge_gates(REPO)
    assert gate.status == doctor.OK, gate.messages
    # the shipped split stack was actually inspected, not vacuously skipped
    assert any("lite_offload_global" in m for m in gate.messages)


def test_repo_stack_layout_clean(doctor):
    result = doctor.check_stack_layout(REPO)
    assert result.status == doctor.OK, result.messages


# ── hard gate #2: bridge placement ───────────────────────────────────────────

def test_bridge_hard_gate_exits_one(doctor, tmp_path, capsys):
    root = make_sandbox(tmp_path)
    add_stack(root, "split_bad", entries=("onboard", "offboard"),
              bridge=BAD_BRIDGE)
    assert doctor.main(["--project-root", str(root)]) == 1
    out = capsys.readouterr().out
    assert "trajectory_override" in out, "the gate must NAME the violation"
    assert "hard-gate failure" in out


def test_bridge_hard_gate_check_names_rfcs(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    add_stack(root, "split_bad", entries=("onboard", "offboard"),
              bridge=BAD_BRIDGE)
    result = doctor.check_bridge_gates(root)
    assert result.hard and result.status == doctor.FAIL
    text = " ".join(result.messages)
    assert "RFC #379" in text and "RFC #380" in text


def test_valid_split_stack_passes_gate(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    add_stack(root, "split_ok", entries=("onboard", "offboard"),
              bridge=GOOD_BRIDGE)
    assert doctor.main(["--project-root", str(root)]) == 0
    assert doctor.check_bridge_gates(root).status == doctor.OK


# ── hard gate #1: dep conflicts ──────────────────────────────────────────────

def test_dep_conflict_hard_gate_exits_one(doctor, tmp_path, capsys):
    root = make_sandbox(tmp_path)
    add_module(root, "mod_a", pip=["numpy==1.26.0"])
    add_module(root, "mod_b", pip=["numpy==2.0.0"])
    assert doctor.main(["--project-root", str(root)]) == 1
    out = capsys.readouterr().out
    assert "hard-gate failure" in out
    assert "module-dep-conflicts" in out


def test_same_pin_is_not_a_conflict(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    add_module(root, "mod_a", pip=["numpy==2.0.0"])
    add_module(root, "mod_b", pip=["numpy==2.0.0"])
    assert doctor.check_layer_conflicts(root).status == doctor.OK


# ── soft findings never gate (observe-and-report posture) ────────────────────

def test_split_stack_without_bridge_reports_but_does_not_gate(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    add_stack(root, "split_missing_bridge", entries=("onboard", "offboard"))
    result = doctor.check_stack_layout(root)
    assert result.status == doctor.WARN
    assert any("bridge.yaml" in m for m in result.messages)
    # anatomy problems report; only the two enumerated gates exit non-zero
    assert doctor.main(["--project-root", str(root)]) == 0


def test_unsplit_stack_needs_no_bridge(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    add_stack(root, "plain", entries=("stack",))
    assert doctor.check_stack_layout(root).status == doctor.OK


def test_broken_anatomy_reports_but_does_not_gate(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    stack = add_stack(root, "ragged", readme="too short")
    (stack / "modules.repos").write_text(
        yaml.safe_dump({"repositories": {}}), encoding="utf-8")  # no compat
    result = doctor.check_stack_layout(root)
    assert result.status == doctor.WARN
    text = " ".join(result.messages)
    assert "airstack_compat" in text and "README.md" in text
    assert doctor.main(["--project-root", str(root)]) == 0


def test_invalid_manifest_reports_but_does_not_gate(doctor, tmp_path):
    root = make_sandbox(tmp_path)
    (root / "modules" / "broken").mkdir(parents=True)
    (root / "modules" / "broken" / "module.yaml").write_text(
        "name: broken\n", encoding="utf-8")  # missing required fields
    result = doctor.check_module_manifests(root)
    assert result.status == doctor.WARN
    assert any("INVALID" in m for m in result.messages)
    assert doctor.main(["--project-root", str(root)]) == 0


# ── consistency with the stack-layout contract test ──────────────────────────

def test_layout_check_matches_layout_contract_semantics(doctor, tmp_path):
    """Same anatomy rules as tests/meta/test_stack_layout_contract.py: the
    dispatcher-include ban, the trailer requirement when wiring.md exists."""
    root = make_sandbox(tmp_path)
    stack = add_stack(root, "recursive")
    (stack / "launch" / "stack.launch.xml").write_text(
        '<launch>\n  <include file="$(find-pkg-share autonomy_bringup)'
        '/launch/robot.launch.xml" />\n</launch>\n', encoding="utf-8")
    (stack / "wiring.md").write_text("# hand-written, no trailer\n",
                                     encoding="utf-8")
    result = doctor.check_stack_layout(root)
    assert result.status == doctor.WARN
    text = " ".join(result.messages)
    assert "robot.launch.xml" in text, "dispatcher include must be reported"
    assert "trailer" in text, "hand-edited wiring.md must be reported"


# ── safety-floor scan (unit-level; --live capture needs a running stack) ─────

def _graph(edges):
    return {"version": 1, "nodes": sorted({e["node"] for e in edges}),
            "topics": {}, "edges": edges}


def _pub(node, topic):
    return {"node": node, "topic": topic, "dir": "pub",
            "type": "x/msg/Y", "qos_profile": {}}


def test_safety_floor_quiet_for_blessed_chain(doctor):
    graph = _graph([
        _pub("/robot_1/control/pid_controller",
             "/robot_1/interface/cmd_roll_pitch_yawrate_thrust"),
        _pub("/robot_1/interface/odom_modifier", "/robot_1/interface/cmd_pose"),
        _pub("/robot_1/trajectory_controller/trajectory_control_node",
             "/robot_1/trajectory_controller/tracking_point"),
        _pub("/robot_1/trajectory_controller/trajectory_control_node",
             "/robot_1/trajectory_controller/look_ahead"),
    ])
    result = doctor.check_safety_floor(graph)
    assert result.status == doctor.OK, result.messages


def test_safety_floor_flags_unblessed_setpoint_publisher(doctor):
    graph = _graph([
        _pub("/robot_1/my_rogue_rl_policy",
             "/robot_1/interface/cmd_roll_pitch_yawrate_thrust"),
    ])
    result = doctor.check_safety_floor(graph)
    assert result.status == doctor.WARN
    assert any("my_rogue_rl_policy" in m and "UNBLESSED" in m
               for m in result.messages)


def test_safety_floor_flags_controller_impersonator(doctor):
    graph = _graph([
        _pub("/robot_1/fake_controller", "/robot_1/trajectory_controller/tracking_point"),
    ])
    result = doctor.check_safety_floor(graph)
    assert result.status == doctor.WARN
    assert any("impersonating" in m for m in result.messages)


def test_safety_floor_lists_command_authority_without_flagging(doctor):
    """trajectory_override publishers inherit the safety apparatus — they are
    the command-authority map (informational), never violations."""
    graph = _graph([
        _pub("/robot_1/takeoff_landing_planner/takeoff_landing_task",
             "/robot_1/trajectory_controller/trajectory_override"),
        _pub("/robot_1/droan/planner",
             "/robot_1/trajectory_controller/trajectory_segment_to_add"),
    ])
    result = doctor.check_safety_floor(graph)
    assert result.status == doctor.OK
    assert any("command-authority map" in m for m in result.messages)


# ── stack inference ──────────────────────────────────────────────────────────

def test_infer_stack_prefers_explicit_then_env(doctor, monkeypatch):
    monkeypatch.setenv("AIRSTACK_STACK_DIR", "/root/AirStack/stacks/lite_default")
    assert doctor.infer_stack("explicit") == "explicit"
    assert doctor.infer_stack(None) == "lite_default"
    monkeypatch.delenv("AIRSTACK_STACK_DIR")
    assert doctor.infer_stack(None) is None
