# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Contract tests for split-stack bridge.yaml + tools/gen_dds_router.py
(RFC #380 §2, RFC #379 §4).

A split stack's ``bridge.yaml`` is the explicit list of every topic/service/
action crossing the machine boundary; ``gen_dds_router.py`` derives the
DDS-router config from it. Contracts pinned here:

- **Schema** — the shipped ``lite_offload_global/bridge.yaml`` validates; the
  usual authoring mistakes (absolute names, wrong direction/qos enums, two
  kinds in one entry, kind/type mismatch) are named errors.
- **Hard gate #2** (RFC #379 §4 / #380 §2) — a bridge listing
  ``trajectory_override`` (or any control-setpoint / trajectory-group /
  ``trajectory_controller/*`` name) fails ``--check`` with exit 1, naming the
  entry and citing the RFCs. ``global_plan`` crosses; trajectory commands
  don't.
- **Determinism** — generation is a pure function of bridge.yaml: identical
  inputs produce byte-identical router configs, with no timestamps and no
  absolute paths.
- **Router format** — output parses as YAML and follows the shared
  ``autonomy_bringup/config/dds_router.yaml`` conventions (inherited from the
  removed legacy split's ``onboard_local_offboard_global`` router config):
  ``$(env ROBOT_NAME)`` interpolation, rt/ topics, rq/rr service pairs, the
  five action sub-endpoints, participants on ``$(env ROS_DOMAIN_ID)`` /
  ``$(var gcs_domain)``.
"""
import copy
import importlib.util
import json

import pytest
import yaml

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

REPO = repo_path()
BRIDGE_PATH = REPO / "stacks" / "lite_offload_global" / "bridge.yaml"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def gen():
    return _load(REPO / "tools" / "gen_dds_router.py", "airstack_gen_dds_router")


@pytest.fixture(scope="module")
def real_bridge():
    return yaml.safe_load(BRIDGE_PATH.read_text(encoding="utf-8"))


def _minimal_bridge(*entries):
    return {
        "version": 1,
        "stack": "synthetic_split",
        "bridge": list(entries),
    }


GOOD_TOPIC = {
    "topic": "global_plan",
    "type": "nav_msgs/msg/Path",
    "direction": "offboard_to_onboard",
    "qos": "reliable",
}


# ── schema ───────────────────────────────────────────────────────────────────

def test_real_bridge_is_schema_valid(gen, real_bridge):
    errors = gen.validate_bridge(real_bridge)
    assert errors == [], (
        "lite_offload_global/bridge.yaml must validate:\n"
        + json.dumps(errors, indent=2)
    )


def test_real_bridge_check_mode_exits_zero(gen, capsys):
    assert gen.main([str(BRIDGE_PATH), "--check"]) == 0
    verdict = json.loads(capsys.readouterr().out)
    assert verdict == {"valid": True, "errors": []}


def test_real_bridge_carries_the_split_essentials(real_bridge):
    """The split's raison d'être is readable in the bridge list itself."""
    names = {
        next(k for k in ("topic", "service", "action") if k in e): e[
            next(k for k in ("topic", "service", "action") if k in e)]
        for e in real_bridge["bridge"]
    }
    topics = {e["topic"] for e in real_bridge["bridge"] if "topic" in e}
    assert "global_plan" in topics, "global_plan must cross (offboard planner)"
    assert "odometry_conversion/odometry" in topics
    assert "sensors/ouster/point_cloud" in topics, "vdb_mapping's input"
    assert names  # sanity


@pytest.mark.parametrize("mutate, expect_fragment", [
    (lambda e: e.update(topic="/robot_1/global_plan"), "relative"),
    (lambda e: e.update(direction="up"), "not one of"),
    (lambda e: e.update(qos="mostly_reliable"), "not one of"),
    (lambda e: e.update(type="nav_msgs/Path"), "interface type"),
    (lambda e: e.update(type="nav_msgs/srv/Path"), "does not match the entry kind"),
    (lambda e: e.update(service="also_a_service"), "exactly one of"),
    (lambda e: e.update(extra_field=True), "unknown fields"),
    (lambda e: e.pop("qos"), "required for topics"),
])
def test_schema_rejects_common_mistakes(gen, mutate, expect_fragment):
    entry = copy.deepcopy(GOOD_TOPIC)
    mutate(entry)
    errors = gen.validate_bridge(_minimal_bridge(entry))
    assert errors, f"expected an error after mutating to {entry}"
    assert any(expect_fragment in e["message"] for e in errors), (
        f"no error mentioned {expect_fragment!r}:\n" + json.dumps(errors, indent=2)
    )


# ── hard gate #2 ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize("name", [
    "trajectory_controller/trajectory_override",
    "trajectory_override",                       # bare basename still gated
    "trajectory_controller/trajectory_segment_to_add",
    "trajectory_controller/set_trajectory_mode",
    "trajectory_controller/tracking_point",
    "trajectory_controller/look_ahead",
    "trajectory_controller/trajectory_vis",      # the whole group stays off
    "control_setpoint",
    "interface/cmd_roll_pitch_yawrate_thrust",   # concrete control setpoint
])
def test_hard_gate_rejects_control_and_trajectory_names(gen, name):
    entry = {
        "topic": name,
        "type": "airstack_msgs/msg/TrajectoryXYZVYaw",
        "direction": "offboard_to_onboard",
        "qos": "reliable",
    }
    errors = gen.validate_bridge(_minimal_bridge(entry))
    assert errors, f"{name} must be rejected by the placement hard gate"
    messages = " ".join(e["message"] for e in errors)
    assert name in messages, "the violation must NAME the offending entry"
    assert "RFC #379" in messages and "RFC #380" in messages, (
        "the violation must cite RFC #379 §4 / RFC #380 §2"
    )


def test_hard_gate_gates_services_too(gen):
    entry = {
        "service": "trajectory_controller/set_trajectory_mode",
        "type": "airstack_msgs/srv/TrajectoryMode",
        "direction": "offboard_to_onboard",
    }
    errors = gen.validate_bridge(_minimal_bridge(entry))
    assert any("set_trajectory_mode" in e["message"] for e in errors)


def test_check_mode_exits_one_on_synthetic_trajectory_bridge(gen, tmp_path, capsys):
    bad = _minimal_bridge(GOOD_TOPIC, {
        "topic": "trajectory_controller/trajectory_override",
        "type": "airstack_msgs/msg/TrajectoryXYZVYaw",
        "direction": "offboard_to_onboard",
        "qos": "reliable",
    })
    path = tmp_path / "bridge.yaml"
    path.write_text(yaml.safe_dump(bad), encoding="utf-8")
    assert gen.main([str(path), "--check"]) == 1
    verdict = json.loads(capsys.readouterr().out)
    assert verdict["valid"] is False
    assert any("trajectory_override" in e["message"] for e in verdict["errors"])


def test_generation_refuses_invalid_bridge(gen, tmp_path):
    bad = _minimal_bridge({
        "topic": "trajectory_controller/trajectory_override",
        "type": "airstack_msgs/msg/TrajectoryXYZVYaw",
        "direction": "offboard_to_onboard",
        "qos": "reliable",
    })
    path = tmp_path / "bridge.yaml"
    path.write_text(yaml.safe_dump(bad), encoding="utf-8")
    out = tmp_path / "router.yaml"
    assert gen.main([str(path), "--out", str(out)]) == 1
    assert not out.exists(), "no router config may be generated past the gate"


def test_allowed_names_pass_the_gate(gen):
    """The neighboring interchanges the gate must NOT catch."""
    for name, type_name in [
        ("global_plan", "nav_msgs/msg/Path"),
        ("odometry_conversion/odometry", "nav_msgs/msg/Odometry"),
        ("takeoff_landing_planner/trajectory_completion_percentage",
         "std_msgs/msg/Float32"),
    ]:
        entry = {"topic": name, "type": type_name,
                 "direction": "onboard_to_offboard", "qos": "reliable"}
        assert gen.validate_bridge(_minimal_bridge(entry)) == [], name


# ── determinism + router format ──────────────────────────────────────────────

def test_generation_is_deterministic(gen, tmp_path):
    out_a = tmp_path / "a.yaml"
    out_b = tmp_path / "b.yaml"
    assert gen.main([str(BRIDGE_PATH), "--out", str(out_a)]) == 0
    assert gen.main([str(BRIDGE_PATH), "--out", str(out_b)]) == 0
    text_a = out_a.read_text(encoding="utf-8")
    assert text_a == out_b.read_text(encoding="utf-8")
    assert str(REPO) not in text_a, "no absolute paths in generated output"
    assert str(tmp_path) not in text_a


def test_generated_config_matches_legacy_router_format(gen, tmp_path, real_bridge):
    out = tmp_path / "router.yaml"
    assert gen.main([str(BRIDGE_PATH), "--out", str(out)]) == 0
    text = out.read_text(encoding="utf-8")
    data = yaml.safe_load(text)

    # participants follow the legacy interpolation conventions
    assert data["participants"][0]["domain"] == "$(env ROS_DOMAIN_ID)"
    assert data["participants"][1]["domain"] == "$(var gcs_domain)"

    allow = [e["name"] for e in data["allowlist"]]
    # every topic entry becomes exactly one rt/ name in robot namespace
    assert "rt/$(env ROBOT_NAME)/global_plan" in allow
    assert "rt/$(env ROBOT_NAME)/sensors/ouster/point_cloud" in allow
    # services expand to the rq/rr pair
    assert "rq/$(env ROBOT_NAME)/interface/robot_commandRequest" in allow
    assert "rr/$(env ROBOT_NAME)/interface/robot_commandReply" in allow
    # actions expand to the five DDS sub-endpoints (8 names)
    nav = [n for n in allow if "tasks/navigate/_action" in n]
    assert len(nav) == 8, nav
    assert "rt/$(env ROBOT_NAME)/tasks/navigate/_action/feedback" in nav
    assert "rq/$(env ROBOT_NAME)/tasks/navigate/_action/send_goalRequest" in nav

    # the gate's guarantee holds in the OUTPUT too — belt and braces
    forbidden = [n for n in allow if "trajectory_controller" in n
                 or n.rsplit("/", 1)[-1] in gen.FORBIDDEN_BASENAMES]
    assert forbidden == [], forbidden

    # entry count bookkeeping: topics=1, services=2, actions=8 endpoints each
    kinds = {"topic": 0, "service": 0, "action": 0}
    for entry in real_bridge["bridge"]:
        kinds[next(k for k in kinds if k in entry)] += 1
    expected = kinds["topic"] + 2 * kinds["service"] + 8 * kinds["action"]
    assert len(allow) == expected


def test_default_out_path_uses_stack_name(gen):
    path = gen.default_out_path(BRIDGE_PATH, REPO)
    assert path == REPO / ".airstack" / "generated" / "dds_router.lite_offload_global.yaml"
