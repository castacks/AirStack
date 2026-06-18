"""Tests for the per-task ``cbf_exempt_drones`` list and its decoupling from
the teleop role.

Constructs the real ``SwarmCommander`` with parameter overrides (no launch
file / no running interfaces needed — the node just declares params, builds
the scenario and wiring, and we inspect ``cbf_exempt_names``). Skipped where
rclpy is unavailable (pure-numpy environments).
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter  # noqa: E402

from svg_ground_control.swarm_commander import SwarmCommander  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_commander(**params) -> SwarmCommander:
    """Construct the commander with the given parameter overrides."""
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return SwarmCommander(parameter_overrides=overrides)


def test_exempt_list_parsed_into_set() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        cbf_exempt_drones="drone_1, drone_3",
    )
    try:
        assert node.cbf_exempt_names == {"drone_1", "drone_3"}
    finally:
        node.destroy_node()


def test_auto_drone_can_be_exempt() -> None:
    # A policy-driven (auto) drone — NOT teleop — can be exempt. This is the
    # squeeze intruder / hybrid_squeeze case.
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        teleop_drones="",
        cbf_exempt_drones="drone_3",
    )
    try:
        assert "drone_3" in node.cbf_exempt_names
        d3 = next(d for d in node.drones if d.name == "drone_3")
        assert d3.role == "auto"
    finally:
        node.destroy_node()


def test_teleop_is_not_auto_exempt() -> None:
    # Behavior change: a teleop drone is exempt ONLY if listed. Listed here it
    # is exempt; absent from the list it would be CBF-corrected.
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        teleop_drones="drone_3",
        cbf_exempt_drones="",
    )
    try:
        assert node.cbf_exempt_names == set()
        d3 = next(d for d in node.drones if d.name == "drone_3")
        assert d3.role == "teleop"   # teleop role, but NOT exempt
    finally:
        node.destroy_node()


def test_teleop_exempt_when_listed() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        teleop_drones="drone_3",
        cbf_exempt_drones="drone_3",
    )
    try:
        assert "drone_3" in node.cbf_exempt_names
    finally:
        node.destroy_node()


def test_unknown_exempt_name_rejected() -> None:
    with pytest.raises(ValueError):
        make_commander(
            drone_names=["drone_1", "drone_2"],
            cbf_exempt_drones="drone_9",
        )


def test_external_cannot_be_exempt() -> None:
    # External drones are never commanded, so exempting one is a config error.
    with pytest.raises(ValueError):
        make_commander(
            drone_names=["drone_1", "drone_2"],
            external_drones="drone_2",
            cbf_exempt_drones="drone_2",
        )
