# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Robot-name-map resolution contract.

Drives ``robot/docker/robot_name_map/resolve_robot_name.py`` as a subprocess
(exactly how ``robot/docker/.bashrc`` invokes it) against the committed
``default_robot_name_map.yaml`` and asserts the resolved name/domain pairs.

Regression anchor for the double-digit-replica bug: the old rule
``.*robot-.*(\\d+)`` let the greedy ``.*`` swallow leading digits, so
``airstack-robot-desktop-12`` resolved to robot_2/domain 2 — a silent DDS
collision with the real robot 2. The fixed rule ``.*robot-\\D*(\\d+)`` captures
the whole trailing number.

Also checks identity parity with the fleet resolver
(``tools/fleet/resolve_fleet.py``): for a homogeneous fleet whose robots are
robot_1..robot_N in file order, a ``airstack-robot-desktop-<i>`` container
must land on the same robot index in both resolvers.
"""
import subprocess
import sys

import pytest

from harness.discovery import TESTS_DIR

pytestmark = pytest.mark.unit

REPO = TESTS_DIR.parent
RESOLVER = REPO / "robot" / "docker" / "robot_name_map" / "resolve_robot_name.py"
MAP_FILE = (
    REPO / "robot" / "docker" / "robot_name_map" / "default_robot_name_map.yaml"
)


def _resolve(container_name):
    """Run the resolver as .bashrc does; return (exit_code, stdout, stderr)."""
    proc = subprocess.run(
        [sys.executable, str(RESOLVER), container_name, str(MAP_FILE)],
        capture_output=True, text=True, timeout=30,
    )
    return proc.returncode, proc.stdout, proc.stderr


def _parse_exports(stdout):
    exports = {}
    for line in stdout.splitlines():
        key, _, value = line.partition("=")
        exports[key] = value
    return exports


@pytest.mark.parametrize(
    "container_name,robot_name,domain_id",
    [
        ("airstack-robot-desktop-1", "robot_1", "1"),
        ("airstack-robot-desktop-3", "robot_3", "3"),
        # double digits: the greedy legacy rule resolved these to the LAST
        # digit only (robot_0 / robot_2 / robot_1) — the collision bug
        ("airstack-robot-desktop-10", "robot_10", "10"),
        ("airstack-robot-desktop-12", "robot_12", "12"),
        ("airstack-robot-desktop-21", "robot_21", "21"),
        # bare hostname form (ROBOT_NAME_SOURCE=hostname)
        ("robot-21", "robot_21", "21"),
    ],
)
def test_container_names_resolve_to_full_index(container_name, robot_name,
                                               domain_id):
    code, stdout, stderr = _resolve(container_name)
    assert code == 0, f"resolver failed for {container_name}: {stderr}"
    exports = _parse_exports(stdout)
    assert exports.get("ROBOT_NAME") == robot_name, (
        f"{container_name}: expected ROBOT_NAME={robot_name}, "
        f"got {exports.get('ROBOT_NAME')!r} (stdout: {stdout!r})"
    )
    assert exports.get("ROS_DOMAIN_ID") == domain_id, (
        f"{container_name}: expected ROS_DOMAIN_ID={domain_id}, "
        f"got {exports.get('ROS_DOMAIN_ID')!r}"
    )


def test_unmatched_name_falls_through_to_catch_all():
    code, stdout, stderr = _resolve("some-unrelated-host")
    assert code == 0, f"catch-all rule should match anything: {stderr}"
    exports = _parse_exports(stdout)
    assert exports.get("ROBOT_NAME") == "unknown_robot"
    assert exports.get("ROS_DOMAIN_ID") == "0"


def test_fleet_resolver_identity_parity():
    """The fleet resolver's trailing-index identity rule must agree with the
    legacy map for the homogeneous case (robot N in file order = robot_N)."""
    fleet_dir = REPO / "tools" / "fleet"
    sys.path.insert(0, str(fleet_dir))
    try:
        import resolve_fleet
    except Exception as exc:  # pragma: no cover - environment-dependent
        pytest.skip(f"tools/fleet/resolve_fleet.py not importable: {exc}")
    finally:
        sys.path.remove(str(fleet_dir))

    fleet = {"robots": {f"robot_{i}": {} for i in range(1, 25)}}
    for index in (1, 3, 10, 12, 21):
        container = f"airstack-robot-desktop-{index}"
        fleet_key = resolve_fleet.resolve_identity(fleet, container)
        code, stdout, _ = _resolve(container)
        assert code == 0
        legacy_name = _parse_exports(stdout)["ROBOT_NAME"]
        assert fleet_key == legacy_name == f"robot_{index}", (
            f"{container}: fleet resolver says {fleet_key!r}, legacy map says "
            f"{legacy_name!r} — the two identity rules must agree"
        )
