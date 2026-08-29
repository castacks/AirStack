"""Simple-sim smoke test — mark ``simple_sim``.

Brings the stack up with ``COMPOSE_PROFILES=simple`` (``SIM_CONFIG["simplesim"]``:
the simple-robot service replaces robot-desktop, no GCS) and gates only on what
simple-sim actually provides:

- containers: ``simple-sim`` + ``simple-robot`` Running
- sim ready: ``/clock`` published by the sim node (domain 1; also proves the
  sim workspace's at-startup colcon build finished)
- the sim's mock-MAVROS odometry visible from the robot container
  (cross-container DDS on the fixed SIM_IP network)
- sentinel ROS 2 nodes: ``robot_state_publisher`` + ``trajectory_control_node``.
  MAVROS is deliberately NOT a sentinel — ``interface.launch.py`` skips it when
  ``SIM_TYPE=simple`` (the sim node mocks the MAVROS surface itself).

simple-sim is single-robot: its topics/services are hardcoded to ``robot_1`` on
``ROS_DOMAIN_ID=1``. Collection guards in ``harness.collection`` skip this module
unless ``--sim simplesim`` (and skip other sim campaigns' tests under it).

Run: ``airstack test -m simple_sim --sim simplesim --num-robots 1 -v``
"""
import time

import pytest

from conftest import (
    container_running,
    current_test_id,
    get_metrics,
    get_robot_containers,
    logger,
    ros2_exec,
    wait_for_first_message,
)

# What actually runs per robot under SIM_TYPE=simple (full_default stack minus
# MAVROS). Kept deliberately small and honest.
SENTINEL_NODE_TEMPLATES = [
    "/robot_{N}/robot_state_publisher",
    "/robot_{N}/trajectory_controller/trajectory_control_node",
]

# Published by the sim node (MavrosMockNode) in place of real MAVROS —
# simulation/simple-sim/ros_ws/src/sim/src/sim_node.cpp.
MOCK_ODOM_TOPIC = "/robot_1/interface/mavros/local_position/odom"


def _poll_until(predicate, timeout, interval, fail_msg):
    """Sleep-poll ``predicate`` up to ``timeout`` seconds."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(interval)
    pytest.fail(fail_msg() if callable(fail_msg) else fail_msg)


@pytest.mark.simple_sim
@pytest.mark.timeout(1200)
class TestSimpleSimSmoke:

    @pytest.mark.dependency(name="ss_containers")
    def test_containers_running(self, airstack_env):
        """simple-robot and simple-sim containers Running within 120s."""
        pattern = airstack_env["robot_pattern"]
        sim_container = airstack_env["sim_container"]

        def ready():
            robots = get_robot_containers(pattern)
            return (
                len(robots) >= airstack_env["num_robots"]
                and all(container_running(c) for c in robots)
                and container_running(sim_container)
            )

        _poll_until(
            ready,
            timeout=120,
            interval=3,
            fail_msg=lambda: (
                f"simple-sim stack not Running after 120s: robots="
                f"{get_robot_containers(pattern)} sim_running="
                f"{container_running(sim_container)}"
            ),
        )

    @pytest.mark.dependency(name="ss_clock", depends=["ss_containers"])
    def test_sim_publishes_clock(self, airstack_env):
        """The sim node publishes /clock on domain 1 (600s hard timeout —
        the sim workspace colcon-builds at container start)."""
        cfg = airstack_env["cfg"]
        m = get_metrics()
        tid = current_test_id()
        start = airstack_env["up_started_at"]

        if (
            wait_for_first_message(
                airstack_env["sim_container"],
                "/clock",
                domain_id=1,
                setup_bash=cfg["sim_setup_bash"],
                timeout=600,
            )
            is None
        ):
            m.record(tid, "sim_ready_duration_s", "timeout", unit="s")
            pytest.fail("simple-sim never published /clock within 600s")
        m.record(tid, "sim_ready_duration_s", round(time.time() - start, 2), unit="s")

    @pytest.mark.dependency(name="ss_odom", depends=["ss_clock"])
    def test_mock_mavros_odometry_reaches_robot(self, airstack_env):
        """The sim's mock-MAVROS odometry is visible from the robot container."""
        cfg = airstack_env["cfg"]
        robots = get_robot_containers(airstack_env["robot_pattern"])
        assert robots, "no simple-robot container found"
        elapsed = wait_for_first_message(
            robots[0],
            MOCK_ODOM_TOPIC,
            domain_id=1,
            setup_bash=cfg["robot_setup_bash"],
            timeout=180,
        )
        assert elapsed is not None, (
            f"{MOCK_ODOM_TOPIC} never reached {robots[0]} within 180s"
        )

    @pytest.mark.dependency(depends=["ss_containers"])
    def test_sentinel_nodes_present(self, airstack_env):
        """robot_state_publisher + trajectory_control_node up within 300s
        (MAVROS intentionally absent under SIM_TYPE=simple)."""
        cfg = airstack_env["cfg"]
        expected = {t.format(N=1) for t in SENTINEL_NODE_TEMPLATES}
        last_missing = [expected]

        def ready():
            robots = get_robot_containers(airstack_env["robot_pattern"])
            if not robots:
                return False
            result = ros2_exec(
                robots[0],
                "ros2 node list 2>/dev/null",
                domain_id=1,
                setup_bash=cfg["robot_setup_bash"],
                timeout=20,
            )
            if result.returncode != 0:
                return False
            missing = expected - set(result.stdout.splitlines())
            last_missing[0] = missing
            return not missing

        _poll_until(
            ready,
            timeout=300,
            interval=5,
            fail_msg=lambda: f"sentinel nodes missing after 300s: {sorted(last_missing[0])}",
        )
        logger.info("All %d simple-sim sentinel nodes present", len(expected))
