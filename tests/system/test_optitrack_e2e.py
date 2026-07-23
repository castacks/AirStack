"""OptiTrack NatNet end-to-end (sim).

A single dedicated bring-up that exercises the whole OptiTrack path in Isaac Sim:
the in-sim NatNet **emulator** streams rigid-body poses → ``natnet_ros2`` publishes
the drone pose → the ``vision_pose`` bridge feeds MAVROS → PX4 EKF2 fuses it.

This intentionally does NOT add ``isaacsim_natnet`` as a third parametrized sim in
the ``airstack_env`` matrix (that would re-run the entire liveliness/sensors/flight
suite under NatNet for two assertions). Instead we bring the NatNet stack up **once**
here and assert only the NatNet-specific chain. The cheap, GPU-free half of this
(host emulator → ``natnet_ros2`` Hz) lives in ``tests/integration/natnet/``.

Mark: ``optitrack``. Needs Docker + GPU + Isaac Sim license; skips cleanly when the
isaac-sim image isn't built locally.
"""
import os
import time

import pytest

from conftest import (  # noqa: E402 — pytest adds tests/ to sys.path
    airstack_cmd,
    container_running,
    find_container,
    get_metrics,
    get_robot_containers,
    logger,
    missing_images,
    read_log_tail,
    sample_hz,
    wait_for_container,
    wait_for_first_message,
)

pytestmark = pytest.mark.optitrack

# Single-drone NatNet Isaac stack: the natnet Pegasus script spawns the emulator
# alongside PX4, and LAUNCH_NATNET=true brings up natnet_ros2 + the vision_pose /
# gp_origin / param bridges on the robot.
_E2E_ENV = {
    "NUM_ROBOTS": "1",
    "COMPOSE_PROFILES": "desktop,isaac-sim",
    "AUTOLAUNCH": "true",
    "ISAAC_SIM_USE_STANDALONE": "true",
    "ISAAC_SIM_SCRIPT_NAME": "example_one_px4_pegasus_natnet_launch_script.py",
    "PLAY_SIM_ON_START": "true",
    "LAUNCH_NATNET": "true",
    # Headless: no X on the CI runner.
    "QT_QPA_PLATFORM": "offscreen",
}

_ROBOT_PATTERN = "robot.*desktop"
_ROBOT_SETUP_BASH = "/root/AirStack/robot/ros_ws/install/setup.bash"
_ROBOT_DOMAIN = 1
# Drone body's relative pose topic from natnet_config.yaml, namespaced per robot.
_NATNET_POSE_TOPIC = os.environ.get("NATNET_POSE_TOPIC", "perception/optitrack/drone")
_NATNET_MIN_HZ = 5.0
# PX4 fused local position (proves EKF2 accepted the external vision).
_PX4_LOCAL_POSE_TOPIC = "interface/mavros/local_position/pose"
# Cold Isaac boot: Pegasus load + Play + emulator UDP connect.
_FIRST_MSG_TIMEOUT = 180


@pytest.fixture(scope="module")
def optitrack_sim_stack(request):
    """Bring the NatNet Isaac stack up once for the module; tear it down after.

    Reuses an already-running robot-desktop container (fast local iteration);
    otherwise brings the stack up. Skips when the isaac-sim image isn't built.
    """
    existing = find_container(_ROBOT_PATTERN)
    if existing and container_running(existing):
        yield {"container": existing, "brought_up": False}
        return

    missing = missing_images(env=_E2E_ENV)
    if missing:
        pytest.skip("isaac-sim / robot image not built locally: " + ", ".join(missing))

    airstack_cmd("down", timeout=120, log_name="optitrack_e2e")
    result = airstack_cmd("up", env_overrides=_E2E_ENV, timeout=300, log_name="optitrack_e2e")
    if result.returncode != 0:
        pytest.fail(f"`airstack up` (natnet isaac) failed:\n{read_log_tail('optitrack_e2e')}")

    container = wait_for_container(_ROBOT_PATTERN, timeout=180)
    assert container, "robot-desktop container not Running after 180s"
    try:
        yield {"container": container, "brought_up": True}
    finally:
        airstack_cmd("down", timeout=120, log_name="optitrack_e2e")


def _robot_container(stack):
    # robot_1 lives on the first (index-1) replica.
    return get_robot_containers(_ROBOT_PATTERN)[0] if not stack["brought_up"] \
        else wait_for_container(_ROBOT_PATTERN, timeout=60)


class TestOptitrackE2E:

    @pytest.mark.dependency(name="natnet_pose")
    def test_natnet_pose_alive(self, optitrack_sim_stack):
        """Emulator → natnet_ros2 → vision_pose: the drone pose_cov streams >= 5 Hz."""
        container = _robot_container(optitrack_sim_stack)
        topic = f"/robot_{_ROBOT_DOMAIN}/{_NATNET_POSE_TOPIC}/pose_cov"

        first = wait_for_first_message(
            container, topic, domain_id=_ROBOT_DOMAIN,
            setup_bash=_ROBOT_SETUP_BASH, timeout=_FIRST_MSG_TIMEOUT,
        )
        assert first is not None, (
            f"no NatNet pose on {topic} within {_FIRST_MSG_TIMEOUT}s "
            "(emulator → natnet_ros2 path down)"
        )
        hz = sample_hz(container, topic, domain_id=_ROBOT_DOMAIN,
                       setup_bash=_ROBOT_SETUP_BASH, duration=5, window=20)
        get_metrics().record("test_optitrack_e2e.natnet_pose_hz",
                             "natnet_pose_hz", hz if hz is not None else "none", unit="Hz")
        assert hz is not None and hz >= _NATNET_MIN_HZ, \
            f"{topic} at {hz} Hz (< {_NATNET_MIN_HZ})"

    @pytest.mark.dependency(depends=["natnet_pose"])
    def test_px4_fuses_vision(self, optitrack_sim_stack):
        """PX4 EKF2 fuses the external vision: local_position/pose streams."""
        container = _robot_container(optitrack_sim_stack)
        topic = f"/robot_{_ROBOT_DOMAIN}/{_PX4_LOCAL_POSE_TOPIC}"

        first = wait_for_first_message(
            container, topic, domain_id=_ROBOT_DOMAIN,
            setup_bash=_ROBOT_SETUP_BASH, timeout=_FIRST_MSG_TIMEOUT,
        )
        assert first is not None, (
            f"no PX4 local_position on {topic} within {_FIRST_MSG_TIMEOUT}s "
            "(EKF2 not fusing external vision)"
        )
