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
from system.test_fixed_trajectory import (
    TARGET_ALTITUDE_M,
    _landing_one_robot,
    _run_parallel,
    _takeoff_one_robot,
    _trajectory_one_robot,
)

pytestmark = pytest.mark.optitrack

# Single-drone NatNet Isaac stack: the natnet Pegasus script spawns the emulator
# alongside PX4, and LAUNCH_NATNET=true brings up natnet_ros2 + the vision_pose /
# gp_origin / param bridges on the robot.
#
# The PX4_EV_* entries switch PX4 SITL's EKF2 to mocap external vision and turn GPS,
# baro and range aiding OFF, so the OptiTrack stream is the vehicle's ONLY position
# source. They map to PX4_PARAM_* env vars in simulation/isaac-sim/docker/docker-compose.yaml
# (PX4's rcS applies those at boot) and mirror the deployment-validated set in
# robot/ros_ws/src/perception/natnet_ros2/config/px4_params.yaml.
#
# Without them EKF2_EV_CTRL is 0, PX4 silently discards the vision and flies on sim GPS —
# which is what made the previous version of this module's fusion check vacuous.
_E2E_ENV = {
    "NUM_ROBOTS": "1",
    "COMPOSE_PROFILES": "desktop,isaac-sim",
    "AUTOLAUNCH": "true",
    "ISAAC_SIM_USE_STANDALONE": "true",
    "ISAAC_SIM_SCRIPT_NAME": "example_one_px4_pegasus_natnet_launch_script.py",
    "PLAY_SIM_ON_START": "true",
    "LAUNCH_NATNET": "true",
    # EKF2 external-vision (mocap) configuration — see comment above.
    "PX4_EV_CTRL": "11",          # fuse vision horizontal pos + vertical pos + yaw
    "PX4_EV_HGT_REF": "3",        # vision is the height reference
    "PX4_EV_GPS_CTRL": "0",       # no GPS fusion
    "PX4_EV_MAG_TYPE": "5",       # mag off; yaw comes from vision
    "PX4_EV_BARO_CTRL": "0",      # no baro fusion
    "PX4_EV_SYS_HAS_BARO": "0",   # remove baro at the system level (height datum)
    "PX4_EV_RNG_CTRL": "0",       # no range-finder aiding
    "PX4_EV_DELAY": "7.0",
    "PX4_EV_NOISE_MD": "1",
    "PX4_EV_EVP_NOISE": "0.05",
    "PX4_EV_EVA_NOISE": "0.05",
    # Headless: no X on the CI runner.
    "QT_QPA_PLATFORM": "offscreen",
}

# The trajectory flown to prove fusion. Circle is the enforced PR gate: sustained lateral
# motion is where a wrong EV delay or a too-tight innovation gate actually shows up, which
# a stationary hover would never reveal.
_E2E_TRAJECTORY = "Circle"

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

# The flight helpers imported from test_fixed_trajectory take an `airstack_env`-style cfg;
# `robot_setup_bash` is the only key any of them reads.
_TRAJ_CFG = {"robot_setup_bash": _ROBOT_SETUP_BASH}


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

    @pytest.mark.dependency(name="ev_ready", depends=["natnet_pose"])
    def test_px4_fuses_vision(self, optitrack_sim_stack):
        """PX4 publishes a fused local position, so it is ready to be commanded.

        This only establishes that an estimate EXISTS — it is deliberately not the proof
        that vision is being fused, because local_position/pose publishes off any aiding
        source. The flight below is the proof: with GPS, baro and range aiding disabled in
        _E2E_ENV, mocap is the only thing that can produce this estimate at all.
        """
        container = _robot_container(optitrack_sim_stack)
        topic = f"/robot_{_ROBOT_DOMAIN}/{_PX4_LOCAL_POSE_TOPIC}"

        first = wait_for_first_message(
            container, topic, domain_id=_ROBOT_DOMAIN,
            setup_bash=_ROBOT_SETUP_BASH, timeout=_FIRST_MSG_TIMEOUT,
        )
        assert first is not None, (
            f"no PX4 local_position on {topic} within {_FIRST_MSG_TIMEOUT}s — "
            "EKF2 has no valid position estimate. With GPS/baro/range aiding off, that "
            "means the external-vision path never reached it."
        )

    @pytest.mark.dependency(name="ev_takeoff", depends=["ev_ready"])
    @pytest.mark.timeout(2400)
    def test_takeoff(self, optitrack_sim_stack):
        """Take off to TARGET_ALTITUDE_M flying on the mocap-fused estimate."""
        container = _robot_container(optitrack_sim_stack)
        _run_parallel(1, lambda n: _takeoff_one_robot(
            n, container, _TRAJ_CFG, TARGET_ALTITUDE_M))

    @pytest.mark.dependency(name="ev_circle", depends=["ev_takeoff"])
    @pytest.mark.timeout(2400)
    def test_circle_trajectory(self, optitrack_sim_stack):
        """Fly a Circle with mocap as the only position source.

        This is the end-to-end proof: emulator → natnet_ros2 → vision_pose → MAVROS →
        EKF2 → controller → airframe. Cross-track error is scored by the same code the
        autonomy benchmark uses, so a mocap regression shows up as path deviation rather
        than as a topic that merely exists.
        """
        container = _robot_container(optitrack_sim_stack)
        _run_parallel(1, lambda n: _trajectory_one_robot(
            n, container, _TRAJ_CFG, _E2E_TRAJECTORY))

    @pytest.mark.dependency(name="ev_land", depends=["ev_takeoff"])
    @pytest.mark.timeout(2400)
    def test_landing(self, optitrack_sim_stack):
        """Land the drone; runs even when the trajectory phase fails."""
        container = _robot_container(optitrack_sim_stack)
        _run_parallel(1, lambda n: _landing_one_robot(n, container, _TRAJ_CFG))
