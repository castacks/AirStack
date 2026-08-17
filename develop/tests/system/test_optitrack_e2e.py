"""OptiTrack NatNet end-to-end (sim).

A single dedicated bring-up that exercises the whole OptiTrack path in Isaac Sim:
the in-sim NatNet **emulator** streams rigid-body poses → ``natnet_ros2`` publishes
the drone pose → the ``vision_pose`` bridge feeds MAVROS → PX4 EKF2 fuses it.

This brings the NatNet stack up **once** and asserts only one NatNet-specific test. 
The cheap, GPU-free half of this (host emulator → ``natnet_ros2`` Hz) lives in ``tests/integration/natnet/``.

Mark: ``optitrack``. Needs Docker + GPU + Isaac Sim license; skips cleanly when the
isaac-sim image isn't built locally.
"""
import os
import re
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
    ros2_exec,
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
# PX4_PARAM_SET selects simulation/isaac-sim/docker/px4-params/external-vision.env, which
# switches PX4 SITL's EKF2 to mocap external vision and turns GPS, baro and range aiding
# OFF, so the OptiTrack stream is the vehicle's ONLY position source. PX4's rcS applies
# those PX4_PARAM_* entries at boot; they mirror the deployment-validated set in
# robot/ros_ws/src/perception/natnet_ros2/config/px4_params.yaml.
#
# Without it EKF2_EV_CTRL is 0, PX4 silently discards the vision and flies on sim GPS —
# which is what made the previous version of this module's fusion check vacuous.
_E2E_ENV = {
    "NUM_ROBOTS": "1",
    "COMPOSE_PROFILES": "desktop,isaac-sim",
    "AUTOLAUNCH": "true",
    "ISAAC_SIM_USE_STANDALONE": "true",
    "ISAAC_SIM_SCRIPT_NAME": "example_one_px4_pegasus_natnet_launch_script.py",
    "PLAY_SIM_ON_START": "true",
    "LAUNCH_NATNET": "true",
    # EKF2 external-vision (mocap) configuration — see comment above. Selects
    # simulation/isaac-sim/docker/px4-params/external-vision.env, which turns GPS, baro
    # and range aiding off, leaving mocap as the vehicle's only position source.
    "PX4_PARAM_SET": "external-vision",
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
# PX4 fused local position (proves EKF2 accepted the external vision). odom, not pose:
# it goes live only once EKF2 has converged and home is set, which is what PX4's arming
# preflight requires. See test_px4_ready in test_takeoff_hover_land.py — pose-era signals
# fire ~25s earlier, and arming in that window returns "failed to arm".
_PX4_LOCAL_POSE_TOPIC = "interface/mavros/local_position/odom"
# MAVROS param plugin node; hosts the FCU param table as ROS 2 parameters. Same
# service px4_param_setter reads through.
_EV_PARAM_NODE = "interface/mavros/param"
# One proves vision fusion is on, the other proves GPS aiding is off — together they
# are the precondition every test below assumes but none of them check.
_EV_EXPECTED = {"EKF2_EV_CTRL": 11, "EKF2_GPS_CTRL": 0}
# MAVROS pulls the param table lazily after FCU connect; px4_param_setter budgets
# settle_sec 10 + 30 retries for the same reason.
_EV_PARAM_TIMEOUT = 90
# `ros2 param get` prints "Integer value is: 11" / "Double value is: 7.0".
_PARAM_VALUE_RE = re.compile(r"value is:\s*(-?[\d.]+)")
# Cold Isaac boot: Pegasus load + Play + emulator UDP connect.
_FIRST_MSG_TIMEOUT = 180

# Belt-and-braces behind the odom gate: EV-only convergence is slower and less predictable
# than the GPS case the autonomy suites were tuned against, and TakeoffTask does not retry
# its own ARM (takeoff_landing_task.cpp send_robot_command).
_ARM_ATTEMPTS = 5
_ARM_RETRY_S = 2.0
_ARM_SERVICE = "interface/robot_command"
_ARM_COMMAND = 1  # airstack_msgs/srv/RobotCommand.Request.ARM

# The flight helpers imported from test_fixed_trajectory take an `airstack_env`-style cfg;
# `robot_setup_bash` is the only key any of them reads.
_TRAJ_CFG = {"robot_setup_bash": _ROBOT_SETUP_BASH}


def _arm_with_retries(container: str) -> None:
    """Arm the vehicle, retrying while PX4's preflight is still rejecting it.

    Uses the same robot_command service TakeoffTask arms through, so a successful
    call here leaves TakeoffTask's `is_armed_` set and it skips its own arming.
    """
    service = f"/robot_{_ROBOT_DOMAIN}/{_ARM_SERVICE}"
    last = ""
    started = time.time()
    for attempt in range(1, _ARM_ATTEMPTS + 1):
        result = ros2_exec(
            container,
            f'timeout 10 ros2 service call {service} '
            f'airstack_msgs/srv/RobotCommand "{{command: {_ARM_COMMAND}}}"',
            domain_id=_ROBOT_DOMAIN, setup_bash=_ROBOT_SETUP_BASH, timeout=20,
        )
        last = (result.stdout or "") + (result.stderr or "")
        if "success=True" in last.replace(" ", ""):
            logger.info("armed on attempt %d/%d", attempt, _ARM_ATTEMPTS)
            return
        logger.info("arm attempt %d/%d refused (PX4 preflight not ready yet)",
                    attempt, _ARM_ATTEMPTS)
        if attempt < _ARM_ATTEMPTS:
            time.sleep(_ARM_RETRY_S)

    pytest.fail(
        f"could not arm after {_ARM_ATTEMPTS} attempts over "
        f"{time.time() - started:.0f}s — PX4 preflight still rejecting. "
        "With GPS/baro/range aiding off, this means EKF2 has not converged on the "
        f"vision estimate. Last response:\n{last.strip()[-400:]}"
    )


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

    @pytest.mark.dependency(name="ev_params", depends=["natnet_pose"])
    def test_ev_params_applied(self, optitrack_sim_stack):
        """The external-vision param set actually reached the FCU.

        Everything below assumes PX4_PARAM_SET=external-vision took effect. If it
        silently did not, EKF2_EV_CTRL stays 0 and EKF2_GPS_CTRL stays 7, the vehicle
        flies the Circle on sim GPS, and every other test here still passes. This reads
        the live values back off the FCU, so it covers the whole chain: compose env_file
        -> container env -> Pegasus -> PX4 rcS -> FCU.
        """
        container = _robot_container(optitrack_sim_stack)
        node = f"/robot_{_ROBOT_DOMAIN}/{_EV_PARAM_NODE}"

        unread = dict(_EV_EXPECTED)
        actual = {}
        deadline = time.time() + _EV_PARAM_TIMEOUT
        while unread and time.time() < deadline:
            for name in list(unread):
                result = ros2_exec(
                    container, f"ros2 param get {node} {name}",
                    domain_id=_ROBOT_DOMAIN, setup_bash=_ROBOT_SETUP_BASH, timeout=20,
                )
                # An unpulled param prints "Parameter not set." and still exits 0, so
                # match on the value line rather than the return code.
                match = _PARAM_VALUE_RE.search(result.stdout or "")
                if match:
                    actual[name] = float(match.group(1))
                    del unread[name]
            if unread:
                time.sleep(2.0)

        assert not unread, (
            f"{', '.join(sorted(unread))} never appeared in the MAVROS param table "
            f"within {_EV_PARAM_TIMEOUT}s — the MAVROS/FCU link is down, which is a "
            "different failure from a wrong parameter."
        )
        wrong = {k: v for k, v in sorted(actual.items()) if v != _EV_EXPECTED[k]}
        assert not wrong, (
            f"PX4 is not configured for external vision: {wrong} (expected "
            f"{ {k: _EV_EXPECTED[k] for k in wrong} }). PX4_PARAM_SET=external-vision "
            "did not reach the FCU, so the flight below would fly on GPS and pass anyway."
        )
        logger.info("EV params confirmed on FCU: %s", actual)

    @pytest.mark.dependency(name="ev_ready", depends=["ev_params"])
    def test_px4_fuses_vision(self, optitrack_sim_stack):
        """PX4 publishes local_position/odom, so EKF2 has converged and home is set.

        This only establishes that a converged estimate EXISTS — it is deliberately not
        the proof that vision is being fused, because odom publishes off any aiding
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
            f"no PX4 local_position/odom on {topic} within {_FIRST_MSG_TIMEOUT}s — "
            "EKF2 never converged or never set a home position. With GPS/baro/range "
            "aiding off, that means the external-vision path never reached it."
        )

    @pytest.mark.dependency(name="ev_takeoff", depends=["ev_ready"])
    @pytest.mark.timeout(2400)
    def test_takeoff(self, optitrack_sim_stack):
        """Take off to TARGET_ALTITUDE_M flying on the mocap-fused estimate."""
        container = _robot_container(optitrack_sim_stack)
        _arm_with_retries(container)
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
