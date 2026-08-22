# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""NatNet - robot autonomy integration tests.

Host-side variants stream frames to ``natnet_ros2_node`` in the robot container and
assert pose topics stay alive at >= 5 Hz: (1) raw ``NatNetUnicastServer`` hand-built
single-body frames; (2) ``NatNetServerManager`` sampling an in-memory USD stage
(Isaac wrapper path, no sim/GPU); (3) a multi-body profile (drone + target) that
exercises per-body topic overrides and the pose / pose_cov toggles.

The node is parameterised with the flattened per-body arrays
(``body_names`` / ``body_ids`` / ``body_topics`` / ``body_pose`` / ``body_pose_cov``)
that natnet_ros2.launch.py derives from a robot's natnet_config.yaml profile.

Multi-robot (NUM_ROBOTS=3, per-robot profiles) is exercised in-sim by
``tests/system/test_liveliness.py::test_natnet_pose_alive``.
"""

from __future__ import annotations

import subprocess
import sys
import threading
import time

import pytest

from conftest import (  # noqa: E402 — pytest adds tests/ to sys.path
    docker_exec,
    repo_path,
    ros2_env,
    sample_hz,
    wait_for_first_message,
)

# Emulator is not pip-installed on the host; add extension root + test helpers.
_EXT_ROOT = repo_path("simulation/isaac-sim/extensions/optitrack.natnet.emulator")
for _path in (_EXT_ROOT, _EXT_ROOT / "test"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType  # noqa: E402
from optitrack.natnet.emulator.server import natnet_data_types as dt  # noqa: E402
from natnet_test_helpers import ephemeral_udp_port  # noqa: E402

pytestmark = pytest.mark.integration

_ROBOT_SETUP = "/root/AirStack/robot/ros_ws/install/setup.bash"
_NATNET_NODE = "/root/AirStack/robot/ros_ws/install/natnet_ros2/lib/natnet_ros2/natnet_ros2_node"
_WARMUP_S = 2.0
_STREAM_HOLD_S = 12.0
_MIN_HZ = 5.0

# Robot image has route/netstat but not `ip`; /proc/net/route is always present.
_DEFAULT_GATEWAY_CMD = (
    """awk '$2 == "00000000" { printf "%d.%d.%d.%d\\n", """
    """"0x" substr($3,7,2), "0x" substr($3,5,2), "0x" substr($3,3,2), "0x" substr($3,1,2); exit }' """
    """/proc/net/route"""
)


def _docker_default_gateway(container: str) -> str:
    result = docker_exec(container, _DEFAULT_GATEWAY_CMD, timeout=10)
    gateway = result.stdout.strip()
    if not gateway:
        pytest.skip(f"Could not resolve default gateway inside {container}")
    return gateway


def _container_env(container: str, var: str, default: str) -> str:
    # ROBOT_NAME / ROS_DOMAIN_ID are set in .bashrc (login shell), not container ENV.
    # .bashrc may print "Sourcing ..." to stdout; take the last line as the value.
    result = docker_exec(container, f"bash -lc 'echo ${var}'")
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    value = lines[-1] if lines else ""
    return value if value else default


def _natnet_node_available(container: str) -> bool:
    result = docker_exec(container, f"test -x {_NATNET_NODE} && echo yes || echo no")
    return "yes" in result.stdout


def _stop_stale_natnet_nodes(container: str) -> None:
    docker_exec(container, "pkill -f natnet_ros2_node || true")
    time.sleep(0.5)


# Each body: (streaming_id, rigid_body_name). The raw server frame carries ids only;
# the node maps ids → topics via its body_* params.
_DRONE_BODY = (1, "Drone")
_TARGET_BODY = (100, "Target")


def _make_frame(frame_num: int, body_ids) -> dt.sFrameOfMocapData:
    frame = dt.sFrameOfMocapData()
    frame.iFrame = frame_num
    frame.nRigidBodies = len(body_ids)
    for slot, body_id in enumerate(body_ids):
        rb = frame.RigidBodies[slot]
        rb.ID = body_id
        rb.qw = 1.0
        # Bit 0 = tracking valid; natnet_ros2 skips bodies without it (natnet_logic.hpp).
        rb.params = 1
    return frame


def _frame_publisher(
    server: NatNetUnicastServer, stop_event: threading.Event, body_ids=(1,)
) -> None:
    frame_num = 0
    interval = 1.0 / server.publish_rate
    while not stop_event.is_set():
        server.enqueue_mocap_data(_make_frame(frame_num, body_ids))
        frame_num += 1
        time.sleep(interval)


def _launch_natnet_node(container, host_ip, command_port, domain_id, bodies=None):
    """Start natnet_ros2_node in the container pointed at the host emulator.

    ``bodies`` is a list of (id, name, topic, pose, pose_cov); defaults to a single
    Drone body on topic ``perception/optitrack/drone`` (the shipped config default).
    """
    if bodies is None:
        bodies = [(1, "Drone", "perception/optitrack/drone", "true", "true")]
    ids = ",".join(str(b[0]) for b in bodies)
    names = ",".join(b[1] for b in bodies)
    topics = ",".join(b[2] for b in bodies)
    pose = ",".join(b[3] for b in bodies)
    pose_cov = ",".join(b[4] for b in bodies)
    launch_cmd = (
        f"bash -lc '{ros2_env(_ROBOT_SETUP, domain_id)} && "
        f"exec {_NATNET_NODE} --ros-args "
        f"-p server_ip:={host_ip} "
        f"-p command_port:={command_port} "
        f"-p body_names:=[{names}] "
        f"-p body_ids:=[{ids}] "
        f"-p body_topics:=[{topics}] "
        f"-p body_pose:=[{pose}] "
        f"-p body_pose_cov:=[{pose_cov}]'"
    )
    return subprocess.Popen(
        ["docker", "exec", container, "bash", "-c", launch_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _assert_pose_stream(
    container, robot_name, domain_id, topic="perception/optitrack/drone", pose_cov=True
):
    """Wait for the pose topic then assert a sustained rate >= _MIN_HZ.

    A body configured with ``body_pose_cov=false`` never publishes the ``/pose_cov``
    variant, so detect the first message on whichever topic the body actually emits.
    """
    pose_topic = f"/{robot_name}/{topic}"
    detect_topic = f"{pose_topic}/pose_cov" if pose_cov else pose_topic

    time.sleep(_WARMUP_S)
    first_msg_s = wait_for_first_message(
        container, detect_topic, domain_id, _ROBOT_SETUP, timeout=int(_STREAM_HOLD_S)
    )
    assert first_msg_s is not None, (
        f"No messages on {detect_topic} within {_STREAM_HOLD_S}s "
        "(NatNet connect or frame stream failed)"
    )
    hz = sample_hz(
        container,
        pose_topic,
        domain_id,
        _ROBOT_SETUP,
        duration=min(8, int(_STREAM_HOLD_S - first_msg_s)),
        window=20,
    )
    assert hz is not None, f"No sustained stream on {pose_topic}"
    assert hz >= _MIN_HZ, f"Expected >= {_MIN_HZ} Hz on {pose_topic}, got {hz}"


def _terminate(proc) -> None:
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def test_natnet_ros2_receives_drone_pose_hz(robot_autonomy_stack):
    """Raw-server path: hand-built frames on NatNetUnicastServer."""
    container = robot_autonomy_stack["container"]

    if not _natnet_node_available(container):
        pytest.skip(
            "natnet_ros2_node not built — run airstack setup (NatNet SDK) and "
            "bws --packages-select natnet_ros2 in the robot container"
        )

    _stop_stale_natnet_nodes(container)

    host_ip = _docker_default_gateway(container)
    command_port = ephemeral_udp_port(host_ip)
    robot_name = _container_env(container, "ROBOT_NAME", "robot_1")
    domain_id = int(_container_env(container, "ROS_DOMAIN_ID", "0"))

    server = NatNetUnicastServer(
        local_interface=host_ip,
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
        command_port=command_port,
    )
    server.publish_rate = 50

    stop_event = threading.Event()
    publisher = threading.Thread(
        target=_frame_publisher, args=(server, stop_event), daemon=True
    )

    node_proc: subprocess.Popen[str] | None = None
    try:
        # Seed dummy frames before the client connects; keep streaming the whole window.
        publisher.start()
        time.sleep(0.1)
        server.start()
        node_proc = _launch_natnet_node(container, host_ip, command_port, domain_id)
        _assert_pose_stream(container, robot_name, domain_id)
    finally:
        stop_event.set()
        publisher.join(timeout=2.0)
        _terminate(node_proc)
        server.shutdown()


def test_natnet_ros2_receives_isaac_wrapper_pose_hz(robot_autonomy_stack):
    """Isaac-wrapper path: NatNetServerManager.sample_once on a moving USD prim.

    Tests that the wrapper feeds the real robot client end-to-end. Pose-value fidelity
    is covered by test_pose_streaming.py loopback.
    """
    pytest.importorskip("pxr")
    import math

    from pxr import Gf, Usd, UsdGeom

    from optitrack.natnet.emulator.isaac import (
        BodyBinding,
        NatNetInterfaceConfig,
        NatNetServerManager,
        author_interface,
    )

    container = robot_autonomy_stack["container"]
    if not _natnet_node_available(container):
        pytest.skip("natnet_ros2_node not built — run airstack setup (NatNet SDK)")

    _stop_stale_natnet_nodes(container)

    host_ip = _docker_default_gateway(container)
    command_port = ephemeral_udp_port(host_ip)
    data_port = ephemeral_udp_port(host_ip)
    while data_port == command_port:
        data_port = ephemeral_udp_port(host_ip)
    robot_name = _container_env(container, "ROBOT_NAME", "robot_1")
    domain_id = int(_container_env(container, "ROS_DOMAIN_ID", "0"))

    stage = Usd.Stage.CreateInMemory()
    xform = UsdGeom.Xform.Define(stage, "/World/base_link")
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 1.0))
    cfg = NatNetInterfaceConfig(
        server_ip=host_ip,
        command_port=command_port,
        data_port=data_port,
        publish_rate=50.0,
        bodies=[BodyBinding("Drone", "/World/base_link", streaming_id=1)],
    )
    author_interface(stage, "/World/NatNetInterface", cfg)

    manager = NatNetServerManager(server_factory=None)  # real server factory
    stop_event = threading.Event()

    def _sampler():
        # Stand in for the in-sim physics-step callback: move the prim and sample.
        interval = 1.0 / cfg.publish_rate
        t = 0.0
        while not stop_event.is_set():
            translate_op.Set(Gf.Vec3d(math.sin(t), 0.0, 1.0))
            manager.sample_once(stage)
            t += interval
            time.sleep(interval)

    sampler = threading.Thread(target=_sampler, daemon=True)

    node_proc: subprocess.Popen[str] | None = None
    try:
        assert manager.start_server(cfg) is True
        sampler.start()
        time.sleep(0.1)
        node_proc = _launch_natnet_node(container, host_ip, command_port, domain_id)
        _assert_pose_stream(container, robot_name, domain_id)
    finally:
        stop_event.set()
        sampler.join(timeout=2.0)
        _terminate(node_proc)
        manager.stop_server()


def test_natnet_ros2_multi_body_drone_and_target(robot_autonomy_stack):
    """Multi-body profile: one robot tracks a drone + a static target.

    Streams two bodies (drone id 1, target id 100) and configures the node like a
    robot profile with two bodies and distinct relative topics. Asserts: the drone
    pose streams >= 5 Hz on its custom topic; the target pose streams on its own
    topic; and the target's pose_cov topic is absent (body_pose_cov=false).
    """
    container = robot_autonomy_stack["container"]

    if not _natnet_node_available(container):
        pytest.skip("natnet_ros2_node not built — run airstack setup (NatNet SDK)")

    _stop_stale_natnet_nodes(container)

    host_ip = _docker_default_gateway(container)
    command_port = ephemeral_udp_port(host_ip)
    robot_name = _container_env(container, "ROBOT_NAME", "robot_1")
    domain_id = int(_container_env(container, "ROS_DOMAIN_ID", "0"))

    server = NatNetUnicastServer(
        local_interface=host_ip,
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
        command_port=command_port,
    )
    server.publish_rate = 50

    bodies = [
        (_DRONE_BODY[0], _DRONE_BODY[1], "perception/optitrack/drone", "true", "true"),
        (_TARGET_BODY[0], _TARGET_BODY[1], "perception/optitrack/target", "true", "false"),
    ]
    body_ids = (_DRONE_BODY[0], _TARGET_BODY[0])

    stop_event = threading.Event()
    publisher = threading.Thread(
        target=_frame_publisher, args=(server, stop_event, body_ids), daemon=True
    )

    node_proc: subprocess.Popen[str] | None = None
    try:
        publisher.start()
        time.sleep(0.1)
        server.start()
        node_proc = _launch_natnet_node(container, host_ip, command_port, domain_id, bodies)
        # Drone (pose + pose_cov) and target (pose only) both stream.
        _assert_pose_stream(container, robot_name, domain_id, "perception/optitrack/drone")
        _assert_pose_stream(
            container, robot_name, domain_id, "perception/optitrack/target", pose_cov=False
        )

        # body_pose_cov=false → the target pose_cov publisher must not exist.
        target_cov = f"/{robot_name}/perception/optitrack/target/pose_cov"
        topics = docker_exec(
            container,
            f"bash -lc '{ros2_env(_ROBOT_SETUP, domain_id)} && ros2 topic list'",
            timeout=15,
        ).stdout
        assert target_cov not in topics.split(), (
            f"{target_cov} should not exist when body_pose_cov=false; topics:\n{topics}"
        )
    finally:
        stop_event.set()
        publisher.join(timeout=2.0)
        _terminate(node_proc)
        server.shutdown()
