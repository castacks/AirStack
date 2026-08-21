"""Shared sim-topic test infrastructure: sim target configs + ros2 topic sampling.

Used by the liveliness / sensors / takeoff system tests to probe topic liveness and
publish rates on the robot/sim containers.
"""
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

from harness.commands import ROS_DISTRO_SETUP, _parse_hz, _run_teed, ros2_exec
from harness.session import logger

SIM_CONFIG = {
    "msairsim": {
        "profile": "ms-airsim",
        "sim_container": "ms-airsim",
        "sim_setup_bash": "/root/ros_ws/install/setup.bash",
        "robot_setup_bash": "/root/AirStack/robot/ros_ws/install/setup.bash",
        "extra_env": {
            "URDF_FILE": "robot_descriptions/iris/urdf/iris_stereo.ms-airsim.urdf",
            # Clear any user-set paths in .env so entrypoint auto-fetches Blocks.
            # Shell env wins over --env-file in docker compose substitution.
            "MS_AIRSIM_ENV_DIR": "",
            "MS_AIRSIM_BINARY_PATH": "",
        },
    },
    "isaacsim": {
        "profile": "isaac-sim",
        "sim_container": "isaac-sim",
        "sim_setup_bash": "/opt/ros/jazzy/setup.bash",
        "robot_setup_bash": "/root/AirStack/robot/ros_ws/install/setup.bash",
        "extra_env": {
            "ISAAC_SIM_USE_STANDALONE": "true",
            "ISAAC_SIM_SCRIPT_NAME": "example_multi_px4_pegasus_launch_script.py",
            "PLAY_SIM_ON_START": "true",
            # Multi script gates RTX LiDAR on this flag; example_one always spawns it.
            # `sensors` tests expect ouster topics + lidar_point_cloud_filter path.
            "ENABLE_LIDAR": "true",
        },
    },
}


class SimulatorHealthError(RuntimeError):
    """A readiness wait stopped because its simulator process became unhealthy."""


def wait_for_first_message(
    container,
    topic,
    domain_id,
    setup_bash,
    timeout=60,
    health_check=None,
    health_grace=15,
):
    """Wait up to `timeout` seconds for one message on `topic`. Returns seconds
    elapsed on success, None on timeout. Each attempt sources the workspace
    and runs `ros2 topic echo --once`; if the workspace isn't built yet or the
    topic has no publisher, the attempt fails fast and we retry.
    """
    start = time.time()
    deadline = start + timeout
    logger.info("Probing %s on domain %d in %s (timeout=%ds)",
                topic, domain_id, container, timeout)
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        if health_check is not None and time.time() - start >= health_grace:
            health = health_check()
            if isinstance(health, tuple):
                healthy, detail = health
            else:
                healthy, detail = bool(health), "simulator health probe failed"
            if not healthy:
                raise SimulatorHealthError(
                    f"infrastructure simulator process failure while waiting "
                    f"for {topic}: {detail}"
                )
        per_attempt = min(max(1, int(deadline - time.time())), 10)
        try:
            result = ros2_exec(
                container,
                f"timeout {per_attempt} ros2 topic echo --once {topic}",
                domain_id=domain_id, setup_bash=setup_bash, timeout=per_attempt + 5,
            )
        except subprocess.TimeoutExpired:
            logger.warning("Attempt %d subprocess timeout for %s, retrying", attempt, topic)
            time.sleep(2)
            continue
        # ros2 prints "---" on its own line after a real message.
        if result.stdout.rstrip().endswith("---"):
            elapsed = round(time.time() - start, 2)
            logger.info("Got first message on %s after %.2fs (attempt %d)",
                        topic, elapsed, attempt)
            return elapsed
        logger.warning("Attempt %d failed for %s, retrying", attempt, topic)
        time.sleep(2)
    logger.error("Timed out waiting for first message on %s after %ds",
                 topic, timeout)
    return None


def sample_hz(container, topic, domain_id, setup_bash, duration=5, window=10):
    """Sample publish rate on `topic` for `duration` seconds. Returns float or None."""
    result = ros2_exec(
        container,
        f"timeout {duration} ros2 topic hz --window {window} {topic} 2>&1",
        domain_id=domain_id, setup_bash=setup_bash, timeout=duration + 15,
    )
    return _parse_hz(result.stdout + result.stderr)


def parallel_sample_hz(container, topic_domain_pairs, setup_bash, duration=5, window=10):
    """Sample Hz for multiple topics concurrently; return {topic: hz_or_None}.

    One `docker exec` that backgrounds each `ros2 topic hz` probe, waits for all,
    then cats each probe's temp file.
    """
    probes = []
    temp_files = {}
    for i, (topic, domain) in enumerate(topic_domain_pairs):
        fname = f"/tmp/hz_{i}.out"
        temp_files[topic] = fname
        probes.append(
            f"(ROS_DOMAIN_ID={domain} timeout {duration} "
            f"ros2 topic hz --window {window} {topic} > {fname} 2>&1) &"
        )
    # Newlines, not `&& ... &`: bash precedence makes `A && B && C & D &` only
    # apply the && chain to C, so later backgrounded probes would miss the
    # sourced PATH. One statement per line sidesteps this entirely.
    lines = [f"source {ROS_DISTRO_SETUP}", f"source {setup_bash}"] + probes + ["wait"]
    for fname in temp_files.values():
        lines.append(f"echo '===FILE {fname}==='")
        lines.append(f"cat {fname} 2>/dev/null || true")
    script = "\n".join(lines)
    result = _run_teed(
        ["docker", "exec", container, "bash", "-c", script],
        timeout=duration + 30,
    )
    rates = {}
    if result.returncode == 0 or result.stdout:
        chunks = result.stdout.split("===FILE ")
        for chunk in chunks[1:]:
            header, _, content = chunk.partition("===")
            fname = header.strip()
            topic = next((t for t, f in temp_files.items() if f == fname), None)
            if topic:
                rates[topic] = _parse_hz(content)
    for topic, _ in topic_domain_pairs:
        rates.setdefault(topic, None)
    return rates


def _echo_once_received_message(result):
    """True if ``ros2 topic echo --once`` printed a full message (trailing ``---``)."""
    out = (result.stdout or "").rstrip()
    return out.endswith("---")


def parallel_echo_once_robot_topics(
    probes, setup_bash, per_topic_timeout,
):
    """Liveliness for heavy topics (e.g. PointCloud2): ``echo --once`` per probe in parallel.

    ``ros2 topic hz`` often never reports a rate on large point clouds (decode backlog).

    Parameters
    ----------
    probes : list[tuple[str, str, int]]
        ``(container_name, topic, ros_domain_id)`` — use the **robot container** that
        hosts that domain's graph (replica ``n`` for ``robot_n``).
    setup_bash : str
        Workspace ``setup.bash`` path inside the container.
    per_topic_timeout : int
        Wall seconds per ``timeout … ros2 topic echo --once``.

    Returns
    -------
    dict[str, float | None]
        ``{topic: 1.0}`` if a message arrived, else ``{topic: None}`` (metrics use 1.0
        as a nonzero "alive" placeholder, not a measured Hz).
    """
    rates = {}

    def _one(container, topic, domain_id):
        cmd = f"timeout {per_topic_timeout} ros2 topic echo --once {topic}"
        return topic, ros2_exec(
            container,
            cmd,
            domain_id=domain_id,
            setup_bash=setup_bash,
            timeout=per_topic_timeout + 15,
        )

    with ThreadPoolExecutor(max_workers=max(1, len(probes))) as pool:
        futures = {
            pool.submit(_one, container, topic, domain_id): topic
            for container, topic, domain_id in probes
        }
        for fut in as_completed(futures):
            topic = futures[fut]
            try:
                _, result = fut.result()
            except Exception as e:
                logger.warning("echo-once probe failed for %s: %s", topic, e)
                rates[topic] = None
                continue
            rates[topic] = 1.0 if _echo_once_received_message(result) else None
    for _, topic, _ in probes:
        rates.setdefault(topic, None)
    return rates
