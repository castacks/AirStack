# Copyright 2026 AirLab CMU
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 sensor stream checks (sim + robot) for system tests.

Used by ``test_sensors.py``. Liveliness (``test_liveliness.py``) stays limited to
containers, tmux, and sentinel nodes; sensor Hz / LiDAR validation lives here.

**Isaac Sim (`env["sim"] == "isaacsim"`)** — Pegasus / OmniGraph ROS bridges are
easily overwhelmed when many ``ros2 topic hz`` clients run at once. This module
therefore caps concurrent Hz clients (see ``ISAACSIM_HZ_CHUNK_SIZE``):

- **Sim container:** ``/clock`` alone, then ``image_rect`` topics in chunks, then
  ``depth_ground_truth`` in chunks (each chunk at most two probes per ``docker exec``).
- **Robot container:** same chunking for image then depth. ms-airsim still uses one
  parallel batch of all stereo/depth topics.

**Filtered LiDAR** (``PointCloud2``) — ``ros2 topic hz`` often never prints a rate
on large clouds; this suite uses ``parallel_echo_once_robot_topics`` instead.

See ``tests/README.md`` § *Isaac Sim and the sensors mark* for full documentation.
"""

from __future__ import annotations

import time

from conftest import (
    ROS_DISTRO_SETUP,
    docker_exec,
    get_robot_containers,
    logger,
    parallel_echo_once_robot_topics,
    parallel_sample_hz,
    ros2_exec,
)

# Stereo + depth under /robot_{N}/sensors/front_stereo/... — sampled on the **sim**
# container (source) and on the **robot** container (bridge / DDS path).
STEREO_DEPTH_TOPIC_TEMPLATES = [
    "/robot_{N}/sensors/front_stereo/left/image_rect",
    "/robot_{N}/sensors/front_stereo/right/image_rect",
    "/robot_{N}/sensors/front_stereo/left/depth_ground_truth",
    "/robot_{N}/sensors/front_stereo/right/depth_ground_truth",
]

# Filtered LiDAR (Isaac / Pegasus + lidar_point_cloud_filter). ``ros2 topic hz`` stalls
# on large PointCloud2; liveliness uses ``echo --once`` per robot container.
ROBOT_LIDAR_TOPIC_TEMPLATES = [
    "/robot_{N}/sensors/ouster/point_cloud",
]

LIDAR_CLOUD_VALIDATE_SCRIPT = (
    "/root/AirStack/robot/ros_ws/src/sensors/lidar_point_cloud_filter/scripts/"
    "validate_lidar_filter_clouds.py"
)

# Shorter Hz sample during sensor stability polling.
STABLE_HZ_DURATION_S = 5
STABLE_HZ_WINDOW = 5

# Isaac Sim ROS bridge: max concurrent ``ros2 topic hz`` per ``parallel_sample_hz`` exec.
ISAACSIM_HZ_CHUNK_SIZE = 2


def _parallel_sample_hz_isaacsim_chunks(
    container,
    pairs,
    *,
    setup_bash,
    duration,
    window,
    log_label,
):
    """Run ``parallel_sample_hz`` in fixed-size chunks to limit bridge load.

    Each ``docker exec`` runs at most ``ISAACSIM_HZ_CHUNK_SIZE`` concurrent
    ``ros2 topic hz`` processes, regardless of how many ``pairs`` span robots.
    """
    rates = {}
    if not pairs:
        return rates
    step = ISAACSIM_HZ_CHUNK_SIZE
    nchunks = (len(pairs) + step - 1) // step
    for i in range(0, len(pairs), step):
        chunk = pairs[i : i + step]
        ci = i // step + 1
        logger.info(
            "%s chunk %d/%d — %d topic(s)",
            log_label,
            ci,
            nchunks,
            len(chunk),
        )
        rates.update(
            parallel_sample_hz(
                container,
                chunk,
                setup_bash=setup_bash,
                duration=duration,
                window=window,
            )
        )
    return rates


def sim_side_topics(num_robots):
    """Return (topic, domain_id) tuples for all sim-side topics at a given robot count.

    ``/clock`` is included once on domain 1. ``parallel_sample_hz`` keys results by
    topic string only, so a separate ``/clock`` sample per robot domain would reuse
    one dict entry and overwrite per-probe temp file paths.
    """
    topics = []
    if num_robots > 0:
        topics.append(("/clock", 1))
    for n in range(1, num_robots + 1):
        for tmpl in STEREO_DEPTH_TOPIC_TEMPLATES:
            topics.append((tmpl.format(N=n), n))
    return topics


def check_sim_publishing(env, *, duration=10, window=5):
    """Hz sample of sim-side topics (/clock + stereo/depth). Returns (ok, msg, rates).

    Isaac Sim: ``/clock`` alone; stereo image and depth topic lists are chunked
    (``_parallel_sample_hz_isaacsim_chunks``) so multi-robot runs still cap
    concurrent ``ros2 topic hz`` clients per exec at ``ISAACSIM_HZ_CHUNK_SIZE``.
    """
    cfg = env["cfg"]
    topics = sim_side_topics(env["num_robots"])
    logger.info("Sampling Hz for %d sim-side topics (duration=%ss)", len(topics), duration)

    if env.get("sim") == "isaacsim":
        clock_pairs = [p for p in topics if p[0] == "/clock"]
        stereo_pairs = [p for p in topics if p[0] != "/clock"]
        image_pairs = [p for p in stereo_pairs if "image_rect" in p[0]]
        depth_pairs = [p for p in stereo_pairs if "image_rect" not in p[0]]
        rates = {}
        if clock_pairs:
            logger.info("Isaac sim Hz: batch 1 — %d /clock topic(s)", len(clock_pairs))
            rates.update(
                parallel_sample_hz(
                    env["sim_container"],
                    clock_pairs,
                    setup_bash=cfg["sim_setup_bash"],
                    duration=duration,
                    window=window,
                )
            )
        if image_pairs:
            rates.update(
                _parallel_sample_hz_isaacsim_chunks(
                    env["sim_container"],
                    image_pairs,
                    setup_bash=cfg["sim_setup_bash"],
                    duration=duration,
                    window=window,
                    log_label="Isaac sim Hz: stereo image",
                )
            )
        if depth_pairs:
            rates.update(
                _parallel_sample_hz_isaacsim_chunks(
                    env["sim_container"],
                    depth_pairs,
                    setup_bash=cfg["sim_setup_bash"],
                    duration=duration,
                    window=window,
                    log_label="Isaac sim Hz: stereo depth",
                )
            )
    else:
        rates = parallel_sample_hz(
            env["sim_container"],
            topics,
            setup_bash=cfg["sim_setup_bash"],
            duration=duration,
            window=window,
        )
    stalled = [t for t, hz in rates.items() if hz is None or hz == 0.0]
    if stalled:
        logger.warning("Stalled topics: %s", stalled)
        return False, f"stalled topics: {stalled}", rates
    logger.info("%d topics healthy", len(rates))
    return True, f"{len(rates)} topics healthy", rates


def robot_stereo_depth_hz_pairs(num_robots):
    """Same topic names as sim-side stereo/depth; observed on the robot DDS graph."""
    return [
        (tmpl.format(N=n), n)
        for n in range(1, num_robots + 1)
        for tmpl in STEREO_DEPTH_TOPIC_TEMPLATES
    ]


def check_robot_stereo_hz(env, *, duration=10, window=5):
    """Hz sample stereo + depth on each robot domain (validates sim→robot path).

    Isaac Sim: ``image_pairs`` / ``depth_pairs`` list every robot's topics, but
    ``_parallel_sample_hz_isaacsim_chunks`` runs ``parallel_sample_hz`` repeatedly
    with at most ``ISAACSIM_HZ_CHUNK_SIZE`` topics per exec (default 2), so
    multi-robot runs do not spawn all robots' Hz clients in one batch.
    ms-airsim uses a single parallel batch of all pairs on ``robots[0]``.
    """
    cfg = env["cfg"]
    robots = get_robot_containers(env["robot_pattern"])
    if not robots:
        return False, "no robot containers for robot stereo hz", {}
    pairs = robot_stereo_depth_hz_pairs(env["num_robots"])
    logger.info(
        "Sampling Hz for %d robot-side stereo/depth topics (duration=%ss)",
        len(pairs),
        duration,
    )

    if env.get("sim") == "isaacsim":
        image_pairs = [p for p in pairs if "image_rect" in p[0]]
        depth_pairs = [p for p in pairs if "image_rect" not in p[0]]
        rates = {}
        if image_pairs:
            rates.update(
                _parallel_sample_hz_isaacsim_chunks(
                    robots[0],
                    image_pairs,
                    setup_bash=cfg["robot_setup_bash"],
                    duration=duration,
                    window=window,
                    log_label="Isaac robot Hz: stereo image",
                )
            )
        if depth_pairs:
            rates.update(
                _parallel_sample_hz_isaacsim_chunks(
                    robots[0],
                    depth_pairs,
                    setup_bash=cfg["robot_setup_bash"],
                    duration=duration,
                    window=window,
                    log_label="Isaac robot Hz: stereo depth",
                )
            )
    else:
        rates = parallel_sample_hz(
            robots[0],
            pairs,
            setup_bash=cfg["robot_setup_bash"],
            duration=duration,
            window=window,
        )
    stalled = [t for t, hz in rates.items() if hz is None or hz == 0.0]
    if stalled:
        logger.warning("Stalled robot stereo/depth topics: %s", stalled)
        return False, f"stalled robot stereo/depth topics: {stalled}", rates
    logger.info("%d robot stereo/depth topics healthy", len(rates))
    return True, f"{len(rates)} robot stereo/depth topics healthy", rates


def robot_lidar_echo_probes(env):
    """(container, topic, ros_domain_id) for each robot's filtered ouster cloud."""
    robots = get_robot_containers(env["robot_pattern"])
    probes = []
    for n in range(1, env["num_robots"] + 1):
        if n - 1 >= len(robots):
            break
        for tmpl in ROBOT_LIDAR_TOPIC_TEMPLATES:
            probes.append((robots[n - 1], tmpl.format(N=n), n))
    return probes


def check_robot_filtered_lidar(env, *, duration=10, window=5):
    """Filtered LiDAR is publishing (Isaac only): ``echo --once`` per robot container."""
    if env.get("sim") != "isaacsim":
        return True, "robot lidar skipped (not isaacsim)", {}
    cfg = env["cfg"]
    robots = get_robot_containers(env["robot_pattern"])
    if not robots:
        return False, "no robot containers for lidar liveliness", {}
    if len(robots) < env["num_robots"]:
        return (
            False,
            f"lidar liveliness: need {env['num_robots']} robot container(s), have {len(robots)}",
            {},
        )
    probes = robot_lidar_echo_probes(env)
    per_topic_timeout = max(40, int(duration * 4))
    logger.info(
        "Filtered LiDAR: echo-once for %d topic(s) (isaacsim, timeout=%ss per topic)",
        len(probes),
        per_topic_timeout,
    )
    rates = parallel_echo_once_robot_topics(
        probes,
        setup_bash=cfg["robot_setup_bash"],
        per_topic_timeout=per_topic_timeout,
    )
    stalled = [t for t, hz in rates.items() if hz is None or hz == 0.0]
    if stalled:
        logger.warning("No message on robot lidar topics: %s", stalled)
        return False, f"no message on robot lidar topics: {stalled}", rates
    logger.info("%d robot lidar topic(s) received a message", len(rates))
    return True, f"{len(rates)} robot lidar topic(s) alive", rates


def check_lidar_filtered_cloud_sanity(env):
    """One-shot filtered/raw cloud checks inside each robot container (isaacsim)."""
    if env.get("sim") != "isaacsim":
        return True, "lidar cloud sanity skipped (not isaacsim)", {}
    cfg = env["cfg"]
    robots = get_robot_containers(env["robot_pattern"])
    if len(robots) < env["num_robots"]:
        return (
            False,
            f"lidar sanity: only {len(robots)}/{env['num_robots']} robot containers",
            {},
        )
    inner_base = (
        f"source {ROS_DISTRO_SETUP} && source {cfg['robot_setup_bash']} && "
        f"export ROS_DOMAIN_ID={{domain}} && "
        f"python3 {LIDAR_CLOUD_VALIDATE_SCRIPT} --robot-num {{n}}"
    )
    for n in range(1, env["num_robots"] + 1):
        rc = robots[n - 1]
        inner = inner_base.format(domain=n, n=n)
        logger.info("Lidar cloud sanity robot_%d in %s", n, rc)
        result = docker_exec(rc, inner, timeout=120)
        if result.returncode != 0:
            tail = (result.stdout + result.stderr)[-800:]
            return (
                False,
                f"lidar cloud validation failed robot_{n} rc={result.returncode}: {tail}",
                {},
            )
    return True, f"lidar cloud ok for {env['num_robots']} robot(s)", {}


def read_clock_once(sim_container, setup_bash):
    """Read one /clock message. Returns (sim_t_seconds, wall_t_seconds) or (None, None)."""
    result = ros2_exec(
        sim_container,
        "timeout 5 ros2 topic echo --once /clock",
        domain_id=1,
        setup_bash=setup_bash,
        timeout=10,
    )
    if result.returncode != 0:
        logger.warning(
            "ros2 topic echo /clock failed (rc=%d): stderr=%s",
            result.returncode,
            result.stderr.strip()[:300],
        )
        return None, None
    sec = nsec = None
    for line in result.stdout.splitlines():
        s = line.strip()
        if s.startswith("sec:") and sec is None:
            try:
                sec = int(s.split(":", 1)[1].strip())
            except ValueError:
                pass
        elif s.startswith("nanosec:") and nsec is None:
            try:
                nsec = int(s.split(":", 1)[1].strip())
            except ValueError:
                pass
        if sec is not None and nsec is not None:
            return sec + nsec * 1e-9, time.time()
    logger.warning("could not parse /clock sec/nanosec. stdout head=%r", result.stdout[:300])
    return None, None


def check_realtime_factor(env, sample_interval=20.0):
    """RTF = Δ sim_time / Δ wall_time from /clock. Returns (ok, msg, rtf_or_None)."""
    cfg = env["cfg"]
    sim_container = env["sim_container"]
    setup_bash = cfg["sim_setup_bash"]

    logger.info("RTF: reading initial /clock from %s", sim_container)
    sim_t1, wall_t1 = read_clock_once(sim_container, setup_bash)
    if sim_t1 is None:
        return False, "failed to read initial /clock", None
    logger.info("RTF: initial sim_t=%.3f, sleeping %.1fs", sim_t1, sample_interval)

    time.sleep(sample_interval)

    sim_t2, wall_t2 = read_clock_once(sim_container, setup_bash)
    if sim_t2 is None:
        return False, "failed to read final /clock", None

    wall_delta = wall_t2 - wall_t1
    sim_delta = sim_t2 - sim_t1
    logger.info(
        "RTF: final sim_t=%.3f (sim Δ=%.3fs, wall Δ=%.3fs)",
        sim_t2,
        sim_delta,
        wall_delta,
    )
    if wall_delta <= 0:
        return False, "non-positive wall time delta", None
    rtf = sim_delta / wall_delta
    logger.info("RTF: %.3f", rtf)
    if rtf < 0.1:
        return False, f"RTF={rtf:.3f} (sim near-stalled)", rtf
    return True, f"RTF={rtf:.3f}", rtf
