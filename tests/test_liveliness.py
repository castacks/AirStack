"""Liveliness stress tests — parametrized over (sim × num_robots × iteration).

Tests verify the stack is up, tmux processes are alive, sim is producing data,
expected ROS2 nodes exist, and the system remains stable under a poll window.

See plan: /home/oalama/.claude/plans/piped-crafting-meteor-liveliness.md
"""
import time

import pytest

from conftest import (
    container_running,
    current_test_id,
    docker_exec,
    find_all_containers,
    get_metrics,
    get_robot_containers,
    logger,
    parallel_sample_hz,
    ros2_exec,
    wait_for_first_message,
)


# ── liveliness-specific topic list ─────────────────────────────────────────

SENSOR_TOPIC_TEMPLATES = [
    "/robot_{N}/sensors/front_stereo/left/image_rect",
    "/robot_{N}/sensors/front_stereo/right/image_rect",
    "/robot_{N}/sensors/front_stereo/left/depth_ground_truth",
    "/robot_{N}/sensors/front_stereo/right/depth_ground_truth",
]

SENTINEL_NODE_TEMPLATES = [
    "/robot_{N}/interface/mavros/mavros",
    "/robot_{N}/robot_state_publisher",
    "/robot_{N}/trajectory_controller/trajectory_control_node",
]


def sim_side_topics(num_robots):
    """Return (topic, domain_id) tuples for all sim-side topics at a given robot count.

    Both ms-airsim and Pegasus publish /clock per-robot-domain (ROS 2 has no
    cross-domain discovery), so there's one /clock on each robot's domain.
    """
    topics = []
    for n in range(1, num_robots + 1):
        topics.append(("/clock", n))
        for tmpl in SENSOR_TOPIC_TEMPLATES:
            topics.append((tmpl.format(N=n), n))
    return topics


# ── probe helpers (shared between one-shot tests and test_stable) ──────────

def _parse_panes(raw):
    """Return (crashed, active_count). Input lines: 'session:window|pane_pid|title|kids'.

    Crashed: pane with 0 descendants whose title isn't 'shell' (shell-tagged panes
    are intentionally idle bash). Active: pane with at least one descendant.
    """
    crashed = []
    active = 0
    for line in raw.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split("|")
        if len(parts) != 4:
            continue
        _, _, title, kids = parts
        kid_count = int(kids.strip() or 0)
        if kid_count == 0 and title != "shell":
            crashed.append(line)
        elif kid_count > 0:
            active += 1
    return crashed, active


def _check_tmux_panes(env):
    """Return (ok, msg). Zero-descendant pane (not a 'shell'-tagged idle bash) = crashed."""
    # Format: session:window|pane_pid|pane_title|descendant_count
    cmd = (
        "tmux list-panes -a -F '#{session_name}:#{window_name}|#{pane_pid}|#{pane_title}' "
        "| while IFS='|' read -r w pid t; do "
        "kids=$(pgrep -P \"$pid\" | wc -l); "
        "printf '%s|%s|%s|%s\\n' \"$w\" \"$pid\" \"$t\" \"$kids\"; "
        "done"
    )
    counts = {}
    sim_container = env["sim_container"]
    logger.info("Listing tmux panes in %s", sim_container)
    result = docker_exec(sim_container, cmd, timeout=10)
    if result.returncode != 0:
        return False, f"tmux list-panes failed in {sim_container}"
    sim_crashed, sim_active = _parse_panes(result.stdout)
    counts[sim_container] = sim_active
    if sim_crashed:
        logger.warning("Sim panes crashed in %s: %s", sim_container, sim_crashed)
        return False, f"sim panes crashed: {sim_crashed}"

    for rc in get_robot_containers(env["robot_pattern"]):
        logger.info("Listing tmux panes in %s", rc)
        r = docker_exec(rc, cmd, timeout=10)
        if r.returncode != 0:
            return False, f"tmux list-panes failed in {rc}"
        rcrashed, ractive = _parse_panes(r.stdout)
        counts[rc] = ractive
        if rcrashed:
            logger.warning("Robot %s panes crashed: %s", rc, rcrashed)
            return False, f"robot {rc} panes crashed: {rcrashed}"
    summary = ", ".join(f"{c}={n}" for c, n in counts.items())
    logger.info("All tmux panes active (%s)", summary)
    return True, f"all tmux panes active ({summary})"


def _check_sentinel_nodes(env):
    """Return (ok, msg). Expected sentinels per robot domain."""
    cfg = env["cfg"]
    robot_containers = get_robot_containers(env["robot_pattern"])
    if len(robot_containers) < env["num_robots"]:
        return False, f"only {len(robot_containers)}/{env['num_robots']} robot containers visible"
    all_missing = {}
    for n in range(1, env["num_robots"] + 1):
        logger.info("Checking sentinel nodes for robot_%d on domain %d in %s",
                    n, n, robot_containers[n - 1])
        result = ros2_exec(
            robot_containers[n - 1],
            "ros2 node list 2>/dev/null",
            domain_id=n, setup_bash=cfg["robot_setup_bash"], timeout=20,
        )
        if result.returncode != 0:
            return False, f"ros2 node list failed for robot_{n}"
        nodes = set(result.stdout.splitlines())
        expected = {t.format(N=n) for t in SENTINEL_NODE_TEMPLATES}
        missing = expected - nodes
        if missing:
            logger.warning("robot_%d missing nodes: %s", n, sorted(missing))
            all_missing[f"robot_{n}"] = sorted(missing)
    if all_missing:
        return False, f"missing sentinel nodes: {all_missing}"
    total = env["num_robots"] * len(SENTINEL_NODE_TEMPLATES)
    logger.info("All %d sentinel nodes present", total)
    return True, f"all {total} sentinel nodes present"


def _check_sim_publishing(env):
    """Parallel Hz sample of all sim-side topics. Returns (ok, msg, rates_dict)."""
    cfg = env["cfg"]
    topics = sim_side_topics(env["num_robots"])
    logger.info("Sampling Hz for %d sim-side topics", len(topics))
    rates = parallel_sample_hz(
        env["sim_container"], topics,
        setup_bash=cfg["sim_setup_bash"], duration=10, window=5,
    )
    stalled = [t for t, hz in rates.items() if hz is None or hz == 0.0]
    if stalled:
        logger.warning("Stalled topics: %s", stalled)
        return False, f"stalled topics: {stalled}", rates
    logger.info("%d topics healthy", len(rates))
    return True, f"{len(rates)} topics healthy", rates


# ── tests ──────────────────────────────────────────────────────────────────

def _poll_until(predicate, timeout, interval, fail_msg):
    """Sleep-poll `predicate` up to `timeout` seconds. On deadline, fail via
    `pytest.fail` with `fail_msg` (str) or `fail_msg()` (callable), so the
    message can reflect predicate-collected state."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(interval)
    pytest.fail(fail_msg() if callable(fail_msg) else fail_msg)


@pytest.mark.liveliness
@pytest.mark.timeout(1800)
class TestLiveliness:

    @pytest.mark.dependency(name="containers")
    def test_robot_containers_running(self, airstack_env):
        """Wait up to 120s for N robot containers to be Running."""
        num_robots = airstack_env["num_robots"]
        pattern = airstack_env["robot_pattern"]

        def ready():
            containers = get_robot_containers(pattern)
            return (len(containers) >= num_robots
                    and all(container_running(c) for c in containers))

        _poll_until(ready, timeout=120, interval=3,
                    fail_msg=lambda: f"only {len(get_robot_containers(pattern))}/"
                                     f"{num_robots} robot containers Running after 120s")

    @pytest.mark.dependency(name="sim_container", depends=["containers"])
    def test_sim_container_running(self, airstack_env):
        sc = airstack_env["sim_container"]
        _poll_until(lambda: container_running(sc),
                    timeout=120, interval=3,
                    fail_msg=f"{sc} not Running after 120s")

    @pytest.mark.dependency(depends=["containers"])
    def test_gcs_container_running(self, airstack_env):
        def ready():
            names = find_all_containers("gcs")
            return bool(names) and all(container_running(n) for n in names)
        _poll_until(ready, timeout=120, interval=3,
                    fail_msg="gcs container not Running after 120s")

    @pytest.mark.dependency(name="sim_ready", depends=["sim_container"])
    def test_sim_ready_time(self, airstack_env):
        """Wait for first /clock message from the sim container. 600s hard timeout.

        /clock is published per-robot-domain (see sim_side_topics); we listen on
        robot_1's domain as the readiness signal.
        """
        cfg = airstack_env["cfg"]
        m = get_metrics()
        tid = current_test_id()
        start = airstack_env["up_started_at"]

        if wait_for_first_message(
            airstack_env["sim_container"], "/clock",
            domain_id=1, setup_bash=cfg["sim_setup_bash"], timeout=600,
        ) is None:
            m.record(tid, "sim_ready_duration_s", "timeout", unit="s")
            pytest.fail("sim never published /clock within 600s")
        m.record(tid, "sim_ready_duration_s", round(time.time() - start, 2), unit="s")

    @pytest.mark.dependency(name="tmux", depends=["containers"])
    def test_tmux_panes_have_expected_processes(self, airstack_env):
        ok, msg = _check_tmux_panes(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="sim_publishing", depends=["sim_ready"])
    def test_sim_publishing(self, airstack_env):
        """Parallel Hz sample of all sim-side topics. Fail on any stalled topic."""
        ok, msg, _ = _check_sim_publishing(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="nodes", depends=["containers"])
    def test_sentinel_nodes_present(self, airstack_env):
        """Wait up to 300s for the expected sentinel nodes per robot."""
        last_msg = [""]

        def ready():
            ok, msg = _check_sentinel_nodes(airstack_env)
            last_msg[0] = msg
            return ok

        _poll_until(ready, timeout=300, interval=5,
                    fail_msg=lambda: f"sentinel nodes not ready after 300s: {last_msg[0]}")

    @pytest.mark.dependency(depends=["sim_ready", "sim_publishing", "nodes", "tmux"])
    def test_stable(self, airstack_env, request):
        """Poll every --stable-interval for up to --stable-duration. Early exit on failure.

        Only the raw hz time series is recorded per topic; compare_metrics.py
        derives mean/min/max/start_mean/end_mean from it."""
        duration = request.config.getoption("--stable-duration")
        interval = request.config.getoption("--stable-interval")
        m = get_metrics()
        tid = current_test_id()

        series = {}
        elapsed = 0

        try:
            while elapsed < duration:
                time.sleep(interval)
                elapsed += interval

                ok_t, msg_t = _check_tmux_panes(airstack_env)
                ok_n, msg_n = _check_sentinel_nodes(airstack_env)
                ok_p, msg_p, rates = _check_sim_publishing(airstack_env)

                for topic, hz in rates.items():
                    key = topic.lstrip("/").replace("/", ".")
                    series.setdefault(key, []).append({"t": elapsed, "hz": hz or 0.0})

                if not (ok_t and ok_n and ok_p):
                    pytest.fail(
                        f"instability at t={elapsed}s: tmux={msg_t} | "
                        f"nodes={msg_n} | publishing={msg_p}"
                    )
        finally:
            for key, samples in series.items():
                if samples:
                    m.record_list(tid, f"{key}.hz_samples", samples)
