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

@pytest.mark.liveliness
@pytest.mark.timeout(1800)
class TestLiveliness:

    def test_robot_containers_running(self, airstack_env):
        """Wait up to 120s for N robot containers to be Running."""
        num_robots = airstack_env["num_robots"]
        pattern = airstack_env["robot_pattern"]

        deadline = time.time() + 120
        while time.time() < deadline:
            containers = get_robot_containers(pattern)
            if len(containers) >= num_robots and all(container_running(c) for c in containers):
                return
            time.sleep(3)
        pytest.fail(f"only {len(get_robot_containers(pattern))}/{num_robots} robot "
                    f"containers Running after 120s")

    def test_sim_container_running(self, airstack_env):
        sc = airstack_env["sim_container"]
        deadline = time.time() + 120
        while time.time() < deadline:
            if container_running(sc):
                return
            time.sleep(3)
        pytest.fail(f"{sc} not Running after 120s")

    def test_gcs_container_running(self, airstack_env):
        deadline = time.time() + 120
        while time.time() < deadline:
            names = find_all_containers("gcs")
            if names and all(container_running(n) for n in names):
                return
            time.sleep(3)
        pytest.fail("gcs container not Running after 120s")

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

    def test_tmux_panes_have_expected_processes(self, airstack_env):
        ok, msg = _check_tmux_panes(airstack_env)
        assert ok, msg

    def test_sim_publishing(self, airstack_env):
        """Parallel Hz sample of all sim-side topics. Fail on any stalled topic."""
        ok, msg, _ = _check_sim_publishing(airstack_env)
        assert ok, msg

    def test_sentinel_nodes_present(self, airstack_env):
        """Wait up to 300s for the expected sentinel nodes per robot."""
        deadline = time.time() + 300
        ok, msg = False, ""
        while time.time() < deadline:
            ok, msg = _check_sentinel_nodes(airstack_env)
            if ok:
                return
            time.sleep(5)
        pytest.fail(f"sentinel nodes not ready after 300s: {msg}")

    def test_stable(self, airstack_env, request):
        """Poll every --stable-interval for up to --stable-duration. Early exit on failure."""
        duration = request.config.getoption("--stable-duration")
        interval = request.config.getoption("--stable-interval")
        m = get_metrics()
        tid = current_test_id()

        series = {}
        elapsed = 0

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
                self._record_stable_aggregates(m, tid, series)
                pytest.fail(
                    f"instability at t={elapsed}s: tmux={msg_t} | "
                    f"nodes={msg_n} | publishing={msg_p}"
                )

        self._record_stable_aggregates(m, tid, series)

    @staticmethod
    def _record_stable_aggregates(m, tid, series):
        """Record aggregate stats + full time series per topic."""
        for key, samples in series.items():
            hz_values = [s["hz"] for s in samples]
            if not hz_values:
                continue
            m.record_list(tid, f"{key}.hz_samples", samples)
            m.record(tid, f"{key}.hz_first", hz_values[0], unit="Hz",
                     direction="higher_is_better")
            m.record(tid, f"{key}.hz_last", hz_values[-1], unit="Hz",
                     direction="higher_is_better")
            m.record(tid, f"{key}.hz_mean",
                     round(sum(hz_values) / len(hz_values), 2),
                     unit="Hz", direction="higher_is_better")
            m.record(tid, f"{key}.hz_range",
                     round(max(hz_values) - min(hz_values), 2),
                     unit="Hz", direction="lower_is_better")
