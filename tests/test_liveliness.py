"""Liveliness stress tests — parametrized over (sim × num_robots × iteration).

Verifies Docker stack bring-up: containers Running, sim publishes ``/clock``,
tmux panes host expected processes, sentinel ROS 2 nodes exist, compute
snapshots, and a short stability window (infra only — no camera/LiDAR Hz here).

Sensor topic rates, bridge stereo Hz, LiDAR echo/sanity, and sim RTF live in
``test_sensors.py`` (``@pytest.mark.sensors``), ordered after this module.
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
    ros2_exec,
    sample_compute_usage,
    wait_for_first_message,
)


SENTINEL_NODE_TEMPLATES = [
    "/robot_{N}/interface/mavros/mavros",
    "/robot_{N}/robot_state_publisher",
    "/robot_{N}/trajectory_controller/trajectory_control_node",
]


def _parse_panes(raw):
    """Return (crashed, active_count). Input lines: 'session:window|pane_pid|title|kids'.

    Crashed: pane with no direct children whose title isn't 'shell' (shell-tagged
    panes are intentionally idle bash). Active: pane with at least one direct
    child.
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
    """Return (ok, msg). Pane with no direct child processes (and not a
    'shell'-tagged idle bash) = crashed."""
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
        logger.info(
            "Checking sentinel nodes for robot_%d on domain %d in %s",
            n,
            n,
            robot_containers[n - 1],
        )
        result = ros2_exec(
            robot_containers[n - 1],
            "ros2 node list 2>/dev/null",
            domain_id=n,
            setup_bash=cfg["robot_setup_bash"],
            timeout=20,
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


def _check_compute_usage(env):
    """Snapshot compute resources. Returns (ok, msg, samples_dict)."""
    logger.info("Sampling compute usage")
    try:
        samples = sample_compute_usage(env["sim_container"])
    except Exception as e:
        logger.warning("Compute sampling raised: %s", e)
        return False, f"compute sampling failed: {e}", {}
    if not samples:
        return False, "no compute samples returned", {}
    logger.info("Sampled %d compute metrics", len(samples))
    return True, f"{len(samples)} compute metrics sampled", samples


def _poll_until(predicate, timeout, interval, fail_msg):
    """Sleep-poll `predicate` up to `timeout` seconds."""
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
            return len(containers) >= num_robots and all(
                container_running(c) for c in containers
            )

        _poll_until(
            ready,
            timeout=120,
            interval=3,
            fail_msg=lambda: f"only {len(get_robot_containers(pattern))}/"
            f"{num_robots} robot containers Running after 120s",
        )

    @pytest.mark.dependency(name="sim_container", depends=["containers"])
    def test_sim_container_running(self, airstack_env):
        sc = airstack_env["sim_container"]
        _poll_until(
            lambda: container_running(sc),
            timeout=120,
            interval=3,
            fail_msg=f"{sc} not Running after 120s",
        )

    @pytest.mark.dependency(depends=["containers"])
    def test_gcs_container_running(self, airstack_env):
        def ready():
            names = find_all_containers("gcs")
            return bool(names) and all(container_running(n) for n in names)

        _poll_until(
            ready,
            timeout=120,
            interval=3,
            fail_msg="gcs container not Running after 120s",
        )

    @pytest.mark.dependency(name="sim_ready", depends=["sim_container"])
    def test_sim_ready_time(self, airstack_env):
        """Wait for first /clock message from the sim container (600s hard timeout)."""
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
            pytest.fail("sim never published /clock within 600s")
        m.record(tid, "sim_ready_duration_s", round(time.time() - start, 2), unit="s")

    @pytest.mark.dependency(name="tmux", depends=["containers"])
    def test_tmux_panes_have_expected_processes(self, airstack_env):
        ok, msg = _check_tmux_panes(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="compute", depends=["sim_ready"])
    def test_compute_usage(self, airstack_env):
        """Snapshot per-container CPU/mem/IO + host + GPU (diagnostic metrics)."""
        ok, msg, _ = _check_compute_usage(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="nodes", depends=["containers"])
    def test_sentinel_nodes_present(self, airstack_env):
        """Wait up to 300s for the expected sentinel nodes per robot."""
        last_msg = [""]

        def ready():
            ok, msg = _check_sentinel_nodes(airstack_env)
            last_msg[0] = msg
            return ok

        _poll_until(
            ready,
            timeout=300,
            interval=5,
            fail_msg=lambda: f"sentinel nodes not ready after 300s: {last_msg[0]}",
        )

    @pytest.mark.dependency(depends=["sim_ready", "nodes", "tmux"])
    def test_stable(self, airstack_env, request):
        """Poll infra only: tmux, sentinel nodes, compute (no sensor topic Hz)."""
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
                _, _, compute = _check_compute_usage(airstack_env)

                for key, value in compute.items():
                    series.setdefault(key, []).append({"t": elapsed, "value": value})

                if not (ok_t and ok_n):
                    pytest.fail(
                        f"instability at t={elapsed}s: tmux={msg_t} | nodes={msg_n}"
                    )
        finally:
            for key, samples in series.items():
                if samples:
                    m.record_list(tid, f"{key}_samples", samples)
