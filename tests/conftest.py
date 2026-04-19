import json
import logging
import os
import re
import shlex
import subprocess
import time
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

import pytest

AIRSTACK_ROOT = os.environ.get("AIRSTACK_ROOT", str(Path(__file__).parent.parent))
RUN_DIR = None
LOGS_DIR = None
ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"

# Track the currently-running pytest item so current_log() and current_test_id()
# can pick up the parametrize id without tests having to pass `request` around.
_CURRENT_ITEM = None

logger = logging.getLogger("airstack")
logger.setLevel(logging.INFO)
_LOG_FORMAT = logging.Formatter("[%(asctime)s] %(levelname)s %(message)s", "%H:%M:%S")
_test_log_handler = None


# ── pytest config / hooks ──────────────────────────────────────────────────

def pytest_addoption(parser):
    parser.addoption("--sim", default="msairsim,isaacsim",
                     help="Comma-separated sim targets: msairsim, isaacsim")
    parser.addoption("--num-robots", default="1,3",
                     help="Comma-separated robot counts, e.g. 1,3")
    parser.addoption("--stress-iterations", type=int, default=3,
                     help="Number of up/down iterations per (sim, num_robots) config")
    parser.addoption("--stable-duration", type=int, default=120,
                     help="Seconds test_stable polls for")
    parser.addoption("--stable-interval", type=int, default=10,
                     help="Seconds between polls in test_stable")
    parser.addoption("--gui", action="store_true", default=False,
                     help="Show sim GUI windows for visual sanity checks. "
                          "Default: headless (no X, good for CI).")


def pytest_configure(config):
    global RUN_DIR, LOGS_DIR
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    results_root = Path(AIRSTACK_ROOT) / "tests" / "results"
    RUN_DIR = results_root / timestamp
    LOGS_DIR = RUN_DIR / "logs"
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    config.option.xmlpath = str(RUN_DIR / "results.xml")


def pytest_runtest_setup(item):
    global _CURRENT_ITEM, _test_log_handler
    _CURRENT_ITEM = item
    log_path = LOGS_DIR / f"{current_log()}.log"
    _test_log_handler = logging.FileHandler(log_path)
    _test_log_handler.setFormatter(_LOG_FORMAT)
    logger.addHandler(_test_log_handler)


def pytest_runtest_teardown(item):
    global _CURRENT_ITEM, _test_log_handler
    if _test_log_handler is not None:
        logger.removeHandler(_test_log_handler)
        _test_log_handler.close()
        _test_log_handler = None
    _CURRENT_ITEM = None


@contextmanager
def logger_to(log_name):
    """Temporarily route `logger` to a different file. Suspends any handlers
    already attached so narration isn't duplicated across files."""
    existing = list(logger.handlers)
    for h in existing:
        logger.removeHandler(h)
    fh = logging.FileHandler(LOGS_DIR / f"{log_name}.log")
    fh.setFormatter(_LOG_FORMAT)
    logger.addHandler(fh)
    try:
        yield
    finally:
        logger.removeHandler(fh)
        fh.close()
        for h in existing:
            logger.addHandler(h)


def pytest_generate_tests(metafunc):
    """Parametrize airstack_env over (sim, num_robots, iteration) from CLI flags.

    Only triggers for tests that request the airstack_env fixture — other tests
    are untouched.
    """
    if "airstack_env" not in metafunc.fixturenames:
        return
    sims = [s.strip() for s in metafunc.config.getoption("--sim").split(",") if s.strip()]
    nums = [int(x) for x in metafunc.config.getoption("--num-robots").split(",") if x.strip()]
    iterations = metafunc.config.getoption("--stress-iterations")
    params = [(s, n, i) for s in sims for n in nums for i in range(iterations)]
    ids = [f"{s}-{n}-iter{i}" for s, n, i in params]
    metafunc.parametrize("airstack_env", params, ids=ids, indirect=True, scope="class")


# ── logging / subprocess helpers ───────────────────────────────────────────

def current_log():
    """Log name for the currently-running pytest item, or None outside a test.

    Subprocess helpers default to this so every call fired from a test auto-logs
    to the right file without plumbing log_name through every layer.
    """
    if _CURRENT_ITEM is None:
        return None
    # "test_liveliness.py::TestLiveliness::test_foo[id]" →
    # "test_liveliness.TestLiveliness.test_foo[id]"
    return (_CURRENT_ITEM.nodeid
            .replace("/", ".")
            .replace(".py::", ".")
            .replace("::", "."))


def read_log_tail(log_name=None, lines=50):
    log_name = log_name or current_log()
    if not log_name:
        return ""
    log_path = LOGS_DIR / f"{log_name}.log"
    if log_path.exists():
        all_lines = log_path.read_text().splitlines()
        return "\n".join(all_lines[-lines:])
    return ""


def _run_teed(cmd_list, timeout, log_name=None, env=None, cwd=None):
    """Run a subprocess, teeing stdout+stderr live to the log file and
    capturing them for parsing.

    
    """
    log_name = log_name or current_log()
    if not log_name:
        return subprocess.run(cmd_list, capture_output=True, text=True,
                              timeout=timeout, env=env, cwd=cwd)
    log_path = LOGS_DIR / f"{log_name}.log"
    quoted = " ".join(shlex.quote(a) for a in cmd_list)
    with open(log_path, "a") as f:
        f.write(f"\n$ {quoted}\n")
    shell_cmd = f"{quoted} 2>&1 | tee -a {shlex.quote(str(log_path))}"
    return subprocess.run(["bash", "-c", shell_cmd],
                          capture_output=True, text=True,
                          timeout=timeout, env=env, cwd=cwd)


def docker_exec(container, cmd, timeout=60, log_name=None):
    full_cmd = ["docker", "exec", container, "bash", "-c", cmd]
    return _run_teed(full_cmd, timeout=timeout, log_name=log_name)


def airstack_cmd(*args, env_overrides=None, timeout=1800, log_name=None):
    env = os.environ.copy()
    if env_overrides:
        env.update(env_overrides)
    cmd = [str(Path(AIRSTACK_ROOT) / "airstack.sh")] + list(args)
    return _run_teed(cmd, timeout=timeout, log_name=log_name,
                     env=env, cwd=AIRSTACK_ROOT)


def ros2_env(setup_bash, domain_id):
    """Shell prefix that makes `ros2` available on the requested domain."""
    return (
        f"source {ROS_DISTRO_SETUP} && source {setup_bash} "
        f"&& export ROS_DOMAIN_ID={domain_id}"
    )


def ros2_exec(container, ros2_cmd, domain_id=0, setup_bash=None, timeout=15, log_name=None):
    """Run `ros2 ...` inside a container with the right workspace sourced."""
    setup = setup_bash or "/root/AirStack/robot/ros_ws/install/setup.bash"
    inner = f"{ros2_env(setup, domain_id)} && {ros2_cmd}"
    return docker_exec(container, inner, timeout=timeout, log_name=log_name)


_HZ_RE = re.compile(r"average rate:\s+([\d.]+)")


def _parse_hz(text):
    m = _HZ_RE.search(text or "")
    return float(m.group(1)) if m else None


# ── container helpers ──────────────────────────────────────────────────────

def find_all_containers(name_pattern):
    result = _run_teed(
        ["docker", "ps", "--filter", f"name={name_pattern}", "--format", "{{.Names}}"],
        timeout=10,
    )
    return [n for n in result.stdout.strip().splitlines() if n]


def find_container(name_pattern):
    names = find_all_containers(name_pattern)
    return names[0] if names else None


def get_robot_containers(pattern="robot.*desktop"):
    """Return a sorted list of currently-running robot container names."""
    return sorted(find_all_containers(pattern))


def container_running(name):
    """True if the named container is currently Running."""
    result = _run_teed(
        ["docker", "inspect", "-f", "{{.State.Running}}", name],
        timeout=10,
    )
    return "true" in result.stdout


def wait_for_container(name_pattern, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        name = find_container(name_pattern)
        if name and container_running(name):
            return name
        time.sleep(5)
    raise TimeoutError(f"Container matching '{name_pattern}' not running after {timeout}s")


def docker_image_size_mb(service, env=None):
    compose_env = os.environ.copy()
    if env:
        compose_env.update(env)
    result = _run_teed(
        ["docker", "compose", "-f", str(Path(AIRSTACK_ROOT) / "docker-compose.yaml"),
         "config", "--images"],
        timeout=30, env=compose_env, cwd=AIRSTACK_ROOT,
    )
    image = next((l.strip() for l in result.stdout.strip().splitlines() if service in l), None)
    if not image:
        return None
    result = _run_teed(
        ["docker", "image", "inspect", image, "--format", "{{.Size}}"],
        timeout=10,
    )
    if result.returncode == 0 and result.stdout.strip():
        return round(int(result.stdout.strip()) / 1_000_000, 1)
    return None


# ── metrics ────────────────────────────────────────────────────────────────

class MetricsRecorder:
    def __init__(self, path):
        self._path = path
        self._data = json.loads(path.read_text()) if path.exists() else {}

    def record(self, test_name, key, value, unit="", direction="lower_is_better"):
        if test_name not in self._data:
            self._data[test_name] = {}
        self._data[test_name][key] = {
            "value": value, "unit": unit, "direction": direction,
        }
        self._path.write_text(json.dumps(self._data, indent=2))

    def record_list(self, test_name, key, values):
        """Store a raw list (time series) — not scored by compare_metrics."""
        if test_name not in self._data:
            self._data[test_name] = {}
        self._data[test_name][key] = {"samples": values}
        self._path.write_text(json.dumps(self._data, indent=2))


METRICS = None


def get_metrics():
    global METRICS
    if METRICS is None:
        METRICS = MetricsRecorder(RUN_DIR / "metrics.json")
    return METRICS


def current_test_id():
    """Full pytest test id for this test invocation — used as the metrics.json key."""
    if _CURRENT_ITEM is None:
        return "unknown"
    return _CURRENT_ITEM.nodeid


# ── shared sim test infrastructure (liveliness, comms, takeoff all reuse) ──

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
        },
    },
}


def wait_for_first_message(container, topic, domain_id, setup_bash, timeout=60):
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


@pytest.fixture
def airstack_env(request):
    """Parametrized fixture: runs `airstack up`, yields env dict, tears down.

    Parametrized by `pytest_generate_tests` over (sim, num_robots, iteration)
    tuples derived from CLI flags.

    Deliberately does NOT wait for containers or ROS2 nodes to be ready — tests
    own their wait loops + timeout metrics so failures attribute to the right layer.
    """
    sim, num_robots, iteration = request.param
    cfg = SIM_CONFIG[sim]
    log = f"airstack_env[{_CURRENT_ITEM.callspec.id}]"

    headless = not request.config.getoption("--gui")
    env_overrides = {
        "AUTOLAUNCH": "true",
        "NUM_ROBOTS": str(num_robots),
        "COMPOSE_PROFILES": f"desktop,{cfg['profile']}",
        "MS_AIRSIM_HEADLESS": "true" if headless else "false",
        "ISAAC_SIM_HEADLESS": "true" if headless else "false",
    }
    if headless:
        # Forces rviz/Qt apps to render offscreen instead of spawning windows.
        env_overrides["QT_QPA_PLATFORM"] = "offscreen"
    env_overrides.update(cfg.get("extra_env", {}))

    with logger_to(log):
        logger.info("Bringing up stack: sim=%s num_robots=%d iter=%d headless=%s",
                    sim, num_robots, iteration, headless)
        t0 = time.time()
        up_result = airstack_cmd("up",
                                 env_overrides=env_overrides, timeout=180, log_name=log)
        up_cmd_duration_s = round(time.time() - t0, 2)
        logger.info("airstack up returned %d in %.2fs",
                    up_result.returncode, up_cmd_duration_s)
        assert up_result.returncode == 0, \
            f"airstack up failed:\n{read_log_tail(log)}"

    env = {
        "sim": sim,
        "num_robots": num_robots,
        "iteration": iteration,
        "sim_container": cfg["sim_container"],
        "robot_pattern": "robot.*desktop",
        "up_started_at": t0,
        "cfg": cfg,
    }

    tid = current_test_id()
    m = get_metrics()
    m.record(tid, "airstack_up_duration_s", up_cmd_duration_s, unit="s")

    try:
        yield env
    finally:
        with logger_to(log):
            logger.info("Tearing down stack")
            t3 = time.time()
            airstack_cmd("down", timeout=120, log_name=log)
            down_duration_s = round(time.time() - t3, 2)
            logger.info("Teardown finished in %.2fs", down_duration_s)
        m.record(tid, "airstack_down_duration_s", down_duration_s, unit="s")