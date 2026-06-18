import importlib.util
import json
import logging
import os
import re
import shlex
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

import pytest
import yaml

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
            # Use the NatNet launch script so test_natnet_pose_alive always runs.
            # It is a superset of the plain script: same drones + NatNet emulator.
            "ISAAC_SIM_SCRIPT_NAME": "example_multi_px4_pegasus_natnet_launch_script.py",
            "PLAY_SIM_ON_START": "true",
            # Multi script gates RTX LiDAR on this flag; example_one always spawns it.
            # `sensors` tests expect ouster topics + lidar_point_cloud_filter path.
            "ENABLE_LIDAR": "true",
            # Always enable the robot-side NatNet stack for Isaac Sim tests so that
            # test_natnet_pose_alive runs regardless of the user's local .env setting.
            "LAUNCH_NATNET": "true",
            "SITL_PARAM_PROFILE": "px4-vision",
        },
    },
}

AIRSTACK_ROOT = os.environ.get("AIRSTACK_ROOT", str(Path(__file__).parent.parent))
COLCON_UNIT_TEST_PACKAGES_YAML = (
    Path(AIRSTACK_ROOT) / "tests" / "colcon_unit_test_packages.yaml"
)


def repo_path(*parts: str) -> Path:
    """Resolve a path relative to the repository root.

    ``AIRSTACK_ROOT`` is exported by CI and defaults to the repo root locally,
    so this is the single source of truth for cross-tree paths — no test or
    proxy file should hardcode ``Path(__file__).parents[N]`` walks.
    """
    return Path(AIRSTACK_ROOT).joinpath(*parts)


def register_unit_tests(target_globals: dict, test_dir: Path, *module_files: str) -> None:
    """Register co-located unit tests with a thin proxy module under ``tests/``.

    AirStack keeps unit test **source** next to each package (colcon convention),
    but CI runs pytest tests/. Each proxy file calls this helper so pytest
    discovers the real tests.

    For each module_file in test_dir:

    1. Prepend test_dir and test_dir.parent (package/extension root) to
       sys.path so the loaded module can import production code and sibling
       helpers (e.g. natnet_test_helpers).
    2. Load the file with importlib.util.spec_from_file_location under a
       synthetic module name (_unit_<package>_<stem>) to avoid circular
       imports when proxy and source share the same basename.
    3. Copy every test_* callable from the loaded module into
       target_globals (typically the proxy's globals()) so pytest
       collects them as if they lived under tests/.
    4. Wrap each callable with pytest.mark.unit so -m unit works even
       when the source module omits pytestmark.

    Args:
        target_globals: Namespace to populate (pass globals() from the proxy).
        test_dir: Directory containing co-located test modules
            (e.g. repo_path("robot/ros_ws/src/<pkg>/test")).
        *module_files: One or more filenames under test_dir; multiple files
            may be folded into a single proxy.

    pytest <package>/test/ or colcon test bypasses proxies; use a local conftest.py in the package test/ dir for sys.path setup instead.
    """
    for path in (test_dir, test_dir.parent):
        entry = str(path)
        if entry not in sys.path:
            sys.path.insert(0, entry)
    prefix = re.sub(r"\W", "_", test_dir.parent.name)
    for module_file in module_files:
        real_file = test_dir / module_file
        module_name = f"_unit_{prefix}_{Path(module_file).stem}"
        spec = importlib.util.spec_from_file_location(module_name, real_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for name in dir(module):
            if name.startswith("test_"):
                target_globals[name] = pytest.mark.unit(getattr(module, name))


def load_colcon_unit_test_config(workspace="robot"):
    """Load colcon test package list and pytest args from tests/colcon_unit_test_packages.yaml."""
    if not COLCON_UNIT_TEST_PACKAGES_YAML.is_file():
        raise FileNotFoundError(
            f"Missing {COLCON_UNIT_TEST_PACKAGES_YAML} — add packages to gate in colcon test."
        )
    with COLCON_UNIT_TEST_PACKAGES_YAML.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if workspace not in data:
        raise KeyError(
            f"No '{workspace}' entry in {COLCON_UNIT_TEST_PACKAGES_YAML.name}"
        )
    cfg = data[workspace] or {}
    packages = cfg.get("packages") or []
    if not packages:
        raise ValueError(
            f"'{workspace}.packages' is empty in {COLCON_UNIT_TEST_PACKAGES_YAML.name}"
        )
    return packages, cfg.get("pytest_args", "")


def colcon_test_robot_command(workspace="robot"):
    """Shell command for colcon test over unit-test packages (robot workspace)."""
    packages, pytest_args = load_colcon_unit_test_config(workspace)
    pkg_list = " ".join(packages)
    ros_plugin_disable = (
        "PYTEST_ADDOPTS='-p no:launch_testing -p no:launch_testing_ros' "
    )
    cmd = (
        ros_plugin_disable
        + f"colcon test --packages-select {pkg_list} "
        "--event-handlers console_direct+ --return-code-on-test-failure"
    )
    if pytest_args:
        cmd += f' --pytest-args "{pytest_args}"'
    return cmd
# Unit tests live co-located with their ROS 2 packages in robot/ros_ws/src/.
# Thin proxy files under tests/robot/ (etc.) call register_unit_tests() so
# `pytest tests/` and `airstack test -m unit` discover them without sys.path
# boilerplate in this root conftest.
RUN_DIR = None
LOGS_DIR = None
ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"

# Track the currently-running pytest item so current_log() and current_test_id()
# can pick up the parametrize id without tests having to pass `request` around.
_CURRENT_ITEM = None
METRICS = None


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
    parser.addoption("--stress-iterations", type=int, default=1,
                     help="Number of up/down iterations per (sim, num_robots) config")
    parser.addoption("--stable-duration", type=int, default=120,
                     help="Seconds test_stable polls for")
    parser.addoption("--stable-interval", type=int, default=10,
                     help="Seconds between polls in test_stable")
    parser.addoption("--gui", action="store_true", default=False,
                     help="Show sim GUI windows for visual sanity checks. "
                          "Default: headless (no X, good for CI).")
    parser.addoption("--takeoff-velocities", default="0.5",
                     help="Comma-separated takeoff/land velocities (m/s) to "
                          "sweep in test_takeoff_hover_land. Default: 0.5,1,2")
    parser.addoption("--run-integration", action="store_true", default=False,
                     help="Let integration tests (tests/integration/) bring up "
                          "the robot container themselves. Without it they reuse "
                          "an already-running container or skip. Keeps a plain "
                          "`pytest tests/` from spinning up Docker.")


def _chmod_world_writable(path: Path) -> None:
    """Best-effort: allow Docker and host users to share tests/results."""
    try:
        path.chmod(0o777)
    except OSError:
        pass


def pytest_configure(config):
    global RUN_DIR, LOGS_DIR
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    results_root = Path(AIRSTACK_ROOT) / "tests" / "results"
    results_root.mkdir(parents=True, exist_ok=True)
    _chmod_world_writable(results_root)
    RUN_DIR = results_root / timestamp
    LOGS_DIR = RUN_DIR / "logs"
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    _chmod_world_writable(RUN_DIR)
    _chmod_world_writable(LOGS_DIR)
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


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """Attach phase reports to the item so fixtures can inspect pass/fail."""
    outcome = yield
    rep = outcome.get_result()
    setattr(item, f"_rep_{rep.when}", rep)


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


# Run cheap/fast-fail tests first so real problems surface early:
# unit → docker image builds → colcon builds → integration (robot container) →
# liveliness (full stack infra) → sensors → autonomy flight tests.
_MODULE_ORDER = [
    # Unit tests first — fast, hermetic, no Docker.  Any module whose dotted
    # name starts with "robot." or "sim." is a proxy for a package-level unit
    # test and sorts into this leading slot via the prefix check below.
    "__unit__",
    "system.test_build_docker",
    "system.test_build_packages",
    # Integration needs robot-desktop image + colcon-built packages (e.g. natnet_ros2).
    "__integration__",
    "system.test_liveliness",
    "system.test_sensors",
    "system.test_takeoff_hover_land",
]

# Within test_takeoff_hover_land, each (env, velocity) runs phases in this chain order.
_AUTONOMY_PHASE_ORDER = [
    "test_px4_ready",
    "test_takeoff",
    "test_hover",
    "test_landing",
]


def _rank(name, order):
    """Index of `name` in `order`; `len(order)` if unknown (i.e., sort last)."""
    return order.index(name) if name in order else len(order)


def _module_key(item):
    """Return the ordering key for an item.

    Unit-test proxies live under robot/, sim/, or gcs/.
    Integration tests live under integration/.
    Everything else uses the dotted module __name__ against _MODULE_ORDER.
    """
    if item.nodeid.startswith(("robot/", "sim/", "gcs/")):
        return _rank("__unit__", _MODULE_ORDER)
    if item.nodeid.startswith("integration/"):
        return _rank("__integration__", _MODULE_ORDER)
    return _rank(getattr(item.module, "__name__", ""), _MODULE_ORDER)


def pytest_collection_modifyitems(items):
    # 1. Cross-module: enforce `_MODULE_ORDER`. Stable sort keeps within-module
    #    order intact, so pytest's default file/class order survives.
    items.sort(key=_module_key)

    # 2. Within test_takeoff_hover_land: sort by (airstack_env, velocity, phase) so each
    #    (sim, robots, iter) env brings up the stack once and the drone goes
    #    ground→air→ground per velocity.
    def phase(item):
        if getattr(item.module, "__name__", "") != "system.test_takeoff_hover_land":
            return None
        name = item.originalname or item.name.split("[", 1)[0]
        return _rank(name, _AUTONOMY_PHASE_ORDER)

    def sort_key(item):
        cs = getattr(item, "callspec", None)
        env = cs.params.get("airstack_env", ()) if cs else ()
        vel = float(cs.params.get("velocity", 0.0)) if cs else 0.0
        return (env, vel, phase(item))

    slots = [(i, it) for i, it in enumerate(items) if phase(it) is not None]
    if slots:
        sorted_items = sorted((it for _, it in slots), key=sort_key)
        for (i, _), new_item in zip(slots, sorted_items):
            items[i] = new_item

    # 3. Rewrite bracketed test IDs into a consistent hierarchy: sim > robots >
    # velocity > iteration. Bypasses pytest's own concatenation (which would
    # otherwise order by reverse-parametrize-call order). Keeps pytest console,
    # JUnit XML, and metrics.json all in the same natural order without
    # refactoring the parametrize structure.
    for item in items:
        cs = getattr(item, "callspec", None)
        if cs is None:
            continue
        env = cs.params.get("airstack_env")
        parts = []
        if env:
            sim, n, i = env
            parts.append(f"{sim}-rob#{n}")
        if "velocity" in cs.params:
            parts.append(f"v{cs.params['velocity']}")
        if env:
            parts.append(f"iter{i}")
        if not parts:
            continue
        new_id = "-".join(parts)
        if cs.id == new_id:
            continue
        item.name = item.name.replace(f"[{cs.id}]", f"[{new_id}]")
        item._nodeid = item._nodeid.replace(f"[{cs.id}]", f"[{new_id}]")


# ── logging / subprocess helpers ───────────────────────────────────────────

def _nodeid_dotted(nodeid, with_path_sep=False):
    """pytest nodeid → `module.Class.test_name[params]` form. When
    `with_path_sep=True`, also flattens `/` in path prefixes (for log filenames)."""
    out = nodeid.replace(".py::", ".").replace("::", ".")
    return out.replace("/", ".") if with_path_sep else out


def current_log():
    """Log name for the currently-running pytest item, or None outside a test.

    Subprocess helpers default to this so every call fired from a test auto-logs
    to the right file without plumbing log_name through every layer."""
    if _CURRENT_ITEM is None:
        return None
    return _nodeid_dotted(_CURRENT_ITEM.nodeid, with_path_sep=True)


# Matches any ANSI escape sequence (color codes, cursor moves, etc.)
_ANSI_RE = re.compile(r'\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')


def read_log_tail(log_name=None, lines=50):
    """Return the last *lines* lines of a test log file, cleaned up for display.

    Strips ANSI escape codes and normalises carriage-return progress lines
    (e.g. Docker build context updates) so the result renders as plain text
    with real newline characters.
    """
    log_name = log_name or current_log()
    if not log_name:
        return ""
    log_path = LOGS_DIR / f"{log_name}.log"
    if log_path.exists():
        text = log_path.read_text()
        text = _ANSI_RE.sub('', text)               # remove colour codes
        text = text.replace('\r\n', '\n').replace('\r', '\n')  # normalise CR
        all_lines = text.splitlines()
        return "\n".join(all_lines[-lines:])
    return ""


def _run_teed(cmd_list, timeout, log_name=None, env=None, cwd=None):
    """Run a subprocess, teeing stdout+stderr live to the log file and
    capturing them for parsing."""
    log_name = log_name or current_log()
    if not log_name:
        return subprocess.run(cmd_list, capture_output=True, text=True,
                              timeout=timeout, env=env, cwd=cwd)
    log_path = LOGS_DIR / f"{log_name}.log"
    quoted = " ".join(shlex.quote(a) for a in cmd_list)
    with open(log_path, "a") as f:
        f.write(f"\n$ {quoted}\n")
    shell_cmd = f"set -o pipefail; {quoted} 2>&1 | tee -a {shlex.quote(str(log_path))}"
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
    """Return running robot containers sorted by their replica index"""
    def _index(name):
        tail = name.rsplit("-", 1)[-1]
        return int(tail) if tail.isdigit() else 0
    return sorted(find_all_containers(pattern), key=_index)


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


# ── compute-usage sampling ─────────────────────────────────────────────────

_BYTES_RE = re.compile(r"([\d.]+)\s*([kKMGT]?i?B)$")
_BYTES_TO_MB = {
    "B": 1 / (1024 * 1024),
    "KiB": 1 / 1024, "KB": 1 / 1000, "kB": 1 / 1000,
    "MiB": 1, "MB": 1,
    "GiB": 1024, "GB": 1000,
    "TiB": 1024 * 1024, "TB": 1_000_000,
}


def _parse_docker_bytes(s):
    """Parse a docker-stats byte string (e.g. '123.4MiB', '0B') to MB."""
    m = _BYTES_RE.match((s or "").strip())
    if not m:
        return 0.0
    return float(m.group(1)) * _BYTES_TO_MB.get(m.group(2), 1)


def sample_compute_usage(sim_container):
    """Snapshot of compute resources: per-container CPU/mem/disk-IO/net-IO plus
    global host CPU/mem and GPU util/VRAM/temp/power. Returns {key: value},
    keys shaped `{entity}.{metric}` where entity is the full container name or
    'host'. Per-robot replicas (e.g. airstack-robot-desktop-1/2/3) are kept
    distinct so raw metrics.json preserves per-robot data; parse_metrics
    pools them at report time. Silently omits metrics that fail to sample."""
    import psutil

    out = {}

    stats = _run_teed(
        ["docker", "stats", "--no-stream", "--format", "{{json .}}"],
        timeout=20,
    )
    for line in stats.stdout.strip().splitlines():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        name = d.get("Name", "")
        if not name or name.startswith("docker-test-run"):
            continue
        out[f"{name}.cpu_pct"] = float(d.get("CPUPerc", "0%").rstrip("%") or 0)
        mem_raw = d.get("MemUsage", "").split("/")[0].strip()
        out[f"{name}.mem_mb"] = _parse_docker_bytes(mem_raw)
        for io_field, metric in (("BlockIO", "disk_io_mb"), ("NetIO", "net_io_mb")):
            parts = (d.get(io_field, "") or "").split("/")
            total = sum(_parse_docker_bytes(p.strip()) for p in parts)
            out[f"{name}.{metric}"] = total

    out["host.cpu_pct"] = psutil.cpu_percent(interval=0.5)
    out["host.mem_mb"] = psutil.virtual_memory().used / (1024 * 1024)

    gpu = _run_teed(
        ["docker", "exec", sim_container, "nvidia-smi",
         "--query-gpu=utilization.gpu,memory.used,temperature.gpu,power.draw",
         "--format=csv,noheader,nounits"],
        timeout=10,
    )
    if gpu.returncode == 0 and gpu.stdout.strip():
        fields = [f.strip() for f in gpu.stdout.strip().splitlines()[0].split(",")]
        if len(fields) >= 4:
            try:
                out["host.gpu_pct"] = float(fields[0])
                out["host.vram_mb"] = float(fields[1])
                out["host.gpu_temp_c"] = float(fields[2])
                out["host.gpu_power_w"] = float(fields[3])
            except ValueError:
                pass

    return out


def _compose_images(env=None):
    """Resolved image refs that `docker compose up` would use under `env`."""
    compose_env = os.environ.copy()
    if env:
        compose_env.update(env)
    result = _run_teed(
        ["docker", "compose", "-f", str(Path(AIRSTACK_ROOT) / "docker-compose.yaml"),
         "config", "--images"],
        timeout=30, env=compose_env, cwd=AIRSTACK_ROOT,
    )
    return [l.strip() for l in result.stdout.strip().splitlines() if l.strip()]


def missing_images(env=None):
    """Images required by the current compose config but not present locally.
    Used by airstack_env to fail fast instead of letting `airstack up` hang
    pulling/building when images haven't been prebuilt."""
    missing = []
    for image in _compose_images(env=env):
        result = _run_teed(
            ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
            timeout=10,
        )
        if result.returncode != 0:
            missing.append(image)
    return missing


def docker_image_size_mb(service, env=None):
    image = next((i for i in _compose_images(env=env) if service in i), None)
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
        self._lock = threading.Lock()

    def _flush(self):
        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        tmp.write_text(json.dumps(self._data, indent=2))
        os.replace(tmp, self._path)

    def record(self, test_name, key, value, unit="", direction="lower_is_better", **extra):
        with self._lock:
            if test_name not in self._data:
                self._data[test_name] = {}
            entry = {"value": value, "unit": unit, "direction": direction}
            entry.update(extra)
            self._data[test_name][key] = entry
            self._flush()

    def record_list(self, test_name, key, values):
        """Store a raw list (time series) — not scored by parse_metrics."""
        with self._lock:
            if test_name not in self._data:
                self._data[test_name] = {}
            self._data[test_name][key] = {"samples": values}
            self._flush()

def get_metrics():
    global METRICS
    if METRICS is None:
        METRICS = MetricsRecorder(RUN_DIR / "metrics.json")
    return METRICS


def current_test_id():
    """Test id used as the metrics.json key. Matches JUnit XML's classname.name
    format so parse_metrics.py can merge results.xml and metrics.json entries."""
    if _CURRENT_ITEM is None:
        return "unknown"
    return _nodeid_dotted(_CURRENT_ITEM.nodeid)


# ── shared sim test infrastructure (liveliness, sensors, comms, takeoff reuse) ──

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
    # Route fixture narration to a file whose name tracks the post-rewrite
    # test id (see pytest_collection_modifyitems), so airstack up/down output
    # lands next to the triggering test's own log instead of under pytest's
    # stale callspec.id.
    log = f"airstack_env.{_nodeid_dotted(_CURRENT_ITEM.nodeid, with_path_sep=True)}"

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
        missing = missing_images(env=env_overrides)
        if missing:
            pytest.fail(
                "Required docker images not built locally:\n  - "
                + "\n  - ".join(missing)
                + "\nBuild them first, e.g. `airstack test -m build_docker` "
                  "or `airstack image-build <service>`."
            )
        logger.info("Shutting down any previously running stack")
        airstack_cmd("down", timeout=120, log_name=log)
        logger.info("Bringing up stack: sim=%s num_robots=%d iter=%d headless=%s",
                    sim, num_robots, iteration, headless)
        t0 = time.time()
        up_result = airstack_cmd("up",
                                 env_overrides=env_overrides, timeout=180, log_name=log)
        up_cmd_duration_s = round(time.time() - t0, 2)
        logger.info("airstack up returned %d in %.2fs",
                    up_result.returncode, up_cmd_duration_s)
        if up_result.returncode != 0:
            pytest.fail(f"airstack up failed:\n{read_log_tail(log)}")

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


# ── integration tier (tests/integration/) ─────────────────────────────────

_INTEGRATION_ROBOT_PATTERN = "robot.*desktop"
# Robot-only bring-up: autonomy stack on, no sim profile, single robot.
_INTEGRATION_ENV = {
    "AUTOLAUNCH": "true",
    "NUM_ROBOTS": "1",
    "COMPOSE_PROFILES": "desktop",
}


@pytest.fixture(scope="module")
def robot_autonomy_stack(request):
    """Robot-desktop container for integration tests (no sim, no GPU).

    Yields ``{"container": <name>, "brought_up": bool}``. Reuses an already
    running container (fast local iteration, left running afterward); otherwise
    runs ``airstack up robot-desktop`` only when ``--run-integration`` is passed
    (and tears it down). Without the flag and with nothing running, the test is
    skipped, so a plain ``pytest tests/`` never spins up Docker for this tier.
    """
    existing = find_container(_INTEGRATION_ROBOT_PATTERN)
    if existing and container_running(existing):
        yield {"container": existing, "brought_up": False}
        return

    if not request.config.getoption("--run-integration"):
        pytest.skip(
            "No running robot-desktop container. Start one "
            "(`AUTOLAUNCH=false airstack up robot-desktop`) or pass "
            "`--run-integration` to let the harness bring it up."
        )

    log = "robot_autonomy_stack"
    with logger_to(log):
        missing = missing_images(env=_INTEGRATION_ENV)
        if missing:
            pytest.skip("robot-desktop image not built locally: " + ", ".join(missing))
        airstack_cmd("down", timeout=120, log_name=log)
        result = airstack_cmd("up", "robot-desktop",
                              env_overrides=_INTEGRATION_ENV, timeout=180, log_name=log)
        if result.returncode != 0:
            pytest.fail(f"`airstack up robot-desktop` failed:\n{read_log_tail(log)}")

    container = wait_for_container(_INTEGRATION_ROBOT_PATTERN, timeout=120)
    assert container, "robot-desktop container not Running after 120s"
    try:
        yield {"container": container, "brought_up": True}
    finally:
        with logger_to(log):
            airstack_cmd("down", timeout=120, log_name=log)