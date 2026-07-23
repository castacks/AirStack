import sys
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

# Make tests/ importable so `from harness import ...` (and `from run_summary import ...`)
# resolve regardless of pytest's import mode.
_TESTS_DIR = str(Path(__file__).resolve().parent)
if _TESTS_DIR not in sys.path:
    sys.path.insert(0, _TESTS_DIR)

from harness import collection, session
# Re-export the harness helper API so existing `from conftest import <name>` in the
# system tests + sensor_probes keeps working unchanged.
from harness import *  # noqa: F401,F403
from harness.commands import _nodeid_dotted
from harness.discovery import _is_unit_item

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
    parser.addoption("--trajectory-types", default="Circle,Figure8,Racetrack,Line",
                     help="Comma-separated fixed trajectory types to sweep in "
                          "test_fixed_trajectory. Default: Circle,Figure8,Racetrack,Line")


def pytest_configure(config):
    run_dir = session.init_run_dir(AIRSTACK_ROOT)
    config.option.xmlpath = str(run_dir / "results.xml")

    # Co-located unit tests import their own package (e.g. `optitrack.natnet.emulator`,
    # `lidar_point_cloud_filter.validation_core`). Put each package/extension import
    # root (the parent of its test/ dir) on sys.path so they resolve without a
    # per-package conftest.py — a second conftest.py collides with this root one as
    # module `conftest` under --import-mode=importlib and breaks `from conftest import`.
    for d in unit_test_dirs():
        root = str(d.parent)
        if root not in sys.path:
            sys.path.insert(0, root)

    # Collect co-located unit tests: their files live outside tests/, so add the
    # explicit non-linter test files to the collection args. Skip when an explicit
    # path was given on the CLI (args_source == ARGS) so `pytest tests/system/foo.py`
    # still narrows as expected.
    src_name = getattr(getattr(config, "args_source", None), "name", "TESTPATHS")
    if src_name != "ARGS":
        for f in unit_test_files():
            entry = str(f)
            if entry not in config.args:
                config.args.append(entry)


def pytest_itemcollected(item):
    """Auto-mark co-located unit tests `unit` (before -m filtering runs).

    Idempotent when the source already declares the mark.
    """
    if _is_unit_item(item):
        item.add_marker(pytest.mark.unit)


def pytest_runtest_setup(item):
    session.set_current_item(item)


def pytest_runtest_teardown(item):
    session.set_current_item(None)


def pytest_sessionfinish(exitstatus):
    """Write summary.txt with key metrics so users don't need to dig through logs."""
    run_dir = session.run_dir()
    if run_dir is None:
        return
    try:
        from run_summary import write_summary
        summary_path = write_summary(run_dir)
        logger.info("Wrote run summary to %s", summary_path)
    except Exception as exc:
        logger.warning("Failed to write run summary: %s", exc)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """Attach phase reports to the item so fixtures can inspect pass/fail."""
    outcome = yield
    rep = outcome.get_result()
    setattr(item, f"_rep_{rep.when}", rep)


@contextmanager
def logger_to(log_name):
    """No-op kept for fixture call sites; output goes to pytest log_cli only."""
    yield


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


def pytest_collection_modifyitems(items):
    collection.modify_items(items)


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
    log = f"airstack_env.{_nodeid_dotted(session.current_item().nodeid, with_path_sep=True)}"

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
    runs ``airstack up robot-desktop`` and tears it down after the module.
    Behaves like the ``build_packages`` fixture — always brings up Docker when
    no container is found.
    """
    existing = find_container(_INTEGRATION_ROBOT_PATTERN)
    if existing and container_running(existing):
        yield {"container": existing, "brought_up": False}
        return

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
