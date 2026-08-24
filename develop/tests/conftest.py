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

from harness import collection, session as harness_session
# Re-export the harness helper API so existing `from conftest import <name>` in the
# system tests + sensor_probes keeps working unchanged.
from harness import *  # noqa: F401,F403
from harness.commands import _nodeid_dotted
from harness.discovery import _is_unit_item
from harness.run_meta import write_run_meta

# ── pytest config / hooks ──────────────────────────────────────────────────

def pytest_addoption(parser):
    parser.addoption("--sim", default="isaacsim",
                     help="Comma-separated sim targets: isaacsim, msairsim, "
                          "simplesim. Default isaacsim; pass --sim msairsim "
                          "to opt in. simplesim only drives the simple_sim "
                          "smoke test (-m simple_sim).")
    parser.addoption("--num-robots", default="1,3",
                     help="Comma-separated robot counts, e.g. 1,3")
    parser.addoption("--stack", default=None,
                     help="Stack to launch as <name>[:<entry>], same syntax "
                          "as `airstack up --stack` (sets AIRSTACK_STACK_DIR/"
                          "AIRSTACK_STACK_ENTRY). Split stacks need the entry "
                          "(e.g. lite_offload_global:onboard). Default: None "
                          "= the default dispatch, stacks/full_default. The "
                          "wiring test drift-checks against "
                          "stacks/<name>/wiring.md.")
    parser.addoption("--fleet", default=None,
                     help="Fleet preset under config/fleets/ (RFC #380 §2), "
                          "e.g. sim_three_mixed. Sets FLEET_CONFIG_FILE for "
                          "airstack up and derives NUM_ROBOTS from the "
                          "fleet's robot count (overriding --num-robots). "
                          "Default: None (legacy --num-robots behavior, "
                          "unchanged).")
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
                          "sweep in test_takeoff_hover_land. Default: 0.5")
    parser.addoption("--trajectory-types", default="Circle,Figure8,Racetrack,Line",
                     help="Comma-separated fixed trajectory types to sweep in "
                          "test_fixed_trajectory. Default: Circle,Figure8,Racetrack,Line")
    parser.addoption("--waypoints", default="30,0,10; 30,30,10; 0,30,10",
                     help="Ordered waypoint route for test_waypoint_flight as "
                          "'x,y,z; x,y,z; ...', relative to the robot pose at "
                          "dispatch (x forward along heading, z up). The route "
                          "must END AWAY from the start: NavigateTask succeeds "
                          "when the robot is within tolerance of the FINAL "
                          "pose, so a closed loop succeeds instantly without "
                          "flying. Legs should be >= 2x --waypoint-tolerance "
                          "or the corridor check cannot discriminate "
                          "route-following from goal-beelining. Default: open "
                          "30 m square (3 corners) climbing 10 m above "
                          "takeoff altitude, so the route clears scene "
                          "clutter (e.g. AirSim Blocks) — this test judges "
                          "route-following, not obstacle avoidance.")
    parser.addoption("--waypoint-tolerance", default="15",
                     help="Pass distance (m) to each intermediate waypoint in "
                          "test_waypoint_flight. Calibrated to stock droan_gl "
                          "plan-following, which trades deviation for path "
                          "progress 1:1 and cuts corners deeply (7-10 m "
                          "observed in Isaac). Default: 15")
    parser.addoption("--goal-tolerance", default="2.5",
                     help="Pass distance (m) to the FINAL waypoint in "
                          "test_waypoint_flight: NavigateTask goal tolerance "
                          "(1.5 m) plus tracking-point lag margin. Default: 2.5")
    parser.addoption("--waypoint-timeout", default="120",
                     help="Per-waypoint time budget (s, odometry clock) in "
                          "test_waypoint_flight. Default: 120")
    parser.addoption("--no-image-build", action="store_true", default=False,
                     help="CI flag: skip image-build in system-tests.yml. "
                          "Ignored by pytest itself.")


def pytest_configure(config):
    run_dir = harness_session.init_run_dir(AIRSTACK_ROOT)
    config.option.xmlpath = str(run_dir / "results.xml")

    # Co-located unit tests import their own package (e.g.
    # `lidar_point_cloud_filter.validation_core`). Put each package/extension import
    # root (the parent of its test/ dir) on sys.path so they resolve without a
    # per-package conftest.py — a second conftest.py collides with this root one as
    # module `conftest` under --import-mode=importlib and breaks `from conftest import`.
    # The test/ dir itself goes on too, for sibling helper modules, but only when it
    # ships no conftest.py — otherwise that file wins the `conftest` name and the
    # collision above is exactly what happens.
    for d in unit_test_dirs():
        roots = [d.parent] if (d / "conftest.py").exists() else [d.parent, d]
        for root in roots:
            if str(root) not in sys.path:
                sys.path.insert(0, str(root))

    # Collect co-located unit tests: their files live outside tests/, so pytest never
    # reaches them by recursion — append the non-linter test files explicitly. Only for
    # a run that means "everything": `pytest tests/system/foo.py` must still narrow.
    # See harness.discovery.collection_is_broad.
    src_name = getattr(getattr(config, "args_source", None), "name", "TESTPATHS")
    config.airstack_unit_tests_injected = src_name != "ARGS" or collection_is_broad(
        config.args, config.invocation_params.dir
    )
    if config.airstack_unit_tests_injected:
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
    harness_session.set_current_item(item)


def pytest_runtest_teardown(item):
    harness_session.set_current_item(None)


@pytest.hookimpl(trylast=True)
def pytest_sessionfinish(session, exitstatus):
    """Persist run outcome metadata and a human-readable summary."""
    run_dir = harness_session.run_dir()
    if run_dir is None:
        return
    try:
        terminal = session.config.pluginmanager.getplugin("terminalreporter")
        reports = [
            report
            for entries in getattr(terminal, "stats", {}).values()
            for report in entries
        ]
        meta_path = write_run_meta(
            run_dir,
            session.items,
            exitstatus,
            session.config.option.markexpr,
            reports,
        )
        logger.info("Wrote run metadata to %s", meta_path)
    except Exception as exc:
        logger.warning("Failed to write run metadata: %s", exc)
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
    fleet = metafunc.config.getoption("--fleet")
    if fleet:
        # A fleet defines its own robot roster: campaigns run at exactly the
        # fleet's robot count — the --num-robots matrix would otherwise spawn
        # campaigns expecting robots the fleet never declares.
        import os as _os
        import yaml as _yaml
        fleet_path = _os.path.join(AIRSTACK_ROOT, "config", "fleets", f"{fleet}.yaml")
        with open(fleet_path, encoding="utf-8") as fh:
            fleet_doc = _yaml.safe_load(fh) or {}
        nums = [len(fleet_doc.get("robots") or {})]
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
    log = f"airstack_env.{_nodeid_dotted(harness_session.current_item().nodeid, with_path_sep=True)}"

    headless = not request.config.getoption("--gui")
    env_overrides = {
        "AUTOLAUNCH": "true",
        "NUM_ROBOTS": str(num_robots),
        # simplesim overrides this: its simple-robot service replaces
        # robot-desktop, so the desktop profile must stay off (see SIM_CONFIG).
        "COMPOSE_PROFILES": cfg.get("compose_profiles", f"desktop,{cfg['profile']}"),
        "MS_AIRSIM_HEADLESS": "true" if headless else "false",
        "ISAAC_SIM_HEADLESS": "true" if headless else "false",
    }
    if headless:
        # Forces rviz/Qt apps to render offscreen instead of spawning windows.
        env_overrides["QT_QPA_PLATFORM"] = "offscreen"
    env_overrides.update(cfg.get("extra_env", {}))

    # Stack dispatch (RFC #379 §3): route robot.launch.xml to the stack's
    # entry launch file (unset = the full_default default). Container path —
    # stacks/ is bind-mounted at /root/AirStack/stacks.
    # Accepts the same <name>[:<entry>] syntax as `airstack up --stack` —
    # split stacks (e.g. lite_offload_global:onboard) have no stack.launch.xml,
    # only per-half entries, so the bare name would dispatch to a nonexistent
    # entry and strand the launch after the dispatcher preamble.
    stack = request.config.getoption("--stack")
    if stack:
        stack_name, _, stack_entry = stack.partition(":")
        env_overrides["AIRSTACK_STACK_DIR"] = f"/root/AirStack/stacks/{stack_name}"
        env_overrides["AIRSTACK_STACK_ENTRY"] = stack_entry or "stack"

    # Fleet dispatch (RFC #380 §2): FLEET_CONFIG_FILE (container path) opts
    # the run into fleet resolution; NUM_ROBOTS is derived from the fleet's
    # robot count (overriding this parametrization's num_robots). `airstack
    # up` sees the env var, validates the fleet, and auto-includes the
    # generated per-robot compose when the fleet is heterogeneous. Isaac runs
    # pin the generic fleet spawner explicitly (parametrized sim scripts in
    # extra_env would otherwise shadow it). --fleet absent = byte-identical
    # legacy behavior.
    fleet = request.config.getoption("--fleet")
    if fleet:
        import yaml as _yaml
        fleet_path = Path(AIRSTACK_ROOT) / "config" / "fleets" / f"{fleet}.yaml"
        assert fleet_path.is_file(), f"--fleet {fleet}: no such file {fleet_path}"
        with fleet_path.open(encoding="utf-8") as f:
            fleet_robots = len((_yaml.safe_load(f) or {}).get("robots") or {})
        env_overrides["FLEET_CONFIG_FILE"] = f"/root/AirStack/config/fleets/{fleet}.yaml"
        env_overrides["NUM_ROBOTS"] = str(fleet_robots)
        num_robots = fleet_robots
        if sim == "isaacsim":
            env_overrides["ISAAC_SIM_SCRIPT_NAME"] = "fleet_spawn.py"

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
        "robot_pattern": cfg.get("robot_pattern", "robot.*desktop"),
        "up_started_at": t0,
        "cfg": cfg,
        # None = the default dispatch (stacks/full_default); else the
        # stacks/<name> explicitly launched (entry suffix stripped — goldens
        # and doctor lookups key on the stack folder, not the entry).
        "stack": stack.partition(":")[0] if stack else None,
        # None = the folder's default entry (stack.launch.xml); else the
        # split-stack half explicitly launched (e.g. "onboard").
        "stack_entry": (stack.partition(":")[2] or None) if stack else None,
        # None = legacy NUM_ROBOTS behavior; else the config/fleets/<name> flown.
        "fleet": fleet,
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
