"""Test collection ordering.

Cross-module order (unit → docker/package builds → integration → sim tiers), the
per-module phase chains for the autonomy flight tests, and a readable rewrite of the
parametrize ids. conftest's ``pytest_collection_modifyitems`` hook delegates to
``modify_items``.
"""
import pytest

from harness.discovery import _is_unit_item

# Run cheap/fast-fail tests first so real problems surface early:
# docker image builds → colcon workspace builds → liveliness (infra) → sensors
# (ROS topic streams) → autonomy flight tests.
_MODULE_ORDER = [
    # Unit tests first — fast, hermetic, no Docker.  Co-located package unit tests
    # (see unit_test_files) sort into this leading slot via the path check below.
    "__unit__",
    # Harness contract tests: hermetic, and they guard the collection of everything
    # above, so they belong with the fast tier rather than after the sim suites.
    "test_collection_contract",
    "test_metrics_reporting_contract",
    # System tests follow in dependency order.
    "system.test_build_docker",
    "system.test_build_packages",
    # Integration tests (tests/integration/) need the robot-desktop image + colcon
    # build, so they run after build_packages and before the sim tiers.
    "__integration__",
    "system.test_liveliness",
    # simple-sim smoke test: own mark (simple_sim), never part of the
    # isaac/airsim campaigns (not in run_meta.SIMULATION_MODULES).
    "system.test_simple_sim",
    "system.test_wiring_snapshot",
    "system.test_sensors",
    "system.test_takeoff_hover_land",
    "system.test_fixed_trajectory",
]

# Within test_takeoff_hover_land, each (env, velocity) runs phases in this chain order.
_AUTONOMY_PHASE_ORDER = [
    "test_px4_ready",
    "test_takeoff",
    "test_hover",
    "test_landing",
]

# Within test_fixed_trajectory, each (env, trajectory_type) runs phases in this order.
_FIXED_TRAJ_PHASE_ORDER = [
    "test_px4_ready",
    "test_takeoff",
    "test_fixed_trajectory",
    "test_landing",
]

# Maps module name → phase order list for per-module chain sorting.
_MODULE_PHASE_ORDERS = {
    "system.test_takeoff_hover_land": _AUTONOMY_PHASE_ORDER,
    "system.test_fixed_trajectory": _FIXED_TRAJ_PHASE_ORDER,
}


def _rank(name, order):
    """Index of `name` in `order`; `len(order)` if unknown (i.e., sort last)."""
    return order.index(name) if name in order else len(order)


def _module_key(item):
    """Return the ordering key for an item.

    Co-located unit tests (collected from package ``test/`` dirs outside ``tests/``)
    are identified by path. Integration tests live under ``integration/``.
    Everything else uses the dotted module ``__name__`` against ``_MODULE_ORDER``.
    """
    if _is_unit_item(item):
        return _rank("__unit__", _MODULE_ORDER)
    if item.nodeid.startswith("integration/"):
        return _rank("__integration__", _MODULE_ORDER)
    return _rank(getattr(item.module, "__name__", ""), _MODULE_ORDER)


def _apply_simplesim_guard(items):
    """Skip mismatched (test, sim-target) pairs involving simplesim.

    The `simple_sim` smoke test gates only on what simple-sim provides, and the
    isaac/airsim campaign tests gate on PX4/MAVROS that simple-sim never runs.
    Skipping at collection time (rather than inside a test) prevents the
    class-scoped airstack_env fixture from bringing up a stack that the test
    would immediately skip. isaac/airsim campaigns are untouched: without a
    simplesim target or a simple_sim item nothing here fires.
    """
    for item in items:
        cs = getattr(item, "callspec", None)
        env = cs.params.get("airstack_env") if cs else None
        if not env:
            continue
        sim, num_robots, _ = env
        is_simple_test = item.get_closest_marker("simple_sim") is not None
        if is_simple_test and sim != "simplesim":
            item.add_marker(pytest.mark.skip(
                reason="simple_sim smoke test runs only with --sim simplesim"))
        elif is_simple_test and num_robots != 1:
            item.add_marker(pytest.mark.skip(
                reason="simple-sim is single-robot (topics hardcoded to "
                       "robot_1); use --num-robots 1"))
        elif not is_simple_test and sim == "simplesim":
            item.add_marker(pytest.mark.skip(
                reason="--sim simplesim only drives the simple_sim smoke test "
                       "(-m simple_sim)"))


def modify_items(items):
    # 0. simplesim pairing guard (skip markers only; no reordering).
    _apply_simplesim_guard(items)

    # 1. Cross-module: enforce `_MODULE_ORDER`. Stable sort keeps within-module
    #    order intact, so pytest's default file/class order survives.
    items.sort(key=_module_key)

    # 2. Within each parametrized autonomy-style module, sort by
    #    (airstack_env, secondary_param, phase) so each env brings up the stack
    #    once and the drone goes ground→air→ground per secondary parameter.
    for mod_name, phase_order in _MODULE_PHASE_ORDERS.items():
        def _phase(item, _order=phase_order, _mod=mod_name):
            if getattr(item.module, "__name__", "") != _mod:
                return None
            name = item.originalname or item.name.split("[", 1)[0]
            return _rank(name, _order)

        def _sort_key(item, _mod=mod_name):
            cs = getattr(item, "callspec", None)
            env = cs.params.get("airstack_env", ()) if cs else ()
            # test_takeoff_hover_land sweeps velocity; test_fixed_trajectory sweeps type
            secondary = (
                float(cs.params["velocity"]) if cs and "velocity" in cs.params
                else (cs.params.get("trajectory_type", "") if cs else "")
            )
            return (env, secondary, _phase(item))

        slots = [(i, it) for i, it in enumerate(items) if _phase(it) is not None]
        if slots:
            sorted_items = sorted((it for _, it in slots), key=_sort_key)
            for (i, _), new_item in zip(slots, sorted_items):
                items[i] = new_item

    # 3. Rewrite bracketed test IDs into a consistent hierarchy:
    #    sim > robots > secondary param > iteration.
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
        if "trajectory_type" in cs.params:
            parts.append(f"traj{cs.params['trajectory_type']}")
        if env:
            parts.append(f"iter{i}")
        if not parts:
            continue
        new_id = "-".join(parts)
        if cs.id == new_id:
            continue
        item.name = item.name.replace(f"[{cs.id}]", f"[{new_id}]")
        item._nodeid = item._nodeid.replace(f"[{cs.id}]", f"[{new_id}]")
