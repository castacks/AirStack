"""Pure unit tests for RAYFRONTS_MODE=shared support in semantic_search_task
(build plan `_plans/raven_single_rayfronts_shared_plan.md` §2.3, §5 WP-C
item 1).

Covers exactly the three things that are pure in that change:
  * the regex `_filter_raven` uses to recognize raven's per-tick status line
    (now also matching `[LVLM-guided]`),
  * env parsing for `RAYFRONTS_MODE` (`_resolve_rayfronts_mode`),
  * the shared-mode status-gate predicate that replaces counting
    `ms/batch` lines out of a spawned rayfronts' stdout
    (`_rayfronts_shared_ready` / `_rayfronts_shared_frames`).

`node.py` imports rclpy plus geometry_msgs/nav_msgs/sensor_msgs/std_msgs/
action_msgs/task_msgs/airstack_msgs at module level. None of those are
pip-installable outside a sourced ROS 2 workspace — rclpy is a compiled
binding, not a wheel (confirmed: bare `python3 -c "import rclpy"` on this
host fails with ModuleNotFoundError, and `uv run --with rclpy` has nothing
to install). To exercise the PURE functions above on a bare host via
`uv run --with pytest`, this file installs minimal stand-in modules for
every ROS import node.py needs, then loads node.py by file path. It never
instantiates SemanticSearchTaskNode itself — that needs a real rclpy Node
and belongs in the integration tier (see raven_nav's
test/integration/test_node_roundtrip.py, run inside the robot container by
scripts/raven_rayfronts_tests.sh).

Run:
    cd robot/ros_ws/src/global/planners/semantic_search_task
    uv run --with pytest python -m pytest test/test_shared_mode.py -q
"""
import importlib.util
import sys
import types
from pathlib import Path

import pytest

_NODE_PATH = (Path(__file__).resolve().parent.parent
              / 'semantic_search_task' / 'node.py')


class _Dummy:
    """Stand-in for any ROS message/enum/node class. node.py only touches
    these names at class-body/def time when this module is imported (they
    are called for real only inside methods, which these tests never
    invoke) — so a permissive any-attribute, any-call placeholder is enough
    to satisfy the import without ROS actually being installed."""

    def __init__(self, *args, **kwargs):
        pass

    def __call__(self, *args, **kwargs):
        return _Dummy()

    def __getattr__(self, item):
        return _Dummy()


def _stub_module(name: str, **attrs) -> types.ModuleType:
    mod = sys.modules.get(name)
    if mod is None:
        mod = types.ModuleType(name)
        sys.modules[name] = mod
    for k, v in attrs.items():
        setattr(mod, k, v)
    return mod


def _install_ros_stubs() -> None:
    """Install fake rclpy / *_msgs packages so `import node.py` succeeds
    with no ROS workspace sourced. Idempotent: sys.modules entries are
    reused if another test file in the same run already installed them."""
    rclpy_mod = _stub_module(
        'rclpy', ok=lambda: True, init=lambda *a, **kw: None,
        shutdown=lambda *a, **kw: None)
    executors_mod = _stub_module(
        'rclpy.executors', MultiThreadedExecutor=_Dummy,
        SingleThreadedExecutor=_Dummy)
    # `import rclpy.executors` alone does not bind `.executors` onto an
    # already-sys.modules-cached `rclpy` (that binding is normally done by
    # the import machinery's own loader, which we bypass here) — set it
    # explicitly or `rclpy.executors.MultiThreadedExecutor` in main() breaks.
    rclpy_mod.executors = executors_mod
    _stub_module(
        'rclpy.action', ActionServer=_Dummy, ActionClient=_Dummy,
        CancelResponse=type('CancelResponse', (), {'ACCEPT': 1, 'REJECT': 2}),
        GoalResponse=type('GoalResponse', (), {'ACCEPT': 1, 'REJECT': 2}))
    _stub_module('rclpy.callback_groups', ReentrantCallbackGroup=_Dummy)
    _stub_module('rclpy.node', Node=_Dummy)
    _stub_module(
        'rclpy.qos', QoSProfile=_Dummy,
        DurabilityPolicy=type('DurabilityPolicy', (),
                              {'TRANSIENT_LOCAL': 1, 'VOLATILE': 2}),
        HistoryPolicy=type('HistoryPolicy', (), {'KEEP_LAST': 1}),
        ReliabilityPolicy=type('ReliabilityPolicy', (),
                               {'RELIABLE': 1, 'BEST_EFFORT': 2}))
    _stub_module('geometry_msgs')
    _stub_module('geometry_msgs.msg', Point=_Dummy, Polygon=_Dummy,
                 PolygonStamped=_Dummy, Pose=_Dummy, PoseStamped=_Dummy,
                 PoseArray=_Dummy)
    _stub_module('visualization_msgs')
    _stub_module('visualization_msgs.msg',
                 Marker=type('Marker', (),
                             {'ADD': 0, 'DELETE': 2, 'LINE_STRIP': 4}))
    _stub_module('nav_msgs')
    _stub_module('nav_msgs.msg', Odometry=_Dummy, Path=_Dummy)
    _stub_module('sensor_msgs')
    _stub_module('sensor_msgs.msg', PointCloud2=_Dummy)
    _stub_module('sensor_msgs_py')
    _stub_module('sensor_msgs_py.point_cloud2', read_points=lambda *a, **kw: [])
    _stub_module('std_msgs')
    _stub_module('std_msgs.msg', String=_Dummy, Empty=_Dummy)
    _stub_module('action_msgs')
    _stub_module('action_msgs.srv', CancelGoal=_Dummy)
    _stub_module('task_msgs')
    _stub_module('task_msgs.action', SemanticSearchTask=_Dummy, NavigateTask=_Dummy)
    _stub_module('airstack_msgs')
    _stub_module('airstack_msgs.msg', TrajectoryXYZVYaw=_Dummy, WaypointXYZVYaw=_Dummy)


def _load_node_module():
    _install_ros_stubs()
    spec = importlib.util.spec_from_file_location(
        'semantic_search_task_node_under_test', str(_NODE_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def node_mod():
    assert _NODE_PATH.is_file(), f'node.py not found at {_NODE_PATH}'
    return _load_node_module()


# ── _filter_raven regex (gains LVLM-guided) ──────────────────────────────────

def test_filter_raven_passes_lvlm_guided_tag(node_mod):
    line = '[LVLM-guided] steering toward guiding objects'
    assert node_mod._filter_raven(line) == line


@pytest.mark.parametrize('tag', ['Frontier-based', 'Ray-based', 'Voxel-based'])
def test_filter_raven_still_passes_the_original_three_tags(node_mod, tag):
    line = f'[{tag}] some status text'
    assert node_mod._filter_raven(line) == line


def test_filter_raven_tag_match_is_case_sensitive(node_mod):
    # raven's own status line is spelled exactly "[LVLM-guided]" (see the
    # plan §3's behaviour list) — a differently-cased tag must NOT match,
    # matching the original three tags' same case-sensitive behaviour.
    assert node_mod._filter_raven('[lvlm-guided] lowercase tag') is None


def test_filter_raven_still_drops_unmatched_lines(node_mod):
    assert node_mod._filter_raven('just some noisy INFO line') is None


def test_filter_raven_still_handles_errors_and_lifecycle_lines(node_mod):
    # Regression: nothing else about _filter_raven's behaviour changed.
    assert node_mod._filter_raven('Traceback (most recent call last):') == \
        'ERROR: Traceback (most recent call last):'
    assert node_mod._filter_raven('raven_nav started, ready') == 'raven_nav started'
    assert node_mod._filter_raven('Waiting for odometry...') == 'Waiting for odometry...'


# ── RAYFRONTS_MODE env parsing ───────────────────────────────────────────────

@pytest.mark.parametrize('raw,expected', [
    (None, 'per_robot'),
    ('', 'per_robot'),
    ('per_robot', 'per_robot'),
    ('PER_ROBOT', 'per_robot'),
    ('  per_robot  ', 'per_robot'),
    ('shared', 'shared'),
    ('SHARED', 'shared'),
    ('  Shared  ', 'shared'),
    ('bogus', 'per_robot'),          # unrecognized -> falls back, never crashes
    ('shared_but_typo', 'per_robot'),
])
def test_resolve_rayfronts_mode(node_mod, raw, expected):
    assert node_mod._resolve_rayfronts_mode(raw) == expected


def test_resolve_rayfronts_mode_matches_os_getenv_default_semantics(node_mod, monkeypatch):
    # __init__ calls _resolve_rayfronts_mode(os.getenv('RAYFRONTS_MODE')) —
    # confirm the unset-env-var path (os.getenv returns None) round-trips to
    # the documented default, i.e. RAYFRONTS_MODE=per_robot is byte-for-byte
    # today's behaviour whether the var is unset or explicitly 'per_robot'.
    monkeypatch.delenv('RAYFRONTS_MODE', raising=False)
    import os
    assert node_mod._resolve_rayfronts_mode(os.getenv('RAYFRONTS_MODE')) == 'per_robot'
    monkeypatch.setenv('RAYFRONTS_MODE', 'shared')
    assert node_mod._resolve_rayfronts_mode(os.getenv('RAYFRONTS_MODE')) == 'shared'


# ── shared-mode status-gate predicate ────────────────────────────────────────

def test_shared_ready_true_when_anchored_and_enough_frames(node_mod):
    assert node_mod._rayfronts_shared_ready(
        {'anchored': True, 'frames_robot': 8}, 8) is True


def test_shared_ready_false_when_anchored_but_not_enough_frames(node_mod):
    assert node_mod._rayfronts_shared_ready(
        {'anchored': True, 'frames_robot': 7}, 8) is False


def test_shared_ready_false_when_frames_ok_but_not_anchored(node_mod):
    # This is the case the plan calls out explicitly: "wait until anchored
    # AND frames_robot >= required_batches" — plenty of frames from a server
    # that hasn't anchored THIS robot yet must not satisfy the gate.
    assert node_mod._rayfronts_shared_ready(
        {'anchored': False, 'frames_robot': 999}, 8) is False


def test_shared_ready_false_on_missing_keys(node_mod):
    assert node_mod._rayfronts_shared_ready({}, 8) is False


def test_shared_ready_false_on_non_dict_status(node_mod):
    assert node_mod._rayfronts_shared_ready(None, 8) is False
    assert node_mod._rayfronts_shared_ready('not a dict', 8) is False
    assert node_mod._rayfronts_shared_ready(['nope'], 8) is False


def test_shared_ready_coerces_numeric_string_frames(node_mod):
    # json.loads gives real ints, but keep this robust to a hand-published
    # status message (e.g. from `ros2 topic pub` while debugging).
    assert node_mod._rayfronts_shared_ready(
        {'anchored': True, 'frames_robot': '8'}, 8) is True


def test_shared_ready_false_on_malformed_frames(node_mod):
    assert node_mod._rayfronts_shared_ready(
        {'anchored': True, 'frames_robot': 'not-a-number'}, 8) is False


def test_shared_ready_boundary_required_batches_zero(node_mod):
    assert node_mod._rayfronts_shared_ready(
        {'anchored': True, 'frames_robot': 0}, 0) is True


def test_shared_frames_reads_int(node_mod):
    assert node_mod._rayfronts_shared_frames({'frames_robot': 5}) == 5
    assert node_mod._rayfronts_shared_frames({'frames_robot': '5'}) == 5


@pytest.mark.parametrize('status', [{}, {'frames_robot': None},
                                    {'frames_robot': 'nope'}, None, 'x'])
def test_shared_frames_defaults_to_zero_on_malformed_input(node_mod, status):
    assert node_mod._rayfronts_shared_frames(status) == 0


# ── wiring regression guard (static — no rclpy needed) ───────────────────────

def test_execute_shared_branch_gates_on_the_pure_predicate():
    """The shared-mode wait loop in _execute must exit on
    _rayfronts_shared_ready(shared_status, required_batches) — i.e. anchored
    AND frames_robot >= required_batches — never on a bare frame count. This
    greps the source rather than driving _execute itself, which needs a live
    rclpy action goal_handle/executor (see raven_nav's
    test/integration/test_node_roundtrip.py for that tier)."""
    src = _NODE_PATH.read_text()
    assert '_rayfronts_shared_ready(shared_status, required_batches)' in src
    # And _cleanup_existing must not pkill the shared server.
    assert "if self._rayfronts_mode != 'shared':" in src
