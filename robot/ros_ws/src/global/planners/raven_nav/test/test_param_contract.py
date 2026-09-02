"""Every parameter semantic_search_task can pass must be DECLARED.

rclpy aborts a node with `ParameterNotDeclaredException` when `-p name:=v`
names a parameter it never declared, so this is a hard startup contract — and
`semantic_search_task` spawns raven_nav with `-p` flags and no `--params-file`,
which also makes `raven_nav/params.py`'s defaults the ones that actually run.

The list is checked two ways: against the frozen contract (§2.1 of the build
plan) and against the live `semantic_search_task/node.py` source, so a new
`-p` flag added there fails here instead of at 3 a.m. in the container.
"""
import ast
import pathlib
import re

import pytest

from raven_nav import params as P

HERE = pathlib.Path(__file__).resolve().parent
PLANNERS = HERE.parents[1]
SST_NODE = PLANNERS / 'semantic_search_task' / 'semantic_search_task' / 'node.py'

# Frozen contract, §2.1 of _plans/raven_single_rayfronts_shared_plan.md.
CONTRACT_PARAMS = [
    'query_labels', 'target_labels', 'min_altitude_agl', 'max_altitude_agl',
    'use_sim_time', 'frontier_only_baseline', 'vlfm_baseline',
    'vlfm_use_voxel_targets', 'vlfm_value_weight', 'vlfm_ray_blacklist',
    'conavgpt_baseline', 'conavgpt_leader_id', 'conavgpt_round_period_s',
    'conavgpt_max_regions', 'conavgpt_assignment_ttl_s',
    'conavgpt_replan_on_reach_m', 'coverage_complete_threshold', 'results_dir',
    'score_threshold', 'voxel_score_threshold', 'voxel_min_confidence',
    'voxel_min_cluster_size', 'bundle_len', 'timer_period',
    'nav_output_enabled', 'results_dump_period_s', 'coverage_cell_size_m',
    'coverage_raycast_range_m', 'coverage_raycast_min_step_m',
    'voxel_confirm_hits', 'voxel_track_max_misses', 'voxel_proximity_engage_m',
    'ray_confirm_hits', 'ray_track_max_misses', 'commit_swap_improvement_frac',
    'commit_min_hold_s', 'commit_radius_m', 'ray_reach_factor',
    'target_behind_penalty_weight', 'commit_switch_margin_m', 'debug_auction',
    'debug_coordination', 'debug_ray_table', 'debug_table_max_rows',
    'debug_table_period_sec', 'bb_release_timeout_s',
]

NEW_PARAMS = ['lvlm_enabled', 'lvlm_request_interval_s', 'lvlm_ray_threshold',
              'lvlm_vlm_url', 'lvlm_vlm_model', 'lvlm_image_topic']


@pytest.mark.parametrize('name', CONTRACT_PARAMS)
def test_contract_parameter_is_declared(name):
    assert name in P.PARAM_NAMES


@pytest.mark.parametrize('name', NEW_PARAMS)
def test_new_lvlm_parameter_is_declared(name):
    assert name in P.PARAM_NAMES


def test_no_duplicate_declarations():
    assert len(P.PARAM_NAMES) == len(set(P.PARAM_NAMES))


def test_table_is_exactly_the_contract_plus_the_new_lvlm_knobs():
    assert set(P.PARAM_NAMES) == set(CONTRACT_PARAMS) | set(NEW_PARAMS)


def test_use_sim_time_is_not_declared_twice():
    """rclpy declares it itself; declaring it again raises
    ParameterAlreadyDeclaredException at startup."""
    assert P.param('use_sim_time').auto is True
    assert [p.name for p in P.PARAM_TABLE if p.auto] == ['use_sim_time']


@pytest.mark.skipif(not SST_NODE.exists(), reason='semantic_search_task absent')
def test_every_p_flag_in_semantic_search_task_is_declared():
    """Scrape `-p name:=...` out of the spawner and check each name.

    The Co-NavGPT sidecar and the LVLM baseline are separate executables; only
    the raven_nav argv matters, but every name that appears anywhere in that
    file is either a raven_nav parameter or one of those two, so the check is
    against the union and only fails on a genuinely new raven flag."""
    src = SST_NODE.read_text()
    names = set(re.findall(r"'?-p'?,\s*\(?\s*f?'([a-z_0-9]+):=", src))
    # names only the sidecar processes take
    other_executables = {'target_objects'}
    missing = sorted(names - other_executables - set(P.PARAM_NAMES))
    assert not missing, f'semantic_search_task passes undeclared params: {missing}'


@pytest.mark.skipif(not SST_NODE.exists(), reason='semantic_search_task absent')
def test_goal_override_parameters_are_declared():
    """The `(name, goal_value, sentinel)` override loop."""
    src = SST_NODE.read_text()
    block = src[src.index('for pname, gval, sentinel in ('):]
    block = block[:block.index(')\n            ):') + 3] if ')\n            ):' in block else block[:2000]
    names = set(re.findall(r"\('([a-z_0-9]+)',\s*(?:int\()?goal", block))
    assert names, 'the override loop moved — update this test'
    assert names <= set(P.PARAM_NAMES)


# ── which are live, which are accepted-but-inert ────────────────────────────
def test_inert_set_is_exactly_the_removed_coordination_and_baseline_knobs():
    assert set(P.INERT_PARAMS) == {
        'vlfm_baseline', 'vlfm_use_voxel_targets', 'vlfm_value_weight',
        'vlfm_ray_blacklist', 'conavgpt_baseline', 'conavgpt_leader_id',
        'conavgpt_round_period_s', 'conavgpt_max_regions',
        'conavgpt_assignment_ttl_s', 'conavgpt_replan_on_reach_m',
        'bundle_len', 'voxel_confirm_hits', 'voxel_track_max_misses',
        'voxel_proximity_engage_m', 'ray_confirm_hits', 'ray_track_max_misses',
        'commit_swap_improvement_frac', 'commit_min_hold_s', 'commit_radius_m',
        'ray_reach_factor', 'target_behind_penalty_weight',
        'commit_switch_margin_m', 'debug_auction', 'bb_release_timeout_s',
    }


def test_frontier_only_baseline_stays_live():
    assert P.param('frontier_only_baseline').active is True


def test_defaults_are_the_original_raven_values():
    assert P.param('score_threshold').default == 0.95        # OG ray:42
    assert P.param('voxel_score_threshold').default == 0.98  # OG voxel:51
    assert P.param('voxel_min_cluster_size').default == 30   # OG voxel:78
    assert P.param('min_altitude_agl').default == 1.5        # OG frontier:33
    # These two are env-resolved (default None in the table); the OG literals
    # are the floor of the resolver.
    assert P.LVLM_INTERVAL_DEFAULT_S == 30.0                 # OG internvl3:35
    assert P.LVLM_RAY_THRESHOLD_DEFAULT == 0.9               # OG lvlm:78
    assert P.resolve_lvlm_interval_s(None, environ={}) == 30.0
    assert P.resolve_lvlm_ray_threshold(None, environ={}) == 0.9


def test_env_resolved_params_declare_no_literal_default():
    """A literal in the table would silently beat the env var, because the node
    only consults the resolver for parameters whose table default is None."""
    for name in ('lvlm_enabled', 'lvlm_request_interval_s',
                 'lvlm_ray_threshold'):
        assert P.param(name).default is None, name


def test_every_param_carries_a_json_safe_default():
    for p in P.PARAM_TABLE:
        assert isinstance(p.default, (bool, int, float, str, list, type(None))), \
            p.name


# ── env resolution for the new knobs ────────────────────────────────────────
def test_lvlm_enabled_defaults_true_and_follows_raven_lvlm():
    assert P.resolve_lvlm_enabled(None, environ={}) is True
    assert P.resolve_lvlm_enabled(None, environ={'RAVEN_LVLM': 'false'}) is False
    assert P.resolve_lvlm_enabled(None, environ={'RAVEN_LVLM': '0'}) is False
    assert P.resolve_lvlm_enabled(None, environ={'RAVEN_LVLM': 'yes'}) is True
    # an explicit parameter always wins
    assert P.resolve_lvlm_enabled(False, environ={'RAVEN_LVLM': 'true'}) is False


def test_lvlm_ray_threshold_follows_its_env_var():
    """The gate is a softmax over the WHOLE query set, so a long background
    vocabulary dilutes every column and the OG 0.9 becomes unreachable — the
    operator has to be able to move it without touching the mission goal."""
    assert P.resolve_lvlm_ray_threshold(None, environ={}) == 0.9
    assert P.resolve_lvlm_ray_threshold(
        None, environ={'RAVEN_LVLM_RAY_THRESHOLD': '0.35'}) == 0.35
    # an explicit -p always wins
    assert P.resolve_lvlm_ray_threshold(
        0.5, environ={'RAVEN_LVLM_RAY_THRESHOLD': '0.35'}) == 0.5


def test_lvlm_interval_follows_its_env_var():
    assert P.resolve_lvlm_interval_s(None, environ={}) == 30.0
    assert P.resolve_lvlm_interval_s(
        None, environ={'RAVEN_LVLM_INTERVAL_S': '5'}) == 5.0
    assert P.resolve_lvlm_interval_s(
        12.0, environ={'RAVEN_LVLM_INTERVAL_S': '5'}) == 12.0


@pytest.mark.parametrize('resolver,var,default', [
    ('resolve_lvlm_ray_threshold', 'RAVEN_LVLM_RAY_THRESHOLD', 0.9),
    ('resolve_lvlm_interval_s', 'RAVEN_LVLM_INTERVAL_S', 30.0),
])
def test_garbage_env_value_warns_once_and_falls_back(resolver, var, default):
    warnings = []
    got = getattr(P, resolver)(None, environ={var: 'not-a-number'},
                               warn=warnings.append)
    assert got == default
    assert len(warnings) == 1
    assert var in warnings[0] and str(default) in warnings[0]


@pytest.mark.parametrize('raw,want', [
    ('', 30.0), ('  ', 30.0), ('0', 0.0), ('1e2', 100.0), ('-5', -5.0),
])
def test_env_float_parsing(raw, want):
    assert P.env_float('X', 30.0, environ={'X': raw}) == want


def test_env_float_does_not_warn_when_unset():
    warnings = []
    assert P.env_float('X', 7.0, environ={}, warn=warnings.append) == 7.0
    assert warnings == []


def test_image_topic_default_is_the_zed_left_rect():
    assert P.resolve_image_topic('', 'robot_2') == \
        '/robot_2/sensors/front_stereo/left/image_rect'
    assert P.resolve_image_topic('/other', 'robot_2') == '/other'


# ── the node really does declare them all ───────────────────────────────────
def test_node_declares_from_the_table_not_by_hand():
    """A hand-written declare_parameter in the node would drift from the table
    the contract tests check, so there must be exactly one, in the loop."""
    src = (HERE.parent / 'raven_nav' / 'raven_nav_node.py').read_text()
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'declare_parameter']
    assert len(calls) == 1, 'declare_parameter must only be called from the loop'
    assert 'for spec in P.PARAM_TABLE' in src
