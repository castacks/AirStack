"""The declared-parameter table, nav-mode vocabulary and log-line formats.

Pure: no rclpy, no numpy. `raven_nav_node` walks `PARAM_TABLE` to declare every
parameter, and `test_param_contract.py` / `test_log_contract.py` assert the
external contract against this module without needing ROS.

Two rules the table encodes:

* every parameter `semantic_search_task` can pass on the `ros2 run` command
  line must be DECLARED, or rclpy aborts the node at startup with
  `ParameterNotDeclaredException`. That is why the multi-robot and baseline
  knobs are still here after the coordination code was deleted;
* the ones the single-agent RAVEN logic no longer has anything to do with are
  marked `active=False`: the node logs them once, as ignored, and moves on.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, List, Optional


@dataclass(frozen=True)
class Param:
    name: str
    default: Any
    active: bool = True
    auto: bool = False      # declared by rclpy itself (use_sim_time)
    doc: str = ''


# NOTE on defaults: `semantic_search_task` spawns raven_nav with `-p` flags and
# NO --params-file, so `config/raven_nav.yaml` is never read on the mission
# path and these defaults are what actually run.
PARAM_TABLE: List[Param] = [
    # ── mission inputs ──────────────────────────────────────────────────────
    Param('query_labels', ['red building', 'water tower', 'radio tower'],
          doc='every rayfronts query column, in registration order'),
    Param('target_labels', [''],
          doc='subset of query_labels to navigate toward; empty = all'),
    Param('min_altitude_agl', 1.5,
          doc='OG frontier_behavior.py:33 used the literal 1.5 m here'),
    Param('max_altitude_agl', 100.0,
          doc='deviation 1: every waypoint z is clamped into this band'),
    Param('use_sim_time', True, auto=True,
          doc='declared by rclpy; listed so the contract test sees it'),

    # ── behaviour thresholds (OG values) ────────────────────────────────────
    Param('score_threshold', 0.95,
          doc='OG ray_behavior.py:42'),
    Param('voxel_score_threshold', 0.98,
          doc='OG voxel_behavior.py:51'),
    Param('voxel_min_cluster_size', 30,
          doc='OG voxel_behavior.py:78'),
    Param('voxel_min_confidence', 0.0,
          doc='drop reported AABBs below this mean score; 0 = report all'),

    # ── LVLM-guided behaviour (new) ─────────────────────────────────────────
    Param('lvlm_enabled', None,
          doc="None => env RAVEN_LVLM, default true"),
    Param('lvlm_request_interval_s', None,
          doc='None => env RAVEN_LVLM_INTERVAL_S, default 30.0 '
              '(OG LVLM/internvl3.py:35)'),
    Param('lvlm_ray_threshold', None,
          doc='None => env RAVEN_LVLM_RAY_THRESHOLD, default 0.9 '
              '(OG lvlm_behavior.py:78). Tunable at run time because the gate '
              'is a SOFTMAX over the whole query set: a 33-label background '
              'vocabulary dilutes every column, and 0.9 becomes unreachable.'),
    Param('lvlm_vlm_url', '',
          doc="'' => env VLM_URL, then OPENAI_BASE_URL, then "
              'http://offboard-compute:8000/v1'),
    Param('lvlm_vlm_model', '',
          doc="'' => env CONAVGPT2_VLM_MODEL, else the first served model"),
    Param('lvlm_image_topic', '',
          doc="'' => /{robot}/sensors/front_stereo/left/image_rect"),

    # ── coverage / completion ───────────────────────────────────────────────
    Param('coverage_complete_threshold', 0.90),
    Param('coverage_cell_size_m', 0.5),
    Param('coverage_raycast_range_m', 30.0),
    Param('coverage_raycast_min_step_m', 5.0),

    # ── output ──────────────────────────────────────────────────────────────
    Param('results_dir', ''),
    Param('results_dump_period_s', 3.0),
    Param('timer_period', 0.5),
    Param('nav_output_enabled', True),
    Param('debug_coordination', True,
          doc='kept: gates the per-tick [coverage]/[event] chatter'),
    Param('debug_ray_table', True),
    Param('debug_table_max_rows', 30),
    Param('debug_table_period_sec', 5.0),

    # ── the one baseline that survives ──────────────────────────────────────
    Param('frontier_only_baseline', False,
          doc='pure frontier navigation; passive voxel detection stays on'),

    # ── accepted but inert (multi-robot / other baselines were removed) ─────
    Param('vlfm_baseline', False, active=False),
    Param('vlfm_use_voxel_targets', False, active=False),
    Param('vlfm_value_weight', 300.0, active=False),
    Param('vlfm_ray_blacklist', False, active=False),
    Param('conavgpt_baseline', False, active=False),
    Param('conavgpt_leader_id', 1, active=False),
    Param('conavgpt_round_period_s', 30.0, active=False),
    Param('conavgpt_max_regions', 12, active=False),
    Param('conavgpt_assignment_ttl_s', 90.0, active=False),
    Param('conavgpt_replan_on_reach_m', 8.0, active=False),
    Param('bundle_len', 1, active=False),
    Param('voxel_confirm_hits', 2, active=False),
    Param('voxel_track_max_misses', 4, active=False),
    Param('voxel_proximity_engage_m', 12.0, active=False),
    Param('ray_confirm_hits', 1, active=False),
    Param('ray_track_max_misses', 4, active=False),
    Param('commit_swap_improvement_frac', 0.25, active=False),
    Param('commit_min_hold_s', 4.0, active=False),
    Param('commit_radius_m', 3.0, active=False),
    Param('ray_reach_factor', 3.0, active=False),
    Param('target_behind_penalty_weight', 8.0, active=False),
    Param('commit_switch_margin_m', 8.0, active=False),
    Param('debug_auction', True, active=False),
    Param('bb_release_timeout_s', 8.0, active=False),
]

PARAM_NAMES = [p.name for p in PARAM_TABLE]
INERT_PARAMS = [p.name for p in PARAM_TABLE if not p.active]


def param(name: str) -> Param:
    for p in PARAM_TABLE:
        if p.name == name:
            return p
    raise KeyError(name)


# ── navigation_mode vocabulary ──────────────────────────────────────────────
# semantic_search_task ends the mission on the literal string 'complete'.
NAV_MODE_TAG = {
    'Frontier-based': 'frontier',
    'Ray-based':      'ray',
    'Voxel-based':    'voxel',
    'LVLM-guided':    'lvlm',
    'Complete':       'complete',
}
NAV_MODE_IDLE = 'idle'

# The bracketed tags `semantic_search_task._filter_raven` greps for.
STATUS_TAGS = tuple(NAV_MODE_TAG.keys() - {'Complete'})

# Literal log substrings the same filter matches. Changing any of these blanks
# the operator-facing action feedback.
LOG_STARTED = 'raven_nav started'
LOG_WAITING_ODOM = 'waiting for odometry...'
LOG_BOOT_GPS = 'boot GPS captured'
LOG_SEARCH_AREA = 'search_area'


def format_status_line(behavior_mode: str, current_target: str = '',
                       completed=()) -> str:
    """The per-tick status line. Must start with the bracketed mode tag."""
    parts = [f'[{behavior_mode}]']
    if current_target:
        parts.append(f'target={current_target}')
    completed = list(completed)
    if completed:
        parts.append(f'completed={completed}')
    return ' '.join(parts)


# ── env helpers (kept pure so the tests can exercise them) ──────────────────
_TRUE = {'1', 'true', 'yes', 'on', 'y', 't'}
_FALSE = {'0', 'false', 'no', 'off', 'n', 'f'}


def env_bool(name: str, default: bool, environ=None) -> bool:
    env = os.environ if environ is None else environ
    raw = str(env.get(name, '')).strip().lower()
    if raw in _TRUE:
        return True
    if raw in _FALSE:
        return False
    return bool(default)


def env_float(name: str, default: float, environ=None, warn=None) -> float:
    """Env var as a float. Unset or garbage -> `default` (and one warning)."""
    env = os.environ if environ is None else environ
    raw = str(env.get(name, '')).strip()
    if not raw:
        return float(default)
    try:
        return float(raw)
    except ValueError:
        if warn is not None:
            warn(f'[params] {name}={raw!r} is not a number — using {default}')
        return float(default)


def resolve_lvlm_enabled(param_value: Optional[bool], environ=None) -> bool:
    """`lvlm_enabled` unset (None) falls back to env RAVEN_LVLM, default true."""
    if param_value is not None:
        return bool(param_value)
    return env_bool('RAVEN_LVLM', True, environ=environ)


# The OG literals, and the floors of the two env fallbacks below.
LVLM_INTERVAL_DEFAULT_S = 30.0      # OG LVLM/internvl3.py:35
LVLM_RAY_THRESHOLD_DEFAULT = 0.9    # OG lvlm_behavior.py:78


def resolve_lvlm_interval_s(param_value: Optional[float], environ=None,
                            warn=None) -> float:
    """`-p lvlm_request_interval_s` wins; otherwise env RAVEN_LVLM_INTERVAL_S,
    otherwise the OG 30 s."""
    if param_value is not None:
        return float(param_value)
    return env_float('RAVEN_LVLM_INTERVAL_S', LVLM_INTERVAL_DEFAULT_S,
                     environ=environ, warn=warn)


def resolve_lvlm_ray_threshold(param_value: Optional[float], environ=None,
                               warn=None) -> float:
    """`-p lvlm_ray_threshold` wins; otherwise env RAVEN_LVLM_RAY_THRESHOLD,
    otherwise the OG 0.9.

    Worth an env knob because the score is a softmax over the ENTIRE query set:
    with the target labels plus a ~33-label background vocabulary, no column
    reaches 0.9 and the behaviour can never fire, so the operator needs to move
    the gate without touching the mission goal.
    """
    if param_value is not None:
        return float(param_value)
    return env_float('RAVEN_LVLM_RAY_THRESHOLD', LVLM_RAY_THRESHOLD_DEFAULT,
                     environ=environ, warn=warn)


def resolve_image_topic(param_value: str, robot_name: str) -> str:
    return (str(param_value)
            or f'/{robot_name}/sensors/front_stereo/left/image_rect')
