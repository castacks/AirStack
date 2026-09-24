"""Offline unit tests for the `gpt` (Co-NavGPT2) nav_mode of the shared search
planner, now that the frontier source is the 3D voxel map.

NO Isaac Sim, NO GPU, NO ROS, NO VLM. Pure python + numpy + scipy.

THE CLAIM UNDER TEST. The conavgpt2 arm is offered the SAME candidate list as
vlfm / frontier — `_voxel_frontiers` -> `target_point_list` — and differs only
in selection: one top-down BEV per candidate, numbered, shown to a VLM that
answers `{"robot_i": "frontier_k"}`, and `target_point_list[k]` becomes the
goal for a whole round. For that to be true with 3D frontiers, three things
that were NOT true before have to hold:

  1. the launch actually selects `frontier_source: voxel3d` for this arm
     (paper.yaml never did, so the arm flew upstream's 2D slab);
  2. the label the VLM's `frontier_k` refers to is drawn on image k — the
     candidate images are cut from `target_edge_map` BY LABEL, and two
     stratified candidates over the same ground used to overwrite each other
     so "frontier N" was a blank map;
  3. the source-level search-area filter is applied in the MAP frame like the
     pick-side one, because this arm has no pick-side filter to fall back on.

HOW THE PLANNER'S OWN CODE IS REACHED. As in test_frontier_baseline.py:
`planner_node.py` imports rclpy / cv_bridge / open3d at module scope, so its
methods are sliced out of the source and exec'd onto a stub class. The REAL
`_voxel_frontiers`, `_rank_frontiers`, `_assign`, `_record_round`,
`_resolve_frontier`, `_fallback_goal`, `_recover_into_area`,
`_frontier_altitude` and `_waypoint_z` run here. The vendored
`chat_utils.get_all_candidate_maps` — the loop that cuts one image per label
— is ALSO sliced from the conavgpt2 package and run for real, with cv2's
dilate and the PIL renderer replaced by numpy stand-ins that count red
pixels. Only the network call (`chat_with_gpt4v`) is a fake, returning a
scripted assignment.

Run:  python3 tests/test_conavgpt2_baseline.py
  or: python3 -m pytest tests/test_conavgpt2_baseline.py -s
"""

import io
import json
import math
import os
import re
import sys
import tempfile
import threading
import traceback
import types

import numpy as np
from scipy import ndimage

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from search_baselines import sector as sect          # noqa: E402
from search_baselines.voxel_map import VoxelMap, OCCUPIED as VOX_OCCUPIED  # noqa: E402

_PLANNER_PY = os.path.join(_PKG_ROOT, 'search_baselines', 'planner_node.py')
_AGENT_PY = os.path.join(_PKG_ROOT, 'search_baselines', 'airstack_agent.py')
_CONFIG = os.path.join(_PKG_ROOT, 'config')
_LAUNCH = os.path.join(_PKG_ROOT, 'launch')
# The vendored upstream library is a sibling package.
_VENDOR_CHAT = os.path.normpath(os.path.join(
    _PKG_ROOT, '..', 'conavgpt2', 'conavgpt2', 'vendor', 'utils', 'chat_utils.py'))

with open(_PLANNER_PY, encoding='utf-8') as _f:
    SRC = _f.read()
with open(_AGENT_PY, encoding='utf-8') as _f:
    AGENT_SRC = _f.read()
with open(_VENDOR_CHAT, encoding='utf-8') as _f:
    CHAT_SRC = _f.read()

SCRATCH = tempfile.mkdtemp(prefix='conavgpt2_test_')

# Findings that are not test failures: real, reproducible, worth an operator's
# attention. Printed as a block at the end of the run.
NOTES = []


def note(tag, msg):
    NOTES.append((tag, msg))


# ── the slicing harness ──────────────────────────────────────────────────────

def _slice(src, start, end):
    """Source between two markers, `start` included and `end` excluded."""
    i = src.index(start)
    j = src.index(end, i + len(start))
    return src[i:j]


class _Clock:
    """Stands in for the `time` module inside the sliced code."""

    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


CLOCK = _Clock()


class _RosClock:
    """`self.get_clock().now().nanoseconds`, driven by the same clock."""

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(CLOCK.time() * 1e9))


class _String:
    """std_msgs/String, as much of it as _record_round touches."""

    def __init__(self):
        self.data = ''


class FakePub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


# ── the REAL vendored candidate-image loop ───────────────────────────────────
#
# `get_all_candidate_maps` is what turns the edge map into the numbered images
# the VLM is shown. It needs cv2 (a 5x5 dilate) and the PIL renderer that
# stamps the number and the robot markers. Both are replaced by numpy stand-ins
# that preserve the one property under test: how many RED pixels — the
# frontier — image k ends up with.

class _FakeCv2:
    MORPH_RECT = 0

    @staticmethod
    def getStructuringElement(_shape, ksize):
        return np.ones((int(ksize[1]), int(ksize[0])), dtype=np.uint8)

    @staticmethod
    def dilate(img, kernel):
        return ndimage.grey_dilation(img, footprint=kernel > 0).astype(img.dtype)


class _Rendered:
    def __init__(self, red_pixels):
        self.red_pixels = int(red_pixels)

    def save(self, buffered, format=None):
        buffered.write(str(self.red_pixels).encode())


class _FakeVu:
    @staticmethod
    def write_number(image, _pose, _number):
        red = np.all(image == np.array([255, 0, 0], dtype=image.dtype), axis=-1)
        return _Rendered(red.sum())


_CHAT_NS = {'np': np, 'cv2': _FakeCv2, 'vu': _FakeVu, 'BytesIO': io.BytesIO}
exec(compile(_slice(CHAT_SRC, 'def get_all_candidate_maps',
                    'def get_all_candidate_obs_maps'),
             '<chat_utils:get_all_candidate_maps>', 'exec'), _CHAT_NS)
real_get_all_candidate_maps = _CHAT_NS['get_all_candidate_maps']


def red_pixels_per_image(candidate_maps):
    return [int(b.getvalue().decode()) for b in candidate_maps]


class FakeChatUtils:
    """The vendored module as `_assign` sees it: the real image loop, a fake
    network call that answers from a script, and LAST_CALL telemetry."""

    def __init__(self):
        self.LAST_CALL = {}
        self.answers = []
        self.requests = []          # num_frontier per call, as upstream counts it
        self.get_all_candidate_maps = real_get_all_candidate_maps

    def message_prepare(self, prompt, candidate_map_list, navigation_instruct):
        content = [{'type': 'text', 'text': navigation_instruct}]
        content += [{'type': 'image_url', 'bytes': b.getvalue()}
                    for b in candidate_map_list]
        return [{'role': 'system', 'content': prompt},
                {'role': 'user', 'content': content}]

    def chat_with_gpt4v(self, chat_history, model=None):
        # Upstream's own count of what it offered: one image per label.
        num_frontier = len(chat_history[1]['content']) - 1
        self.requests.append(num_frontier)
        ans = self.answers.pop(0) if self.answers else {'robot_0': 'frontier_0'}
        self.LAST_CALL.clear()
        self.LAST_CALL.update({
            'model': 'fake-vlm', 'num_frontier': num_frontier, 'attempts': 1,
            'server_s': 0.01, 'prompt_tokens': 10, 'completion_tokens': 5,
            'total_tokens': 15, 'parse': 'ok', 'invalid_ids': [],
            'errors': [], 'response_text': json.dumps(ans)})
        return ans


CHAT = FakeChatUtils()

_NS = {
    'np': np, 'math': math, 'time': CLOCK, 'sect': sect, 'json': json, 'os': os,
    'threading': threading, 'traceback': traceback, 'VOX_OCCUPIED': VOX_OCCUPIED,
    'chat_utils': CHAT,
    'system_prompt': types.SimpleNamespace(system_prompt='SYSTEM'),
    'explored_map_utils': types.SimpleNamespace(LAST_DET={}),
    'detect_frontier': None,        # the `nearest` arm only; never reached here
    'String': _String,
}

exec(compile(_slice(AGENT_SRC, 'def o3d_xz_to_map_xy', 'class AirStackAgent'),
             '<airstack_agent:o3d_xz_to_map_xy>', 'exec'), _NS)
exec(compile('class _StubAgent:\n'
             + _slice(AGENT_SRC, '    def grid_to_map_xy', '    def plan_path_map_xy'),
             '<airstack_agent:grid transforms>', 'exec'), _NS)

_PLANNER_BODY = ''.join([
    _slice(SRC, '    def _voxel_frontiers', '    # ── VLFM'),
    _slice(SRC, '    @staticmethod\n    def _points_in_polygon',
           '    def _vlfm_keyframe'),
    _slice(SRC, '    def _to_map_arr', '    def _frontier_pick'),
    _slice(SRC, '    def _blacklisted_mask', '    def _stall_watchdog'),
    _slice(SRC, '    def _assign', '    # ── actuation'),
    _slice(SRC, '    def _agent_xy', '    def _path_msg'),
    _slice(SRC, '    def _goal_xy', '    def _build_path'),
    _slice(SRC, '    def _is_visited', '    def _update_targets'),
])
exec(compile('class _StubPlanner:\n' + _PLANNER_BODY,
             '<planner_node:gpt arm>', 'exec'), _NS)
StubPlanner = _NS['_StubPlanner']


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_kw):
        self.lines.append(str(msg))

    info = warn = warning = error = debug = _log


class FakeAgent(_NS['_StubAgent']):
    """Only the state the sliced planner methods read off an agent."""

    def __init__(self, map_size=96, res_cm=100):
        self.args = types.SimpleNamespace(map_resolution=res_cm)
        self.map_size = int(map_size)
        self.origins_grid = (self.map_size // 2, self.map_size // 2)
        self.current_grid_pose = [self.map_size // 2, self.map_size // 2]
        self.found_goal = False
        self.nearest_point = None
        # open3d (x, y=height, z): map (0, 0, 12) -> o3d (0, 12, 0)
        self.camera_position = np.array([0.0, 12.0, 0.0])

    @property
    def res_m(self):
        return self.args.map_resolution / 100.0

    def set_xy(self, x, y):
        self.current_grid_pose = self.map_xy_to_grid(x, y)
        return self

    def xy(self):
        return self.grid_to_map_xy(*self.current_grid_pose)


class FakeVoxelMap:
    """A frontier source that returns a scripted (fr, gain) — for tests about
    what the planner DOES with candidates, not how they are carved."""

    def __init__(self, fr, gain):
        self.fr = np.asarray(fr, dtype=float).reshape(-1, 3)
        self.gain = np.asarray(gain, dtype=float)
        self.kwargs = None

    def integrate(self, *a, **k):
        return None

    def forget_older_than(self, max_age_ticks):
        return 0, 0

    def frontiers_persistent(self, **kwargs):
        self.kwargs = kwargs
        return self.fr.copy(), self.gain.copy()


def empty_merged():
    return types.SimpleNamespace(points=np.zeros((0, 3)))


def merged_from_map_points(pts_map):
    """Map-frame (x, y, z) -> the open3d frame `_voxel_frontiers` expects
    (`world = [pts.x, -pts.z, pts.y]`), i.e. o3d = (x, z, -y)."""
    p = np.asarray(pts_map, dtype=float)
    return types.SimpleNamespace(
        points=np.stack([p[:, 0], p[:, 2], -p[:, 1]], axis=1))


def make_planner(*, poly=None, voxel_map=None, max_frontiers=12, stratify=True,
                 vox_size=2.0, num_robots=1, warmup=0, round_period=0.0,
                 num_local_steps=25, min_alt=0.0, max_alt=0.0, fr_z=(0.0, 0.0),
                 altitude=12.0, inspect=0.0, map_size=96, res_cm=100,
                 nav_mode='gpt'):
    p = StubPlanner()
    p._log = _Logger()
    p.get_logger = lambda: p._log
    p._ros_clock = _RosClock()
    p.get_clock = lambda: p._ros_clock
    p._nav_mode = nav_mode
    p._frontier_source = 'voxel3d'
    p._agents = [FakeAgent(map_size, res_cm) for _ in range(num_robots)]
    p._num_robots = len(p._agents)
    p._map_size_cm = int(map_size * res_cm)
    p._map_resolution = int(res_cm)
    p._voxel_map = voxel_map
    p._vox_size = float(vox_size)
    p._vox_carve_samples = 24
    p._vox_max_points = 20000
    p._vox_max_range = 60.0
    p._vox_neigh_r = 1
    p._vox_min_unobs = 4
    p._vox_min_empty = 2
    p._vox_min_occ = 0
    p._fronti_subsampling = 2
    p._fronti_min_cells = 1
    p._fr_z_stratify = bool(stratify)
    p._fr_z_min, p._fr_z_max = float(fr_z[0]), float(fr_z[1])
    p._min_alt, p._max_alt = float(min_alt), float(max_alt)
    p._max_frontiers = int(max_frontiers)
    p._search_poly = poly
    p._marker_offset = None
    p._frontier_z = []
    p._frontier_candidates = []
    p._goal_z_cache = {}
    p._goal_points = []
    p._round = 0
    p._last_round_start = None
    p._warmup_steps = int(warmup)
    p._round_period_s = float(round_period)
    p._num_local_steps = int(num_local_steps)
    p._plan_period_s = 1.0
    p._goal_name = 'person'
    p._publish_vis = False
    p._track_instances = True
    p._active_target = None
    p._altitude = float(altitude)
    p._inspect_altitude = float(inspect)
    p._run_id = 'test'
    p._stats_path = os.path.join(SCRATCH, f'rounds_{id(p)}.jsonl')
    p._round_stats_pub = FakePub()
    # async round + blacklist + lock state the recheck reads
    p._pending_round = None
    p._frontier_blacklist = []
    p._blacklist_r = 8.0
    p._blacklist_ttl = 0.0
    p._locked_goal = p._locked_score = p._locked_goal_xy = None
    p._lm_sweep = None
    p._lm_idx = 0
    p._lm_leg_z = None
    p._visited_targets = []
    p._visit_radius = 12.0
    p._vox_forget_s = 0.0          # forgetting is exercised on the real map only
    return p


def top_view_for(p):
    size = p._map_size_cm // p._map_resolution
    return np.zeros((size, size, 3), dtype=np.uint8)


def assign_once(p, edge, pts, step=100):
    p._frontier_candidates = list(pts)
    size = edge.shape[0]
    obstacle = np.zeros((size, size), dtype=np.float32)
    grid_pose = [list(a.current_grid_pose) for a in p._agents]
    vis_pose = [[gp[1] * 480.0 / size, int((size - gp[0]) * 480.0 / size), 0.0]
                for gp in grid_pose]
    p._assign(edge, pts, top_view_for(p), vis_pose, obstacle, obstacle,
              grid_pose, step)
    return list(p._goal_points)


def run_round(p, edge, pts, answers, step=100):
    """A full VLM round the way `_tick` sees it: the tick that STARTS the
    round (goals held / nearest meanwhile), the worker finishing, and the
    tick that APPLIES the answer. Returns the applied goals."""
    CHAT.answers = list(answers)
    assign_once(p, edge, pts, step)
    pend = p._pending_round
    if pend is None:
        return list(p._goal_points)          # warm-up / no candidates: no round
    for _ in range(200):
        if pend.get('done'):
            break
        import time as _t
        _t.sleep(0.01)
    assert pend.get('done'), 'the VLM worker never finished'
    assert pend.get('error') is None, pend.get('error')
    return assign_once(p, edge, pts, step)


def _yaml_scalars(path):
    """Enough YAML for these flat, one-level config files."""
    out = {}
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            body = line.split('#', 1)[0].rstrip()
            if not body.strip() or ':' not in body:
                continue
            k, _, val = body.partition(':')
            if not k.startswith('    ') or k.startswith('     '):
                continue
            k, val = k.strip(), val.strip()
            if val:
                out[k] = val
    return out


SOURCE_KEYS = ['frontier_source', 'voxel_size_m', 'voxel_max_range_m',
               'voxel_neighborhood_r', 'voxel_min_unobserved',
               'voxel_min_empty', 'voxel_min_occupied',
               'frontier_subsampling', 'frontier_subsampling_min_cells',
               'frontier_z_stratify', 'frontier_z_min_m', 'frontier_z_max_m']

_SQUARE = np.array([[-30.0, -30.0], [30.0, -30.0], [30.0, 30.0], [-30.0, 30.0]])


# ── 1. the launch selects the 3D source ──────────────────────────────────────

def test_01_launch_default_selects_the_same_3d_frontiers_as_vlfm():
    """conavgpt2.launch.xml -> conavgpt2.yaml -> voxel3d, with every
    frontier-SOURCE parameter effectively equal to vlfm.yaml's and
    frontier.yaml's. paper.yaml (the previous default) sets none of them and so
    inherits slab2d: it was flying upstream's 2D map while the other arms flew
    the voxel map, which broke the 'same candidates' premise outright."""
    with open(os.path.join(_LAUNCH, 'conavgpt2.launch.xml'), encoding='utf-8') as fh:
        launch = fh.read()
    m = re.search(r'name="method_params_file"\s+default="([^"]+)"', launch)
    assert m, 'conavgpt2.launch.xml has no method_params_file default'
    assert m.group(1).endswith('/config/conavgpt2.yaml'), m.group(1)

    base = _yaml_scalars(os.path.join(_CONFIG, 'planner.yaml'))
    assert base['frontier_source'] == "'slab2d'", (
        'planner.yaml default changed; this test assumes slab2d is the base')
    conav = _yaml_scalars(os.path.join(_CONFIG, 'conavgpt2.yaml'))
    vlfm = _yaml_scalars(os.path.join(_CONFIG, 'vlfm.yaml'))
    fro = _yaml_scalars(os.path.join(_CONFIG, 'frontier.yaml'))
    paper = _yaml_scalars(os.path.join(_CONFIG, 'paper.yaml'))

    assert conav['nav_mode'] == "'gpt'", conav.get('nav_mode')
    assert conav['frontier_source'] == "'voxel3d'", conav.get('frontier_source')

    eff = {name: dict(base, **ov) for name, ov in
           (('conavgpt2', conav), ('vlfm', vlfm), ('frontier', fro))}
    for k in SOURCE_KEYS:
        vals = {name: e.get(k) for name, e in eff.items()}
        assert len(set(vals.values())) == 1, f'{k}: {vals}'
    assert eff['conavgpt2']['frontier_source'] == "'voxel3d'"

    # the 2D-faithful overlay really is 2D, and says so
    assert 'frontier_source' not in paper
    assert dict(base, **paper)['frontier_source'] == "'slab2d'"
    # ... and carries the same paper cadence/threshold the new default keeps
    for k in ('round_period_s', 'num_local_steps', 'plan_period_s', 'sem_threshold'):
        assert conav.get(k) == paper.get(k), (k, conav.get(k), paper.get(k))

    # what still differs between this arm and vlfm, for the record
    other = ['max_frontiers', 'vlfm_distance_penalty', 'sector_partition',
             'search_area_pad_m', 'round_period_s', 'sem_threshold',
             'plan_period_s', 'num_local_steps']
    delta = {k: (eff['conavgpt2'].get(k), eff['vlfm'].get(k)) for k in other
             if eff['conavgpt2'].get(k) != eff['vlfm'].get(k)}
    for k, (a, b) in sorted(delta.items()):
        print(f'    config delta  {k}: conavgpt2={a!r}  vlfm={b!r}')
    # THE DETECTOR GATE MUST BE UNIFORM. It used to differ -- conavgpt2/paper
    # kept upstream's indoor 0.85 while vlfm ran planner.yaml's 0.5 -- which
    # left conavgpt2 needing a detection the aerial detector rarely produces,
    # so `found_goal` differed between arms for a reason that is NOT the
    # selection policy. Every layer is now pinned to one value.
    assert 'sem_threshold' not in delta, (
        f"the arms gate detections differently again: {delta['sem_threshold']}")
    print('    launch -> conavgpt2.yaml -> voxel3d; source parameters identical '
          'to vlfm/frontier; paper.yaml stays the 2D-faithful overlay')


# ── 2. every candidate image shows its candidate ─────────────────────────────

def test_02_every_frontier_id_the_vlm_sees_is_drawn_on_its_image():
    """Stratified candidates at two heights over the same ground land in the
    same grid cell. The candidate images are cut by label (`edge == i+1`), so
    the second disc used to erase the first label and image 0 was blank.
    Now: one offered candidate per cell, labels 1..N in list order, every
    image has red pixels, and the offered count is refilled from lower ranks
    rather than shrunk by the duplicates."""
    #            x     y     z     gain      level (vox 2 m)
    fr = np.array([[10.0, 10.0, 3.0],     # A  100  L1  -> cell (58, 38)
                   [10.0, 10.0, 9.0],     # B   95  L4  same cell as A
                   [12.0, 12.0, 15.0],    # C   90  L7  disc overlaps A's
                   [-20.0, 5.0, 3.0],     # D   80  L1
                   [30.0, -30.0, 9.0],    # E   70  L4
                   [30.0, -30.0, 15.0]])  # F   60  L7  same cell as E
    gain = np.array([100.0, 95.0, 90.0, 80.0, 70.0, 60.0])
    vm = FakeVoxelMap(fr, gain)
    p = make_planner(voxel_map=vm, max_frontiers=12, stratify=True)
    agent = p._agents[0]

    edge, pts = p._voxel_frontiers(empty_merged())
    # stratified order would be A, B, C, D, E, F; B and F are 2D duplicates
    assert len(pts) == 4, pts
    assert pts[0] == list(agent.map_xy_to_grid(10.0, 10.0))
    assert pts[1] == list(agent.map_xy_to_grid(12.0, 12.0))
    assert pts[2] == list(agent.map_xy_to_grid(-20.0, 5.0))
    assert pts[3] == list(agent.map_xy_to_grid(30.0, -30.0))
    assert p._frontier_z == [3.0, 15.0, 3.0, 9.0], p._frontier_z
    labels = sorted(int(v) for v in np.unique(edge) if v > 0)
    assert labels == [1, 2, 3, 4], labels
    assert int(edge.max()) == len(pts)
    # A keeps its whole disc; C keeps the part A did not cover
    assert int((edge == 1).sum()) == 25, int((edge == 1).sum())
    assert 0 < int((edge == 2).sum()) < 25, int((edge == 2).sum())

    # the REAL vendored loop: one image per label, each with the frontier on it
    maps = real_get_all_candidate_maps(edge, top_view_for(p), [[0.0, 0.0, 0.0]])
    assert len(maps) == len(pts), (len(maps), len(pts))
    reds = red_pixels_per_image(maps)
    assert all(r > 0 for r in reds), reds

    # the cap counts OFFERED candidates, not ranked ones: 3 asked, 3 given,
    # still spanning three heights
    q = make_planner(voxel_map=FakeVoxelMap(fr, gain), max_frontiers=3)
    e3, p3 = q._voxel_frontiers(empty_merged())
    assert len(p3) == 3 and int(e3.max()) == 3, (len(p3), e3.max())
    assert q._frontier_z == [3.0, 15.0, 3.0], q._frontier_z

    # stratify off: pure gain order A B C D E F, duplicates B and F still
    # collapsed -> A, C, D, E
    r_ = make_planner(voxel_map=FakeVoxelMap(fr, gain), stratify=False)
    e0, p0 = r_._voxel_frontiers(empty_merged())
    assert len(p0) == 4 and int(e0.max()) == 4, p0
    assert r_._frontier_z == [3.0, 15.0, 3.0, 9.0], r_._frontier_z
    print(f'    6 ranked -> {len(pts)} offered (2 cell-duplicates dropped); '
          f'labels {labels}; red pixels per image {reds}; cap 3 -> 3 offered')


def test_02b_the_premise_is_the_vendored_loop():
    """Pin the two facts the fix rests on to the vendored source: images are
    cut by label up to edge.max(), and the response id is validated against
    the IMAGE count. If either moves, the label discipline in
    _voxel_frontiers is no longer what keeps `frontier_k` == list[k]."""
    loop = _slice(CHAT_SRC, 'def get_all_candidate_maps', 'def get_all_candidate_obs_maps')
    assert 'for i in range(int(target_edge_map.max())):' in loop
    assert 'path_map[target_edge_map == i+1] = 1' in loop
    call = _slice(CHAT_SRC, 'def chat_with_gpt4v', 'ground_json = {')
    assert "num_frontier = len(chat_history[1]['content'])-1" in call
    pa = _slice(CHAT_SRC, 'def parse_assignment', 'def chat_with_gpt4v')
    assert 'k < 0 or k >= num_frontier' in pa
    assert 'parse_assignment(response_message or "",' in call
    assert 'CONFIG.num_agents, num_frontier)' in call
    # and the planner resolves the id straight into the shared list
    res = _slice(SRC, '    def _resolve_frontier', '    def _random_goal')
    assert "int(str(assignment[f'robot_{i}']).split('_')[1])" in res
    assert 'return target_point_list[idx]' in res
    print('    images cut by label 1..edge.max(); id validated against the image '
          'count; planner indexes target_point_list[id]')


# ── 3. the source filter is in the map frame ─────────────────────────────────

def test_03_regression_source_filter_uses_the_map_frame():
    """`fr` is grid-frame metres; `search_area_xy` is authored in the map
    frame. With a non-zero grid->map offset the old filter judged the polygon
    displaced by that offset. The gpt arm has no pick-side filter (vlfm and
    frontier re-filter with `_to_map_arr` in their pick), so for it the source
    filter is the ONLY sector restriction."""
    fr = np.array([[50.0, 0.0, 5.0],      # grid (50, 0) -> map (0, 0): inside
                   [0.0, 0.0, 5.0]])      # grid (0, 0)  -> map (-50, 0): outside
    gain = np.array([10.0, 100.0])
    poly = np.array([[-20.0, -20.0], [20.0, -20.0], [20.0, 20.0], [-20.0, 20.0]])
    p = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=poly, map_size=200)
    p._marker_offset = np.array([50.0, 0.0, 0.0])
    edge, pts = p._voxel_frontiers(empty_merged())
    assert len(pts) == 1, pts
    assert pts[0] == list(p._agents[0].map_xy_to_grid(50.0, 0.0)), pts
    assert p._frontier_z == [5.0]

    # no offset yet (first tick): identity, and the higher-gain one wins
    q = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=poly, map_size=200)
    e2, pts2 = q._voxel_frontiers(empty_merged())
    assert len(pts2) == 1 and pts2[0] == list(q._agents[0].map_xy_to_grid(0.0, 0.0))

    # the source uses the same transform the picks and the path use
    vox = _slice(SRC, '    def _voxel_frontiers', '    def _rank_frontiers')
    assert 'self._to_map_arr(fr[:, :2])' in vox
    for name, end in (('_vlfm_pick', '    def _to_map_arr'),
                      ('_frontier_pick', '    def _lawnmower_pick')):
        s = _slice(SRC, f'    def {name}', end)
        assert 'self._points_in_polygon(self._to_map_arr(xy), self._search_poly)' in s
    print('    offset (50, 0): grid (50,0) kept as map (0,0); grid (0,0) '
          'rejected as map (-50,0). Same _to_map_arr as the picks.')


# ── 4. frontier_k -> target_point_list[k], for a whole round ─────────────────

def test_04_the_vlm_answer_indexes_the_shared_candidate_list():
    """The real `_assign` (gpt branch) with the real image loop and a scripted
    VLM: `robot_0: frontier_2` makes candidate 2 the goal, the round is
    recorded with as many images as candidates, and the goal is HELD — no
    per-tick re-pick, no `_commit`; the round period is the commitment."""
    fr = np.array([[10.0, 10.0, 3.0], [-20.0, 5.0, 9.0], [30.0, -30.0, 15.0],
                   [-30.0, -25.0, 3.0]])
    gain = np.array([100.0, 90.0, 80.0, 70.0])
    p = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2)
    edge, pts = p._voxel_frontiers(empty_merged())
    assert len(pts) == 4

    goals = run_round(p, edge, pts, [{'robot_0': 'frontier_2', 'reason': 'x'}])
    assert goals == [pts[2]], (goals, pts)
    assert p._round == 1
    assert p._last_round_start == CLOCK.time()
    assert CHAT.requests[-1] == 4, CHAT.requests[-1]

    # the round record: one image per candidate, and the 3D source's own
    # telemetry in place of the slab detector's
    with open(p._stats_path, encoding='utf-8') as fh:
        rows = [json.loads(ln) for ln in fh if ln.strip()]
    assert len(rows) == 1
    row = rows[0]
    assert row['num_frontiers'] == 4 and row['num_images'] == 4, row
    assert row['assignment'] == {'robot_0': 'frontier_2', 'reason': 'x'}
    assert row['frontier_det']['source'] == 'voxel3d'
    assert row['frontier_det']['z_m'] == [3.0, 9.0, 15.0, 3.0]
    assert row['frontier_det']['n_offered'] == 4
    assert row['parse'] == 'ok' and row['round_budget_s'] == 25.0
    assert len(p._round_stats_pub.msgs) == 1
    assert json.loads(p._round_stats_pub.msgs[0].data)['round'] == 1

    # the goal it flies is that candidate, in map metres
    agent = p._agents[0]
    assert p._goal_xy(0, agent) == agent.grid_to_map_xy(*pts[2])

    # an id the planner cannot use falls back to frontier_0 — its own guard,
    # behind the vendored one that already rejects ids >= image count
    p2 = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2)
    e2, pts2 = p2._voxel_frontiers(empty_merged())
    for bad in ({'robot_0': 'frontier_9'}, {'robot_0': 'nonsense'},
                {'robot_1': 'frontier_1'}):
        g = run_round(p2, e2, pts2, [bad])
        assert g == [pts2[0]], (bad, g)
        assert any('unusable assignment' in ln for ln in p2._log.lines)

    # no _commit in this arm: the assignment is held between rounds by _tick
    asn = _slice(SRC, '    def _assign', '    def _start_round')
    gpt = asn[asn.index("if self._nav_mode == 'gpt':"):asn.index("elif self._nav_mode == 'vlfm':")]
    assert 'self._commit' not in gpt
    assert 'self._apply_round(pend, map_size)' in gpt
    assert 'self._resolve_frontier(' in _slice(SRC, '    def _apply_round',
                                                '    def _nearest_candidate')
    print('    frontier_2 -> pts[2]; round logged with 4 images for 4 '
          'candidates and voxel3d z list; bad ids -> frontier_0; no _commit')


# ── 5. no candidates: stay in the sector ─────────────────────────────────────

def test_05_no_candidates_steer_into_the_sector_not_a_random_cell():
    """With the polygon applied at the source, 'every frontier is out of
    bounds' — and t=0 before the first carve — reaches _assign as an EMPTY
    list. Upstream's answer was a random cell of the whole grid, re-drawn every
    tick; on a 280 m extent that is up to 140 m outside the sector. Now the
    sector centroid, via the same `_recover_into_area` the other arms use."""
    poly = np.array([[10.0, 10.0], [40.0, 10.0], [40.0, 40.0], [10.0, 40.0]])
    p = make_planner(voxel_map=FakeVoxelMap(np.zeros((0, 3)), np.zeros(0)), poly=poly)
    agent = p._agents[0]
    edge, pts = p._voxel_frontiers(empty_merged())
    assert pts == []
    goals = run_round(p, edge, pts, [])
    cx, cy = sect.polygon_centroid(poly)
    assert goals == [p._map_to_grid(agent, cx, cy)], (goals, (cx, cy))
    assert any('no frontier candidates' in ln for ln in p._log.lines)
    assert CHAT.requests == [] or CHAT.requests[-1] != 0, 'the VLM was called with nothing'
    # no round was consumed, so the next tick asks again
    assert p._round == 0 and p._last_round_start is None

    # with an offset, the centroid still comes out in the GRID frame
    q = make_planner(voxel_map=FakeVoxelMap(np.zeros((0, 3)), np.zeros(0)), poly=poly)
    q._marker_offset = np.array([-15.0, 7.0, 0.0])
    g = run_round(q, *q._voxel_frontiers(empty_merged()), [])
    gx, gy = q._agents[0].grid_to_map_xy(*g[0])
    assert math.dist(q._to_map(gx, gy), (cx, cy)) <= 2.0 * q._agents[0].res_m, (
        q._to_map(gx, gy), (cx, cy))

    # no polygon at all -> upstream's random cell, inside the grid
    r_ = make_planner(voxel_map=FakeVoxelMap(np.zeros((0, 3)), np.zeros(0)), poly=None)
    for _ in range(20):
        g = run_round(r_, *r_._voxel_frontiers(empty_merged()), [])
        assert 0 <= g[0][0] < 96 and 0 <= g[0][1] < 96, g

    # every arm's no-candidate branch goes the same way
    asn = _slice(SRC, '    def _assign', '    def _record_round')
    assert asn.count('self._fallback_goal(self._agents[i], map_size)') == 5, (
        asn.count('self._fallback_goal(self._agents[i], map_size)'))
    assert 'self._random_goal(map_size)' not in asn
    # warm-up too: below warmup_steps the gpt arm does not call the VLM
    w = make_planner(voxel_map=FakeVoxelMap(
        np.array([[10.0, 10.0, 3.0]]), np.array([5.0])), poly=poly, warmup=30)
    before = len(CHAT.requests)
    ew, pw = w._voxel_frontiers(empty_merged())
    g = run_round(w, ew, pw, [], step=3)
    assert len(CHAT.requests) == before, 'VLM called during warm-up'
    assert w._pending_round is None
    # a candidate exists, so the drone heads for the nearest one meanwhile
    assert g == [pw[0]], g
    w2 = make_planner(voxel_map=FakeVoxelMap(np.zeros((0, 3)), np.zeros(0)),
                      poly=poly, warmup=30)
    g2 = run_round(w2, *w2._voxel_frontiers(empty_merged()), [], step=3)
    assert g2 == [w2._map_to_grid(w2._agents[0], cx, cy)], g2
    print(f'    empty list -> centroid ({cx:.0f}, {cy:.0f}) cell {goals[0]}, '
          'offset-correct; no polygon -> random in-grid; five branches share '
          '_fallback_goal; warm-up skips the VLM')


# ── 6. the frontier's own height is flown, and held ──────────────────────────

def test_06_assigned_frontier_sets_the_waypoint_height_and_holds_it():
    """A 3D frontier carries a z; the waypoint flies at it, clamped to the
    flight band. The list is re-extracted every tick and the assigned frontier
    is carved away as the drone closes on it, at which point a fresh lookup
    finds nothing — for this arm that is most of a 25-tick approach at the
    wrong height. While the goal cell is unchanged, the assigned z stands."""
    fr = np.array([[10.0, 10.0, 3.0], [-20.0, 5.0, 9.0], [30.0, -30.0, 22.0]])
    gain = np.array([100.0, 90.0, 80.0])
    p = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2,
                     min_alt=4.0, max_alt=18.0, altitude=12.0)
    agent = p._agents[0]
    edge, pts = p._voxel_frontiers(empty_merged())
    assert p._frontier_z == [3.0, 9.0, 22.0]

    run_round(p, edge, pts, [{'robot_0': 'frontier_1'}])
    assert p._frontier_altitude(0) == 9.0
    assert p._waypoint_z(agent) == 9.0

    # clamped to the band at both ends
    for k, want in ((0, 4.0), (2, 18.0)):
        q = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2,
                         min_alt=4.0, max_alt=18.0)
        e, c = q._voxel_frontiers(empty_merged())
        run_round(q, e, c, [{'robot_0': f'frontier_{k}'}])
        assert q._waypoint_z(q._agents[0]) == want, (k, q._waypoint_z(q._agents[0]))

    # next tick: the assigned frontier has been carved away, list re-extracted
    p._voxel_map = FakeVoxelMap(fr[[0, 2]], gain[[0, 2]])
    edge2, pts2 = p._voxel_frontiers(empty_merged())
    p._frontier_candidates = list(pts2)
    assert pts[1] not in pts2
    assert p._goal_points == [pts[1]], 'the goal must be held between rounds'
    assert p._frontier_altitude(0) == 9.0, 'height jumped when the frontier vanished'
    assert p._waypoint_z(agent) == 9.0

    # a NEW goal at a cell nobody offered: no height known -> cruise
    p._goal_points = [[3, 3]]
    assert p._frontier_altitude(0) is None
    assert p._waypoint_z(agent) == 12.0

    # 2D source: never a frontier height
    p._frontier_source = 'slab2d'
    p._goal_points = [pts2[0]]
    assert p._frontier_altitude(0) is None

    # the detected target overrides everything, as for every arm
    p._frontier_source = 'voxel3d'
    p._inspect_altitude = 3.0
    agent.found_goal = True
    agent.nearest_point = np.array([5.0, 1.7, 5.0])       # o3d: height is [1]
    assert p._waypoint_z(agent) == 4.7
    agent.found_goal = False
    print('    frontier_1 at 9 m -> waypoint 9 m; 3 m/22 m clamp to [4, 18]; '
          'held at 9 m after the frontier is carved; new unknown cell -> cruise')


# ── 7. the cadence is upstream's, and _tick is shared ────────────────────────

def test_07_round_cadence_is_upstreams_and_the_tick_is_shared():
    """round_period_s 0 (conavgpt2.yaml / paper.yaml) selects `step %
    num_local_steps == num_local_steps - 1`; the candidate list the round
    indexes into is the one every arm builds; and between rounds the goal
    from `_assign` is what `_command` flies."""
    tick = _slice(SRC, '    def _tick', '    def _voxel_frontiers')
    assert 'if self._round_period_s > 0.0:' in tick
    assert 'step % self._num_local_steps == self._num_local_steps - 1' in tick
    assert 'self._frontier_candidates = list(target_point_list)' in tick
    assert 'if assignment_due and not found_goal:' in tick
    assert tick.count('self._nav_mode ==') == 1, 'a second nav_mode branch in _tick'
    assert "if self._frontier_source == 'voxel3d' and self._voxel_map is not None:" in tick
    assert 'self._voxel_frontiers(merged)' in tick
    # the same target_edge_map / target_point_list pair reaches _assign
    assert 'self._assign(target_edge_map, target_point_list, top_view_map,' in tick
    # every tick flies whatever _goal_points holds, for every arm
    assert 'agent.act(self._goal_points[i], grid_pose)' in tick
    assert 'self._command(i, agent, offsets[i])' in tick

    asn = _slice(SRC, '    def _assign', '    def _start_round')
    gpt = asn[asn.index("if self._nav_mode == 'gpt':"):asn.index("elif self._nav_mode == 'vlfm':")]
    assert 'step >= self._warmup_steps' in gpt
    start = _slice(SRC, '    def _start_round', '    def _apply_round')
    assert 'self._last_round_start = round_start' in start
    assert 'chat_utils.get_all_candidate_maps(' in start
    assert "threading.Thread(target=_work, name='vlm-round', daemon=True).start()" in start
    # the tick applies a finished round regardless of cadence
    assert "if pend is not None and pend.get('done'):" in tick
    # the assignment runs through the same budget accounting as the timer mode
    rec = _slice(SRC, '    def _record_round', '    def _resolve_frontier')
    assert 'self._num_local_steps * self._plan_period_s' in rec

    # numerically: 25 local steps at 1 s -> a round every 25 ticks
    due = [s for s in range(100) if s % 25 == 25 - 1]
    assert due == [24, 49, 74, 99]
    note('gpt viz',
         "_publish_frontier_cloud colours the committed goal from "
         "`_locked_goal_xy`, which only the vlfm/frontier picks set via "
         "_commit; the gpt arm never sets it, so its frontier cloud never "
         "shows a green goal (the MarkerArray path, which matches "
         "`_goal_points` by cell, does). Cosmetic.")
    print('    step-based cadence pinned; _tick unconditioned; assignment '
          'indexes the shared list; goal held by _tick between rounds')


# ── 8. end to end: a carved map, through the real loop, to a goal ────────────

def test_08_end_to_end_carved_map_to_a_held_3d_goal():
    """A real VoxelMap carved from a synthetic ground plane and one wall,
    through the real `_voxel_frontiers`, the real vendored image loop, the
    real `_assign`: the VLM is shown N images for N candidates, its answer is
    a candidate inside the polygon, and the waypoint flies at that
    candidate's height."""
    vm = VoxelMap((-40.0, -40.0, 0.0), (40.0, 40.0, 24.0), 2.0)
    rng = np.random.default_rng(7)
    cam = np.array([0.0, 0.0, 12.0])
    ground = np.column_stack([rng.uniform(-30.0, 30.0, 5000),
                              rng.uniform(-30.0, 30.0, 5000),
                              np.zeros(5000)])
    wall = np.column_stack([np.full(600, 20.0), rng.uniform(-10.0, 10.0, 600),
                            rng.uniform(0.0, 10.0, 600)])
    pts_map = np.vstack([ground, wall])

    poly = np.array([[-36.0, -36.0], [36.0, -36.0], [36.0, 36.0], [-36.0, 36.0]])
    p = make_planner(voxel_map=vm, poly=poly, max_frontiers=8, stratify=True,
                     min_alt=2.0, max_alt=20.0, fr_z=(0.5, 20.0), altitude=12.0)
    p._vox_carve_samples = 32
    agent = p._agents[0]
    agent.camera_position = np.array([cam[0], cam[2], -cam[1]])

    edge, cands = p._voxel_frontiers(merged_from_map_points(pts_map))
    assert 3 <= len(cands) <= 8, len(cands)
    assert len(p._frontier_z) == len(cands)
    labels = sorted(int(v) for v in np.unique(edge) if v > 0)
    assert labels == list(range(1, len(cands) + 1)), labels
    cells = [tuple(c) for c in cands]
    assert len(set(cells)) == len(cells), 'duplicate cells offered'
    heights = sorted({int(z // 2.0) for z in p._frontier_z})
    xy = np.array([agent.grid_to_map_xy(*c) for c in cands])
    assert p._points_in_polygon(xy, poly).all(), 'candidate outside the polygon'

    maps = real_get_all_candidate_maps(edge, top_view_for(p), [[0.0, 0.0, 0.0]])
    assert len(maps) == len(cands)
    assert all(r > 0 for r in red_pixels_per_image(maps))

    k = len(cands) - 1
    goals = run_round(p, edge, cands, [{'robot_0': f'frontier_{k}'}])
    assert goals == [cands[k]]
    assert CHAT.requests[-1] == len(cands)
    want_z = min(20.0, max(2.0, p._frontier_z[k]))
    assert p._waypoint_z(agent) == want_z, (p._waypoint_z(agent), want_z)

    # a second observation re-extracts the list; the goal and its z stand
    edge2, cands2 = p._voxel_frontiers(merged_from_map_points(pts_map))
    p._frontier_candidates = list(cands2)
    assert p._goal_points == [cands[k]]
    assert p._waypoint_z(agent) == want_z
    print(f'    {len(cands)} candidates over {len(heights)} heights from a carved '
          f'map; {len(maps)} images, all drawn; frontier_{k} -> cell {cands[k]} '
          f'at z {p._frontier_z[k]:.1f} m, held across re-extraction')


# ── 10. the round is asynchronous ────────────────────────────────────────────

def test_10_the_tick_never_waits_for_the_vlm():
    """Starting a round returns at once with a goal to fly (the previous one,
    or the nearest candidate); the answer is applied on the tick it lands,
    against the list the VLM was shown; a second round is not started while
    one is in flight."""
    fr = np.array([[10.0, 10.0, 3.0], [-20.0, 5.0, 9.0], [30.0, -30.0, 15.0]])
    gain = np.array([100.0, 90.0, 80.0])
    p = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2)
    edge, pts = p._voxel_frontiers(empty_merged())
    p._agents[0].set_xy(-18.0, 6.0)            # next to candidate 1

    gate = threading.Event()
    real = CHAT.chat_with_gpt4v

    def slow(message, model=None):
        gate.wait(5.0)
        return real(message, model)
    CHAT.chat_with_gpt4v = slow
    try:
        CHAT.answers = [{'robot_0': 'frontier_2'}]
        g0 = assign_once(p, edge, pts)
        assert p._pending_round is not None and not p._pending_round['done']
        assert g0 == [pts[1]], f'meanwhile goal should be the NEAREST candidate: {g0}'
        assert p._round == 1
        # another tick while the VLM is still thinking: same goal, no new round
        g1 = assign_once(p, edge, pts)
        assert g1 == [pts[1]] and p._round == 1
        gate.set()
        for _ in range(300):
            if p._pending_round['done']:
                break
            import time as _t
            _t.sleep(0.01)
        assert p._pending_round['done']
        # the map moved on: a different list is offered now, but the answer
        # indexes the list the VLM SAW
        moved = [[c[0] + 30, c[1] + 30] for c in pts]
        g2 = assign_once(p, edge, moved)
        assert g2 == [pts[2]], (g2, pts[2])
        assert p._pending_round is None
        with open(p._stats_path, encoding='utf-8') as fh:
            rows = [json.loads(ln) for ln in fh if ln.strip()]
        assert len(rows) == 1 and rows[0]['assignment'] == {'robot_0': 'frontier_2'}
    finally:
        CHAT.chat_with_gpt4v = real
    print('    round started -> nearest candidate flown; no second round in '
          'flight; answer applied against the snapshot list; one row logged')


def test_10b_lenient_parse_of_a_truncated_reply():
    """The vendored parser (sliced) recovers `robot_0` from a reply cut off
    mid-reason, drops phantom robots, and still refuses a bad or missing id."""
    i = CHAT_SRC.index('_ASSIGN_RE = ')
    j = CHAT_SRC.index('def chat_with_gpt4v(')
    import ast as _ast
    ns = {'re': re, 'ast': _ast, 'LAST_CALL': {'invalid_ids': [], 'parse': 'failed'}}
    exec(compile(CHAT_SRC[i:j], '<chat_utils:parse_assignment>', 'exec'), ns)
    pa = ns['parse_assignment']
    trunc = ('{  \n    "robot_0": "frontier_5",\n    "robot_1": "frontier_6",\n'
             '    "robot_2": "frontier_7",\n    "reason": "Robot 0 is assigned to')
    assert pa(trunc, 1, 8) == {'robot_0': 'frontier_5', 'reason': 'Robot 0 is assigned to'}
    assert ns['LAST_CALL']['lenient'] is True
    assert pa(trunc, 2, 8) == {'robot_0': 'frontier_5', 'robot_1': 'frontier_6',
                               'reason': 'Robot 0 is assigned to'}
    assert pa('{"robot_0": "frontier_1", "reason": "close"}', 1, 8) == {
        'robot_0': 'frontier_1', 'reason': 'close'}
    assert pa('{"robot_0": "frontier_9"}', 1, 8) is None
    assert pa('{"robot_1": "frontier_2"}', 1, 8) is None
    assert 'retries = 2' in CHAT_SRC and 'max_tokens=160' in CHAT_SRC
    print('    truncated reply -> robot_0 recovered (lenient), phantom robots '
          'dropped, bad/missing ids refused; 2 retries, 160 tokens')


# ── 11. a goal that turned out to be inside an obstacle ──────────────────────

def _occupy_around(vm, x, y, z):
    idx = vm.to_idx(np.array([[x, y, z]]))[0]
    vm.grid[idx[0], idx[1], idx[2]] = VOX_OCCUPIED


def test_11_goal_inside_newly_observed_voxels_is_dropped_and_repicked():
    """Every arm re-checks its goal against the live voxel map each tick:
    the gpt / vlfm / frontier arms blacklist and re-pick, the lawnmower lifts
    the leg. A goal in free space is left alone."""
    vm = VoxelMap((-40.0, -40.0, 0.0), (40.0, 40.0, 24.0), 2.0)
    fr = np.array([[10.0, 10.0, 9.0], [-20.0, 5.0, 9.0], [30.0, -30.0, 9.0]])
    gain = np.array([100.0, 90.0, 80.0])
    p = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2,
                     min_alt=2.0, max_alt=20.0)
    p._voxel_map = FakeVoxelMap(fr, gain)      # for extraction
    edge, pts = p._voxel_frontiers(empty_merged())
    run_round(p, edge, pts, [{'robot_0': 'frontier_0'}])
    assert p._goal_points == [pts[0]]
    p._voxel_map = vm                          # the LIVE map for the recheck
    agent = p._agents[0]
    assert p._frontier_altitude(0) == 9.0
    assert not p._goal_voxel_blocked(0, agent), 'free space must not block'
    assert not p._recheck_goals() and p._goal_points == [pts[0]]

    # the camera now sees a canopy exactly where the goal is
    gx, gy = agent.grid_to_map_xy(*pts[0])
    _occupy_around(vm, gx + 1.0, gy - 1.0, 10.0)     # neighbouring voxel
    assert p._goal_voxel_blocked(0, agent)
    assert p._recheck_goals()
    assert p._goal_points == [], 'the blocked goal must be dropped'
    assert len(p._frontier_blacklist) == 1
    assert any('newly observed voxels' in ln for ln in p._log.lines)
    # ...and the source no longer offers that frontier
    p._voxel_map = FakeVoxelMap(fr, gain)
    edge2, pts2 = p._voxel_frontiers(empty_merged())
    assert pts[0] not in pts2 and len(pts2) == 2, pts2
    # the next assign flies the nearest remaining candidate at once
    g = assign_once(p, edge2, pts2)
    assert g and g[0] in pts2

    # a person being visited is never rechecked (the goal is not a frontier)
    q = make_planner(voxel_map=FakeVoxelMap(fr, gain), poly=_SQUARE * 2)
    run_round(q, *q._voxel_frontiers(empty_merged()), [{'robot_0': 'frontier_0'}])
    q._voxel_map = vm
    q._active_target = (5.0, 5.0)
    assert not q._recheck_goals()

    # lawnmower: lift, not blacklist
    class _Sw:
        def __init__(self): self.calls = []
        def blocked(self, now=None): self.calls.append(now); return 'lifted'
    r_ = make_planner(voxel_map=vm, poly=_SQUARE * 2, nav_mode='lawnmower')
    r_._goal_points = [pts[0]]
    r_._lm_leg_z = (10.0, 12.0)
    r_._lm_sweep = _Sw()
    assert r_._recheck_goals() and r_._lm_sweep.calls and not r_._frontier_blacklist
    assert any('lifted' in ln for ln in r_._log.lines)
    # no voxel map -> never blocks
    s_ = make_planner(voxel_map=None, poly=_SQUARE * 2)
    s_._goal_points = [pts[0]]
    assert not s_._recheck_goals()
    print('    free goal kept; goal beside an occupied voxel -> blacklisted, '
          'dropped from the source, re-picked; target detour exempt; '
          'lawnmower lifts instead')


def test_11b_a_person_inside_occupied_voxels_is_given_up():
    """A detection whose approach point is inside the canopy the camera has
    now mapped is a false positive the drone can never reach: it is marked
    VISITED (never re-selected) and the frontier search resumes. A person in
    free space keeps the drone."""
    vm = VoxelMap((-40.0, -40.0, 0.0), (40.0, 40.0, 24.0), 2.0)
    p = make_planner(voxel_map=vm, poly=_SQUARE * 2, inspect=3.0, altitude=12.0)
    agent = p._agents[0]
    agent.found_goal = True
    agent.nearest_point = np.array([5.0, 0.2, -5.0])      # o3d: height is [1]
    p._goal_points = [[50, 50]]
    p._active_target = (5.0, 5.0)
    # free: nothing happens, the target is kept
    assert not p._recheck_goals()
    assert p._active_target == (5.0, 5.0) and p._visited_targets == []
    # the approach point (3.2 m over the detection) is in a mapped canopy
    z = p._waypoint_z(agent)
    _occupy_around(vm, 5.0, 5.0, z)
    assert p._recheck_goals()
    assert p._active_target is None and p._goal_points == []
    assert p._visited_targets == [(5.0, 5.0)]
    assert any('GIVING UP' in ln for ln in p._log.lines)
    # ...and _update_targets will not hand it back: it is 'visited' now
    assert p._is_visited((5.4, 4.8))
    print(f'    person at (5, 5): free -> kept; canopy at {z:.1f} m -> given up, '
          'marked visited, search resumes')


def test_11c_the_map_forgets_and_frontiers_go_with_it():
    """`forget_older_than`: voxels not re-written within the window return
    to UNOBSERVED and the persistent frontiers standing in them are dropped —
    the map is for navigation, not for surveying the scene."""
    from search_baselines.voxel_map import UNOBSERVED
    vm = VoxelMap((-40.0, -40.0, 0.0), (40.0, 40.0, 20.0), 2.0)
    rng = np.random.default_rng(1)
    west = np.column_stack([rng.uniform(-30, -10, 2000), rng.uniform(-30, 30, 2000),
                            np.zeros(2000)])
    vm.integrate(np.array([-20.0, 0.0, 12.0]), west, carve_samples=24, max_range_m=60)
    fr1, _ = vm.frontiers_persistent(subsampling=2, subsampling_min_fronti=1)
    n_west = int((vm.grid[:20] != UNOBSERVED).sum())
    assert n_west > 100 and len(fr1) > 10
    for _ in range(5):
        east = np.column_stack([rng.uniform(10, 30, 2000), rng.uniform(-30, 30, 2000),
                                np.zeros(2000)])
        vm.integrate(np.array([20.0, 0.0, 12.0]), east, carve_samples=24, max_range_m=60)
    fr2, _ = vm.frontiers_persistent(subsampling=2, subsampling_min_fronti=1)
    assert len(fr2) > len(fr1), 'frontiers should have accumulated'
    assert vm.forget_older_than(0) == (0, 0), '0 disables forgetting'
    n_forgot, n_dropped = vm.forget_older_than(3)
    assert n_forgot >= n_west and n_dropped > 0
    assert (vm.grid[:20] == UNOBSERVED).all(), 'the west half must be forgotten'
    assert (vm.grid[20:] != UNOBSERVED).sum() > 100, 'the recent east half stays'
    fr3, _ = vm.frontiers_persistent(subsampling=2, subsampling_min_fronti=1)
    assert len(fr3) < len(fr2) and (fr3[:, 0] > -10).all(), 'stale west frontiers linger'
    # the planner applies it every tick from voxel_forget_after_s
    vox = _slice(SRC, '    def _voxel_frontiers', '    def _rank_frontiers')
    assert 'self._voxel_map.forget_older_than(' in vox and '_vox_forget_s' in vox
    print(f'    west half ({n_west} voxels) forgotten after the drone moved east; '
          f'{n_dropped} stale frontiers dropped ({len(fr2)} -> {len(fr3)})')


# ── 9. parity: the arms differ only in selection ─────────────────────────────

def test_09_parity_the_gpt_arm_shares_source_filter_and_actuation():
    """Nothing this arm flies through is conditioned on nav_mode except the
    selection branch in _assign — the same guarantee test_frontier_baseline
    makes for vlfm vs frontier, extended to gpt."""
    for start, end in (('    def _voxel_frontiers', '    def _rank_frontiers'),
                       ('    def _command', '    def _await'),
                       ('    def _update_targets', '    def _goal_xy'),
                       ('    def _frontier_altitude', '    def _waypoint_z'),
                       ('    def _fallback_goal', '    # ── actuation'),
                       ('    def _recover_into_area', '    def _frontier_pick')):
        s = _slice(SRC, start, end)
        assert 'self._nav_mode' not in s, f'{start.strip()} branches on nav_mode'
    init = _slice(SRC, '    def _init_agents', '    # ── subscriptions')
    # the voxel map is built for every arm, gated on the source alone
    vox_build = init[init.index("if self._frontier_source == 'voxel3d':"):]
    assert 'self._voxel_map = VoxelMap(' in vox_build
    # only the gpt arm preflights the generative endpoint
    assert "if self._vlm_preflight and self._nav_mode == 'gpt':" in init
    print('    frontier source, fallback, altitude, target visiting and '
          'actuation are unconditioned on nav_mode; VLM preflight is gpt-only')


# ── runner ───────────────────────────────────────────────────────────────────

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith('test_') and callable(o)]
    failures = []
    print(f'conavgpt2 baseline: {len(tests)} tests, offline '
          f'(numpy {np.__version__})\n')
    for name, fn in tests:
        doc = (fn.__doc__ or '').strip().splitlines()
        print(f'  {name}')
        if doc:
            print(f'    -- {doc[0]}')
        try:
            fn()
            print('    PASS\n')
        except Exception as exc:                        # noqa: BLE001
            import traceback
            failures.append((name, exc))
            print('    FAIL: ' + str(exc).replace('\n', '\n          '))
            print(_indent(traceback.format_exc(), '          '))
    if NOTES:
        print('\n' + '=' * 78)
        print('FINDINGS (not failures)')
        print('=' * 78)
        seen = set()
        for tag, msg in NOTES:
            if (tag, msg) in seen:
                continue
            seen.add((tag, msg))
            print(f'\n[{tag}]')
            print(_indent(_wrap(msg, 74), '  '))
    print('\n' + '=' * 78)
    if failures:
        print(f'FAILED {len(failures)} of {len(tests)}: '
              + ', '.join(n for n, _ in failures))
        return 1
    print(f'PASSED {len(tests)} of {len(tests)}')
    return 0


def _wrap(s, width):
    import textwrap as _tw
    return '\n'.join(_tw.fill(par, width) for par in s.split('\n'))


def _indent(s, pad):
    import textwrap as _tw
    return _tw.indent(s, pad)


if __name__ == '__main__':
    sys.exit(main())
