"""Offline unit tests for the `lawnmower` nav_mode of the shared search planner.

NO Isaac Sim, NO GPU, NO ROS. Pure python + numpy.

THE CLAIM UNDER TEST. The lawnmower drives the drone the way the frontier arms
do: ONE short goal at a time through the same `_command` (activator
NavigateTask + a two-pose Path on /global_plan), never the coverage path and
never a bare lane end hundreds of metres out. First a transit to the sector
in legs of `lawnmower_leg_m`, then the lanes in legs of the same length; a leg
is done when reached or passed, and a leg the drone makes no progress on is
skipped rather than held forever.

HOW THE PLANNER'S OWN CODE IS REACHED. As in test_frontier_baseline.py: the
methods are sliced out of planner_node.py and exec'd onto a stub class, so the
REAL `_lawnmower_pick`, `_build_path`, `_through_xy` and `_command` run, with
ROS message construction and the publishers replaced by fakes that record what
they were handed. The grid<->metre transforms are the real ones from
airstack_agent.py.

Run:  python3 -m pytest tests/test_lawnmower_baseline.py -s
"""

import math
import os
import re
import sys
import threading
import types

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from search_baselines import lawnmower as lm      # noqa: E402
from search_baselines import sector as sect       # noqa: E402
from search_baselines import clearance as clr     # noqa: E402

_PLANNER_PY = os.path.join(_PKG_ROOT, 'search_baselines', 'planner_node.py')
_AGENT_PY = os.path.join(_PKG_ROOT, 'search_baselines', 'airstack_agent.py')
_CONFIG = os.path.join(_PKG_ROOT, 'config')

with open(_PLANNER_PY, encoding='utf-8') as _f:
    SRC = _f.read()
with open(_AGENT_PY, encoding='utf-8') as _f:
    AGENT_SRC = _f.read()
with open(os.path.join(_PKG_ROOT, 'search_baselines', 'lawnmower.py'),
          encoding='utf-8') as _f:
    SRC_LM = _f.read()


# ── the slicing harness ──────────────────────────────────────────────────────

def _slice(src, start, end):
    i = src.index(start)
    j = src.index(end, i + len(start))
    return src[i:j]


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


CLOCK = _Clock()
# `threading`: airstack_agent's module scope now holds a detector-patch lock
_NS = {'np': np, 'math': math, 'time': CLOCK, 'lm': lm, 'sect': sect, 'clr': clr,
       'os': os, 'threading': threading, 're': re}

exec(compile(_slice(AGENT_SRC, 'def o3d_xz_to_map_xy', 'class AirStackAgent'),
             '<airstack_agent:o3d_xz_to_map_xy>', 'exec'), _NS)
exec(compile('class _StubAgent:\n'
             + _slice(AGENT_SRC, '    def grid_to_map_xy', '    def plan_path_map_xy'),
             '<airstack_agent:grid transforms>', 'exec'), _NS)

_PLANNER_BODY = ''.join([
    # _to_map_arr, _to_map, _map_to_grid_xy, _map_to_grid, _recover_into_area
    _slice(SRC, '    def _to_map_arr', '    def _frontier_pick'),
    _slice(SRC, '    def _robot_index_of_self', '    def _p(self'),
    _slice(SRC, '    def _lawnmower_pick', '    def _commit'),
    # _goal_xy, _build_path, _on_target, _through_xy
    _slice(SRC, '    def _goal_xy', '    def _agent_xy'),
    _slice(SRC, '    def _agent_xy', '    def _frontier_altitude'),
    _slice(SRC, '    def _clamp_alt', '    def _path_msg'),
    # _intent_speed (target / transit-to-sector / explore), then _command
    _slice(SRC, '    def _intent_speed', '    def _await'),
])
exec(compile('class _StubPlanner:\n' + _PLANNER_BODY,
             '<planner_node:lawnmower>', 'exec'), _NS)
StubPlanner = _NS['_StubPlanner']


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_kw):
        self.lines.append(str(msg))

    info = warn = warning = error = debug = _log

    def grep(self, pat):
        return [ln for ln in self.lines if re.search(pat, ln)]


class FakeAgent(_NS['_StubAgent']):
    """suburb_mini's grid: 480 cells over 280 m -> 58 cm."""

    def __init__(self, map_size=480, res_cm=58):
        self.args = types.SimpleNamespace(map_resolution=res_cm)
        self.map_size = int(map_size)
        self.origins_grid = (self.map_size // 2, self.map_size // 2)
        self.current_grid_pose = [self.map_size // 2, self.map_size // 2]
        self.found_goal = False
        self.nearest_point = None

    @property
    def res_m(self):
        return self.args.map_resolution / 100.0

    def set_grid_xy(self, x, y):
        self.current_grid_pose = self.map_xy_to_grid(x, y)
        return self


class FakePath:
    """What `_path_msg` builds, minus ROS: poses in the frame the drone flies
    (grid minus offset), exactly as the real one subtracts it. `z` is one
    height or one per pose, as the real `_path_msg` accepts."""

    def __init__(self, pts, offset, z):
        self.poses = []
        zs = list(z) if isinstance(z, (list, tuple, np.ndarray)) else [z] * len(pts)
        for (x, y), pz in zip(pts, zs):
            pos = types.SimpleNamespace(x=float(x - offset[0]),
                                        y=float(y - offset[1]), z=float(pz))
            self.poses.append(types.SimpleNamespace(
                pose=types.SimpleNamespace(position=pos)))

    def xy(self, k):
        return np.array([self.poses[k].pose.position.x,
                         self.poses[k].pose.position.y])

    def z(self, k):
        return float(self.poses[k].pose.position.z)


class FakePub:
    def __init__(self):
        self.last = None
        self.count = 0

    def publish(self, msg):
        self.last = msg
        self.count += 1


SQUARE_130 = np.array([[-130.0, -130.0], [130.0, -130.0],
                       [130.0, 130.0], [-130.0, 130.0]])


def make_planner(*, poly=SQUARE_130, offset=None, leg_m=25.0, reach=8.0,
                 stall_s=30.0, spacing=27.0, track_instances=True,
                 obstacles=None, clearance=3.0, lift_m=5.0, lifts=0,
                 min_alt=0.0, max_alt=0.0):
    p = StubPlanner()
    p._log = _Logger()
    p.get_logger = lambda: p._log
    p._search_poly = poly
    p._marker_offset = None if offset is None else np.array(
        [offset[0], offset[1], 0.0])
    # what _init_agents leaves behind for the lawnmower arm
    p._lm_spacing_used = float(spacing)
    p._lm_axis = 'auto'
    p._lm_reach = float(reach)
    p._lm_leg_m = float(leg_m)
    p._lm_stall_s = float(stall_s)
    # the 3D lawnmower: known obstacles, clearance, stall lifts, flight band
    p._obstacles = obstacles
    p._obst_clear = float(clearance)
    p._lm_lift_m = float(lift_m)
    p._lm_lifts = int(lifts)
    p._min_alt, p._max_alt = float(min_alt), float(max_alt)
    p._lm_leg_z = None
    # a fleet of one unless a test says otherwise (entry-edge choice reads it)
    p._sector_count = 1
    p._robots = ['robot_1']
    p._p_cached = lambda name, default: default
    # speed by intent: _command asks droan for 3.0 in transit, 1.5 on a target
    p._nav_mode = 'lawnmower'
    p._speeds = []
    p._explore_speed, p._target_speed = 3.0, 1.5
    # no third gear here: transit_speed_mps 0 is the shipped default, so the
    # sweep asks for explore/target only (test_search_area_source.py covers
    # the transit gear)
    p._transit_speed = 0.0
    p._search_poly_converted = True
    p._set_local_speed = lambda i, v: p._speeds.append(v)
    p._lm_path = lm.boustrophedon(poly, spacing)
    p._lm_needs_anchor = True
    p._lm_sweep = None
    p._lm_idx = 0
    p._lm_leg = None
    p._lm_last_pick_t = None
    p._lm_phase = None
    p._plan_period_s = 1.0
    p._track_instances = bool(track_instances)
    p._active_target = None
    p._goal_points = []
    # actuation, as configured for every arm
    p._plan_output = 'waypoints'
    p._path_extension_m = 2.0
    p._altitude = 12.0
    p._waypoint_z = lambda agent: p._altitude
    p._path_msg = lambda pts, offset, z=None: FakePath(pts, offset, z)
    p._plan_pubs = [FakePub()]
    p._nav_activation = 'activator'
    p._activations = []
    p._ensure_activator = lambda i: p._activations.append(i)
    return p


def offset_of(p):
    o = p._marker_offset
    return np.zeros(3) if o is None else o


def place(agent, p, map_xy):
    """Put the drone at a MAP-frame point: the grid holds map + offset."""
    o = offset_of(p)
    return agent.set_grid_xy(map_xy[0] + o[0], map_xy[1] + o[1])


def tick(p, agent):
    """One planner tick for the lawnmower arm: what `_tick` -> `_assign` ->
    `_command` do, with the target gate `_tick` applies."""
    if not p._on_target(agent):
        cell = p._lawnmower_pick(agent)
        p._goal_points = [cell if cell is not None else [1, 1]]
    p._command(0, agent, offset_of(p))
    return p._plan_pubs[0].last


def fly(p, agent, start_map, ticks, dt=1.0, stop_at_lap=None):
    """Teleport the drone to each published goal in turn."""
    cur = np.array(start_map, dtype=float)
    place(agent, p, cur)
    goals, seconds, phases = [], [], []
    for _ in range(ticks):
        path = tick(p, agent)
        g0, g1 = path.xy(0), path.xy(1)
        goals.append(g0)
        seconds.append(g1)
        phases.append(p._lm_sweep.phase)
        cur = g0.copy()
        place(agent, p, cur)
        CLOCK.advance(dt)
        if stop_at_lap is not None and p._lm_sweep.lap >= stop_at_lap:
            break
    return np.array(goals), np.array(seconds), phases


# ── tests ────────────────────────────────────────────────────────────────────

def test_01_lawnmower_module_selftest():
    """The geometry module's own proof, including the Sweep sequencer."""
    lm._selftest()


def test_02_one_short_goal_at_a_time_never_the_path():
    p = make_planner()
    agent = FakeAgent()
    res = agent.res_m
    path_len = float(np.sum(np.linalg.norm(np.diff(p._lm_path, axis=0), axis=1)))
    assert path_len > 1000.0, 'the test sector should need a path far longer than a leg'

    # Spawn at the block centre (suburb_mini's DRONE_XY=0,0): 130 m from the
    # nearest lane end, so there IS a transit to make.
    start = np.array([0.0, 0.0])
    place(agent, p, start)
    path = tick(p, agent)
    assert len(path.poses) == 2, f'{len(path.poses)} poses published: not a short goal'
    d0 = float(np.linalg.norm(path.xy(0) - start))
    assert d0 <= p._lm_leg_m + res, f'first goal {d0:.1f} m out > leg {p._lm_leg_m}'
    assert p._lm_sweep.phase == 'transit'
    assert p._activations == [0], 'the activator NavigateTask was not requested'
    entry = p._lm_sweep.loop[0]
    assert float(np.hypot(*(entry - start))) > 100.0, 'expected a real transit'
    # the second pose is the NEXT leg, not a 2 m extension of the approach
    assert float(np.linalg.norm(path.xy(1) - path.xy(0))) > 2.0 + 1e-6
    assert np.linalg.norm(path.xy(1) - p._lm_sweep.point(1)) <= 1e-6

    goals, seconds, phases = fly(p, agent, start, ticks=4000, stop_at_lap=1)
    assert p._lm_sweep.lap >= 1, 'did not complete a lap of the sector'
    hops = np.linalg.norm(np.diff(goals, axis=0), axis=1)
    assert hops.max() <= p._lm_leg_m + 2 * res, (
        f'a hop of {hops.max():.1f} m > leg {p._lm_leg_m}: a goal was not short')
    k = phases.index('sweep')
    assert phases[0] == 'transit' and 'transit' not in phases[k:], (
        'transit must be flown once, first')
    assert k == len(p._lm_sweep.transit), 'transit legs not flown in order'
    # every lane point of the boustrophedon was, at some tick, THE goal
    worst = max(min(np.linalg.norm(goals - v, axis=1)) for v in p._lm_path)
    cell = res * math.sqrt(2.0)             # per-axis floor -> diagonal of a cell
    assert worst <= cell + 1e-9, f'a lane point was never a goal (worst {worst:.2f} m)'
    # and the drone was never asked for something it was not standing near:
    # each second pose is the leg after the first
    assert np.all(np.linalg.norm(seconds - goals, axis=1) <= p._lm_leg_m + 2 * res)
    assert not p._log.grep('skipping'), 'a teleporting drone never stalls'
    print(f'    {len(goals)} goals, max hop {hops.max():.1f} m (leg '
          f'{p._lm_leg_m:.0f} m) over a {path_len:.0f} m lane path; '
          f'{k} transit legs then {len(p._lm_sweep.loop)} sweep goals; '
          'two poses per publish; activator requested once')


def test_03_regression_lane_goals_survive_the_grid_offset():
    """The lanes live in the MAP frame, the agent and `_goal_points` in the
    GRID frame. Before the fix `_lawnmower_pick` fed a map-frame waypoint to
    `map_xy_to_grid` directly and compared the grid-frame position against
    map-frame lanes, so on any scene with a map origin the drone was sent to
    the waypoint MINUS the offset and never registered arrival."""
    off = (37.0, -12.0)
    # A sector that still fits the +-139 m grid once shifted by the offset;
    # the full +-130 m square would not, and the map-edge clamp (test_07) is
    # a different behaviour from the one under test here.
    poly = SQUARE_130 * (80.0 / 130.0)
    p = make_planner(offset=off, poly=poly)
    agent = FakeAgent()
    # Off-centre: the square's lane ends are symmetric about the origin, and
    # from (0, 0) two are equidistant, so a sub-cell shift would flip a tie
    # that has nothing to do with the offset.
    start = np.array([5.0, -20.0])
    place(agent, p, start)
    path = tick(p, agent)
    goal_map = p._lm_sweep.point(0)
    sent = path.xy(0)
    assert np.linalg.norm(sent - goal_map) <= agent.res_m * math.sqrt(2.0) + 1e-9, (
        f'sent {sent} but the lane goal is {goal_map}')
    pre_fix = goal_map - np.array(off)
    assert np.linalg.norm(sent - pre_fix) > 30.0, 'the pre-fix answer came back'

    # The whole flight is offset-invariant: same goals, in the same order.
    g_off, _, _ = fly(p, agent, start, ticks=4000, stop_at_lap=1)
    assert not p._log.grep('outside the occupancy grid'), 'sector must fit the grid'
    p0 = make_planner(offset=None, poly=poly)
    g_zero, _, _ = fly(p0, FakeAgent(), start, ticks=4000, stop_at_lap=1)
    n = min(len(g_off), len(g_zero))
    assert n > 40 and abs(len(g_off) - len(g_zero)) <= 1, (len(g_off), len(g_zero))
    assert np.abs(g_off[:n] - g_zero[:n]).max() <= 2 * agent.res_m, (
        'goal sequence changed with the grid offset')
    print(f'    offset {off}: goal within {agent.res_m:.2f} m of the lane point; '
          f'{n}-goal sequence identical to the zero-offset flight')


def test_04_regression_recover_into_area_uses_the_same_transform():
    off = (37.0, -12.0)
    p = make_planner(offset=off)
    agent = FakeAgent()
    place(agent, p, (0.0, 0.0))
    cell = p._recover_into_area(agent, 5)
    back = np.array(p._to_map(*agent.grid_to_map_xy(*cell)))
    cx, cy = sect.polygon_centroid(p._search_poly)
    assert np.linalg.norm(back - (cx, cy)) <= agent.res_m * math.sqrt(2.0) + 1e-9, (
        back, (cx, cy))
    print(f'    recovery goal lands on the centroid ({cx:.0f}, {cy:.0f}) '
          f'through an offset of {off}')


def test_04b_speed_follows_intent():
    """3 m/s while sweeping, 1.5 m/s the moment a person is the goal, back
    to 3 when the detour ends — what `_command` asks droan for each tick."""
    p = make_planner()
    agent = FakeAgent()
    place(agent, p, (0.0, 0.0))
    tick(p, agent)
    assert p._speeds[-1] == 3.0
    p._active_target = (20.0, 20.0)
    tick(p, agent)
    assert p._speeds[-1] == 1.5
    p._active_target = None
    tick(p, agent)
    assert p._speeds[-1] == 3.0
    src = _slice(SRC, '    def _command', '    def _await')
    assert 'self._set_local_speed(' in src and 'self._nav_mode' not in src
    setter = _slice(SRC, '    def _set_local_speed', '    def _speed_label')
    assert "self._nav_mode == 'gpt'" in setter, 'the gpt arm must be left at one speed'
    assert 'self._send_activator(i' in setter, 'the cap rides on a re-sent NavigateTask goal'
    print('    transit 3.0 -> target 1.5 -> transit 3.0; gpt arm exempt')


def test_05_target_detour_pauses_the_sweep_and_resumes_without_a_skip():
    p = make_planner()
    agent = FakeAgent()
    goals, _, _ = fly(p, agent, (0.0, 0.0), ticks=12)
    idx_before = p._lm_sweep.idx
    # A detected person: _tick holds _assign off and _goal_xy flies the target.
    tgt_grid = (60.0, 40.0)
    p._active_target = tgt_grid
    agent.set_grid_xy(*tgt_grid)             # ... and the drone goes there
    for _ in range(40):                      # longer than lawnmower_stall_s
        path = tick(p, agent)
        CLOCK.advance(1.0)
    off = offset_of(p)
    assert np.allclose(path.xy(0), np.array(tgt_grid) - off[:2]), 'not flying the target'
    assert abs(np.linalg.norm(path.xy(1) - path.xy(0)) - p._path_extension_m) < 1e-6, (
        'the lane continuation must not be attached to a target goal')
    assert p._lm_sweep.idx == idx_before, 'the sweep advanced during the detour'
    # Target visited: back to the lanes, from where it left off, with no stall.
    p._active_target = None
    path = tick(p, agent)
    assert p._log.grep('resuming'), 'expected the resume log'
    assert not p._log.grep('skipping'), 'the detour was read as a stall'
    assert p._lm_sweep.idx == idx_before, 'the sweep did not resume where it left off'
    assert np.linalg.norm(path.xy(0) - p._lm_sweep.point(idx_before)) <= 1.0
    print(f'    40 s on a target: sweep index held at {idx_before}, lane '
          'continuation dropped from the target path, resumed with no skip')


def test_06_no_progress_skips_the_leg():
    p = make_planner(stall_s=30.0)
    agent = FakeAgent()
    place(agent, p, (0.0, 0.0))
    tick(p, agent)
    idx0 = p._lm_sweep.idx
    for _ in range(29):                      # stuck: drone never moves
        CLOCK.advance(1.0)
        tick(p, agent)
    assert p._lm_sweep.idx == idx0 and not p._log.grep('skipping')
    CLOCK.advance(2.0)
    tick(p, agent)
    assert p._lm_sweep.idx == idx0 + 1, 'stall did not skip the leg'
    assert p._log.grep('no progress on goal'), 'skip must be logged'
    print(f'    31 s without progress: goal #{idx0} skipped, #{idx0 + 1} sent')


def test_07_lane_ends_past_the_map_edge_are_clamped_not_stalled():
    """A padded sector wider than the map has its lane ends clipped by
    `map_xy_to_grid`. Reach must be judged against the goal actually sent."""
    wide = SQUARE_130 * (155.0 / 130.0)       # +-155 m in a +-139 m grid
    p = make_planner(poly=wide)
    agent = FakeAgent()
    goals, _, _ = fly(p, agent, (0.0, 0.0), ticks=4000, stop_at_lap=1)
    half = (agent.map_size // 2 - 2) * agent.res_m
    assert np.abs(goals).max() <= half + agent.res_m, 'a goal left the grid'
    assert p._log.grep('outside the occupancy grid'), 'the clamp must be logged'
    assert not p._log.grep('skipping'), (
        'clipped lane ends were waited on until the stall timer fired')
    assert p._lm_sweep.lap >= 1
    print(f'    +-155 m sector in a +-{half:.0f} m grid: goals clamped, '
          f'lap completed with no stall skips')


# ── the 3D lawnmower ─────────────────────────────────────────────────────────

def _boxes_on_lanes(p, n=3, half=6.0, top=18.0, cls='tree'):
    """Boxes sitting ON lane points (so legs must cross them), in map frame."""
    path = p._lm_path
    picks = [path[k] for k in range(2, len(path), max(1, len(path) // n))][:n]
    return clr.KnownObstacles(
        [(float(x), float(y), half, half, top, 0.0, cls) for x, y in picks],
        inflate_m=1.0), picks


def test_10_legs_over_known_obstacles_are_lifted_and_come_back_down():
    """Every published pose carries the height of ITS leg: cruise over open
    ground, box top + clearance over a known box, clamped to the band; the
    through pose carries the NEXT leg's height so the descent happens on
    the way to the next goal."""
    p0 = make_planner()
    ob, picks = _boxes_on_lanes(p0, n=3, top=18.0)
    p = make_planner(obstacles=ob, clearance=3.0, max_alt=25.0)
    agent = FakeAgent()
    goals, seconds, phases = fly(p, agent, (0.0, 0.0), ticks=4000, stop_at_lap=1)
    assert p._lm_sweep.lap >= 1
    # re-fly, recording z on both poses
    p = make_planner(obstacles=ob, clearance=3.0, max_alt=25.0)
    agent = FakeAgent()
    cur = np.array([0.0, 0.0]); place(agent, p, cur)
    zs, tz, gs = [], [], []
    for _ in range(4000):
        path = tick(p, agent)
        zs.append(path.z(0)); tz.append(path.z(1)); gs.append(path.xy(0))
        cur = path.xy(0).copy(); place(agent, p, cur); CLOCK.advance(1.0)
        if p._lm_sweep.lap >= 1:
            break
    zs, tz, gs = np.array(zs), np.array(tz), np.array(gs)
    assert set(np.round(zs, 3)) <= {12.0, 21.0}, sorted(set(np.round(zs, 3)))
    assert (zs == 21.0).sum() >= 3, 'no leg was lifted over a box'
    assert (zs == 12.0).sum() > (zs == 21.0).sum(), 'most legs must stay at cruise'
    # a lifted leg's goal is the leg that crosses a box: verify against the
    # obstacle field directly, both ways
    for k in range(1, len(gs)):
        top, _ = ob.top_along(gs[k - 1], gs[k])
        want = 21.0 if top is not None else 12.0
        assert abs(zs[k] - want) < 1e-9, (k, gs[k - 1], gs[k], zs[k], want)
    # the through pose is the NEXT leg's height: after a lifted leg whose next
    # leg is clear it is already 12 m, i.e. the descent starts at the goal
    down = [(k) for k in range(len(zs)) if zs[k] == 21.0 and tz[k] == 12.0]
    assert down, 'no lifted leg was followed by a descending through pose'
    # ... and equals what the next tick publishes as its goal height
    for k in range(len(zs) - 1):
        if p._lm_sweep.phase == 'sweep':
            pass
    mism = sum(1 for k in range(len(zs) - 1) if abs(tz[k] - zs[k + 1]) > 1e-9)
    assert mism <= 2, f'{mism} through-pose heights disagree with the next goal'
    print(f'    {len(zs)} goals: {(zs == 21.0).sum()} lifted to 21 m over '
          f'{len(picks)} boxes (top 18 + 3), {(zs == 12.0).sum()} at cruise; '
          f'{len(down)} descents start at the lifted goal')


def test_10b_reach_is_judged_in_xy_only():
    """A goal reached at ANY height is reached: `Sweep` sees only xy, so a
    drone that flew over a canopy is never asked back down onto it."""
    src = _slice(SRC_LM, '    def _reached', '    def _advance_to')
    assert 'cur_xy' in _slice(SRC_LM, '    def update', '    def _leg_z')
    assert 'z' not in src.replace('reach_m', '').replace('hypot', ''), src
    sw = lm.Sweep(np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 50.0]]), reach_m=8.0,
                  leg_m=25.0, z_fn=lambda p, q, lift: 30.0)
    g0 = sw.update(np.array([0.0, 0.0]))
    assert g0.z == 30.0
    g1 = sw.update(g0.xy)                   # 'arrived' — no z in the call at all
    assert g1.index == g0.index + 1
    print('    Sweep.update takes xy only; arrival at the goal xy advances '
          'regardless of the 30 m leg height')


def test_11_stall_lifts_before_it_skips():
    p = make_planner(stall_s=12.0, lifts=2, lift_m=5.0, max_alt=25.0)
    agent = FakeAgent()
    place(agent, p, (0.0, 0.0))
    path = tick(p, agent)
    idx0 = p._lm_sweep.idx
    z0 = path.z(0)
    assert z0 == 12.0
    # 13 s stuck -> lifted once, same goal
    for _ in range(13):
        CLOCK.advance(1.0); path = tick(p, agent)
    assert p._lm_sweep.idx == idx0 and path.z(0) == 17.0, (p._lm_sweep.idx, path.z(0))
    assert p._log.grep('lifting it to 17.0 m') and not p._log.grep('skipping')
    # another 13 s -> lifted twice
    for _ in range(13):
        CLOCK.advance(1.0); path = tick(p, agent)
    assert p._lm_sweep.idx == idx0 and path.z(0) == 22.0
    # another 13 s -> out of lifts: skipped, next goal back at cruise
    for _ in range(13):
        CLOCK.advance(1.0); path = tick(p, agent)
    assert p._lm_sweep.idx == idx0 + 1 and path.z(0) == 12.0
    assert p._log.grep('after 2 lift'), 'skip must say the lifts were used up'
    # the band caps a lift
    q = make_planner(stall_s=12.0, lifts=2, lift_m=5.0, max_alt=15.0)
    a2 = FakeAgent(); place(a2, q, (0.0, 0.0)); tick(q, a2)
    for _ in range(13):
        CLOCK.advance(1.0); path = tick(q, a2)
    assert path.z(0) == 15.0
    print('    12 s stall: 12 -> 17 -> 22 m on the same goal, then skipped; '
          'lift capped at max_altitude 15 m')


def test_11b_blocked_goal_lifts_then_skips_immediately():
    """`Sweep.blocked()` — the planner's per-tick voxel re-check found the
    current goal inside an obstacle: same ladder as a stall, applied now."""
    sw = lm.Sweep(np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 50.0], [0.0, 50.0]]),
                  reach_m=8.0, leg_m=25.0, z_fn=lambda p, q, lift: 12.0 + 5.0 * lift,
                  stall_lifts=2)
    g = sw.update(np.zeros(2), 0.0)
    k, z0 = g.index, g.z
    assert z0 == 12.0
    assert sw.blocked(1.0) == 'lifted' and sw.update(np.zeros(2), 1.5).z == 17.0
    assert sw.update(np.zeros(2), 1.5).index == k
    assert sw.blocked(2.0) == 'lifted' and sw.update(np.zeros(2), 2.5).z == 22.0
    assert sw.blocked(3.0) == 'skipped' and sw.update(np.zeros(2), 3.5).index == k + 1
    assert sw.update(np.zeros(2), 3.5).z == 12.0, 'the next goal starts at cruise'
    # without a z_fn there is nothing to lift: straight to skip
    sw2 = lm.Sweep(np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 50.0]]), reach_m=8.0, leg_m=25.0)
    k2 = sw2.update(np.zeros(2), 0.0).index
    assert sw2.blocked(1.0) == 'skipped' and sw2.update(np.zeros(2), 1.5).index == k2 + 1
    assert lm.Sweep(np.zeros((0, 2)), 5.0).blocked(0.0) is None
    print('    blocked(): 12 -> 17 -> 22 m on the same goal, then skipped; '
          'no z_fn -> skipped at once; empty sweep -> None')


def test_11c_neighbouring_drones_enter_at_opposite_edges():
    """Robot index 0 enters its sector at the lane end nearest the spawn,
    index 1 at the end nearest the sector corner FARTHEST from it — so two
    drones spawned together do not sweep their strips in step."""
    import os as _os
    entries = {}
    for name in ('robot_1', 'robot_2', 'robot_3'):
        _os.environ['ROBOT_NAME'] = name
        p = make_planner()
        p._sector_count = 3
        p._p_cached = lambda n, d: d
        agent = FakeAgent()
        place(agent, p, (0.0, 0.0))
        tick(p, agent)
        entries[name] = tuple(np.round(p._lm_sweep.loop[0], 1))
    _os.environ.pop('ROBOT_NAME', None)
    assert entries['robot_1'] == entries['robot_3'], entries
    assert entries['robot_1'] != entries['robot_2'], entries
    # opposite ends of the lane axis, not merely different points
    gap = float(np.linalg.norm(np.subtract(entries['robot_1'], entries['robot_2'])))
    assert gap > 50.0, f'entries should be at opposite edges: {entries}'
    print(f'    entries from (0,0): robot_1 {entries["robot_1"]}, '
          f'robot_2 {entries["robot_2"]} ({gap:.0f} m apart), robot_3 = robot_1')


def test_12_clearance_module_and_world_shift():
    clr._selftest()
    import json, tempfile
    doc = [{"class": "person", "bbox_world": {"center_xyz_m": [1, 2, 0.9], "size_xyz_m": [0.7, 0.7, 1.8]}},
           {"class": "house", "bbox_world": {"center_xyz_m": [-30.0, 25.0, 4.0], "size_xyz_m": [16.0, 12.0, 8.0]}},
           {"class": "tree", "bbox_world": {"center_xyz_m": [10.0, 10.0, 8.0], "size_xyz_m": [12.0, 12.0, 16.0]}},
           {"class": "car", "bbox_world": {"center_xyz_m": [0.0, 0.0, 0.7], "size_xyz_m": [4.5, 1.9, 1.4]}},
           {"class": "bogus"}]
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump(doc, f); f.close()
    boxes = clr.load_boxes(f.name)
    assert [b[6] for b in boxes] == ['house', 'tree', 'car'], boxes
    ob = clr.KnownObstacles(boxes, inflate_m=2.0)
    # world -> map for a map whose origin is at world (-30, 25): the house
    # is now centred on the map origin
    m = ob.shifted(-30.0, 25.0, 0.0)
    assert m.top_at((0.0, 0.0)) == (8.0, 0)
    assert ob.top_at((0.0, 0.0))[1] == 2, 'world frame: the car sits at the origin'
    # leg heights through it: house top 8 + 3 = 11 < cruise 12 -> cruise;
    # tree top 16 + 3 = 19
    assert clr.leg_z(m.top_along((-20, 0), (20, 0))[0], 12.0, 3.0) == 12.0
    assert clr.leg_z(m.top_along((30, -20), (50, -10))[0], 12.0, 3.0) == 19.0
    print('    person dropped, 3 obstacle boxes kept; world->map shift puts the '
          'house on the map origin; 8 m house under a 12 m cruise needs no lift, '
          '16 m tree -> 19 m')


def _yaml_scalars(path):
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


def test_08_config_legs_are_short_and_the_sector_fits_the_map():
    cfg = _yaml_scalars(os.path.join(_CONFIG, 'lawnmower.yaml'))
    leg = float(cfg['lawnmower_leg_m'])
    reach = float(cfg['lawnmower_reach_radius_m'])
    stall = float(cfg['lawnmower_stall_s'])
    pad = float(cfg['search_area_pad_m'])
    assert 0.0 < reach < leg <= 60.0, (reach, leg)
    assert stall > 0.0
    assert pad == 0.0, 'the lawnmower must fly the same area as the other arms'
    # every scene's search area (plus the lawnmower pad) fits its grid
    for scene in ('suburb_mini.yaml', 'modular_house.yaml'):
        path = os.path.join(_CONFIG, scene)
        if not os.path.exists(path):
            continue
        text = open(path, encoding='utf-8').read()
        m = re.search(r'search_area_xy:\s*\[([^\]]*)\]', text, re.S)
        e = re.search(r'map_extent_m:\s*([0-9.]+)', text)
        if not m or not e:
            continue
        nums = [float(v) for v in re.findall(r'-?[0-9.]+', m.group(1))]
        extent = float(e.group(1))
        assert max(abs(v) for v in nums) + pad <= extent / 2.0, (
            f'{scene}: sector +pad {max(abs(v) for v in nums) + pad:.0f} m '
            f'exceeds the {extent / 2:.0f} m half-map')
    print(f'    leg {leg:.0f} m > reach {reach:.0f} m, stall {stall:.0f} s, '
          'pad 0; sectors fit their grids')


def test_09_same_actuation_as_the_frontier_arms():
    import ast
    pick = _slice(SRC, '    def _lawnmower_pick', '    def _commit')
    fn = ast.parse('class _T:\n' + pick).body[0].body[0]
    lines = pick.splitlines()
    doc = fn.body[0]
    if isinstance(doc, ast.Expr) and isinstance(doc.value, ast.Constant):
        # line numbers are 1-based and offset by the prepended class line
        del lines[doc.lineno - 2:doc.end_lineno - 1]
    pick = '\n'.join(ln.split('#', 1)[0] for ln in lines)
    for forbidden in ('NavigateTask', 'send_goal', '_plan_pubs', 'Path(',
                      '_nav_clients'):
        assert forbidden not in pick, f'_lawnmower_pick actuates on its own: {forbidden}'
    cmd = _slice(SRC, '    def _command', '    def _await')
    assert 'self._nav_mode' not in cmd and 'lawnmower' not in cmd, (
        'actuation branches on the method')
    assert 'through_xy=self._through_xy(agent)' in cmd
    tick_src = _slice(SRC, '    def _tick', '    def _voxel_frontiers')
    assert tick_src.count('self._nav_mode ==') == 1, 'the arms no longer share the tick'
    asn = _slice(SRC, '    def _assign', '    def _record_round')
    assert '_lawnmower_pick(self._agents[i])' in asn
    assert '_vlfm_pick(target_point_list, self._agents[i])' in asn
    through = _slice(SRC, '    def _through_xy', '    def _agent_xy')
    assert 'self._nav_mode' not in through
    print('    lawnmower picks a cell into _goal_points and _command does the '
          'rest — no nav_mode in the actuation, one tick for every arm')


if __name__ == '__main__':
    import pytest
    sys.exit(pytest.main([__file__, '-s', '-v']))
