"""Offline unit tests for the `frontier` nav_mode of the shared search planner.

NO Isaac Sim, NO GPU, NO ROS. Pure python + numpy + scipy.

THE CLAIM UNDER TEST. The `frontier` arm is "VLFM without the VLM": the same
3D voxel frontiers, the same sector restriction, the same commitment and the
same actuation, with the value-map lookup replaced by

    score = information_gain - vlfm_distance_penalty * distance
    score = VisitCost.score(score, i, j)          # ... - weight * visit_cost

HOW THE PLANNER'S OWN CODE IS REACHED. `planner_node.py` imports rclpy,
cv_bridge and open3d at module scope, so it cannot be imported outside the
container. The methods under test touch none of that, so their SOURCE is sliced
out of the file and exec'd onto a stub class. That runs the REAL code — edit
`_frontier_pick` and these tests see the edit — with a handful of `self._*`
attributes supplied by hand. `airstack_agent.py` has the same problem (it
imports the vendored upstream tree) and its two grid<->metre transforms are
sliced the same way, so the test agent's geometry is the flight geometry rather
than a re-derivation of it.

`visit_cost.py`, `voxel_map.py` and `sector.py` import standalone and are used
directly.

Run:  python3 tests/test_frontier_baseline.py
  or: python3 -m pytest tests/test_frontier_baseline.py -s
"""

import ast
import math
import os
import sys
import types

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from search_baselines import sector as sect          # noqa: E402
from search_baselines.visit_cost import VisitCost    # noqa: E402
from search_baselines.voxel_map import VoxelMap      # noqa: E402

_PLANNER_PY = os.path.join(_PKG_ROOT, 'search_baselines', 'planner_node.py')
_AGENT_PY = os.path.join(_PKG_ROOT, 'search_baselines', 'airstack_agent.py')
_CONFIG = os.path.join(_PKG_ROOT, 'config')

with open(_PLANNER_PY, encoding='utf-8') as _f:
    SRC = _f.read()
with open(_AGENT_PY, encoding='utf-8') as _f:
    AGENT_SRC = _f.read()

# Findings that are not test failures: behaviour that is real, reproducible and
# worth an operator's attention, printed as a block at the end of the run.
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
    """Stands in for the `time` module inside the sliced code, so `_commit`'s
    lock timer is driven by the test rather than by the wall clock."""

    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


CLOCK = _Clock()

# `sect` because `_recover_into_area` (in the vlfm/frontier slice) takes the
# sector centroid from it.
_NS = {'np': np, 'math': math, 'time': CLOCK, 'sect': sect}

# o3d_xz_to_map_xy is a module-level helper in airstack_agent that the sliced
# `_goal_xy` calls; take the real one.
exec(compile(_slice(AGENT_SRC, 'def o3d_xz_to_map_xy', 'class AirStackAgent'),
             '<airstack_agent:o3d_xz_to_map_xy>', 'exec'), _NS)

_AGENT_BODY = _slice(AGENT_SRC, '    def grid_to_map_xy', '    def plan_path_map_xy')
exec(compile('class _StubAgent:\n' + _AGENT_BODY,
             '<airstack_agent:grid transforms>', 'exec'), _NS)

_RANK_SRC = _slice(SRC, '    def _rank_frontiers', '    # ── VLFM')

_PLANNER_BODY = ''.join([
    _RANK_SRC,
    _slice(SRC, '    @staticmethod\n    def _points_in_polygon',
           '    def _vlfm_keyframe'),
    _slice(SRC, '    def _vlfm_pick', '    def _frontier_pick'),
    _slice(SRC, '    def _frontier_pick', '    def _lawnmower_pick'),
    _slice(SRC, '    def _commit', '    def _crop_to_map'),
    _slice(SRC, '    def _agent_xy', '    def _frontier_altitude'),
    _slice(SRC, '    def _goal_xy', '    def _build_path'),
])
exec(compile('class _StubPlanner:\n' + _PLANNER_BODY,
             '<planner_node:selection>', 'exec'), _NS)

# The pre-fix grouping, for the ties-to-even regression. Built by substituting
# ONE expression in the real source, so it stays a faithful "what it used to do"
# rather than a hand-written imitation.
_FLOOR_EXPR = 'int(float(fr[k, 2]) // self._vox_size)'
_ROUND_EXPR = 'int(round(float(fr[k, 2]) / self._vox_size))'
assert _FLOOR_EXPR in _RANK_SRC, (
    '_rank_frontiers no longer groups levels with floor division; the '
    'ties-to-even regression test below is pinned to that expression')
exec(compile('class _RoundRankPlanner:\n'
             + _RANK_SRC.replace(_FLOOR_EXPR, _ROUND_EXPR),
             '<planner_node:_rank_frontiers pre-fix>', 'exec'), _NS)

StubPlanner = _NS['_StubPlanner']
RoundRankPlanner = _NS['_RoundRankPlanner']


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_kw):
        self.lines.append(str(msg))

    info = warn = warning = error = debug = _log


class FakeAgent(_NS['_StubAgent']):
    """Only the state the sliced planner methods read off an agent."""

    def __init__(self, map_size=96, res_cm=500):
        self.args = types.SimpleNamespace(map_resolution=res_cm)
        self.map_size = int(map_size)
        self.origins_grid = (self.map_size // 2, self.map_size // 2)
        self.current_grid_pose = [self.map_size // 2, self.map_size // 2]
        self.found_goal = False
        self.nearest_point = None

    @property
    def res_m(self):
        return self.args.map_resolution / 100.0

    def set_xy(self, x, y):
        self.current_grid_pose = self.map_xy_to_grid(x, y)
        return self

    def xy(self):
        return self.grid_to_map_xy(*self.current_grid_pose)


def make_planner(*, poly=None, visit_cost=None, dist_penalty=0.0,
                 stratify=True, max_frontiers=12, vox_size=2.0,
                 lock_s=6.0, swap_frac=0.35, unlock_radius=8.0,
                 track_instances=True):
    p = StubPlanner()
    p._log = _Logger()
    p.get_logger = lambda: p._log
    p._search_poly = poly
    p._visit_cost = visit_cost
    p._vlfm_dist_penalty = float(dist_penalty)
    p._fr_z_stratify = bool(stratify)
    p._max_frontiers = int(max_frontiers)
    p._vox_size = float(vox_size)
    p._lock_s = float(lock_s)
    p._swap_frac = float(swap_frac)
    p._unlock_radius = float(unlock_radius)
    p._locked_goal = None
    p._locked_goal_xy = None
    p._locked_score = None
    p._locked_at = 0.0
    p._track_instances = bool(track_instances)
    p._active_target = None
    # grid -> map offset the sector test and the recovery goal go through;
    # None is 'first tick not yet run', the identity.
    p._marker_offset = None
    p._goal_points = []
    # vlfm-side attributes, so the same stub can drive `_vlfm_pick` for parity.
    p._value_map = None
    p._vlfm_min_conf = 0.02
    return p


def make_visit_cost(agent, radius_m=25.0, weight=1.0, decay_per_s=0.0):
    return VisitCost(agent.map_size, agent.res_m, agent.origins_grid,
                     radius_m=radius_m, decay_per_s=decay_per_s, weight=weight)


def cells_for(agent, xys):
    """Map-frame points -> the grid cells the planner would offer as candidates,
    in the order given (list position IS the information-gain rank)."""
    return [list(agent.map_xy_to_grid(x, y)) for x, y in xys]


def pick(p, cells, agent):
    """One selection with the commitment released, so the score is what decides."""
    p._locked_goal = None
    p._locked_goal_xy = None
    p._locked_score = None
    return p._frontier_pick(cells, agent)


def rank_gain(k, n):
    """The gain `_frontier_pick` derives from list position, restated."""
    return 1.0 - (float(k) / max(1, n - 1)) if n > 1 else 1.0


# ── 1. information gain is maximised ─────────────────────────────────────────

def test_01_maximises_information_gain():
    """No visit cost, no distance penalty -> the top-ranked candidate wins."""
    agent = FakeAgent()
    for n in (1, 2, 3, 5, 12, 50):
        xys = [(-200.0 + 8.0 * k, 30.0 * math.sin(0.7 * k)) for k in range(n)]
        cells = cells_for(agent, xys)
        # (a) visit cost absent entirely
        p = make_planner()
        got = pick(p, cells, agent)
        assert got == cells[0], f'n={n}: no-visit-cost pick {got} != {cells[0]}'
        # (b) visit cost present but zero-weight: identical decision
        vc = make_visit_cost(agent, weight=0.0)
        for x, y in xys:
            vc.visit((x, y))          # every candidate visited, weight 0
        p = make_planner(visit_cost=vc)
        got = pick(p, cells, agent)
        assert got == cells[0], f'n={n}: zero-weight pick {got} != {cells[0]}'
    print('    argmax(score) == candidate 0 for n in 1,2,3,5,12,50, '
          'with and without a zero-weight VisitCost')


def test_01b_gain_is_ordinal_rank_not_measured_gain():
    """The gain `_frontier_pick` maximises is LIST POSITION, not the voxel map's
    unobserved-neighbour count. Two consequences are pinned here because they
    decide how the visit-cost trade-off behaves.

    (i) The gap between adjacent ranks is a constant 1/(n-1), whatever the true
        gains are. A candidate whose measured gain is 1 unobserved neighbour
        behind the leader and one that is 400 behind score the same if they sit
        at the same rank.
    (ii) With `frontier_z_stratify` on -- the default -- `_rank_frontiers`
         interleaves heights, so list position is NOT gain order. Rank 0 is
         still the global gain argmax, but rank 1 onwards is a stratified
         order, so the rank-derived gain is not monotone in measured gain.
    """
    n = 6
    gaps = {round(rank_gain(k, n) - rank_gain(k + 1, n), 12) for k in range(n - 1)}
    assert gaps == {round(1.0 / (n - 1), 12)}, gaps

    # Two frontiers per height, three heights, at voxel centres (odd multiples
    # of half the voxel size) as the voxel map produces them.
    p = make_planner(stratify=True, max_frontiers=6, vox_size=2.0)
    fr = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0],
                   [0.0, 10.0, 3.0], [10.0, 10.0, 3.0],
                   [0.0, 20.0, 5.0], [10.0, 20.0, 5.0]])
    gain = np.array([100.0, 90.0, 80.0, 70.0, 60.0, 50.0])
    order = p._rank_frontiers(fr, gain)
    listed = gain[order]
    assert order[0] == int(np.argmax(gain)), 'rank 0 is not the global gain argmax'
    assert [float(v) for v in listed] == [100.0, 80.0, 60.0, 90.0, 70.0, 50.0], \
        list(listed)
    assert not all(listed[k] >= listed[k + 1] for k in range(len(listed) - 1)), (
        'stratified list came back in gain order; the example no longer '
        'exercises the interleave')
    # The proxy the selector then uses, against the truth it stands for.
    proxy = [rank_gain(k, len(order)) for k in range(len(order))]
    print(f'    stratified list: measured gain {[float(v) for v in listed]}')
    print(f'                     rank proxy    {[round(v, 2) for v in proxy]}')
    note('gain proxy',
         'planner_node.py:1523 derives information gain from LIST POSITION '
         '("the voxel path already ranks candidates by unobserved-neighbour '
         'count, so position in the list IS the ranking"). That comment holds '
         'only when frontier_z_stratify is off. With it ON (the default, '
         'planner_node.py:358) _rank_frontiers interleaves heights, so rank 1+ '
         'is a stratified order and the proxy is not monotone in measured '
         'gain. Rank 0 -- the pick when no visit cost applies -- is still the '
         'global gain argmax, so the argmax claim survives; the ORDERING that '
         'the visit-cost and distance terms trade against does not.')
    note('gain proxy',
         'The proxy also discards magnitude: adjacent ranks are always '
         '1/(n-1) apart, so a 1-neighbour gain gap and a 400-neighbour gain '
         'gap buy the same resistance to visit cost.')


# ── 2. visit cost moves it on, and excludes nothing ──────────────────────────

def _spread_candidates(agent, n=5, step=70.0):
    """n candidates far enough apart that their visit stamps do not overlap,
    centred so none of them is off the grid.

    `map_xy_to_grid` CLIPS to [1, size-2], so a candidate outside the map comes
    back as a cell on the border and two distant candidates silently collapse
    onto adjacent cells. The assertion below is what caught that.
    """
    xys = [((k - 0.5 * (n - 1)) * step + 3.7, 1.3) for k in range(n)]
    cells = cells_for(agent, xys)
    for (x, y), c in zip(xys, cells):
        rx, ry = agent.grid_to_map_xy(*c)
        assert abs(rx - x) <= agent.res_m and abs(ry - y) <= agent.res_m, (
            f'candidate ({x}, {y}) was clipped to cell {c} = ({rx}, {ry}); '
            f'the test layout does not fit the {agent.map_size}-cell grid')
    return xys, cells


def test_02_visit_cost_moves_on_and_blacklists_nothing():
    agent = FakeAgent()
    xys, cells = _spread_candidates(agent, n=5)

    vc = make_visit_cost(agent, weight=1.0)
    p = make_planner(visit_cost=vc)
    assert pick(p, cells, agent) == cells[0], 'clean field should take rank 0'

    # Deposit on candidate 0's ground -> its normalised cost is 1, the rest 0.
    vc.visit(xys[0])
    assert abs(vc._normalised_at(*cells[0]) - 1.0) < 1e-6
    assert vc._normalised_at(*cells[1]) == 0.0
    got = pick(p, cells, agent)
    assert got == cells[1], f'visited rank 0 still selected: {got}'
    # ... and the shift is a SCORE shift, not an exclusion: candidate 0 is
    # still in the candidate set and still scored.
    scores = [vc.score(rank_gain(k, len(cells)), *cells[k])
              for k in range(len(cells))]
    assert scores[0] == rank_gain(0, 5) - 1.0, scores
    assert all(math.isfinite(s) for s in scores), scores

    # NEAR, not at: the raised-cosine stamp means a deposit one cell away still
    # taxes the candidate, so the shift is not an artefact of an exact hit.
    vc2 = make_visit_cost(agent, weight=1.0)
    p2 = make_planner(visit_cost=vc2)
    vc2.visit((xys[0][0] + 8.0, xys[0][1] - 6.0))
    assert vc2._normalised_at(*cells[0]) > 0.5
    assert pick(p2, cells, agent) == cells[1], 'near-miss deposit did not shift'

    # EQUALISATION restores it, on the honest trigger. Sweep every candidate
    # repeatedly: the one extra deposit rank 0 carries is diluted, the spread
    # of normalised cost collapses toward 0, a term that is near-identical
    # across candidates cannot change an argmax, and selection reverts to gain.
    spreads = []
    for _ in range(20):
        for x, y in xys:
            vc.visit((x, y))
        n = [vc._normalised_at(*c) for c in cells]
        spreads.append(max(n) - min(n))
    assert spreads[0] > spreads[-1], spreads
    assert spreads[-1] < 0.06, spreads[-1]
    got = pick(p, cells, agent)
    assert got == cells[0], (
        f'costs equalised (spread {spreads[-1]:.4f}) but rank 0 was not '
        f'selectable again: {got}')
    print(f'    visited rank 0 -> pick {cells[1]}; cost spread over 20 further '
          f'sweeps {spreads[0]:.3f} -> {spreads[-1]:.3f} -> pick {cells[0]} '
          f'again (nothing was excluded, only taxed)')


def test_02b_decay_alone_cannot_restore_a_visited_candidate():
    """`VisitCost.score` divides by the field's RUNNING PEAK, and `decay`
    scales the field and the peak by the same factor. Uniform exponential
    forgetting is therefore a NO-OP on selection: it changes no normalised
    cost and no argmax. Decay only bites RELATIVE to fresh deposits.

    This is not a failure -- the default is decay_per_s: 0 -- but the config
    comment reads as if decay alone brings old ground back."""
    agent = FakeAgent()
    xys, cells = _spread_candidates(agent, n=5)

    vd = make_visit_cost(agent, weight=1.0, decay_per_s=0.05)
    p = make_planner(visit_cost=vd)
    vd.visit(xys[0])
    before = [vd._normalised_at(*c) for c in cells]
    assert pick(p, cells, agent) == cells[1]
    vd.decay(600.0)                      # ten minutes of forgetting
    after = [vd._normalised_at(*c) for c in cells]
    assert max(abs(a - b) for a, b in zip(before, after)) < 1e-6, (before, after)
    assert pick(p, cells, agent) == cells[1], (
        'uniform decay changed the pick; the peak normalisation no longer '
        'cancels')

    # Fresh mass elsewhere IS what makes the old visit cheap again.
    vd.visit(xys[2])
    vd.visit(xys[2])
    rel = vd._normalised_at(*cells[0])
    assert rel < 0.9, f'aged visit still at full relative cost {rel}'
    print(f'    decay(600 s) alone: normalised cost unchanged '
          f'{[round(v, 3) for v in after]}; after fresh deposits elsewhere the '
          f'aged candidate falls to {rel:.3f}')
    note('visit cost decay',
         'VisitCost.score normalises by the running peak and VisitCost.decay '
         'scales field and peak together (visit_cost.py:193-194), so uniform '
         'decay changes no score and no argmax. config/frontier.yaml says '
         '"Non-zero makes long-ago-visited ground attractive again sooner"; '
         'it does so only relative to NEWER deposits, never on its own. '
         'Harmless at the shipped default of 0.')


# ── 3. re-sweep ──────────────────────────────────────────────────────────────

def test_03_resweep_returns_to_visited_ground():
    """A long run with a FIXED candidate set and FIXED gains -- the hardest
    case for the arm, because without a visit cost it would park on rank 0
    forever. Fly to whatever is picked, stamp the ground, repeat."""
    agent = FakeAgent()
    xys, cells = _spread_candidates(agent, n=6, step=70.0)
    vc = make_visit_cost(agent, radius_m=25.0, weight=1.0)
    p = make_planner(visit_cost=vc)

    seq = []
    for _ in range(48):
        got = pick(p, cells, agent)
        assert got is not None
        k = cells.index(got)
        seq.append(k)
        agent.set_xy(*xys[k])            # arrived
        vc.visit(xys[k])                 # and looked at the ground

    # It does not park on the best candidate...
    assert seq[0] == 0, seq
    assert seq[1] != 0, f'never left the first pick: {seq}'
    assert seq.count(0) < len(seq) // 2, f'parked on rank 0: {seq}'
    # ... it opens the sector out in rank order first ...
    assert seq[:5] == [0, 1, 2, 3, 4], seq[:5]
    # ... and it comes BACK. Every candidate it can reach (see test_03c) is
    # selected again and again, which is the second and third pass the
    # detector needs.
    reachable = [k for k in range(len(cells))
                 if (k / (len(cells) - 1.0)) < vc.weight]
    for k in reachable:
        assert seq.count(k) >= 2, (
            f'candidate {k} was never re-selected in {len(seq)} picks: {seq}')
    assert seq.index(0, 1) < 12, f'took too long to re-sweep the best: {seq}'
    st = vc.stats()
    print(f'    48 picks over 6 fixed candidates: {seq}')
    print(f'    counts {[seq.count(k) for k in range(len(cells))]}; '
          f'visit coverage {st["coverage"] * 100:.1f}%, peak {st["max"]:.2f}')


def test_03b_resweep_survives_the_distance_penalty():
    """Same run with the shipped penalty (0.002/m). The penalty must not turn
    into a de-facto blacklist by pinning the drone to one corner."""
    agent = FakeAgent()
    xys, cells = _spread_candidates(agent, n=6, step=70.0)
    vc = make_visit_cost(agent, radius_m=25.0, weight=1.0)
    p = make_planner(visit_cost=vc, dist_penalty=0.002)
    seq = []
    for _ in range(40):
        k = cells.index(pick(p, cells, agent))
        seq.append(k)
        agent.set_xy(*xys[k])
        vc.visit(xys[k])
    assert len(set(seq)) == len(cells), f'penalty pinned the search: {seq}'
    assert seq.count(0) >= 2, f'best candidate never re-swept: {seq}'
    print(f'    with vlfm_distance_penalty=0.002: {seq}')


def test_03c_reachability_bound_of_the_rank_proxy():
    """WHERE THE "no blacklist" CLAIM IS TIGHT. The rank-derived gain spans
    exactly [0, 1] and the normalised visit cost spans exactly [0, 1], so
    candidate k can only ever out-score candidate 0 if

        weight * (cost_0 - cost_k) > gain_0 - gain_k = k / (n - 1)

    and `cost_0 - cost_k <= 1`. Candidate k is therefore UNREACHABLE while the
    list holds still, for every k with k / (n - 1) >= weight. At the shipped
    weight of 1.0 that is exactly the last entry in the list; at 0.5 it is the
    whole back half. Pinned here because it is the one sense in which the arm
    does exclude a candidate outright."""
    agent = FakeAgent()
    n = 6
    for w in (0.5, 0.8, 1.0, 1.5):
        agent.set_xy(0.0, 0.0)
        xys, cells = _spread_candidates(agent, n=n, step=70.0)
        vc = make_visit_cost(agent, radius_m=25.0, weight=w)
        p = make_planner(visit_cost=vc)
        seq = []
        for _ in range(60):
            k = cells.index(pick(p, cells, agent))
            seq.append(k)
            agent.set_xy(*xys[k])
            vc.visit(xys[k])
        predicted = [k for k in range(n) if k / (n - 1.0) < w]
        assert sorted(set(seq)) == predicted, (
            f'weight {w}: reached {sorted(set(seq))}, bound predicts '
            f'{predicted}')
        print(f'    weight {w}: reachable ranks {predicted} '
              f'(bound k/(n-1) < weight), observed {sorted(set(seq))}')
    note('visit cost weight',
         'The rank proxy floors at 0 and the normalised visit cost ceilings at '
         '1, so with visit_cost_weight <= 1.0 (frontier.yaml ships 1.0) a '
         'candidate at rank k is unreachable whenever k/(n-1) >= weight: the '
         'last entry of every candidate list at weight 1.0, the back half at '
         'weight 0.5. It is not ground that is excluded -- the list is '
         're-ranked every tick, so the same region is reachable at a better '
         'rank next tick (test_03d) -- but the docstring\'s "nothing is ever '
         'removed from consideration" is true of the SET and not of the list '
         'positions. Raise visit_cost_weight above 1.0 to make the whole list '
         'reachable within a single tick.')


def test_03d_nothing_is_permanently_excluded_when_the_list_re_ranks():
    """The real candidate list is re-extracted and re-ranked every tick as the
    map fills, so a region sitting at the unreachable tail this tick sits
    higher next tick. With that rotation every region is swept, repeatedly --
    which is the claim that actually matters."""
    agent = FakeAgent()
    agent.set_xy(0.0, 0.0)
    xys, cells = _spread_candidates(agent, n=6, step=70.0)
    vc = make_visit_cost(agent, radius_m=25.0, weight=1.0)
    p = make_planner(visit_cost=vc)
    seq = []
    for step in range(60):
        rot = list(range(step % 6, 6)) + list(range(step % 6))
        got = pick(p, [cells[i] for i in rot], agent)
        k = cells.index(got)
        seq.append(k)
        agent.set_xy(*xys[k])
        vc.visit(xys[k])
    counts = [seq.count(k) for k in range(6)]
    assert min(counts) >= 3, f'a region was starved: {counts} / {seq}'
    assert max(counts) <= 3 * min(counts), f'coverage badly skewed: {counts}'
    print(f'    with the list re-ranked each tick: counts {counts} over '
          f'{len(seq)} picks -- every region swept repeatedly')


# ── 4. distance penalty ──────────────────────────────────────────────────────

def test_04_distance_penalty_prefers_the_nearer_candidate():
    """Adjacent ranks cannot have equal gain -- the proxy is 1/(n-1) apart by
    construction (see test_01b) -- so the property tested is the CROSSOVER: the
    lower-ranked, nearer candidate wins exactly once the penalty pays for the
    gain gap, and the crossover sits where the arithmetic says it does."""
    agent = FakeAgent()
    agent.set_xy(0.0, 0.0)
    here = np.array(agent.xy())
    n = 5
    # rank 0 far away, rank 1 close, the rest far and irrelevant.
    xys = [(300.0, 0.0), (20.0, 0.0), (320.0, 40.0), (330.0, -40.0), (340.0, 0.0)]
    cells = cells_for(agent, xys)
    xy = np.array([agent.grid_to_map_xy(*c) for c in cells])
    d0 = float(np.linalg.norm(xy[0] - here))
    d1 = float(np.linalg.norm(xy[1] - here))
    dgain = rank_gain(0, n) - rank_gain(1, n)
    crossover = dgain / (d0 - d1)

    p = make_planner(dist_penalty=0.0)
    assert pick(p, cells, agent) == cells[0], 'penalty 0 must be pure gain'
    for frac, expect in ((0.5, 0), (0.9, 0), (1.1, 1), (2.0, 1), (10.0, 1)):
        p = make_planner(dist_penalty=crossover * frac)
        got = pick(p, cells, agent)
        assert got == cells[expect], (
            f'penalty {crossover * frac:.5f} (x{frac} of crossover '
            f'{crossover:.5f}): picked {got}, expected {cells[expect]}')

    # Equal-distance control: with both at the same range the penalty cannot
    # change anything, at any magnitude.
    xys_eq = [(200.0, 0.0), (-200.0, 0.0), (0.0, 200.0)]
    cells_eq = cells_for(agent, xys_eq)
    for pen in (0.0, 0.002, 0.05, 5.0):
        p = make_planner(dist_penalty=pen)
        assert pick(p, cells_eq, agent) == cells_eq[0], (
            f'equal-distance candidates reordered at penalty {pen}')

    # Monotone: raising the penalty never sends the drone further away.
    p_far = []
    for pen in np.linspace(0.0, 4.0 * crossover, 25):
        p = make_planner(dist_penalty=float(pen))
        k = cells.index(pick(p, cells, agent))
        p_far.append(float(np.linalg.norm(xy[k] - here)))
    assert all(p_far[i] >= p_far[i + 1] - 1e-9 for i in range(len(p_far) - 1)), p_far
    print(f'    d(rank0)={d0:.0f} m d(rank1)={d1:.0f} m, gain gap {dgain:.2f} '
          f'-> crossover penalty {crossover:.5f}/m; switch observed there, '
          f'chosen distance monotone non-increasing in the penalty')


# ── 5. sector restriction ────────────────────────────────────────────────────

_SQUARE = np.array([[-100.0, -100.0], [100.0, -100.0],
                    [100.0, 100.0], [-100.0, 100.0]])


def test_05_sector_restriction():
    agent = FakeAgent()
    agent.set_xy(0.0, 0.0)

    # Best-gain candidate outside -> the best INSIDE candidate is taken.
    xys = [(180.0, 0.0), (50.0, 10.0), (-30.0, -40.0), (0.0, 175.0)]
    cells = cells_for(agent, xys)
    p = make_planner(poly=_SQUARE)
    got = pick(p, cells, agent)
    assert got == cells[1], f'out-of-sector rank 0 was selected: {got}'
    # every offered pick is inside, over many random candidate sets
    rng = np.random.default_rng(7)
    for _ in range(200):
        pts = rng.uniform(-400.0, 400.0, size=(8, 2))
        cs = cells_for(agent, [tuple(v) for v in pts])
        got = pick(p, cs, agent)
        if got is None:
            assert not StubPlanner._points_in_polygon(
                np.array([agent.grid_to_map_xy(*c) for c in cs]), _SQUARE).any()
            continue
        gx, gy = agent.grid_to_map_xy(*got)
        assert bool(StubPlanner._points_in_polygon(
            np.array([[gx, gy]]), _SQUARE)[0]), f'picked ({gx}, {gy}) out of sector'

    # ALL outside -> steer back INSIDE (the sector centroid), not None: None
    # would hold the last goal and park a robot on the boundary for the rest
    # of the run (see _recover_into_area).
    out = cells_for(agent, [(500.0, 500.0), (-600.0, 300.0), (0.0, -450.0)])
    got = pick(p, out, agent)
    assert got is not None, 'all-outside must recover, not hold'
    cx, cy = sect.polygon_centroid(_SQUARE)
    gx, gy = agent.grid_to_map_xy(*got)
    assert math.dist((gx, gy), (cx, cy)) <= agent.res_m * math.sqrt(2.0) + 1e-9, (
        f'recovery goal ({gx:.1f}, {gy:.1f}) is not the centroid ({cx:.0f}, {cy:.0f})')
    assert any('outside the search area' in ln for ln in p._log.lines)

    # No polygon at all -> unbounded, rank 0 wins wherever it is.
    p_free = make_planner(poly=None)
    assert pick(p_free, out, agent) == out[0]

    # A real strip sector from sector.py: robot 0 must refuse robot 1's ground.
    padded = np.asarray(sect.pad_polygon(_SQUARE, 25.0), dtype=float)
    s0 = np.asarray(sect.sector_for(_SQUARE, 2, 0, mode='strips', margin_m=25.0),
                    dtype=float)
    s1 = np.asarray(sect.sector_for(_SQUARE, 2, 1, mode='strips', margin_m=25.0),
                    dtype=float)
    assert len(s0) >= 3 and len(s1) >= 3
    c0 = sect.polygon_centroid(s0)
    c1 = sect.polygon_centroid(s1)
    p0 = make_planner(poly=s0)
    cs = cells_for(agent, [tuple(c1), tuple(c0)])
    got = p0._frontier_pick(cs, agent) if (p0._locked_goal is None) else None
    assert got == cs[1], (
        f'robot 0 selected a candidate at the centroid of robot 1\'s strip: {got}')
    # and the 25 m pad really did grow the searchable area
    assert sect.polygon_area(padded) > sect.polygon_area(_SQUARE)
    print(f'    square sector: out-of-bounds rank 0 rejected, 200 random sets '
          f'all in-bounds, all-outside -> centroid; strips sector 0 refuses '
          f'sector 1 centroid ({c1[0]:.0f}, {c1[1]:.0f})')


# ── 6. z-stratification ──────────────────────────────────────────────────────

def _levels_of(p, fr, order):
    return [int(float(fr[k, 2]) // p._vox_size) for k in order]


def test_06_z_stratification():
    fr = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0], [20.0, 0.0, 1.0],
                   [0.0, 10.0, 3.0], [10.0, 10.0, 3.0],
                   [0.0, 20.0, 5.0], [10.0, 20.0, 5.0]])
    gain = np.array([100.0, 95.0, 90.0, 80.0, 70.0, 60.0, 50.0])

    # off -> pure gain order, capped
    p = make_planner(stratify=False, max_frontiers=4, vox_size=2.0)
    order = p._rank_frontiers(fr, gain)
    assert list(order) == [0, 1, 2, 3], list(order)
    assert list(gain[order]) == sorted(gain[order])[::-1]

    # on -> best-per-height first, rank 0 still the global best
    p = make_planner(stratify=True, max_frontiers=7, vox_size=2.0)
    order = p._rank_frontiers(fr, gain)
    assert order[0] == 0, 'rank 0 is not the global gain argmax'
    assert [float(v) for v in gain[order]] == \
        [100.0, 80.0, 60.0, 95.0, 70.0, 50.0, 90.0], list(gain[order])
    lv = _levels_of(p, fr, order)
    assert len(set(lv[:3])) == 3, f'first three picks share a height: {lv}'
    assert set(lv) == {0, 1, 2}, lv

    # cap honoured, and the cap still spans heights
    for cap in (1, 2, 3, 5):
        p = make_planner(stratify=True, max_frontiers=cap, vox_size=2.0)
        o = p._rank_frontiers(fr, gain)
        assert len(o) == cap, (cap, len(o))
        assert o[0] == 0
        assert len(set(_levels_of(p, fr, o))) == min(cap, 3), _levels_of(p, fr, o)
    print('    stratify off -> pure gain order; on -> one per height before a '
          'second at any, rank 0 = global argmax, cap honoured')


def test_06b_regression_floor_not_ties_to_even():
    """Voxel centres sit at odd multiples of half the voxel size, so z/vox is
    always a .5 and `round()` bankers-rounds adjacent layers together in pairs.
    Pin the floor grouping by running the pre-fix expression side by side."""
    vox = 2.0
    zs = [1.0, 3.0, 5.0, 7.0, 9.0, 11.0]          # six voxel centres
    fr = np.array([[float(i), 0.0, z] for i, z in enumerate(zs)])
    gain = np.array([100.0, 90.0, 80.0, 70.0, 60.0, 50.0])

    p = make_planner(stratify=True, max_frontiers=6, vox_size=vox)
    order = p._rank_frontiers(fr, gain)
    got_levels = {int(z // vox) for z in zs}
    assert len(got_levels) == len(zs), (
        f'floor grouping merged heights: {sorted(got_levels)}')
    assert sorted(order) == list(range(6)), sorted(order)
    assert len(set(_levels_of(p, fr, order))) == 6, _levels_of(p, fr, order)

    q = RoundRankPlanner()
    q._fr_z_stratify, q._max_frontiers, q._vox_size = True, 6, vox
    bad_levels = {int(round(z / vox)) for z in zs}
    # ties-to-even merges (3, 5) and (7, 9): 1->0, 3->2, 5->2, 7->4, 9->4, 11->6
    assert sorted(bad_levels) == [0, 2, 4, 6], sorted(bad_levels)
    assert len(bad_levels) < len(zs), 'the pre-fix expression merged nothing'

    # Two frontiers per true height: the bug is observable in the ORDER, not
    # only in the level count.
    zs2 = [1.0, 1.0, 3.0, 3.0, 5.0, 5.0]
    fr2 = np.array([[float(i), 0.0, z] for i, z in enumerate(zs2)])
    gain2 = np.array([100.0, 90.0, 80.0, 70.0, 60.0, 50.0])
    p2 = make_planner(stratify=True, max_frontiers=6, vox_size=vox)
    q2 = RoundRankPlanner()
    q2._fr_z_stratify, q2._max_frontiers, q2._vox_size = True, 6, vox
    good = [float(v) for v in gain2[p2._rank_frontiers(fr2, gain2)]]
    bad = [float(v) for v in gain2[q2._rank_frontiers(fr2, gain2)]]
    assert good == [100.0, 80.0, 60.0, 90.0, 70.0, 50.0], good
    assert bad != good, 'the pre-fix expression produced the same order'
    assert bad == [100.0, 80.0, 90.0, 70.0, 60.0, 50.0], bad
    print(f'    floor: {len(zs)} heights -> {len(got_levels)} levels, order {good}')
    print(f'    round (pre-fix): {len(zs)} heights -> {len(bad_levels)} levels '
          f'{sorted(bad_levels)}, order {bad}')


def test_06c_rank_frontiers_edge_cases():
    """Empty and single-frontier inputs. The empty case raises rather than
    returning an empty order -- reachable only if a caller other than
    `_voxel_frontiers` (which returns early on an empty set) ever calls it."""
    p = make_planner(stratify=True, max_frontiers=6, vox_size=2.0)
    one = p._rank_frontiers(np.array([[0.0, 0.0, 3.0]]), np.array([5.0]))
    assert list(one) == [0], list(one)
    p_off = make_planner(stratify=False, max_frontiers=6, vox_size=2.0)
    assert list(p_off._rank_frontiers(np.zeros((0, 3)), np.zeros(0))) == []
    raised = None
    try:
        p._rank_frontiers(np.zeros((0, 3)), np.zeros(0))
    except Exception as exc:          # noqa: BLE001
        raised = exc
    assert isinstance(raised, ValueError), (
        f'empty stratified input no longer raises; got {raised!r}')
    note('_rank_frontiers',
         'planner_node.py:1375 `max(len(v) for v in levels.values())` raises '
         'ValueError on an empty frontier set when frontier_z_stratify is on; '
         'the stratify-off path returns an empty order. Unreachable today -- '
         '_voxel_frontiers returns before calling it (planner_node.py:1328) -- '
         'but it is an asymmetry a second caller would trip over.')
    print('    single frontier ok; empty + stratify raises ValueError '
          '(guarded upstream), empty + no stratify returns []')


# ── 7. commitment / lock ─────────────────────────────────────────────────────

def test_07_commitment_lock():
    p = make_planner(lock_s=6.0, swap_frac=0.35, unlock_radius=8.0)
    here_far = (0.0, 0.0)
    goal, goal_xy = [10, 10], (100.0, 100.0)

    # first pick takes the lock
    assert p._commit(goal, 0.80, goal_xy, here_far) == goal
    assert p._locked_goal_xy == goal_xy

    # inside lock_s: even a much better rival is refused
    CLOCK.advance(1.0)
    assert p._commit([20, 20], 5.00, (200.0, 200.0), here_far) == goal
    CLOCK.advance(4.0)
    assert p._commit([20, 20], 5.00, (200.0, 200.0), here_far) == goal

    # past lock_s but inside the swap margin: still refused
    CLOCK.advance(2.0)          # held 7.0 s
    margin = 0.35 * (abs(0.80) + 1e-6)
    assert p._commit([20, 20], 0.80 + 0.5 * margin, (200.0, 200.0), here_far) == goal
    # past lock_s and past the margin: swap
    got = p._commit([20, 20], 0.80 + 1.5 * margin, (200.0, 200.0), here_far)
    assert got == [20, 20], got
    assert p._locked_goal_xy == (200.0, 200.0)
    assert p._locked_at == CLOCK.time(), 'the swap did not restart the lock timer'
    print(f'    lock held for {p._lock_s:.0f} s, swap margin '
          f'{p._swap_frac:.2f}*|score| = {margin:.4f}: rival refused inside the '
          f'window and inside the margin, accepted outside both')


def test_07b_regression_release_on_arrival_at_the_locked_goal():
    """The release test is `dist(here, LOCKED goal xy) <= unlock_radius`. It
    must not require the challenger to be the same grid cell as the locked
    goal: candidates are re-extracted every tick and the locked cell is rarely
    offered again, so a same-cell test almost never fires and the drone parks
    on the frontier it reached for the rest of the run."""
    p = make_planner(lock_s=600.0, swap_frac=0.35, unlock_radius=8.0)
    goal, goal_xy = [10, 10], (100.0, 100.0)
    assert p._commit(goal, 0.80, goal_xy, (0.0, 0.0)) == goal

    # arrived at the LOCKED goal, challenger is a different cell AND worse, and
    # the lock window has barely opened. It must still release.
    CLOCK.advance(0.5)
    rival = [37, 4]
    got = p._commit(rival, -0.90, (-260.0, 15.0), (103.0, 104.0))
    assert got == rival, (
        f'arrived within {p._unlock_radius} m of the locked goal but held: {got}')
    assert p._locked_goal_xy == (-260.0, 15.0)
    assert any('releasing for the next' in ln for ln in p._log.lines)

    # the same-cell requirement really is gone
    src = _slice(SRC, '    def _commit', '    def _crop_to_map')
    # comments stripped: the docstring/comment in _commit quotes the old test
    # by name, and a raw substring search matches the explanation, not the code.
    code = '\n'.join(ln.split('#', 1)[0] for ln in src.splitlines())
    code = code.replace(ast.get_docstring(_METHODS['_commit']) or '', '')
    assert 'cand == self._locked_goal' not in code, (
        'the pre-fix same-cell equality is back in _commit')
    rel = [n for n in ast.walk(_METHODS['_commit']) if isinstance(n, ast.If)
           and 'self._unlock_radius' in ast.unparse(n.test)]
    assert len(rel) == 1, 'expected exactly one unlock_radius test in _commit'
    assert 'self._locked_goal_xy' in ast.unparse(rel[0].test), ast.unparse(rel[0].test)
    assert 'cand' not in {n.id for n in ast.walk(rel[0].test)
                          if isinstance(n, ast.Name)}, (
        'the release test reads the challenger; it must read only the locked '
        'goal and the current position')

    # measured against the LOCKED goal, not the challenger: sitting on top of a
    # candidate you have not committed to must NOT release the lock.
    q = make_planner(lock_s=600.0, unlock_radius=8.0)
    assert q._commit([1, 1], 0.5, (400.0, 400.0), (0.0, 0.0)) == [1, 1]
    CLOCK.advance(0.5)
    held = q._commit([2, 2], 9.9, (0.5, 0.5), (0.0, 0.0))
    assert held == [1, 1], (
        f'released because the CHALLENGER was underfoot, not the locked goal: '
        f'{held}')

    # boundary
    r = make_planner(lock_s=600.0, unlock_radius=8.0)
    r._commit([1, 1], 0.5, (0.0, 0.0), (100.0, 0.0))
    assert r._commit([2, 2], 0.1, (50.0, 0.0), (8.0, 0.0)) == [2, 2], 'at radius'
    s = make_planner(lock_s=600.0, unlock_radius=8.0)
    s._commit([1, 1], 0.5, (0.0, 0.0), (100.0, 0.0))
    assert s._commit([2, 2], 0.1, (50.0, 0.0), (8.001, 0.0)) == [1, 1], 'past radius'
    print('    arrival at the locked goal releases for any challenger, at any '
          'score, at any cell; arrival at the CHALLENGER does not; boundary at '
          'exactly unlock_radius_m releases')


def test_07c_swap_margin_collapses_near_zero_score():
    """`margin = swap_margin_frac * (abs(locked_score) + 1e-6)`. VLFM values
    are >= 0, so |score| is the score and the margin reads as a percentage. The
    frontier arm's score is `gain - weight*visit_cost`, which crosses zero, and
    at the crossing the margin is ~3.5e-7 -- the hysteresis vanishes exactly in
    the middle of the frontier arm's range, and grows again as the score goes
    negative."""
    p = make_planner(lock_s=1.0, swap_frac=0.35)
    p._commit([1, 1], 0.0, (400.0, 400.0), (0.0, 0.0))
    CLOCK.advance(2.0)
    got = p._commit([2, 2], 1e-5, (500.0, 500.0), (0.0, 0.0))
    assert got == [2, 2], f'expected a swap on a 1e-5 improvement, got {got}'

    q = make_planner(lock_s=1.0, swap_frac=0.35)
    q._commit([1, 1], -0.80, (400.0, 400.0), (0.0, 0.0))
    CLOCK.advance(2.0)
    assert q._commit([2, 2], -0.80 + 0.20, (500.0, 500.0), (0.0, 0.0)) == [1, 1]
    assert q._commit([2, 2], -0.80 + 0.30, (500.0, 500.0), (0.0, 0.0)) == [2, 2]
    note('_commit margin',
         'planner_node.py:1605 sizes the swap margin as '
         'swap_margin_frac * |locked_score|. The vlfm arm scores in [0, 1] so '
         'that reads as a percentage; the frontier arm scores '
         'gain - weight*visit_cost, which crosses zero, so the hysteresis '
         'collapses to ~3.5e-7 at score 0 (any improvement swaps) and is '
         'largest when the score is most negative. Shared code, different '
         'effective behaviour between the two arms.')
    print('    at locked_score=0 a 1e-5 rival swaps; at -0.80 a rival needs '
          '+0.28 -- the margin tracks |score|, not the score range')


# ── 8. target override ───────────────────────────────────────────────────────

def test_08_target_overrides_the_frontier():
    agent = FakeAgent()
    agent.set_xy(0.0, 0.0)
    p = make_planner(track_instances=True)
    p._goal_points = [[10, 10]]
    frontier_xy = agent.grid_to_map_xy(10, 10)

    # no target -> the frontier
    p._active_target = None
    assert p._goal_xy(0, agent) == frontier_xy

    # a detected human -> the human, regardless of the frontier or the lock
    p._active_target = (37.5, -12.5)
    p._locked_goal = [10, 10]
    p._locked_goal_xy = frontier_xy
    p._locked_score = 99.0
    p._locked_at = CLOCK.time()
    assert p._goal_xy(0, agent) == (37.5, -12.5), (
        'the frontier hysteresis suppressed a divert to a detected target')
    # ... and the frontier machinery is untouched by the divert
    assert p._locked_goal == [10, 10] and p._locked_goal_xy == frontier_xy

    # structural: the frontier assignment is skipped outright while a target is
    # active -- `found_goal = self._active_target is not None` and
    # `if assignment_due and not found_goal: self._assign(...)`.
    tick = _slice(SRC, '    def _tick', '    def _voxel_frontiers')
    assert 'found_goal = self._active_target is not None' in tick
    assert 'if assignment_due and not found_goal:' in tick
    assert tick.index('found_goal = self._active_target is not None') \
        < tick.index('if assignment_due and not found_goal:')
    goal_src = _slice(SRC, '    def _goal_xy', '    def _build_path')
    assert goal_src.index('self._active_target') < goal_src.index('_goal_points')
    print('    _goal_xy returns the active target ahead of the frontier, the '
          'lock is not consulted, and _assign is skipped entirely while a '
          'target is active')


# ── 9. parity with vlfm ──────────────────────────────────────────────────────

_TREE = ast.parse(SRC)
_CLS = next(n for n in ast.walk(_TREE) if isinstance(n, ast.ClassDef)
            and any(getattr(m, 'name', '') == '_frontier_pick' for m in n.body))
_METHODS = {m.name: m for m in _CLS.body if isinstance(m, ast.FunctionDef)}


def _self_attrs(node):
    return {n.attr for n in ast.walk(node)
            if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
            and n.value.id == 'self'}


def test_09_parity_the_two_arms_differ_only_in_scoring():
    v = _self_attrs(_METHODS['_vlfm_pick'])
    f = _self_attrs(_METHODS['_frontier_pick'])
    shared_required = {'_points_in_polygon', '_search_poly', '_agent_xy',
                       '_commit', '_vlfm_dist_penalty', 'get_logger'}
    assert shared_required <= (v & f), shared_required - (v & f)
    assert (v - f) == {'_value_map', '_vlfm_min_conf'}, v - f
    assert (f - v) == {'_visit_cost'}, f - v

    # identical control flow around the scoring expression
    for name in ('_vlfm_pick', '_frontier_pick'):
        s = _slice(SRC, f'    def {name}', '    def ' + (
            '_frontier_pick' if name == '_vlfm_pick' else '_lawnmower_pick'))
        assert 'if not target_point_list:\n            return None' in s, name
        assert 'agent.grid_to_map_xy(p[0], p[1]) for p in target_point_list' in s, name
        assert 'self._points_in_polygon(self._to_map_arr(xy), self._search_poly)' in s, name
        assert 'if not keep.any():' in s, name
        assert 'idx = np.flatnonzero(keep)' in s, name
        assert 'here = np.array(self._agent_xy(agent))' in s, name
        assert 'self._vlfm_dist_penalty * d' in s, name
        assert 'return self._commit(' in s, name

    # the shared frontier source is not conditioned on nav_mode
    tick = _slice(SRC, '    def _tick', '    def _voxel_frontiers')
    assert "self._nav_mode == 'vlfm'" in tick, 'expected the keyframe guard'
    assert tick.count('self._nav_mode ==') == 1, (
        'a second nav_mode branch appeared in _tick; the arms no longer share '
        'the tick')
    for shared in ('self._voxel_frontiers(merged)', 'self._update_targets(',
                   'self._command(i, agent, offsets[i])',
                   'self._map_process.Map_Extraction('):
        assert shared in tick, shared
    vox = _slice(SRC, '    def _voxel_frontiers', '    def _rank_frontiers')
    assert 'self._nav_mode' not in vox, 'the frontier source branches on nav_mode'
    # in the MAP frame, through the same transform the pick functions use
    assert 'self._points_in_polygon(self._to_map_arr(fr[:, :2]),' in vox
    assert 'self._rank_frontiers(fr, gain, cap=fr.shape[0])' in vox
    cmd = _slice(SRC, '    def _command', '    def _await')
    assert 'self._nav_mode' not in cmd, 'actuation branches on nav_mode'
    tgt = _slice(SRC, '    def _update_targets', '    def _goal_xy')
    assert 'self._nav_mode' not in tgt, 'target visiting branches on nav_mode'

    # the two _assign branches have the same shape
    asn = _slice(SRC, '    def _assign', '    def _record_round')
    assert '_vlfm_pick(target_point_list, self._agents[i])' in asn
    assert '_frontier_pick(target_point_list, self._agents[i])' in asn
    # every no-candidate branch recovers into the sector the same way
    assert asn.count('self._fallback_goal(self._agents[i], map_size)') >= 5
    assert 'self._random_goal(map_size)' not in asn
    note('_assign',
         "_assign guards the vlfm pick with `pick if pick is not None else "
         "...` while the frontier pick uses `... or self._fallback_goal(...)`. "
         "Cosmetic today (both picks return a 2-element list or None, and a "
         "2-element list is truthy), but the `or` form would also fall "
         "through on an empty list.")
    note('_commit logging',
         "planner_node.py:1608 logs the swap as 'vlfm: swap after ...' in both "
         "arms, so a frontier run's logs claim vlfm swaps.")
    print('    self-attribute delta is exactly {_value_map, _vlfm_min_conf} vs '
          '{_visit_cost}; frontier source, sector filter, commitment, target '
          'visiting and actuation are all unconditioned on nav_mode')


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


def test_09b_config_parity_of_the_shared_machinery():
    """The code is shared; the two arms' overlays are not. Assert only on the
    keys that decide the FRONTIER SOURCE -- differ there and the arms are not
    seeing the same candidates at all -- and report the rest."""
    base = _yaml_scalars(os.path.join(_CONFIG, 'planner.yaml'))
    fro = dict(base, **_yaml_scalars(os.path.join(_CONFIG, 'frontier.yaml')))
    vlf = dict(base, **_yaml_scalars(os.path.join(_CONFIG, 'vlfm.yaml')))

    source_keys = ['frontier_source', 'voxel_size_m', 'voxel_max_range_m',
                   'voxel_neighborhood_r', 'voxel_min_unobserved',
                   'voxel_min_empty', 'voxel_min_occupied',
                   'frontier_subsampling', 'frontier_subsampling_min_cells',
                   'frontier_z_stratify', 'frontier_z_min_m', 'frontier_z_max_m']
    src_delta = {k: (fro.get(k), vlf.get(k)) for k in source_keys
                 if fro.get(k) != vlf.get(k)}
    assert not src_delta, f'the arms do not see the same frontiers: {src_delta}'

    other = ['max_frontiers', 'vlfm_distance_penalty', 'sector_partition',
             'search_area_pad_m', 'frontier_lock_s', 'frontier_swap_margin_frac',
             'frontier_unlock_radius_m', 'plan_period_s', 'round_period_s',
             'flight_altitude_m', 'goal_tolerance_m']
    delta = {k: (fro.get(k), vlf.get(k)) for k in other
             if fro.get(k) != vlf.get(k)}
    for k, (a, b) in sorted(delta.items()):
        print(f'    config delta  {k}: frontier={a!r}  vlfm={b!r}')
    assert 'search_area_pad_m' not in delta, (
        'the arms sector different polygons again: frontier='
        f'{fro.get("search_area_pad_m")!r} vlfm={vlf.get("search_area_pad_m")!r}')
    if delta:
        note('config parity',
             'frontier.yaml and vlfm.yaml agree on every frontier-SOURCE '
             'parameter, but differ on ' + ', '.join(
                 f'{k} ({a} vs {b})' for k, (a, b) in sorted(delta.items()))
             + '. search_area_pad_m is no longer among them -- the frontier '
               'arm\'s 25 m pad was removed, so both arms now sector the same '
               'unpadded search_area_xy and the "same search area" half of the '
               'claim holds. vlfm_distance_penalty still differs 2x and '
               'max_frontiers 0 (derived, 6-12) vs 12, so the two arms remain '
               'differently scored and differently tuned.')
    print('    frontier-source parameters identical across the two overlays')


# ── 10. the frontier source itself, end to end ───────────────────────────────

def test_10_voxel_frontiers_feed_a_gain_ranked_list():
    """A real VoxelMap, carved offline, through the real
    `frontiers_persistent` and the real `_rank_frontiers`: the premise
    `_frontier_pick` rests on is that the list it receives is gain-ordered."""
    vm = VoxelMap((-40.0, -40.0, 0.0), (40.0, 40.0, 20.0), 2.0)
    rng = np.random.default_rng(3)
    cam = np.array([0.0, 0.0, 12.0])
    ground = np.column_stack([rng.uniform(-25.0, 25.0, 4000),
                              rng.uniform(-25.0, 25.0, 4000),
                              np.zeros(4000)])
    vm.integrate(cam, ground, carve_samples=32, max_range_m=60.0)
    fr, gain = vm.frontiers_persistent(neighborhood_r=1, min_unobserved=4,
                                       min_empty=2, subsampling=2,
                                       subsampling_min_fronti=1)
    assert fr.shape[0] > 5, f'only {fr.shape[0]} frontiers carved'

    p_off = make_planner(stratify=False, max_frontiers=12, vox_size=2.0)
    order = p_off._rank_frontiers(fr, gain)
    g = gain[order]
    assert all(g[i] >= g[i + 1] for i in range(len(g) - 1)), list(g)
    assert g[0] == gain.max()

    p_on = make_planner(stratify=True, max_frontiers=12, vox_size=2.0)
    order_on = p_on._rank_frontiers(fr, gain)
    g_on = gain[order_on]
    assert g_on[0] == gain.max()
    monotone = all(g_on[i] >= g_on[i + 1] for i in range(len(g_on) - 1))
    heights = len({int(fr[k, 2] // 2.0) for k in order_on})
    print(f'    {fr.shape[0]} frontiers from a carved map; stratify off -> '
          f'gain-monotone list; stratify on -> {heights} heights in the top '
          f'{len(order_on)}, gain-monotone={monotone}')
    if not monotone:
        note('gain proxy',
             f'On a carved map the shipped default (stratify on, '
             f'max_frontiers 12) returns a list spanning {heights} heights '
             f'whose measured gains are NOT descending, confirming test_01b '
             f'on real frontier data rather than a constructed example.')

    # and the sector filter really is applied at the source, before ranking
    poly = np.array([[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]])
    keep = StubPlanner._points_in_polygon(fr[:, :2], poly)
    assert keep.any() and not keep.all(), 'pick a polygon that actually cuts'
    print(f'    sector filter at the source keeps {int(keep.sum())} of '
          f'{fr.shape[0]}')


# ── runner ───────────────────────────────────────────────────────────────────

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith('test_') and callable(o)]
    failures = []
    print(f'frontier baseline: {len(tests)} tests, offline '
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
            print(textwrap_indent(traceback.format_exc(), '          '))
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
            print(textwrap_indent(_wrap(msg, 74), '  '))
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


def textwrap_indent(s, pad):
    import textwrap as _tw
    return _tw.indent(s, pad)


if __name__ == '__main__':
    sys.exit(main())
