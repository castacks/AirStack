"""Consensus-based bundle auction (CBBA, single-target) for target assignment.

Each robot bids on its rays and confirmed BBs (compute_my_bids / compute_bb_bids,
value = -distance, higher = closer = wins), broadcasts them, then rebuilds the
same robots×instances cost matrix from the gossiped bids and solves it
identically with greedy-with-removal (assign_global). Bids are deduped per
physical target via triangulation (_same_bid_instance), so two drones on two
different houses don't contest. Ties broken by lower robot id.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from raven_nav.ray_groups import RayGroup
from raven_nav.ray_targets import is_same_target


# Centre proximity under which two BBs (or a ray pointing at a BB) are one target.
BB_MATCH_M = 8.0


def _behind_penalty(entry, heading_xy, behind_weight) -> float:
    """Gentle target heading: front and side unpenalized; a reduced penalty only
    for targets behind the drone. So target selection is nearest-first (distance
    dominates) but won't fully reverse for something only marginally behind.
    (Anti-zigzag momentum still applies to frontier exploration, not here.)"""
    if heading_xy is None or behind_weight <= 0.0:
        return 0.0
    d = np.asarray(entry.avg_dir, dtype=float)[:2]
    n = float(np.linalg.norm(d))
    if n < 1e-6:
        return 0.0
    cs = float(np.dot(d / n, heading_xy))
    return behind_weight * max(-cs, 0.0)


@dataclass
class BidEntry:
    """One bid row = one ray group + its bid value.

    Used both for the local broadcast (my own bids) and for caching peer
    bids in PeerState. The group descriptor is intentionally lightweight —
    avg_origin + avg_dir + label + num_rays + avg_score is enough to run
    is_same_target() and the auction.
    """
    label: str
    value: float
                                  # finalize_bid_values folds in ray reach + heading
    avg_origin: np.ndarray
    avg_dir: np.ndarray
    num_rays: int
    avg_score: float
    is_bb: bool = False
    size: Optional[np.ndarray] = None


def compute_my_bids(groups: List[RayGroup]) -> List[BidEntry]:
    """One BidEntry per RayGroup.

    Multiple groups with the same label produce multiple entries — the
    auction will sort out which (if any) compete with peer bids.
    """
    out: List[BidEntry] = []
    for g in groups:
        out.append(BidEntry(
            label=g.label,
            value=-float(g.min_dist_to_robot),
            avg_origin=np.asarray(g.avg_origin, dtype=float),
            avg_dir=np.asarray(g.avg_dir, dtype=float),
            num_rays=int(g.num_rays),
            avg_score=float(g.avg_score),
        ))
    return out


def compute_bb_bids(clusters, robot_pos) -> List[BidEntry]:
    """One BidEntry per confirmed cluster. clusters: list of (label, centre(3),
    size(3)) in the robot's local frame. value = -surface distance to the drone
    (raw; heading folded in by finalize_bid_values)."""
    out: List[BidEntry] = []
    rp = np.asarray(robot_pos, dtype=float)
    for label, center, size in clusters:
        c = np.asarray(center, dtype=float)
        s = np.asarray(size, dtype=float)
        gap = np.clip(np.abs(rp - c) - s / 2.0, 0.0, None)
        surf = float(np.linalg.norm(gap))
        bearing = c - rp
        bn = float(np.linalg.norm(bearing))
        avg_dir = bearing / bn if bn > 1e-6 else np.array([1.0, 0.0, 0.0])
        out.append(BidEntry(
            label=label, value=-surf, avg_origin=c.copy(), avg_dir=avg_dir,
            num_rays=0, avg_score=1.0, is_bb=True, size=s.copy()))
    return out


def finalize_bid_values(entries, heading_xy, ray_reach_factor: float = 1.0,
                        behind_penalty: float = 0.0) -> None:
    """Fold heading into each bid's value (in place). Higher = better.

    Both rays and BBs use a gentle behind-only penalty, so target selection is
    nearest-first (distance dominates) without skipping a near-but-side target for
    a far-ahead one. Rays are additionally scaled by ray_reach_factor (the target
    sits further out than the ray's origin, so rays don't look closer than BBs).
    (Frontier exploration keeps its own anti-zigzag momentum — not here.)"""
    for e in entries:
        if not e.is_bb:
            e.value *= ray_reach_factor
        e.value -= _behind_penalty(e, heading_xy, behind_penalty)


class _GroupView:
    """Adapter so BidEntry can be passed to is_same_target() (which expects
    label/avg_origin/avg_dir on the object). Defined here to keep ray_targets
    independent of the BidEntry type.
    """
    __slots__ = ('label', 'avg_origin', 'avg_dir')

    def __init__(self, entry: BidEntry) -> None:
        self.label = entry.label
        self.avg_origin = entry.avg_origin
        self.avg_dir = entry.avg_dir


# CBBA consensus assignment: every robot rebuilds the same robots×instances cost
# matrix from gossiped bids (cost = -value) and solves it identically. value is
# the broadcast utility, so all robots agree.
_INFEASIBLE_COST = 1e6


def _same_bid_instance(a: BidEntry, b: BidEntry, polygon_xy) -> bool:
    """Same physical target. BB↔BB: centres close. BB↔ray: ray points at the BB
    centre (ahead, within BB_MATCH_M). ray↔ray: triangulation (is_same_target)."""
    if a.label != b.label:
        return False
    if a.is_bb and b.is_bb:
        return float(np.linalg.norm(
            a.avg_origin[:2] - b.avg_origin[:2])) <= BB_MATCH_M
    if a.is_bb or b.is_bb:
        bb, ray = (a, b) if a.is_bb else (b, a)
        o = np.asarray(ray.avg_origin, dtype=float)
        d = np.asarray(ray.avg_dir, dtype=float)
        dn = float(np.linalg.norm(d))
        if dn < 1e-6:
            return False
        d = d / dn
        v = np.asarray(bb.avg_origin, dtype=float) - o
        t = float(np.dot(v, d))
        if t <= 0.0:
            return False
        return float(np.linalg.norm(v - t * d)) <= BB_MATCH_M
    same, _, _ = is_same_target(a, b, polygon_xy)
    return same


def _cluster_bid_instances(all_bids, polygon_xy):
    """Union-find bids (same label + _same_bid_instance) into instances. Returns
    one dict {robot_id: that robot's best BidEntry} per instance."""
    n = len(all_bids)
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        _, bi = all_bids[i]
        for j in range(i + 1, n):
            _, bj = all_bids[j]
            if bi.label != bj.label:
                continue
            if _same_bid_instance(bi, bj, polygon_xy):
                union(i, j)

    comps: Dict[int, List[int]] = {}
    for i in range(n):
        comps.setdefault(find(i), []).append(i)

    clusters: List[Dict[int, BidEntry]] = []
    for idxs in comps.values():
        per_robot: Dict[int, BidEntry] = {}
        for i in idxs:
            rid, b = all_bids[i]
            # keep the robot's highest-utility bid for this instance
            if rid not in per_robot or b.value > per_robot[rid].value:
                per_robot[rid] = b
        clusters.append(per_robot)
    return clusters


def _solve_cbba(cost: np.ndarray, robot_ids: List[int]) -> Dict[int, int]:
    """Greedy with removal: take the cheapest feasible (robot, target) pair,
    assign it, remove both; repeat. Removing the winner lets a peer win the next
    target. Ties: lower robot id, then lower target index."""
    R, T = cost.shape
    used_r: set = set()
    used_t: set = set()
    assigned: Dict[int, int] = {}
    while len(used_r) < R and len(used_t) < T:
        best = None
        for r in range(R):
            if r in used_r:
                continue
            for t in range(T):
                if t in used_t or cost[r, t] >= _INFEASIBLE_COST:
                    continue
                key = (float(cost[r, t]), robot_ids[r], t)
                if best is None or key < best[0]:
                    best = (key, r, t)
        if best is None:
            break
        _, r, t = best
        assigned[robot_ids[r]] = t
        used_r.add(r)
        used_t.add(t)
    return assigned


def assign_global(
    my_id: int,
    my_bids: List[BidEntry],
    peer_bids: Dict[str, List[BidEntry]],
    peer_ids: Dict[str, int],
    polygon_xy: Optional[np.ndarray] = None,
) -> Optional[Tuple[str, BidEntry]]:
    """CBBA consensus assignment. Returns (label, my BidEntry) for the instance
    assigned to my_id, or None."""
    if not my_bids:
        return None

    all_bids: List[Tuple[int, BidEntry]] = [(my_id, b) for b in my_bids]
    for name, entries in peer_bids.items():
        pid = peer_ids.get(name)
        if pid is None:
            continue
        all_bids.extend((pid, b) for b in entries)

    clusters = _cluster_bid_instances(all_bids, polygon_xy)
    if not clusters:
        return None

    robot_ids = sorted({rid for c in clusters for rid in c})
    if my_id not in robot_ids:
        return None
    row_of = {rid: r for r, rid in enumerate(robot_ids)}
    R, T = len(robot_ids), len(clusters)
    cost = np.full((R, T), _INFEASIBLE_COST, dtype=float)
    for t, c in enumerate(clusters):
        for rid, b in c.items():
            cost[row_of[rid], t] = -float(b.value)

    assigned = _solve_cbba(cost, robot_ids)

    my_t = assigned.get(my_id)
    if my_t is None:
        return None
    entry = clusters[my_t].get(my_id)
    if entry is None:
        return None
    return entry.label, entry


# ── Consensus over the unified task table (BBs + ray-leads) ──────────────────
#
# Conflict-free assignment by *shared-solution equivalence*: every robot solves
# the identical problem from gossiped inputs (positions, peer ages, explore
# scalars) and reaches the identical bundles — no bid race, no reactive yields.
# Stability from winner retention; robustness under degradation from
# age-weighting (per-robot by design: w=1 under healthy comms, so the shared
# solution only bends where consistency is already lost).


# Task localization states, most-localized first (priority for proximity merge).
TASK_STATES = ('bb-visited', 'bb-observing', 'ray-localized', 'ray')
_STATE_RANK = {s: i for i, s in enumerate(('ray', 'ray-localized',
                                           'bb-observing', 'bb-visited'))}


@dataclass
class Task:
    """One physical target instance shared across robots (BB or ray-lead)."""
    key: Tuple          # canonical (label, gx, gy); aligns the task across robots
    label: str
    centroid: np.ndarray   # (3,) target point estimate (BB centre or ray point)
    size: np.ndarray       # (3,) extents (zeros for a ray-lead point)
    status: str = 'bb-observing'   # one of TASK_STATES
    origin: Optional[np.ndarray] = None      # (3,) ray-lead bearing origin
    direction: Optional[np.ndarray] = None   # (3,) ray-lead bearing direction
    confidence: float = 0.0        # BB detection confidence / ray semantic score


def _task_surface_dist_xy(pos_xy, task) -> float:
    gap = np.clip(np.abs(np.asarray(pos_xy, float)[:2] - task.centroid[:2])
                  - task.size[:2] / 2.0, 0.0, None)
    return float(np.linalg.norm(gap))


# Peer repulsion for the consensus assignment: soft exp cost added to a task near
# an already-assigned one, so drones fan out across different areas (mild — it
# reorders choices, it does not force big detours).
PEER_REPULSION_W = 15.0
PEER_REPULSION_SCALE = 15.0
# Soft BB preference: a ray costs this much more than a like-distance BB (its true
# range is unknown). Sets the BB-vs-ray crossover distance (~this many m): a BB
# within it beats a near ray; a farther BB loses to a near ray (no cross-map).
RAY_COST_PENALTY = 25.0
# Max chain distance from a bundle's last task to an appended one.
BUNDLE_MAX_DETOUR_M = 20.0
# Poach guard: an agent that already owns tasks won't split off a foreign task a
# closer agent owns by more than this margin. (The soft geo cost penalty was
# inert — the winning pick is always its own owner, so it never flipped a
# decision — and was removed; this hard gate is the part that does real work.)
GEO_MARGIN_M = 15.0
# Decline (outside option): markup over frontier distance so a comparable
# real target still wins.
EXPLORE_MARKUP_M = 10.0
EXPLORE_KEY = ('~explore',)
_W_EPS = 1e-3


def build_bb_tasks(bb_list, match_m: float, key_grid: float) -> List[Task]:
    """Cluster (label, centre(3), size(3)) BBs from ALL sources (own + peers) into
    shared tasks. Single-linkage on xy centre proximity within a label; processed
    in a deterministic order so every robot that sees the same set yields the same
    tasks/keys. key = quantised mean centre (coarse grid; targets are far apart)."""
    items = sorted(
        ((str(lab), np.asarray(c, float), np.asarray(s, float)) for lab, c, s in bb_list),
        key=lambda t: (t[0], round(float(t[1][0]), 2), round(float(t[1][1]), 2)))
    n = len(items)
    used = [False] * n
    tasks: List[Task] = []
    for i in range(n):
        if used[i]:
            continue
        used[i] = True
        grp = [i]
        ci = items[i][1]
        for j in range(i + 1, n):
            if used[j] or items[j][0] != items[i][0]:
                continue
            if float(np.linalg.norm(items[j][1][:2] - ci[:2])) <= match_m:
                used[j] = True
                grp.append(j)
        cs = np.array([items[k][1] for k in grp], float)
        ss = np.array([items[k][2] for k in grp], float)
        cen = cs.mean(axis=0)
        siz = ss.max(axis=0)
        lab = items[grp[0]][0]
        key = (lab, int(round(cen[0] / key_grid)), int(round(cen[1] / key_grid)))
        tasks.append(Task(key=key, label=lab, centroid=cen, size=siz))
    return tasks


def _canon(t):
    return (t[0], round(float(t[1][0]), 2), round(float(t[1][1]), 2))


def build_tasks(items, match_m: float, key_grid: float) -> List[Task]:
    """Build the shared task list. BB fragments of one physical target fuse
    (union) within match_m. Ray-leads are passed through AS-IS — one task per
    filtered ray (already clustered upstream by compute_ray_groups); they are not
    re-merged here. Deterministic -> same tasks per robot.

    items: (label, point(3), size(3), status[, origin(3)[, direction(3)[, conf]]])."""
    norm = []
    for it in items:
        org = it[4] if len(it) > 4 and it[4] is not None else None
        dirc = it[5] if len(it) > 5 and it[5] is not None else None
        conf = float(it[6]) if len(it) > 6 and it[6] is not None else 0.0
        norm.append((str(it[0]), np.asarray(it[1], float), np.asarray(it[2], float),
                     str(it[3]), None if org is None else np.asarray(org, float),
                     None if dirc is None else np.asarray(dirc, float), conf))
    bb = sorted([x for x in norm if x[3].startswith('bb')], key=_canon)
    ray = sorted([x for x in norm if not x[3].startswith('bb')], key=_canon)
    tasks: List[Task] = []

    def _key(lab, pt):
        return (lab, int(round(pt[0] / key_grid)), int(round(pt[1] / key_grid)),
                len(tasks))

    used = [False] * len(bb)
    for i in range(len(bb)):
        if used[i]:
            continue
        used[i] = True
        grp = [i]
        ci = bb[i][1]
        for j in range(i + 1, len(bb)):
            if used[j] or bb[j][0] != bb[i][0]:
                continue
            if float(np.linalg.norm(bb[j][1][:2] - ci[:2])) <= match_m:
                used[j] = True
                grp.append(j)
        lo = np.min([bb[k][1][:3] - bb[k][2] / 2.0 for k in grp], axis=0)
        hi = np.max([bb[k][1][:3] + bb[k][2] / 2.0 for k in grp], axis=0)
        pt = (lo + hi) / 2.0
        st = ('bb-visited' if any(bb[k][3] == 'bb-visited' for k in grp)
              else 'bb-observing')
        tasks.append(Task(key=_key(bb[i][0], pt), label=bb[i][0],
                          centroid=pt, size=hi - lo, status=st,
                          confidence=max(bb[k][6] for k in grp)))

    for lab, pt, siz, st, org, dirc, conf in ray:
        tasks.append(Task(
            key=_key(lab, pt), label=lab, centroid=pt.copy(), size=siz.copy(),
            status=st, origin=None if org is None else org.copy(),
            direction=None if dirc is None else dirc.copy(), confidence=conf))
    return tasks


class ConsensusAssigner:
    """Stateful shared-solution assigner with CBBA-style ordered bundles.

    assign() returns {agent_id: [task_key, ...]} — head is executed, tail is a
    reservation replanned each tick; [] = declined in favour of local
    exploration. Same map on every robot given the same inputs. Heads-first
    (pass 1 gives every agent a head before any bundle grows), then strict
    breadth-first growth to bundle_len, gated by BUNDLE_MAX_DETOUR_M; a bundle
    never grows past a ray (unknown range — can't route through it). The
    decline option is feasible only for an agent that owns none of the
    remaining tasks, so no task is ever orphaned by exploration."""

    def __init__(self, bundle_len: int = 1) -> None:
        self.bundle_len = max(1, int(bundle_len))
        # agent_id -> ordered centroids of its previous bundle ([0] = head:
        # full switch_margin retention; tail: half).
        self._prev: Dict[int, List[np.ndarray]] = {}
        self._prev_explored: set = set()
        # Inputs + per-pick cost breakdown of the last solve (JSON-safe).
        self.last_debug: dict = {}

    def assign(self, tasks: List[Task], agent_pos: Dict[int, np.ndarray],
               switch_margin: float, match_m: float,
               agent_weight: Optional[Dict[int, float]] = None,
               explore_dist: Optional[Dict[int, float]] = None,
               ) -> Dict[int, list]:
        if not tasks or not agent_pos:
            self._prev = {}
            self._prev_explored = set()
            self.last_debug = {}
            return {}
        aids = sorted(agent_pos)
        w = {a: (float(agent_weight[a]) if agent_weight and a in agent_weight
                 else 1.0) for a in aids}
        ex = {a: float(explore_dist[a]) for a in aids
              if explore_dist and explore_dist.get(a) is not None
              and float(explore_dist[a]) >= 0.0}
        by_key = {t.key: t for t in tasks}

        # Age-weighted distance: a stale peer's rows inflate, so it stops
        # owning/winning tasks and a live agent inherits them (w=1 -> plain
        # distance, shared solution preserved).
        eff: Dict[Tuple[int, Tuple], float] = {}
        for a in aids:
            p = np.asarray(agent_pos[a], float)
            for t in tasks:
                eff[(a, t.key)] = _task_surface_dist_xy(p, t) / max(w[a], _W_EPS)
        owner = {t.key: min(aids, key=lambda a: (eff[(a, t.key)], a))
                 for t in tasks}
        owner_dist = {t.key: eff[(owner[t.key], t.key)] for t in tasks}

        def _is_ray(tk) -> bool:
            return str(by_key[tk].status).startswith('ray')

        taken: List[Tuple[int, np.ndarray]] = []

        def _breakdown(base, a, tk, is_head):
            rp = RAY_COST_PENALTY if _is_ray(tk) else 0.0
            rep = 0.0
            c = by_key[tk].centroid[:2]
            for ta, tc in taken:
                d = float(np.linalg.norm(c - tc[:2]))
                rep += w[ta] * PEER_REPULSION_W * float(
                    np.exp(-d / PEER_REPULSION_SCALE))
            ret = 0.0
            for i, pc in enumerate(self._prev.get(a) or []):
                if float(np.linalg.norm(c - np.asarray(pc)[:2])) <= match_m:
                    ret = -switch_margin if (is_head and i == 0) \
                        else -0.5 * switch_margin
                    break
            return base + rp + rep + ret, {
                'base': round(base, 2), 'ray_pen': round(rp, 2),
                'repulsion': round(rep, 2),
                'retention': round(ret, 2)}

        def _fmt(tk):
            return 'explore' if tk == EXPLORE_KEY else list(tk)

        bundles: Dict[int, list] = {a: [] for a in aids}
        explored: set = set()
        remaining = set(by_key)
        passes_dbg: list = []

        # Pass 1 — heads (or decline). Poach gate: an agent that owns remaining
        # tasks can neither decline NOR take a clearly-foreign task (geo excess
        # beyond the margin) — otherwise "can't decline" degenerates into
        # "grab the cheapest task anywhere", splitting clusters a nearby agent
        # would have bundled (136/136 observed far-splits came from this).
        pool = set(aids)
        picks_dbg: list = []
        while pool and remaining:
            per_agent: dict = {}
            for a in pool:
                a_owns = any(owner[tk] == a for tk in remaining)
                for tk in remaining:
                    if (a_owns and owner[tk] != a
                            and eff[(a, tk)] - owner_dist[tk] > GEO_MARGIN_M):
                        continue
                    c, bd = _breakdown(eff[(a, tk)], a, tk, True)
                    k = (c, a, tk)
                    if a not in per_agent or k < per_agent[a][0]:
                        per_agent[a] = (k, bd)
                if a in ex and all(owner[tk] != a for tk in remaining):
                    ret = -0.5 * switch_margin if a in self._prev_explored else 0.0
                    k = (ex[a] + EXPLORE_MARKUP_M + ret, a, EXPLORE_KEY)
                    bd = {'base': round(ex[a], 2), 'markup': EXPLORE_MARKUP_M,
                          'retention': round(ret, 2)}
                    if k < per_agent[a][0]:
                        per_agent[a] = (k, bd)
            if not per_agent:
                break
            (c, a, tk), bd = min(per_agent.values(), key=lambda kb: kb[0])
            if tk == EXPLORE_KEY:
                explored.add(a)
            else:
                bundles[a].append(tk)
                remaining.discard(tk)
                taken.append((a, by_key[tk].centroid))
            pool.discard(a)
            picks_dbg.append({
                'agent': a, 'pick': _fmt(tk), 'cost': round(c, 2), **bd,
                'alts': [{'agent': a2, 'pick': _fmt(k2[2]),
                          'cost': round(k2[0], 2)}
                         for a2, (k2, _) in sorted(per_agent.items())
                         if a2 != a]})
        passes_dbg.append({'pass': 1, 'picks': picks_dbg})

        # Passes 2..L — strict breadth-first growth, ray-terminal.
        for depth in range(2, self.bundle_len + 1):
            pool = {a for a in aids if len(bundles[a]) == depth - 1
                    and not _is_ray(bundles[a][-1])}
            picks_dbg = []
            while pool and remaining:
                per_agent = {}
                for a in pool:
                    last = by_key[bundles[a][-1]]
                    for tk in remaining:
                        chain = _task_surface_dist_xy(
                            last.centroid, by_key[tk])
                        if chain > BUNDLE_MAX_DETOUR_M:
                            continue
                        c, bd = _breakdown(chain, a, tk, False)
                        k = (c, a, tk)
                        if a not in per_agent or k < per_agent[a][0]:
                            per_agent[a] = (k, bd)
                if not per_agent:
                    break
                (c, a, tk), bd = min(per_agent.values(), key=lambda kb: kb[0])
                bundles[a].append(tk)
                remaining.discard(tk)
                taken.append((a, by_key[tk].centroid))
                pool.discard(a)
                picks_dbg.append({
                    'agent': a, 'pick': _fmt(tk), 'cost': round(c, 2), **bd,
                    'alts': [{'agent': a2, 'pick': _fmt(k2[2]),
                              'cost': round(k2[0], 2)}
                             for a2, (k2, _) in sorted(per_agent.items())
                             if a2 != a]})
            if picks_dbg:
                passes_dbg.append({'pass': depth, 'picks': picks_dbg})

        # Full-precision inputs + incoming retention state so an offline
        # analyzer can replay this solve exactly (analyze_auction_solve.py).
        prev_in = {str(a): [[float(c[0]), float(c[1])] for c in cs]
                   for a, cs in self._prev.items()}
        prev_explored_in = set(self._prev_explored)
        self._prev = {a: [by_key[tk].centroid.copy() for tk in ks]
                      for a, ks in bundles.items() if ks}
        self._prev_explored = set(explored)
        assigned = {a: list(ks) for a, ks in bundles.items()
                    if ks or a in explored}
        self.last_debug = {
            'params': {'bundle_len': self.bundle_len,
                       'switch_margin': switch_margin, 'match_m': match_m,
                       'ray_penalty': RAY_COST_PENALTY,
                       'geo_margin': GEO_MARGIN_M,
                       'explore_markup': EXPLORE_MARKUP_M,
                       'detour_cap': BUNDLE_MAX_DETOUR_M},
            'agents': {str(a): {
                'pos': [float(agent_pos[a][0]), float(agent_pos[a][1])],
                'w': float(w[a]),
                'explore_dist': float(ex[a]) if a in ex else None,
                'explored_prev_tick': a in prev_explored_in} for a in aids},
            'tasks': [{'key': list(t.key), 'label': t.label,
                       'status': t.status,
                       'xy': [float(t.centroid[0]), float(t.centroid[1])],
                       'size': [float(t.size[0]), float(t.size[1])],
                       'owner': owner[t.key],
                       'owner_dist': round(owner_dist[t.key], 1)}
                      for t in tasks],
            'prev': prev_in,
            'prev_explored': sorted(prev_explored_in),
            'passes': passes_dbg,
            'result': {str(a): ('explore' if not ks else [list(k) for k in ks])
                       for a, ks in assigned.items()},
            'unassigned_tasks': [list(k) for k in sorted(remaining, key=str)],
        }
        return assigned

    def my_task(self, assigned: Dict[int, list], my_id: int,
                tasks: List[Task]) -> Optional[Task]:
        ks = assigned.get(my_id)
        if not ks:
            return None
        return next((t for t in tasks if t.key == ks[0]), None)
