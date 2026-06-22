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


# ── CBAA consensus over confirmed-target (BB) tasks ──────────────────────────
#
# Conflict-free assignment by *shared-solution equivalence*: with full gossip
# every robot has the same agent positions + the same confirmed-target set, so
# each robot solves the identical problem and reaches the identical assignment —
# no bid gossip race, no reactive yields. Stability comes from retaining the
# previous winner unless a challenger beats it by a margin (CBBA winner
# retention), computed identically on every robot. Cost is plain xy distance
# (diminishing-marginal-gain clean — no boosts).


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


def _task_surface_dist_xy(pos_xy, task) -> float:
    gap = np.clip(np.abs(np.asarray(pos_xy, float)[:2] - task.centroid[:2])
                  - task.size[:2] / 2.0, 0.0, None)
    return float(np.linalg.norm(gap))


# Peer repulsion for the consensus assignment: soft exp cost added to a task near
# an already-assigned one, so drones fan out across different areas (mild — it
# reorders choices, it does not force big detours).
PEER_REPULSION_W = 15.0
PEER_REPULSION_SCALE = 15.0
# Ray-lead cost surcharge: a localized BB is preferred over a ray at a comparable
# distance (a ray's true range is unknown, so it costs more to resolve). A ray
# must be this much closer than a BB to win it.
RAY_COST_PENALTY = 15.0


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

    items: (label, point(3), size(3), status[, origin(3)[, direction(3)]])."""
    norm = []
    for it in items:
        org = it[4] if len(it) > 4 and it[4] is not None else None
        dirc = it[5] if len(it) > 5 and it[5] is not None else None
        norm.append((str(it[0]), np.asarray(it[1], float), np.asarray(it[2], float),
                     str(it[3]), None if org is None else np.asarray(org, float),
                     None if dirc is None else np.asarray(dirc, float)))
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
                          centroid=pt, size=hi - lo, status=st))

    for lab, pt, siz, st, org, dirc in ray:
        tasks.append(Task(
            key=_key(lab, pt), label=lab, centroid=pt.copy(), size=siz.copy(),
            status=st, origin=None if org is None else org.copy(),
            direction=None if dirc is None else dirc.copy()))
    return tasks


class ConsensusAssigner:
    """Stateful CBAA winner-retention assigner. assign() returns {agent_id:
    task_key} — the same conflict-free 1:1 map on every robot given the same
    inputs. my_task() extracts this robot's Task."""

    def __init__(self) -> None:
        # agent_id -> centroid(3) of its previously-assigned task (the incumbent).
        self._prev: Dict[int, np.ndarray] = {}

    def assign(self, tasks: List[Task], agent_pos: Dict[int, np.ndarray],
               switch_margin: float, match_m: float) -> Dict[int, Tuple]:
        if not tasks or not agent_pos:
            self._prev = {}
            return {}
        aids = sorted(agent_pos)
        by_key = {t.key: t for t in tasks}
        base: Dict[Tuple[int, Tuple], float] = {}
        for a in aids:
            p = np.asarray(agent_pos[a], float)
            inc = self._prev.get(a)
            for t in tasks:
                c = _task_surface_dist_xy(p, t)
                if str(t.status).startswith('ray'):
                    c += RAY_COST_PENALTY   # BBs preferred over rays at like distance
                if inc is not None and float(
                        np.linalg.norm(t.centroid[:2] - inc[:2])) <= match_m:
                    c -= switch_margin   # incumbent retention (CBBA winner stickiness)
                base[(a, t.key)] = c
        used_a: set = set()
        used_t: set = set()
        assigned: Dict[int, Tuple] = {}
        taken_centroids: List[np.ndarray] = []
        while len(used_a) < len(aids) and len(used_t) < len(by_key):
            best = None
            for a in aids:
                if a in used_a:
                    continue
                for t in tasks:
                    if t.key in used_t:
                        continue
                    # Peer repulsion: penalise a task near an already-assigned
                    # one so drones spread to different areas (frontier
                    # _peer_penalty analog). Only reorders which task a drone
                    # takes — it still always takes one when free.
                    c = base[(a, t.key)]
                    for ac in taken_centroids:
                        d = float(np.linalg.norm(t.centroid[:2] - ac[:2]))
                        c += PEER_REPULSION_W * float(
                            np.exp(-d / PEER_REPULSION_SCALE))
                    k = (c, a, t.key)
                    if best is None or k < best:
                        best = k
            if best is None:
                break
            _, a, tk = best
            assigned[a] = tk
            used_a.add(a)
            used_t.add(tk)
            taken_centroids.append(by_key[tk].centroid)
        self._prev = {a: by_key[tk].centroid for a, tk in assigned.items()}
        return assigned

    def my_task(self, assigned: Dict[int, Tuple], my_id: int,
                tasks: List[Task]) -> Optional[Task]:
        tk = assigned.get(my_id)
        if tk is None:
            return None
        return next((t for t in tasks if t.key == tk), None)
