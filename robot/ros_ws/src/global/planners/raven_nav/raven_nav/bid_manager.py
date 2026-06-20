"""Consensus-based auction for ray-mode target assignment.

Per-ray-group bids (one row per RayGroup) + triangulation-aware assignment:

  - compute_my_bids() emits one BidEntry per RayGroup with its descriptor
    (origin, direction, label, num_rays, avg_score) and bid value
    (= -min_dist_to_robot, higher = closer = wins).

  - assign() goes group-by-group on my bids. For each of my groups it checks
    whether ANY peer's same-label group converges on the same target via
    is_same_target(); only same-target peers contest the bid. Different-target
    peer bids never knock me off mine — so two drones looking at two different
    houses can both pursue their own.

Ties broken by lower robot id.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from raven_nav.ray_groups import RayGroup
from raven_nav.ray_targets import is_same_target


# Heading bias: prefer claiming targets whose ray points along the drone's
# heading (cosine momentum + rear-hemisphere reverse, like frontier_behavior).
RAY_MOMENTUM_WEIGHT = 20.0
RAY_REVERSE_SURCHARGE = 40.0

# Centre proximity under which two BBs (or a ray pointing at a BB) are one target.
BB_MATCH_M = 8.0


def _heading_penalty(entry, robot_xy, heading_xy) -> float:
    if heading_xy is None:
        return 0.0
    d = np.asarray(entry.avg_dir, dtype=float)[:2]
    n = float(np.linalg.norm(d))
    if n < 1e-6:
        return 0.0
    cs = float(np.dot(d / n, heading_xy))
    return RAY_MOMENTUM_WEIGHT * (1.0 - cs) + RAY_REVERSE_SURCHARGE * max(-cs, 0.0)


def _bb_heading_penalty(entry, heading_xy, behind_weight) -> float:
    """Gentle BB heading: front and side unpenalized; a reduced penalty only for
    BBs behind the drone, so it favors a ray/target in front but can still turn
    back to a BB if there's nothing better."""
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
    value: float                  # utility; higher = better. -distance, then
                                  # finalize_bid_values folds in BB bonus + heading
    avg_origin: np.ndarray        # (3,) ray origin, or BB centre for is_bb
    avg_dir: np.ndarray           # (3,) unit; bearing to the BB centre for is_bb
    num_rays: int                 # 0 marks a BB bid (rays always have >= 1)
    avg_score: float
    is_bb: bool = False
    size: Optional[np.ndarray] = None   # (3,) full extents, BB only


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
    (raw; BB bonus + heading folded in by finalize_bid_values)."""
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


def finalize_bid_values(entries, robot_xy, heading_xy,
                        ray_reach_factor: float = 1.0,
                        bb_behind_penalty: float = 0.0) -> None:
    """Fold heading into each bid's value (in place). Higher = better.

    BBs: only a gentle behind penalty (front/side free) so it favors a target in
    front but can still turn back to a BB when nothing better is ahead. Rays:
    distance scaled by ray_reach_factor (the target sits further out than the
    ray's origin, so rays don't masquerade as close vs BBs), plus the full
    momentum+reverse heading penalty (anti-zigzag during exploration)."""
    for e in entries:
        if e.is_bb:
            e.value -= _bb_heading_penalty(e, heading_xy, bb_behind_penalty)
            continue
        e.value *= ray_reach_factor
        e.value -= _heading_penalty(e, robot_xy, heading_xy)


def assign(
    my_id: int,
    my_bids: List[BidEntry],
    peer_bids: Dict[str, List[BidEntry]],
    peer_ids: Dict[str, int],
    polygon_xy: Optional[np.ndarray] = None,
    robot_xy: Optional[np.ndarray] = None,
    heading_xy: Optional[np.ndarray] = None,
) -> Optional[Tuple[str, BidEntry]]:
    """Return (label, winning_entry) this robot won, or None.

    A peer bid only contests one of my bids when is_same_target() says the
    two ray groups point at the same physical target. Different-target peer
    bids are ignored for that pair.

    Among my un-contested winners, the best wins: closest physical target minus
    a heading penalty (prefer targets whose origin is ahead). The contest above
    stays pure distance so a far peer can't steal a target I'm next to.
    """
    if not my_bids:
        return None
    won: List[BidEntry] = []
    for mine in my_bids:
        winner_value = mine.value
        winner_id = my_id
        for peer_name, p_entries in peer_bids.items():
            pid = peer_ids.get(peer_name)
            if pid is None:
                continue
            for p in p_entries:
                if p.label != mine.label:
                    continue
                # Synthesize minimal RayGroup-shaped objects for the
                # triangulation test (it only reads label/avg_origin/avg_dir).
                if not _same_bid_instance(mine, p, polygon_xy):
                    continue
                if p.value > winner_value or \
                        (p.value == winner_value and pid < winner_id):
                    winner_value = p.value
                    winner_id = pid
        if winner_id == my_id:   # uncontested, or tied and kept by id
            won.append(mine)
    if not won:
        return None
    # value already folds BB bonus + heading (finalize_bid_values); when called
    # with raw bids (no finalize) this is plain -distance.
    best = max(won, key=lambda e: e.value)
    return best.label, best


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


# CBBA / Hungarian consensus assignment: every robot rebuilds the same
# robots×instances cost matrix from gossiped bids (cost = -value) and solves it
# identically. value is the broadcast utility, so all robots agree.
_INFEASIBLE_COST = 1e6   # robot has no bid for this instance; filtered after solve


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
        best = None  # (key, r, t)
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


def _solve_hungarian(cost: np.ndarray, robot_ids: List[int]) -> Dict[int, int]:
    """Global min-cost matching (scipy). Falls back to CBBA without scipy."""
    try:
        from scipy.optimize import linear_sum_assignment
    except ImportError:
        return _solve_cbba(cost, robot_ids)
    row_ind, col_ind = linear_sum_assignment(cost)
    assigned: Dict[int, int] = {}
    for r, t in zip(row_ind, col_ind):
        if cost[r, t] >= _INFEASIBLE_COST:
            continue
        assigned[robot_ids[r]] = int(t)
    return assigned


def assign_global(
    strategy: str,
    my_id: int,
    my_bids: List[BidEntry],
    peer_bids: Dict[str, List[BidEntry]],
    peer_ids: Dict[str, int],
    polygon_xy: Optional[np.ndarray] = None,
) -> Optional[Tuple[str, BidEntry]]:
    """Consensus assignment ('cbba' | 'hungarian'). Returns (label, my BidEntry)
    for the instance assigned to my_id, or None."""
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
            cost[row_of[rid], t] = -float(b.value)   # -value = distance

    if strategy == 'hungarian':
        assigned = _solve_hungarian(cost, robot_ids)
    else:
        assigned = _solve_cbba(cost, robot_ids)

    my_t = assigned.get(my_id)
    if my_t is None:
        return None
    entry = clusters[my_t].get(my_id)
    if entry is None:
        return None
    return entry.label, entry
