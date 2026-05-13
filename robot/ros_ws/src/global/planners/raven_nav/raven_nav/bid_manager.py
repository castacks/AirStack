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


# Sentinel bid for a target this robot has won and is committed to. Pegged
# high enough that no realistic distance bid from a peer can outbid it.
LOCKED_BID = 1e9


@dataclass
class BidEntry:
    """One bid row = one ray group + its bid value.

    Used both for the local broadcast (my own bids) and for caching peer
    bids in PeerState. The group descriptor is intentionally lightweight —
    avg_origin + avg_dir + label + num_rays + avg_score is enough to run
    is_same_target() and the auction.
    """
    label: str
    value: float                  # -distance; higher = closer = wins
    avg_origin: np.ndarray        # (3,) in the receiver's local frame
    avg_dir: np.ndarray           # (3,) unit
    num_rays: int
    avg_score: float


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


def assign(
    my_id: int,
    my_bids: List[BidEntry],
    peer_bids: Dict[str, List[BidEntry]],
    peer_ids: Dict[str, int],
    polygon_xy: Optional[np.ndarray] = None,
) -> Optional[Tuple[str, BidEntry]]:
    """Return (label, winning_entry) this robot won, or None.

    A peer bid only contests one of my bids when is_same_target() says the
    two ray groups point at the same physical target. Different-target peer
    bids are ignored for that pair.

    Among my un-contested winners across all targets, the highest bid wins
    (closest physical target).
    """
    if not my_bids:
        return None
    won: List[BidEntry] = []
    for mine in my_bids:
        winner_value = mine.value
        winner_id = my_id
        contested = False
        for peer_name, p_entries in peer_bids.items():
            pid = peer_ids.get(peer_name)
            if pid is None:
                continue
            for p in p_entries:
                if p.label != mine.label:
                    continue
                # Synthesize minimal RayGroup-shaped objects for the
                # triangulation test (it only reads label/avg_origin/avg_dir).
                same, _, _ = is_same_target(
                    _GroupView(mine), _GroupView(p), polygon_xy,
                )
                if not same:
                    continue
                if p.value > winner_value or \
                        (p.value == winner_value and pid < winner_id):
                    winner_value = p.value
                    winner_id = pid
                    contested = True
        if winner_id == my_id and not contested:
            won.append(mine)
        elif winner_id == my_id and contested:
            # Tied but with my id — still ours.
            won.append(mine)
    if not won:
        return None
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
